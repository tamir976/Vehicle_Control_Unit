#include "include/control.h"
#include "include/encoder.h"
#include "include/monitor.h"
#include "include/decoder.h"
#include "include/flexcan_conf.h"
#include <math.h>

#define STEER_MAX 1500
#define STEER_DELTA_UP 10
#define STEER_DELTA_DOWN 25
#define MAX_STEER_RATE 100.0f
#define MAX_USER_TORQUE 500.0f
#define DSU_SUPPORTED 1

static int16_t last_torque = 0;
static uint8_t steer_rate_counter = 0;

static bool ipas_initialized = false;
static bool steer_angle_enabled = false;
static uint8_t ipas_reset_counter = 0;
static int16_t last_ipas_angle = 0;
static const float ANGLE_MAX_BP[3] = {0.f, 5.f, 10.f};
static const float ANGLE_MAX_V[3] = {510.f, 300.f, 150.f};

static const float ANGLE_DELTA_BP[3] = {0.f, 5.f, 15.f};
static const float ANGLE_DELTA_V[3]  = {5.f, 3.f, 1.f};
static const float ANGLE_DELTA_VU[3] = {5.f, 3.5f, 1.5f};


SteerCommand gSteerCommand = {0};
AccelCommand gAccelCommand = {0};
PcsCommand gPcsCommand = {0};
PcsCommand_2 gPcsCommand2 = {0};
AccelCancelCommand gAccelCancelCommand = {0};
FcwCommand gFcwCommand = {0};
UICommand gUICommand = {0};
SteeringIpasCommand gSteeringIpasCommand = {0};
SteeringIpasCommaCommand gSteeringIpasCommaCommand = {0};
IpasControllerState gIpasControllerState = {0};

static void ipas_reset_state(void){
    gIpasControllerState.initialized= true;
    gIpasControllerState.steer_angle_enabled = false;
    gIpasControllerState.ipas_reset_counter = 0u;
    gIpasControllerState.last_angle = 0;
    gIpasControllerState.frame = 0u;
}
static void ipas_default(void){
    gSteeringIpasCommand.ANGLE = 0;
    gSteeringIpasCommand.DIRECTION_CMD = 2u;
    gSteeringIpasCommand.SET_ME_X00 = 0u;
    gSteeringIpasCommand.SET_ME_X00_1 = 0u;
    gSteeringIpasCommand.SET_ME_X10 = 0x10u;
    gSteeringIpasCommand.SET_ME_X40 = 0x40;

}

static float clipf(float v, float lo, float hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static float absf(float v){
    return v < 0 ? -v : v;
}

static float interpf(float x, const float *bp, const float *v, uint8_t len){
    if(x <= bp[0]) return v[0];
    if(x >= bp[len-1]) return v[len-1];

    for(uint8_t i=0;i<len-1;i++){
        if(x <= bp[i+1]){
            float t = (x - bp[i]) / (bp[i+1] - bp[i]);
            return v[i] + t*(v[i+1] - v[i]);
        }
    }
    return v[len-1];
}

static uint8_t ipas_direction_cmd(int16_t angle){
    if(angle < 0){
        return 3u;
    }
    if(angle >0){
        return 1u;
    }
    return 2u;
}

static ipas_command_from_angle(int16_t angle, bool enabled){
    ipas_default();
    gSteeringIpasCommand.STATE = enabled ? 3u : 1u;
    gSteeringIpasCommand.ANGLE = angle;
    gSteeringIpasCommand.DIRECTION_CMD = ipas_direction_cmd(angle);    
}

static void ipas_update_enable_state(bool control_enabled, bool ipas_active){
    if(control_enabled && !gIpasControllerState.steer_angle_enabled){
        gIpasControllerState.steer_angle_enabled = true;
        gIpasControllerState.ipas_reset_counter = 0u;
    }else if(control_enabled && gIpasControllerState.steer_angle_enabled){
        if(ipas_active){
            gIpasControllerState.ipas_reset_counter = 0u;
        }else if(gIpasControllerState.ipas_reset_counter < 255){
            gIpasControllerState.ipas_reset_counter++;
        }
        if(gIpasControllerState.ipas_reset_counter > 10u){
            gIpasControllerState.steer_angle_enabled = false;
        }
    }else{
        gIpasControllerState.steer_angle_enabled = false;
        gIpasControllerState.ipas_reset_counter = 0u;
    }
}

void create_ipas_command(CarState *cs, ){

}

void ProcessCommands(CarControl *cc, CarState *cs){
    taskENTER_CRITICAL();
    // bool lat_active = cc->latActive;
    bool lat_active = true; //for testing ipas
    bool ipas_activeCs = cs->ipasActive;
    bool ipas_activeCc = false; //for testing ipas
    cc->emergency = false;
     if(!ipas_initialized){
        steer_angle_enabled = false;
        ipas_reset_counter = 0;
        last_ipas_angle = 0;
        ipas_initialized = true;
    }
    bool lat_active_ipas = cc->latActive && (fabsf(cs->steeringTorque) < MAX_USER_TORQUE);
    if(lat_active_ipas){
        if(!steer_angle_enabled){
            steer_angle_enabled = true;
            ipas_reset_counter = 0;
        }else{
            if(cs->ipasActive){
                ipas_reset_counter = 0;
            }else{
                ipas_reset_counter++;
            }
            if(ipas_reset_counter > 10){
                steer_angle_enabled = false;
            }
        }
    }else{
        steer_angle_enabled = false;
        ipas_reset_counter = 0;
    }

    int16_t apply_angle;
    cc->actuators.steeringAngleDegCmd = 0.f;
    if(!steer_angle_enabled || !cs->ipasActive){
        apply_angle = (int16_t)(cs->steeringAngleDeg);
    }else{
        float requested = cc->actuators.steeringAngleDegCmd;

        float angle_limit = interpf(cs->vEgo, ANGLE_MAX_BP, ANGLE_MAX_V, 3);
        requested = clipf(requested, -angle_limit, angle_limit);

        bool winding_up = ((last_ipas_angle * requested) > 0.f) &&
                        (absf(requested) > absf((float)last_ipas_angle));

        float rate_limit = interpf(cs->vEgo,
                                ANGLE_DELTA_BP,
                                winding_up ? ANGLE_DELTA_V : ANGLE_DELTA_VU,
                                3);

        requested = clipf(requested,
                        last_ipas_angle - rate_limit,
                        last_ipas_angle + rate_limit);

        apply_angle = (int16_t)requested;
    }

    last_ipas_angle = apply_angle;

    if(!lat_active_ipas){
        gSteeringIpasCommand.STATE = 1u;
        gSteeringIpasCommand.ANGLE = 0;
        gSteeringIpasCommand.DIRECTION_CMD = 2u;
    }else{
        gSteeringIpasCommand.STATE = steer_angle_enabled ? 3u : 1u;
        gSteeringIpasCommand.ANGLE = apply_angle;

        if(apply_angle < 0){
            gSteeringIpasCommand.DIRECTION_CMD = 3u;
        }else if(apply_angle > 0){
            gSteeringIpasCommand.DIRECTION_CMD = 1u;
        }else{
            gSteeringIpasCommand.DIRECTION_CMD = 2u;
        }
    }

    gSteeringIpasCommand.SET_ME_X10 = 0x10;
    gSteeringIpasCommand.SET_ME_X40 = 0x40;

    if(!ipas_activeCs && ipas_activeCc){
        int16_t desired_torque = (int16_t)(cc->actuators.torqueCmd);
        
        bool driver_override = fabsf(cs->steeringTorque) > MAX_USER_TORQUE;
        int16_t delta = desired_torque - last_torque;
        if(delta > STEER_DELTA_UP) delta = STEER_DELTA_UP;
        if(delta < -STEER_DELTA_DOWN) delta = -STEER_DELTA_DOWN;
        
        int16_t apply_torque = last_torque + delta;
        if(apply_torque > STEER_MAX) apply_torque = STEER_MAX;
        if(apply_torque < -STEER_MAX) apply_torque = -STEER_MAX;
        
        bool high_rate = fabsf(cs->steeringRateDeg) >= MAX_STEER_RATE;
        if(high_rate && lat_active){
            steer_rate_counter++;
        }else{
            steer_rate_counter = 0;
        }
        bool steer_allowed = (steer_rate_counter < 18);
        
        if(!lat_active || driver_override || !steer_allowed){
            apply_torque = 0;
            gSteerCommand.STEER_REQUEST = false;
        }else{
            gSteerCommand.STEER_REQUEST = true;
        }
        gSteerCommand.STEER_TORQUE_CMD = apply_torque;
        gSteerCommand.SET_ME_1 = true;
        last_torque = apply_torque;
    }else{
        gSteerCommand.STEER_TORQUE_CMD = 0;
        gSteerCommand.STEER_REQUEST = false;
        gSteerCommand.SET_ME_1 = true;
    }
    
    gAccelCommand.ACCEL_CMD = cc->actuators.accelCmd;
    gAccelCommand.ACCEL_CMD_ALT = cc->actuators.accelCmd;
    gAccelCommand.ACC_TYPE = 1u;

    static uint8_t distance_button = 0;
    if(cs->buttonPressed){
        distance_button ^=1u;
    }
    gAccelCommand.DISTANCE = distance_button;
    
    gAccelCommand.MINI_CAR = (cs->vEgo < 12.0f) ? true : false;

    static bool permit_braking = true;
    float accel = cc->actuators.accelCmd;
    if(accel < 0.2f){
        permit_braking = true;
    }else if(accel > 0.3f){
        permit_braking = false;
    }
    gAccelCommand.PERMIT_BRAKING = permit_braking;
    
    static bool last_standstill = false;
    static bool standstill_req = false;
    if(cs->standStill && !last_standstill){
        standstill_req = true;
    }
    if(!cs->cruiseState.enabled){
        standstill_req = false;
    }
    last_standstill = cs->standStill;
    
    gAccelCommand.RELEASE_STANSTILL = !standstill_req;
    gAccelCommand.CANCEL_REQ = cc->emergency;
    gAccelCommand.ALLOW_LONG_PRESS = 1u;
    gAccelCommand.ACC_CUT_IN = 0u;
    gAccelCommand.ACC_MALFUNCTION = cs->accFaulted;
    gAccelCommand.RADAR_DIRTY = false;
    gAccelCommand.LEAD_VEHICLE_STOPPED = cs->standStill;

    if(DSU_SUPPORTED == 0){
        gAccelCancelCommand.GAS_RELEASED = false;
        gAccelCancelCommand.CRUISE_ACTIVE = cs->cruiseState.enabled;
        gAccelCancelCommand.ACC_BRAKING = (cc->brakeCmd > 0.01f);
        gAccelCancelCommand.ACCEL_NET = cc->actuators.accelCmd;
        gAccelCancelCommand.CRUISE_STATE = cs->cruiseState.enabled ? true : false;
        gAccelCancelCommand.CANCEL_REQ = cc->cancelReq;
    }


    // bool pcs_active = cc->emergency;
    
    // gPcsCommand.COUNTER = 0u;
    // gPcsCommand.FORCE = (int16_t)fminf(cc->actuators.accelCmd, 0.0f) * 2000.0f;
    // gPcsCommand.STATE = pcs_active ? 3u : 0u;
    // gPcsCommand.BRAKE_STATUS = 0u;
    // gPcsCommand.PRECOLLISION_ACTIVE = pcs_active;

    // gPcsCommand2.DSS1GDRV = fminf(cc->actuators.accelCmd, 0.0f);
    // gPcsCommand2.PCSALM = pcs_active;
    // gPcsCommand2.IBTRGR = pcs_active;
    // gPcsCommand2.PBATRGR = pcs_active;
    // gPcsCommand2.PREFILL = pcs_active;
    // gPcsCommand2.AVSTRGR = pcs_active;
    
    gFcwCommand.FCW = cc->emergency; 
    gFcwCommand.PCS_INDICATOR = 0u;
    gFcwCommand.SET_ME_X20 = 0x20;
    gFcwCommand.SET_ME_X10 = 0x10;
    gFcwCommand.PCS_OFF = true;
    gFcwCommand.PCS_SENSITIVITY = 0u;
    gFcwCommand.PCS_DUST = false;
    gFcwCommand.PCS_TEMP = false;
    gFcwCommand.PCS_DUST2 = false;
    gFcwCommand.PCS_TEMP2 = false;
    gFcwCommand.FRD_ADJ = 0u;
    
    gUICommand.TWO_BEEPS = cc->emergency; //take from the !accelEnable
    gUICommand.LDA_ALERT = (lat_active && !cc->steeringEnable);
    gUICommand.LEFT_LINE = cs->leftBlinker ? 3u : 1u;
    gUICommand.RIGHT_LINE = cs->rightBlinker ? 3u : 1u;
    gUICommand.BARRIERS = cc->vcuEnabled;
    gUICommand.SET_ME_X01 = true;
    gUICommand.SET_ME_X02 = 2u;
    gUICommand.LKAS_STATUS = 1u;
    gUICommand.LDW_EXIST = true;
    gUICommand.ADJUSTING_CAMERA = false;
    gUICommand.LANE_SWAY_BUZZER = false;
    gUICommand.LANE_SWAY_FLD = 7u;
    gUICommand.LANE_SWAY_SENSITIVITY = 2u;
    gUICommand.LANE_SWAY_TOGGLE = true;
    gUICommand.LANE_SWAY_WARNING = 0u;
    gUICommand.LDA_FRONT_CAMERA_BLOCKED = 0u;
    gUICommand.LDA_MALFUNCTION = false;
    gUICommand.LDA_MESSAGES = 0u;
    gUICommand.LDA_ON_MESSAGE = 0u;
    gUICommand.LDA_SA_TOGGLE = 1u;
    gUICommand.LDA_SENSITIVITY = 2u;
    gUICommand.LDA_UNAVAILABLE = false;
    gUICommand.LDA_UNAVAILABLE_QUIET = false;
    gUICommand.LDW_EXIST = true;
    gUICommand.REPEATED_BEAPS = false;
    gUICommand.TAKE_CONTROL = 0u;
    taskEXIT_CRITICAL();

}


void ControlTask(void *pv){
    (void)pv;
    CarState CsSnap;
    CarControl CcSnap;
    TickType_t lastWakeTime = xTaskGetTickCount();
    for(;;){
        taskENTER_CRITICAL();
        CsSnap = gCarState[CsActiveId];
        CcSnap = gCarControl[CcActiveId];
        taskEXIT_CRITICAL();
        ProcessCommands(&CcSnap, &CsSnap);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10u));
    }
}
