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
#define ANGLE_MAX_LEN 3
#define ANGLE_DELTA_LEN 3

#define TOYOTA_ACC_ACCEL_MAX 3.5f
#define TOYOTA_ACC_ACCEL_MIN -3.5f
#define TOYOTA_ACC_PITCH_RC 0.5f
#define TOYOTA_ACC_AEGO_RC 0.25f
#define ACCELERATION_DUE_TO_GRAVITY 9.81f
#define TOYOTA_ACC_PID_KF 1.0f
#define TOYOTA_ACC_PID_UNWIND (0.03f * DT_CTRL * 3.0f)
#define TOYOTA_ACC_WINDUP_LIMIT 0.12f
#define TOYOTA_ACC_WINDDOWN_LIMIT -0.12f

static const float ANGLE_MAX_BP[3] = {0.f, 5.f, 10.f};
static const float ANGLE_MAX_V[3] = {510.f, 400.f, 300.f};

static const float ANGLE_DELTA_BP[3] = {0.f, 5.f, 15.f};
static const float ANGLE_DELTA_V[3]  = {5.f, 3.f, 1.f};
static const float ANGLE_DELTA_VU[3] = {5.f, 4.f, 3.0f};
static const float TOYOTA_LONG_PID_KP_BP[1] = {0.0f};
static const float TOYOTA_LONG_PID_KP_V[1] = {0.0f};
static const float TOYOTA_LONG_PID_KI_BP[3] = {0.0F, 5.0F, 35.0F};
static const float TOYOTA_LONG_PID_KI_V[3] = {3.6F, 2.4F, 1.5F};
static const float TOYOTA_LONG_PID_KD_BP[1] = {0.0F};
static const float TOYOTA_LONG_PID_KD_V[1] = {0.0F};

SteerCommand gSteerCommand = {0};
AccelCommand gAccelCommand = {0};
PcsCommand gPcsCommand = {0};
PcsCommand_2 gPcsCommand2 = {0};
AccelCancelCommand gAccelCancelCommand = {0};
FcwCommand gFcwCommand = {0};
UICommand gUICommand = {0};
SteeringIpasCommand gSteeringIpasCommand = {0};
IpasControllerState gIpasControllerState = {0};
static ToyotaAccControllerState acc_state = {0};
static uint8_t acc_control_divider = 0u;
static bool acc_command_ready = false;

static void ipas_reset_state(void){
    gIpasControllerState.initialized= true;
    gIpasControllerState.steer_angle_enabled = false;
    gIpasControllerState.ipas_reset_counter = 0u;
    gIpasControllerState.last_angle = 0.f;
}
static void ipas_default(SteeringIpasCommand *cmd){
    cmd->STATE = 1u;
    cmd->ANGLE = 0.f;
    cmd->DIRECTION_CMD = 2u;
    cmd->SET_ME_X00 = 0u;
    cmd->SET_ME_X00_1 = 0u;
    cmd->SET_ME_X10 = 0x10u;
    cmd->SET_ME_X40 = 0x40;
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
    if(len == 0) return 0.0f;
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

static float math_signf(float value){
    if(value > 0.f) return 1.f;
    if(value < 0.f) return -1.f;
    return 0.f;
}

static float math_rate_limitf(float value, float last_value, float down_step, float up_step){
    return clipf(value, last_value + down_step, last_value + up_step);
}

static void pid_reset(PidController *pid){
    pid->speed = 0.f;
    pid->p = 0.f;
    pid->i = 0.f;
    pid->d = 0.f;
    pid->f = 0.f;
    pid->control = 0.f;
}

static void pid_init(PidController *pid,
                     const float *k_p_bp, const float *k_p_v, uint8_t k_p_len,
                     const float *k_i_bp, const float *k_i_v, uint8_t k_i_len,
                     const float *k_d_bp, const float *k_d_v, uint8_t k_d_len,
                     float k_f, float pos_limit, float neg_limit, float rate) {
  pid->k_p_bp = k_p_bp;
  pid->k_p_v = k_p_v;
  pid->k_p_len = k_p_len;
  pid->k_i_bp = k_i_bp;
  pid->k_i_v = k_i_v;
  pid->k_i_len = k_i_len;
  pid->k_d_bp = k_d_bp;
  pid->k_d_v = k_d_v;
  pid->k_d_len = k_d_len;
  pid->k_f = k_f;
  pid->pos_limit = pos_limit;
  pid->neg_limit = neg_limit;
  pid->i_unwind_rate = 0.3F / rate;
  pid->i_rate = 1.0F / rate;

  pid_reset(pid);
}

static float pid_update(PidController *pid, float error,
                        float error_rate, float speed, bool override, float feedforward, bool freeze_integrator
){
    pid->speed = speed;
    const float k_p = interpf(speed, pid->k_p_bp, pid->k_p_v, pid->k_p_len);
    const float k_i = interpf(speed, pid->k_i_bp, pid->k_i_v, pid->k_i_len);
    const float k_d = interpf(speed, pid->k_d_bp, pid->k_d_v, pid->k_d_len);
    pid->p = error * k_p;
    pid->f = feedforward * pid->k_f;
    pid->d = error_rate * k_d;
    if(override){
        pid->i -= pid->i_unwind_rate * math_signf(pid->i);
    }else if(!freeze_integrator){
        pid->i += error * k_i * pid->i_rate;
        float control_no_i = pid->p + pid->d + pid->f;
        control_no_i = clipf(control_no_i, pid->neg_limit, pid->pos_limit);
        pid->i = clipf(pid->i, pid->neg_limit - control_no_i, pid->pos_limit - control_no_i);
    }
    const float control = pid->p + pid->i + pid->d + pid->f;
    pid->control = clipf(control, pid->neg_limit, pid->pos_limit);
    return pid->control;
}

static void math_first_order_filter_update_alpha(MathFirstOrderFilter *filter, float rc){
    filter->alpha = filter->dt / (rc + filter->dt);
}

static void math_first_order_filter_init(MathFirstOrderFilter *filter, float x0, float dt, float rc, bool initialized){
    filter->x = x0;
    filter->dt = dt;
    filter->initialized = initialized;
    math_first_order_filter_update_alpha(filter, rc);
}

static float math_first_order_filter_update(MathFirstOrderFilter *filter, float x){
    if(filter->initialized){
        filter->x = ((1.0f - filter->alpha) * filter->x) + (filter->alpha * x);
    }else{
        filter->initialized = true;
        filter->x = x;
    }
    return filter->x;
}

static void toyota_acc_reset_state(void){
    acc_state.initialized = true;
    acc_state.last_standstill = false;
    acc_state.standstill_req = false;
    acc_state.permit_braking = true;
    acc_state.distance_button = 0u;
    acc_state.prev_accel = 0.f;
    acc_state.accel = 0.f;
    math_first_order_filter_init(&acc_state.pitch_filter, 0.f, DT_CTRL, TOYOTA_ACC_PITCH_RC, true);
    math_first_order_filter_init(&acc_state.aego_filter, 0.f, DT_CTRL * 3.0f, TOYOTA_ACC_AEGO_RC, true);
    pid_init(&acc_state.long_pid,
             TOYOTA_LONG_PID_KP_BP, TOYOTA_LONG_PID_KP_V, 1u,
             TOYOTA_LONG_PID_KI_BP, TOYOTA_LONG_PID_KI_V, 3u,
             TOYOTA_LONG_PID_KD_BP, TOYOTA_LONG_PID_KD_V, 1u,
             TOYOTA_ACC_PID_KF, TOYOTA_ACC_ACCEL_MAX, TOYOTA_ACC_ACCEL_MIN, 1.0f / (DT_CTRL * 3.0f));
    acc_control_divider = 0u;
    acc_command_ready = false;
}

void toyota_acc_default_command(AccelCommand *cmd){
    cmd->ACCEL_CMD = 0.f;
    cmd->ALLOW_LONG_PRESS = 0u;
    cmd->ACC_MALFUNCTION = false;
    cmd->RADAR_DIRTY = false;
    cmd->DISTANCE = false;
    cmd->MINI_CAR = false;
    cmd->ACC_TYPE = 0u;
    cmd->CANCEL_REQ = false;
    cmd->ACC_CUT_IN = 0u;
    cmd->LEAD_VEHICLE_STOPPED = false;
    cmd->PERMIT_BRAKING = false;
    cmd->RELEASE_STANSTILL = false;
    cmd->ITS_CONNECT_LEAD = 0u;
    cmd->ACCEL_CMD_ALT = 0.f;
}

static void toyota_acc_update_standstill_state(const CarState *cs){
    if(cs->standStill && !acc_state.last_standstill){
        acc_state.standstill_req = false;
    }else if(cs->pcmAccStatus != 8u){
        acc_state.standstill_req = false;
    }
    acc_state.last_standstill = cs->standStill;
}

static void toyota_acc_update_distance_button(const HudControl *hud_control, const CarState *cs){
    const int desired_distance = 4 - (int)hud_control->leadDistanceBars;
    if(cs->cruiseState.enabled && ((int)cs->pcmFollowDistance != desired_distance)){
        acc_state.distance_button = acc_state.distance_button == 0u ? 1u : 0u;
    }else{
        acc_state.distance_button = 0u;
    }
}

static float toyota_acc_compute_accel_due_to_pitch(float pitch_input){
    math_first_order_filter_update(&acc_state.pitch_filter, pitch_input);
    const float filtered_pitch = acc_state.pitch_filter.x;
    const float downhill_pitch = filtered_pitch < 0.0f ? filtered_pitch : 0.f;
    return sinf(downhill_pitch) * ACCELERATION_DUE_TO_GRAVITY;
}

static float toyota_acc_compute_aego_future(const CarState *cs){
    static const float future_t_bp[2] = {2.0f, 5.0f};
    static const float future_t_v[2] = {0.25f, 0.5f};
    const float a_ego_blended = cs->aEgo;
    const float prev_aego = acc_state.aego_filter.x;
    math_first_order_filter_update(&acc_state.aego_filter, a_ego_blended);
    const float j_ego = (acc_state.aego_filter.x - prev_aego) / (DT_CTRL * 3.0f);
    const float future_t = interpf(cs->vEgo, future_t_bp, future_t_v, 2u);
    return a_ego_blended + (j_ego * future_t);
}

static float toyota_acc_compute_pcm_accel_cmd(const CarControl *cc, const CarState *cs, bool long_active, bool stopping_active){
    float pcm_accel_cmd = cc->actuators.accelCmd;
    if(long_active){
        pcm_accel_cmd = math_rate_limitf(pcm_accel_cmd, acc_state.prev_accel, TOYOTA_ACC_WINDDOWN_LIMIT, TOYOTA_ACC_WINDUP_LIMIT);
    }
    acc_state.prev_accel = pcm_accel_cmd;
    const float pitch_input = 0.0f;
    const float accel_due_to_pitch = toyota_acc_compute_accel_due_to_pitch(pitch_input);
    const float net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch;
    const float a_ego_future = toyota_acc_compute_aego_future(cs);
    if(long_active){
        acc_state.long_pid.i -= TOYOTA_ACC_PID_UNWIND * math_signf(acc_state.long_pid.i);
        const float error_future = pcm_accel_cmd - a_ego_future;
        const bool freeze_integrator = cc->actuators.longcontrolstate != pid;
        pcm_accel_cmd = pid_update(&acc_state.long_pid, error_future, 0.f, cs->vEgo, false, pcm_accel_cmd, freeze_integrator);
    }else{
        pid_reset(&acc_state.long_pid);
    }
    const float accel_cmd_with_pitch = cc->actuators.accelCmd + accel_due_to_pitch;
    const float net_acceleration_request_min = (accel_cmd_with_pitch < net_acceleration_request) ? accel_cmd_with_pitch : net_acceleration_request;
    if((net_acceleration_request_min < 0.2f) || stopping_active || !long_active){
        acc_state.permit_braking = true;
    }else if(net_acceleration_request_min > 0.3f){
        acc_state.permit_braking = false;
    }
    return clipf(pcm_accel_cmd, TOYOTA_ACC_ACCEL_MIN, TOYOTA_ACC_ACCEL_MAX);
}

static void toyota_acc_make_command(AccelCommand *cmd, const CarState *cs, float accel, bool lead, bool pcm_cancel_cmd, bool fcw_alert){
    toyota_acc_default_command(cmd);
    cmd->ACCEL_CMD = accel;
    cmd->ACC_TYPE = 1u;
    cmd->DISTANCE = acc_state.distance_button;
    cmd->MINI_CAR = lead;
    cmd->PERMIT_BRAKING = acc_state.permit_braking;
    cmd->RELEASE_STANSTILL = !acc_state.standstill_req;
    cmd->CANCEL_REQ = pcm_cancel_cmd;
    cmd->ALLOW_LONG_PRESS = 1u;
    cmd->ACC_CUT_IN = fcw_alert;
    cmd->ACC_MALFUNCTION = false;
    cmd->RADAR_DIRTY = false;
    cmd->LEAD_VEHICLE_STOPPED = false;
    cmd->ACCEL_CMD_ALT = 0.0f;
}

static void process_accel_cmd(AccelCommand *cmd, const CarControl *cc, const CarState *cs){
    if((cc == NULL) || (cs == NULL)){
        toyota_acc_default_command(cmd);
        return;
    }
    if(!acc_state.initialized){
        toyota_acc_reset_state();
    }
    const bool control_enabled = cc->vcuEnabled == true;
    const HudControl hud_control = {
        .leadVisible = false,
        .leadDistanceBars = 2u,
        .visualAlert = VisualAlertNone,
    };
    const bool fcw_alert = hud_control.visualAlert == VisualAlertFcw;
    const bool lead = hud_control.leadVisible || cs->vEgo < 40.0f;
    const bool long_active = control_enabled && (cc->longActive == true) && (cc->accEnable == true);
    const bool stopping_active = cc->actuators.longcontrolstate == stopping;
    const bool pcm_cancel_cmd = cc->emergency == true;
    if (!pcm_cancel_cmd){

        toyota_acc_update_distance_button(&hud_control, cs);

        toyota_acc_update_standstill_state(cs);

        const float pcm_accel_cmd = toyota_acc_compute_pcm_accel_cmd(cc, cs, long_active, stopping_active);

        acc_state.accel = pcm_accel_cmd;
        
        toyota_acc_make_command(cmd, cs, pcm_accel_cmd, lead, pcm_cancel_cmd, fcw_alert);
        return;
    }
    else {
        
        toyota_acc_default_command(cmd);
        cmd->ACCEL_CMD = 0.0F;
        cmd->CANCEL_REQ = true;
        cmd->PERMIT_BRAKING = true;
        cmd->RELEASE_STANSTILL = true;
        cmd->MINI_CAR = lead;
        cmd->ACC_TYPE = 1u;
        cmd->ACC_CUT_IN = false;
        cmd->DISTANCE = acc_state.distance_button;
        return;
    }
}
bool toyota_acc_command_is_ready(void){
    return acc_command_ready;
}

static uint8_t ipas_direction_cmd(float angle){
    if(angle < 0.f){
        return 3u;
    }
    if(angle >0.f){
        return 1u;
    }
    return 2u;
}


static void ipas_command_from_angle(SteeringIpasCommand *cmd, float angle, bool enabled){
    ipas_default(cmd);
    cmd->STATE = enabled ? 3u : 1u;
    cmd->ANGLE = angle;
    cmd->DIRECTION_CMD = ipas_direction_cmd(angle);    
}

static void ipas_update_enable_state(bool control_enabled, bool ipas_active){
    if(control_enabled && !gIpasControllerState.steer_angle_enabled){
        gIpasControllerState.steer_angle_enabled = true;
        gIpasControllerState.ipas_reset_counter = 0u;
    }else if(control_enabled && gIpasControllerState.steer_angle_enabled){
        if(ipas_active){
            gIpasControllerState.ipas_reset_counter = 0u;
        }else if(gIpasControllerState.ipas_reset_counter < 255u){
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

static float ipas_limited_angle(const CarControl *cc, const CarState *cs){
	if(!gIpasControllerState.steer_angle_enabled || (cs->ipasState != enabled)){
		return (cs->steeringAngleDeg);
	}
    float requested_angle = cc->actuators.steeringAngleDegCmd;
    const float angle_limit = interpf(cs->vEgo, ANGLE_MAX_BP, ANGLE_MAX_V, ANGLE_MAX_LEN);
    requested_angle = clipf(requested_angle, -angle_limit, angle_limit);

    const bool winding_up = ((gIpasControllerState.last_angle * requested_angle) > 0.0f) && (absf(requested_angle) > absf(gIpasControllerState.last_angle));
    const float angle_rate_limit = interpf(cs->vEgo, ANGLE_DELTA_BP, winding_up ? ANGLE_DELTA_V : ANGLE_DELTA_VU, ANGLE_DELTA_LEN);
    requested_angle = clipf(requested_angle, gIpasControllerState.last_angle - angle_rate_limit, gIpasControllerState.last_angle + angle_rate_limit);
    return requested_angle;
}



void create_ipas_command(CarState *cs, CarControl *cc, SteeringIpasCommand *cmd){
   if((cc == NULL) || (cs == NULL)){
       ipas_default(cmd);
       return;
   }
    if(!gIpasControllerState.initialized){
        ipas_reset_state();
    }
    const bool lat_active = (cc->latActive != false) && (cs->cruiseState.enabled) &&
                            (absf(cs->steeringTorque) < MAX_USER_TORQUE);
    if(lat_active){
    	ipas_update_enable_state((cc->vcuEnabled != false), (cs->ipasState == enabled));
    }
    const float apply_angle = ipas_limited_angle(cc, cs);
    gIpasControllerState.last_angle = apply_angle;
    if(!lat_active){
        ipas_command_from_angle(cmd, 0.f, false);
        return;
    }
    ipas_command_from_angle(cmd, apply_angle, gIpasControllerState.steer_angle_enabled);
}

static void update_fcw_command(FcwCommand *cmd, CarState *cs, CarControl *cc){
    (void)cs;
    (void)cc;

    cmd->FCW = true; 
    cmd->PCS_INDICATOR = 0u;
    cmd->SET_ME_X20 = 0x20;
    cmd->SET_ME_X10 = 0x10;
    cmd->PCS_OFF = true;
    cmd->PCS_SENSITIVITY = 0u;
    cmd->PCS_DUST = false;
    cmd->PCS_TEMP = false;
    cmd->PCS_DUST2 = false;
    cmd->PCS_TEMP2 = false;
    cmd->FRD_ADJ = 0u;
}

static void update_ui_command(UICommand *cmd, CarState *cs, CarControl *cc){
    (void)cc;

    cmd->TWO_BEEPS = false; //take from the !accelEnable
    cmd->LDA_ALERT = 0u;
    cmd->LEFT_LINE = cs->leftBlinker ? 3u : 1u;
    cmd->RIGHT_LINE = cs->rightBlinker ? 3u : 1u;
    cmd->BARRIERS = 0u;
    cmd->SET_ME_X01 = true;
    cmd->SET_ME_X02 = 2u;
    cmd->LKAS_STATUS = 1u;
    cmd->LDW_EXIST = true;
    cmd->ADJUSTING_CAMERA = false;
    cmd->LANE_SWAY_BUZZER = false;
    cmd->LANE_SWAY_FLD = 7u;
    cmd->LANE_SWAY_SENSITIVITY = 2u;
    cmd->LANE_SWAY_TOGGLE = true;
    cmd->LANE_SWAY_WARNING = 0u;
    cmd->LDA_FRONT_CAMERA_BLOCKED = 0u;
    cmd->LDA_MALFUNCTION = false;
    cmd->LDA_MESSAGES = 0u;
    cmd->LDA_ON_MESSAGE = 0u;
    cmd->LDA_SA_TOGGLE = 1u;
    cmd->LDA_SENSITIVITY = 2u;
    cmd->LDA_UNAVAILABLE = false;
    cmd->LDA_UNAVAILABLE_QUIET = false;
    cmd->LDW_EXIST = true;
    cmd->REPEATED_BEAPS = false;
    cmd->TAKE_CONTROL = 0u;
}

void ProcessCommands(CarControl *cc, CarState *cs){
    SteeringIpasCommand ipasCommand;
    AccelCommand accelCommand;
    FcwCommand fcwCommand;
    UICommand uiCommand;

    taskENTER_CRITICAL();
    ipasCommand = gSteeringIpasCommand;
    accelCommand = gAccelCommand;
    fcwCommand = gFcwCommand;
    uiCommand = gUICommand;
    taskEXIT_CRITICAL();

    create_ipas_command(cs, cc, &ipasCommand);

    acc_control_divider = (uint8_t)((acc_control_divider + 1u) % 3u);
    if(acc_control_divider == 0u){
        process_accel_cmd(&accelCommand, cc, cs);
        acc_command_ready = true;
    }

    update_fcw_command(&fcwCommand, cs, cc);
    update_ui_command(&uiCommand, cs, cc);

    taskENTER_CRITICAL();
    gSteeringIpasCommand = ipasCommand;
    gAccelCommand = accelCommand;
    gFcwCommand = fcwCommand;
    gUICommand = uiCommand;
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
