#include "include/decoder.h"
#include "include/car_control.h"
#include "include/can_frame.h"
#include <string.h>
#include <math.h>

CarState gCarState[2] = {0};
CarControl gCarControl[2] = {0};
volatile uint8_t CsActiveId = 0;
volatile uint8_t CcActiveId = 0;
KF1D speed_kf;

typedef enum {
    SIGNAL_UNSIGNED,
    SIGNAL_SIGNED,
    SIGNAL_FLOAT,
    SIGNAL_BOOL
} dataType;

typedef union 
{
    uint64_t u;
    int64_t i;
    float f;
    bool b;
} SignalValue;


static void KF1D_Init(KF1D *kf,
                      float x0_init, float x1_init,
                      float A00, float A01, float A10, float A11,
                      float C0,  float C1,
                      float K0,  float K1)
{
    kf->x0 = x0_init;
    kf->x1 = x1_init;
    kf->K0 = K0;
    kf->K1 = K1;

    // precompute A_K = A - K*C
    kf->AK0 = A00 - K0 * C0;
    kf->AK1 = A01 - K0 * C1;
    kf->AK2 = A10 - K1 * C0;
    kf->AK3 = A11 - K1 * C1;
}

static void KF1D_Update(KF1D *kf, float meas)
{
    float new_x0 = kf->AK0 * kf->x0 + kf->AK1 * kf->x1 + kf->K0 * meas;
    float new_x1 = kf->AK2 * kf->x0 + kf->AK3 * kf->x1 + kf->K1 * meas;
    kf->x0 = new_x0;
    kf->x1 = new_x1;
}

static void KF1D_SetX(KF1D *kf, float x0, float x1)
{
    kf->x0 = x0;
    kf->x1 = x1;
}

static void UpdateSpeedKF(KF1D *kf, float v_ego_raw, float *v_ego, float *v_ego_dot)
{
    // reset if speed jumps by more than 2.0 (car starts at non-zero speed)
    if(fabsf(v_ego_raw - kf->x0) > 2.0f)
    {
        KF1D_SetX(kf, v_ego_raw, 0.0f);
    }

    KF1D_Update(kf, v_ego_raw);

    *v_ego = kf->x0;   // filtered velocity
    *v_ego_dot = kf->x1;   // filtered acceleration
}

static bool FindFrameIdCs(uint32_t id, CanFrameEntry *out){
    if (CanCache_CopyFrame(&gCan0Cache, id, out) == TRUE) { return TRUE; }
    if (CanCache_CopyFrame(&gCan1Cache, id, out) == TRUE) { return TRUE; }    
    if (CanCache_CopyFrame(&gCan2Cache, id, out) == TRUE) { return TRUE; }
    return FALSE;
}
static bool FindFrameIdCc(uint32_t id, CanFrameEntry *out){
    if (CanCache_CopyFrame(&gCan3Cache, id, out) == true) { return true;}
    return false;
}

static uint64_t ExtractLE(const uint8_t data[8], const uint8_t dlc, uint8_t startBit, uint8_t length)
{
    if((dlc > 8u) || (length == 0u) || (length > 64u) || (startBit >= (dlc * 8u)) || ((startBit + length) > (dlc * 8u))){
        return 0u;
    }

    uint64_t raw = 0;
    for (uint8_t i = 0; i < dlc; i++)
    {
        raw |= ((uint64_t)data[i] << (i * 8u));
    }
    return (raw >> startBit) & ((length == 64u) ? UINT64_MAX : ((1ULL << length) - 1ULL));
}

static uint64_t ExtractBE(const uint8_t data[8], const uint8_t dlc, uint8_t startBit, uint8_t length)
{
    if((dlc > 8u) || (length == 0u) || (length > 64u) || (startBit >= (dlc * 8u))){
        return 0u;
    }

    uint8_t byteIndex = startBit / 8u;
    uint8_t bitInByte = startBit % 8u;
    uint8_t msbLinear = (dlc - 1u - byteIndex) * 8u + bitInByte;
    int16_t lsbLinear = (int16_t)msbLinear - (int16_t)(length - 1u);
    if(lsbLinear < 0){
        return 0; 
    }
    uint64_t raw = 0u;
    for(uint8_t i = 0u; i < dlc; i++){
        raw |= (uint64_t)data[i] << ((dlc - 1 - i) * 8u);
    }

    return (raw >> lsbLinear) & ((length == 64u) ? UINT64_MAX : ((1ULL << length) - 1ULL));
}

static int64_t SignExtend(uint64_t value, uint8_t length){
    uint64_t sign_bit = 1ULL << (length - 1u);
    if(value & sign_bit){
        return (int64_t)(value | ~(sign_bit - 1ULL));
    }
    return (int64_t)value;
}

static SignalValue DecodeSignal(const uint8_t data[8], const uint8_t dlc, uint8_t startBit, uint8_t length, uint8_t is_signed, float scale, float offset, uint8_t byteOrder, dataType type){
    SignalValue result = {0};
    if((dlc > 8u) || (length == 0u) || (length > 64u)){
        return result;
    }

    uint64_t raw;
    if(byteOrder == 0u){
        raw = ExtractBE(data, dlc, startBit, length);
    }else{
        raw = ExtractLE(data, dlc, startBit, length);
    }
    float phys = 0.0f;
    if(is_signed){
        int64_t signed_val = SignExtend(raw, length);
        phys = ((float)signed_val) * scale + offset;
    }else{
        phys = ((float)raw) * scale + offset;
    }
    switch(type){
        case SIGNAL_FLOAT:
            result.f = phys;
            break;
        case SIGNAL_BOOL:
            result.b = (bool)phys;
            break;
        case SIGNAL_SIGNED:
            result.i = (int64_t)phys;
            break;
        case SIGNAL_UNSIGNED:
            result.u = (uint64_t)phys;
            break;
    }
    return result;
}

static float DecodeFloat(const uint8_t data[8], const uint8_t dlc, uint8_t startBit, uint8_t length, uint8_t is_signed, float scale, float offset, uint8_t byteOrder)
{
    return DecodeSignal(data, dlc, startBit, length, is_signed, scale, offset, byteOrder, SIGNAL_FLOAT).f;
}

static bool DecodeBool(const uint8_t data[8], const uint8_t dlc, uint8_t startBit, uint8_t length, uint8_t byteOrder)
{
    return DecodeSignal(data, dlc, startBit, length, 0u, 1.0f, 0.0f, byteOrder, SIGNAL_BOOL).b;
}

static int64_t DecodeSigned(const uint8_t data[8], const uint8_t dlc, uint8_t startBit, uint8_t length, float scale, float offset, uint8_t byteOrder)
{
    return DecodeSignal(data, dlc, startBit, length, 1u, scale, offset, byteOrder, SIGNAL_SIGNED).i;
}

static uint64_t DecodeUnsigned(const uint8_t data[8], const uint8_t dlc, uint8_t startBit, uint8_t length, float scale, float offset, uint8_t byteOrder)
{
    return DecodeSignal(data, dlc, startBit, length, 0u, scale, offset, byteOrder, SIGNAL_UNSIGNED).u;
}

static void DecodeWheelSpeedsCs(CarState *cs){
    CanFrameEntry entry;
    if(!FindFrameIdCs(ID_WHEEL_SPEEDS, &entry)) return;
    float fl = DecodeFloat(entry.data, entry.dlc, 23u, 16u, 0u, 0.01f, -67.67f, 0u);
    float fr = DecodeFloat(entry.data, entry.dlc, 7u, 16u, 0u, 0.01f, -67.67f, 0u);
    float rl = DecodeFloat(entry.data, entry.dlc, 55u, 16u, 0u, 0.01f, -67.67f, 0u);
    float rr = DecodeFloat(entry.data, entry.dlc, 39u, 16u, 0u, 0.01f, -67.67f, 0u);
    float vEgoRaw = (fl + fr + rl + rr) * 0.25f;
    float v_ego, a_ego;
    UpdateSpeedKF(&speed_kf, vEgoRaw, &v_ego, &a_ego);
    cs->vEgo = v_ego;
    cs->aEgo = a_ego;
    cs->cruiseState.vEgoCluster = v_ego * 1.015f;
    cs->standStill = (fabsf(vEgoRaw) < 1e-3f);
}

static void DecodeSteeringCs(CarState *cs){
    CanFrameEntry angle_entry, entry, torque;
    int32_t scale = 66;
    if(FindFrameIdCs(ID_STEER_ANGLE_SENSOR, &angle_entry) == true){
        float angle = DecodeFloat(angle_entry.data, angle_entry.dlc, 3u, 12u, 1u, 1.5f, 0.0f, 0u);
        float frac = DecodeFloat(angle_entry.data, angle_entry.dlc, 39u, 4u, 1u, 0.1f, 0.0f, 0u);
        cs->steeringRateDeg = DecodeSigned(angle_entry.data, angle_entry.dlc, 35u, 12u, 1.0f, 0.0f, 0u);
        cs->steeringAngleDeg = angle + frac; 
    }
    if(FindFrameIdCs(ID_LTA_RELATED, &entry) == true){
        cs->steeringPressed = DecodeBool(entry.data, entry.dlc, 63u, 1u, 0u);        
    }
    if(FindFrameIdCs(ID_STEER_TORQUE, &torque) == true){
        cs->steeringTorque = DecodeSigned(torque.data, torque.dlc, 15u, 16u, 1.0f, 0.0f, 0u);
        cs->steeringTorqueEps = DecodeSigned(torque.data, torque.dlc, 47u, 16u, 1.0f, 0.0f, 0u) * scale;
    }
}

static void DecodeEngineAndPedalCs(CarState *cs){
    CanFrameEntry rpm_entry, gas_entry;
    if(FindFrameIdCs(ID_ENGINE_RPM, &rpm_entry) == true){
        cs->engineRpm = DecodeFloat(rpm_entry.data, rpm_entry.dlc, 7u, 16u, 1u, 0.78125f, 0.0f, 0u);
    }
    if(FindFrameIdCs(ID_GAS_PEDAL, &gas_entry) == true){
        cs->gasPressed = DecodeBool(gas_entry.data, gas_entry.dlc, 3u, 1u, 0u);
    }
}

static void DecodeBrakeCs(CarState *cs){
    CanFrameEntry entry, brake;
    if(FindFrameIdCs(ID_BRAKE_MODULE, &entry) == true){
        cs->brakePressed = DecodeBool(entry.data, entry.dlc, 37u, 1u, 0u);
    }
    if(FindFrameIdCs(ID_BODY_CONTROL_STATE, &brake) == true){
        cs->parkingBrake = DecodeBool(brake.data, brake.dlc, 60u, 1u, 0u);
    }
}

static void DecodeGearCs(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameIdCs(ID_GEAR_PACKET, &entry) == true){
        uint8_t gear = (uint8_t)DecodeUnsigned(entry.data, entry.dlc, 13u, 6u, 1.0f, 0.0f, 0u);
        switch (gear)
        {
        case 0: cs->gear = drive; break;
        case 1: cs->gear = s; break;
        case 8: cs->gear = neutral; break;
        case 16: cs->gear = reverse; break;
        case 32: cs->gear = park; break;
        default: cs->gear = gear_unknown; break;
        }
    }
}

static void DecodeCruiseCs(CarState *cs){
    CanFrameEntry pcm2, pcm;
    if(FindFrameIdCs(ID_PCM_CRUISE_2, &pcm2) == true){
        cs->accFaulted = DecodeBool(pcm2.data, pcm2.dlc, 47u, 1u, 0u);
        cs->cruiseState.available = DecodeBool(pcm2.data, pcm2.dlc, 15u, 1u, 0u);
        float speed = (float)DecodeUnsigned(pcm2.data, pcm2.dlc, 23u, 8u, 1.0f, 0.0f, 0u);
        cs->cruiseState.SetSpeed = speed * (1 / 3.6f);
        uint8_t low_speed_lockout = (uint8_t)DecodeUnsigned(pcm2.data, pcm2.dlc, 14u, 2u, 1.0f, 0.0f, 0u);
        if(cs->accFaulted && (low_speed_lockout == 2u)){
            cs->lockoutState = true;
        }else{
            cs->lockoutState = false;
        }
        cs->pcmFollowDistance = DecodeUnsigned(pcm2.data, pcm2.dlc, 12u, 2u, 1.0f, 0.0f, 0u);
    }
    if(FindFrameIdCs(ID_PCM_CRUISE, &pcm) == true){
        cs->cruiseState.enabled = DecodeBool(pcm.data, pcm.dlc, 5u, 1u, 0u);    
        uint8_t state = (uint8_t)DecodeUnsigned(pcm.data, pcm.dlc, 55u, 4u, 1.0f, 0.0f, 0u);
        cs->pcmAccStatus = state;
        cs->cruiseState.nonAdaptive = (state >= 1 && state <= 6);
        if(state == 7){
            cs->cruiseState.standStill = true;
        }else{
            cs->cruiseState.standStill = false;
        }
    }else{
        return;
    }
}

static void DecodeBlinkersCs(CarState *cs){
    CanFrameEntry blinkers;
    if(FindFrameIdCs(ID_BLINKERS_STATE, &blinkers) == true){
        uint8_t blinker = (uint8_t)DecodeUnsigned(blinkers.data, blinkers.dlc, 29u, 2u, 1.0f, 0.0f, 0u);
        cs->hazardLight = DecodeBool(blinkers.data, blinkers.dlc, 27u, 1u, 0u);
        switch(blinker){
            case 1: 
                if(cs->hazardLight){
                    cs->leftBlinker = true;
                    cs->rightBlinker = true;
                    break;
                }
                cs->leftBlinker = true;
                cs->rightBlinker = false;
                break;
            case 2u:
                if(cs->hazardLight){
                    cs->leftBlinker = true;
                    cs->rightBlinker = true;
                    break;
                }
                cs->leftBlinker = false;
                cs->rightBlinker = true;
                break;
            default:
                cs->leftBlinker = false;
                cs->rightBlinker = false;
                break;
        }
    }else{
        return;
    }
}

static void DecodeAccType(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameIdCs(ID_ACC_CONTROL, &entry) == true){
        cs->accType = DecodeUnsigned(entry.data, entry.dlc, 23u, 2u, 1.0f, 0.0f, 0u);
    }else{
        return;
    }
}

static void DecodeIpasCs(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameIdCs(ID_EPS_STATUS, &entry) == true){
        uint8_t state = DecodeUnsigned(entry.data, entry.dlc, 3u, 4u, 1.0f, 0.0f, 0u);
        switch (state)
        {
        case 3u:
            cs->ipasState = enabled;
            break;
        case 5u:
            cs->ipasState = override;
            break;
        case 1u:
            cs->ipasState = disabled;
            break;
        default:
            cs->ipasState = ipas_unknown;
            break;
        }
    }else{
        return;
    }
}


static void DecodeAccCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_ACCEL_CMD, &entry)) return;
    cc->actuators.accelCmd = DecodeFloat(entry.data, entry.dlc, 0u, 16u, 1u, 0.001f, 0.0f, 1u);
    cc->throttleCmd = DecodeFloat(entry.data, entry.dlc, 16u, 10u, 0u, 0.001f, 0.0f, 1u);
    cc->brakeCmd = DecodeFloat(entry.data, entry.dlc, 26u, 10u, 0u, 0.001f, 0.0f, 1u);
    cc->accEnable = DecodeBool(entry.data, entry.dlc, 36u, 1u, 1u);
    cc->emergency = DecodeBool(entry.data, entry.dlc, 37u, 1u, 1u);
}

static void DecodeSteerCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_STEER_CMD, &entry)) return;
    cc->actuators.steeringAngleDegCmd = DecodeFloat(entry.data, entry.dlc, 0u, 16u, 1u, 0.1f, 0.0f, 1u);
    cc->steeringEnable = DecodeBool(entry.data, entry.dlc, 16u, 1u, 1u);
}

static void DecodeSafetyCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_SAFETY_CMD, &entry)) return;
    cc->vcuEnabled = DecodeBool(entry.data, entry.dlc, 0u, 1u, 1u);
    cc->longActive = DecodeBool(entry.data, entry.dlc, 1u, 1u, 1u);
    cc->latActive = DecodeBool(entry.data, entry.dlc, 2u, 1u, 1u);
}

static void DecodeGearCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_GEAR_CMD, &entry)) return;
    uint8_t gear = DecodeUnsigned(entry.data, entry.dlc, 0u, 8u, 1.0f, 0.0f, 1u);
    switch (gear)
    {
    case 0: cc->gearCmd = Gear_NONE; break;
    case 1: cc->gearCmd = PARK; break;
    case 2: cc->gearCmd = REVERSE; break;
    case 3: cc->gearCmd = NEUTRAL; break;
    case 4: cc->gearCmd = DRIVE; break;
    case 5: cc->gearCmd = LOW; break; 
    default: cc->gearCmd = Gear_UNKNOWN; break;
    }
}

static void DecodeBlinkerCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_BODY_CMD, &entry)) return;
    uint8_t indicator = DecodeUnsigned(entry.data, entry.dlc, 0u, 2u, 1.0f, 0.0f, 1u);
    cc->hazardLights = DecodeBool(entry.data, entry.dlc, 2u, 1u, 1u);
    switch (indicator)
    {
    case 0: cc->turnIndicator = Turn_NONE; break;
    case 1: cc->turnIndicator = DISABLE; break;
    case 2: cc->turnIndicator = LEFT; break;
    case 3: cc->turnIndicator = RIGHT; break;
    default: cc->turnIndicator = Turn_UNKNOWN; break;
    }
}

void CarState_clear(CarState *cs){
    (void)memset(cs, 0, sizeof(*cs));
    cs->ipasState = disabled;
}

void CarControl_clear(CarControl *cc){
    (void)memset(cc, 0, sizeof(*cc));
}

void CarState_update(CarState *cs){
    DecodeWheelSpeedsCs(cs);
    DecodeSteeringCs(cs);
    DecodeEngineAndPedalCs(cs);
    DecodeBrakeCs(cs);
    DecodeGearCs(cs);
    DecodeCruiseCs(cs);
    DecodeBlinkersCs(cs);
    DecodeIpasCs(cs);
    DecodeAccType(cs);
    cs->last_update_tick = xTaskGetTickCount();
}

void CarControl_update(CarControl *cc){
    DecodeAccCc(cc);
    DecodeSteerCc(cc);
    DecodeSafetyCc(cc);
    DecodeGearCc(cc);
    DecodeBlinkerCc(cc);
    cc->last_update_tick = xTaskGetTickCount();
}

void CsDecodeTask(void *pv){
    (void)pv;
    uint8_t writeId;
    TickType_t lastWake = xTaskGetTickCount();
    KF1D_Init(&speed_kf, 0.0f, 0.0f, KF_A00, KF_A01, KF_A10, KF_A11, KF_C0, KF_C1, KF_K0, KF_K1);
    for(;;){
        writeId = (CsActiveId == 0) ? 1 : 0;
        CarState_clear(&gCarState[writeId]);
        CarState_update(&gCarState[writeId]);
        taskENTER_CRITICAL();
        CsActiveId = writeId;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

void CcDecodeTask(void *pv){
    (void)pv;
    uint8_t writeId;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;){
        writeId = (CcActiveId == 0) ? 1 : 0;
        CarControl_clear(&gCarControl[writeId]);
        CarControl_update(&gCarControl[writeId]);
        taskENTER_CRITICAL();
        CcActiveId = writeId;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}
