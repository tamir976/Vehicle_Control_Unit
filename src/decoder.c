#include "include/decoder.h"
#include "include/car_control.h"
#include "include/can_frame.h"
#include <string.h>
#include <math.h>

CarState gCarState[2] = {0};
CarControl gCarControl[2] = {0};
volatile uint8_t CsActiveId = 0;
volatile uint8_t CcActiveId = 0;

static bool FindFrameIdCs(uint32_t id, CanFrameEntry *out){
    if (CanCache_CopyFrame(&gCan0Cache, id, out) == TRUE) { return TRUE; }
    if (CanCache_CopyFrame(&gCan1Cache, id, out) == TRUE) { return TRUE; }    
    if (CanCache_CopyFrame(&gCan2Cache, id, out) == TRUE) { return TRUE; }
    return FALSE;
}
static bool FindFrameIdCc(uint32_t id, CanFrameEntry *out){
    if(CanCache_CopyFrame(&gCan3Cache, id, out) == TRUE){
        return true;
    }
    return false;
}

static uint64_t ExtractUnsignedLE(const uint8_t data[8], uint16_t startBit, uint8_t length)
{
    uint64_t raw = 0;
    for (uint8_t i = 0; i < 8u; i++)
    {
        raw |= ((uint64_t)data[i]) << (i * 8u);
    }
    if(length == 64u){
        return raw >> startBit;
    }
    return (raw >> startBit) & ((1ULL << length) - 1ULL);
}

static uint64_t ExtractUnsignedBE(const uint8_t data[8], uint16_t startBit, uint8_t length)
{
    uint64_t raw = 0u;
    int32_t bitPos = (int32_t)startBit;

    for (uint8_t i = 0u; i < length; i++)
    {
        uint8_t byteIndex = (uint8_t)(bitPos / 8);
        uint8_t bitIndex = (uint8_t)(bitPos % 8);
        uint8_t bit = (data[byteIndex] >> bitIndex) & 0x01u;
        raw = (raw << 1u) | bit;

        if ((bitPos % 8) == 0)
        {
            bitPos += 15;
        }
        else
        {
            bitPos -= 1;
        }
    }

    return raw;
}

static int64_t SignExtendU64(uint64_t value, uint8_t length){
    if((length == 0u) || (length >= 64u)){
        return (int64_t)value;
    }
    uint64_t sign_bit = 1ULL << (length - 1u);
    uint64_t mask = (1ULL << length) - 1ULL;
    value &= mask;
    if (value & sign_bit) {
        value |= ~mask;
    }
    return (int64_t)value;
}

static float DecodePhysLE(const uint8_t data[8], uint16_t startBit, uint8_t length, uint8_t is_signed, float scale, float offset){
    uint64_t raw = ExtractUnsignedLE(data, startBit, length);
    if(is_signed){
        int64_t signed_val = SignExtendU64(raw, length);
        return ((float)signed_val) * scale + offset;
    }
    return ((float)raw) * scale + offset;
}

static float DecodePhysBE(const uint8_t data[8], uint16_t startBit, uint8_t length, uint8_t is_signed, float scale, float offset){
    uint64_t raw = ExtractUnsignedBE(data, startBit, length);
    if(is_signed){
        int64_t signed_val = SignExtendU64(raw, length);
        return ((float)signed_val) * scale + offset;
    }
    return ((float)raw) * scale + offset;
}

static void DecodeSpeedCs(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameIdCs(ID_SPEED, &entry) == FALSE) return;
    float speed = DecodePhysBE(entry.data, 47u, 16u, 0u, 0.01f, 0.0f);
    cs->cruiseState.speedCluster = speed;
}

static void DecodeWheelSpeedsCs(CarState *cs){
    CanFrameEntry entry;
    if(!FindFrameIdCs(ID_WHEEL_SPEEDS, &entry)) return;
    float fl = DecodePhysBE(entry.data, 23u, 16u, 0u, 0.01f, -67.67f);
    float fr = DecodePhysBE(entry.data, 7u, 16u, 0u, 0.01f, -67.67f);
    float rl = DecodePhysBE(entry.data, 55u, 16u, 0u, 0.01f, -67.67f);
    float rr = DecodePhysBE(entry.data, 39u, 16u, 0u, 0.01f, -67.67f);
    
    cs->vEgo = (fl + fr + rl + rr) * 0.25f;
    cs->standStill = (fabsf(cs->vEgo) < 1e-3f);
}

static void DecodeSteeringCs(CarState *cs){
    CanFrameEntry angle, torque;
    if(FindFrameIdCs(ID_STEER_ANGLE_SENSOR, &angle) == true){
        float main = DecodePhysBE(angle.data, 3u, 12u, 1u, 1.5f, 0.0f);
        float frac = DecodePhysBE(angle.data, 39u, 4u, 1u, 0.1f, 0.0f);
        cs->steeringAngleDeg = main + frac; 
        cs->steeringRateDeg = DecodePhysBE(angle.data, 35u, 12u, 1u, 1.0f, 0.0f);
    }
    if(FindFrameIdCs(ID_STEER_TORQUE, &torque) == true){
        cs->steeringTorque = DecodePhysBE(torque.data, 15u, 16u, 1u, 1.0f, 0.0f);
        cs->steeringPressed = fabsf(cs->steeringTorque) > 1.0f;
    }
}

static void DecodeDynamicCs(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameIdCs(ID_KINEMATICS, &entry) == FALSE) return;
    cs->aEgo = DecodePhysBE(entry.data, 17u, 10u, 0u, 0.03589f, -18.375f);
    cs->yawRate = DecodePhysBE(entry.data, 1u, 10u, 0u, 0.244f, -125.0f);
}

static void DecodeEngineAndPedalCs(CarState *cs){
    CanFrameEntry rpm_entry, gas_entry;
    if(FindFrameIdCs(ID_ENGINE_RPM, &rpm_entry) == TRUE){
        cs->engineRpm = DecodePhysBE(rpm_entry.data, 7u, 16u, 1u, 0.78125f, 0.0f);
    }
    if(FindFrameIdCs(ID_GAS_PEDAL, &gas_entry) == TRUE){
        cs->gasPedal = DecodePhysBE(gas_entry.data, 55u, 8u, 0u, 0.5f, 0.0f);
        if(cs->gasPedal > 0.01f){
            cs->gasPressed = true;
        }else{
            cs->gasPressed = false;
        }
    }
}

static void DecodeBrakeCs(CarState *cs){
    CanFrameEntry entry, brake;
    if(!FindFrameIdCs(ID_BRAKE_MODULE, &entry)) return;
    cs->brakePressed = ExtractUnsignedBE(entry.data, 37u, 1u);
    if(!FindFrameIdCs(ID_BODY_CONTROL_STATE, &brake)) return;
    cs->parkingBrake = ExtractUnsignedBE(brake.data, 60u, 1u);
}

static void DecodeGearCs(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameIdCs(ID_GEAR_PACKET, &entry) == FALSE) return;
    uint8_t gear = ExtractUnsignedBE(entry.data, 47u, 4u);
    switch (gear)
    {
    case 0: cs->gear = park; break;
    case 1: cs->gear = reverse; break;
    case 2: cs->gear = neutral; break;
    case 3: cs->gear = drive; break;
    default: cs->gear = gear_unknown; break;
    }
}

static void DecodeCruiseCs(CarState *cs){
    CanFrameEntry pcm2, pcm;
    bool has_pcm2 = FindFrameIdCs(ID_PCM_CRUISE_2, &pcm2);
    if(has_pcm2){
        cs->accFaulted = ExtractUnsignedBE(pcm2.data, 47u, 1u);
        cs->cruiseState.available = ExtractUnsignedBE(pcm2.data, 15u, 1u);
        cs->cruiseState.speed = DecodePhysBE(pcm2.data, 23u, 8u, 0u, 1.0f, 0.0f) * 0.27778f;
    }
    if(FindFrameIdCs(ID_PCM_CRUISE, &pcm)){
        cs->cruiseState.enabled = ExtractUnsignedBE(pcm.data, 5u, 1u);
        uint8_t state = ExtractUnsignedBE(pcm.data, 55u, 4u);
        cs->cruiseState.standStill = (state == 7);
        cs->cruiseState.nonAdaptive = (state >= 1 && state <= 6);
    }
    if(has_pcm2){
        uint8_t low_speed_lockout = ExtractUnsignedBE(pcm2.data, 14u, 2u);
        cs->lockoutState = (low_speed_lockout == 2u);
    }
}
static void DecodeBodyCs(CarState *cs){
    CanFrameEntry blinkers;
    if(FindFrameIdCs(ID_BLINKERS_STATE, &blinkers)){
        uint8_t turn = ExtractUnsignedBE(blinkers.data, 29u, 2u);
        cs->leftBlinker = (turn == 1u);
        cs->rightBlinker = (turn == 2u);
    }
}

static void DecodeIpasCs(CarState *cs){
    CanFrameEntry entry;
    if(!FindFrameIdCs(ID_EPS_STATUS, &entry)) return;
    uint8_t state = ExtractUnsignedBE(entry.data, 3u, 4u);
    cs->ipasActive = (state == 3u);
}

static void DecodeButtonsCs(CarState *cs){
    CanFrameEntry entry;
    static uint8_t prev = 0;
    if(!FindFrameIdCs(ID_ACC_CONTROL, &entry)) return;
    uint8_t current = ExtractUnsignedBE(entry.data, 23u, 2u);
    cs->buttonPressed = false;
    if(current != prev){
        cs->buttonPressed = true;
        cs->buttonType = gadAdjustCruise;
    }
    prev = current;
}

static void DecodeAccCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_ACCEL_CMD, &entry)) return;
    cc->actuators.accelCmd = DecodePhysLE(entry.data, 0u, 16u, 1u, 0.001f, 0.0f);
    cc->throttleCmd = DecodePhysLE(entry.data, 16u, 10u, 0u, 0.001f, 0.0f);
    cc->brakeCmd = DecodePhysLE(entry.data, 26u, 10u, 0u, 0.001f, 0.0f);
    cc->accEnable = ExtractUnsignedLE(entry.data, 36u, 1u);
    cc->emergency = ExtractUnsignedLE(entry.data, 37u, 1u);
}

static void DecodeSteerCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_STEER_CMD, &entry)) return;
    cc->actuators.steeringAngleDegCmd = DecodePhysLE(entry.data, 0u, 16u, 1u, 0.1f, 0.0f);
    cc->steeringEnable = ExtractUnsignedLE(entry.data, 16u, 1u);
}

static void DecodeSafetyCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_SAFETY_CMD, &entry)) return;
    cc->vcuEnabled = ExtractUnsignedLE(entry.data, 0u, 1u);
    cc->longActive = ExtractUnsignedLE(entry.data, 1u, 1u);
    cc->latActive = ExtractUnsignedLE(entry.data, 2u, 1u);
}

static void DecodeGearCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_GEAR_CMD, &entry)) return;
    uint8_t gear = DecodePhysLE(entry.data, 0u, 8u, 0u, 1.0f, 0.0f);
    switch (gear)
    {
    case 0: cc->gearCmd = Gear_NONE; break;
    case 1: cc->gearCmd = PARK; break;
    case 2: cc->gearCmd = REVERSE; break;
    case 3: cc->gearCmd = NEUTRAL; break;
    case 4: cc->gearCmd = DRIVE; break;  
    default: cc->gearCmd = Gear_UNKNOWN; break;
    }
}

static void DecodeBlinkerCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_BODY_CMD, &entry)) return;
    uint8_t indicator = DecodePhysLE(entry.data, 0u, 2u, 0u, 1.0f, 0.0f);
    switch (indicator)
    {
    case 0: cc->turnIndicator = Turn_NONE; break;
    case 1: cc->turnIndicator = DISABLE; break;
    case 2: cc->turnIndicator = LEFT; break;
    case 3: cc->turnIndicator = RIGHT; break;
    default: cc->turnIndicator = Turn_UNKNOWN; break;
    }
}

static void DecodeHazardCc(CarControl *cc){
    CanFrameEntry entry;
    if(!FindFrameIdCc(ID_VCU_BODY_CMD, &entry)) return;
    cc->hazardLights = ExtractUnsignedLE(entry.data, 2u, 1u);
}

void CarState_clear(CarState *cs){
    (void)memset(cs, 0, sizeof(*cs));
}

void CarControl_clear(CarControl *cc){
    (void)memset(cc, 0, sizeof(*cc));
}

void CarState_update(CarState *cs){
    DecodeSpeedCs(cs);
    DecodeWheelSpeedsCs(cs);
    DecodeSteeringCs(cs);
    DecodeDynamicCs(cs);
    DecodeEngineAndPedalCs(cs);
    DecodeBrakeCs(cs);
    DecodeGearCs(cs);
    DecodeCruiseCs(cs);
    DecodeBodyCs(cs);
    DecodeIpasCs(cs);
    DecodeButtonsCs(cs);
    cs->last_update_tick = xTaskGetTickCount();
}

void CarControl_update(CarControl *cc){
    DecodeAccCc(cc);
    DecodeSteerCc(cc);
    DecodeSafetyCc(cc);
    DecodeGearCc(cc);
    DecodeBlinkerCc(cc);
    DecodeHazardCc(cc);
    cc->last_update_tick = xTaskGetTickCount();
}

void CsDecodeTask(void *pv){
    (void)pv;
    uint8_t writeId;
    TickType_t lastWake = xTaskGetTickCount();
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
