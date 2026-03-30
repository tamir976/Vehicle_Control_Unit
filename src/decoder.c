#include "include/decoder.h"
#include <string.h>

#define STEER_OFFSET_DT_SEC     (0.01f)
#define STEER_OFFSET_RC_SEC     (60.0f)
#define STEER_OFFSET_ALPHA      (STEER_OFFSET_DT_SEC / (STEER_OFFSET_RC_SEC + STEER_OFFSET_DT_SEC))

static uint8_t sAccurateSteerAngleSeen = 0u;
static uint8_t sAngleOffsetInitialized = 0u;
static float sSteerAngleOffsetDeg = 0.0f;

static const CanFrameEntry *CanCache_Fetch(const CanFrameCache *cache, uint32_t id)
{
    uint32_t index = CanHash(id);
    uint32_t start = index;
    
    taskENTER_CRITICAL();
    while (1)
    {
        const CanFrameEntry *entry = &cache->entries[index];
        if(entry->used != 0u && entry->id == id){
            if(entry->valid == TRUE){
                taskEXIT_CRITICAL();
                return entry;
            }else{
                taskEXIT_CRITICAL();
                return NULL;
            }
        }
        index = (index + 1u) & (CAN_CACHE_SIZE - 1u);
        if (index == start)
        {
            taskEXIT_CRITICAL();
            return NULL;
        }
    }
}

static const CanFrameEntry *FindFrameByBus(uint32_t id)
{
    const CanFrameEntry *entry;
    entry = CanCache_Fetch(&gCan0Cache, id);
    if (entry != NULL) { return entry; }
    entry = CanCache_Fetch(&gCan1Cache, id);
    if (entry != NULL) { return entry; }
    entry = CanCache_Fetch(&gCan3Cache, id);
    if(entry != NULL) { return entry; }
    return NULL;
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

static uint8_t DecodeBoolLE(const uint8_t data[8], uint16_t startBit){
    uint64_t raw = ExtractUnsignedLE(data, startBit, 1u);
    return (raw != 0u) ? 1u : 0u;
}

static uint8_t DecodeBoolBE(const uint8_t data[8], uint16_t startBit){
    uint64_t raw = ExtractUnsignedBE(data, startBit, 1u);
    return (raw != 0u) ? 1u : 0u;
}


static void DecodeSpeed(CarState *cs){
    const CanFrameEntry *entry = FindFrameByBus(ID_SPEED);
    if(entry == NULL) return;
    float speed = DecodePhysBE(entry->data, 47u, 16u, 0u, 0.01f, 0.0f);
    cs->out.vEgo = speed;
    cs->last_update_tick = entry->lastRxTick;
}
static void DecodeWheelSpeeds(CarState *cs){
    const CanFrameEntry *entry = FindFrameByBus(ID_WHEEL_SPEEDS);
    if(entry == NULL) return;
    cs->wheel_speed_fr = DecodePhysBE(entry->data, 7u, 16u, 0u, 0.01f, -67.67f);
    cs->wheel_speed_fl = DecodePhysBE(entry->data, 23u, 16u, 0u, 0.01f, -67.67f);
    cs->wheel_speed_rr = DecodePhysBE(entry->data, 39u, 16u, 0u, 0.01f, -67.67f);
    cs->wheel_speed_rl = DecodePhysBE(entry->data, 55u, 16u, 0u, 0.01f, -67.67f);
}
static void DecodeSteering(CarState *cs){
    const CanFrameEntry *angle = FindFrameByBus(ID_STEER_ANGLE_SENSOR);
    const CanFrameEntry *torque = FindFrameByBus(ID_STEER_TORQUE);
    float torqueSensorAngleDeg;
    uint8_t steerAngleInitializing;

    if(angle != NULL){
        float angle_main = DecodePhysBE(angle->data, 3u, 12u, 1u, 1.5f, 0.0f);
        float angle_frac = DecodePhysBE(angle->data, 39u, 4u, 1u, 0.1f, 0.0f);
        float steer_rate = DecodePhysBE(angle->data, 35u, 12u, 1u, 1.0f, 0.0f);
        cs->out.steeringAngleDeg = angle_main + angle_frac;
        cs->out.steeringRateDeg = steer_rate;
    }

    if(torque != NULL) {
        cs->out.steeringTorque = DecodePhysBE(torque->data, 15u, 16u, 1u, 1.0f, 0.0f);
        cs->out.steeringTorqueEps = DecodePhysBE(torque->data, 47u, 16u, 1u, 1.0f, 0.0f);

        torqueSensorAngleDeg = DecodePhysBE(torque->data, 31u, 16u, 1u, 0.0573f, 0.0f);
        steerAngleInitializing = DecodeBoolBE(torque->data, 3u);

        if ((fabsf(torqueSensorAngleDeg) > 1e-3f) && (steerAngleInitializing == 0u))
        {
            sAccurateSteerAngleSeen = 1u;
        }

        if ((sAccurateSteerAngleSeen != 0u) &&
            (fabsf(cs->out.steeringAngleDeg) < 90.0f) &&
            (fabsf(cs->out.steeringRateDeg) < 100.0f))
        {
            float offsetSample = torqueSensorAngleDeg - cs->out.steeringAngleDeg;

            if (sAngleOffsetInitialized != 0u)
            {
                sSteerAngleOffsetDeg = ((1.0f - STEER_OFFSET_ALPHA) * sSteerAngleOffsetDeg) + (STEER_OFFSET_ALPHA * offsetSample);
            }
            else
            {
                sSteerAngleOffsetDeg = offsetSample;
                sAngleOffsetInitialized = 1u;
            }
        }

        if (sAngleOffsetInitialized != 0u)
        {
            cs->out.steeringAngleOffsetDeg = sSteerAngleOffsetDeg;
            cs->out.steeringAngleDeg = torqueSensorAngleDeg - sSteerAngleOffsetDeg;
        }
    }
}
static void DecodeDynamic(CarState *cs){
    const CanFrameEntry *entry = FindFrameByBus(ID_KINEMATICS);
    if(entry == NULL) return;
    cs->accel_x = DecodePhysBE(entry->data, 17u, 10u, 0u, 0.03589f, -18.375f);
    cs->accel_y = DecodePhysBE(entry->data, 33u, 10u, 0u, 0.03589f, -18.375f);
    cs->yaw_rate = DecodePhysBE(entry->data, 1u, 10u, 0u, 0.244f, -125.0f);
    cs->out.aEgo = cs->accel_x;
}
static void DecodeEngineAndPedal(CarState *cs){
    const CanFrameEntry *rpm_entry = FindFrameByBus(ID_ENGINE_RPM);
    const CanFrameEntry *gas_entry = FindFrameByBus(ID_GAS_PEDAL_HYBRID);
    if(rpm_entry != NULL){
        cs->engine_rpm = DecodePhysBE(rpm_entry->data, 7u, 16u, 1u, 0.78125f, 0.0f);
    }
    if(gas_entry != NULL){
        cs->gas_pedal = DecodePhysBE(gas_entry->data, 23u, 8u, 0u, 0.005f, 0.0f);
    }
}
static void DecodeGear(CarState *cs){
    const CanFrameEntry *entry = FindFrameByBus(ID_GEAR_PACKET);
    if(entry == NULL) return;
    cs->gear = (uint8_t)ExtractUnsignedBE(entry->data, 13u, 6u);
}

static void DecodeCruise(CarState *cs){
    const CanFrameEntry *pcm2_entry = FindFrameByBus(ID_PCM_CRUISE_2);
    const CanFrameEntry *pcmsm_entry = FindFrameByBus(ID_PCM_CRUISE_SM);
    if(pcm2_entry != NULL){
        cs->brake_pressed = DecodeBoolBE(pcm2_entry->data, 3u);
        cs->pcm_follow_distance = (uint8_t)ExtractUnsignedBE(pcm2_entry->data, 12u, 2u);
        uint8_t main_on = DecodeBoolBE(pcm2_entry->data, 15u);
        if(!main_on){
              cs->out.cruiseState.enabled = 0u;
        }
    }
    if(pcmsm_entry != NULL){
        uint8_t cruiseState = (uint8_t)ExtractUnsignedBE(pcmsm_entry->data, 11u, 4u);
        cs->out.cruiseState.enabled = (cruiseState == 6u) ? 1u : 0u;
    }
}
static void DecodeBody(CarState *cs){
    const CanFrameEntry *body = FindFrameByBus(ID_BODY_CONTROL_STATE);
    const CanFrameEntry *blinkers = FindFrameByBus(ID_BLINKERS_STATE);
    if(body != NULL){
        cs->door_open_fl = DecodeBoolBE(body->data, 45u);
        cs->door_open_fr = DecodeBoolBE(body->data, 44u);
        cs->door_open_rl = DecodeBoolBE(body->data, 42u);
        cs->door_open_rr = DecodeBoolBE(body->data, 43u);
    }
    if(blinkers != NULL){
        uint8_t turn = (uint8_t)ExtractUnsignedBE(blinkers->data, 29u, 2u);
        cs->left_blinker = (turn == 1u) ? 1u : 0u;
        cs->right_blinker = (turn == 2u) ? 1u : 0u;
    }
}

static void DecodeVsc(CarState *cs){
    const CanFrameEntry *entry = FindFrameByBus(ID_VSC1S07);
    if(entry == NULL) return;
    cs->abs_active = DecodeBoolBE(entry->data, 2u);
    cs->traction_control_active = DecodeBoolBE(entry->data, 1u);
    cs->gvc = DecodePhysBE(entry->data, 39u, 8u, 1u, 0.04f, 0.0f);
    cs->eps_active = 0u;
}

static void DecodeAccControl(CarState *cs){
    const CanFrameEntry *entry = FindFrameByBus(ID_ACC_CONTROL);
    if(entry == NULL) return;
    cs->acc_type = (uint8_t)ExtractUnsignedBE(entry->data, 23u, 2u);
}
static void DecodeIpas(CarState *cs){
    const CanFrameEntry *entry = FindFrameByBus(ID_STEERING_IPAS);
    if(entry == NULL) return;
    uint8_t state = (uint8_t)ExtractUnsignedBE(entry->data, 7u, 4u);
    cs->ipas_active = state == 3u ? 1u : 0u;
}

static void DeriveState(CarState *cs){
    cs->out.standstill = (fabsf(cs->out.vEgo) < 0.1f) ? 1u : 0u;
}

void CarState_clear(CarState *cs){
    (void)memset(cs, 0, sizeof(*cs));
    sAccurateSteerAngleSeen = 0u;
    sAngleOffsetInitialized = 0u;
    sSteerAngleOffsetDeg = 0.0f;
}

void CarState_update(CarState *cs){
    DecodeSpeed(cs);
    DecodeWheelSpeeds(cs);
    DecodeSteering(cs);
    DecodeDynamic(cs);
    DecodeEngineAndPedal(cs);
    DecodeGear(cs);
    DecodeCruise(cs);
    DecodeBody(cs);
    DecodeVsc(cs);
    DecodeAccControl(cs);
    DecodeIpas(cs);
    DeriveState(cs);
    cs->last_update_tick = xTaskGetTickCount();
}
