#include "include/decoder.h"
#include "include/car_control.h"
#include "include/can_frame.h"
#include <string.h>

#define STEER_OFFSET_DT_SEC     (0.01f)
#define STEER_OFFSET_RC_SEC     (60.0f)
#define STEER_OFFSET_ALPHA      (STEER_OFFSET_DT_SEC / (STEER_OFFSET_RC_SEC + STEER_OFFSET_DT_SEC))

static uint8_t sAccurateSteerAngleSeen = 0u;
static uint8_t sAngleOffsetInitialized = 0u;
static float sSteerAngleOffsetDeg = 0.0f;
static CarControl sCarControlBuf[2];
static CarState sCarStateBuf[2];
static volatile uint8_t sCarControlActiveIdx = 0u;
static volatile uint8_t sCarStateActiveIdx = 0u;

static boolean FindFrameByBus(uint32_t id, CanFrameEntry *out)
{
    if (CanCache_CopyFrame(&gCan0Cache, id, out) == TRUE) { return TRUE; }
    if (CanCache_CopyFrame(&gCan1Cache, id, out) == TRUE) { return TRUE; }
    if (CanCache_CopyFrame(&gCan3Cache, id, out) == TRUE) { return TRUE; }
    return FALSE;
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
    CanFrameEntry entry;
    if(FindFrameByBus(ID_SPEED, &entry) == FALSE) return;
    float speed = DecodePhysBE(entry.data, 47u, 16u, 0u, 0.01f, 0.0f);
    cs->out.vEgo = speed;
    cs->last_update_tick = entry.lastRxTick;
}
static void DecodeWheelSpeeds(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameByBus(ID_WHEEL_SPEEDS, &entry) == FALSE) return;
    cs->wheel_speed_fr = DecodePhysBE(entry.data, 7u, 16u, 0u, 0.01f, -67.67f);
    cs->wheel_speed_fl = DecodePhysBE(entry.data, 23u, 16u, 0u, 0.01f, -67.67f);
    cs->wheel_speed_rr = DecodePhysBE(entry.data, 39u, 16u, 0u, 0.01f, -67.67f);
    cs->wheel_speed_rl = DecodePhysBE(entry.data, 55u, 16u, 0u, 0.01f, -67.67f);
}
static void DecodeSteering(CarState *cs){
    CanFrameEntry angle;
    CanFrameEntry torque;
    boolean hasAngle = FindFrameByBus(ID_STEER_ANGLE_SENSOR, &angle);
    boolean hasTorque = FindFrameByBus(ID_STEER_TORQUE, &torque);
    float torqueSensorAngleDeg;
    uint8_t steerAngleInitializing;

    if(hasAngle == TRUE){
        float angle_main = DecodePhysBE(angle.data, 3u, 12u, 1u, 1.5f, 0.0f);
        float angle_frac = DecodePhysBE(angle.data, 39u, 4u, 1u, 0.1f, 0.0f);
        float steer_rate = DecodePhysBE(angle.data, 35u, 12u, 1u, 1.0f, 0.0f);
        cs->out.steeringAngleDeg = angle_main + angle_frac;
        cs->out.steeringRateDeg = steer_rate;
    }

    if(hasTorque == TRUE) {
        cs->out.steeringTorque = DecodePhysBE(torque.data, 15u, 16u, 1u, 1.0f, 0.0f);
        cs->out.steeringTorqueEps = DecodePhysBE(torque.data, 47u, 16u, 1u, 1.0f, 0.0f);

        torqueSensorAngleDeg = DecodePhysBE(torque.data, 31u, 16u, 1u, 0.0573f, 0.0f);
        steerAngleInitializing = DecodeBoolBE(torque.data, 3u);

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
    CanFrameEntry entry;
    if(FindFrameByBus(ID_KINEMATICS, &entry) == FALSE) return;
    cs->accel_x = DecodePhysBE(entry.data, 17u, 10u, 0u, 0.03589f, -18.375f);
    cs->accel_y = DecodePhysBE(entry.data, 33u, 10u, 0u, 0.03589f, -18.375f);
    cs->yaw_rate = DecodePhysBE(entry.data, 1u, 10u, 0u, 0.244f, -125.0f);
    cs->out.aEgo = cs->accel_x;
}
static void DecodeEngineAndPedal(CarState *cs){
    CanFrameEntry rpm_entry;
    CanFrameEntry gas_entry;
    if(FindFrameByBus(ID_ENGINE_RPM, &rpm_entry) == TRUE){
        cs->engine_rpm = DecodePhysBE(rpm_entry.data, 7u, 16u, 1u, 0.78125f, 0.0f);
    }
    if(FindFrameByBus(ID_GAS_PEDAL_HYBRID, &gas_entry) == TRUE){
        cs->gas_pedal = DecodePhysBE(gas_entry.data, 23u, 8u, 0u, 0.005f, 0.0f);
    }
}
static void DecodeGear(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameByBus(ID_GEAR_PACKET, &entry) == FALSE) return;
    cs->gear = (uint8_t)ExtractUnsignedBE(entry.data, 13u, 6u);
}

static void DecodeCruise(CarState *cs){
    CanFrameEntry pcm2_entry;
    CanFrameEntry pcmsm_entry;
    if(FindFrameByBus(ID_PCM_CRUISE_2, &pcm2_entry) == TRUE){
        cs->brake_pressed = DecodeBoolBE(pcm2_entry.data, 3u);
        cs->pcm_follow_distance = (uint8_t)ExtractUnsignedBE(pcm2_entry.data, 12u, 2u);
        uint8_t main_on = DecodeBoolBE(pcm2_entry.data, 15u);
        if(!main_on){
              cs->out.cruiseState.enabled = 0u;
        }
    }
    if(FindFrameByBus(ID_PCM_CRUISE_SM, &pcmsm_entry) == TRUE){
        uint8_t cruiseState = (uint8_t)ExtractUnsignedBE(pcmsm_entry.data, 11u, 4u);
        cs->out.cruiseState.enabled = (cruiseState == 6u) ? 1u : 0u;
    }
}
static void DecodeBody(CarState *cs){
    CanFrameEntry body;
    CanFrameEntry blinkers;
    if(FindFrameByBus(ID_BODY_CONTROL_STATE, &body) == TRUE){
        cs->door_open_fl = DecodeBoolBE(body.data, 45u);
        cs->door_open_fr = DecodeBoolBE(body.data, 44u);
        cs->door_open_rl = DecodeBoolBE(body.data, 42u);
        cs->door_open_rr = DecodeBoolBE(body.data, 43u);
    }
    if(FindFrameByBus(ID_BLINKERS_STATE, &blinkers) == TRUE){
        uint8_t turn = (uint8_t)ExtractUnsignedBE(blinkers.data, 29u, 2u);
        cs->left_blinker = (turn == 1u) ? 1u : 0u;
        cs->right_blinker = (turn == 2u) ? 1u : 0u;
    }
}

static void DecodeVsc(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameByBus(ID_VSC1S07, &entry) == FALSE) return;
    cs->abs_active = DecodeBoolBE(entry.data, 2u);
    cs->traction_control_active = DecodeBoolBE(entry.data, 1u);
    cs->gvc = DecodePhysBE(entry.data, 39u, 8u, 1u, 0.04f, 0.0f);
    cs->eps_active = 0u;
}

static void DecodeAccControl(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameByBus(ID_ACC_CONTROL, &entry) == FALSE) return;
    cs->acc_type = (uint8_t)ExtractUnsignedBE(entry.data, 23u, 2u);
}
static void DecodeIpas(CarState *cs){
    CanFrameEntry entry;
    if(FindFrameByBus(ID_STEERING_IPAS, &entry) == FALSE) return;
    uint8_t state = (uint8_t)ExtractUnsignedBE(entry.data, 7u, 4u);
    cs->ipas_active = state == 3u ? 1u : 0u;
}

static void DeriveState(CarState *cs){
    cs->out.standstill = (fabsf(cs->out.vEgo) < 0.1f) ? 1u : 0u;
}

static void DecodeSteeringControl(CarControl *cc){
    CanFrameEntry steer_entry;
    if(FindFrameByBus(ID_STEER_ANGLE, &steer_entry) == FALSE) return;
}

void CarState_clear(CarState *cs){
    (void)memset(cs, 0, sizeof(*cs));
    sAccurateSteerAngleSeen = 0u;
    sAngleOffsetInitialized = 0u;
    sSteerAngleOffsetDeg = 0.0f;
}

void CarControl_clear(CarControl *cc){
    (void)memset(cc, 0, sizeof(*cc));
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

void CarControl_update(CarControl *cc){
    DecodeSteeringControl(cc);
    cc->last_update_tick = xTaskGetTickCount();
}

void CarState_GetSnapshot(CarState *out)
{
    uint8_t activeIdx;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    activeIdx = sCarStateActiveIdx;
    (void)memcpy(out, &sCarStateBuf[activeIdx], sizeof(*out));
    taskEXIT_CRITICAL();
}

void CarControl_GetSnapshot(CarControl *out)
{
    uint8_t activeIdx;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    activeIdx = sCarControlActiveIdx;
    (void)memcpy(out, &sCarControlBuf[activeIdx], sizeof(*out));
    taskEXIT_CRITICAL();
}

void DecoderTask(void *pv)
{
    uint8_t inactiveStateIdx;
    uint8_t inactiveControlIdx;
    CarState *nextState;
    CarControl *nextControl;

    (void)pv;
    CarState_clear(&sCarStateBuf[0]);
    CarState_clear(&sCarStateBuf[1]);
    CarControl_clear(&sCarControlBuf[0]);
    CarControl_clear(&sCarControlBuf[1]);

    for(;;){
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10u));

        taskENTER_CRITICAL();
        inactiveStateIdx = (uint8_t)(1u - sCarStateActiveIdx);
        inactiveControlIdx = (uint8_t)(1u - sCarControlActiveIdx);
        taskEXIT_CRITICAL();

        nextState = &sCarStateBuf[inactiveStateIdx];
        nextControl = &sCarControlBuf[inactiveControlIdx];

        CarState_clear(nextState);
        CarControl_clear(nextControl);
        CarState_update(nextState);
        CarControl_update(nextControl);

        taskENTER_CRITICAL();
        sCarStateActiveIdx = inactiveStateIdx;
        sCarControlActiveIdx = inactiveControlIdx;
        taskEXIT_CRITICAL();
    }
}
