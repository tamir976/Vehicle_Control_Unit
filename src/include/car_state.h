
#ifndef INCLUDE_CAR_STATE_H_
#define INCLUDE_CAR_STATE_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <stdbool.h>

#define ID_KINEMATICS (0x24u)
#define ID_STEER_ANGLE_SENSOR (0x25u)
#define ID_WHEEL_SPEEDS (0xAAu)
#define ID_SPEED (0xB4u)
#define ID_BRAKE_MODULE (0x226u)
#define ID_GAS_PEDAL (0x2C1u)
#define ID_STEER_TORQUE (0x260u)
#define ID_EPS_STATUS (0x262u)
#define ID_STEERING_LKA (0x2E4u)
#define ID_STEERING_LTA (0x191)
#define ID_PCM_CRUISE_2 (0x1D3u)
#define ID_PCM_CRUISE_SM (0x399u)
#define ID_DSU_CRUISE (0x365u)
#define ID_ACC_CONTROL (0x343u)
#define ID_GEAR_PACKET (0x3BCu)
#define ID_GEAR_HYBRID (0x127u)
#define ID_BLINKERS_STATE (0x614u)
#define ID_BODY_CONTROL_STATE (0x620u)
#define ID_LIGHT_STALK (0x622u)
#define ID_VSC1S07 (0x320u)
#define ID_ESP_CONTROL (0x3B7u)
#define ID_LKAS_HUD (0x412u)
#define ID_PCS_HUD (0x411u)
#define ID_BSM (0x3B7u)
#define ID_STEERING_IPAS (0x266u)
#define ID_STEERING_IPAS_COMMA (0x167u)
#define ID_PCM_CRUISE (0x1D2u)
#define ID_PRE_COLLISION (0x283u)
#define ID_PRE_COLLISION_2 (0x344)
#define ID_ENGINE_RPM (0x1C4u)
#define ID_GAS_PEDAL_HYBRID (0x245u)
#define ID_LTA_RELATED (0x371u)


typedef struct {
	bool enabled; //cruise enable is used for acc
    float SetSpeed;
    float vEgoCluster;
    bool available;
    float speedOffset;
    bool standStill;
    bool nonAdaptive;
} CruiseState;

typedef enum {
    gear_unknown,
    park,
    drive,
    neutral,
    reverse,
    s
} GearShifter;

typedef enum {
    leftBlinker,
    rightBlinker,
    accelCruise,
    decelCruise,
    cancel,
    lkas,
    mainCruise,
    setCruise,
    resumeCruise,
    gadAdjustCruise
} Type;

typedef enum{
    ipas_unknown = 0u,
    disabled = 1u,
    enabled = 3u,
    override = 5u
} IpasState;

typedef struct 
{
    float vEgo; //velocity used for acc
    float aEgo; //current acceleration use for acc
    float yawRate; //no use
    bool standStill; //used for acc(calculate from vEgo 1e-3)
    bool gasPressed; //safety guards
    float engineRpm; //metadata
    bool brakePressed; //safety guards
    bool parkingBrake; //safety guards
    float steeringAngleDeg; //steeringAngle
    int32_t steeringRateDeg; //no use
    int32_t steeringTorque; //used for ipas_steering calculation
    int32_t steeringTorqueEps; //no use
    uint8_t pcmAccStatus; //used for acc standstill state
    uint8_t pcmFollowDistance; //used for acc distance button
    uint8_t accType; //used for acc command
    float steeringPressed; //safety guard
    bool accFaulted; //it is for lockout state
    bool lockoutState; //lockout state pcm cruise
    GearShifter gear; //no use
    bool buttonPressed; //no use
    Type buttonType; //no use
    bool leftBlinker; //no use
    bool rightBlinker; //no use
    bool hazardLight; //no use
    IpasState ipasState; //ipas_steering used for
    CruiseState cruiseState; //
    TickType_t last_update_tick;
}CarState;


#endif /* INCLUDE_CAR_STATE_H_ */
