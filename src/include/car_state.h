
#ifndef INCLUDE_CAR_STATE_H_
#define INCLUDE_CAR_STATE_H_

#include <stdint.h>
#include <FreeRTOS.h>

#define ID_KINEMATICS (0x24u)
#define ID_STEER_ANGLE_SENSOR (0x25u)
#define ID_WHEEL_SPEEDS (0xAAu)
#define ID_SPEED (0xB4u)
#define ID_GEAR_HYBRID (0x127u)
#define ID_ENGINE_RPM (0x1C4u)
#define ID_PCM_CRUISE_2 (0x1D3u)
#define ID_BRAKE_2 (0x230u)
#define ID_GAS_PEDAL_HYBRID (0x245u)
#define ID_STEER_TORQUE (0x260u)
#define ID_STEERING_IPAS (0x266u)
#define ID_VSC1S07 (0x320u)
#define ID_ACC_CONTROL (0x343u)
#define ID_PCM_CRUISE_SM (0x399u)
#define ID_GEAR_PACKET (0x3BCu)
#define ID_PCM_CRUISE_ALT (0x3F1u)
#define ID_BLINKERS_STATE (0x614u)
#define ID_BODY_CONTROL_STATE (0x620u)
#define ID_EPS_STATUS (0x262u)

typedef struct {
	uint8_t enabled;
} CruiseState;

typedef struct{
    float vEgo;
    float aEgo;
    float steeringAngleDeg;
    float steeringAngleOffsetDeg;
    float steeringRateDeg;
    float steeringTorque;
    float steeringTorqueEps;
    uint8_t standstill;
    CruiseState cruiseState;
} CarStateOut;

typedef struct {
    CarStateOut out;
    uint8_t pcm_acc_status;
    uint8_t pcm_follow_distance;
    uint8_t acc_type;
    float gvc;
    uint32_t lkas_hud;
    float wheel_speed_fl;
    float wheel_speed_fr;
    float wheel_speed_rl;
    float wheel_speed_rr;
    float yaw_rate;
    float accel_x;
    float accel_y;
    float engine_rpm;
    float gas_pedal;
    uint8_t gear;
    uint8_t brake_pressed;
    uint8_t left_blinker;
    uint8_t right_blinker;
    uint8_t door_open_fl;
    uint8_t door_open_fr;
    uint8_t door_open_rl;
    uint8_t door_open_rr;
    uint8_t eps_active;
    uint8_t abs_active;
    uint8_t ipas_active;
    uint8_t traction_control_active;
    TickType_t last_update_tick;
} CarState;

CarState gCarState;

#endif /* INCLUDE_CAR_STATE_H_ */
