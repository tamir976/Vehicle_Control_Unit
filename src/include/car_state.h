
#ifndef INCLUDE_CAR_STATE_H_
#define INCLUDE_CAR_STATE_H_

#include <stdint.h>
#include "can_descriptors.h"

typedef struct{
    uint8_t used;
    uint32_t id;
    uint16_t msg_index;
} MsgLookupEntry;

typedef struct {
    float vehicle_speed;
    float wheel_speed_fl;
    float wheel_speed_fr;
    float wheel_speed_rl;
    float wheel_speed_rr;
    float steer_angle;
    float steer_torque_driver;
    float steer_torque_eps;
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
    uint8_t traction_control_active;
    TickType_t last_update_tick;
} CarState;

typedef struct{
    uint8_t data[8];
    uint8_t dlc;
    uint32_t id;
} LocalFrame;

static MsgLookupEntry gMsgMapBus0[MESSAGE_COUNT];
static MsgLookupEntry gMsgMapBus1[MESSAGE_COUNT];
static CarState gCarState;

#endif /* INCLUDE_CAR_STATE_H_ */
