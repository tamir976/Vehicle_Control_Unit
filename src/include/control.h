#ifndef INCLUDE_CONTROL_H_
#define INCLUDE_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>
#include "car_control.h"
#include "car_state.h"

typedef struct{
    bool initialized;
    bool steer_angle_enabled;
    uint8_t ipas_reset_counter;
    float last_angle;
    uint32_t frame;
} IpasControllerState;

typedef struct {
    bool STEER_REQUEST;
    int16_t STEER_TORQUE_CMD;
    bool SET_ME_1;
    uint8_t COUNTER;
    uint8_t CHECKSUM;
    uint8_t data[8];
} SteerCommand;

typedef struct{
    float x;
    float dt;
    float alpha;
    bool initialized;
} MathFirstOrderFilter;

typedef struct{
    const float *k_p_bp;
    const float *k_p_v;
    uint8_t k_p_len;
    const float *k_i_bp;
    const float *k_i_v;
    uint8_t k_i_len;
    const float *k_d_bp;
    const float *k_d_v;
    uint8_t k_d_len;
    float k_f;
    float pos_limit;
    float neg_limit;
    float i_unwind_rate;
    float i_rate;
    float speed;
    float p;
    float i;
    float d;
    float f;
    float control;
} PidController;

typedef struct{
    bool initialized;
    bool last_standstill;
    bool standstill_req;
    bool permit_braking;
    uint8_t distance_button;
    float prev_accel;
    float accel;
    MathFirstOrderFilter pitch_filter;
    MathFirstOrderFilter aego_filter;
    PidController long_pid;
    uint32_t frame;
} ToyotaAccControllerState;

typedef struct{
    float ACCEL_CMD;
    uint8_t ALLOW_LONG_PRESS;
    bool ACC_MALFUNCTION;
    bool RADAR_DIRTY;
    bool DISTANCE;
    bool MINI_CAR;
    uint8_t ACC_TYPE;
    bool CANCEL_REQ;
    uint8_t ACC_CUT_IN;
    bool LEAD_VEHICLE_STOPPED;
    bool PERMIT_BRAKING;
    bool RELEASE_STANSTILL;
    uint8_t ITS_CONNECT_LEAD;
    float ACCEL_CMD_ALT;
    uint8_t CHECKSUM;
    uint8_t data[8];
} AccelCommand;

typedef struct {
    uint8_t COUNTER;
    uint8_t SET_ME_X00;
    int16_t FORCE;
    uint8_t SET_ME_X002;
    uint8_t BRAKE_STATUS;
    uint8_t STATE;
    bool SET_ME_X003;
    bool PRECOLLISION_ACTIVE;
    uint8_t CHECKSUM;
    uint8_t data[8];
} PcsCommand;

typedef struct {
    float DSS1GDRV;
    bool PCSALM;
    bool IBTRGR;
    uint8_t PBATRGR;
    bool PREFILL;
    bool AVSTRGR;
    uint8_t CHECKSUM;
    uint8_t data[8];
} PcsCommand_2;

typedef struct {
    bool GAS_RELEASED;
    bool CRUISE_ACTIVE;
    bool ACC_BRAKING;
    float ACCEL_NET;
    int16_t NEUTRAL_FORCE;
    uint8_t CRUISE_STATE;
    bool CANCEL_REQ;
    uint8_t CHECKSUM;
    uint8_t data[8];
} AccelCancelCommand;

typedef struct {
    uint8_t PCS_INDICATOR;
    bool FCW;
    uint8_t SET_ME_X20;
    bool PCS_DUST;
    bool PCS_TEMP;
    bool PCS_DUST2;
    bool PCS_TEMP2;
    uint8_t SET_ME_X10;
    bool PCS_OFF;
    uint8_t FRD_ADJ;
    uint8_t PCS_SENSITIVITY;
    uint8_t data[8];
} FcwCommand;

typedef struct {
    uint8_t BARRIERS;
    uint8_t RIGHT_LINE;
    uint8_t LEFT_LINE;
    uint8_t LKAS_STATUS;
    uint8_t LDA_ALERT;
    bool LDW_EXIST;
    bool TWO_BEEPS;
    bool ADJUSTING_CAMERA;
    bool LDA_UNAVAILABLE_QUIET;
    bool LDA_MALFUNCTION;
    bool LDA_UNAVAILABLE;
    uint8_t LDA_SENSITIVITY;
    uint8_t LDA_SA_TOGGLE;
    uint8_t LDA_MESSAGES;
    uint8_t LDA_ON_MESSAGE;
    bool REPEATED_BEAPS;
    bool LANE_SWAY_TOGGLE;
    uint8_t LANE_SWAY_SENSITIVITY;
    uint8_t TAKE_CONTROL;
    uint8_t LDA_FRONT_CAMERA_BLOCKED;
    uint8_t LANE_SWAY_BUZZER;
    uint8_t LANE_SWAY_FLD;
    uint8_t LANE_SWAY_WARNING;
    bool SET_ME_X01;
    uint8_t SET_ME_X02;
    uint8_t data[8];
} UICommand;

typedef struct{
    uint8_t STATE;
    float ANGLE;
    uint8_t SET_ME_X10;
    uint8_t SET_ME_X00;
    uint8_t DIRECTION_CMD;
    uint8_t SET_ME_X40;
    uint8_t SET_ME_X00_1;
    uint8_t CHECKSUM;   
    uint8_t data[8];
} SteeringIpasCommand;

extern SteerCommand gSteerCommand;
extern AccelCommand gAccelCommand;
extern PcsCommand gPcsCommand;
extern PcsCommand_2 gPcsCommand2;
extern AccelCancelCommand gAccelCancelCommand;
extern FcwCommand gFcwCommand;
extern UICommand gUICommand; 
extern SteeringIpasCommand gSteeringIpasCommand;
extern IpasControllerState gIpasControllerState;
void ProcessCommands(CarControl *cc, CarState *cs);
uint32_t toyota_acc_get_frame_counter(void);
float toyota_acc_get_last_accel(void);
void toyota_acc_reset_process_state(void);
bool toyota_acc_command_is_ready(void);

void ControlTask(void *pv);

#endif
