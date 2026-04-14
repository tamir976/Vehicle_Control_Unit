#ifndef INCLUDE_CONTROL_H_
#define INCLUDE_CONTROL_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "car_control.h"
#include "car_state.h"
#include <FlexCAN_Ip.h>

typedef struct {
    TickType_t last_run;
    TickType_t period;
} MsgTimer_t;

extern MsgTimer_t gMsgTimers[];

typedef struct {
    uint8_t STEER_REQUEST;
    int16_t STEER_TORQUE_CMD;
    uint8_t SET_ME_1;
    uint8_t COUNTER;
    uint8_t CHECKSUM;
    uint8_t data[8];
} SteerCommand;

typedef struct{
    float ACCEL_CMD;
    uint8_t ACCEL_TYPE;
    uint8_t ACC_TYPE;
    uint8_t DISTANCE;
    uint8_t MINI_CAR;
    uint8_t PERMIT_BRAKING;
    uint8_t ACC_MALFUNCTION;
    uint8_t RELEASE_STANSTILL;
    uint8_t CANCEL_REQ;
    uint8_t ALLOW_LONG_PRESS;
    uint8_t ACC_CUT_IN;
    uint8_t RADAR_DIRTY;
    uint8_t LEAD_VEHICLE_STOPPED;
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
    uint8_t SET_ME_X003;
    uint8_t PRECOLLISION_ACTIVE;
    uint8_t CHECKSUM;
    uint8_t data[8];
} PcsCommand;

typedef struct {
    int16_t DSS1GDRV;
    uint8_t PCSALM;
    uint8_t IBTRGR;
    uint8_t PBATRGR;
    uint8_t PREFILL;
    uint8_t AVSTRGR;
    uint8_t CHECKSUM;
    uint8_t data[8];
} PcsCommand_2;

typedef struct {
    uint8_t GAS_RELEASED;
    uint8_t CRUISE_ACTIVE;
    uint8_t ACC_BRAKING;
    float ACCEL_NET;
    int16_t NEUTRAL_FORCE;
    uint8_t CRUISE_STATE;
    uint8_t CANCEL_REQ;
    uint8_t CHECKSUM;
    uint8_t data[8];
} AccelCancelCommand;

typedef struct {
    uint8_t PCS_INDICATOR;
    uint8_t FCW;
    uint8_t SET_ME_X20;
    uint8_t PCS_DUST;
    uint8_t PCS_TEMP;
    uint8_t PCS_DUST2;
    uint8_t PCS_TEMP2;
    uint8_t SET_ME_X10;
    uint8_t PCS_OFF;
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
    uint8_t LDW_EXIST;
    uint8_t TWO_BEEPS;
    uint8_t ADJUSTING_CAMERA;
    uint8_t LDA_UNAVAILABLE_QUIET;
    uint8_t LDA_MALFUNCTION;
    uint8_t LDA_UNAVAILABLE;
    uint8_t LDA_SENSITIVITY;
    uint8_t LDA_SA_TOGGLE;
    uint8_t LDA_MESSAGES;
    uint8_t LDA_ON_MESSAGE;
    uint8_t REPEATED_BEAPS;
    uint8_t LANE_SWAY_TOGGLE;
    uint8_t LANE_SWAY_SENSITIVITY;
    uint8_t TAKE_CONTROL;
    uint8_t LDA_FRONT_CAMERA_BLOCKED;
    uint8_t LANE_SWAY_BUZZER;
    uint8_t LANE_SWAY_FLD;
    uint8_t LANE_SWAY_WARNING;
    uint8_t SET_ME_X01;
    uint8_t SET_ME_X02;
    uint8_t data[8];
} UICommand;

typedef struct{
    uint8_t STATE;
    int16_t ANGLE;
    uint8_t SET_ME_X10;
    uint8_t SET_ME_X00;
    uint8_t DIRECTION_CMD;
    uint8_t SET_ME_X40;
    uint8_t SET_ME_X00_1;
    uint8_t CHECKSUM;   
    uint8_t data[8];
} SteeringIpasCommand;

typedef struct{
    uint8_t STATE;
    int16_t ANGLE;
    uint8_t SET_ME_X10;
    uint8_t SET_ME_X00;
    uint8_t DIRECTION_CMD;
    uint8_t SET_ME_X40;
    uint8_t SET_ME_X00_1;
    uint8_t CHECKSUM;
    uint8_t data[8];
} SteeringIpasCommaCommand;

extern SteerCommand gSteerCommand;
extern AccelCommand gAccelCommand;
extern PcsCommand gPcsCommand;
extern PcsCommand_2 gPcsCommand2;
extern AccelCancelCommand gAccelCancelCommand;
extern FcwCommand gFcwCommand;
extern UICommand gUICommand; 
extern SteeringIpasCommand gSteeringIpasCommand;
extern SteeringIpasCommaCommand gSteeringIpasCommaCommand;

void create_steer_command(SteerCommand *cmd);
void create_accel_command(AccelCommand *cmd);
void create_pcs_commands(PcsCommand *cmd, PcsCommand_2 *cmd2);
void create_acc_cancel_command(AccelCancelCommand *cmd);
void create_fcw_command(FcwCommand *cmd);
void create_ui_command(UICommand *cmd);
void create_ipas_steer_command(SteeringIpasCommand *cmd, SteeringIpasCommaCommand *cmd2,boolean apgs_enabled);

void process_commands(CarState *cs, CarControl *cc);

void ControlManager(CarControl *cc, CarState *cs);
void ControlTask(void *pv);

#endif
