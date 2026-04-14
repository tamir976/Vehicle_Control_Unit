#include "include/control.h"
#include "include/encoder.h"
#include "include/monitor.h"
#include "include/decoder.h"

SteerCommand gSteerCommand = {0};
AccelCommand gAccelCommand = {0};
PcsCommand gPcsCommand = {0};
PcsCommand_2 gPcsCommand2 = {0};
AccelCancelCommand gAccelCancelCommand = {0};
FcwCommand gFcwCommand = {0};
UICommand gUICommand = {0};
SteeringIpasCommand gSteeringIpasCommand = {0};
SteeringIpasCommaCommand gSteeringIpasCommaCommand = {0};
static boolean apgs_enabled;

MsgTimer_t gMsgTimers[] = {
    {0, pdMS_TO_TICKS(10u)}, // Steer command timer
    {0, pdMS_TO_TICKS(100u)}, // Accel command timer
    {0, pdMS_TO_TICKS(100u)}, // PCS command timer
    {0, pdMS_TO_TICKS(100u)}, // Acc cancel command timer
    {0, pdMS_TO_TICKS(10u)}, // FCW command timer
    {0, pdMS_TO_TICKS(100u)}, // UI command timer
    {0, pdMS_TO_TICKS(100u)}, // IPAS steer command timer
};

void create_steer_command(SteerCommand *cmd){
    uint8_t bus = 0u;
    uint32_t msg_id = ID_STEERING_LKA;
    uint8_t dlc = 5u;
    encode_steer_command(msg_id, cmd, dlc);
    CAN_Send(msg_id, bus, cmd->data, dlc);
}
void create_accel_command(AccelCommand *cmd){
    uint8_t bus = 0u;
    uint32_t msg_id = ID_ACC_CONTROL;
    uint8_t dlc = 8u;
    encode_accel_command(msg_id, cmd, dlc);
    // CAN_Send(msg_id, bus, cmd->data, dlc);
}
void create_pcs_commands(PcsCommand *cmd, PcsCommand_2 *cmd2){
    uint8_t bus = 0u;
    uint32_t msg_id = ID_PRE_COLLISION;
    uint32_t msg_id_2 = ID_PRE_COLLISION_2;
    uint8_t dlc = 7u;
    uint8_t dlc2 = 8u;
    encode_pcs_command(msg_id, cmd, dlc);
    encode_pcs_command_2(msg_id_2, cmd2, dlc2);
    // CAN_Send(msg_id, bus, cmd->data, dlc);
    // CAN_Send(msg_id_2, bus, cmd2->data, dlc2);
    
}
void create_acc_cancel_command(AccelCancelCommand *cmd){
    uint8_t bus = 0;
    uint32_t msg_id = ID_PCM_CRUISE;
    uint8_t dlc = 8u;
    encode_acc_cancel_command(msg_id, cmd, dlc);
    // CAN_Send(msg_id, bus, cmd->data, dlc);
}
void create_fcw_command(FcwCommand *cmd){
    uint8_t bus = 0;
    uint32_t msg_id = ID_PCS_HUD;
    uint8_t dlc = 8u;
    encode_fcw_command(msg_id, cmd, dlc);
    CAN_Send(msg_id, bus, cmd->data, dlc);
}
void create_ui_command(UICommand *cmd){
    uint8_t bus = 0;
    uint32_t msg_id = ID_LKAS_HUD;
    uint8_t dlc = 8u;
    encode_ui_command(msg_id, cmd, dlc);
    // CAN_Send(msg_id, bus, cmd->data, dlc);
}
void create_ipas_steer_command(SteeringIpasCommand *cmd, SteeringIpasCommaCommand *cmd2, boolean apgs_enabled){
    uint8_t bus = 0;
    uint32_t msg_id = ID_STEERING_IPAS;
    uint32_t msg_id_2 = ID_STEERING_IPAS_COMMA;
    uint8_t dlc = 8u;
    if(apgs_enabled){
        encode_ipas_steer_command(msg_id, cmd, dlc);
        // CAN_Send(msg_id, bus, cmd->data, dlc);
    }else{
        encode_ipas_steer_comma_command(msg_id_2, cmd2, dlc);
        // CAN_Send(msg_id_2, bus, cmd2->data, dlc);
    }
}

void process_commands(CarState *cs, CarControl *cc){
//    taskENTER_CRITICAL();
    gSteerCommand.STEER_REQUEST = 0;
    gSteerCommand.SET_ME_1 = 1;
    gSteerCommand.STEER_TORQUE_CMD = 0;
    gFcwCommand.FCW = 0;
    gFcwCommand.PCS_INDICATOR = 0;
    gFcwCommand.SET_ME_X20 = 0x20;
    gFcwCommand.PCS_DUST = 0;
    gFcwCommand.PCS_TEMP = 0;
    gFcwCommand.PCS_DUST2 = 0;
    gFcwCommand.PCS_TEMP2 = 0;
    gFcwCommand.SET_ME_X10 = 0x10;
    gFcwCommand.PCS_OFF = 1;
    gFcwCommand.FRD_ADJ = 0;
    gFcwCommand.PCS_SENSITIVITY = 0;
    apgs_enabled = FALSE;    
//    taskEXIT_CRITICAL();
}

void ControlManager(CarControl *cc, CarState *cs){
    TickType_t now = xTaskGetTickCount();
    process_commands(cs, cc);
    if((now - gMsgTimers[0].last_run) >= gMsgTimers[0].period){
        gMsgTimers[0].last_run = now;
        create_steer_command(&gSteerCommand);
    }
    if ((now - gMsgTimers[1].last_run) >= gMsgTimers[1].period){
        gMsgTimers[1].last_run = now;
        create_accel_command(&gAccelCommand);
    }
    if ((now - gMsgTimers[2].last_run) >= gMsgTimers[2].period){
        gMsgTimers[2].last_run = now;
        create_pcs_commands(&gPcsCommand, &gPcsCommand2);
    }
    if ((now - gMsgTimers[3].last_run) >= gMsgTimers[3].period){
        gMsgTimers[3].last_run = now;
        create_acc_cancel_command(&gAccelCancelCommand);
    }
    if ((now - gMsgTimers[4].last_run) >= gMsgTimers[4].period){
        gMsgTimers[4].last_run = now;
        create_fcw_command(&gFcwCommand);
    }
    if ((now - gMsgTimers[5].last_run) >= gMsgTimers[5].period){
        gMsgTimers[5].last_run = now;
        create_ui_command(&gUICommand);
    }
    if ((now - gMsgTimers[6].last_run) >= gMsgTimers[6].period){
        gMsgTimers[6].last_run = now;
        create_ipas_steer_command(&gSteeringIpasCommand, &gSteeringIpasCommaCommand, apgs_enabled);
    }
}


void ControlTask(void *pv){
    (void)pv;
    CarState stateSnap;
    CarControl controlSnap;
    TickType_t lastWakeTime = xTaskGetTickCount();
    for(;;){
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10u));
        CarState_GetSnapshot(&stateSnap);
        CarControl_GetSnapshot(&controlSnap);
        ControlManager(&controlSnap, &stateSnap);
    }
}
