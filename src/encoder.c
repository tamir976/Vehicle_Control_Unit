#include "include/encoder.h"
#include "include/flexcan_conf.h"
#include "include/monitor.h"
#include "include/flexcan_conf.h"
#include "include/can_frame.h"
#include "queue.h"
#include "FreeRTOS.h"


MsgTimer_t gMsgTimers[] = {
    {0, pdMS_TO_TICKS(10u)}, // Steer command timer
    {0, pdMS_TO_TICKS(33u)}, // Accel command timer
    {0, pdMS_TO_TICKS(100u)}, // PCS command timer
    {0, pdMS_TO_TICKS(10u)}, // Acc cancel command timer when active
    {0, pdMS_TO_TICKS(1000u)}, // FCW command timer
    {0, pdMS_TO_TICKS(200u)}, // UI command timer
    {0, pdMS_TO_TICKS(10u)}, // IPAS steer command timer
};

uint8_t calculate_checksum(uint32_t id, const uint8_t *data, uint8_t dlc){
    uint32_t sum = dlc;
    uint32_t addr = id;

    while(addr){
        sum += addr & 0xFFu;
        addr >>= 8u;
    }

    if (dlc > 0u)
    {
        for (uint8_t i = 0u; i < (dlc - 1u); i++){
            sum += data[i];
        }
    }

    return (uint8_t)(sum & 0xFFu);
}

void encode_steer_command(uint32_t id, SteerCommand *cmd, uint8_t dlc){
    
    memset(cmd->data, 0, 8u);
    set_bits(cmd->data, 0, 1, cmd->STEER_REQUEST);
    set_bits(cmd->data, 6, 6, cmd->COUNTER);
    set_bits(cmd->data, 7, 1, cmd->SET_ME_1);
    set_bits(cmd->data, 15, 16, cmd->STEER_TORQUE_CMD);
    cmd->CHECKSUM = calculate_checksum(id, cmd->data, dlc);
    set_bits(cmd->data, 39, 8, cmd->CHECKSUM);
    cmd->COUNTER = (cmd->COUNTER + 1u) % 0x40u;
}

void encode_acc_command(uint32_t id, AccelCommand *cmd, uint8_t dlc){
    
    memset(cmd->data, 0, 8u);
    int32_t accel_raw = (int32_t)(cmd->ACCEL_CMD * 1000.0f);
    if (accel_raw > 32767)
    {
        accel_raw = 32767;
    }
    else if (accel_raw < -32768)
    {
        accel_raw = -32768;
    }
    set_bits(cmd->data, 7, 16, (uint16_t)((int16_t)accel_raw));
    set_bits(cmd->data, 17, 2, cmd->ALLOW_LONG_PRESS);
    set_bits(cmd->data, 18, 1, cmd->ACC_MALFUNCTION);
    set_bits(cmd->data, 19, 1, cmd->RADAR_DIRTY);
    set_bits(cmd->data, 20, 1, cmd->DISTANCE);
    set_bits(cmd->data, 21, 1, cmd->MINI_CAR);
    set_bits(cmd->data, 23, 2, cmd->ACC_TYPE);
    set_bits(cmd->data, 24, 1, cmd->CANCEL_REQ);
    set_bits(cmd->data, 25, 1, cmd->ACC_CUT_IN);
    set_bits(cmd->data, 29, 1, cmd->LEAD_VEHICLE_STOPPED);
    set_bits(cmd->data, 30, 1, cmd->PERMIT_BRAKING);
    set_bits(cmd->data, 31, 1, cmd->RELEASE_STANSTILL);
    set_bits(cmd->data, 39, 8, cmd->ITS_CONNECT_LEAD);
    int32_t accel_alt_raw = (int32_t)(cmd->ACCEL_CMD_ALT * 20.0f);
    if(accel_alt_raw > 127)
    {
        accel_alt_raw = 127;
    }
    else if (accel_alt_raw < -128)
    {
        accel_alt_raw = -128;
    }
    set_bits(cmd->data, 47, 8, (uint16_t)((int16_t)accel_alt_raw));
    cmd->CHECKSUM = calculate_checksum(id, cmd->data, dlc);
    set_bits(cmd->data, 63, 8, cmd->CHECKSUM);
}

void encode_pcs_command(uint32_t id, PcsCommand *cmd, uint8_t dlc){
    
    memset(cmd->data, 0, 8u);
    set_bits(cmd->data, 7, 8, cmd->COUNTER);
    set_bits(cmd->data, 15, 8, cmd->SET_ME_X00);
    set_bits(cmd->data, 23, 16, cmd->FORCE);
    set_bits(cmd->data, 33, 8, cmd->SET_ME_X002);
    set_bits(cmd->data, 39, 3, cmd->BRAKE_STATUS);
    set_bits(cmd->data, 36, 3, cmd->STATE);
    set_bits(cmd->data, 40, 1, cmd->SET_ME_X003);
    set_bits(cmd->data, 41, 1, cmd->PRECOLLISION_ACTIVE);
    cmd->CHECKSUM = calculate_checksum(id, cmd->data, dlc);
    set_bits(cmd->data, 55, 8, cmd->CHECKSUM);
    cmd->COUNTER = (uint8_t)(cmd->COUNTER + 1u);
}

void encode_pcs_command_2(uint32_t id, PcsCommand_2 *cmd, uint8_t dlc){
    
    memset(cmd->data, 0, 8u);
    set_bits(cmd->data, 7, 10, (uint16_t)cmd->DSS1GDRV);
    set_bits(cmd->data, 17, 1, cmd->PCSALM);
    set_bits(cmd->data, 27, 1, cmd->IBTRGR);
    set_bits(cmd->data, 30, 2, cmd->PBATRGR);
    set_bits(cmd->data, 33, 1, cmd->PREFILL);
    set_bits(cmd->data, 36, 1, cmd->AVSTRGR);
    cmd->CHECKSUM = calculate_checksum(id, cmd->data, dlc);
    set_bits(cmd->data, 63, 8, cmd->CHECKSUM);
}

void encode_acc_cancel_command(uint32_t id, AccelCancelCommand *cmd, uint8_t dlc){
    
    memset(cmd->data, 0, 8u);
    int32_t accel_net_raw = (int32_t)(cmd->ACCEL_NET * 1024.0f);

    if (accel_net_raw > 32767)
    {
        accel_net_raw = 32767;
    }
    else if (accel_net_raw < -32768)
    {
        accel_net_raw = -32768;
    }

    set_bits(cmd->data, 4, 1, cmd->GAS_RELEASED);
    set_bits(cmd->data, 5, 1, cmd->CRUISE_ACTIVE);
    set_bits(cmd->data, 12, 1, cmd->ACC_BRAKING);
    set_bits(cmd->data, 23, 16, (uint16_t)((int16_t)accel_net_raw));
    set_bits(cmd->data, 39, 16, cmd->NEUTRAL_FORCE);
    set_bits(cmd->data, 55, 4, cmd->CRUISE_STATE);
    set_bits_le(cmd->data, 49, 1, cmd->CANCEL_REQ);
    cmd->CHECKSUM = calculate_checksum(id, cmd->data, dlc);
    set_bits(cmd->data, 63, 8, cmd->CHECKSUM);
}

void encode_fcw_command(uint32_t id, FcwCommand *cmd, uint8_t dlc){
    
    (void)id;
    (void)dlc;
    memset(cmd->data, 0, 8u);
    set_bits(cmd->data, 7, 2, cmd->PCS_INDICATOR);
    set_bits(cmd->data, 4, 1, cmd->FCW);
    set_bits(cmd->data, 15, 8, cmd->SET_ME_X20);
    set_bits(cmd->data, 34, 1, cmd->PCS_DUST);
    set_bits(cmd->data, 35, 1, cmd->PCS_TEMP);
    set_bits(cmd->data, 41, 1, cmd->PCS_DUST2);
    set_bits(cmd->data, 42, 1, cmd->PCS_TEMP2);
    set_bits(cmd->data, 39, 8, cmd->SET_ME_X10);
    set_bits(cmd->data, 40, 1, cmd->PCS_OFF);
    set_bits(cmd->data, 53, 3, cmd->FRD_ADJ);
    set_bits(cmd->data, 55, 8, cmd->PCS_SENSITIVITY);
}

void encode_ui_command(uint32_t id, UICommand *cmd, uint8_t dlc){
    
    (void)id;
    (void)dlc;
    memset(cmd->data, 0, 8u);
    set_bits(cmd->data, 1, 2, cmd->BARRIERS);
    set_bits(cmd->data, 3, 2, cmd->RIGHT_LINE);
    set_bits(cmd->data, 5, 2, cmd->LEFT_LINE);
    set_bits(cmd->data, 7, 2, cmd->LKAS_STATUS);
    set_bits(cmd->data, 9, 2, cmd->LDA_ALERT);
    set_bits(cmd->data, 10, 1, cmd->LDW_EXIST);
    set_bits(cmd->data, 12, 1, cmd->TWO_BEEPS);
    set_bits(cmd->data, 13, 1, cmd->ADJUSTING_CAMERA);
    set_bits(cmd->data, 14, 1, cmd->LDA_UNAVAILABLE_QUIET);
    set_bits(cmd->data, 15, 1, cmd->LDA_MALFUNCTION);
    set_bits(cmd->data, 16, 1, cmd->LDA_UNAVAILABLE);
    set_bits(cmd->data, 18, 2, cmd->LDA_SENSITIVITY);
    set_bits(cmd->data, 20, 2, cmd->LDA_SA_TOGGLE);
    set_bits(cmd->data, 23, 3, cmd->LDA_MESSAGES);
    set_bits(cmd->data, 31, 2, cmd->LDA_ON_MESSAGE);
    set_bits(cmd->data, 32, 1, cmd->REPEATED_BEAPS);
    set_bits(cmd->data, 43, 1, cmd->LANE_SWAY_TOGGLE);
    set_bits(cmd->data, 45, 2, cmd->LANE_SWAY_SENSITIVITY);
    set_bits(cmd->data, 46, 1, cmd->TAKE_CONTROL);
    set_bits(cmd->data, 47, 1, cmd->LDA_FRONT_CAMERA_BLOCKED);
    set_bits(cmd->data, 50, 2, cmd->LANE_SWAY_BUZZER);
    set_bits(cmd->data, 53, 3, cmd->LANE_SWAY_FLD);
    set_bits(cmd->data, 55, 2, cmd->LANE_SWAY_WARNING);
    set_bits(cmd->data, 42, 1, cmd->SET_ME_X01);
    set_bits(cmd->data, 63, 8, cmd->SET_ME_X02);       
}

void encode_ipas_steer_command(uint32_t id, SteeringIpasCommand *cmd, uint8_t dlc){
    
    memset(cmd->data, 0, 8u);
    set_bits(cmd->data, 7, 4, cmd->STATE);
    set_bits(cmd->data, 3, 12, cmd->ANGLE);
    set_bits(cmd->data, 23, 8, cmd->SET_ME_X10);
    set_bits(cmd->data, 31, 8, cmd->SET_ME_X00);
    set_bits(cmd->data, 38, 2, cmd->DIRECTION_CMD);
    set_bits(cmd->data, 47, 8, cmd->SET_ME_X40);
    set_bits(cmd->data, 55, 8, cmd->SET_ME_X00_1);   
    cmd->CHECKSUM = calculate_checksum(id, cmd->data, dlc);
    set_bits(cmd->data, 63, 8, cmd->CHECKSUM);
}

void encode_ipas_steer_comma_command(uint32_t id, SteeringIpasCommaCommand *cmd, uint8_t dlc){
    memset(cmd->data, 0, 8u);
    set_bits(cmd->data, 7, 4, cmd->STATE);
    set_bits(cmd->data, 3, 12, cmd->ANGLE);
    set_bits(cmd->data, 23, 8, cmd->SET_ME_X10);
    set_bits(cmd->data, 31, 8, cmd->SET_ME_X00);
    set_bits(cmd->data, 38, 2, cmd->DIRECTION_CMD);
    set_bits(cmd->data, 47, 8, cmd->SET_ME_X40);
    set_bits(cmd->data, 55, 8, cmd->SET_ME_X00_1);   
    cmd->CHECKSUM = calculate_checksum(id, cmd->data, dlc);
    set_bits(cmd->data, 63, 8, cmd->CHECKSUM);   
}

void SendFrame(uint8_t instance, uint32_t msg_id, uint8_t *data, uint8_t dlc){
    Flexcan_Ip_DataInfoType txi = {0};
    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    txi.fd_enable = false;
    txi.is_polling = false;
    txi.is_remote = false;
    txi.data_length = dlc;
    int32_t mbx = GetTxMB(instance);
    if(mbx >= 0){
        (void)FlexCAN_Ip_Send(instance, mbx, &txi, msg_id, data);
    }
}

void create_steer_command(SteerCommand *cmd){
    uint8_t bus = 0u;
    uint32_t msg_id = ID_STEERING_LKA;
    uint8_t dlc = 5u;
    uint8_t data[8];
    taskENTER_CRITICAL();
    encode_steer_command(msg_id, cmd, dlc);
    memcpy(data, cmd->data, dlc);
    taskEXIT_CRITICAL();
    SendFrame(bus, msg_id, data, dlc);
}
void create_acc_command(AccelCommand *cmd){
    uint8_t bus = 0u;
    uint32_t msg_id = ID_ACC_CONTROL;
    uint8_t dlc = 8u;
    uint8_t data[8];
    taskENTER_CRITICAL();
    encode_acc_command(msg_id, cmd, dlc);
    memcpy(data, cmd->data, dlc);
    taskEXIT_CRITICAL();
    SendFrame(bus, msg_id, data, dlc);
}
void create_pcs_commands(PcsCommand *cmd, PcsCommand_2 *cmd2){
    uint8_t bus = 0u;
    uint32_t msg_id = ID_PRE_COLLISION;
    uint32_t msg_id_2 = ID_PRE_COLLISION_2;
    uint8_t dlc = 7u;
    uint8_t dlc2 = 8u;
    uint8_t data1[8];
    uint8_t data2[8];
    taskENTER_CRITICAL();
    encode_pcs_command(msg_id, cmd, dlc);
    encode_pcs_command_2(msg_id_2, cmd2, dlc2);
    memcpy(data1, cmd->data, dlc);
    memcpy(data2, cmd2->data, dlc2);
    taskEXIT_CRITICAL();
    SendFrame(bus, msg_id, data1, dlc);
    SendFrame(bus, msg_id_2, data2, dlc2);
}

void create_acc_cancel_command(AccelCancelCommand *cmd){
    uint8_t bus = 0;
    uint32_t msg_id = ID_PCM_CRUISE;
    uint8_t dlc = 8u;
    uint8_t data[8];
    taskENTER_CRITICAL();
    encode_acc_cancel_command(msg_id, cmd, dlc);
    memcpy(data, cmd->data, dlc);
    taskEXIT_CRITICAL();
    SendFrame(bus, msg_id, data, dlc);
}
void create_fcw_command(FcwCommand *cmd){
    uint8_t bus = 0;
    uint32_t msg_id = ID_PCS_HUD;
    uint8_t dlc = 8u;
    uint8_t data[8];
    taskENTER_CRITICAL();
    encode_fcw_command(msg_id, cmd, dlc);
    memcpy(data, cmd->data, dlc);
    taskEXIT_CRITICAL();
    SendFrame(bus, msg_id, data, dlc);
}
void create_ui_command(UICommand *cmd){
    uint8_t bus = 0;
    uint32_t msg_id = ID_LKAS_HUD;
    uint8_t dlc = 8u;
    uint8_t data[8];
    taskENTER_CRITICAL();
    encode_ui_command(msg_id, cmd, dlc);
    memcpy(data, cmd->data, dlc);
    taskEXIT_CRITICAL();
    SendFrame(bus, msg_id, data, dlc);
}
void create_ipas_steer_command(SteeringIpasCommand *cmd, SteeringIpasCommaCommand *cmd2, bool apgs_enabled){
    uint8_t bus = 0;
    uint32_t msg_id = ID_STEERING_IPAS;
    uint8_t dlc = 8u;
    uint8_t data[8];
    (void)cmd2;
    if(apgs_enabled){
        taskENTER_CRITICAL();
        encode_ipas_steer_command(msg_id, cmd, dlc);
        memcpy(data, cmd->data, dlc);
        taskEXIT_CRITICAL();
        SendFrame(bus, msg_id, data, dlc);
    }
}

void Encoder(void){
    TickType_t now = xTaskGetTickCount();
    if((now - gMsgTimers[0].last_run) >= gMsgTimers[0].period){
        gMsgTimers[0].last_run = now;
        create_steer_command(&gSteerCommand);
    }
    if ((now - gMsgTimers[1].last_run) >= gMsgTimers[1].period){
        gMsgTimers[1].last_run = now;
        create_acc_command(&gAccelCommand);
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
        create_ipas_steer_command(&gSteeringIpasCommand, &gSteeringIpasCommaCommand, true);
    }
}

void EncoderTask(void *pv){
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;){
        Encoder();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}
