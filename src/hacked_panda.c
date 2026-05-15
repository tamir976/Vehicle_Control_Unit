#include "include/hacked_panda.h"
#include "include/monitor.h"
#include <string.h>


const uint8_t PRIUS_SPEED_MSG_DATA[8] = {0x30, 0x00, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
const uint8_t PRIUS_GEAR_MSG_DATA[8]  = {0x07, 0xA0, 0x1F, 0x00, 0x08, 0x00, 0x10, 0x00};
QueueHandle_t epsQueue4;
QueueHandle_t epsQueue5;

#define EPS_RX_DRAIN_BUDGET (8u)

void EpsForward4(Flexcan_Ip_MsgBuffType *tx4){
    Flexcan_Ip_DataInfoType txi;
    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    txi.fd_enable = false;
    txi.is_polling = false;
    txi.is_remote = false;
    uint8_t payload[8];
    uint8_t dlc = (tx4->dataLen > 8u) ? 8u : tx4->dataLen;
    uint32_t msg_id = tx4->msgId;
    (void)memcpy(payload, tx4->data, dlc);
    if(msg_id == PRIUS_GEAR_MSGID){
    	(void)memcpy(payload, PRIUS_GEAR_MSG_DATA, dlc);
    	dlc = 8u;
    }
    else if(msg_id == PRIUS_SPEED_MSGID){
    	(void)memcpy(payload, PRIUS_SPEED_MSG_DATA, dlc);
    	dlc  = 8u;
    }
    txi.data_length = dlc;
    int32_t mbx = GetTxMB(INST_FLEXCAN5);
    if(mbx >= 0){
    	(void)FlexCAN_Ip_Send(INST_FLEXCAN5, mbx, &txi, msg_id, payload);
    }
}

void EpsForward5(Flexcan_Ip_MsgBuffType *tx5){
    Flexcan_Ip_DataInfoType txi;
    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    txi.fd_enable = false;
    txi.is_polling = false;
    txi.is_remote = false;
    uint8_t payload[8];
    uint8_t dlc = (tx5->dataLen > 8u) ? 8u : tx5->dataLen;
    uint32_t msg_id = tx5->msgId;
    (void)memcpy(payload, tx5->data, dlc);
    txi.data_length = dlc;
    int32_t mbx = GetTxMB(INST_FLEXCAN4);
    if(mbx >= 0){
    	(void)FlexCAN_Ip_Send(INST_FLEXCAN4, mbx, &txi, msg_id, payload);
    }

}

void EPS4Task(void *pv){
    (void)pv;
    Flexcan_Ip_MsgBuffType tx4;
    for(;;){
        if(xQueueReceive(epsQueue4, &tx4, portMAX_DELAY) == pdTRUE){
            EpsForward4(&tx4);
            for(uint8_t drained = 1u; drained < EPS_RX_DRAIN_BUDGET; drained++){
                if(xQueueReceive(epsQueue4, &tx4, 0u) != pdTRUE){
                    break;
                }
                EpsForward4(&tx4);
            }
        }
    }
}

void EPS5Task(void *pv){
    (void)pv;
    Flexcan_Ip_MsgBuffType tx5;
    for(;;){
        if(xQueueReceive(epsQueue5, &tx5, portMAX_DELAY) == pdTRUE){
            EpsForward5(&tx5);
            for(uint8_t drained = 1u; drained < EPS_RX_DRAIN_BUDGET; drained++){
                if(xQueueReceive(epsQueue5, &tx5, 0u) != pdTRUE){
                    break;
                }
                EpsForward5(&tx5);
            }
        }
    }
}
