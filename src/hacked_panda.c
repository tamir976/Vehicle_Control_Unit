#include "include/hacked_panda.h"
#include "include/monitor.h"


const uint8_t PRIUS_SPEED_MSG_DATA[8] = {0x30, 0x00, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
const uint8_t PRIUS_GEAR_MSG_DATA[8]  = {0x07, 0xA0, 0x1F, 0x00, 0x08, 0x00, 0x10, 0x00};

void ForwardFrame(uint8_t fromInst, uint8_t toInst, const Flexcan_Ip_MsgBuffType *msg){
    Flexcan_Ip_DataInfoType txi;
    uint8_t payload[8];
    uint8_t dlc = msg->dataLen;
    uint32_t id = msg->msgId;
    (void)memcpy(payload, msg->data, dlc);
    if((fromInst == INST_FLEXCAN4) && (toInst == INST_FLEXCAN5)){
    	dlc = 8;
    	if(id == PRIUS_GEAR_MSGID){
    		(void)memcpy(payload, PRIUS_GEAR_MSG_DATA, dlc);
    	}else if(id == PRIUS_SPEED_MSGID){
    		(void)memcpy(payload, PRIUS_SPEED_MSG_DATA, dlc);
    	}
    }
    txi.data_length = dlc;
    txi.fd_enable = FALSE;
    txi.is_polling = FALSE;
    txi.is_remote = FALSE;
    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    CAN_SendFrame(toInst, &txi, id, payload);
}

void EPSTask(void *pv){
    (void)pv;
    epsMsg msg;
    for(;;){
        if(xQueueReceive(epsQueue, &msg, portMAX_DELAY) == pdPASS){
            if(msg.instance == INST_FLEXCAN4){
                ForwardFrame(INST_FLEXCAN4, INST_FLEXCAN5, &msg.frame);
            }
            else{
                ForwardFrame(INST_FLEXCAN5, INST_FLEXCAN4, &msg.frame);
            }
        }
    }
}