#include "include/hacked_panda.h"

void ForwardFrame(uint8_t fromInst, uint8_t toInst, const Flexcan_Ip_MsgBuffType *msg){
    Flexcan_Ip_DataInfoType txi;
    uint8_t payload[8];
    Flexcan_Ip_StatusType st;
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
    st = FlexCAN_Ip_Send(toInst, GetTxMB(toInst), &txi, id, payload);
    if(st != FLEXCAN_STATUS_SUCCESS){
    	gStats.canTxFails++;
    }
}
