#include "include/flexcan_conf.h"

Flexcan_Ip_MsgBuffType can0_rx[CAN0_RX_SIZE];
Flexcan_Ip_MsgBuffType can1_rx[CAN1_RX_SIZE];
Flexcan_Ip_MsgBuffType can2_rx[CAN2_RX_SIZE];
Flexcan_Ip_MsgBuffType can3_rx[CAN3_RX_SIZE];
Flexcan_Ip_MsgBuffType can4_rx[CAN4_RX_SIZE];
Flexcan_Ip_MsgBuffType can5_rx[CAN5_RX_SIZE];

boolean can0_tx[CAN0_TX_SIZE];
boolean can1_tx[CAN1_TX_SIZE];
boolean can2_tx[CAN2_TX_SIZE];
boolean can3_tx[CAN3_TX_SIZE];
boolean can4_tx[CAN4_TX_SIZE];
boolean can5_tx[CAN5_TX_SIZE];


uint8_t GetTxMB(uint8_t instance){
    switch (instance)
    {
    case INST_FLEXCAN0:
        for(uint8_t i = 0; i < CAN0_TX_SIZE; i++){
            if (!can0_tx[i]){
                can0_tx[i] = true;
                return CAN0_RX_SIZE + i;
            }
        }
        return -1;
    case INST_FLEXCAN1:
        for(uint8_t i = 0; i < CAN1_TX_SIZE; i++){
            if (!can1_tx[i]){
                can1_tx[i] = true;
                return CAN1_RX_SIZE + i;
            }
        }
        return -1;
    case INST_FLEXCAN2:
        for(uint8_t i = 0; i < CAN2_TX_SIZE; i++){
            if (!can2_tx[i]){
                can2_tx[i] = true;
                return CAN2_RX_SIZE + i;
            }
        }
        return -1;
    case INST_FLEXCAN3:
        for(uint8_t i = 0; i < CAN3_TX_SIZE; i++){
            if (!can3_tx[i]){
                can3_tx[i] = true;
                return CAN3_RX_SIZE + i;
            }
        }
        return -1;
    case INST_FLEXCAN4:
        for(uint8_t i = 0; i < CAN4_TX_SIZE; i++){
            if (!can4_tx[i]){
                can4_tx[i] = true;
                return CAN4_RX_SIZE + i;
            }
        }
        return -1;
    case INST_FLEXCAN5:
        for(uint8_t i = 0; i < CAN5_TX_SIZE; i++){
            if (!can5_tx[i]){
                can5_tx[i] = true;
                return CAN5_RX_SIZE + i;
            }
        }
        return -1;
    
    default:
        return -1;
    }
}

void CAN_SendFrame(uint8_t instance, const Flexcan_Ip_DataInfoType *txInfo, uint32_t msgId, const uint8_t *data)
{
    uint8_t txNum = GetTxMB(instance);
    FlexCAN_Ip_Send(instance, txNum, txInfo, msgId, data);
}

void Can_InitOne(uint8_t inst, Flexcan_Ip_StateType *state, const Flexcan_Ip_ConfigType *config, Flexcan_Ip_MsgBuffType *rxMb)
{
    Flexcan_Ip_DataInfoType rxStd;
    rxStd.msg_id_type = FLEXCAN_MSG_ID_STD;
    rxStd.data_length = 8u;
    rxStd.is_polling  = FALSE;
    rxStd.is_remote   = FALSE;
    rxStd.fd_enable   = FALSE;

    FlexCAN_Ip_Init(inst, state, config);
    FlexCAN_Ip_EnterFreezeMode(inst);
    (void)FlexCAN_Ip_SetRxMaskType(inst, FLEXCAN_RX_MASK_INDIVIDUAL);
    switch (inst)
    {
    case INST_FLEXCAN0:
        for(uint8_t i = 0u; i < CAN0_RX_SIZE; i++){
            (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
            FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000u);
            FlexCAN_Ip_Receive(inst, i, &rxMb[i], FALSE);
        }
        break;
    case INST_FLEXCAN1:
        for(uint8_t i = 0u; i < CAN1_RX_SIZE; i++){
            (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
            FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000u);
            FlexCAN_Ip_Receive(inst, i, &rxMb[i], FALSE);
        }
        break;
    case INST_FLEXCAN2:
        for(uint8_t i = 0u; i < CAN2_RX_SIZE; i++){
            (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
            FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000u);
            FlexCAN_Ip_Receive(inst, i, &rxMb[i], FALSE);
        }
        break;
    case INST_FLEXCAN3:
        for(uint8_t i = 0u; i < CAN3_RX_SIZE; i++){
            (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
            FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000u);
            FlexCAN_Ip_Receive(inst, i, &rxMb[i], FALSE);
        }
        break;
    case INST_FLEXCAN4:
        for(uint8_t i = 0u; i < CAN4_RX_SIZE; i++){
            (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
            FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000u);
            FlexCAN_Ip_Receive(inst, i, &rxMb[i], FALSE);
        }
        break;
    case INST_FLEXCAN5:
        for(uint8_t i = 0u; i < CAN5_RX_SIZE; i++){
            (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
            FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000u);
            FlexCAN_Ip_Receive(inst, i, &rxMb[i], FALSE);
        }
        break;
    default:
        break;
    }
    FlexCAN_Ip_ExitFreezeMode(inst);
    (void)FlexCAN_Ip_SetStartMode(inst);
    WaitStartMode(inst);
    FlexCAN_Ip_EnableInterrupts_Privileged(inst);
}
