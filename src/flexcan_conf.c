#include "include/flexcan_conf.h"
#include "OsIf.h"
#include "include/monitor.h"
#include "task.h"
#include <string.h>

Flexcan_Ip_MsgBuffType can0_rx[CAN0_RX_SIZE];
Flexcan_Ip_MsgBuffType can1_rx[CAN1_RX_SIZE];
Flexcan_Ip_MsgBuffType can2_rx[CAN2_RX_SIZE];
Flexcan_Ip_MsgBuffType can3_rx[CAN3_RX_SIZE];
Flexcan_Ip_MsgBuffType can4_rx[CAN4_RX_SIZE];
Flexcan_Ip_MsgBuffType can5_rx[CAN5_RX_SIZE];

QueueHandle_t g_canCarRxQueue;
QueueHandle_t g_canPcRxQueue;

CanTxPool_t g_canTxPool[6] = {
    [INST_FLEXCAN0] = { .freeMask = 0u, .rxSize = CAN0_RX_SIZE, .txSize = CAN0_TX_SIZE},
    [INST_FLEXCAN1] = { .freeMask = 0u, .rxSize = CAN1_RX_SIZE, .txSize = CAN1_TX_SIZE},
    [INST_FLEXCAN2] = { .freeMask = 0u, .rxSize = CAN2_RX_SIZE, .txSize = CAN2_TX_SIZE},
    [INST_FLEXCAN3] = { .freeMask = 0u, .rxSize = CAN3_RX_SIZE, .txSize = CAN3_TX_SIZE},
    [INST_FLEXCAN4] = { .freeMask = 0u, .rxSize = CAN4_RX_SIZE, .txSize = CAN4_TX_SIZE},
    [INST_FLEXCAN5] = { .freeMask = 0u, .rxSize = CAN5_RX_SIZE, .txSize = CAN5_TX_SIZE}
};


void Init_TxPool(void){
    for(uint8_t i = 0; i < 6; i++){
        uint8_t txSz = g_canTxPool[i].txSize;
        g_canTxPool[i].freeMask = (txSz == 32u) ? 0xFFFFFFFFu : ((1UL << txSz) - 1UL);
    }
}

int32_t GetTxMB(uint8_t instance){
    CanTxPool_t *pool = &g_canTxPool[instance];
    int32_t result = -1;
    taskENTER_CRITICAL();{
        uint32_t mask = pool->freeMask;
        if(mask != 0){
            uint32_t bit = mask & (-mask);
            uint8_t txIdx = (uint8_t)__builtin_ctz(bit);
            pool->freeMask &= ~bit;
            result = (int32_t)(pool->rxSize + txIdx);
        }
    }
    taskEXIT_CRITICAL();
    return result;
}

void ReleaseTxMb(uint8_t instance, uint8_t mbIdx){
    CanTxPool_t *pool = &g_canTxPool[instance];
    uint8_t txIdx = mbIdx - pool->rxSize;

    UBaseType_t savedMask = taskENTER_CRITICAL_FROM_ISR();
    {
        pool->freeMask |= (1UL <<txIdx);
    }
    taskEXIT_CRITICAL_FROM_ISR(savedMask);
}

void Can_InitOne(uint8_t inst, Flexcan_Ip_StateType *state, const Flexcan_Ip_ConfigType *config, Flexcan_Ip_MsgBuffType *rxMb)
{
    const Flexcan_Ip_DataInfoType rxStd = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8u,
        .is_polling  = FALSE,
        .is_remote   = FALSE,
        .fd_enable   = FALSE
    };
    

    FlexCAN_Ip_Init(inst, state, config);
    FlexCAN_Ip_EnterFreezeMode(inst);
    (void)FlexCAN_Ip_SetRxMaskType(inst, FLEXCAN_RX_MASK_INDIVIDUAL);
    uint8_t rxSize = g_canTxPool[inst].rxSize;
    for(uint8_t i = 0u; i<rxSize; i++){
        (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
        (void)FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000);
        (void)FlexCAN_Ip_Receive(inst, i, &rxMb[i], false);
    }
    FlexCAN_Ip_ExitFreezeMode(inst);
    (void)FlexCAN_Ip_SetStartMode(inst);
    if(WaitStartMode(inst)){}
    FlexCAN_Ip_EnableInterrupts_Privileged(inst);
}

bool WaitStartMode(uint8_t instance){
	uint32_t start = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
	const uint32_t timeoutTicks = OsIf_MicrosToTicks(100000u, OSIF_COUNTER_SYSTEM);

    while (FlexCAN_Ip_GetStartMode(instance) == FALSE)
    {
        if(OsIf_GetElapsed(&start, OSIF_COUNTER_SYSTEM) >= timeoutTicks){
            return false;
        }
    }
    return true;
}
