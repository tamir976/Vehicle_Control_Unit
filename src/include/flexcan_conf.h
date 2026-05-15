#ifndef INCLUDE_FLEXCAN_CONF_H_
#define INCLUDE_FLEXCAN_CONF_H_
#include "FlexCAN_Ip.h"
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

#define INST_FLEXCAN0 (0u)
#define INST_FLEXCAN1 (1u)
#define INST_FLEXCAN2 (2u)
#define INST_FLEXCAN3 (3u)
#define INST_FLEXCAN4 (4u)
#define INST_FLEXCAN5 (5u)

#define CAN0_RX_SIZE (32u)
#define CAN1_RX_SIZE (32u)
#define CAN2_RX_SIZE (8u)
#define CAN3_RX_SIZE (20u)
#define CAN4_RX_SIZE (8u)
#define CAN5_RX_SIZE (8u)

#define CAN0_TX_SIZE (32u)
#define CAN1_TX_SIZE (32u)
#define CAN2_TX_SIZE (8u)
#define CAN3_TX_SIZE (12u)
#define CAN4_TX_SIZE (8u)
#define CAN5_TX_SIZE (8u)

typedef struct{
    volatile uint32_t freeMask;
    uint8_t rxSize;
    uint8_t txSize;
} CanTxPool_t;

#define TJA1153_START_ID (0x555u)
#define TJA1153_CONFIG_ID (0x18DA00F1u)

extern QueueHandle_t g_canCarRxQueue;
extern QueueHandle_t g_canPcRxQueue;
extern CanTxPool_t g_canTxPool[6];
extern Flexcan_Ip_MsgBuffType can0_rx[CAN0_RX_SIZE];
extern Flexcan_Ip_MsgBuffType can1_rx[CAN1_RX_SIZE];
extern Flexcan_Ip_MsgBuffType can2_rx[CAN2_RX_SIZE];
extern Flexcan_Ip_MsgBuffType can3_rx[CAN3_RX_SIZE];
extern Flexcan_Ip_MsgBuffType can4_rx[CAN4_RX_SIZE];
extern Flexcan_Ip_MsgBuffType can5_rx[CAN5_RX_SIZE];

typedef struct {
    uint8_t instance;
    Flexcan_Ip_DataInfoType txInfo;
    uint32_t msgId;
    uint8_t data[8];
} CanTxQueue;

typedef struct 
{
    uint8_t instance;
    Flexcan_Ip_MsgBuffType frame;
} CanRx;


void Init_TxPool(void);
int32_t GetTxMB(uint8_t instance);
void ReleaseTxMb(uint8_t instance, uint8_t mbIdx);
void Can_InitOne(uint8_t inst, Flexcan_Ip_StateType *state, const Flexcan_Ip_ConfigType *config, Flexcan_Ip_MsgBuffType *rxMb);
bool WaitStartMode(uint8_t instance);
#endif
