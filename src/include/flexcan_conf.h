#ifndef INCLUDE_FLEXCAN_CONF_H_
#define INCLUDE_FLEXCAN_CONF_H_


#include "FlexCAN_Ip.h"
#include <stdint.h>

#define INST_FLEXCAN0 (0u)
#define INST_FLEXCAN1 (1u)
#define INST_FLEXCAN2 (2u)
#define INST_FLEXCAN3 (3u)
#define INST_FLEXCAN4 (4u)
#define INST_FLEXCAN5 (5u)

#define CAN_RX_SIZE (6u)
#define CAN0_TX_SIZE (58u)
#define CAN1_TX_SIZE (58u)
#define CAN2_TX_SIZE (10u)
#define CAN3_TX_SIZE (26u)
#define CAN4_TX_SIZE (10u)
#define CAN5_TX_SIZE (10u)

#define TJA1153_START_ID (0x555u)
#define TJA1153_CONFIG_ID (0x18DA00F1u)

Flexcan_Ip_MsgBuffType can_rx[6][CAN_RX_SIZE];
volatile boolean can_rx_flag[6];
uint8_t txMb[6] = {0u};
uint8_t rxMb[6] = {0u};

uint8_t GetTxMB(uint8_t instance);
uint8_t GetRxMB(uint8_t instance);

#endif
