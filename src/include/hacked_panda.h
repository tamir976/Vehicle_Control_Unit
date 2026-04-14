#ifndef INCLUDE_HACKED_PANDA_H_
#define INCLUDE_HACKED_PANDA_H_

#include <stdint.h>
#include <FlexCAN_Ip.h>
#include "flexcan_conf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define PRIUS_SPEED_MSGID   (0x0B4u)
#define PRIUS_GEAR_MSGID    (0x127u)

extern const uint8_t PRIUS_SPEED_MSG_DATA[8];
extern const uint8_t PRIUS_GEAR_MSG_DATA[8];
typedef struct{
    uint8_t instance;
    Flexcan_Ip_MsgBuffType frame;
} epsMsg;

void ForwardFrame(uint8_t fromInst, uint8_t toInst, const Flexcan_Ip_MsgBuffType *msg);
void EPSTask(void *pv);
#endif
