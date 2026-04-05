#ifndef INCLUDE_HACKED_PANDA_H_
#define INCLUDE_HACKED_PANDA_H_

#include <stdint.h>
#include "FlexCAN_Ip.h"
#include "flexcan_conf.h"
#include "monitor.h"

#define PRIUS_SPEED_MSGID   (0x0B4u)
#define PRIUS_GEAR_MSGID    (0x127u)

const uint8_t PRIUS_SPEED_MSG_DATA[8] = {0x30, 0x00, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
const uint8_t PRIUS_GEAR_MSG_DATA[8]  = {0x07, 0xA0, 0x1F, 0x00, 0x08, 0x00, 0x10, 0x00};

void ForwardFrame(uint8_t fromInst, uint8_t toInst, const Flexcan_Ip_MsgBuffType *msg);
#endif
