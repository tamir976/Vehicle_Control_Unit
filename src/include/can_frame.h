#ifndef INCLUDE_CAN_FRAME_H_
#define INCLUDE_CAN_FRAME_H_

#include <stdint.h>
#include "FlexCAN_Ip.h"
#include "FreeRTOS.h"
#include "task.h"

#define CAN_CACHE_SIZE        (1024u)
#define CAN_DEFAULT_TIMEOUT   pdMS_TO_TICKS(500u)


static inline uint32_t CanHash(uint32_t id)
{
    return id & (CAN_CACHE_SIZE - 1u);
}


typedef struct
{
    uint8_t used;
    uint8_t bus;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    boolean valid;
    uint32_t timestamp;
    TickType_t lastRxTick;
} CanFrameEntry;

typedef struct
{
    CanFrameEntry *entries;
} CanFrameCache;

extern CanFrameEntry bus0Table[CAN_CACHE_SIZE];
extern CanFrameEntry bus1Table[CAN_CACHE_SIZE];
extern CanFrameEntry bus3Table[CAN_CACHE_SIZE];

extern CanFrameCache gCan0Cache;
extern CanFrameCache gCan1Cache;
extern CanFrameCache gCan3Cache;

void CanCache_UpdateFromISR(uint8_t bus, const Flexcan_Ip_MsgBuffType *rxMsg);

#endif /* INCLUDE_CAN_FRAME_H_ */
