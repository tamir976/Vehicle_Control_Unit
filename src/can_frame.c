#include "include/can_frame.h"
#include "include/monitor.h"
#include <string.h>

CanFrameEntry bus0Table[CAN_CACHE_SIZE];
CanFrameEntry bus1Table[CAN_CACHE_SIZE];
CanFrameEntry bus2Table[CAN_CACHE_SIZE];
CanFrameEntry bus3Table[CAN_CACHE_SIZE];

CanFrameCache gCan0Cache =
{
    .entries = bus0Table,
};
CanFrameCache gCan1Cache =
{
    .entries = bus1Table,
};
CanFrameCache gCan2Cache =
{
    .entries = bus2Table,
};
CanFrameCache gCan3Cache =
{
    .entries = bus3Table,
};


static CanFrameEntry *CanCache_FindOrCreate(uint8_t bus, uint32_t id)
{
    uint32_t index = CanHash(id);
    uint32_t start = index;

    while (1)
    {
        CanFrameEntry *entry;

        if (bus == 0u) { entry = &gCan0Cache.entries[index]; }
        else if (bus == 1u) { entry = &gCan1Cache.entries[index]; }
        else if (bus == 2u) {entry = &gCan2Cache.entries[index];}
        else if (bus == 3u) { entry = &gCan3Cache.entries[index]; }
        else { return NULL; }

        if (entry->used == 0u)
        {
            entry->bus = bus;
            entry->id = id;
            return entry;
        }
        else if (entry->id == id)
        {
            return entry;
        }

        index = (index + 1u) & (CAN_CACHE_SIZE - 1u);
        if (index == start)
        {
            return NULL;  /* cache full */
        }
    }
}
void CanCache_UpdateFromISR(uint8_t bus, const Flexcan_Ip_MsgBuffType *rxMsg)
{
    uint32_t id;
    uint8_t dlc;
    CanFrameEntry *entry;
    UBaseType_t savedInterruptStatus;

    id = rxMsg->msgId;
    dlc = rxMsg->dataLen;

    savedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    entry = CanCache_FindOrCreate(bus, id);
    if (entry != NULL)
    {
        entry->dlc = dlc;
        entry->used = 1u;
        (void)memcpy(entry->data, rxMsg->data, dlc);
        if (dlc < 8u)
        {
            (void)memset(&entry->data[dlc], 0, (8u - dlc));
        }

        entry->valid = TRUE;
        entry->timestamp = rxMsg->time_stamp;
        entry->lastRxTick = xTaskGetTickCountFromISR();
    }
    else
    {
        gStats.canCacheOverflow++;
    }

    taskEXIT_CRITICAL_FROM_ISR(savedInterruptStatus);
}
