#include "include/can_frame.h"
#include "include/monitor.h"
#include "include/flexcan_conf.h"
#include <task.h>
#include <string.h>

CanFrameEntry bus0Table[CAN_CACHE_SIZE];
CanFrameEntry bus1Table[CAN_CACHE_SIZE];
CanFrameEntry bus2Table[CAN_CACHE_SIZE];
CanFrameEntry bus3Table[CAN_CACHE_SIZE];

CanFrameCache gCan0Cache ={
    .entries = bus0Table,
};
CanFrameCache gCan1Cache ={
    .entries = bus1Table,
};
CanFrameCache gCan2Cache ={
    .entries = bus2Table,
};
CanFrameCache gCan3Cache ={
    .entries = bus3Table,
};


CanFrameEntry *CanCache_FindOrCreate(uint8_t bus, uint32_t id)
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

bool CanCache_CopyFrame(const CanFrameCache *cache, uint32_t id, CanFrameEntry *out)
{
    uint32_t index;
    uint32_t start;
    bool found;

    index = CanHash(id);
    start = index;
    found = FALSE;

    taskENTER_CRITICAL();
    while (1)
    {
        const CanFrameEntry *entry = &cache->entries[index];

        if ((entry->used != 0u) && (entry->id == id))
        {
            if (entry->valid == TRUE)
            {
                (void)memcpy(out, entry, sizeof(*out));
                found = TRUE;
            }
            break;
        }

        index = (index + 1u) & (CAN_CACHE_SIZE - 1u);
        if (index == start)
        {
            break;
        }
    }
    taskEXIT_CRITICAL();

    return found;
}
void CanCache_Update(uint8_t bus, const Flexcan_Ip_MsgBuffType *rxMsg)
{
    uint32_t id = rxMsg->msgId;
    uint8_t dlc = rxMsg->dataLen;
    
    taskENTER_CRITICAL();
    CanFrameEntry *entry = CanCache_FindOrCreate(bus, id);    
    if (entry != NULL)
    {
        entry->dlc = dlc;
        entry->used = 1u;
        (void)memcpy(entry->data, rxMsg->data, dlc);
        if (dlc < 8u)
        {
            (void)memset(&entry->data[dlc], 0, (8u - dlc));
        }

        entry->valid = true;
        entry->timestamp = rxMsg->time_stamp;
        entry->lastRxTick = xTaskGetTickCount();
        
    }
    taskEXIT_CRITICAL();
}


void CanCarRxTask(void *pv){
    (void)pv;
    CanRx canRx_;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;){
        if(xQueueReceive(g_canCarRxQueue, &canRx_, portMAX_DELAY) == pdTRUE){
            CanCache_Update(canRx_.instance, &canRx_.frame);
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }
}

void CanPcRxTask(void *pv){
    (void)pv;
    CanRx canRx_;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;){
        if(xQueueReceive(g_canPcRxQueue, &canRx_, portMAX_DELAY) == pdTRUE){
            CanCache_Update(canRx_.instance, &canRx_.frame);
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }
}