#include "include/monitor.h"
#include <Lpuart_Uart_Ip.h>
#include <Lpuart_Uart_Ip_Sa_PBcfg.h>
#include "include/car_state.h"
#include "include/flexcan_conf.h"
#include "include/oled_display.h"
#include "include/can_frame.h"
#include "include/car_control.h"
#include "include/decoder.h"


void CanCache_RefreshValidity(void)
{
    TickType_t now;
    now = xTaskGetTickCount();

    for(size_t i = 0u; i < CAN_CACHE_SIZE; i++){
        taskENTER_CRITICAL();
        if(gCan0Cache.entries[i].used != 0u){
            if((now - gCan0Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan0Cache.entries[i].valid = false;
            }
        }
        if(gCan1Cache.entries[i].used != 0u){
            if((now - gCan1Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan1Cache.entries[i].valid = false;
            }
        }
        if(gCan2Cache.entries[i].used != 0u){
            if((now - gCan2Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan2Cache.entries[i].valid = false;
            }
        }
        if(gCan3Cache.entries[i].used != 0u){
            if((now - gCan3Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan3Cache.entries[i].valid = false;
            }
        }
        taskEXIT_CRITICAL();
    }
}


void MonitorTask(void *pv)
{
    TickType_t lastWakeTime;
    (void)pv;

    lastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        CanCache_RefreshValidity();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100u));
    }
}
