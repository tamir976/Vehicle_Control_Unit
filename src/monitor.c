#include "include/monitor.h"
#include <Lpuart_Uart_Ip.h>
#include <Lpuart_Uart_Ip_Sa_PBcfg.h>
#include <stdio.h>
#include "include/car_state.h"
#include "include/flexcan_conf.h"
#include "include/oled_display.h"
#include "include/can_frame.h"
#include "include/car_control.h"
#include "include/decoder.h"
#include "include/control.h"
#include "include/encoder.h"


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
    uint32_t lastAccTxCount = 0u;
    bool monitorInitialized = false;
    (void)pv;
    lastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        CanCache_RefreshValidity();

        CarControl ccSnap;
        CarState csSnap;
        AccelCommand accelSnap;

        taskENTER_CRITICAL();
        ccSnap = gCarControl[CcActiveId];
        csSnap = gCarState[CsActiveId];
        accelSnap = gAccelCommand;
        taskEXIT_CRITICAL();
        char buffer[512];
        int length = snprintf(buffer, sizeof(buffer),
                              "\r\n"
                              "CS: pcm_status=%u follow_dist=%u acc_type=%u\r\n"
                              "CC: emergency=%u cancel_req=%u long_state=%u\r\n"
                              "ACC_CMD: cancel=%u dist_btn=%u mini=%u permit_brake=%u release_standstill=%u\r\n",
                              (unsigned)csSnap.pcmAccStatus,
                              (unsigned)csSnap.pcmFollowDistance,
                              (unsigned)csSnap.accType,
                              (unsigned)ccSnap.emergency,
                              (unsigned)ccSnap.cancelReq,
                              (unsigned)ccSnap.actuators.longcontrolstate,
                              (unsigned)accelSnap.CANCEL_REQ,
                              (unsigned)accelSnap.DISTANCE,
                              (unsigned)accelSnap.MINI_CAR,
                              (unsigned)accelSnap.PERMIT_BRAKING,
                              (unsigned)accelSnap.RELEASE_STANSTILL);

        if(length > 0){
            (void)Lpuart_Uart_Ip_SyncSend(INST_UART2,
                                         (const uint8_t *)buffer,
                                         (uint32_t)length,
                                         10000u);
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500u));
    }
}
