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
        CarControl ccSnap;
        CarState csSnap;
        SteeringIpasCommand ipasSnap;
        IpasControllerState ipasStateSnap;
        int32_t reqCentiDeg;
        int32_t reqAbsCentiDeg;
        int32_t outCentiDeg;
        int32_t outAbsCentiDeg;
        const char *reqSign;
        const char *outSign;

        taskENTER_CRITICAL();
        ccSnap = gCarControl[CcActiveId];
        csSnap = gCarState[CsActiveId];
        ipasSnap = gSteeringIpasCommand;
        ipasStateSnap = gIpasControllerState;
        taskEXIT_CRITICAL();

        reqCentiDeg = (ccSnap.actuators.steeringAngleDegCmd >= 0.0f)
                    ? (int32_t)((ccSnap.actuators.steeringAngleDegCmd * 100.0f) + 0.5f)
                    : (int32_t)((ccSnap.actuators.steeringAngleDegCmd * 100.0f) - 0.5f);
        reqAbsCentiDeg = (reqCentiDeg < 0) ? -reqCentiDeg : reqCentiDeg;
        reqSign = (reqCentiDeg < 0) ? "-" : "";
        outCentiDeg = (ipasSnap.ANGLE >= 0.0f)
                    ? (int32_t)((ipasSnap.ANGLE * 100.0f) + 0.5f)
                    : (int32_t)((ipasSnap.ANGLE * 100.0f) - 0.5f);
        outAbsCentiDeg = (outCentiDeg < 0) ? -outCentiDeg : outCentiDeg;
        outSign = (outCentiDeg < 0) ? "-" : "";

        char buffer[128];
        int length = snprintf(buffer, sizeof(buffer),
                              "cc=%s%ld.%02ld out=%s%ld.%02ld ipas=%u en=%u st=%u\r\n",
                              reqSign,
                              (long)(reqAbsCentiDeg / 100),
                              (long)(reqAbsCentiDeg % 100),
                              outSign,
                              (long)(outAbsCentiDeg / 100),
                              (long)(outAbsCentiDeg % 100),
                              (unsigned)csSnap.ipasState,
                              (unsigned)ipasStateSnap.steer_angle_enabled,
                              (unsigned)ipasSnap.STATE);

        if (length > 0)
        {
            (void)Lpuart_Uart_Ip_SyncSend(INST_UART2,
                                         (const uint8_t *)buffer,
                                         (uint32_t)length,
                                         10000u);
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500u));
    }
}
