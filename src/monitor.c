#include "include/monitor.h"
#include "include/car_state.h"
#include "include/can_frame.h"
#include "include/car_control.h"
#include "include/control.h"
#include "include/decoder.h"
#include <Lpuart_Uart_Ip.h>
#include <Lpuart_Uart_Ip_Sa_PBcfg.h>
#include <stdio.h>

static int32_t AngleToCentiDeg(float angle)
{
    return (int32_t)(angle * 100.0f);
}


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
        float applyAngle;
        float ccAngle;

        taskENTER_CRITICAL();
        applyAngle = gSteeringIpasCommand.ANGLE;
        ccAngle = gCarControl[CcActiveId].actuators.steeringAngleDegCmd;
        taskEXIT_CRITICAL();

        char buffer[96];
        int length = snprintf(buffer, sizeof(buffer),
                              "IPAS_ANGLE: apply_cd=%ld cc_cd=%ld\r\n",
                              (long)AngleToCentiDeg(applyAngle),
                              (long)AngleToCentiDeg(ccAngle));

        if(length > 0){
            if(length >= (int)sizeof(buffer)){
                length = (int)sizeof(buffer) - 1;
            }
            (void)Lpuart_Uart_Ip_SyncSend(INST_UART2,
                                          (const uint8_t *)buffer,
                                          (uint32_t)length,
                                          80000u);
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500u));
    }
}
