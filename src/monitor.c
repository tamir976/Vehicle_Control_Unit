#include "include/monitor.h"
#include <Lpuart_Uart_Ip.h>
#include <Lpuart_Uart_Ip_Sa_PBcfg.h>
#include "include/car_state.h"
#include "include/flexcan_conf.h"
#include "include/oled_display.h"
#include "include/can_frame.h"
#include "include/car_control.h"
#include "include/decoder.h"

AppStats gStats = {0};

static void Monitor_DrawStatus(void)
{
    char line[22];
    (void)snprintf(line, sizeof(line), "canTxFails:%lu", (unsigned long)gStats.canTxFails);
    ssd1306_draw_text(0u, 0u, line);
}

void Uart_SendString(const char *msg){
    if(msg == NULL){
        return;
    }
    (void)Lpuart_Uart_Ip_SyncSend(INST_UART2,
                                  (const uint8_t *)msg,
                                  (uint32_t)strlen(msg),
                                  10000u);
}


void Fatal_Error(const char *msg){
	ssd1306_draw_text(0u, 0u, "Fatal Error");
	if(msg != NULL){
		ssd1306_draw_text(0u, 16u, msg);
	}
	taskDISABLE_INTERRUPTS();
	for(;;){
        
	}
}

void CanCache_RefreshValidity(void)
{
    TickType_t now;
    now = xTaskGetTickCount();

    taskENTER_CRITICAL();
    for(size_t i = 0u; i < CAN_CACHE_SIZE; i++){
        if(gCan0Cache.entries[i].used != 0u){
            if((now - gCan0Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan0Cache.entries[i].valid = FALSE;
            }
        }
        if(gCan1Cache.entries[i].used != 0u){
            if((now - gCan1Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan1Cache.entries[i].valid = FALSE;
            }
        }
        if(gCan2Cache.entries[i].used != 0u){
            if((now - gCan2Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan2Cache.entries[i].valid = FALSE;
            }
        }
        if(gCan3Cache.entries[i].used != 0u){
            if((now - gCan3Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan3Cache.entries[i].valid = FALSE;
            }
        }
    }
    taskEXIT_CRITICAL();
}

void CanCheckFlag(void){
    Monitor_DrawStatus();
}

void UartTask(void *pv)
{
    TickType_t lastWakeTime;
    CarState snap;
    char line[256];
    int n;

    (void)pv;
    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        CarState_GetSnapshot(&snap);

        n = snprintf(line,
                     sizeof(line),
                     "v=%.2f a=%.2f steer=%.1f rpm=%.0f gas=%.1f gear=%u brk=%u abs=%u tc=%u\r\n",
                     (double)snap.out.vEgo,
                     (double)snap.out.aEgo,
                     (double)snap.out.steeringAngleDeg,
                     (double)snap.engine_rpm,
                     (double)snap.gas_pedal,
                     (unsigned int)snap.gear,
                     (unsigned int)snap.brake_pressed,
                     (unsigned int)snap.abs_active,
                     (unsigned int)snap.traction_control_active);

        if (n > 0)
        {
            Uart_SendString(line);
        }


        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10000u));
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
        CanCheckFlag();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10u));
    }
}
