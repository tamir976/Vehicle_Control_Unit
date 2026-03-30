#include "include/monitor.h"
#include "include/can_frame.h"
#include "include/oled_display.h"

AppStats gStats = {0};


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
    size_t i;
    now = xTaskGetTickCount();

    taskENTER_CRITICAL();
    for(i = 0u; i < CAN_CACHE_SIZE; i++){
        if(gCan0Cache.entries[i].used != 0u){
            if((now - gCan0Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan0Cache.entries[i].valid = FALSE;
            }else{
                gCan0Cache.entries[i].valid = TRUE;
            }
        }
        if(gCan1Cache.entries[i].used != 0u){
            if((now - gCan1Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan1Cache.entries[i].valid = FALSE;
            }else{
                gCan1Cache.entries[i].valid = TRUE;
            }
        }
        if(gCan3Cache.entries[i].used != 0u){
            if((now - gCan3Cache.entries[i].lastRxTick) > CAN_DEFAULT_TIMEOUT){
                gCan3Cache.entries[i].valid = FALSE;
            }else{
                gCan3Cache.entries[i].valid = TRUE;
            }
        }
    }
    taskEXIT_CRITICAL();
}

