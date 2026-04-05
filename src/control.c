#include <include/control.h>


void ControlTask(void *pv){
    TickType_t lastWakeTime;
    (void)pv;
    lastWakeTime = xTaskGetTickCount();

    for(;;){
        taskENTER_CRITICAL();
        TickType_t now = xTaskGetTickCount();
        if((now - gCarControl.last_update_tick) > pdMS_TO_TICKS(1000u)){
            /* No update in 1 second, reset controls */
            gCarControl.steeringAngleDeg = 0.0f;
            gCarControl.acc_value = 0.0f;
        }
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100u));
    }
}