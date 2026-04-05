#ifndef INCLUDE_MONITOR_H_
#define INCLUDE_MONITOR_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <stdio.h>
#include "Lpuart_Uart_Ip.h"
#include "Lpuart_Uart_Ip_Sa_PBcfg.h"
#include "car_state.h"
#include "flexcan_conf.h"
#include "oled_display.h"
#include "can_frame.h"

#define INST_UART2 (2)
typedef struct{
    volatile uint32_t canCacheOverflow;
    volatile uint32_t canTxFails;
    volatile uint32_t canInitFails;

    /* CAN Error Statistics - per instance */
    volatile uint32_t canBusOff[6];
    volatile uint32_t canErrors[6];
    volatile uint32_t canRecoveryFails[6];
    volatile uint32_t canRxWarnings[6];
    volatile uint32_t canTxWarnings[6];
    volatile uint32_t canRxFifoOverflow[6];
    volatile uint32_t canUnknownErrors[6];

    /* Specific error types */
    volatile uint32_t canCrcErrors[6];
    volatile uint32_t canFormErrors[6];
    volatile uint32_t canStuffErrors[6];
    volatile uint32_t canBit0Errors[6];
    volatile uint32_t canBit1Errors[6];
} AppStats;

extern AppStats gStats;

void Fatal_Error(const char *msg);
void CanCache_RefreshValidity(void);
void CanCheckFlag(void);
void UartTask(void *pv); 
void Uart_SendString(const char *msg);
void CacheMonitorTask(void *pv);

#endif /* INCLUDE_MONITOR_H_ */
