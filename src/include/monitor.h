#ifndef INCLUDE_MONITOR_H_
#define INCLUDE_MONITOR_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

#define INST_UART2 (2)

void CanCache_RefreshValidity(void);
void MonitorTask(void *pv);

#endif /* INCLUDE_MONITOR_H_ */
