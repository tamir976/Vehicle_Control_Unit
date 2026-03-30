
#ifndef INCLUDE_MONITOR_H_
#define INCLUDE_MONITOR_H_

#include <stdint.h>

typedef struct{
	volatile uint32_t canCacheOverflow;
	volatile uint32_t canTxFails;
	volatile uint32_t canInitFails;
} AppStats;

extern AppStats gStats;

void Fatal_Error(const char *msg);
void CanCache_RefreshValidity(void);

#endif /* INCLUDE_MONITOR_H_ */
