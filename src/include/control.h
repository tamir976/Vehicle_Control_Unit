#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "monitor.h"
#include "car_control.h"

void ControlTask(void *pv);