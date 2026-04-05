#ifndef INCLUDE_CAR_CONTROL_H_
#define INCLUDE_CAR_CONTROL_H_

#include <stdint.h>
#include <FreeRTOS.h>

#define ID_STEER_ANGLE (0x111)
#define ID_ACC_VALUE (0x222)

typedef struct {
	float steeringAngleDeg;
	float acc_value;
	TickType_t last_update_tick;
} CarControl;

CarControl gCarControl;

#endif /* INCLUDE_CAR_CONTROL_H_ */
