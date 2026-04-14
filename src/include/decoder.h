#ifndef INCLUDE_DECODER_H_
#define INCLUDE_DECODER_H_

#include "car_state.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdint.h>
#include <math.h>
#include "car_control.h"

void CarState_clear(CarState *cs);
void CarControl_clear(CarControl *cc);
void CarState_update(CarState *cs);
void CarControl_update(CarControl *cc);
void CarState_GetSnapshot(CarState *out);
void CarControl_GetSnapshot(CarControl *out);
void DecoderTask(void *pv);

#endif /* INCLUDE_DECODER_H_ */
