#ifndef INCLUDE_DECODER_H_
#define INCLUDE_DECODER_H_

#include "car_state.h"
#include "can_frame.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <math.h>
#include "car_control.h"

void CarState_clear(CarState *cs);
void CarControl_clear(CarControl *cc);
void CarState_update(CarState *cs);
void DecoderTask(void *pv);

#endif /* INCLUDE_DECODER_H_ */
