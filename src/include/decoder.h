#ifndef INCLUDE_DECODER_H_
#define INCLUDE_DECODER_H_

#include "car_state.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdint.h>
#include <math.h>
#include "car_control.h"

extern CarState gCarState[2];
extern CarControl gCarControl[2];
extern volatile uint8_t CsActiveId;
extern volatile uint8_t CcActiveId;


void CarState_clear(CarState *cs);
void CarControl_clear(CarControl *cc);
void CarState_update(CarState *cs);
void CarControl_update(CarControl *cc);
void CsDecodeTask(void *pv);
void CcDecodeTask(void *pv);

#endif /* INCLUDE_DECODER_H_ */
