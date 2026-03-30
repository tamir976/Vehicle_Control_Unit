/*
 * decoder.h
 *
 *  Created on: 26 Mar 2026
 *      Author: 20235607
 */

#ifndef INCLUDE_DECODER_H_
#define INCLUDE_DECODER_H_

#include "car_state.h"
#include "can_frame.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <math.h>

void CarState_clear(CarState *cs);
void CarState_update(CarState *cs);

#endif /* INCLUDE_DECODER_H_ */
