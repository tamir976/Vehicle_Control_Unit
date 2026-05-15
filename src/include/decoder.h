#ifndef INCLUDE_DECODER_H_
#define INCLUDE_DECODER_H_

#include "car_state.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdint.h>
#include <math.h>
#include "car_control.h"
#define DT_CTRL 0.01f
#define KF_A00 1.0f
#define KF_A01 DT_CTRL
#define KF_A10 0.0f
#define KF_A11 1.0f
#define KF_C0 1.0f
#define KF_C1 0.0f
#define KF_K0 0.8f
#define KF_K1 0.2f

extern CarState gCarState[2];
extern CarControl gCarControl[2];
extern volatile uint8_t CsActiveId;
extern volatile uint8_t CcActiveId;

typedef struct {
    float x0;
    float x1;
    float AK0;
    float AK1;
    float AK2;
    float AK3;
    float K0;
    float K1;
} KF1D;

void CarState_clear(CarState *cs);
void CarControl_clear(CarControl *cc);
void CarState_update(CarState *cs);
void CarControl_update(CarControl *cc);
void CsDecodeTask(void *pv);
void CcDecodeTask(void *pv);

#endif /* INCLUDE_DECODER_H_ */
