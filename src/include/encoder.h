#include <stdint.h>
#include "car_state.h"
#include "car_control.h"


void Encoder_Init(void);
uint8_t Encoder_PackMessage(uint32_t id, uint8_t *data, const CarState *cs, const CarControl *cc, uint32_t frame);
