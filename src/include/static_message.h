/*
 * receive_task.h
 *
 *  Created on: 17 Mar 2026
 *      Author: 20235607
 */

#ifndef INCLUDE_STATIC_MESSAGE_H_
#define INCLUDE_STATIC_MESSAGE_H_

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t addr;
    uint8_t bus;
    uint32_t freq_100;
    uint8_t vl_len;
    uint8_t vl[8]; // Max length of a CAN message payload
} static_dsu_msg_t;

extern const static_dsu_msg_t PRIUS_DSU_MSGS_C[];
extern const size_t PRIUS_DSU_MSGS_C_COUNT;




#endif /* INCLUDE_STATIC_MESSAGE_H_ */
