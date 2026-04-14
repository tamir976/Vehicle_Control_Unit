#ifndef INCLUDE_ENCODER_H_
#define INCLUDE_ENCODER_H_


#include <stdint.h>
#include <FlexCAN_Ip.h>
#include <string.h>
#include "control.h"

void encode_steer_command(uint32_t id, SteerCommand *cmd, uint8_t dlc);
void encode_accel_command(uint32_t id, AccelCommand *cmd, uint8_t dlc);
void encode_pcs_command(uint32_t id, PcsCommand *cmd, uint8_t dlc);
void encode_pcs_command_2(uint32_t id, PcsCommand_2 *cmd, uint8_t dlc);
void encode_acc_cancel_command(uint32_t id, AccelCancelCommand *cmd, uint8_t dlc);
void encode_fcw_command(uint32_t id, FcwCommand *cmd, uint8_t dlc);
void encode_ui_command(uint32_t id, UICommand *cmd, uint8_t dlc);
void encode_ipas_steer_command(uint32_t id, SteeringIpasCommand *cmd, uint8_t dlc);
void encode_ipas_steer_comma_command(uint32_t id, SteeringIpasCommaCommand *cmd, uint8_t dlc);
uint8_t calculate_checksum(uint32_t id, const uint8_t *data, uint8_t dlc);
void CAN_Send(uint32_t id, uint8_t bus, uint8_t *data, uint8_t dlc);
static inline void set_bits_be(uint8_t *data, uint16_t start, uint8_t len, uint32_t value){
    if(len < 32){
        value &= (1u << len) - 1u;
    }

    for(uint8_t i = 0; i < len; i++){
        uint16_t bit = start;
        uint8_t srcBit = (uint8_t)(len - 1u - i);
        uint8_t byte;
        uint8_t offset;

        for(uint8_t step = 0; step < i; step++){
            if((bit % 8u) == 0u){
                bit = (uint16_t)(bit + 15u);
            }else{
                bit--;
            }
        }

        byte = (uint8_t)(bit / 8u);
        offset = (uint8_t)(bit % 8u);
        if(value & (1u << srcBit)){
            data[byte] |= (1u << offset);
        }else{
            data[byte] &= ~(1u << offset);
        }
    }
}

static inline void set_bits_le(uint8_t *data, uint16_t start, uint8_t len, uint32_t value){
    if(len < 32u){
        value &= (1u << len) - 1u;
    }

    for(uint8_t i = 0u; i < len; i++){
        uint16_t bit = (uint16_t)(start + i);
        uint8_t byte = (uint8_t)(bit / 8u);
        uint8_t offset = (uint8_t)(bit % 8u);
        if((value & (1u << i)) != 0u){
            data[byte] |= (uint8_t)(1u << offset);
        }else{
            data[byte] &= (uint8_t)~(1u << offset);
        }
    }
}

#define set_bits(data, start, len, value) set_bits_be((data), (start), (len), (value))
#endif
