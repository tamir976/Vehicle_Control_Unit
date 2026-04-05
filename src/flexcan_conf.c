#include "include/flexcan_conf.h"

uint8_t GetTxMB(uint8_t instance){

    uint8_t mb = CAN_RX_SIZE + txMb[instance];

    if(instance == INST_FLEXCAN0){
        txMb[instance] = (txMb[instance] + 1u) % CAN0_TX_SIZE;    
        return mb;
    }
    else if (instance == INST_FLEXCAN1){
        txMb[instance] = (txMb[instance] + 1u) % CAN1_TX_SIZE;    
        return mb;
    }
    else if (instance == INST_FLEXCAN2){
        txMb[instance] = (txMb[instance] + 1u) % CAN2_TX_SIZE;    
        return mb;
    }
    else if (instance == INST_FLEXCAN3){
        txMb[instance] = (txMb[instance] + 1u) % CAN3_TX_SIZE;    
        return mb;
    }
    else if (instance == INST_FLEXCAN4){
        txMb[instance] = (txMb[instance] + 1u) % CAN4_TX_SIZE;    
        return mb;
    }
    else if (instance == INST_FLEXCAN5){
        txMb[instance] = (txMb[instance] + 1u) % CAN5_TX_SIZE;    
        return mb;
    }
    else{
        return 0u;
    }
}

uint8_t GetRxMB(uint8_t instance){
    uint8_t mb = 0 + rxMb[instance];
    rxMb[instance] = (rxMb[instance] + 1u) % CAN_RX_SIZE;
    return mb;
}