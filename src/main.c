#include "S32K344.h"
#include "Clock_Ip.h"
#include "Clock_Ip_Cfg.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Port_Ip_Cfg.h"
#include "Siul2_Dio_Ip.h"
#include "Siul2_Dio_Ip_Cfg.h"
#include "IntCtrl_Ip.h"
#include "IntCtrl_Ip_Cfg.h"
#include "FlexCAN_Ip.h"
#include "FlexCAN_Ip_Cfg.h"
#include "FlexCAN_Ip_sa_PBcfg.h"
#include "FlexCAN_Ip_Irq.h"
#include "FlexCAN_Ip_Types.h"
#include "Lpi2c_Ip_Cfg.h"
#include "Lpi2c_Ip_Sa_PBcfg.h"
#include "Lpi2c_Ip_Irq.h"
#include "Lpi2c_Ip.h"
#include "Lpuart_Uart_Ip.h"
#include "Lpuart_Uart_Ip_Sa_PBcfg.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "OsIf.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "include/monitor.h"
#include "include/oled_display.h"
#include "include/static_message.h"
#include "include/car_state.h"
#include "include/can_frame.h"
#include "include/decoder.h"
#include "include/flexcan_conf.h"
#include "include/hacked_panda.h"
#include "include/car_control.h"
#include "include/control.h"
#include "include/encoder.h"

#define TJA1153_START_ID    (0x555u)
#define TJA1153_CONFIG_ID   (0x18DA00F1u)

volatile int exit_code = 0;
static void Transceivers_Enable(void)
{
    Siul2_Dio_Ip_WritePin(can0_en_PORT, can0_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can1_en_PORT, can1_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can2_en_PORT, can2_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can3_en_PORT, can3_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can4_en_PORT, can4_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can5_en_PORT, can5_en_PIN, STD_HIGH);

    Siul2_Dio_Ip_WritePin(can0_stb_PORT, can0_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can1_stb_PORT, can1_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can2_stb_PORT, can2_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can3_stb_PORT, can3_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can4_stb_PORT, can4_stb_PIN, STD_LOW);
    Siul2_Dio_Ip_WritePin(can5_stb_PORT, can5_stb_PIN, STD_LOW);
}


static void SetupCan_TJA1153(void) {
	Flexcan_Ip_DataInfoType txi = {0};
	uint8_t d[8] = {0};
	txi.msg_id_type = FLEXCAN_MSG_ID_STD;
	txi.data_length = 8u;
	txi.is_polling = TRUE;
	txi.is_remote = FALSE;
	/* CAN4 */
    int32_t mb = GetTxMB(INST_FLEXCAN4);
    if(mb >= 0){
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, mb, &txi, TJA1153_START_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN4, (uint8_t)mb);
        d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
        txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
        txi.data_length = 6u;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN4, (uint8_t)mb);
        d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN4, (uint8_t)mb);
        d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN4, (uint8_t)mb);
        d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
        txi.data_length = 8u;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN4, (uint8_t)mb);
        Siul2_Dio_Ip_WritePin(can4_stb_PORT, can4_stb_PIN, STD_HIGH);
        /* CAN5 */
        (void)memset(d, 0, sizeof(d));
        txi.msg_id_type = FLEXCAN_MSG_ID_STD;
        txi.data_length = 8u;
        mb = GetTxMB(INST_FLEXCAN5);
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, mb, &txi, TJA1153_START_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN5, (uint8_t)mb);
        d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
        txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
        txi.data_length = 6u;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN5, (uint8_t)mb);
        d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN5, (uint8_t)mb);
        d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN5, (uint8_t)mb);
        d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
        txi.data_length = 8u;
        (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, mb, &txi, TJA1153_CONFIG_ID, d, 1000u);
        ReleaseTxMb(INST_FLEXCAN5, (uint8_t)mb);
        Siul2_Dio_Ip_WritePin(can5_stb_PORT, can5_stb_PIN, STD_HIGH);
    }
}



static void Can_Init(void)
{
    Can_InitOne(INST_FLEXCAN0, &FlexCAN_State0, &FlexCAN_Config0, can0_rx);
    Can_InitOne(INST_FLEXCAN1, &FlexCAN_State1, &FlexCAN_Config1, can1_rx);
    Can_InitOne(INST_FLEXCAN2, &FlexCAN_State2, &FlexCAN_Config2, can2_rx);
    Can_InitOne(INST_FLEXCAN3, &FlexCAN_State3, &FlexCAN_Config3, can3_rx);
    Can_InitOne(INST_FLEXCAN4, &FlexCAN_State4, &FlexCAN_Config4, can4_rx);
    Can_InitOne(INST_FLEXCAN5, &FlexCAN_State5, &FlexCAN_Config5, can5_rx);
    SetupCan_TJA1153();

}

/* ========================= IRQ HANDLERS ========================= */

void Flexcan0_bus_off_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN0); }
void Flexcan0_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 0u, 31u, FALSE); }
void Flexcan0_2_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 32u, 63u, FALSE); }
void Flexcan1_bus_off_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN1); }
void Flexcan1_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN1, 0u, 31u, FALSE); }
void Flexcan1_2_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN1, 32u, 63u, FALSE); }
void Flexcan2_bus_off_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN2); }
void Flexcan2_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN2, 0u, 15u, FALSE); }
void Flexcan3_bus_off_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN3); }
void Flexcan3_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN3, 0u, 31u, FALSE); }
void Flexcan4_bus_off_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN4); }
void Flexcan4_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN4, 0u, 15u, FALSE); }
void Flexcan5_bus_off_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN5); }
void Flexcan5_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN5, 0u, 15u, FALSE); }

void I2c0_handler(void)
{
    Lpi2c_Ip_MasterIRQHandler(INST_I2C);
}

void Uart2_handler(void){
	Lpuart_Uart_Ip_IrqHandler(INST_UART2);
}


void FlexcanCar_callback(uint8_t instance,
                         Flexcan_Ip_EventType eventType,
                         uint32_t buffIdx,
                         const Flexcan_Ip_StateType *state)
{
    (void)state;
    BaseType_t higherTaskWoken = pdFALSE;
    if (eventType == FLEXCAN_EVENT_RX_COMPLETE){
        Flexcan_Ip_MsgBuffType *rxBuf = NULL;
        switch (instance)
        {
        case INST_FLEXCAN0:
            rxBuf = &can0_rx[buffIdx];
            break;
        case INST_FLEXCAN1:
            rxBuf = &can1_rx[buffIdx];
            break;
        case INST_FLEXCAN2:
            rxBuf = &can2_rx[buffIdx];
            break;
        default:
            break;
        }
        if(rxBuf != NULL){

            CanRx frame;
            frame.instance = instance;
            frame.frame = *rxBuf;
            (void)xQueueSendFromISR(g_canCarRxQueue, &frame, &higherTaskWoken);
            (void)FlexCAN_Ip_Receive(instance, buffIdx, rxBuf, false);
        }
    }
    if (eventType == FLEXCAN_EVENT_TX_COMPLETE){
        ReleaseTxMb(instance, (uint8_t)buffIdx);
    }
}

void FlexcanPC_callback(uint8_t instance,
                        Flexcan_Ip_EventType eventType,
                        uint32_t buffIdx,
                        const Flexcan_Ip_StateType *state)
{
    (void)state;
    BaseType_t higherTaskWoken = pdFALSE;
    if (eventType == FLEXCAN_EVENT_RX_COMPLETE){
        Flexcan_Ip_MsgBuffType *rxBuf = NULL;
        rxBuf = &can3_rx[buffIdx];
        if(rxBuf != NULL){
            CanRx frame;
            frame.instance = instance;
            frame.frame = *rxBuf;
            (void)xQueueSendFromISR(g_canPcRxQueue, &frame, &higherTaskWoken);
            (void)FlexCAN_Ip_Receive(instance, buffIdx, rxBuf, false);            
        }
    }
    if (eventType == FLEXCAN_EVENT_TX_COMPLETE){
        if (instance == INST_FLEXCAN3){
            ReleaseTxMb(instance, (uint8_t)buffIdx);
        }
    }
}

void EPS_callback_CAN4(uint8_t instance, Flexcan_Ip_EventType eventType,
                        uint32_t buffIdx, const Flexcan_Ip_StateType *state){
    
    (void)state;
    if(eventType == FLEXCAN_EVENT_RX_COMPLETE){
        (void)FlexCAN_Ip_Receive(instance, buffIdx, &can4_rx[buffIdx], false);
        BaseType_t higherTaskWoken = pdFALSE;
        (void)xQueueSendFromISR(epsQueue4, &can4_rx[buffIdx], &higherTaskWoken);
    }
    else if(eventType == FLEXCAN_EVENT_TX_COMPLETE){
        ReleaseTxMb(instance, (uint8_t)buffIdx);
    }
}

void EPS_callback_CAN5(uint8_t instance, Flexcan_Ip_EventType eventType,
                        uint32_t buffIdx, const Flexcan_Ip_StateType *state){
    
    (void)state;
    if(eventType == FLEXCAN_EVENT_RX_COMPLETE){
        (void)FlexCAN_Ip_Receive(instance, buffIdx, &can5_rx[buffIdx], false);
        BaseType_t higherTaskWoken = pdFALSE;
        (void)xQueueSendFromISR(epsQueue5, &can5_rx[buffIdx], &higherTaskWoken);
    }
    else if(eventType == FLEXCAN_EVENT_TX_COMPLETE){
        ReleaseTxMb(instance, (uint8_t)buffIdx);
    }
}

void Flexcan_error_callback(uint8_t instance, 
            Flexcan_Ip_EventType eventType, 
            uint32_t u32ErrStatus, 
            const Flexcan_Ip_StateType *state){
    (void)state;
    (void)u32ErrStatus;
    (void)eventType;
    (void)instance;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __asm__ volatile ("bkpt #0");
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    __asm__ volatile ("bkpt #0");
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}


/* ========================= MAIN ======================== */

int main(void)
{
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
                       g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);
    Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);
    OsIf_Init(NULL);

    IntCtrl_Ip_Init(&IntCtrlConfig_0);
    Lpuart_Uart_Ip_Init(INST_UART2, &Lpuart_Uart_Ip_xHwConfigPB_2);
    Lpi2c_Ip_MasterInit(INST_I2C, &I2c_Lpi2cMaster_HwChannel0_Channel0);

    oled_draw_text(0u, 0u, "Booting");
    Transceivers_Enable();
    
    epsQueue4 = xQueueCreate(8, sizeof(can4_rx[0]));
    epsQueue5 = xQueueCreate(8, sizeof(can5_rx[0]));
    g_canCarRxQueue = xQueueCreate(32, sizeof(CanRx));
    g_canPcRxQueue = xQueueCreate(8, sizeof(CanRx));
    Init_TxPool();
    Can_Init();
   xTaskCreate(StaticDSUTask, "StaticDSU", 512u, NULL, 4u, NULL);
    xTaskCreate(CanCarRxTask, "CanCar", 512u, NULL, 4u, NULL);
    xTaskCreate(CanPcRxTask, "CanPc", 256u, NULL, 4u, NULL);
    xTaskCreate(CsDecodeTask, "CSDecoder", 1024u, NULL, 4u, NULL);
    xTaskCreate(CcDecodeTask, "CCDecoder", 512u, NULL, 4u, NULL);
    xTaskCreate(MonitorTask, "Monitor", 256u, NULL, 4u, NULL);
    xTaskCreate(EPS4Task, "EPS4", 128u, NULL, 4u, NULL);
    xTaskCreate(EPS5Task, "EPS5", 128u, NULL, 4u, NULL);
    xTaskCreate(ControlTask, "Control", 1024u, NULL, 4u, NULL);
    xTaskCreate(EncoderTask, "Encoder", 512u, NULL, 4u, NULL);
    oled_draw_text(0u, 0u, "Initialized");
    vTaskStartScheduler();
    return exit_code;
}
