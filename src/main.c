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
#include "Lpi2c_Ip_Cfg.h"
#include "Lpi2c_Ip_Sa_PBcfg.h"
#include "Lpi2c_Ip_Irq.h"
#include "Lpi2c_Ip.h"
#include "Lpuart_Uart_Ip.h"
#include "Lpuart_Uart_Ip_Sa_PBcfg.h"
#include "FreeRTOS.h"
#include "OsIf.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
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

static void WaitStartMode(uint8_t instance)
{
	uint32_t start = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
	const uint32_t timeoutTicks = OsIf_MicrosToTicks(100000u, OSIF_COUNTER_SYSTEM);

    while (FlexCAN_Ip_GetStartMode(instance) == FALSE)
    {
        if(OsIf_GetElapsed(&start, OSIF_COUNTER_SYSTEM) >= timeoutTicks){
        	Fatal_Error("CAN start timeout");
        }
    }
}

static void SetupCan_TJA1153(void) {
	Flexcan_Ip_DataInfoType txi;
	uint8_t d[8] = {0};
	txi.msg_id_type = FLEXCAN_MSG_ID_STD;
	txi.data_length = 8u;
	txi.is_polling = TRUE;
	txi.is_remote = FALSE;
	/* CAN4 */
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, GetTxMB(INST_FLEXCAN4), &txi, TJA1153_START_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
	txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
	txi.data_length = 6u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, GetTxMB(INST_FLEXCAN4), &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, GetTxMB(INST_FLEXCAN4), &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, GetTxMB(INST_FLEXCAN4), &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
	txi.data_length = 8u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, GetTxMB(INST_FLEXCAN4), &txi, TJA1153_CONFIG_ID, d, 1000u);
	Siul2_Dio_Ip_WritePin(can4_stb_PORT, can4_stb_PIN, STD_HIGH);
	/* CAN5 */
	(void)memset(d, 0, sizeof(d));
	txi.msg_id_type = FLEXCAN_MSG_ID_STD;
	txi.data_length = 8u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, GetTxMB(INST_FLEXCAN5), &txi, TJA1153_START_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
	txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
	txi.data_length = 6u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, GetTxMB(INST_FLEXCAN5), &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, GetTxMB(INST_FLEXCAN5), &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, GetTxMB(INST_FLEXCAN5), &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
	txi.data_length = 8u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, GetTxMB(INST_FLEXCAN5), &txi, TJA1153_CONFIG_ID, d, 1000u);
	Siul2_Dio_Ip_WritePin(can5_stb_PORT, can5_stb_PIN, STD_HIGH);
}

static void Can_InitOne(uint8_t inst,
                        Flexcan_Ip_StateType *state,
                        const Flexcan_Ip_ConfigType *config,
                        Flexcan_Ip_MsgBuffType *rxMb)
{
    Flexcan_Ip_DataInfoType rxStd;
    Flexcan_Ip_StatusType st;

    rxStd.msg_id_type = FLEXCAN_MSG_ID_STD;
    rxStd.data_length = 8u;
    rxStd.is_polling  = FALSE;
    rxStd.is_remote   = FALSE;
    rxStd.fd_enable   = FALSE;

    st = FlexCAN_Ip_Init(inst, state, config);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("CAN init fail"); }

    st = FlexCAN_Ip_EnterFreezeMode(inst);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Freeze fail"); }

    (void)FlexCAN_Ip_SetRxMaskType(inst, FLEXCAN_RX_MASK_INDIVIDUAL);
    for(uint8_t i = 0u; i < CAN_RX_SIZE; i++){
        (void)FlexCAN_Ip_SetRxIndividualMask(inst, i, 0x00000000u);
        st = FlexCAN_Ip_ConfigRxMb(inst, i, &rxStd, 0x000u);
        if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Rx MB cfg fail"); }
        st = FlexCAN_Ip_Receive(inst, i, &rxMb[i], FALSE);
        if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Rx MB start fail"); }
    }
    st = FlexCAN_Ip_ExitFreezeMode(inst);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Exit freeze fail"); }

    (void)FlexCAN_Ip_SetStartMode(inst);
    WaitStartMode(inst);

    FlexCAN_Ip_EnableInterrupts_Privileged(inst);
}

static void Can_Init(void)
{
    Can_InitOne(INST_FLEXCAN0, &FlexCAN_State0, &FlexCAN_Config0, &can_rx[0][0]);
    Can_InitOne(INST_FLEXCAN1, &FlexCAN_State1, &FlexCAN_Config1, &can_rx[1][0]);
    Can_InitOne(INST_FLEXCAN2, &FlexCAN_State2, &FlexCAN_Config2, &can_rx[2][0]);
    Can_InitOne(INST_FLEXCAN3, &FlexCAN_State3, &FlexCAN_Config3, &can_rx[3][0]);
    Can_InitOne(INST_FLEXCAN4, &FlexCAN_State4, &FlexCAN_Config4, &can_rx[4][0]);
    Can_InitOne(INST_FLEXCAN5, &FlexCAN_State5, &FlexCAN_Config5, &can_rx[5][0]);

    SetupCan_TJA1153();
}

/* ========================= IRQ HANDLERS ========================= */

void Flexcan0_0_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 0u, 7u, FALSE); }
void Flexcan0_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 8u, 31u, FALSE); }
void Flexcan0_2_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 32u, 63u, FALSE); }
void Flexcan0_3_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 64u, 95u, FALSE); }
void Flexcan1_0_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN1, 0u, 7u, FALSE); }
void Flexcan1_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN1, 8u, 31u, FALSE); }
void Flexcan1_2_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN1, 32u, 63u, FALSE); }
void Flexcan2_0_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN2, 0u, 7u, FALSE); }
void Flexcan2_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN2, 8u, 15u, FALSE); }
void Flexcan3_0_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN3, 0u, 7u, FALSE); }
void Flexcan3_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN3, 8u, 31u, FALSE); }
void Flexcan4_0_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN4, 0u, 7u, FALSE); }
void Flexcan4_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN4, 8u, 15u, FALSE); }
void Flexcan5_0_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN5, 0u, 7u, FALSE); }
void Flexcan5_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN5, 8u, 15u, FALSE); }

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
    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE))
    {
        if ((instance == INST_FLEXCAN0) ||
            (instance == INST_FLEXCAN1) ||
            (instance == INST_FLEXCAN2))
        {
            can_rx_flag[instance] = TRUE;
            CanCache_UpdateFromISR(instance, &can_rx[instance][buffIdx]);
            (void)FlexCAN_Ip_Receive(instance, buffIdx, &can_rx[instance][buffIdx], FALSE);
        }
    }
}

void FlexcanPC_callback(uint8_t instance,
                        Flexcan_Ip_EventType eventType,
                        uint32_t buffIdx,
                        const Flexcan_Ip_StateType *state)
{
    (void)state;

    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE))
    {
        if (instance == INST_FLEXCAN3)
        {
            can_rx_flag[instance] = TRUE;
            CanCache_UpdateFromISR(instance, &can_rx[instance][buffIdx]);
            (void)FlexCAN_Ip_Receive(instance, buffIdx, &can_rx[instance][buffIdx], FALSE);
        }
    }
}

void EPS_callback(uint8_t instance,
                  Flexcan_Ip_EventType eventType,
                  uint32_t buffIdx,
                  const Flexcan_Ip_StateType *state)
{
    (void)state;

    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE))
    {
        if ((instance == INST_FLEXCAN4) || (instance == INST_FLEXCAN5))
        {
            can_rx_flag[instance] = TRUE;
            if (instance == INST_FLEXCAN4)
            {
                ForwardFrame(instance, INST_FLEXCAN5, &can_rx[instance][buffIdx]);
            }
            else
            {
                ForwardFrame(instance, INST_FLEXCAN4, &can_rx[instance][buffIdx]);
            }

            (void)FlexCAN_Ip_Receive(instance, buffIdx, &can_rx[instance][buffIdx], FALSE);
        }
    }
}

void Flexcan_error_callback(uint8_t instance, Flexcan_Ip_EventType eventType, uint32_t u32ErrStatus, const Flexcan_Ip_StateType *state)
{
    (void)state;

    switch(eventType)
    {
        case FLEXCAN_EVENT_BUSOFF:
            gStats.canBusOff[instance]++;
            can_rx_flag[instance] = FALSE;

            if(FlexCAN_Ip_SetStartMode(instance) == FLEXCAN_STATUS_SUCCESS)
            {
                FlexCAN_Ip_EnableInterrupts_Privileged(instance);
            }
            else if(instance == INST_FLEXCAN4 || instance == INST_FLEXCAN5)
            { 
                Fatal_Error("EPS CAN Bus Off");
            }
            else if(instance == INST_FLEXCAN3)
            {
                Fatal_Error("PC CAN Bus Off");
            }
            else if(instance == INST_FLEXCAN0 || instance == INST_FLEXCAN1 || instance == INST_FLEXCAN2)
            {
                Fatal_Error("Car CAN Bus Off");
            }
            break;

        case FLEXCAN_EVENT_ERROR:
        case FLEXCAN_EVENT_ERROR_FAST:
            gStats.canErrors[instance]++;

            if(u32ErrStatus & (1U << 14)) gStats.canCrcErrors[instance]++;
            if(u32ErrStatus & (1U << 13)) gStats.canFormErrors[instance]++;
            if(u32ErrStatus & (1U << 12)) gStats.canStuffErrors[instance]++;
            break;

        case FLEXCAN_EVENT_RX_WARNING:
            gStats.canRxWarnings[instance]++;
            break;

        case FLEXCAN_EVENT_TX_WARNING:
            gStats.canTxWarnings[instance]++;
            break;

        case FLEXCAN_EVENT_RXFIFO_OVERFLOW:
        case FLEXCAN_EVENT_ENHANCED_RXFIFO_OVERFLOW:
            gStats.canRxFifoOverflow[instance]++;
            break;

        default:
            gStats.canUnknownErrors[instance]++;
            break;
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    Fatal_Error("Stack overflow");
}

void vApplicationMallocFailedHook(void)
{
    Fatal_Error("Malloc Failed");
}


/* ========================= MAIN ======================== */

int main(void)
{
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
                       g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);
    if (Clock_Ip_Init(&Clock_Ip_aClockConfig[0]) != CLOCK_IP_SUCCESS)
    {
        Fatal_Error("Clock init fail");
    }
    OsIf_Init(NULL_PTR);

    IntCtrl_Ip_Init(&IntCtrlConfig_0);
    Lpuart_Uart_Ip_Init(INST_UART2, &Lpuart_Uart_Ip_xHwConfigPB_2);
    Lpi2c_Ip_MasterInit(INST_I2C, &I2c_Lpi2cMaster_HwChannel0_Channel0);

    ssd1306_draw_text(0u, 0u, "Booting");
    Transceivers_Enable();
    Can_Init();
    if (xTaskCreate(StaticDSUTask, "Static_DSU", 512u, NULL, 4u, NULL) != pdPASS)
    {
        Fatal_Error("DSU task fail");
    }

    if (xTaskCreate(CacheMonitorTask, "CacheMon", 256u, NULL, 4u, NULL) != pdPASS)
    {
        Fatal_Error("CacheMon fail");
    }

    if (xTaskCreate(DecoderTask, "Decoder", 2048u, NULL, 4u, NULL) != pdPASS)
    {
        Fatal_Error("Decoder fail");
    }

    if(xTaskCreate(ControlTask, "Control", 2048u, NULL, 4u, NULL)  != pdPASS){
        Fatal_Error("Control task fail");
    }
    ssd1306_draw_text(0u, 0u, "Initialized");
    vTaskStartScheduler();

    Fatal_Error("Scheduler stopped");
    return exit_code;
}
