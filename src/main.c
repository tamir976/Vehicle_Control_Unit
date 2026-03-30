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

#define INST_FLEXCAN0 (0u)
#define INST_FLEXCAN1 (1u)
#define INST_FLEXCAN2 (2u)
#define INST_FLEXCAN3 (3u)
#define INST_FLEXCAN4 (4u)
#define INST_FLEXCAN5 (5u)
#define INST_I2C (0u)
#define INST_UART2 (2)

#define CAN_RX_MB (0u)

#define CAN_TX_MB1 (1u)
#define CAN_TX_MB2 (2u)
#define CAN_TX_MB3 (3u)
#define CAN_TX_MB4 (4u)
#define CAN_TX_MB5 (5u)
#define CAN_TX_MB6 (6u)
#define CAN_TX_MB7 (7u)
#define CAN_TX_MB8 (8u)
#define CAN_TX_MB9 (9u)
#define CAN_TX_MB10 (10u)
#define CAN_TX_MB11 (11u)
#define CAN_TX_MB12 (12u)
#define CAN_TX_MB13 (13u)
#define CAN_TX_MB14 (14u)
#define CAN_TX_MB15 (15u)
#define CAN_TX_MB16 (16u)
#define CAN_TX_MB17 (17u)
#define CAN_TX_MB18 (18u)
#define CAN_TX_MB19 (19u)
#define CAN_TX_MB20 (20u)
#define CAN_TX_MB21 (21u)
#define CAN_TX_MB22 (22u)
#define CAN_TX_MB23 (23u)
#define CAN_TX_MB24 (24u)
#define CAN_TX_MB25 (25u)
#define CAN_TX_MB26 (26u)
#define CAN_TX_MB27 (27u)
#define CAN_TX_MB28 (28u)
#define CAN_TX_MB29 (29u)
#define CAN_TX_MB30 (30u)
#define CAN_TX_MB31 (31u)

#define CAN_TX_MB_COUNT_32  (22u)
#define CAN_TX_MB_COUNT_16  (15u)
#define PRIUS_SPEED_MSGID   (0x0B4u)
#define PRIUS_GEAR_MSGID    (0x127u)
#define UART_TX_TIMEOUT_US  (50000u)
#define UART_TELEM_PERIOD_MS (100u)

static const uint8_t PRIUS_SPEED_MSG_DATA[8] = {0x30, 0x00, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
static const uint8_t PRIUS_GEAR_MSG_DATA[8]  = {0x07, 0xA0, 0x1F, 0x00, 0x08, 0x00, 0x10, 0x00};

#define TJA1153_START_ID    (0x555u)
#define TJA1153_CONFIG_ID   (0x18DA00F1u)

static volatile boolean can_rx_flag[6];
static Flexcan_Ip_MsgBuffType can_rx[6];
static uint8_t txMbNext[6] = {0u};
static CarState gCarState;

volatile int exit_code = 0;

static void Uart_SendString(const char *msg)
{
    if (msg == NULL)
    {
        return;
    }

    (void)Lpuart_Uart_Ip_SyncSend(INST_UART2,
                                  (const uint8 *)msg,
                                  (uint32)strlen(msg),
                                  UART_TX_TIMEOUT_US);
}

static uint8_t GetNextTxMb(uint8_t inst)
{
    static const uint8_t txMbList32[CAN_TX_MB_COUNT_32] =
    {
        CAN_TX_MB10, CAN_TX_MB11, CAN_TX_MB12, CAN_TX_MB13, CAN_TX_MB14, CAN_TX_MB15,
        CAN_TX_MB16, CAN_TX_MB17, CAN_TX_MB18, CAN_TX_MB19, CAN_TX_MB20, CAN_TX_MB21,
        CAN_TX_MB22, CAN_TX_MB23, CAN_TX_MB24, CAN_TX_MB25, CAN_TX_MB26, CAN_TX_MB27,
        CAN_TX_MB28, CAN_TX_MB29, CAN_TX_MB30, CAN_TX_MB31
    };

    static const uint8_t txMbList16[CAN_TX_MB_COUNT_16] =
    {
        CAN_TX_MB1, CAN_TX_MB2, CAN_TX_MB3, CAN_TX_MB4, CAN_TX_MB5,
        CAN_TX_MB6, CAN_TX_MB7, CAN_TX_MB8, CAN_TX_MB9, CAN_TX_MB10,
        CAN_TX_MB11, CAN_TX_MB12, CAN_TX_MB13, CAN_TX_MB14, CAN_TX_MB15
    };

    const uint8_t *txMbList = txMbList32;
    uint8_t txMbCount = CAN_TX_MB_COUNT_32;
    uint8_t mb;

    if ((inst == INST_FLEXCAN2) || (inst == INST_FLEXCAN4) || (inst == INST_FLEXCAN5))
    {
        txMbList = txMbList16;
        txMbCount = CAN_TX_MB_COUNT_16;
    }

    taskENTER_CRITICAL();
    mb = txMbList[txMbNext[inst] % txMbCount];
    txMbNext[inst] = (uint8_t)((txMbNext[inst] + 1u) % txMbCount);
    taskEXIT_CRITICAL();

    return mb;
}

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
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB1, &txi, TJA1153_START_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
	txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
	txi.data_length = 6u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
	txi.data_length = 8u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	Siul2_Dio_Ip_WritePin(can4_stb_PORT, can4_stb_PIN, STD_HIGH);
	/* CAN5 */
	(void)memset(d, 0, sizeof(d));
	txi.msg_id_type = FLEXCAN_MSG_ID_STD;
	txi.data_length = 8u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB1, &txi, TJA1153_START_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
	txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
	txi.data_length = 6u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
	txi.data_length = 8u;
	(void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB1, &txi, TJA1153_CONFIG_ID, d, 1000u);
	Siul2_Dio_Ip_WritePin(can5_stb_PORT, can5_stb_PIN, STD_HIGH);
}

static void ForwardFrame(uint8_t fromInst, uint8_t toInst, const Flexcan_Ip_MsgBuffType *m)
{
    Flexcan_Ip_DataInfoType txi;
    uint8_t payload[8];
    Flexcan_Ip_StatusType st;

    uint8_t dlc = m->dataLen;
    uint32_t id = m->msgId;
    (void)memcpy(payload, m->data, dlc);

    if ((fromInst == INST_FLEXCAN4) && (toInst == INST_FLEXCAN5))
    {
        if (id == PRIUS_SPEED_MSGID)
        {
            (void)memcpy(payload, PRIUS_SPEED_MSG_DATA, 8u);
            dlc = 8u;
        }
        else if (id == PRIUS_GEAR_MSGID)
        {
            (void)memcpy(payload, PRIUS_GEAR_MSG_DATA, 8u);
            dlc = 8u;
        }
    }

    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    txi.is_polling  = FALSE;
    txi.data_length = dlc;
    txi.is_remote   = FALSE;

    st = FlexCAN_Ip_Send(toInst, GetNextTxMb(toInst), &txi, id, payload);
    if (st != FLEXCAN_STATUS_SUCCESS)
    {
        gStats.canTxFails++;
    }
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

    st = FlexCAN_Ip_Init(inst, state, config);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("CAN init fail"); }

    st = FlexCAN_Ip_EnterFreezeMode(inst);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Freeze fail"); }

    (void)FlexCAN_Ip_SetRxMaskType(inst, FLEXCAN_RX_MASK_INDIVIDUAL);
    (void)FlexCAN_Ip_SetRxIndividualMask(inst, CAN_RX_MB, 0x00000000u);

    st = FlexCAN_Ip_ConfigRxMb(inst, CAN_RX_MB, &rxStd, 0x000u);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Rx MB cfg fail"); }

    st = FlexCAN_Ip_Receive(inst, CAN_RX_MB, rxMb, FALSE);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Rx start fail"); }

    st = FlexCAN_Ip_ExitFreezeMode(inst);
    if (st != FLEXCAN_STATUS_SUCCESS) { gStats.canInitFails++; Fatal_Error("Exit freeze fail"); }

    (void)FlexCAN_Ip_SetStartMode(inst);
    WaitStartMode(inst);

    FlexCAN_Ip_EnableInterrupts_Privileged(inst);
}

static void Can_Init(void)
{
    Can_InitOne(INST_FLEXCAN0, &FlexCAN_State0, &FlexCAN_Config0, &can_rx[0]);
    Can_InitOne(INST_FLEXCAN1, &FlexCAN_State1, &FlexCAN_Config1, &can_rx[1]);
    Can_InitOne(INST_FLEXCAN2, &FlexCAN_State2, &FlexCAN_Config2, &can_rx[2]);
    Can_InitOne(INST_FLEXCAN3, &FlexCAN_State3, &FlexCAN_Config3, &can_rx[3]);
    Can_InitOne(INST_FLEXCAN4, &FlexCAN_State4, &FlexCAN_Config4, &can_rx[4]);
    Can_InitOne(INST_FLEXCAN5, &FlexCAN_State5, &FlexCAN_Config5, &can_rx[5]);

    SetupCan_TJA1153();
}

/* ========================= IRQ HANDLERS ========================= */

void Flexcan0_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 0u, 32u, FALSE); }
void Flexcan1_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN1, 0u, 32u, FALSE); }
void Flexcan2_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN2, 0u, 16u, FALSE); }
void Flexcan3_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN3, 0u, 32u, FALSE); }
void Flexcan4_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN4, 0u, 16u, FALSE); }
void Flexcan5_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN5, 0u, 16u, FALSE); }

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

    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == CAN_RX_MB))
    {
        if ((instance == INST_FLEXCAN0) ||
            (instance == INST_FLEXCAN1) ||
            (instance == INST_FLEXCAN2))
        {
            can_rx_flag[instance] = TRUE;
            CanCache_UpdateFromISR(instance, &can_rx[instance]);
            (void)FlexCAN_Ip_Receive(instance, CAN_RX_MB, &can_rx[instance], FALSE);
        }
    }
}

void FlexcanPC_callback(uint8_t instance,
                        Flexcan_Ip_EventType eventType,
                        uint32_t buffIdx,
                        const Flexcan_Ip_StateType *state)
{
    (void)state;

    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == CAN_RX_MB))
    {
        if (instance == INST_FLEXCAN3)
        {
            can_rx_flag[instance] = TRUE;
            CanCache_UpdateFromISR(instance, &can_rx[instance]);
            (void)FlexCAN_Ip_Receive(instance, CAN_RX_MB, &can_rx[instance], FALSE);
        }
    }
}

void EPS_callback(uint8_t instance,
                  Flexcan_Ip_EventType eventType,
                  uint32_t buffIdx,
                  const Flexcan_Ip_StateType *state)
{
    (void)state;

    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == CAN_RX_MB))
    {
        if ((instance == INST_FLEXCAN4) || (instance == INST_FLEXCAN5))
        {
            can_rx_flag[instance] = TRUE;
            if (instance == INST_FLEXCAN4)
            {
                ForwardFrame(instance, INST_FLEXCAN5, &can_rx[instance]);
            }
            else
            {
                ForwardFrame(instance, INST_FLEXCAN4, &can_rx[instance]);
            }

            (void)FlexCAN_Ip_Receive(instance, CAN_RX_MB, &can_rx[instance], FALSE);
        }
    }
}

/* ========================= TASKS ========================= */

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

static void StaticDSUTask(void *pv)
{
    TickType_t lastWakeTime;
    const size_t num_msgs = PRIUS_DSU_MSGS_C_COUNT;
    uint32_t tick10ms = 0u;
    Flexcan_Ip_DataInfoType tx;
    uint8_t payload[8];
    (void)pv;

    lastWakeTime = xTaskGetTickCount();

    tx.is_polling = FALSE;
    tx.is_remote = FALSE;
    tx.msg_id_type = FLEXCAN_MSG_ID_STD;

    for (;;)
    {
        for (size_t i = 0u; i < num_msgs; i++)
        {
            const static_dsu_msg_t *msg = &PRIUS_DSU_MSGS_C[i];

            if ((msg->freq_100 != 0u) && ((tick10ms % msg->freq_100) == 0u))
            {
                uint8_t dlc = msg->vl_len;
                uint8_t bus = msg->bus;
                uint32_t id = msg->addr;
                Flexcan_Ip_StatusType st;

                (void)memcpy(payload, msg->vl, dlc);
                tx.data_length = dlc;

                st = FlexCAN_Ip_Send(bus, GetNextTxMb(bus), &tx, id, payload);
                if (st != FLEXCAN_STATUS_SUCCESS)
                {
                    gStats.canTxFails++;
                }
            }
        }
        tick10ms++;
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10u));
    }
}

static void CacheMonitorTask(void *pv)
{
    TickType_t lastWakeTime;
    (void)pv;

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        CanCache_RefreshValidity();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10u));
    }
}

static void DecoderTask(void *pv)
{
    (void)pv;
    CarState_clear(&gCarState);
    for(;;){
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10u));
        CarState_update(&gCarState);
    }
}

static void UartTask(void *pv)
{
    TickType_t lastWakeTime;
    CarState snap;
    char line[256];
    int n;

    (void)pv;
    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        taskENTER_CRITICAL();
        (void)memcpy(&snap, &gCarState, sizeof(snap));
        taskEXIT_CRITICAL();

        n = snprintf(line,
                     sizeof(line),
                     "v=%.2f a=%.2f steer=%.1f rpm=%.0f gas=%.1f gear=%u brk=%u abs=%u tc=%u\r\n",
                     (double)snap.out.vEgo,
                     (double)snap.out.aEgo,
                     (double)snap.out.steeringAngleDeg,
                     (double)snap.engine_rpm,
                     (double)snap.gas_pedal,
                     (unsigned int)snap.gear,
                     (unsigned int)snap.brake_pressed,
                     (unsigned int)snap.abs_active,
                     (unsigned int)snap.traction_control_active);

        if (n > 0)
        {
            Uart_SendString(line);
        }


        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(UART_TELEM_PERIOD_MS));
    }
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
    if (xTaskCreate(StaticDSUTask, "Static_DSU", 1024u, NULL, 4u, NULL) != pdPASS)
    {
        Fatal_Error("DSU task fail");
    }

    if (xTaskCreate(CacheMonitorTask, "CacheMon", 768u, NULL, 4u, NULL) != pdPASS)
    {
        Fatal_Error("CacheMon fail");
    }

    if (xTaskCreate(DecoderTask, "Decoder", 1024u, NULL, 4u, NULL) != pdPASS)
    {
        Fatal_Error("Decoder fail");
    }

    if (xTaskCreate(UartTask, "UartTelem", 1024u, NULL, 3u, NULL) != pdPASS)
    {
        Fatal_Error("UartTelem fail");
    }

    ssd1306_draw_text(0u, 0u, "Initialized");
    vTaskStartScheduler();

    Fatal_Error("Scheduler stopped");
    return exit_code;
}
