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
#include "oled_display.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "include/static_message.h"

#define INST_FLEXCAN0      (0u)
#define INST_FLEXCAN1      (1u)
#define INST_FLEXCAN2      (2u)
#define INST_FLEXCAN3      (3u)
#define INST_FLEXCAN4      (4u)
#define INST_FLEXCAN5      (5u)
#define INST_I2C           (0u)

#define CAN_RX_MB          (0u)
#define CAN_TX_MB          (1u)

#define PRIUS_SPEED_MSGID  (0x0B4u)
#define PRIUS_GEAR_MSGID   (0x127u)

static const uint8 PRIUS_SPEED_MSG_DATA[8] = {0x30, 0x00, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
static const uint8 PRIUS_GEAR_MSG_DATA[8]  = {0x07, 0xA0, 0x1F, 0x00, 0x08, 0x00, 0x10, 0x00};

#define TJA1153_START_ID   (0x555u)
#define TJA1153_CONFIG_ID  (0x18DA00F1u)
#define RX_STORE_SIZE (256u)

static uint32 err_status[6];
static volatile boolean can_rx_flag[6];
static volatile boolean can_tx_flag[6];
static volatile boolean can_intr_flag[6];
static volatile Flexcan_Ip_MbStateType can_mb_state[6];
static Flexcan_Ip_MsgBuffType can_rx_std[6];

static QueueHandle_t canRxQueue;
static QueueHandle_t can3RxQueue;
static QueueHandle_t canTxQueue;
static SemaphoreHandle_t stateMutex;

typedef struct{
	uint8_t bus;
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
	uint8_t is_extended;
} CanRxFrame;

typedef struct{
	uint8_t bus;
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
	uint8_t is_extended;
} CanTxFrame;

typedef struct{
	uint8_t valid;
	uint8_t bus;
	uint8_t dlc;
	uint8_t is_extended;
	uint32_t id;
	TickType_t tick;
	uint8_t data;
} CanStoredFrame;

static CanStoredFrame rxStore[RX_STORE_SIZE];

volatile int exit_code = 0;

/* ========================= BOARD / CAN HELPERS ========================= */

static void Transceivers_Enable(void)
{
    Siul2_Dio_Ip_WritePin(can0_en_PORT, can0_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can1_en_PORT, can1_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can2_en_PORT, can2_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can3_en_PORT, can3_en_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can5_en_PORT, can5_en_PIN, STD_HIGH);

    Siul2_Dio_Ip_WritePin(can0_stb_PORT, can0_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can1_stb_PORT, can1_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can2_stb_PORT, can2_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can3_stb_PORT, can3_stb_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(can4_stb_PORT, can4_stb_PIN, STD_LOW);
    Siul2_Dio_Ip_WritePin(can5_stb_PORT, can5_stb_PIN, STD_LOW);
}

static void WaitStartMode(uint8 instance)
{
	uint32 timeout = 20000u;

    while ((FlexCAN_Ip_GetStartMode(instance) == FALSE) && (timeout > 0u))
    {
        timeout--;
    }
}

static void SetupCan_TJA1153(void)
{
    Flexcan_Ip_DataInfoType txi;
    uint8 d[8] = {0};
    Flexcan_Ip_StatusType st;
    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    txi.data_length = 8u;
    txi.is_polling  = TRUE;
    txi.is_remote   = FALSE;

    /* CAN4 */
    st = FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB, &txi, TJA1153_START_ID, d, 1000u);
    if(st != FLEXCAN_STATUS_SUCCESS){

    }
    d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
    txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
    txi.data_length = 6u;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
    txi.data_length = 8u;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN4, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    Siul2_Dio_Ip_WritePin(can4_stb_PORT, can4_stb_PIN, STD_HIGH);

    /* CAN5 */
    memset(d, 0, sizeof(d));
    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    txi.data_length = 8u;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_START_ID, d, 1000u);

    d[0] = 0x10; d[1] = 0x00; d[2] = 0x9F; d[3] = 0xFF; d[4] = 0xFF; d[5] = 0xFF;
    txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
    txi.data_length = 6u;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    d[0] = 0x10; d[1] = 0x01; d[2] = 0xC0; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    d[0] = 0x10; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x07; d[5] = 0xFF;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    d[0] = 0x71; d[1] = 0x02; d[2] = 0x50; d[3] = 0x00; d[4] = 0x05; d[5] = 0x06; d[6] = 0x07; d[7] = 0x08;
    txi.data_length = 8u;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    Siul2_Dio_Ip_WritePin(can5_stb_PORT, can5_stb_PIN, STD_HIGH);
}

static void ForwardFrame(uint8 fromInst, uint8 toInst, const Flexcan_Ip_MsgBuffType *m)
{
    Flexcan_Ip_DataInfoType txi;
    uint8 payload[64];
    uint8 dlc = m->dataLen;
    uint32 id = m->msgId;

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
    txi.is_polling  = TRUE;
    txi.data_length = dlc;
    txi.is_remote   = FALSE;

    (void)FlexCAN_Ip_SendBlocking(toInst, CAN_TX_MB, &txi, id, payload, 1u);
}

static void Can_InitOne(uint8 inst, Flexcan_Ip_StateType *state, const Flexcan_Ip_ConfigType *config, Flexcan_Ip_MsgBuffType *rxMb)
{
    Flexcan_Ip_DataInfoType rxStd;

    rxStd.msg_id_type = FLEXCAN_MSG_ID_STD;
    rxStd.data_length = 8u;
    rxStd.is_polling  = FALSE;
    rxStd.is_remote   = FALSE;

    (void)FlexCAN_Ip_Init(inst, state, config);
    (void)FlexCAN_Ip_EnterFreezeMode(inst);
    (void)FlexCAN_Ip_SetRxMaskType(inst, FLEXCAN_RX_MASK_INDIVIDUAL);
    (void)FlexCAN_Ip_SetRxIndividualMask(inst, CAN_RX_MB, 0x00000000u);
    (void)FlexCAN_Ip_ExitFreezeMode(inst);
    (void)FlexCAN_Ip_SetStartMode(inst);
    WaitStartMode(inst);

    (void)FlexCAN_Ip_ConfigRxMb(inst, CAN_RX_MB, &rxStd, 0x000u);
    (void)FlexCAN_Ip_Receive(inst, CAN_RX_MB, rxMb, FALSE);
    FlexCAN_Ip_EnableInterrupts_Privileged(inst);
}

static void Can_Init(void)
{
    Can_InitOne(INST_FLEXCAN0, &FlexCAN_State0, &FlexCAN_Config0, &can_rx_std[0]);
    Can_InitOne(INST_FLEXCAN1, &FlexCAN_State1, &FlexCAN_Config1, &can_rx_std[1]);
    Can_InitOne(INST_FLEXCAN2, &FlexCAN_State2, &FlexCAN_Config2, &can_rx_std[2]);
    Can_InitOne(INST_FLEXCAN3, &FlexCAN_State3, &FlexCAN_Config3, &can_rx_std[3]);
    Can_InitOne(INST_FLEXCAN4, &FlexCAN_State4, &FlexCAN_Config4, &can_rx_std[4]);
    Can_InitOne(INST_FLEXCAN5, &FlexCAN_State5, &FlexCAN_Config5, &can_rx_std[5]);

    SetupCan_TJA1153();
}

/* ========================= MAIN ========================= */
void Flexcan0_0_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN0); }
void Flexcan0_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN0, 0u, 4u, FALSE); }

void Flexcan1_0_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN1); }
void Flexcan1_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN1, 0u, 4u, FALSE); }

void Flexcan2_0_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN2); }
void Flexcan2_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN2, 0u, 4u, FALSE); }

void Flexcan3_0_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN3); }
void Flexcan3_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN3, 0u, 4u, FALSE); }

void Flexcan4_0_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN4); }
void Flexcan4_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN4, 0u, 4u, FALSE); }

void Flexcan5_0_handler(void) { FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN5); }
void Flexcan5_1_handler(void) { FlexCAN_IRQHandler(INST_FLEXCAN5, 0u, 4u, FALSE); }

void I2c0_handler(void)
{
    Lpi2c_Ip_MasterIRQHandler(INST_I2C);
}

/* ========================= FLEXCAN CALLBACKS ========================= */

void Flexcan_callback(uint8 instance, Flexcan_Ip_EventType eventType, uint32 buffIdx, const Flexcan_Ip_StateType *state)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    CanRxFrame frame;

	if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == CAN_RX_MB))
    {
		if((instance == INST_FLEXCAN0) || (instance == INST_FLEXCAN1) || (instance == INST_FLEXCAN2)){
			can_rx_flag[instance] = TRUE;
			can_mb_state[instance] = state->mbs[buffIdx].state;
			can_intr_flag[instance] = state->isIntActive;
			frame.bus = instance;
			frame.id = can_rx_std[instance].msgId;
			frame.dlc = can_rx_std[instance].dataLen;
			frame.is_extended = 0u;
			(void)memcpy(frame.data, can_rx_std[instance].data, frame.dlc);
			(void)xQueueSendFromISR(canRxQueue, &frame, &xHigherPriorityTaskWoken);
			(void)FlexCAN_Ip_Receive(instance, CAN_RX_MB, &can_rx_std[instance], FALSE);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}

        if(instance == INST_FLEXCAN3){
        	can_rx_flag[instance] = TRUE;
        	can_mb_state[instance] = state->mbs[buffIdx].state;
        	can_intr_flag[instance] = state->isIntActive;
			frame.bus = instance;
			frame.id = can_rx_std[instance].msgId;
			frame.dlc = can_rx_std[instance].dataLen;
			frame.is_extended = 0u;
			(void)memcpy(frame.data, can_rx_std[instance].data, frame.dlc);
			(void)xQueueSendFromISR(can3RxQueue, &frame, &xHigherPriorityTaskWoken);
			(void)FlexCAN_Ip_Receive(instance, CAN_RX_MB, &can_rx_std[instance], FALSE);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        }
    }
    if ((eventType == FLEXCAN_EVENT_TX_COMPLETE) && (buffIdx == CAN_TX_MB)){
    	switch(instance){
    	case INST_FLEXCAN0:
    		can_tx_flag[0] = TRUE;
    		can_mb_state[0] = state->mbs[buffIdx].state;
    		can_intr_flag[0] = state->isIntActive;
    		break;
    	case INST_FLEXCAN1:
    		can_tx_flag[1] = TRUE;
    		can_mb_state[1] = state->mbs[buffIdx].state;
    		can_intr_flag[1] = state->isIntActive;
    		break;
    	case INST_FLEXCAN2:
    		can_tx_flag[2] = TRUE;
    		can_mb_state[2] = state->mbs[buffIdx].state;
    		can_intr_flag[2] = state->isIntActive;
    		break;
    	case INST_FLEXCAN3:
    		can_tx_flag[3] = TRUE;
    		can_mb_state[3] = state->mbs[buffIdx].state;
    		can_intr_flag[3] = state->isIntActive;
    		break;
    	default:
    		break;
    	}
    }
}


void Flexcan_callback_err(uint8 instance, Flexcan_Ip_EventType eventType, uint32 u32ErrStatus,
                           const Flexcan_Ip_StateType *flexcanState)
{
    (void)flexcanState;
    switch(eventType){
    case FLEXCAN_EVENT_ERROR:
    	err_status[instance] = u32ErrStatus;
    	break;
    case FLEXCAN_EVENT_BUSOFF:
    	err_status[instance] = u32ErrStatus;
    	break;
    case FLEXCAN_EVENT_RX_WARNING:
    	err_status[instance] = u32ErrStatus;
    	break;
    case FLEXCAN_EVENT_TX_WARNING:
    	err_status[instance] = u32ErrStatus;
    	break;
    default:
    	err_status[instance] = u32ErrStatus;
    	break;
    }
}
void EPS_callback(uint8 instance, Flexcan_Ip_EventType eventType, uint32 buffIdx, const Flexcan_Ip_StateType *state){
    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == CAN_RX_MB))
    {
    	if(instance == INST_FLEXCAN4){
    		can_rx_flag[4] = TRUE;
    		can_mb_state[4] = state->mbs[buffIdx].state;
    		can_intr_flag[4] = state->isIntActive;
    	}
    	else if(instance == INST_FLEXCAN5){
    		can_rx_flag[5] = TRUE;
    		can_mb_state[5] = state->mbs[buffIdx].state;
        	can_intr_flag[5] = state->isIntActive;
    	}
    }
    if ((eventType == FLEXCAN_EVENT_TX_COMPLETE) && (buffIdx == CAN_TX_MB)){
    	if(instance == INST_FLEXCAN4){
    		can_tx_flag[4] = TRUE;
    		can_mb_state[4] = state->mbs[buffIdx].state;
    		can_intr_flag[4] = state->isIntActive;
    	}
    	else if(instance == INST_FLEXCAN5){
    		can_tx_flag[5] = TRUE;
    		can_mb_state[5] = state->mbs[buffIdx].state;
    		can_intr_flag[5] = state->isIntActive;
    	}
    }
}

void EPS_callback_err(uint8 instance, Flexcan_Ip_EventType eventType, uint32 u32ErrStatus, const Flexcan_Ip_StateType *flexcanState){
	(void)flexcanState;
	    switch(eventType){
	    case FLEXCAN_EVENT_ERROR:
	    	err_status[instance] = u32ErrStatus;
	    	break;
	    case FLEXCAN_EVENT_BUSOFF:
	    	err_status[instance] = u32ErrStatus;
	    	break;
	    case FLEXCAN_EVENT_RX_WARNING:
	    	err_status[instance] = u32ErrStatus;
	    	break;
	    case FLEXCAN_EVENT_TX_WARNING:
	    	err_status[instance] = u32ErrStatus;
	    	break;
	    default:
	    	err_status[instance] = u32ErrStatus;
	    	break;
	    }
}
static int32_t FindFrameIndex(uint8_t bus, uint32_t id){
	uint32_t i;
	for(i = 0u; i<RX_STORE_SIZE; i++){
		if((rxStore[i].valid != 0u) &&
			(rxStore[i].bus == bus) &&
			(rxStore[i].id == id)
		){
			return i;
		}
	}
	return -1;
}
static int32_t FindFreeFrameIndex(void){
	uint32_t i;
	for(i = 0u; i < RX_STORE_SIZE; i++){
		if(rxStore[i].valid == 0u){
			return i;
		}
	}
	return -1;
}
static StoreRxFrame(const CanRxFrame *frame){
	int32_t idx;
	idx = FindFrameIndex(frame->bus, frame->id);
	if(idx < 0){
		idx = FindFreeFrameIndex();
	}else{
		rxStore[idx].valid = 1u;
		rxStore[idx].bus = frame->bus;
		rxStore[idx].dlc = frame->dlc;
		rxStore[idx].id = frame->id;
		rxStore[idx].is_extended = frame->is_extended;
		(void)memcpy(rxStore[idx].data, frame->data, frame->dlc);
		rxStore[idx].tick = xTaskGetTickCount();
	}
}
static void statusTask(void *pv)
{
    uint8_t y;
    uint8_t i;
    uint8_t firstErr;
    char line[22];
    (void)pv;

    for (;;)
    {
        y = 0u;
        firstErr = 0xFFu;

        ssd1306_clear();

        ssd1306_draw_text(0u, y, "CAN STATUS");
        y += 8u;

        (void)snprintf(line, sizeof(line), "RX %u%u%u%u%u%u",
                       can_rx_flag[0] ? 1u : 0u,
                       can_rx_flag[1] ? 1u : 0u,
                       can_rx_flag[2] ? 1u : 0u,
                       can_rx_flag[3] ? 1u : 0u,
                       can_rx_flag[4] ? 1u : 0u,
                       can_rx_flag[5] ? 1u : 0u);
        ssd1306_draw_text(0u, y, line);
        y += 8u;

        (void)snprintf(line, sizeof(line), "TX %u%u%u%u%u%u",
                       can_tx_flag[0] ? 1u : 0u,
                       can_tx_flag[1] ? 1u : 0u,
                       can_tx_flag[2] ? 1u : 0u,
                       can_tx_flag[3] ? 1u : 0u,
                       can_tx_flag[4] ? 1u : 0u,
                       can_tx_flag[5] ? 1u : 0u);
        ssd1306_draw_text(0u, y, line);
        y += 8u;

        for (i = 0u; i < 6u; i++)
        {
            if (err_status[i] != 0u)
            {
                firstErr = i;
                break;
            }
        }

        if (firstErr != 0xFFu)
        {
            (void)snprintf(line, sizeof(line), "E%u %08lX",
                           firstErr,
                           (unsigned long)err_status[firstErr]);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "No errors");
        }

        ssd1306_draw_text(0u, y, line);
        ssd1306_update();

        vTaskDelay(pdMS_TO_TICKS(200u));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName){
	(void)xTask;
	(void)pcTaskName;

	taskDISABLE_INTERRUPTS();
	for(;;){

	}
}
void vApplicationMallocFailedHook(void){
	taskDISABLE_INTERRUPTS();
	for(;;){

	}
}
static void canEPSTask(void *pv){
	(void)pv;
	for(;;){
		if (can_rx_flag[4] != FALSE)
		        {
		            can_rx_flag[4] = FALSE;
		            ForwardFrame(INST_FLEXCAN4, INST_FLEXCAN5, &can_rx_std[4]);
		            (void)FlexCAN_Ip_Receive(INST_FLEXCAN4, CAN_RX_MB, &can_rx_std[4], FALSE);
		        }
		        else if (can_rx_flag[5] != FALSE)
		        {
		            can_rx_flag[5] = FALSE;
		            ForwardFrame(INST_FLEXCAN5, INST_FLEXCAN4, &can_rx_std[5]);
		            (void)FlexCAN_Ip_Receive(INST_FLEXCAN5, CAN_RX_MB, &can_rx_std[5], FALSE);
		        }else{
		        	vTaskDelay(pdMS_TO_TICKS(1u));
		        }
	}
}
static void CanRxTask(void *pv){
	CanRxFrame frame;
	(void)pv;
	for(;;){
		if(xQueueReceive(canRxQueue, &frame, portMAX_DELAY) == pdPASS){
			if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdPASS){
				StoreRxFrame(&frame);
				xSemaphoreGive(stateMutex);

			}

		}
	}
}
static void Can3RxTask(void *pv){
	CanRxFrame frame;
	(void)pv;
	for(;;){
		if(xQueueReceive(can3RxQueue, &frame, portMAX_DELAY) == pdPASS){
			if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdPASS){
				StoreRxFrame(&frame);
				xSemaphoreGive(stateMutex);
			}
		}
	}
}

static void CanTxTask(void *pv){
	CanTxFrame tx;
	Flexcan_Ip_DataInfoType txInfo;
	(void)pv;

	txInfo.is_polling = TRUE;
	txInfo.is_remote = FALSE;
	txInfo.msg_id_type = FLEXCAN_MSG_ID_STD;
	for(;;){
		if(xQueueReceive(canTxQueue, &tx, portMAX_DELAY) == pdPASS){
			txInfo.data_length = tx.dlc;
			(void)FlexCAN_Ip_SendBlocking(tx.bus, CAN_TX_MB, &txInfo, tx.id, tx.data, 10u);
		}
	}
}

static void staticDSUTask(void *pv) {
	uint32_t frame = 0u;
	TickType_t lastWakeTime;
	size_t num_msgs;
	size_t i;
	(void)pv;
	lastWakeTime = xTaskGetTickCount();
	num_msgs = sizeof(PRIUS_DSU_MSGS_C) / sizeof(PRIUS_DSU_MSGS_C[0]);
	for(;;){
		for(i = 0u; i<num_msgs; i++){
			const static_dsu_msg_t *msg = &PRIUS_DSU_MSGS_C[i];
			CanTxFrame tx;
			if((msg->freq_100 != 0) && ((frame % msg->freq_100) == 0u)){
				tx.bus = msg->bus;
				tx.dlc = msg->vl_len;
				tx.id = msg->addr;
				tx.is_extended = 0u;
				(void)memcpy(tx.data, msg->vl, msg->vl_len);
				if((msg->freq_100 == 5u) &&
				   (msg->bus = 1u) &&
				   ((msg->addr == 0x240u) || (msg->addr == 0x241u) || (msg->addr == 0x244u) || (msg->addr == 0x245u) || (msg->addr == 0x248u)
				)){
					uint8_t cnt = (uint8_t)((((frame / 5u) %7u) + 1u) << 5);
					uint8_t j;
					for(j = tx.dlc; j > 0u; j--){
						tx.data[j] = tx.data[j-1u];
					}
					tx.data[0] = cnt;
					tx.dlc++;
				}
				else if(((msg->addr == 0x489) || (msg->addr == 0x48Au)) || (msg->bus == 0u)){
					uint8_t cnt =(uint8_t)(((frame / 100u) % 0x0Fu) + 1u);
					if(msg->addr == 0x48Au){
						cnt = (uint8_t)(cnt | 0x80u);
					}
					if(tx.dlc < 8u){
						tx.data[tx.dlc] = cnt;
						tx.dlc++;
					}
				}
				(void)xQueueSend(canTxQueue, &tx, 0u);
			}
		}
		frame++;
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10u));
	}
}
int main(void)
{

    Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
                       g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);
    IntCtrl_Ip_Init(&IntCtrlConfig_0);

    (void)Lpi2c_Ip_MasterInit(INST_I2C, &I2c_Lpi2cMaster_HwChannel0_Channel0);

    Transceivers_Enable();
    Can_Init();

    ssd1306_init();
    ssd1306_clear();
    ssd1306_draw_text(0u, 0u, "Initialized");
    ssd1306_update();
    ssd1306_on();

    canRxQueue = xQueueCreate(32u, sizeof(CanRxFrame));
    can3RxQueue = xQueueCreate(32u, sizeof(CanRxFrame));
    canTxQueue = xQueueCreate(16u, sizeof(CanTxFrame));
    stateMutex = xSemaphoreCreateMutex();
    if((canRxQueue == NULL) || (can3RxQueue == NULL) || (canTxQueue == NULL) || (stateMutex == NULL)){
    	for(;;){

    	}
    }
    (void)xTaskCreate(canRxTask, "CAN_RX", 512u, NULL, 4u, NULL);
    (void)xTaskCreate(can3RxTask, "CAN3_RX", 512u, NULL, 4u, NULL);
    (void)xTaskCreate(canTxTask, "CAN_TX", 512u, NULL, 3u, NULL);
    (void)xTaskCreate(canEPSTask, "CAN4_RX_TX", 512u, NULL, 4u, NULL);
    (void)xTaskCreate(statusTask, "Status", 512u, NULL, 5u, NULL);
    (void)xTaskCreate(staticDSUTask, "Static_DSU", 768u, NULL, 4u, NULL);
    vTaskStartScheduler();
    for (;;)
    {
        if (exit_code != 0)
        {
            break;
        }
    }

    return exit_code;
}
