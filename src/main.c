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

#include "Lpuart_Uart_Ip.h"
#include "Lpuart_Uart_Ip_Cfg.h"
#include "Lpuart_Uart_Ip_Sa_PBcfg.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define INST_FLEXCAN0     (0u)
#define INST_FLEXCAN5     (5u)
#define UART2_INST        (2u)

#define CAN_TX_MB         (8u)
#define CAN_RX_MB_STD     (1u)
#define CAN_RX_MB_EXT     (2u)

#define PRIUS_2017_SPEED_MSGID (0x0B4u)
#define PRIUS_2017_GEAR_MSGID  (0x127u)
static const uint8 PRIUS_2017_SPEED_MSG_DATA[8] = {0x30, 0x00, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
static const uint8 PRIUS_2017_GEAR_MSG_DATA[8] = {0x07, 0xa0, 0x1f, 0x00, 0x08, 0x00, 0x10, 0x00};


#define TJA1153_START_ID  (0x555u)
#define TJA1153_CONFIG_ID (0x18DA00F1u)
#define FLEXCAN_CS_RTR_MASK      (0x00100000UL)
#define FLEXCAN_CS_IDE_MASK      (0x00200000UL)

#define FLEXCAN_CS_DLC_SHIFT     (16U)
#define FLEXCAN_CS_DLC_MASK      (0x000F0000UL)

/* ---------------- Globals ---------------- */
static Flexcan_Ip_MsgBuffType g_rx0_std;
static Flexcan_Ip_MsgBuffType g_rx0_ext;
static Flexcan_Ip_MsgBuffType g_rx5_std;
static Flexcan_Ip_MsgBuffType g_rx5_ext;

static volatile uint8 g_rx0_std_flag = 0u;
static volatile uint8 g_rx0_ext_flag = 0u;
static volatile uint8 g_rx5_std_flag = 0u;
static volatile uint8 g_rx5_ext_flag = 0u;

volatile int exit_code = 0;

/* ---------------- UART helper ---------------- */
static void Uart2_Print(const char *s)
{
    (void)Lpuart_Uart_Ip_SyncSend(UART2_INST, (const uint8 *)s, (uint32)strlen(s), 100000u);
}

static void PrintFrame(uint8 inst, uint32 mb, const Flexcan_Ip_MsgBuffType *m)
{
    char line[256];
    int n = 0;
    const boolean isExt = ((m->cs & FLEXCAN_CS_IDE_MASK) != 0u) ? TRUE : FALSE;
    const boolean isRtr = ((m->cs & FLEXCAN_CS_RTR_MASK) != 0u) ? TRUE : FALSE;
    uint8 dlc = (uint8)((m->cs & FLEXCAN_CS_DLC_MASK) >> FLEXCAN_CS_DLC_SHIFT);
    uint32 id = m->msgId & (isExt ? 0x1FFFFFFFul : 0x7FFul);

    n += snprintf(line + n, sizeof(line) - n,
                      "CAN%u MB%lu %s %s ID=0x%lX CS=0x%08lX DLC=%u DATA=",
                      (unsigned)inst,
                      (unsigned long)mb,
                      isExt ? "EXT" : "STD",
                      isRtr ? "RTR" : "DAT",
                      (unsigned long)id,
                      (unsigned long)m->cs,
                      (unsigned)dlc);

    for (uint8 i = 0; i < dlc; i++)
        {
            n += snprintf(line + n, sizeof(line) - n, "%02X ", m->data[i]);
        }
        n += snprintf(line + n, sizeof(line) - n, "\r\n");

        Uart2_Print(line);
}

void Flexcan0_1_handler(void)
{
    /* ORED MB 0..31 */
    FlexCAN_IRQHandler(INST_FLEXCAN0, 0u, 31u, FALSE);
}

void Flexcan5_1_handler(void)
{
    FlexCAN_IRQHandler(INST_FLEXCAN5, 0u, 31u, FALSE);
}

void Flexcan0_0_handler(void)
{
    /* error/busoff line (if enabled) */
    FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN0);
}

void Flexcan5_0_handler(void)
{
    FlexCAN_Busoff_Error_IRQHandler(INST_FLEXCAN5);
}

void Uart2_handler(void)
{
    Lpuart_Uart_Ip_IrqHandler(UART2_INST);
}

/* ---------------- FlexCAN callbacks ---------------- */
void Flexcan0_Callback(uint8 instance, Flexcan_Ip_EventType eventType, uint32 buffIdx,
                       const Flexcan_Ip_StateType *state)
{
    (void)state;
    (void)instance;

    if (eventType == FLEXCAN_EVENT_RX_COMPLETE)
    {
        if (buffIdx == CAN_RX_MB_STD) { g_rx0_std_flag = 1u; }
        if (buffIdx == CAN_RX_MB_EXT) { g_rx0_ext_flag = 1u; }
    }
}

void Flexcan5_Callback(uint8 instance, Flexcan_Ip_EventType eventType, uint32 buffIdx,
                       const Flexcan_Ip_StateType *state)
{
    (void)state;
    (void)instance;

    if (eventType == FLEXCAN_EVENT_RX_COMPLETE)
    {
        if (buffIdx == CAN_RX_MB_STD) { g_rx5_std_flag = 1u; }
        if (buffIdx == CAN_RX_MB_EXT) { g_rx5_ext_flag = 1u; }
    }
}

static void Transceivers_Enable(void)
{
    Siul2_Dio_Ip_WritePin(CAN0_En_PORT, CAN0_En_PIN, STD_HIGH);
    Siul2_Dio_Ip_WritePin(CAN5_En_PORT, CAN5_En_PIN, STD_HIGH);

    /* Put CAN0 normal immediately */
    Siul2_Dio_Ip_WritePin(CAN0_Stb_PORT, CAN0_Stb_PIN, STD_HIGH);

    /* Put CAN5 into LOCAL CONFIG first (required if TJA1153 unconfigured) */
    Siul2_Dio_Ip_WritePin(CAN5_Stb_PORT, CAN5_Stb_PIN, STD_LOW);
}

/* ---------------- Wait for start mode ---------------- */
static void WaitStartMode(uint8 instance)
{
    uint32 timeout = 200000u;
    while ((FlexCAN_Ip_GetStartMode(instance) == FALSE) && (timeout > 0u))
    {
        timeout--;
    }
}

static void SetupCan5_TJA1153(void)
{
    Flexcan_Ip_DataInfoType txi;
    uint8 d[8] = {0};

    /* Use SendBlocking; polling is OK for blocking calls */
    txi.msg_id_type = FLEXCAN_MSG_ID_STD;
    txi.data_length = 8u;
    txi.fd_enable   = FALSE;
    txi.fd_padding  = FALSE;
    txi.enable_brs  = FALSE;
    txi.is_polling  = TRUE;
    txi.is_remote   = FALSE;

    /* 1) Start frame for auto bitrate */
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_START_ID, d, 1000u);

    /* 2) Filter element 0: mask 0x1FFFFFFF */
    d[0]=0x10; d[1]=0x00; d[2]=0x9F; d[3]=0xFF; d[4]=0xFF; d[5]=0xFF;
    txi.msg_id_type = FLEXCAN_MSG_ID_EXT;
    txi.data_length = 6u;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    /* 3) Filter element 1: EXT ID=0 applied to TPL */
    d[0]=0x10; d[1]=0x01; d[2]=0xC0; d[3]=0x00; d[4]=0x00; d[5]=0x00;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    /* 4) Filter element 2: STD ID=0 mask 0x7FF applied to TPL */
    d[0]=0x10; d[1]=0x02; d[2]=0x50; d[3]=0x00; d[4]=0x07; d[5]=0xFF;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    /* 5) Exit config mode (dev mode, no NVM write, no lock) */
    d[0]=0x71; d[1]=0x02; d[2]=0x03; d[3]=0x04; d[4]=0x05; d[5]=0x06; d[6]=0x07; d[7]=0x08;
    txi.data_length = 8u;
    (void)FlexCAN_Ip_SendBlocking(INST_FLEXCAN5, CAN_TX_MB, &txi, TJA1153_CONFIG_ID, d, 1000u);

    /* After this, transceiver goes standby; set STB HIGH (negated) => NORMAL */
    Siul2_Dio_Ip_WritePin(CAN5_Stb_PORT, CAN5_Stb_PIN, STD_HIGH);
}

static void ForwardFrame(uint8 fromInst, uint8 toInst, const Flexcan_Ip_MsgBuffType *m){
	Flexcan_Ip_DataInfoType txi;
	uint8 payload[64];
	const boolean isExt = ((m->cs & FLEXCAN_CS_IDE_MASK) != 0u) ? TRUE : FALSE;
	const boolean isRtr = ((m->cs & FLEXCAN_CS_RTR_MASK) != 0u) ? TRUE : FALSE;
	uint8 dlc = (uint8)((m->cs & FLEXCAN_CS_DLC_MASK) >> FLEXCAN_CS_DLC_SHIFT);
	uint32 id = m->msgId & (isExt ? 0x1FFFFFFFul : 0x7FFul);
	if(dlc > 64u){
		dlc = 64u;
	}
	(void)memcpy(payload, m->data, dlc);
	if((fromInst == INST_FLEXCAN0) && (toInst == INST_FLEXCAN5) && (isExt == FALSE) && (isRtr == FALSE)){
		if(id == PRIUS_2017_SPEED_MSGID){
			(void)memcpy(payload, PRIUS_2017_SPEED_MSG_DATA, 8u);
			dlc = 8u;
		}
		else if(id == PRIUS_2017_GEAR_MSGID){
			(void)memcpy(payload, PRIUS_2017_GEAR_MSG_DATA, 8u);
			dlc = 8u;
		}
	}
	txi.msg_id_type = isExt ? FLEXCAN_MSG_ID_EXT : FLEXCAN_MSG_ID_STD;
	txi.data_length = dlc;
	txi.fd_enable = FALSE;
	txi.fd_padding = FALSE;
	txi.enable_brs = FALSE;
	txi.is_polling = TRUE;
	txi.is_remote = isRtr;
	(void)FlexCAN_Ip_SendBlocking(toInst, CAN_TX_MB, &txi, id, payload, 5u);
}

static void Can_Init(void)
{
    Flexcan_Ip_DataInfoType rxStd;
    Flexcan_Ip_DataInfoType rxExt;

    rxStd.msg_id_type = FLEXCAN_MSG_ID_STD;
    rxStd.data_length = 8u;
    rxStd.fd_enable   = FALSE;
    rxStd.fd_padding  = FALSE;
    rxStd.enable_brs  = FALSE;
    rxStd.is_polling  = FALSE; /* INTERRUPT MODE */
    rxStd.is_remote   = FALSE;

    rxExt = rxStd;
    rxExt.msg_id_type = FLEXCAN_MSG_ID_EXT;

    /* ------------ CAN0 (TJA1443) ------------ */
    (void)FlexCAN_Ip_Init(INST_FLEXCAN0, &FlexCAN_State0, &FlexCAN_Config0);

    /* Accept all (individual masks) */
    (void)FlexCAN_Ip_EnterFreezeMode(INST_FLEXCAN0);
    (void)FlexCAN_Ip_SetRxMaskType(INST_FLEXCAN0, FLEXCAN_RX_MASK_INDIVIDUAL);
    (void)FlexCAN_Ip_SetRxIndividualMask(INST_FLEXCAN0, CAN_RX_MB_STD, 0x00000000u);
    (void)FlexCAN_Ip_SetRxIndividualMask(INST_FLEXCAN0, CAN_RX_MB_EXT, 0x00000000u);
    (void)FlexCAN_Ip_ExitFreezeMode(INST_FLEXCAN0);

    (void)FlexCAN_Ip_SetStartMode(INST_FLEXCAN0);
    WaitStartMode(INST_FLEXCAN0);

    (void)FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN0, CAN_RX_MB_STD, &rxStd, 0x000u);
    (void)FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN0, CAN_RX_MB_EXT, &rxExt, 0x00000000u);

    (void)FlexCAN_Ip_Receive(INST_FLEXCAN0, CAN_RX_MB_STD, &g_rx0_std, FALSE);
    (void)FlexCAN_Ip_Receive(INST_FLEXCAN0, CAN_RX_MB_EXT, &g_rx0_ext, FALSE);

    FlexCAN_Ip_EnableInterrupts_Privileged(INST_FLEXCAN0);

    /* ------------ CAN5 (TJA1153) ------------ */
    (void)FlexCAN_Ip_Init(INST_FLEXCAN5, &FlexCAN_State1, &FlexCAN_Config5);

    (void)FlexCAN_Ip_EnterFreezeMode(INST_FLEXCAN5);
    (void)FlexCAN_Ip_SetRxMaskType(INST_FLEXCAN5, FLEXCAN_RX_MASK_INDIVIDUAL);
    (void)FlexCAN_Ip_SetRxIndividualMask(INST_FLEXCAN5, CAN_RX_MB_STD, 0x00000000u);
    (void)FlexCAN_Ip_SetRxIndividualMask(INST_FLEXCAN5, CAN_RX_MB_EXT, 0x00000000u);
    (void)FlexCAN_Ip_ExitFreezeMode(INST_FLEXCAN5);

    (void)FlexCAN_Ip_SetStartMode(INST_FLEXCAN5);
    WaitStartMode(INST_FLEXCAN5);

    /* IMPORTANT: unlock/configure secure transceiver before expecting bus RX */
    SetupCan5_TJA1153();

    (void)FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN5, CAN_RX_MB_STD, &rxStd, 0x000u);
    (void)FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN5, CAN_RX_MB_EXT, &rxExt, 0x00000000u);

    (void)FlexCAN_Ip_Receive(INST_FLEXCAN5, CAN_RX_MB_STD, &g_rx5_std, FALSE);
    (void)FlexCAN_Ip_Receive(INST_FLEXCAN5, CAN_RX_MB_EXT, &g_rx5_ext, FALSE);

    FlexCAN_Ip_EnableInterrupts_Privileged(INST_FLEXCAN5);
}

int main(void)
{
    Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);

    /* Pin mux init */
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
                       g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    /* Interrupt controller init (uses your generated IntCtrlConfig_0) */
    IntCtrl_Ip_Init(&IntCtrlConfig_0);

    /* UART2 */
    (void)Lpuart_Uart_Ip_Init(UART2_INST, &Lpuart_Uart_Ip_xHwConfigPB_2);
    Uart2_Print("\r\nBoot OK\r\n");

    /* Transceivers: enable, CAN5 -> local-config first */
    Transceivers_Enable();

    /* CAN init */
    Can_Init();

    Uart2_Print("Listening on CAN0 and CAN5 (MB1=STD, MB2=EXT)...\r\n");

    for (;;)
    {
        if (g_rx0_std_flag) {
            g_rx0_std_flag = 0u;
            PrintFrame(INST_FLEXCAN0, CAN_RX_MB_STD, &g_rx0_std);
            ForwardFrame(INST_FLEXCAN0, INST_FLEXCAN5, &g_rx0_std);
            (void)FlexCAN_Ip_Receive(INST_FLEXCAN0, CAN_RX_MB_STD, &g_rx0_std, FALSE);
        }
        if (g_rx0_ext_flag) {
            g_rx0_ext_flag = 0u;
            PrintFrame(INST_FLEXCAN0, CAN_RX_MB_EXT, &g_rx0_ext);
            ForwardFrame(INST_FLEXCAN0, INST_FLEXCAN5, &g_rx0_ext);
            (void)FlexCAN_Ip_Receive(INST_FLEXCAN0, CAN_RX_MB_EXT, &g_rx0_ext, FALSE);
        }
        if (g_rx5_std_flag) {
            g_rx5_std_flag = 0u;
            PrintFrame(INST_FLEXCAN5, CAN_RX_MB_STD, &g_rx5_std);
            ForwardFrame(INST_FLEXCAN5, INST_FLEXCAN0, &g_rx5_std);
            (void)FlexCAN_Ip_Receive(INST_FLEXCAN5, CAN_RX_MB_STD, &g_rx5_std, FALSE);
        }
        if (g_rx5_ext_flag) {
            g_rx5_ext_flag = 0u;
            PrintFrame(INST_FLEXCAN5, CAN_RX_MB_EXT, &g_rx5_ext);
            ForwardFrame(INST_FLEXCAN5, INST_FLEXCAN0, &g_rx5_ext);
            (void)FlexCAN_Ip_Receive(INST_FLEXCAN5, CAN_RX_MB_EXT, &g_rx5_ext, FALSE);
        }

        if (exit_code != 0) { break; }
    }

    return exit_code;
}
