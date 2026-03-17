/*==================================================================================================
*   Project              : RTD AUTOSAR 4.9 
*   Platform             : CORTEXM
*   Peripheral           : SIUL2
*   Dependencies         : none
*
*   Autosar Version      : 4.9.0
*   Autosar Revision     : ASR_REL_4_9_REV_0000
*   Autosar Conf.Variant :
*   SW Version           : 7.0.0
*   Build Version        : S32K3_S32M27x_Real-Time_Drivers_AUTOSAR_R23-11_Version_7_0_0_D2510_ASR_REL_4_9_REV_0000_20251031
*
*   Copyright 2020 - 2025 NXP
*
*   NXP Confidential. This software is owned or controlled by NXP and may only be
*   used strictly in accordance with the applicable license terms. By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms. If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

/**
*   @file      Tspc_Port_Ip_Cfg.h
*
*   @addtogroup Port_CFG
*   @{
*/

#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
                                         INCLUDE FILES
 1) system and project includes
 2) needed interfaces from external units
 3) internal and external interfaces from this unit
==================================================================================================*/
#include "Tspc_Port_Ip_Cfg.h"

/*==================================================================================================
*                              SOURCE FILE VERSION INFORMATION
==================================================================================================*/
#define TSPC_PORT_IP_VENDOR_ID_CFG_C                       43
#define TSPC_PORT_IP_AR_RELEASE_MAJOR_VERSION_CFG_C        4
#define TSPC_PORT_IP_AR_RELEASE_MINOR_VERSION_CFG_C        9
#define TSPC_PORT_IP_AR_RELEASE_REVISION_VERSION_CFG_C     0
#define TSPC_PORT_IP_SW_MAJOR_VERSION_CFG_C                7
#define TSPC_PORT_IP_SW_MINOR_VERSION_CFG_C                0
#define TSPC_PORT_IP_SW_PATCH_VERSION_CFG_C                0

/*==================================================================================================
*                                     FILE VERSION CHECKS
==================================================================================================*/
/* Check if Tspc_Port_Ip_Cfg.c and Tspc_Port_Ip_Cfg.h are of the same vendor */
#if (TSPC_PORT_IP_VENDOR_ID_CFG_C != TSPC_PORT_IP_VENDOR_ID_CFG_H)
    #error "Tspc_Port_Ip_Cfg.c and Tspc_Port_Ip_Cfg.h have different vendor ids"
#endif
/* Check if Tspc_Port_Ip_Cfg.c and Tspc_Port_Ip_Cfg.h are of the same Autosar version */
#if ((TSPC_PORT_IP_AR_RELEASE_MAJOR_VERSION_CFG_C    != TSPC_PORT_IP_AR_RELEASE_MAJOR_VERSION_CFG_H) || \
    (TSPC_PORT_IP_AR_RELEASE_MINOR_VERSION_CFG_C    != TSPC_PORT_IP_AR_RELEASE_MINOR_VERSION_CFG_H) || \
    (TSPC_PORT_IP_AR_RELEASE_REVISION_VERSION_CFG_C != TSPC_PORT_IP_AR_RELEASE_REVISION_VERSION_CFG_H) \
    )
    #error "AutoSar Version Numbers of Tspc_Port_Ip_Cfg.c and Tspc_Port_Ip_Cfg.h are different"
#endif
/* Check if Tspc_Port_Ip_Cfg.c and Tspc_Port_Ip_Cfg.h are of the same Software version */
#if ((TSPC_PORT_IP_SW_MAJOR_VERSION_CFG_C != TSPC_PORT_IP_SW_MAJOR_VERSION_CFG_H) || \
    (TSPC_PORT_IP_SW_MINOR_VERSION_CFG_C != TSPC_PORT_IP_SW_MINOR_VERSION_CFG_H) || \
    (TSPC_PORT_IP_SW_PATCH_VERSION_CFG_C != TSPC_PORT_IP_SW_PATCH_VERSION_CFG_H)    \
    )
    #error "Software Version Numbers of Tspc_Port_Ip_Cfg.c and Tspc_Port_Ip_Cfg.h are different"
#endif

/*==================================================================================================
                             LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
                                             LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
                                            LOCAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
                                           LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                           GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
                                           GLOBAL VARIABLES
==================================================================================================*/

/* clang-format off */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
PortContainer_0_BOARD_InitPeripherals:
- options: {callFromInitBoot: 'true', coreID: M7_0}
- pin_list:
  - {pin_num: A13, peripheral: SIUL2, signal: 'gpio, 0', pin_signal: PTA0, direction: INPUT/OUTPUT}
  - {pin_num: M15, peripheral: CAN0, signal: can0_rx, pin_signal: PTA6}
  - {pin_num: M16, peripheral: CAN0, signal: can0_tx, pin_signal: PTA7}
  - {pin_num: A2, peripheral: LPUART2, signal: lpuart2_rx, pin_signal: PTA8}
  - {pin_num: B3, peripheral: LPUART2, signal: lpuart2_tx, pin_signal: PTA9, direction: OUTPUT}
  - {pin_num: P16, peripheral: SIUL2, signal: 'gpio, 32', pin_signal: PTB0, direction: OUTPUT}
  - {pin_num: R17, peripheral: SIUL2, signal: 'gpio, 33', pin_signal: PTB1, direction: OUTPUT}
  - {pin_num: P7, peripheral: SIUL2, signal: 'gpio, 56', pin_signal: PTB24, direction: OUTPUT}
  - {pin_num: R9, peripheral: SIUL2, signal: 'gpio, 58', pin_signal: PTB26, direction: OUTPUT}
  - {pin_num: N15, peripheral: CAN1, signal: can1_tx, pin_signal: PTC8}
  - {pin_num: N14, peripheral: CAN1, signal: can1_rx, pin_signal: PTC9}
  - {pin_num: T17, peripheral: CAN5, signal: can5_tx, pin_signal: PTC10}
  - {pin_num: T15, peripheral: CAN5, signal: can5_rx, pin_signal: PTC11}
  - {pin_num: U11, peripheral: SIUL2, signal: 'gpio, 82', pin_signal: PTC18, direction: OUTPUT}
  - {pin_num: T13, peripheral: SIUL2, signal: 'gpio, 84', pin_signal: PTC20, direction: INPUT}
  - {pin_num: R13, peripheral: SIUL2, signal: 'gpio, 85', pin_signal: PTC21, direction: OUTPUT}
  - {pin_num: U14, peripheral: SIUL2, signal: 'gpio, 87', pin_signal: PTC23, direction: INPUT}
  - {pin_num: R15, peripheral: SIUL2, signal: 'gpio, 88', pin_signal: PTC24, direction: OUTPUT}
  - {pin_num: R14, peripheral: SIUL2, signal: 'gpio, 89', pin_signal: PTC25, direction: OUTPUT}
  - {pin_num: P15, peripheral: SIUL2, signal: 'gpio, 90', pin_signal: PTC26, direction: OUTPUT}
  - {pin_num: R16, peripheral: SIUL2, signal: 'gpio, 91', pin_signal: PTC27, direction: INPUT}
  - {pin_num: P17, peripheral: SIUL2, signal: 'gpio, 92', pin_signal: PTC28, direction: OUTPUT}
  - {pin_num: N16, peripheral: SIUL2, signal: 'gpio, 93', pin_signal: PTC29, direction: OUTPUT}
  - {pin_num: N17, peripheral: SIUL2, signal: 'gpio, 94', pin_signal: PTC30, direction: OUTPUT}
  - {pin_num: L14, peripheral: SIUL2, signal: 'gpio, 95', pin_signal: PTC31, direction: OUTPUT}
  - {pin_num: F16, peripheral: SIUL2, signal: 'gpio, 98', pin_signal: PTD2, direction: OUTPUT}
  - {pin_num: F17, peripheral: SIUL2, signal: 'gpio, 99', pin_signal: PTD3, direction: INPUT}
  - {pin_num: G16, peripheral: SIUL2, signal: 'gpio, 100', pin_signal: PTD4, direction: OUTPUT}
  - {pin_num: K15, peripheral: SIUL2, signal: 'gpio, 116', pin_signal: PTD20, direction: OUTPUT}
  - {pin_num: H15, peripheral: SIUL2, signal: 'gpio, 117', pin_signal: PTD21, direction: INPUT}
  - {pin_num: G17, peripheral: SIUL2, signal: 'gpio, 118', pin_signal: PTD22, direction: OUTPUT}
  - {pin_num: E17, peripheral: SIUL2, signal: 'gpio, 119', pin_signal: PTD23, direction: OUTPUT}
  - {pin_num: D16, peripheral: SIUL2, signal: 'gpio, 120', pin_signal: PTD24, direction: INPUT}
  - {pin_num: D13, peripheral: SIUL2, signal: 'gpio, 126', pin_signal: PTD30, direction: OUTPUT}
  - {pin_num: C13, peripheral: SIUL2, signal: 'gpio, 127', pin_signal: PTD31, direction: OUTPUT}
  - {pin_num: D12, peripheral: SIUL2, signal: 'gpio, 145', pin_signal: PTE17, direction: OUTPUT}
  - {pin_num: F2, peripheral: SIUL2, signal: 'gpio, 133', pin_signal: PTE5, direction: OUTPUT}
  - {pin_num: A8, peripheral: CAN2, signal: can2_tx, pin_signal: PTE24}
  - {pin_num: B8, peripheral: CAN2, signal: can2_rx, pin_signal: PTE25}
  - {pin_num: P5, peripheral: LPI2C0, signal: lpi2c0_sda, pin_signal: PTD13, direction: INPUT/OUTPUT}
  - {pin_num: M4, peripheral: LPI2C0, signal: lpi2c0_scl, pin_signal: PTD14, direction: INPUT/OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* No registers that support TSPC were configured*/

/*==================================================================================================
                                      LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
                                           LOCAL FUNCTIONS
==================================================================================================*/

/*==================================================================================================
                                           GLOBAL FUNCTIONS
==================================================================================================*/


#ifdef __cplusplus
}
#endif

/** @} */
