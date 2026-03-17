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

#ifndef SIUL2_PORT_IP_CFG_H
#define SIUL2_PORT_IP_CFG_H

/**
*   @file      Siul2_Port_Ip_Cfg.h
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
#include "S32K344_SIUL2.h"
#include "Siul2_Port_Ip_Types.h"

/*==================================================================================================
*                              SOURCE FILE VERSION INFORMATION
==================================================================================================*/
#define SIUL2_PORT_IP_VENDOR_ID_CFG_H                       43
#define SIUL2_PORT_IP_AR_RELEASE_MAJOR_VERSION_CFG_H        4
#define SIUL2_PORT_IP_AR_RELEASE_MINOR_VERSION_CFG_H        9
#define SIUL2_PORT_IP_AR_RELEASE_REVISION_VERSION_CFG_H     0
#define SIUL2_PORT_IP_SW_MAJOR_VERSION_CFG_H                7
#define SIUL2_PORT_IP_SW_MINOR_VERSION_CFG_H                0
#define SIUL2_PORT_IP_SW_PATCH_VERSION_CFG_H                0

/*==================================================================================================
*                                     FILE VERSION CHECKS
==================================================================================================*/
/* Check if the files Siul2_Port_Ip_Cfg.h and Siul2_Port_Ip_Types.h are of the same version */
#if (SIUL2_PORT_IP_VENDOR_ID_CFG_H != SIUL2_PORT_IP_TYPES_VENDOR_ID_H)
    #error "Siul2_Port_Ip_Cfg.h and Siul2_Port_Ip_Types.h have different vendor ids"
#endif
/* Check if Siul2_Port_Ip_Cfg.h and Siul2_Port_Ip_Types.h are of the same Autosar version */
#if ((SIUL2_PORT_IP_AR_RELEASE_MAJOR_VERSION_CFG_H    != SIUL2_PORT_IP_TYPES_AR_RELEASE_MAJOR_VERSION_H) || \
     (SIUL2_PORT_IP_AR_RELEASE_MINOR_VERSION_CFG_H    != SIUL2_PORT_IP_TYPES_AR_RELEASE_MINOR_VERSION_H) || \
     (SIUL2_PORT_IP_AR_RELEASE_REVISION_VERSION_CFG_H != SIUL2_PORT_IP_TYPES_AR_RELEASE_REVISION_VERSION_H) \
    )
    #error "AutoSar Version Numbers of Siul2_Port_Ip_Cfg.h and Siul2_Port_Ip_Types.h are different"
#endif
/* Check if Siul2_Port_Ip_Cfg.h and Siul2_Port_Ip_Types.h are of the same Software version */
#if ((SIUL2_PORT_IP_SW_MAJOR_VERSION_CFG_H != SIUL2_PORT_IP_TYPES_SW_MAJOR_VERSION_H) || \
     (SIUL2_PORT_IP_SW_MINOR_VERSION_CFG_H != SIUL2_PORT_IP_TYPES_SW_MINOR_VERSION_H) || \
     (SIUL2_PORT_IP_SW_PATCH_VERSION_CFG_H != SIUL2_PORT_IP_TYPES_SW_PATCH_VERSION_H)    \
    )
    #error "Software Version Numbers of Siul2_Port_Ip_Cfg.h and Siul2_Port_Ip_Types.h are different"
#endif
/*==================================================================================================
                                           CONSTANTS
==================================================================================================*/

/*==================================================================================================
                                      DEFINES AND MACROS
==================================================================================================*/
#define SIUL2_MSCR_SSS_MASK                      (0xFU)
#define SIUL2_MSCR_SSS_SHIFT                     (0U)
#define SIUL2_MSCR_SSS_WIDTH                     (4U)
#define SIUL2_MSCR_SSS(x)                        (((uint32)(((uint32)(x)) << SIUL2_MSCR_SSS_SHIFT)) & SIUL2_MSCR_SSS_MASK)

#define SIUL2_MSCR_SRE_MASK                      (0x4000U)
#define SIUL2_MSCR_SRE_SHIFT                     (14U)
#define SIUL2_MSCR_SRE_WIDTH                     (1U)
#define SIUL2_MSCR_SRE(x)                        (((uint32)(((uint32)(x)) << SIUL2_MSCR_SRE_SHIFT)) & SIUL2_MSCR_SRE_MASK)


/*! @brief Definitions for PortContainer_0_BOARD_InitPeripherals Functional Group */

/*! @brief User number of configured pins */
#define NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals 41

#define PORT_START_SEC_CONFIG_DATA_UNSPECIFIED
#include "Port_MemMap.h"

/*! @brief User configuration structure */
extern const Siul2_Port_Ip_PinSettingsConfig g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals[NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals];

#define PORT_STOP_SEC_CONFIG_DATA_UNSPECIFIED
#include "Port_MemMap.h"

/*! @brief Defines for user pin and port configurations */
#define can0_rx_PIN             6u
#define can0_rx_PORT            PTA_L_HALF
#define can0_tx_PIN             7u
#define can0_tx_PORT            PTA_L_HALF
#define uart2_rx_PIN            8u
#define uart2_rx_PORT           PTA_L_HALF
#define uart2_tx_PIN            9u
#define uart2_tx_PORT           PTA_L_HALF
#define can3_en_PIN             0u
#define can3_en_PORT            PTB_L_HALF
#define can3_stb_PIN            1u
#define can3_stb_PORT           PTB_L_HALF
#define can3_led_PIN            8u
#define can3_led_PORT           PTB_H_HALF
#define can4_led_PIN            10u
#define can4_led_PORT           PTB_H_HALF
#define can1_tx_PIN             8u
#define can1_tx_PORT            PTC_L_HALF
#define can1_rx_PIN             9u
#define can1_rx_PORT            PTC_L_HALF
#define can5_tx_PIN             10u
#define can5_tx_PORT            PTC_L_HALF
#define can5_rx_PIN             11u
#define can5_rx_PORT            PTC_L_HALF
#define can0_led_PIN            2u
#define can0_led_PORT           PTC_H_HALF
#define can0_err_PIN            4u
#define can0_err_PORT           PTC_H_HALF
#define can0_stb_PIN            5u
#define can0_stb_PORT           PTC_H_HALF
#define can4_err_PIN            7u
#define can4_err_PORT           PTC_H_HALF
#define can0_en_PIN             8u
#define can0_en_PORT            PTC_H_HALF
#define can4_stb_PIN            9u
#define can4_stb_PORT           PTC_H_HALF
#define can4_en_PIN             10u
#define can4_en_PORT            PTC_H_HALF
#define can3_err_PIN            11u
#define can3_err_PORT           PTC_H_HALF
#define can3_tx_PIN             12u
#define can3_tx_PORT            PTC_H_HALF
#define can3_rx_PIN             13u
#define can3_rx_PORT            PTC_H_HALF
#define can4_tx_PIN             14u
#define can4_tx_PORT            PTC_H_HALF
#define can4_rx_PIN             15u
#define can4_rx_PORT            PTC_H_HALF
#define can1_stb_PIN            2u
#define can1_stb_PORT           PTD_L_HALF
#define can1_err_PIN            3u
#define can1_err_PORT           PTD_L_HALF
#define can2_en_PIN             4u
#define can2_en_PORT            PTD_L_HALF
#define can2_led_PIN            4u
#define can2_led_PORT           PTD_H_HALF
#define can2_err_PIN            5u
#define can2_err_PORT           PTD_H_HALF
#define can2_stb_PIN            6u
#define can2_stb_PORT           PTD_H_HALF
#define can1_en_PIN             7u
#define can1_en_PORT            PTD_H_HALF
#define can5_err_PIN            8u
#define can5_err_PORT           PTD_H_HALF
#define can5_en_PIN             14u
#define can5_en_PORT            PTD_H_HALF
#define can5_led_PIN            15u
#define can5_led_PORT           PTD_H_HALF
#define can5_stb_PIN            1u
#define can5_stb_PORT           PTE_H_HALF
#define can1_led_PIN            5u
#define can1_led_PORT           PTE_L_HALF
#define can2_tx_PIN             8u
#define can2_tx_PORT            PTE_H_HALF
#define can2_rx_PIN             9u
#define can2_rx_PORT            PTE_H_HALF
#define i2c0_sda_PIN            13u
#define i2c0_sda_PORT           PTD_L_HALF
#define i2c0_scl_PIN            14u
#define i2c0_scl_PORT           PTD_L_HALF

/*==================================================================================================
                                           ENUMS
==================================================================================================*/

/*==================================================================================================
                               STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

/*==================================================================================================
                               GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/*==================================================================================================
                               FUNCTION PROTOTYPES
==================================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* SIUL2_PORT_IP_CFG_H */

