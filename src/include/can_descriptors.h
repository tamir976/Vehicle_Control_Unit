#ifndef INCLUDE_CAN_DESCRIPTORS_H_
#define INCLUDE_CAN_DESCRIPTORS_H_

#include <stdint.h>

#define MESSAGE_COUNT 60
#define SIGNAL_COUNT 367

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint16_t signal_start;
    uint8_t signal_count;
} MessageDesc;

typedef struct {
    uint16_t start_bit;
    uint8_t length;
    uint8_t endian;
    uint8_t is_signed;
    float scale;
    float offset;
    uint8_t signal_type;
} SignalDesc; /* signal_type: 0=bool, 1=int/enum, 2=float */

static MessageDesc message_table[] = {
    {0x24, 8, 0, 3}, /* KINEMATICS */
    {0x25, 8, 3, 3}, /* STEER_ANGLE_SENSOR */
    {0x77, 6, 6, 6}, /* ENG2F41 */
    {0x78, 4, 12, 5}, /* ENG2F42 */
    {0xA6, 8, 17, 2}, /* BRAKE */
    {0xAA, 8, 19, 4}, /* WHEEL_SPEEDS */
    {0xB4, 8, 23, 3}, /* SPEED */
    {0x127, 8, 26, 4}, /* GEAR_PACKET_HYBRID */
    {0x161, 7, 30, 1}, /* DSU_SPEED */
    {0x1C4, 8, 31, 2}, /* ENGINE_RPM */
    {0x1D3, 8, 33, 7}, /* PCM_CRUISE_2 */
    {0x228, 4, 40, 3}, /* VSC1S29 */
    {0x230, 7, 43, 1}, /* BRAKE_2 */
    {0x245, 8, 44, 1}, /* GAS_PEDAL_HYBRID */
    {0x260, 8, 45, 6}, /* STEER_TORQUE_SENSOR */
    {0x266, 8, 51, 8}, /* STEERING_IPAS */
    {0x2E6, 8, 59, 3}, /* LEAD_INFO */
    {0x320, 8, 62, 23}, /* VSC1S07 */
    {0x343, 8, 85, 15}, /* ACC_CONTROL */
    {0x361, 8, 100, 4}, /* CLUTCH */
    {0x365, 7, 104, 7}, /* DSU_CRUISE */
    {0x399, 8, 111, 5}, /* PCM_CRUISE_SM */
    {0x3B7, 8, 116, 5}, /* ESP_CONTROL */
    {0x3BC, 8, 121, 7}, /* GEAR_PACKET */
    {0x3ED, 2, 128, 1}, /* REVERSE_CAMERA_STATE */
    {0x3F1, 8, 129, 4}, /* PCM_CRUISE_ALT */
    {0x3FC, 8, 133, 1}, /* SOLAR_SENSOR */
    {0x411, 8, 134, 11}, /* PCS_HUD */
    {0x412, 8, 145, 25}, /* LKAS_HUD */
    {0x413, 8, 170, 9}, /* TIME */
    {0x414, 8, 179, 3}, /* AUTO_HIGH_BEAM */
    {0x420, 8, 182, 10}, /* VSC1S08 */
    {0x43B, 8, 192, 1}, /* AUTOPARK_STATUS */
    {0x489, 8, 193, 13}, /* RSA1 */
    {0x48A, 8, 206, 14}, /* RSA2 */
    {0x48B, 8, 220, 9}, /* RSA3 */
    {0x580, 8, 229, 8}, /* VIN_PART_1 */
    {0x581, 8, 237, 8}, /* VIN_PART_2 */
    {0x582, 8, 245, 1}, /* VIN_PART_3 */
    {0x610, 8, 246, 5}, /* BODY_CONTROL_STATE_2 */
    {0x611, 8, 251, 2}, /* UI_SETTING */
    {0x614, 8, 253, 3}, /* BLINKERS_STATE */
    {0x620, 8, 256, 7}, /* BODY_CONTROL_STATE */
    {0x622, 8, 263, 6}, /* LIGHT_STALK */
    {0x623, 8, 269, 3}, /* CERTIFICATION_ECU */
    {0x638, 8, 272, 3}, /* DOOR_LOCKS */
    {0x6F3, 8, 275, 12}, /* ADAS_TOGGLE_STATE */
    {0xC3, 8, 287, 1}, /* DISABLE_HACKED_PANDA */
    {0xC4, 8, 288, 1}, /* ENABLE_HACKED_PANDA */
    {0x167, 8, 289, 8}, /* STEERING_IPAS_COMMA */
    {0x1D2, 8, 297, 8}, /* PCM_CRUISE */
    {0x283, 7, 305, 9}, /* PRE_COLLISION */
    {0x2C1, 8, 314, 6}, /* GAS_PEDAL */
    {0x2E4, 5, 320, 6}, /* STEERING_LKA */
    {0x344, 8, 326, 7}, /* PRE_COLLISION_2 */
    {0x191, 8, 333, 12}, /* STEERING_LTA */
    {0x226, 8, 345, 3}, /* BRAKE_MODULE */
    {0x262, 8, 348, 5}, /* EPS_STATUS */
    {0x371, 8, 353, 8}, /* LTA_RELATED */
    {0x3F6, 8, 361, 6}, /* BSM */
};

static SignalDesc signal_table[] = {
    {33, 10, 0, 0, 0.03589f, -18.375f, 2}, /* ACCEL_Y (KINEMATICS) */
    {1, 10, 0, 0, 0.244f, -125.0f, 2}, /* YAW_RATE (KINEMATICS) */
    {17, 10, 0, 0, 0.03589f, -18.375f, 2}, /* ACCEL_X (KINEMATICS) */
    {3, 12, 0, 1, 1.5f, 0.0f, 2}, /* STEER_ANGLE (STEER_ANGLE_SENSOR) */
    {39, 4, 0, 1, 0.1f, 0.0f, 2}, /* STEER_FRACTION (STEER_ANGLE_SENSOR) */
    {35, 12, 0, 1, 1.0f, 0.0f, 1}, /* STEER_RATE (STEER_ANGLE_SENSOR) */
    {7, 16, 0, 1, 2.0f, 0.0f, 1}, /* FDRV (ENG2F41) */
    {23, 13, 0, 1, 10.0f, 0.0f, 1}, /* FDRVREAL (ENG2F41) */
    {39, 1, 0, 0, 1.0f, 0.0f, 0}, /* XAECT (ENG2F41) */
    {38, 1, 0, 0, 1.0f, 0.0f, 0}, /* XFDRVCOL (ENG2F41) */
    {34, 3, 0, 0, 1.0f, 0.0f, 1}, /* FDRVSELP (ENG2F41) */
    {47, 8, 0, 0, 1.0f, 0.0f, 1}, /* ENG2F41S (ENG2F41) */
    {7, 16, 0, 1, 2.0f, 0.0f, 1}, /* FAVLMCHH (ENG2F42) */
    {23, 1, 0, 0, 1.0f, 0.0f, 0}, /* CCRNG (ENG2F42) */
    {22, 3, 0, 0, 1.0f, 0.0f, 1}, /* FDRVTYPD (ENG2F42) */
    {18, 1, 0, 0, 1.0f, 0.0f, 0}, /* GEARHD (ENG2F42) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* ENG2F42S (ENG2F42) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* BRAKE_AMOUNT (BRAKE) */
    {23, 8, 0, 0, 40.0f, 0.0f, 1}, /* BRAKE_FORCE (BRAKE) */
    {7, 16, 0, 0, 0.01f, -67.67f, 2}, /* WHEEL_SPEED_FR (WHEEL_SPEEDS) */
    {23, 16, 0, 0, 0.01f, -67.67f, 2}, /* WHEEL_SPEED_FL (WHEEL_SPEEDS) */
    {39, 16, 0, 0, 0.01f, -67.67f, 2}, /* WHEEL_SPEED_RR (WHEEL_SPEEDS) */
    {55, 16, 0, 0, 0.01f, -67.67f, 2}, /* WHEEL_SPEED_RL (WHEEL_SPEEDS) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* ENCODER (SPEED) */
    {47, 16, 0, 0, 0.01f, 0.0f, 2}, /* SPEED (SPEED) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (SPEED) */
    {26, 11, 0, 1, 25.0f, 0.0f, 1}, /* FDRVREAL (GEAR_PACKET_HYBRID) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* UNKNOWN (GEAR_PACKET_HYBRID) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (GEAR_PACKET_HYBRID) */
    {47, 4, 0, 0, 1.0f, 0.0f, 1}, /* GEAR (GEAR_PACKET_HYBRID) | VAL_ 0 "P" 1 "R" 2 "N" 3 "D" 4 "B" */
    {15, 16, 0, 1, 0.00390625f, -30.0f, 2}, /* FORWARD_SPEED (DSU_SPEED) */
    {7, 16, 0, 1, 0.78125f, 0.0f, 2}, /* RPM (ENGINE_RPM) */
    {27, 1, 0, 0, 1.0f, 0.0f, 0}, /* ENGINE_RUNNING (ENGINE_RPM) */
    {3, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRAKE_PRESSED (PCM_CRUISE_2) */
    {12, 2, 0, 0, 1.0f, 0.0f, 1}, /* PCM_FOLLOW_DISTANCE (PCM_CRUISE_2) | VAL_ 1 "far" 2 "medium" 3 "close" */
    {14, 2, 0, 0, 1.0f, 0.0f, 1}, /* LOW_SPEED_LOCKOUT (PCM_CRUISE_2) | VAL_ 2 "low speed locked" 1 "ok" */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* MAIN_ON (PCM_CRUISE_2) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_SPEED (PCM_CRUISE_2) */
    {47, 1, 0, 0, 1.0f, 0.0f, 0}, /* ACC_FAULTED (PCM_CRUISE_2) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (PCM_CRUISE_2) */
    {7, 1, 0, 0, 1.0f, 0.0f, 0}, /* ICBACT (VSC1S29) */
    {6, 15, 0, 1, 0.001f, 0.0f, 2}, /* DVS0PCS (VSC1S29) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* SM228 (VSC1S29) */
    {26, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRAKE_PRESSED (BRAKE_2) */
    {23, 8, 0, 0, 0.005f, 0.0f, 2}, /* GAS_PEDAL (GAS_PEDAL_HYBRID) */
    {47, 16, 0, 1, 1.0f, 0.0f, 1}, /* STEER_TORQUE_EPS (STEER_TORQUE_SENSOR) */
    {15, 16, 0, 1, 1.0f, 0.0f, 1}, /* STEER_TORQUE_DRIVER (STEER_TORQUE_SENSOR) */
    {31, 16, 0, 1, 0.0573f, 0.0f, 2}, /* STEER_ANGLE (STEER_TORQUE_SENSOR) */
    {3, 1, 0, 0, 1.0f, 0.0f, 0}, /* STEER_ANGLE_INITIALIZING (STEER_TORQUE_SENSOR) */
    {0, 1, 0, 0, 1.0f, 0.0f, 0}, /* STEER_OVERRIDE (STEER_TORQUE_SENSOR) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (STEER_TORQUE_SENSOR) */
    {7, 4, 0, 0, 1.0f, 0.0f, 1}, /* STATE (STEERING_IPAS) | VAL_ 3 "enabled" 1 "disabled" */
    {3, 12, 0, 1, 1.5f, 0.0f, 2}, /* ANGLE (STEERING_IPAS) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X10 (STEERING_IPAS) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X00 (STEERING_IPAS) */
    {38, 2, 0, 0, 1.0f, 0.0f, 1}, /* DIRECTION_CMD (STEERING_IPAS) | VAL_ 3 "right" 2 "center" 1 "left" */
    {47, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X40 (STEERING_IPAS) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X00_1 (STEERING_IPAS) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (STEERING_IPAS) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (LEAD_INFO) */
    {23, 12, 0, 1, 0.025f, 0.0f, 2}, /* LEAD_REL_SPEED (LEAD_INFO) */
    {7, 13, 0, 0, 0.05f, 0.0f, 2}, /* LEAD_LONG_DIST (LEAD_INFO) */
    {6, 1, 0, 0, 1.0f, 0.0f, 0}, /* FBKRLY (VSC1S07) */
    {4, 1, 0, 0, 1.0f, 0.0f, 0}, /* FVSCM (VSC1S07) */
    {3, 1, 0, 0, 1.0f, 0.0f, 0}, /* FVSCSFT (VSC1S07) */
    {2, 1, 0, 0, 1.0f, 0.0f, 0}, /* FABS (VSC1S07) */
    {1, 1, 0, 0, 1.0f, 0.0f, 0}, /* TSVSC (VSC1S07) */
    {0, 1, 0, 0, 1.0f, 0.0f, 0}, /* FVSCL (VSC1S07) */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* RQCSTBKB (VSC1S07) */
    {14, 1, 0, 0, 1.0f, 0.0f, 0}, /* PSBSTBY (VSC1S07) */
    {13, 1, 0, 0, 1.0f, 0.0f, 0}, /* P2BRXMK (VSC1S07) */
    {11, 1, 0, 0, 1.0f, 0.0f, 0}, /* MCC (VSC1S07) */
    {10, 1, 0, 0, 1.0f, 0.0f, 0}, /* RQBKB (VSC1S07) */
    {9, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRSTOP (VSC1S07) */
    {8, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRKON (VSC1S07) */
    {23, 8, 0, 1, 1.0f, 0.0f, 1}, /* ASLP (VSC1S07) */
    {31, 2, 0, 0, 1.0f, 0.0f, 1}, /* BRTYPACC (VSC1S07) */
    {26, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRKABT3 (VSC1S07) */
    {25, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRKABT2 (VSC1S07) */
    {24, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRKABT1 (VSC1S07) */
    {39, 8, 0, 1, 0.04f, 0.0f, 2}, /* GVC (VSC1S07) */
    {43, 1, 0, 0, 1.0f, 0.0f, 0}, /* XGVCINV (VSC1S07) */
    {52, 1, 0, 0, 1.0f, 0.0f, 0}, /* S07CNT (VSC1S07) */
    {50, 2, 0, 0, 1.0f, 0.0f, 1}, /* PCSBRSTA (VSC1S07) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* VSC07SUM (VSC1S07) */
    {7, 16, 0, 1, 0.001f, 0.0f, 2}, /* ACCEL_CMD (ACC_CONTROL) */
    {17, 2, 0, 0, 1.0f, 0.0f, 1}, /* ALLOW_LONG_PRESS (ACC_CONTROL) | VAL_ 2 "set speed increase by 5 speed units regardless" 1 "set speed increase by 1 speed unit on short press, 5 speed units on long press" */
    {18, 1, 0, 0, 1.0f, 0.0f, 1}, /* ACC_MALFUNCTION (ACC_CONTROL) | VAL_ 1 "faulted" 0 "ok" */
    {19, 1, 0, 0, 1.0f, 0.0f, 0}, /* RADAR_DIRTY (ACC_CONTROL) */
    {20, 1, 0, 0, 1.0f, 0.0f, 0}, /* DISTANCE (ACC_CONTROL) */
    {21, 1, 0, 0, 1.0f, 0.0f, 0}, /* MINI_CAR (ACC_CONTROL) */
    {23, 2, 0, 0, 1.0f, 0.0f, 1}, /* ACC_TYPE (ACC_CONTROL) | VAL_ 2 "permanent low speed lockout" 1 "ok" */
    {24, 1, 0, 0, 1.0f, 0.0f, 0}, /* CANCEL_REQ (ACC_CONTROL) */
    {25, 1, 0, 0, 1.0f, 0.0f, 1}, /* ACC_CUT_IN (ACC_CONTROL) | VAL_ 1 "CUT-IN Detected" 0 "clear" */
    {29, 1, 0, 0, 1.0f, 0.0f, 0}, /* LEAD_VEHICLE_STOPPED (ACC_CONTROL) */
    {30, 1, 0, 0, 1.0f, 0.0f, 0}, /* PERMIT_BRAKING (ACC_CONTROL) */
    {31, 1, 0, 0, 1.0f, 0.0f, 0}, /* RELEASE_STANDSTILL (ACC_CONTROL) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* ITS_CONNECT_LEAD (ACC_CONTROL) */
    {47, 8, 0, 1, 0.05f, 0.0f, 2}, /* ACCEL_CMD_ALT (ACC_CONTROL) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (ACC_CONTROL) */
    {32, 1, 0, 0, 1.0f, 0.0f, 0}, /* ACC_FAULTED (CLUTCH) */
    {23, 8, 0, 0, 0.005f, 0.0f, 2}, /* GAS_PEDAL_ALT (CLUTCH) */
    {38, 1, 0, 0, 1.0f, 0.0f, 1}, /* CLUTCH_RELEASED (CLUTCH) | VAL_ 0 "clutch pressed any amount" 1 "clutch released" */
    {48, 16, 1, 0, 0.0002f, -6.5536f, 2}, /* ACCEL_NET (CLUTCH) */
    {3, 1, 0, 0, 1.0f, 0.0f, 0}, /* RES_BTN (DSU_CRUISE) */
    {2, 1, 0, 0, 1.0f, 0.0f, 0}, /* SET_BTN (DSU_CRUISE) */
    {1, 1, 0, 0, 1.0f, 0.0f, 0}, /* CANCEL_BTN (DSU_CRUISE) */
    {0, 1, 0, 0, 1.0f, 0.0f, 0}, /* MAIN_ON (DSU_CRUISE) */
    {15, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_SPEED (DSU_CRUISE) */
    {31, 8, 0, 0, 100.0f, -12800.0f, 1}, /* CRUISE_REQUEST (DSU_CRUISE) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* LEAD_DISTANCE (DSU_CRUISE) */
    {4, 1, 0, 0, 1.0f, 0.0f, 0}, /* MAIN_ON (PCM_CRUISE_SM) */
    {11, 4, 0, 0, 1.0f, 0.0f, 1}, /* CRUISE_CONTROL_STATE (PCM_CRUISE_SM) | VAL_ 2 "disabled" 11 "hold" 10 "hold_waiting_user_cmd" 6 "enabled" 5 "faulted" */
    {14, 2, 0, 0, 1.0f, 0.0f, 1}, /* DISTANCE_LINES (PCM_CRUISE_SM) | VAL_ 0 "not displayed" 1 "close" 2 "medium" 3 "far" */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* TEMP_ACC_FAULTED (PCM_CRUISE_SM) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* UI_SET_SPEED (PCM_CRUISE_SM) */
    {13, 1, 0, 0, 1.0f, 0.0f, 0}, /* TC_DISABLED (ESP_CONTROL) */
    {12, 2, 0, 0, 1.0f, 0.0f, 1}, /* VSC_DISABLED (ESP_CONTROL) */
    {18, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRAKE_LIGHTS_ACC (ESP_CONTROL) */
    {33, 1, 1, 0, 1.0f, 0.0f, 0}, /* BRAKE_HOLD_ENABLED (ESP_CONTROL) */
    {36, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRAKE_HOLD_ACTIVE (ESP_CONTROL) */
    {2, 1, 0, 0, 1.0f, 0.0f, 1}, /* SPORT_ON (GEAR_PACKET) | VAL_ 0 "off" 1 "on" */
    {13, 6, 0, 0, 1.0f, 0.0f, 1}, /* GEAR (GEAR_PACKET) | VAL_ 0 "D" 1 "S" 8 "N" 16 "R" 32 "P" */
    {33, 1, 0, 0, 1.0f, 0.0f, 1}, /* SPORT_GEAR_ON (GEAR_PACKET) | VAL_ 0 "off" 1 "on" */
    {38, 3, 0, 0, 1.0f, 0.0f, 1}, /* SPORT_GEAR (GEAR_PACKET) | VAL_ 1 "S1" 2 "S2" 3 "S3" 4 "S4" 5 "S5" 6 "S6" */
    {40, 1, 0, 0, 1.0f, 0.0f, 1}, /* ECON_ON (GEAR_PACKET) | VAL_ 0 "off" 1 "on" */
    {41, 1, 0, 0, 1.0f, 0.0f, 1}, /* B_GEAR_ENGAGED (GEAR_PACKET) | VAL_ 0 "off" 1 "on" */
    {47, 1, 0, 0, 1.0f, 0.0f, 1}, /* DRIVE_ENGAGED (GEAR_PACKET) | VAL_ 0 "off" 1 "on" */
    {9, 2, 0, 0, 1.0f, 0.0f, 1}, /* REVERSE_CAMERA_GUIDELINES (REVERSE_CAMERA_STATE) | VAL_ 3 "No guidelines" 2 "Static guidelines" 1 "Active guidelines" */
    {4, 2, 1, 0, 1.0f, 0.0f, 1}, /* PCM_FOLLOW_DISTANCE (PCM_CRUISE_ALT) | VAL_ 1 "far" 2 "medium" 3 "close" */
    {13, 1, 0, 0, 1.0f, 0.0f, 0}, /* MAIN_ON (PCM_CRUISE_ALT) */
    {10, 1, 0, 0, 1.0f, 0.0f, 0}, /* CRUISE_STATE (PCM_CRUISE_ALT) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* UI_SET_SPEED (PCM_CRUISE_ALT) */
    {55, 13, 0, 0, 1.0f, 0.0f, 1}, /* LUX_SENSOR (SOLAR_SENSOR) */
    {7, 2, 0, 0, 1.0f, 0.0f, 1}, /* PCS_INDICATOR (PCS_HUD) | VAL_ 2 "PCS Faulted" 1 "PCS Turned Off By User" 0 "PCS Enabled" */
    {4, 1, 0, 0, 1.0f, 0.0f, 0}, /* FCW (PCS_HUD) */
    {15, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X20 (PCS_HUD) */
    {34, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCS_DUST (PCS_HUD) */
    {35, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCS_TEMP (PCS_HUD) */
    {41, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCS_DUST2 (PCS_HUD) */
    {42, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCS_TEMP2 (PCS_HUD) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X10 (PCS_HUD) */
    {40, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCS_OFF (PCS_HUD) */
    {53, 3, 0, 0, 1.0f, 0.0f, 1}, /* FRD_ADJ (PCS_HUD) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* PCS_SENSITIVITY (PCS_HUD) | VAL_ 64 "high sensitivity"  128 "mid sensitivity" 192 "low sensitivity" 0 "off" */
    {1, 2, 0, 0, 1.0f, 0.0f, 1}, /* BARRIERS (LKAS_HUD) | VAL_ 3 "left" 2 "right" 1 "both" 0 "none" */
    {3, 2, 0, 0, 1.0f, 0.0f, 1}, /* RIGHT_LINE (LKAS_HUD) | VAL_ 3 "orange" 2 "faded" 1 "solid" 0 "none" */
    {5, 2, 0, 0, 1.0f, 0.0f, 1}, /* LEFT_LINE (LKAS_HUD) | VAL_ 3 "orange" 2 "faded" 1 "solid" 0 "none" */
    {7, 2, 0, 0, 1.0f, 0.0f, 1}, /* LKAS_STATUS (LKAS_HUD) | VAL_ 1 "on" 0 "off" */
    {9, 2, 0, 0, 1.0f, 0.0f, 1}, /* LDA_ALERT (LKAS_HUD) | VAL_ 3 "hold with continuous beep" 2 "LDA unavailable" 1 "hold" 0 "none" */
    {10, 1, 0, 0, 1.0f, 0.0f, 0}, /* LDW_EXIST (LKAS_HUD) */
    {12, 1, 0, 0, 1.0f, 0.0f, 0}, /* TWO_BEEPS (LKAS_HUD) */
    {13, 1, 0, 0, 1.0f, 0.0f, 0}, /* ADJUSTING_CAMERA (LKAS_HUD) */
    {14, 1, 0, 0, 1.0f, 0.0f, 0}, /* LDA_UNAVAILABLE_QUIET (LKAS_HUD) */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* LDA_MALFUNCTION (LKAS_HUD) */
    {16, 1, 0, 0, 1.0f, 0.0f, 0}, /* LDA_UNAVAILABLE (LKAS_HUD) */
    {18, 2, 0, 0, 1.0f, 0.0f, 1}, /* LDA_SENSITIVITY (LKAS_HUD) | VAL_ 2 "standard" 1 "high" 0 "undefined" */
    {20, 2, 0, 0, 1.0f, 0.0f, 1}, /* LDA_SA_TOGGLE (LKAS_HUD) | VAL_ 2 "steering assist off" 1 "steering assist on" */
    {23, 3, 0, 0, 1.0f, 0.0f, 1}, /* LDA_MESSAGES (LKAS_HUD) | VAL_ 4 "lda unavailable at this speed" 1 "lda unavailable below approx 50km/h" 0 "ok" */
    {31, 2, 0, 0, 1.0f, 0.0f, 1}, /* LDA_ON_MESSAGE (LKAS_HUD) | VAL_ 2 "Lane Departure Alert Turned ON, Steering Assist Inactive" 1 "Lane Departure Alert Turned ON, Steering Assist Active" 0 "clear" */
    {32, 1, 0, 0, 1.0f, 0.0f, 0}, /* REPEATED_BEEPS (LKAS_HUD) */
    {43, 1, 0, 0, 1.0f, 0.0f, 0}, /* LANE_SWAY_TOGGLE (LKAS_HUD) */
    {45, 2, 0, 0, 1.0f, 0.0f, 1}, /* LANE_SWAY_SENSITIVITY (LKAS_HUD) */
    {46, 1, 0, 0, 1.0f, 0.0f, 1}, /* TAKE_CONTROL (LKAS_HUD) | VAL_ 1 "take control" 0 "ok" */
    {47, 1, 0, 0, 1.0f, 0.0f, 1}, /* LDA_FRONT_CAMERA_BLOCKED (LKAS_HUD) | VAL_ 1 "lda unavailable" 0 "ok" */
    {50, 2, 0, 0, 1.0f, 0.0f, 1}, /* LANE_SWAY_BUZZER (LKAS_HUD) | VAL_ 3 "ok" 2 "beep twice" 1 "beep twice" 0 "ok" */
    {53, 3, 0, 0, 1.0f, 0.0f, 1}, /* LANE_SWAY_FLD (LKAS_HUD) */
    {55, 2, 0, 0, 1.0f, 0.0f, 1}, /* LANE_SWAY_WARNING (LKAS_HUD) | VAL_ 3 "ok" 2 "orange please take a break" 1 "prompt would you like to take a break" 0 "ok" */
    {42, 1, 0, 0, 1.0f, 0.0f, 0}, /* SET_ME_X01 (LKAS_HUD) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X02 (LKAS_HUD) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* YEAR (TIME) */
    {15, 8, 0, 0, 1.0f, 0.0f, 1}, /* MONTH (TIME) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* DAY (TIME) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* HOUR (TIME) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* MINUTE (TIME) */
    {55, 1, 0, 0, 1.0f, 0.0f, 0}, /* GMT_DIFF (TIME) */
    {54, 4, 0, 0, 1.0f, 0.0f, 1}, /* GMTDIFF_HOURS (TIME) */
    {50, 6, 0, 0, 1.0f, 0.0f, 1}, /* GMTDIFF_MINUTES (TIME) */
    {60, 1, 0, 0, 1.0f, 0.0f, 0}, /* SUMMER (TIME) */
    {47, 8, 0, 0, 0.5f, 0.0f, 2}, /* AHB_DUTY (AUTO_HIGH_BEAM) */
    {55, 4, 0, 0, 1.0f, 0.0f, 1}, /* F_AHB (AUTO_HIGH_BEAM) */
    {51, 4, 0, 0, 1.0f, 0.0f, 1}, /* C_AHB (AUTO_HIGH_BEAM) */
    {7, 16, 0, 1, 1.0f, 0.0f, 1}, /* YR1Z (VSC1S08) */
    {23, 16, 0, 1, 1.0f, 0.0f, 1}, /* YR2Z (VSC1S08) */
    {39, 8, 0, 1, 0.0359f, 0.0f, 2}, /* GL1Z (VSC1S08) */
    {47, 8, 0, 1, 0.0359f, 0.0f, 2}, /* GL2Z (VSC1S08) */
    {55, 1, 0, 0, 1.0f, 0.0f, 0}, /* YRGSDIR (VSC1S08) */
    {51, 1, 0, 0, 1.0f, 0.0f, 0}, /* GLZS (VSC1S08) */
    {50, 1, 0, 0, 1.0f, 0.0f, 0}, /* YRZF (VSC1S08) */
    {49, 1, 0, 0, 1.0f, 0.0f, 0}, /* YRZS (VSC1S08) */
    {48, 1, 0, 0, 1.0f, 0.0f, 0}, /* YRZKS (VSC1S08) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* VSC08SUM (VSC1S08) */
    {7, 4, 0, 0, 1.0f, 0.0f, 1}, /* STATE (AUTOPARK_STATUS) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* TSGN1 (RSA1) | VAL_ 1 "speed sign" 0 "none" */
    {12, 3, 0, 0, 1.0f, 0.0f, 1}, /* TSGNGRY1 (RSA1) */
    {9, 2, 0, 0, 1.0f, 0.0f, 1}, /* TSGNHLT1 (RSA1) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* SPDVAL1 (RSA1) */
    {31, 4, 0, 0, 1.0f, 0.0f, 1}, /* SPLSGN1 (RSA1) */
    {27, 4, 0, 0, 1.0f, 0.0f, 1}, /* SPLSGN2 (RSA1) | VAL_ 15 "conditional blank" 4 "wet road" 5 "rain" 0 "none" */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* TSGN2 (RSA1) | VAL_ 1 "speed sign" 0 "none" */
    {44, 3, 0, 0, 1.0f, 0.0f, 1}, /* TSGNGRY2 (RSA1) */
    {41, 2, 0, 0, 1.0f, 0.0f, 1}, /* TSGNHLT2 (RSA1) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* SPDVAL2 (RSA1) */
    {63, 2, 0, 0, 1.0f, 0.0f, 1}, /* BZRRQ_P (RSA1) */
    {61, 2, 0, 0, 1.0f, 0.0f, 1}, /* BZRRQ_A (RSA1) */
    {59, 4, 0, 0, 1.0f, 0.0f, 1}, /* SYNCID1 (RSA1) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* TSGN3 (RSA2) | VAL_ 0 "none" 1 "speed sign" 2 "0 unlimited" 7 "unlimited" 16 "highway" 17 "no highway" 18 "motorway" 19 "no motorway" 20 "in city" 21 "outside city" 22 "pedestrian area" 23 "no pedestrian area" 65 "no overtaking left" 66 "no overtaking right" 67 "overtaking allowed again" 81 "no right turn" 97 "stop" 105 "yield" 113 "stop" 114 "yield us" 129 "no entry" 138 "no entry tss2" 145 "do not enter" */
    {12, 3, 0, 0, 1.0f, 0.0f, 1}, /* TSGNGRY3 (RSA2) */
    {9, 2, 0, 0, 1.0f, 0.0f, 1}, /* TSGNHLT3 (RSA2) */
    {31, 4, 0, 0, 1.0f, 0.0f, 1}, /* SPLSGN3 (RSA2) | VAL_ 15 "conditional blank" 4 "wet road" 5 "rain" 0 "none" */
    {27, 4, 0, 0, 1.0f, 0.0f, 1}, /* SPLSGN4 (RSA2) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* TSGN4 (RSA2) */
    {44, 3, 0, 0, 1.0f, 0.0f, 1}, /* TSGNGRY4 (RSA2) */
    {41, 2, 0, 0, 1.0f, 0.0f, 1}, /* TSGNHLT4 (RSA2) */
    {54, 1, 0, 0, 1.0f, 0.0f, 0}, /* DPSGNREQ (RSA2) */
    {53, 3, 0, 0, 1.0f, 0.0f, 1}, /* SGNNUMP (RSA2) */
    {50, 3, 0, 0, 1.0f, 0.0f, 1}, /* SGNNUMA (RSA2) */
    {63, 2, 0, 0, 1.0f, 0.0f, 1}, /* SPDUNT (RSA2) */
    {61, 2, 0, 0, 1.0f, 0.0f, 1}, /* TSRWMSG (RSA2) */
    {59, 4, 0, 0, 1.0f, 0.0f, 1}, /* SYNCID2 (RSA2) */
    {7, 1, 0, 0, 1.0f, 0.0f, 0}, /* TSREQPD (RSA3) */
    {6, 1, 0, 0, 1.0f, 0.0f, 0}, /* TSRMSW (RSA3) */
    {5, 2, 0, 0, 1.0f, 0.0f, 1}, /* OTSGNNTM (RSA3) */
    {3, 2, 0, 0, 1.0f, 0.0f, 1}, /* NTLVLSPD (RSA3) */
    {1, 2, 0, 0, 1.0f, 0.0f, 1}, /* OVSPNTM (RSA3) */
    {11, 4, 0, 0, 1.0f, -5.0f, 1}, /* OVSPVALL (RSA3) */
    {19, 4, 0, 0, 1.0f, -5.0f, 1}, /* OVSPVALM (RSA3) */
    {27, 4, 0, 0, 1.0f, -5.0f, 1}, /* OVSPVALH (RSA3) */
    {33, 2, 0, 0, 1.0f, 0.0f, 1}, /* TSRSPU (RSA3) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_1 (VIN_PART_1) */
    {15, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_2 (VIN_PART_1) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_3 (VIN_PART_1) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_4 (VIN_PART_1) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_5 (VIN_PART_1) */
    {47, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_6 (VIN_PART_1) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_7 (VIN_PART_1) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_8 (VIN_PART_1) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_9 (VIN_PART_2) */
    {15, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_10 (VIN_PART_2) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_11 (VIN_PART_2) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_12 (VIN_PART_2) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_13 (VIN_PART_2) */
    {47, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_14 (VIN_PART_2) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_15 (VIN_PART_2) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_16 (VIN_PART_2) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* VIN_17 (VIN_PART_3) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* UI_SPEED (BODY_CONTROL_STATE_2) */
    {30, 7, 0, 0, 1.0f, 0.0f, 1}, /* METER_SLIDER_BRIGHTNESS_PCT (BODY_CONTROL_STATE_2) */
    {37, 1, 0, 0, 1.0f, 0.0f, 1}, /* METER_SLIDER_LOW_BRIGHTNESS (BODY_CONTROL_STATE_2) | VAL_ 1 "Low brightness mode, footwell lights off" 0 "Normal mode, footwell lights on" */
    {38, 1, 0, 0, 1.0f, 0.0f, 1}, /* METER_SLIDER_DIMMED (BODY_CONTROL_STATE_2) | VAL_ 1 "Dimmed" 0 "Not Dimmed" */
    {63, 3, 0, 0, 1.0f, 0.0f, 1}, /* UNITS (BODY_CONTROL_STATE_2) | VAL_ 1 "km (km/L)" 2 "km (L/100km)" 3 "miles (MPG US)" 4 "miles (MPG Imperial)" */
    {26, 2, 0, 0, 1.0f, 0.0f, 1}, /* UNITS (UI_SETTING) | VAL_ 1 "km" 2 "miles" */
    {39, 32, 0, 0, 1.0f, 0.0f, 1}, /* ODOMETER (UI_SETTING) */
    {15, 1, 0, 0, 1.0f, 0.0f, 1}, /* BLINKER_BUTTON_PRESSED (BLINKERS_STATE) | VAL_ 1 "button pressed" 0 "not pressed" */
    {27, 1, 0, 0, 1.0f, 0.0f, 0}, /* HAZARD_LIGHT (BLINKERS_STATE) */
    {29, 2, 0, 0, 1.0f, 0.0f, 1}, /* TURN_SIGNALS (BLINKERS_STATE) | VAL_ 3 "none" 2 "right" 1 "left" */
    {38, 1, 0, 0, 1.0f, 0.0f, 0}, /* METER_DIMMED (BODY_CONTROL_STATE) */
    {60, 1, 0, 0, 1.0f, 0.0f, 0}, /* PARKING_BRAKE (BODY_CONTROL_STATE) */
    {62, 1, 0, 0, 1.0f, 0.0f, 0}, /* SEATBELT_DRIVER_UNLATCHED (BODY_CONTROL_STATE) */
    {45, 1, 0, 0, 1.0f, 0.0f, 0}, /* DOOR_OPEN_FL (BODY_CONTROL_STATE) */
    {42, 1, 0, 0, 1.0f, 0.0f, 0}, /* DOOR_OPEN_RL (BODY_CONTROL_STATE) */
    {43, 1, 0, 0, 1.0f, 0.0f, 0}, /* DOOR_OPEN_RR (BODY_CONTROL_STATE) */
    {44, 1, 0, 0, 1.0f, 0.0f, 0}, /* DOOR_OPEN_FR (BODY_CONTROL_STATE) */
    {37, 1, 0, 0, 1.0f, 0.0f, 0}, /* AUTO_HIGH_BEAM (LIGHT_STALK) */
    {27, 1, 0, 0, 1.0f, 0.0f, 0}, /* FRONT_FOG (LIGHT_STALK) */
    {28, 1, 0, 0, 1.0f, 0.0f, 0}, /* PARKING_LIGHT (LIGHT_STALK) */
    {29, 1, 0, 0, 1.0f, 0.0f, 0}, /* LOW_BEAM (LIGHT_STALK) */
    {30, 1, 0, 0, 1.0f, 0.0f, 0}, /* HIGH_BEAM (LIGHT_STALK) */
    {31, 1, 0, 0, 1.0f, 0.0f, 0}, /* DAYTIME_RUNNING_LIGHT (LIGHT_STALK) */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* DOOR_LOCK_FEEDBACK_LIGHT (CERTIFICATION_ECU) */
    {61, 1, 0, 0, 1.0f, 0.0f, 0}, /* KEYFOB_LOCKING_FEEDBACK_LIGHT (CERTIFICATION_ECU) */
    {62, 1, 0, 0, 1.0f, 0.0f, 0}, /* KEYFOB_UNLOCKING_FEEDBACK_LIGHT (CERTIFICATION_ECU) */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* LOCK_STATUS_CHANGED (DOOR_LOCKS) */
    {20, 1, 0, 0, 1.0f, 0.0f, 1}, /* LOCK_STATUS (DOOR_LOCKS) | VAL_ 0 "locked" 1 "unlocked" */
    {23, 1, 0, 0, 1.0f, 0.0f, 0}, /* LOCKED_VIA_KEYFOB (DOOR_LOCKS) */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* OK_BUTTON_PRESSED (ADAS_TOGGLE_STATE) */
    {24, 1, 0, 0, 1.0f, 0.0f, 0}, /* SWS_TOGGLE_CMD (ADAS_TOGGLE_STATE) */
    {26, 2, 0, 0, 1.0f, 0.0f, 1}, /* SWS_SENSITIVITY_CMD (ADAS_TOGGLE_STATE) */
    {28, 1, 0, 0, 1.0f, 0.0f, 0}, /* LKAS_ON_CMD (ADAS_TOGGLE_STATE) */
    {29, 1, 0, 0, 1.0f, 0.0f, 0}, /* LKAS_OFF_CMD (ADAS_TOGGLE_STATE) */
    {30, 1, 0, 0, 1.0f, 0.0f, 0}, /* LDA_SENSITIVITY_HI_CMD (ADAS_TOGGLE_STATE) */
    {31, 1, 0, 0, 1.0f, 0.0f, 0}, /* LDA_SENSITIVITY_STD_CMD (ADAS_TOGGLE_STATE) */
    {34, 1, 0, 0, 1.0f, 0.0f, 0}, /* IPAS_TOGGLE (ADAS_TOGGLE_STATE) */
    {37, 1, 0, 0, 1.0f, 0.0f, 0}, /* BSM_TOGGLE_CMD (ADAS_TOGGLE_STATE) */
    {38, 1, 0, 0, 1.0f, 0.0f, 0}, /* IPAS_SONAR_TOGGLE (ADAS_TOGGLE_STATE) */
    {40, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCS_TOGGLE_CMD (ADAS_TOGGLE_STATE) */
    {41, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCS_SENSITIVITY_CMD (ADAS_TOGGLE_STATE) */
    {7, 48, 0, 0, 1.0f, 0.0f, 1}, /* RANDOM_MSG (DISABLE_HACKED_PANDA) */
    {7, 48, 0, 0, 1.0f, 0.0f, 1}, /* RANDOM_MSG (ENABLE_HACKED_PANDA) */
    {7, 4, 0, 0, 1.0f, 0.0f, 1}, /* STATE (STEERING_IPAS_COMMA) */
    {3, 12, 0, 1, 1.5f, 0.0f, 2}, /* ANGLE (STEERING_IPAS_COMMA) */
    {23, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X10 (STEERING_IPAS_COMMA) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X00 (STEERING_IPAS_COMMA) */
    {38, 2, 0, 0, 1.0f, 0.0f, 1}, /* DIRECTION_CMD (STEERING_IPAS_COMMA) */
    {47, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X40 (STEERING_IPAS_COMMA) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X00_1 (STEERING_IPAS_COMMA) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (STEERING_IPAS_COMMA) */
    {4, 1, 0, 0, 1.0f, 0.0f, 0}, /* GAS_RELEASED (PCM_CRUISE) */
    {5, 1, 0, 0, 1.0f, 0.0f, 0}, /* CRUISE_ACTIVE (PCM_CRUISE) */
    {12, 1, 0, 0, 1.0f, 0.0f, 0}, /* ACC_BRAKING (PCM_CRUISE) */
    {23, 16, 0, 1, 0.0009765625f, 0.0f, 2}, /* ACCEL_NET (PCM_CRUISE) */
    {39, 16, 0, 1, 2.0f, 0.0f, 1}, /* NEUTRAL_FORCE (PCM_CRUISE) */
    {55, 4, 0, 0, 1.0f, 0.0f, 1}, /* CRUISE_STATE (PCM_CRUISE) | VAL_ 11 "timer_3sec" 10 "adaptive click down" 9 "adaptive click up" 8 "adaptive engaged" 7 "standstill" 6 "non-adaptive click up" 5 "non-adaptive click down" 4 "non-adaptive hold down" 3 "non-adaptive hold up" 2 "non-adaptive being engaged" 1 "non-adaptive engaged" 0 "off" */
    {49, 1, 1, 0, 1.0f, 0.0f, 0}, /* CANCEL_REQ (PCM_CRUISE) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (PCM_CRUISE) */
    {7, 8, 0, 0, 1.0f, 0.0f, 1}, /* _COUNTER (PRE_COLLISION) */
    {15, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X00 (PRE_COLLISION) */
    {23, 16, 0, 1, 2.0f, 0.0f, 1}, /* FORCE (PRE_COLLISION) */
    {33, 8, 0, 0, 1.0f, 0.0f, 1}, /* SET_ME_X002 (PRE_COLLISION) */
    {39, 3, 0, 0, 1.0f, 0.0f, 1}, /* BRAKE_STATUS (PRE_COLLISION) */
    {36, 3, 0, 0, 1.0f, 0.0f, 1}, /* STATE (PRE_COLLISION) | VAL_ 0 "normal" 1 "adaptive_cruise_control" 3 "emergency_braking" */
    {40, 1, 0, 0, 1.0f, 0.0f, 0}, /* SET_ME_X003 (PRE_COLLISION) */
    {41, 1, 0, 0, 1.0f, 0.0f, 0}, /* PRECOLLISION_ACTIVE (PRE_COLLISION) */
    {55, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (PRE_COLLISION) */
    {3, 1, 0, 0, 1.0f, 0.0f, 0}, /* GAS_RELEASED (GAS_PEDAL) */
    {15, 16, 0, 1, 0.03125f, 0.0f, 2}, /* ETQLVSC (GAS_PEDAL) */
    {31, 16, 0, 1, 0.03125f, 0.0f, 2}, /* ETQREAL (GAS_PEDAL) */
    {47, 8, 0, 0, 1.0f, -192.0f, 1}, /* ETQISC (GAS_PEDAL) */
    {55, 8, 0, 0, 0.5f, 0.0f, 2}, /* GAS_PEDAL (GAS_PEDAL) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (GAS_PEDAL) */
    {31, 8, 0, 0, 1.0f, 0.0f, 1}, /* LKA_STATE (STEERING_LKA) */
    {0, 1, 0, 0, 1.0f, 0.0f, 0}, /* STEER_REQUEST (STEERING_LKA) */
    {6, 6, 0, 0, 1.0f, 0.0f, 1}, /* COUNTER (STEERING_LKA) */
    {7, 1, 0, 0, 1.0f, 0.0f, 0}, /* SET_ME_1 (STEERING_LKA) */
    {15, 16, 0, 1, 1.0f, 0.0f, 1}, /* STEER_TORQUE_CMD (STEERING_LKA) */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (STEERING_LKA) */
    {7, 10, 0, 1, 0.1f, 0.0f, 2}, /* DSS1GDRV (PRE_COLLISION_2) */
    {17, 1, 0, 0, 1.0f, 0.0f, 0}, /* PCSALM (PRE_COLLISION_2) */
    {27, 1, 0, 0, 1.0f, 0.0f, 0}, /* IBTRGR (PRE_COLLISION_2) */
    {30, 2, 0, 0, 1.0f, 0.0f, 1}, /* PBATRGR (PRE_COLLISION_2) */
    {33, 1, 0, 0, 1.0f, 0.0f, 0}, /* PREFILL (PRE_COLLISION_2) */
    {36, 1, 0, 0, 1.0f, 0.0f, 0}, /* AVSTRGR (PRE_COLLISION_2) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (PRE_COLLISION_2) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (STEERING_LTA) */
    {29, 2, 0, 0, 1.0f, 0.0f, 1}, /* SETME_X3 (STEERING_LTA) | VAL_ 3 "TSS 2.0" 1 "TSS 2.5 or 2022 RAV4" 0 "TSS 2.0 on Alphard, Highlander, NX" */
    {39, 8, 0, 0, 1.0f, 0.0f, 1}, /* PERCENTAGE (STEERING_LTA) */
    {47, 8, 0, 0, 1.0f, 0.0f, 1}, /* TORQUE_WIND_DOWN (STEERING_LTA) */
    {55, 8, 0, 1, 0.5f, 0.0f, 2}, /* ANGLE (STEERING_LTA) */
    {15, 16, 0, 1, 0.0573f, 0.0f, 2}, /* STEER_ANGLE_CMD (STEERING_LTA) */
    {25, 1, 0, 0, 1.0f, 0.0f, 0}, /* STEER_REQUEST_2 (STEERING_LTA) */
    {26, 1, 0, 0, 1.0f, 0.0f, 0}, /* LKA_ACTIVE (STEERING_LTA) */
    {30, 1, 0, 0, 1.0f, 0.0f, 0}, /* CLEAR_HOLD_STEERING_ALERT (STEERING_LTA) */
    {6, 6, 0, 0, 1.0f, 0.0f, 1}, /* COUNTER (STEERING_LTA) */
    {0, 1, 0, 0, 1.0f, 0.0f, 0}, /* STEER_REQUEST (STEERING_LTA) */
    {7, 1, 0, 0, 1.0f, 0.0f, 0}, /* SETME_X1 (STEERING_LTA) */
    {0, 9, 0, 0, 1.0f, 0.0f, 1}, /* BRAKE_PRESSURE (BRAKE_MODULE) */
    {16, 9, 0, 0, 1.0f, 0.0f, 1}, /* BRAKE_POSITION (BRAKE_MODULE) */
    {37, 1, 0, 0, 1.0f, 0.0f, 0}, /* BRAKE_PRESSED (BRAKE_MODULE) */
    {3, 4, 0, 0, 1.0f, 0.0f, 1}, /* IPAS_STATE (EPS_STATUS) | VAL_ 5 "override" 3 "enabled" 1 "disabled" */
    {31, 7, 0, 0, 1.0f, 0.0f, 1}, /* LKA_STATE (EPS_STATUS) | VAL_ 25 "temporary_fault" 17 "permanent_fault" 11 "lka_missing_unavailable2" 9 "temporary_fault2" 5 "active" 3 "lka_missing_unavailable" 1 "standby" */
    {15, 5, 0, 0, 1.0f, 0.0f, 1}, /* LTA_STATE (EPS_STATUS) | VAL_ 25 "temporary_fault" 9 "temporary_fault2" 5 "active" 3 "lta_missing_unavailable" 1 "standby" */
    {24, 1, 0, 0, 1.0f, 0.0f, 0}, /* TYPE (EPS_STATUS) */
    {63, 8, 0, 0, 1.0f, 0.0f, 1}, /* CHECKSUM (EPS_STATUS) */
    {15, 8, 0, 0, 0.005f, 0.0f, 2}, /* GAS_PEDAL (LTA_RELATED) */
    {23, 16, 0, 1, 0.0573f, 0.0f, 2}, /* STEER_ANGLE (LTA_RELATED) */
    {35, 2, 0, 0, 1.0f, 0.0f, 1}, /* TURN_SIGNALS (LTA_RELATED) */
    {58, 1, 0, 0, 1.0f, 0.0f, 0}, /* UNKNOWN_2 (LTA_RELATED) */
    {59, 1, 0, 0, 1.0f, 0.0f, 0}, /* LDA_SA_TOGGLE (LTA_RELATED) */
    {60, 1, 0, 0, 1.0f, 0.0f, 0}, /* LTA_STEER_REQUEST (LTA_RELATED) */
    {61, 1, 0, 0, 1.0f, 0.0f, 0}, /* UNKNOWN (LTA_RELATED) */
    {63, 1, 0, 0, 1.0f, 0.0f, 0}, /* STEERING_PRESSED (LTA_RELATED) */
    {0, 1, 0, 0, 1.0f, 0.0f, 0}, /* L_ADJACENT (BSM) */
    {8, 1, 0, 0, 1.0f, 0.0f, 0}, /* L_APPROACHING (BSM) */
    {1, 1, 0, 0, 1.0f, 0.0f, 0}, /* R_ADJACENT (BSM) */
    {10, 1, 0, 0, 1.0f, 0.0f, 0}, /* R_APPROACHING (BSM) */
    {7, 1, 0, 0, 1.0f, 0.0f, 0}, /* ADJACENT_ENABLED (BSM) */
    {15, 1, 0, 0, 1.0f, 0.0f, 0}, /* APPROACHING_ENABLED (BSM) */
#endif /* INCLUDE_CAN_DESCRIPTORS_H_ */
};

