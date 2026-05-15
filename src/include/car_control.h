#ifndef INCLUDE_CAR_CONTROL_H_
#define INCLUDE_CAR_CONTROL_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <stdbool.h>

#define ID_VCU_ACCEL_CMD (0x100)
#define ID_VCU_STEER_CMD (0x101)
#define ID_VCU_SAFETY_CMD (0x102)
#define ID_VCU_GEAR_CMD (0x103)
#define ID_VCU_BODY_CMD (0x104)

typedef enum{
    Gear_NONE,
    PARK,
    REVERSE,
    NEUTRAL,
    DRIVE,
    LOW,
    Gear_UNKNOWN
}GearCmd;

typedef enum{
    VisualAlertNone,
    VisualAlertFcw,
    VisualAlertSteerRequired,
    VisualAlertLdw
} VisualAlert;

typedef struct {
    VisualAlert visualAlert;
    bool leadVisible;
    uint8_t leadDistanceBars;
} HudControl;

typedef enum{
    Turn_NONE,
    DISABLE,
    LEFT,
    RIGHT,
    Turn_UNKNOWN
} TurnIndicatorCmd;

typedef enum{
    off,
    pid,
    stopping,
    starting
} LongControlState;

typedef struct{
    float accelCmd; //for accel setpoint
    float torqueCmd; //lkas steer setpoint, no use
    float steeringAngleDegCmd; //ipas steerangle setpoint
    LongControlState longcontrolstate; //no use
} Actuators;

typedef struct {
    Actuators actuators;
    HudControl hudControl;
    float throttleCmd; // no use
    float brakeCmd; //no use
    bool accEnable; // lower level accCmd enable 
    bool emergency; //no use
    bool steeringEnable; //ipasCmd enable
    bool vcuEnabled; //entire board
    bool cancelReq; //no use
    bool longActive; //higher acc flag
    bool latActive; //higher ipas and lkas flag
    GearCmd gearCmd; //no use
    TurnIndicatorCmd turnIndicator; //no use
    bool hazardLights; //no use
    TickType_t last_update_tick;
} CarControl;




#endif /* INCLUDE_CAR_CONTROL_H_ */
