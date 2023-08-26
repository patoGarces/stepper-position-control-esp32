#include "stdint.h"

#define NO_DURATION     -1
#define MIN_VELOCITY_US 2000
#define MAX_VELOCITY_US 500     

#define VEL_PERCENT_DEFAULT 10  // velocidad por defecto al iniciar los motores

#define MOTOR_ENABLE    0
#define MOTOR_DISABLE   1

#define MOT_DIR_CCW     1
#define MOT_DIR_CW      0

#define CANT_MOTORS     3

enum motor_name {
    MOTOR_A,
    MOTOR_B,
    MOTOR_C
};

typedef struct{
    uint8_t stepPin;
    uint8_t dirPin;
}output_motor_pins_t;

typedef struct{
    uint8_t             enablePin;
    output_motor_pins_t motors[CANT_MOTORS];
}output_motors_pins_t;

typedef struct{
    uint32_t    contMotor;
    uint32_t    stepsMotor;
    uint16_t    durationMs;
    uint8_t     dir;
    uint8_t     flagRunning;
    uint8_t     flagToggle;
}motor_control_t;

// typedef struct{
//     motor_control_t movements[3];
// }new_movement_motor_t;

typedef struct{
    uint8_t                 motorVelPercent;
    uint16_t                motorVelUs;
    uint16_t                motorVelContUs;
    uint8_t                 motorsEnable;
    output_motors_pins_t    motorsGpio;
    motor_control_t         motorsControl[CANT_MOTORS];
}motors_control_t;

void setControlPins(uint8_t outputMotor,uint8_t enablePin,uint8_t stepPin,uint8_t dirPin);
void initMotors(void);
void moveAxis(uint8_t dirA,uint32_t stepsA,uint8_t dirB,uint32_t stepsB,uint8_t dirC,uint32_t stepsC,uint16_t duration);
void setVel(uint8_t velocity);
void setEnableMotors(void);
void setDisableMotors(void);
uint8_t getVelPercent(void);