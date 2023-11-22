#include "stdint.h"

#define NO_DURATION     -1
#define MIN_VELOCITY_US 1000
#define MAX_VELOCITY_US 100     

#define VEL_PERCENT_DEFAULT 25  // velocidad por defecto al iniciar los motores

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

enum ramp_state {
    RAMP_UP,
    RAMP_MESETA,
    RAMP_DOWN
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
    uint16_t    velocityUs;
    uint8_t     dir;
    uint32_t    contMotor;
    uint32_t    stepsMotor;
    uint8_t     flagRunning;
    uint8_t     flagToggle;
}motor_control_t;

typedef struct{
    uint8_t                 motorVelPercent;
    uint8_t                 motorsEnable;
    output_motors_pins_t    motorsGpio;
    motor_control_t         motorsControl[CANT_MOTORS];
}motors_control_t;

typedef struct{
    uint16_t actualVelUs;
    uint16_t targetVelUs;
    uint8_t  stateRamp;
    uint32_t distRampSteps;
    uint16_t period;
    uint16_t dxdt;
}control_ramp_t;

void initMotors(output_motors_pins_t pinout);
void moveAxis(int32_t stepsA,int32_t stepsB,int32_t stepsC);
void setVel(uint8_t velocity);
void setEnableMotors(void);
void setDisableMotors(void);
uint8_t getVelPercent(void);
