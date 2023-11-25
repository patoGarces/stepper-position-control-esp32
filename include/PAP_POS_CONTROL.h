#include "stdint.h"

#define MIN_VELOCITY_US 3000
#define MAX_VELOCITY_US 500     

#define VEL_PERCENT_DEFAULT 25  // velocidad por defecto al iniciar los motores

/* Cantidad de ticks hasta que se dispare la interrupcion */
#define MOTOR_PERIOD_US     1000

/* Definicion en la rampa, la velocidad tiene 1000 puntos de definicion*/
#define RAMP_DEFINITION     100
#define PERIOD_RAMP_HANDLER 10

#define RAMP_MAX_ACCEL 5

#define VAL_RAMP_PERCENT 0.1

// #define AVOID_ACCEL_RAMP 

#define MOTOR_ENABLE    0
#define MOTOR_DISABLE   1

#define CANT_MOTORS     3

enum motor_name {
    MOTOR_A,
    MOTOR_B,
    MOTOR_C
};

enum ramp_state {
    RAMP_UP,
    RAMP_MESETA,
    RAMP_DOWN,
    RAMP_END
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
    // uint16_t dxdt;
    float accelMax;
}individual_ramp_t;

typedef struct{
    uint32_t actualTime;
    uint32_t rampUpDownTime;
    uint32_t constantRampTime;
    uint32_t rampTotalTime;
    uint16_t period;
    uint8_t  stateRamp;
    individual_ramp_t individualRamp[CANT_MOTORS];
}control_ramp_t;

void initMotors(output_motors_pins_t pinout);
void moveAxis(int32_t stepsA,int32_t stepsB,int32_t stepsC);
void setVel(uint8_t velocity);
void setEnableMotors(void);
void setDisableMotors(void);
uint8_t getVelPercent(void);
