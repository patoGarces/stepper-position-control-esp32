#ifndef INC_PAP_POS_CONTROL_H
#define INC_PAP_POS_CONTROL_H

#include "stdint.h"

#define MIN_VELOCITY_US 3000
#define MAX_VELOCITY_US 500     

#define VEL_PERCENT_DEFAULT 25  // velocidad por defecto al iniciar los motores

/* Definicion en la rampa, la velocidad tiene 1000 puntos de definicion*/
#define RAMP_DEFINITION     100

#define BASE_PERIOD_TIMER   10

#define RAMP_MAX_ACCEL      5

#define VAL_RAMP_PERCENT    0.1

#define AVOID_ACCEL_RAMP 

#define MOTOR_ENABLE        0
#define MOTOR_DISABLE       1

#define CANT_MOTORS         3

#define QUEUE_MOVES_LENGTH          100
#define DELAY_BETWEEN_MOVES         10                  // en us
#define INTERPOLATION_PRECISION     1000.00

// AutoHome
#define DIRECTION_SEARCH_HOME       0

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

enum move_axis_response {
    MOVE_AXIS_ERROR_LIMITS_EXCEEDED,
    MOVE_AXIS_ERROR_LEN_QUEUE_EXCEEDED,
    MOVE_AXIS_OK,
};


typedef struct{
    uint8_t stepPin;
    uint8_t dirPin;
}output_motor_pins_t;

typedef struct{
    uint8_t pinSensor[CANT_MOTORS];
}input_sensor_pins_t;

typedef struct{
    uint8_t             enablePin;
    output_motor_pins_t motors[CANT_MOTORS];
}output_motors_pins_t;

typedef struct{
    int32_t safetyLimit[CANT_MOTORS];
}safety_limits_t;

typedef struct{
    output_motors_pins_t    motorsGpio;
    input_sensor_pins_t     endOfTravelsGpio;
    safety_limits_t         safetyLimits;      
}pap_position_control_config_t;

typedef struct{
    uint8_t     dir;
    uint16_t    velocityUs;
    uint32_t    actualSteps;
    uint32_t    totalSteps;
    uint8_t     flagRunning;
    uint8_t     flagToggle;
    uint32_t    actualTicks;
}motor_control_t;

typedef struct{
    uint8_t                 motorVelPercent;
    uint8_t                 motorsEnable;
    int32_t                 absolutePosition[CANT_MOTORS];
    motor_control_t         motorsControl[CANT_MOTORS];
}motors_control_t;

typedef struct{
    uint32_t actualVelUs;
    uint32_t targetVelUs;
    uint32_t dxdt;
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

typedef struct{
    int32_t absPosition[CANT_MOTORS];
}absolute_position_t;

void initMotors(pap_position_control_config_t config);
uint8_t moveAxis(int32_t stepsA,int32_t stepsB,int32_t stepsC);
void setVel(uint8_t velocity);
void setEnableMotors(void);
void setDisableMotors(void);
uint8_t getVelPercent(void);
void resetAbsPosition(void);
absolute_position_t getAbsPosition(void);
uint8_t areMotorsMoving(void);
void autoHome(void);

#endif 
