#include "PAP_CONTROL.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"

#define TIMER_GROUP_MOTOR   TIMER_GROUP_0
#define TIMER_NUM_MOTOR     TIMER_0
/* Cantidad de ticks hasta que se dispare la interrupcion */
#define MOTOR_PERIOD_US     10


// uint16_t                outputVelUs = MIN_VELOCITY_US;      // Periodo de cada pulso en us/10
// uint16_t                outputVelContUs = 0;

motors_control_t    outputMotors;

static bool IRAM_ATTR timerInterrupt(void * args) {
    uint8_t motor;
    BaseType_t high_task_awoken = pdFALSE;

    if(outputMotors.motorVelContUs < outputMotors.motorVelUs){
        outputMotors.motorVelContUs++;
    }
    else{
        outputMotors.motorVelContUs = 0;
        for(motor=0;motor<3;motor++){

            if(outputMotors.motorsControl[motor].flagEnable){
                if(outputMotors.motorsControl[motor].contMotor < outputMotors.motorsControl[motor].stepsMotor){
                    outputMotors.motorsControl[motor].contMotor++;

                    gpio_set_level(outputMotors.motorsGpio.motors[motor].stepPin,outputMotors.motorsControl[motor].flagToggle);
                    outputMotors.motorsControl[motor].flagToggle = !outputMotors.motorsControl[motor].flagToggle;  
                }
                else{
                    outputMotors.motorsControl[motor].flagEnable = false;
                }
            }
        }
    }
        
    return (high_task_awoken == pdTRUE);
}

void setControlPins(uint8_t outputMotor,uint8_t enablePin,uint8_t stepPin,uint8_t dirPin){

    outputMotors.motorsGpio.enablePin = enablePin;
    outputMotors.motorsGpio.motors[outputMotor].dirPin = dirPin;
    outputMotors.motorsGpio.motors[outputMotor].stepPin = stepPin;

    gpio_set_direction( outputMotors.motorsGpio.enablePin,GPIO_MODE_OUTPUT );
    gpio_set_direction( outputMotors.motorsGpio.motors[outputMotor].dirPin,GPIO_MODE_OUTPUT );
    gpio_set_direction( outputMotors.motorsGpio.motors[outputMotor].stepPin = stepPin,GPIO_MODE_OUTPUT );
}

void initMotors(void){

    outputMotors.motorVelContUs = 0;
    outputMotors.motorVelPercent = VEL_PERCENT_DEFAULT;
    setVel(outputMotors.motorVelPercent);                   // inician los motores con la velocidad default
    setDisableMotors();                                     // Inician los motores deshabilitados


    /*Inicializo el timer encargado de generar los pulsos para cada motor*/
    timer_config_t timer = {
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,                                      // 80mhz / 80 = 1MHZ = 1us      <- Resolucion de cada tick del timer
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
    };

    timer_init(TIMER_GROUP_MOTOR,TIMER_NUM_MOTOR,&timer);
    timer_set_alarm_value(TIMER_GROUP_MOTOR,TIMER_NUM_MOTOR, MOTOR_PERIOD_US);      // Desborde de interrupcion: 1us de cada tick * MOTOR_PERIOD_US = 10us
    timer_isr_callback_add(TIMER_GROUP_MOTOR,TIMER_NUM_MOTOR,timerInterrupt,NULL, ESP_INTR_FLAG_IRAM );
    timer_set_counter_value(TIMER_GROUP_MOTOR,TIMER_NUM_MOTOR,0);
    timer_enable_intr(TIMER_GROUP_MOTOR, TIMER_NUM_MOTOR);
    timer_start(TIMER_GROUP_MOTOR,TIMER_NUM_MOTOR);
}

void moveAxis(uint8_t motor,uint8_t dir,uint32_t steps,uint16_t duration){
    
    outputMotors.motorsControl[motor].dir = dir;
    outputMotors.motorsControl[motor].durationMs = duration;
    outputMotors.motorsControl[motor].stepsMotor = steps*2;             // Multiplico por 2 ya que son pulsos, y yo cuento cambios de estado en el pin
    outputMotors.motorsControl[motor].contMotor = 0;
    outputMotors.motorsControl[motor].flagEnable = true;

    gpio_set_level(outputMotors.motorsGpio.motors[motor].dirPin,dir);
}

void setVel(uint8_t velocity){

    if(velocity > 0 && velocity <= 100){
        outputMotors.motorVelPercent = velocity;
        outputMotors.motorVelUs = MAX_VELOCITY_US + (((MIN_VELOCITY_US-MAX_VELOCITY_US)*(100-velocity))/100);
        outputMotors.motorVelUs /= 10;

        printf("VelMinUs: %dus,VelMaxUs: %dus,velocity: %d%%,salida: %d\n",MIN_VELOCITY_US,MAX_VELOCITY_US,velocity,outputMotors.motorVelUs);
    }
}

void setEnableMotors(void){
    outputMotors.motorsEnable = MOTOR_ENABLE;
    gpio_set_level(outputMotors.motorsGpio.enablePin,outputMotors.motorsEnable);
}

void setDisableMotors(void){
    outputMotors.motorsEnable = MOTOR_DISABLE;
    gpio_set_level(outputMotors.motorsGpio.enablePin,outputMotors.motorsEnable);
}

uint8_t getVelPercent(void){
    return outputMotors.motorVelPercent;
}