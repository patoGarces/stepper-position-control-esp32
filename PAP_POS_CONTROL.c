#include "include/PAP_POS_CONTROL.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

/* Cantidad de ticks hasta que se dispare la interrupcion */
#define MOTOR_PERIOD_US     1000

#define VAL_RAMP_PERCENT 0.1

QueueHandle_t       handleMoveAxis;
SemaphoreHandle_t   handleSyncMovement;
motors_control_t    outputMotors;
control_ramp_t      rampMotors;
gptimer_handle_t    handleTimer = NULL;

static void reloadAlarm( uint8_t velocity );
static void rampHandler(uint8_t motor, uint32_t actualCont, uint32_t totalSteps);
static void setRampa(void);

static bool IRAM_ATTR timerInterrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    uint8_t motor;
    BaseType_t high_task_awoken = pdFALSE;

    for(motor=0;motor<3;motor++){

        if( outputMotors.motorsControl[motor].flagRunning ){

            if(outputMotors.motorsControl[motor].contMotor < outputMotors.motorsControl[motor].stepsMotor){

                outputMotors.motorsControl[motor].contMotor++;
                gpio_set_level(outputMotors.motorsGpio.motors[motor].stepPin,outputMotors.motorsControl[motor].flagToggle);
                outputMotors.motorsControl[motor].flagToggle = !outputMotors.motorsControl[motor].flagToggle;  

                rampHandler(motor,outputMotors.motorsControl[motor].contMotor,outputMotors.motorsControl[motor].stepsMotor);
            }
            else{ 
                outputMotors.motorsControl[motor].flagRunning = false;
            }
        }
    }
        
    return (high_task_awoken == pdTRUE);
}

static void setRampa( void ){

    rampMotors.actualVel = 1;    
    rampMotors.targetVel = outputMotors.motorVelPercent;     
    reloadAlarm(1);
    rampMotors.stateRamp = RAMP_UP;
    
    rampMotors.distRampSteps[0] = outputMotors.motorsControl[0].stepsMotor * VAL_RAMP_PERCENT;

    rampMotors.period = rampMotors.distRampSteps[0]/100;                   // el periodo es la cantidad de pasos dividido el 100% de velocidad total
    
    if(rampMotors.period == 0){
      rampMotors.period = 1;
    }
}

static void rampHandler(uint8_t motor, uint32_t actualCont, uint32_t totalSteps){

    switch( rampMotors.stateRamp ){

        case RAMP_UP:
            
        //   if( motor.countSteps < rampa.distRampaSteps){           // dentro de la rampa de subida, debo chequear si no me estoy solapando con la rampa de bajada
        //     rampa.stateRampa = RAMPA_BAJADA;
        //   }
            
            if(( actualCont % rampMotors.period) == 0){
            
            if( rampMotors.actualVel < rampMotors.targetVel){
                rampMotors.actualVel++;
                reloadAlarm( rampMotors.actualVel ); 
            }
            else{
                rampMotors.stateRamp = RAMP_MESETA;
            }
            }
        break;

        case RAMP_MESETA:

            if( actualCont > ( outputMotors.motorsControl[0].stepsMotor - rampMotors.distRampSteps[0])){
                rampMotors.stateRamp = RAMP_DOWN;
            }

        break;

        case RAMP_DOWN:

            if(( actualCont % rampMotors.period) == 0){

                if( rampMotors.actualVel > 1){
                    rampMotors.actualVel--;
                    reloadAlarm( rampMotors.actualVel ); 
                }
                else{
                    rampMotors.stateRamp = RAMP_MESETA;
                }
            }
        break;
  }
}

static void handlerQueueAxis(void *pvParameters){

    motor_control_t receiveNewMovement[3];
    uint8_t motor;

    handleMoveAxis = xQueueCreate(100,sizeof(motors_control_t));

    while(1){

        if( !outputMotors.motorsControl[MOTOR_A].flagRunning &&
            !outputMotors.motorsControl[MOTOR_B].flagRunning &&
            !outputMotors.motorsControl[MOTOR_C].flagRunning ) {
             
            if(xQueueReceive( handleMoveAxis,
                ( void * ) &receiveNewMovement,
                0)){

                for( motor = 0; motor < 3; motor++ ){
                    outputMotors.motorsControl[motor] = receiveNewMovement[motor];
                    gpio_set_level(outputMotors.motorsGpio.motors[motor].dirPin,outputMotors.motorsControl[motor].dir); 
                }
                setRampa();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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

    setDisableMotors();                                     // Inician los motores deshabilitados

    gptimer_config_t timerConfig = {

        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimer));
    
    gptimer_event_callbacks_t newCallback ={
        .on_alarm = timerInterrupt,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleTimer,&newCallback,NULL));
    
    outputMotors.motorVelPercent = VEL_PERCENT_DEFAULT;
    setVel(outputMotors.motorVelPercent);                   // inician los motores con la velocidad default

    ESP_ERROR_CHECK(gptimer_enable(handleTimer));
    ESP_ERROR_CHECK(gptimer_start(handleTimer));

    xTaskCreate(handlerQueueAxis,"axis Handler",2048,NULL,4,NULL);
}

static void reloadAlarm( uint8_t velocity ){
    uint16_t velocityUs = 0;

    if(velocity > 0 && velocity <= 100){
        velocityUs = MAX_VELOCITY_US + (((MIN_VELOCITY_US-MAX_VELOCITY_US)*(101-velocity))/100);

        // printf("VelMinUs: %dus,VelMaxUs: %dus,velocity: %d%%,salida: %d\n",MIN_VELOCITY_US,MAX_VELOCITY_US,velocity,outputMotors.motorVelUs);

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = velocityUs,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        gptimer_set_alarm_action(handleTimer, &alarm_config);
    } 
}

void moveAxis(uint8_t dirA,uint32_t stepsA,uint8_t dirB,uint32_t stepsB,uint8_t dirC,uint32_t stepsC,uint16_t duration){          // TODO: encolar estas solicitudes en una estructura de tipo motor_control_t
    
    motor_control_t newMovement[3];

    newMovement[MOTOR_A].dir = dirA;
    newMovement[MOTOR_A].durationMs = duration;
    newMovement[MOTOR_A].stepsMotor = stepsA *2;
    newMovement[MOTOR_A].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_A].flagRunning = true;

    newMovement[MOTOR_B].dir = dirB;
    newMovement[MOTOR_B].durationMs = duration;
    newMovement[MOTOR_B].stepsMotor = stepsB *2;
    newMovement[MOTOR_B].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_B].flagRunning = true;

    newMovement[MOTOR_C].dir = dirC;
    newMovement[MOTOR_C].durationMs = duration;
    newMovement[MOTOR_C].stepsMotor = stepsC *2;
    newMovement[MOTOR_C].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_C].flagRunning = true;

    xQueueSend(handleMoveAxis,&newMovement,0);
    printf("Nuevo movimiento agregado a la cola,pendientes: %d\n",uxQueueMessagesWaiting( handleMoveAxis )); 
}

void setVel(uint8_t velocity){

    if(velocity > 0 && velocity <= 100){
        outputMotors.motorVelPercent = velocity;
        reloadAlarm(velocity);    
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