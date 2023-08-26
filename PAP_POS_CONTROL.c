#include "include/PAP_POS_CONTROL.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

#define TIMER_GROUP_MOTOR   TIMER_GROUP_0
#define TIMER_NUM_MOTOR     TIMER_0
/* Cantidad de ticks hasta que se dispare la interrupcion */
#define MOTOR_PERIOD_US     45

QueueHandle_t       handleMoveAxis;
SemaphoreHandle_t   handleSyncMovement;
motors_control_t    outputMotors;

static bool IRAM_ATTR timerInterrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    uint8_t motor;
    BaseType_t high_task_awoken = pdFALSE;

    if(outputMotors.motorVelContUs < outputMotors.motorVelUs){          // TODO: eliminar esta logica, manejar la velocidad con el periodo de la alarma
        outputMotors.motorVelContUs++;
    }
    else{
        outputMotors.motorVelContUs = 0;
        for(motor=0;motor<3;motor++){

            if(outputMotors.motorsControl[motor].flagRunning){
                if(outputMotors.motorsControl[motor].contMotor < outputMotors.motorsControl[motor].stepsMotor){
                    
                    outputMotors.motorsControl[motor].contMotor++;
                    gpio_set_level(outputMotors.motorsGpio.motors[motor].stepPin,outputMotors.motorsControl[motor].flagToggle);
                    outputMotors.motorsControl[motor].flagToggle = !outputMotors.motorsControl[motor].flagToggle;  
                }
                else{ 
                    outputMotors.motorsControl[motor].flagRunning = false;
                }
            }
        //     else{
        //         finishMovement[motor] = true;

        //         // if( finishMovement[0] && finishMovement[1] && finishMovement[2]){
        //         //     // xSemaphoreGiveFromISR(handleSyncMovement,1);
        //         //     // superFinishMovement = true;
        //         // }
        //     }
        // }
        // if( finishMovement[0] && finishMovement[1] && finishMovement[2]){
        //     // xSemaphoreGiveFromISR(handleSyncMovement,1);
        //     superFinishMovement = true;
        }
    }
        
    return (high_task_awoken == pdTRUE);
}

static void handlerQueueAxis(void *pvParameters){

    new_movement_motor_t receiveNewMovement;

    handleMoveAxis = xQueueCreate(100,sizeof(motors_control_t));
    // handleSyncMovement = xSemaphoreCreateBinary();

    while(1){

        // if( xSemaphoreTake(handleSyncMovement,1 )){
        if( !outputMotors.motorsControl[MOTOR_A].flagRunning &&
            !outputMotors.motorsControl[MOTOR_B].flagRunning &&
            !outputMotors.motorsControl[MOTOR_C].flagRunning ) {
             
            vTaskDelay(pdMS_TO_TICKS(200));                         // TODO: solo para debug, BORRAR
            if(xQueueReceive( handleMoveAxis,
                ( void * ) &receiveNewMovement,
                1)){

                outputMotors.motorsControl[receiveNewMovement.motor] = receiveNewMovement.movement;
                gpio_set_level(outputMotors.motorsGpio.motors[receiveNewMovement.motor].dirPin,receiveNewMovement.movement.dir); 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
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

    outputMotors.motorVelContUs = 0;
    outputMotors.motorVelPercent = VEL_PERCENT_DEFAULT;
    setVel(outputMotors.motorVelPercent);                   // inician los motores con la velocidad default
    setDisableMotors();                                     // Inician los motores deshabilitados

    gptimer_handle_t handleTimer = NULL;

    gptimer_config_t timerConfig = {

        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimer));
    
    gptimer_alarm_config_t alarmMpu ={
        .alarm_count = MOTOR_PERIOD_US,      // Desborde de interrupcion: 1us de cada tick * MOTOR_PERIOD_US = 10us
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(handleTimer,&alarmMpu));
    
    gptimer_event_callbacks_t newCallback ={
        .on_alarm = timerInterrupt,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleTimer,&newCallback,NULL));

    ESP_ERROR_CHECK(gptimer_enable(handleTimer));
    ESP_ERROR_CHECK(gptimer_start(handleTimer));

    xTaskCreate(handlerQueueAxis,"axis Handler",2048,NULL,4,NULL);
}

void moveAxis(uint8_t motor,uint8_t dir,uint32_t steps,uint16_t duration){          // TODO: encolar estas solicitudes en una estructura de tipo motor_control_t
    
    new_movement_motor_t newMovement;

    newMovement.motor = motor;
    newMovement.movement.dir = dir;
    newMovement.movement.durationMs = duration;
    newMovement.movement.stepsMotor = steps*2;             // Multiplico por 2 ya que son pulsos, y yo cuento cambios de estado en el pin
    newMovement.movement.contMotor = 0;
    newMovement.movement.flagRunning = true;

    xQueueSend(handleMoveAxis,&newMovement,0);
    printf("Nuevo movimiento agregado a la cola,pendientes: %d\n",uxQueueMessagesWaiting( handleMoveAxis )); 
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