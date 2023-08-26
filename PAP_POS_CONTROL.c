#include "include/PAP_POS_CONTROL.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

#define TIMER_GROUP_MOTOR   TIMER_GROUP_0
#define TIMER_NUM_MOTOR     TIMER_0
/* Cantidad de ticks hasta que se dispare la interrupcion */
#define MOTOR_PERIOD_US     45

QueueHandle_t       handleMoveAxis;
motors_control_t    outputMotors;

bool finishMovement[3] = {false,false,false};

int16_t absolutePosition[3] = {0,0,0};

static bool IRAM_ATTR timerInterrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    uint8_t motor;
    BaseType_t high_task_awoken = pdFALSE;

    if(outputMotors.motorVelContUs < outputMotors.motorVelUs){          // TODO: eliminar esta logica, manejar la velocidad con el periodo de la alarma
        outputMotors.motorVelContUs++;
    }
    else{
        outputMotors.motorVelContUs = 0;
        for(motor=0;motor<3;motor++){
            // motor=0;

            if(outputMotors.motorsControl[motor].flagEnable){
                if(outputMotors.motorsControl[motor].contMotor < outputMotors.motorsControl[motor].stepsMotor){
                    outputMotors.motorsControl[motor].contMotor++;

                    gpio_set_level(outputMotors.motorsGpio.motors[motor].stepPin,outputMotors.motorsControl[motor].flagToggle);
                    
                    // GPIO.out_w1ts.data = 1 << actualMovement.motorsGpio.motors[motor].stepPin; // high
                    // GPIO.out_w1tc.data = 1 << actualMovement.motorsGpio.motors[motor].stepPin; // low

                    if(outputMotors.motorsControl[motor].dir){
                        absolutePosition[motor]++;
                    }
                    else{
                        absolutePosition[motor]--;
                    }
                    
                    outputMotors.motorsControl[motor].flagToggle = !outputMotors.motorsControl[motor].flagToggle;  
                }
                else{                                                           // TODO: encolar las solicitudes de moveAxis, aca cargar el siguiente movimiento
                    outputMotors.motorsControl[motor].flagEnable = false;
                }
            }
            else{
                finishMovement[motor] = true; // TODO: contemplar el encolado de cada motor por separado
            }
        }
    }
        
    return (high_task_awoken == pdTRUE);
}

static void handlerQueueAxis(void *pvParameters){

    new_movement_motor_t receiveNewMovement;
    handleMoveAxis = xQueueCreate(100,sizeof(motors_control_t));

    printf("handlerQueueAxis task created\n");

    while(1){

        if(xQueueReceive( handleMoveAxis,
                ( void * ) &receiveNewMovement,
                1)){
    
            vTaskDelay(pdMS_TO_TICKS(200)); 
            if(finishMovement[receiveNewMovement.motor]){
                finishMovement[receiveNewMovement.motor] = false;
                outputMotors.motorsControl[receiveNewMovement.motor] = receiveNewMovement.movement;

                gpio_set_level(outputMotors.motorsGpio.motors[receiveNewMovement.motor].dirPin,receiveNewMovement.movement.dir);

                printf("AbsolutePosition: %d: %d\n",receiveNewMovement.motor,absolutePosition[receiveNewMovement.motor]);
                printf("Nuevo movimiento cargado al timer: motor %d,pasos: %ld\n",receiveNewMovement.motor,receiveNewMovement.movement.stepsMotor); 
            }
            else{                                           // Si el motor aun esta trabajando, cargo el movimiento al final de la cola
                xQueueSend(handleMoveAxis,&receiveNewMovement,0);
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
    newMovement.movement.flagEnable = true;


    printf("Movimientos pendientes: %d\n",uxQueueMessagesWaiting( handleMoveAxis ));
    xQueueSend(handleMoveAxis,&newMovement,0);

    printf("Nuevo movimiento agregado a la cola\n"); 

    // outputMotors.motorsControl[motor].dir = dir;
    // outputMotors.motorsControl[motor].durationMs = duration;
    // outputMotors.motorsControl[motor].stepsMotor = steps*2;             // Multiplico por 2 ya que son pulsos, y yo cuento cambios de estado en el pin
    // outputMotors.motorsControl[motor].contMotor = 0;
    // outputMotors.motorsControl[motor].flagEnable = true;
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