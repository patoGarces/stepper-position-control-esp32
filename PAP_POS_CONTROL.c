#include "include/PAP_POS_CONTROL.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

#define PAP_POS_CONTROL_TAL "PAP_POS_CONTROL"

/* Cantidad de ticks hasta que se dispare la interrupcion */
#define MOTOR_PERIOD_US     1000

#define VAL_RAMP_PERCENT 0.1

QueueHandle_t       handleMoveAxis;
SemaphoreHandle_t   handleSyncMovement;
motors_control_t    outputMotors;
control_ramp_t      rampMotors[CANT_MOTORS];

gptimer_handle_t    handleTimerA = NULL;
gptimer_handle_t    handleTimerB = NULL;
gptimer_handle_t    handleTimerC = NULL;

static void generateStep( uint8_t indexMotor );

static void reloadAlarm( uint8_t indexMotor, uint8_t velocity );
static void rampHandler( uint8_t motor, uint32_t actualCont, uint32_t totalSteps );
static void setRampa( void );

static bool IRAM_ATTR timerInterrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {

    if( timer == handleTimerA ){
        generateStep(MOTOR_A); 
    }
    else if( timer == handleTimerB ){
        generateStep(MOTOR_B);
    }
    else{
        generateStep(MOTOR_C);
    }
    return  false;
}

static void generateStep( uint8_t indexMotor ){

    if( outputMotors.motorsControl[indexMotor].flagRunning ){

        if(outputMotors.motorsControl[indexMotor].contMotor < outputMotors.motorsControl[indexMotor].stepsMotor){

            outputMotors.motorsControl[indexMotor].contMotor++;
            gpio_set_level(outputMotors.motorsGpio.motors[indexMotor].stepPin,outputMotors.motorsControl[indexMotor].flagToggle);
            outputMotors.motorsControl[indexMotor].flagToggle = !outputMotors.motorsControl[indexMotor].flagToggle;  

            rampHandler(indexMotor,outputMotors.motorsControl[indexMotor].contMotor,outputMotors.motorsControl[indexMotor].stepsMotor);
        }
        else{ 
            outputMotors.motorsControl[indexMotor].flagRunning = false;
        }
    }
}

static void setRampa( void ){
    uint8_t indexMotor,difVelocity=0;

    for( indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){ 

        rampMotors[indexMotor].actualVel = 1;    
        rampMotors[indexMotor].targetVel = outputMotors.motorsControl[indexMotor].velocity;
        difVelocity = rampMotors[indexMotor].targetVel - rampMotors[indexMotor].actualVel;
        if( difVelocity ==0 ){
            difVelocity = 1;
        }  
        reloadAlarm( indexMotor,1 );
        rampMotors[indexMotor].stateRamp = RAMP_UP;
        rampMotors[indexMotor].distRampSteps = outputMotors.motorsControl[indexMotor].stepsMotor * VAL_RAMP_PERCENT;
        rampMotors[indexMotor].period = rampMotors[indexMotor].distRampSteps / difVelocity ;                   // el periodo es la cantidad de pasos dividido el diferencial de velocidad total
        
        if(rampMotors[indexMotor].period == 0){
            rampMotors[indexMotor].period = 1;
        }
        // printf("Nueva rampa, motor %d,stepMotor: %ld,distanciaRampa: %ld,periodo: %d,targetVel:%d\n",indexMotor,outputMotors.motorsControl[indexMotor].stepsMotor,rampMotors[indexMotor].distRampSteps,rampMotors[indexMotor].period,rampMotors[indexMotor].targetVel);
    }
}

static void rampHandler(uint8_t indexMotor, uint32_t actualCont, uint32_t totalSteps){

    switch( rampMotors[indexMotor].stateRamp ){

        case RAMP_UP:
            
        //   if( motor.countSteps < rampa.distRampaSteps){           // dentro de la rampa de subida, debo chequear si no me estoy solapando con la rampa de bajada
        //     rampa.stateRampa = RAMPA_BAJADA;
        //   }
            
            if(( actualCont % rampMotors[indexMotor].period) == 0){
                if( rampMotors[indexMotor].actualVel < rampMotors[indexMotor].targetVel){
                    rampMotors[indexMotor].actualVel++;
                    reloadAlarm( indexMotor, rampMotors[indexMotor].actualVel ); 
                }
                else{
                    rampMotors[indexMotor].stateRamp = RAMP_MESETA;
                }
            }
        break;

        case RAMP_MESETA:

            if( actualCont > ( outputMotors.motorsControl[indexMotor].stepsMotor - rampMotors[indexMotor].distRampSteps)){
                rampMotors[indexMotor].stateRamp = RAMP_DOWN;
            }

        break;

        case RAMP_DOWN:

            if(( actualCont % rampMotors[indexMotor].period) == 0){

                if( rampMotors[indexMotor].actualVel > 1){
                    rampMotors[indexMotor].actualVel--;
                    reloadAlarm( indexMotor, rampMotors[indexMotor].actualVel ); 
                }
                else{
                    rampMotors[indexMotor].stateRamp = RAMP_MESETA;
                }
            }
        break;
  }
}

static void handlerQueueAxis(void *pvParameters){

    motor_control_t receiveNewMovement[3];
    uint8_t indexMotor;
    uint32_t maxSteps = 0;

    handleMoveAxis = xQueueCreate(100,sizeof(motors_control_t));

    while(1){

        if( !outputMotors.motorsControl[MOTOR_A].flagRunning &&
            !outputMotors.motorsControl[MOTOR_B].flagRunning &&
            !outputMotors.motorsControl[MOTOR_C].flagRunning ) {
             
            if(xQueueReceive( handleMoveAxis,
                ( void * ) &receiveNewMovement,
                0)){

                maxSteps = receiveNewMovement[0].stepsMotor;

                for( indexMotor = 0; indexMotor < 3; indexMotor++ ){

                    if( receiveNewMovement[indexMotor].stepsMotor > maxSteps ){
                        maxSteps = receiveNewMovement[indexMotor].stepsMotor;
                    }
                    outputMotors.motorsControl[indexMotor] = receiveNewMovement[indexMotor];
                    gpio_set_level(outputMotors.motorsGpio.motors[indexMotor].dirPin,outputMotors.motorsControl[indexMotor].dir); 
                }

                for( indexMotor = 0; indexMotor < 2; indexMotor++ ){                    // Calculo la relacion entre la mayor cantidad de pasos y el resto de los ejes, para calcular la velocidad de interpolacion
                    outputMotors.motorsControl[indexMotor].velocity =  (uint32_t)(outputMotors.motorsControl[indexMotor].stepsMotor * 100) / (float)maxSteps;
                    if( outputMotors.motorsControl[indexMotor].velocity == 0 ){
                        outputMotors.motorsControl[indexMotor].velocity = 1;
                    }
                    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "Max: %ld, act: %ld,vel:%d",maxSteps,outputMotors.motorsControl[indexMotor].stepsMotor,outputMotors.motorsControl[indexMotor].velocity);
                }   
                setRampa();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// TODO: hacer float la velocidad!

void initMotors(output_motors_pins_t pinout){

    uint8_t motor;

    outputMotors.motorsGpio = pinout;

    gpio_set_direction( outputMotors.motorsGpio.enablePin,GPIO_MODE_OUTPUT );
    for( motor=0;motor<3;motor++){
        gpio_set_direction( outputMotors.motorsGpio.motors[motor].dirPin,GPIO_MODE_OUTPUT );
        gpio_set_direction( outputMotors.motorsGpio.motors[motor].stepPin,GPIO_MODE_OUTPUT );
    }

    setDisableMotors();

    gptimer_config_t timerConfig = {

        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    // ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimer));
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimerA));
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimerB));
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimerC));
    
    gptimer_event_callbacks_t newCallback ={
        .on_alarm = timerInterrupt,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleTimerA,&newCallback,NULL));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleTimerB,&newCallback,NULL));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleTimerC,&newCallback,NULL));
    
    outputMotors.motorVelPercent = VEL_PERCENT_DEFAULT;
    setVel(outputMotors.motorVelPercent);                   // inician los motores con la velocidad default

    ESP_ERROR_CHECK(gptimer_enable(handleTimerA));
    ESP_ERROR_CHECK(gptimer_enable(handleTimerB));
    ESP_ERROR_CHECK(gptimer_enable(handleTimerC));
    ESP_ERROR_CHECK(gptimer_start(handleTimerA));
    ESP_ERROR_CHECK(gptimer_start(handleTimerB));
    ESP_ERROR_CHECK(gptimer_start(handleTimerC));

    xTaskCreate(handlerQueueAxis,"axis Handler",4096,NULL,4,NULL);          // TODO: medir el tamaño de stack consumido
}

static void reloadAlarm( uint8_t indexMotor, uint8_t velocity ){
    uint16_t velocityUs = 0;

    if(velocity > 0 && velocity <= 100){
        velocityUs = MAX_VELOCITY_US + (((MIN_VELOCITY_US-MAX_VELOCITY_US)*(101-velocity))/100);

        // printf("VelMinUs: %dus,VelMaxUs: %dus,velocity: %d%%,salida: %d\n",MIN_VELOCITY_US,MAX_VELOCITY_US,velocity,outputMotors.motorVelUs);

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = velocityUs,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        switch ( indexMotor ){
            case MOTOR_A:
                gptimer_set_alarm_action(handleTimerA, &alarm_config);
            break;
            case MOTOR_B:
                gptimer_set_alarm_action(handleTimerB, &alarm_config);
            break;
            case MOTOR_C:
                gptimer_set_alarm_action(handleTimerC, &alarm_config);
            break;
        }
    } 
}

void moveAxis(uint8_t dirA,uint32_t stepsA,uint8_t dirB,uint32_t stepsB,uint8_t dirC,uint32_t stepsC){          // TODO: encolar estas solicitudes en una estructura de tipo motor_control_t
    
    motor_control_t newMovement[3];

    newMovement[MOTOR_A].dir = dirA;
    newMovement[MOTOR_A].stepsMotor = stepsA *2;
    newMovement[MOTOR_A].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_A].flagRunning = true;
    newMovement[MOTOR_A].velocity = outputMotors.motorVelPercent;

    newMovement[MOTOR_B].dir = dirB;
    newMovement[MOTOR_B].stepsMotor = stepsB *2;
    newMovement[MOTOR_B].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_B].flagRunning = true;
    newMovement[MOTOR_B].velocity = outputMotors.motorVelPercent;

    newMovement[MOTOR_C].dir = dirC;
    newMovement[MOTOR_C].stepsMotor = stepsC *2;
    newMovement[MOTOR_C].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_C].flagRunning = true;
    newMovement[MOTOR_C].velocity = outputMotors.motorVelPercent;

    xQueueSend(handleMoveAxis,&newMovement,0);
    printf("Nuevo movimiento agregado a la cola,pendientes: %d\n",uxQueueMessagesWaiting( handleMoveAxis )); 
}

void setVel(uint8_t velocity){

    if(velocity > 0 && velocity <= 100){
        outputMotors.motorVelPercent = velocity;
        reloadAlarm(0,velocity);   
        reloadAlarm(1,velocity);   
        reloadAlarm(2,velocity);    
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