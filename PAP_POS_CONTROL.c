#include "include/PAP_POS_CONTROL.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "math.h"

#define PAP_POS_CONTROL_TAL "PAP_POS_CONTROL"

/* Cantidad de ticks hasta que se dispare la interrupcion */
#define MOTOR_PERIOD_US     1000

/* Definicion en la rampa, la velocidad tiene 1000 puntos de definicion*/
#define RAMP_DEFINITION     100

#define VAL_RAMP_PERCENT 0.1

QueueHandle_t       handleMoveAxis;
SemaphoreHandle_t   handleSyncMovement;
motors_control_t    outputMotors;
control_ramp_t      rampMotors[CANT_MOTORS];

gptimer_handle_t    handleTimerA = NULL;
gptimer_handle_t    handleTimerB = NULL;
gptimer_handle_t    handleTimerC = NULL;

static void generateStep( uint8_t indexMotor );
static uint16_t vel2us( uint8_t velInPercent);
static void reloadAlarm( uint8_t indexMotor, uint16_t velocity );
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
    uint8_t indexMotor;
    uint16_t difVelocity=0;

    for( indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){ 

        rampMotors[indexMotor].actualVelUs = vel2us(1);    
        rampMotors[indexMotor].targetVelUs = outputMotors.motorsControl[indexMotor].velocityUs;
        difVelocity = abs(rampMotors[indexMotor].actualVelUs -  rampMotors[indexMotor].targetVelUs);
        if( difVelocity == 0 ){
            difVelocity = 1;
        }  

        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "difVelocity %d,targetUs: %ld,actualUs: %ld\n",difVelocity,rampMotors[indexMotor].targetVelUs , rampMotors[indexMotor].actualVelUs);

        reloadAlarm( indexMotor, rampMotors[indexMotor].actualVelUs );
        rampMotors[indexMotor].stateRamp = RAMP_UP;
        rampMotors[indexMotor].distRampSteps = outputMotors.motorsControl[indexMotor].stepsMotor * VAL_RAMP_PERCENT;
        rampMotors[indexMotor].period = rampMotors[indexMotor].distRampSteps  / RAMP_DEFINITION;                    // el periodo es la cantidad de pasos dividido la resolucion de la rampa
        rampMotors[indexMotor].dxdt = ceil( difVelocity / (float)RAMP_DEFINITION  );                                                // dxdt es la pendiente entre cada paso de la rampa

        if(rampMotors[indexMotor].period == 0){
            rampMotors[indexMotor].period = 1;
        }
        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "distRamp %ld,difVelocity %d, Periodo %d, dxdt: %d\n",rampMotors[indexMotor].distRampSteps,difVelocity,rampMotors[indexMotor].period,rampMotors[indexMotor].dxdt);
    }
}

static void rampHandler(uint8_t indexMotor, uint32_t actualCont, uint32_t totalSteps){

    switch( rampMotors[indexMotor].stateRamp ){

        case RAMP_UP:
            
        //   if( motor.countSteps < rampa.distRampaSteps){           // dentro de la rampa de subida, debo chequear si no me estoy solapando con la rampa de bajada
        //     rampa.stateRampa = RAMPA_BAJADA;
        //   }
            
            if(( actualCont % rampMotors[indexMotor].period) == 0){
                if( rampMotors[indexMotor].actualVelUs > rampMotors[indexMotor].targetVelUs){
                    rampMotors[indexMotor].actualVelUs -= rampMotors[indexMotor].dxdt;
                    reloadAlarm( indexMotor, rampMotors[indexMotor].actualVelUs ); 
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
                if( rampMotors[indexMotor].actualVelUs < vel2us(1) ){
                    rampMotors[indexMotor].actualVelUs += rampMotors[indexMotor].dxdt;
                    reloadAlarm( indexMotor, rampMotors[indexMotor].actualVelUs );    
                }
                else{
                    rampMotors[indexMotor].stateRamp = RAMP_MESETA;
                }
            }
        break;
  }
}

void calculateInterpolation( void ){
    uint32_t maxSteps = 0;
    uint8_t indexMotor;

    maxSteps = outputMotors.motorsControl[MOTOR_A].stepsMotor;

    for( indexMotor = 0; indexMotor < CANT_MOTORS; indexMotor++ ){ 
        if( outputMotors.motorsControl[indexMotor].stepsMotor > maxSteps ){
            maxSteps = outputMotors.motorsControl[indexMotor].stepsMotor;
        }
    }

    for( indexMotor = 0; indexMotor < CANT_MOTORS; indexMotor++ ){ 
        outputMotors.motorsControl[indexMotor].velocityUs =  (uint32_t)(( maxSteps * vel2us(outputMotors.motorVelPercent)) / (float)outputMotors.motorsControl[indexMotor].stepsMotor );
        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "INTERPOLATION: Max: %ld, act: %ld,vel:%ld",maxSteps,outputMotors.motorsControl[indexMotor].stepsMotor,outputMotors.motorsControl[indexMotor].velocityUs);
    }
}

static void handlerQueueAxis(void *pvParameters){

    motor_control_t receiveNewMovement[3];
    uint8_t indexMotor;

    handleMoveAxis = xQueueCreate(100,sizeof(motors_control_t));

    while(1){

        if( !outputMotors.motorsControl[MOTOR_A].flagRunning &&
            !outputMotors.motorsControl[MOTOR_B].flagRunning &&
            !outputMotors.motorsControl[MOTOR_C].flagRunning ) {
             
            if(xQueueReceive( handleMoveAxis,
                ( void * ) &receiveNewMovement,
                0)){

                for( indexMotor = 0; indexMotor < 3; indexMotor++ ){
                    outputMotors.motorsControl[indexMotor] = receiveNewMovement[indexMotor];
                    gpio_set_level(outputMotors.motorsGpio.motors[indexMotor].dirPin,outputMotors.motorsControl[indexMotor].dir); 
                    
                }

                calculateInterpolation();
                setRampa();

                outputMotors.motorsControl[MOTOR_A].flagRunning = true;
                outputMotors.motorsControl[MOTOR_B].flagRunning = true;
                outputMotors.motorsControl[MOTOR_C].flagRunning = true;

                
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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

static void reloadAlarm( uint8_t indexMotor, uint16_t velocityUs ){
    // uint16_t velocityUs = 0;

    // if(velocity > 0 && velocity <= RAMP_DEFINITION){
        // velocityUs = MAX_VELOCITY_US + (((MIN_VELOCITY_US-MAX_VELOCITY_US)*(RAMP_DEFINITION-velocity))/RAMP_DEFINITION);
        // velocityUs = velocity;
        // ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"motor %d: velocity: %d,salida: %d\n",indexMotor,velocity,velocityUs );

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
    // } 
}

void moveAxis(uint8_t dirA,uint32_t stepsA,uint8_t dirB,uint32_t stepsB,uint8_t dirC,uint32_t stepsC){
    
    motor_control_t newMovement[3];

    newMovement[MOTOR_A].dir = dirA;
    newMovement[MOTOR_A].stepsMotor = stepsA *2;
    newMovement[MOTOR_A].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_A].velocityUs = vel2us(outputMotors.motorVelPercent);
    newMovement[MOTOR_A].flagRunning = false;

    newMovement[MOTOR_B].dir = dirB;
    newMovement[MOTOR_B].stepsMotor = stepsB *2;
    newMovement[MOTOR_B].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_B].velocityUs = vel2us(outputMotors.motorVelPercent);
    newMovement[MOTOR_B].flagRunning = false;

    newMovement[MOTOR_C].dir = dirC;
    newMovement[MOTOR_C].stepsMotor = stepsC *2;
    newMovement[MOTOR_C].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_C].velocityUs = vel2us(outputMotors.motorVelPercent);
    newMovement[MOTOR_C].flagRunning = false;

    xQueueSend(handleMoveAxis,&newMovement,0);
    printf("Nuevo movimiento agregado a la cola,pendientes: %d\n",uxQueueMessagesWaiting( handleMoveAxis )); 
}

static uint16_t vel2us( uint8_t velInPercent){
    return MAX_VELOCITY_US + (((MIN_VELOCITY_US-MAX_VELOCITY_US)*(RAMP_DEFINITION-velInPercent))/RAMP_DEFINITION);
}

void setVel(uint8_t velocityInPercent){

    if(velocityInPercent > 0 && velocityInPercent <= 100){
        outputMotors.motorVelPercent = velocityInPercent;   
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