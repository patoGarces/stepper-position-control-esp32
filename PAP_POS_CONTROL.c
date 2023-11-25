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

QueueHandle_t       handleMoveAxis;
SemaphoreHandle_t   handleSyncMovement;
motors_control_t    outputMotors;
control_ramp_t      rampMotors;

gptimer_handle_t    handleBaseTimer = NULL;

// TODO: bug: a alta velocidad el motor de mayor recorrido no se mueve

static void generateStep( uint8_t indexMotor );
static uint16_t vel2us( uint8_t velInPercent);
static void rampHandler( void );
static void calculateRamp( void );


static bool IRAM_ATTR timerInterrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    uint8_t indexMotor;

    for(indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){
        outputMotors.motorsControl[indexMotor].actualTicks += PERIOD_RAMP_HANDLER;

        if( outputMotors.motorsControl[indexMotor].actualTicks > outputMotors.motorsControl[indexMotor].velocityUs){
            outputMotors.motorsControl[indexMotor].actualTicks = 0;
            generateStep(indexMotor);
        }
    }
    #ifndef AVOID_ACCEL_RAMP
    if( outputMotors.motorsControl[MOTOR_A].flagRunning ||
        outputMotors.motorsControl[MOTOR_B].flagRunning ||
        outputMotors.motorsControl[MOTOR_C].flagRunning ){
        rampHandler();
    }
    #endif
    
    return false;
}

static void generateStep( uint8_t indexMotor ){

    if( outputMotors.motorsControl[indexMotor].flagRunning ){
        if(outputMotors.motorsControl[indexMotor].actualSteps < outputMotors.motorsControl[indexMotor].totalSteps){
            outputMotors.motorsControl[indexMotor].actualSteps++;
            gpio_set_level(outputMotors.motorsGpio.motors[indexMotor].stepPin,outputMotors.motorsControl[indexMotor].flagToggle);
            outputMotors.motorsControl[indexMotor].flagToggle = !outputMotors.motorsControl[indexMotor].flagToggle;  
        }
        else{ 
            outputMotors.motorsControl[indexMotor].flagRunning = false;
        }
    }
}

static void calculateRamp( void ){
    uint8_t indexMotor=0;
    uint16_t difVelocity=0;

    rampMotors.rampTotalTime = outputMotors.motorsControl[indexMotor].velocityUs * outputMotors.motorsControl[indexMotor].totalSteps;
    rampMotors.rampUpDownTime =  rampMotors.rampTotalTime * VAL_RAMP_PERCENT;
    rampMotors.constantRampTime = rampMotors.rampTotalTime * ( 1.17 - (2*VAL_RAMP_PERCENT));        // 1.17 Deberia ser 1.00, pero termina la rampa antes que los pasos, con este offset queda sincronizado

    rampMotors.period = rampMotors.rampUpDownTime / RAMP_DEFINITION;
    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "Calculo constantes rampa-> totalTime: %ld, t1: %ld, t2: %ld",rampMotors.rampTotalTime,rampMotors.rampUpDownTime, rampMotors.constantRampTime );

    for( indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){ 

        rampMotors.individualRamp[indexMotor].actualVelUs = vel2us(1);    
        rampMotors.individualRamp[indexMotor].targetVelUs = outputMotors.motorsControl[indexMotor].velocityUs;
        difVelocity = abs(rampMotors.individualRamp[indexMotor].actualVelUs -  rampMotors.individualRamp[indexMotor].targetVelUs);
        if( difVelocity == 0 ){
            difVelocity = 1;
            ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"ERROR DIF DE VELOCIDAD MUY CHICA");
        }
        
        outputMotors.motorsControl[indexMotor].velocityUs = rampMotors.individualRamp[indexMotor].actualVelUs;
        
        rampMotors.individualRamp[indexMotor].dxdt = ceil(difVelocity / RAMP_DEFINITION) ;

        if( !rampMotors.individualRamp[indexMotor].dxdt ){
            rampMotors.individualRamp[indexMotor].dxdt = 1;
        }

        if(rampMotors.period == 0){
            rampMotors.period = 1;
            ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"ERROR PERIODO DE RAMPA MUY CHICO");
        }
        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "difVelocity %d,targetUs: %ld,actualUs: %ld, periodo: %ld, dxdt: %ld",difVelocity,rampMotors.individualRamp[indexMotor].targetVelUs , rampMotors.individualRamp[indexMotor].actualVelUs,rampMotors.period,rampMotors.individualRamp[indexMotor].dxdt);
    }
    rampMotors.actualTime = 0;
    rampMotors.stateRamp = RAMP_UP;
    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"STATE RAMP -> RAMP UP: %ld,vel0: %ld,vel1: %ld,vel2: %ld",rampMotors.actualTime,rampMotors.individualRamp[0].actualVelUs,rampMotors.individualRamp[1].actualVelUs,rampMotors.individualRamp[2].actualVelUs); // TODO: Eliminaar
    gpio_set_level(22,1);
}

static void rampHandler(){
    uint8_t indexMotor=0;

    rampMotors.actualTime += PERIOD_RAMP_HANDLER;

    if(( rampMotors.actualTime % rampMotors.period) == 0){

        // ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"actualCont: %ld, period: %ld",rampMotors.actualTime, rampMotors.period);
        switch( rampMotors.stateRamp ){

            case RAMP_UP:   
            //   if( motor.countSteps < rampa.distRampaSteps){           // dentro de la rampa de subida, debo chequear si no me estoy solapando con la rampa de bajada
            //     rampa.stateRampa = RAMPA_BAJADA;
            //   }  
                if( rampMotors.actualTime < rampMotors.rampUpDownTime ){
                    for(indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){
                        if( rampMotors.individualRamp[indexMotor].actualVelUs > rampMotors.individualRamp[indexMotor].targetVelUs){
                            rampMotors.individualRamp[indexMotor].actualVelUs -= rampMotors.individualRamp[indexMotor].dxdt;
                            outputMotors.motorsControl[indexMotor].velocityUs = rampMotors.individualRamp[indexMotor].actualVelUs; 
                        }
                    } 
                }
                else{
                    rampMotors.stateRamp = RAMP_MESETA;
                    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"STATE RAMP -> RAMP MESETA: %ld,vel0: %ld,vel1: %ld,vel2: %ld",rampMotors.actualTime,rampMotors.individualRamp[0].actualVelUs,rampMotors.individualRamp[1].actualVelUs,rampMotors.individualRamp[2].actualVelUs); // TODO: Eliminaar
                    gpio_set_level(22,0);
                    for(indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){
                        rampMotors.individualRamp[indexMotor].actualVelUs = rampMotors.individualRamp[indexMotor].targetVelUs;
                        outputMotors.motorsControl[indexMotor].velocityUs = rampMotors.individualRamp[indexMotor].actualVelUs;
                    }
                } 
            break;

            case RAMP_MESETA:
                if( rampMotors.actualTime >= ( rampMotors.rampUpDownTime + rampMotors.constantRampTime)){
                    rampMotors.stateRamp = RAMP_DOWN;
                    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"STATE RAMP -> RAMP DOWN: %ld,vel0: %ld,vel1: %ld,vel2: %ld",rampMotors.actualTime,rampMotors.individualRamp[0].actualVelUs,rampMotors.individualRamp[1].actualVelUs,rampMotors.individualRamp[2].actualVelUs); // TODO: Eliminaar
                    gpio_set_level(22,1);
                }
            break;

            case RAMP_DOWN:
                if( rampMotors.actualTime < rampMotors.rampTotalTime ){
                    for(indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){
                        if( rampMotors.individualRamp[indexMotor].actualVelUs < vel2us(1) ){
                            rampMotors.individualRamp[indexMotor].actualVelUs += rampMotors.individualRamp[indexMotor].dxdt;
                            outputMotors.motorsControl[indexMotor].velocityUs = rampMotors.individualRamp[indexMotor].actualVelUs;    
                        }
                    }
                }
                else{
                    rampMotors.stateRamp = RAMP_END;
                    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"STATE RAMP -> RAMP END : %ld,vel0: %ld,vel1: %ld,vel2: %ld",rampMotors.actualTime,rampMotors.individualRamp[0].actualVelUs,rampMotors.individualRamp[1].actualVelUs,rampMotors.individualRamp[2].actualVelUs); // TODO: Eliminaar
                    gpio_set_level(22,0);
                    for(indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){
                        rampMotors.individualRamp[indexMotor].actualVelUs = vel2us(1);
                        outputMotors.motorsControl[indexMotor].velocityUs = rampMotors.individualRamp[indexMotor].actualVelUs;
                    }
                }
            break;
        }
    }
}

void calculateInterpolation( void ){
    uint32_t maxSteps = 0;
    uint8_t indexMotor,indexMax=0;


    maxSteps = outputMotors.motorsControl[MOTOR_A].totalSteps;

    for( indexMotor = 0; indexMotor < CANT_MOTORS; indexMotor++ ){ 
        if( outputMotors.motorsControl[indexMotor].totalSteps > maxSteps ){
            maxSteps = outputMotors.motorsControl[indexMotor].totalSteps;
            indexMax = indexMotor;
        }
    }

    for( indexMotor = 0; indexMotor < CANT_MOTORS; indexMotor++ ){ 
        outputMotors.motorsControl[indexMotor].velocityUs =  (uint32_t)(( maxSteps * vel2us(outputMotors.motorVelPercent)) / (float)outputMotors.motorsControl[indexMotor].totalSteps );
        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "INTERPOLATION: Max: %ld steps, act: %ld steps,vel: %ldus",maxSteps,outputMotors.motorsControl[indexMotor].totalSteps,outputMotors.motorsControl[indexMotor].velocityUs);
    }
    outputMotors.motorsControl[indexMax].velocityUs = MAX_VELOCITY_US*0.95;
     ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "%ld, %ld , %ld ",outputMotors.motorsControl[0].velocityUs,outputMotors.motorsControl[1].velocityUs,outputMotors.motorsControl[2].velocityUs);
    
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

                #ifndef AVOID_ACCEL_RAMP
                    calculateRamp();
                #endif

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

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleBaseTimer));
    
    gptimer_event_callbacks_t newCallback ={
        .on_alarm = timerInterrupt,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleBaseTimer,&newCallback,NULL));
    
    outputMotors.motorVelPercent = VEL_PERCENT_DEFAULT;
    setVel(outputMotors.motorVelPercent);                   // inician los motores con la velocidad default

    ESP_ERROR_CHECK(gptimer_enable(handleBaseTimer));
    ESP_ERROR_CHECK(gptimer_start(handleBaseTimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = BASE_PERIOD_TIMER,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(handleBaseTimer, &alarm_config);
    
    xTaskCreate(handlerQueueAxis,"axis Handler",4096,NULL,4,NULL);          // TODO: medir el tamaño de stack consumido
}


void moveAxis(int32_t stepsA,int32_t stepsB,int32_t stepsC){
    motor_control_t newMovement[3];

    newMovement[MOTOR_A].dir = stepsA > 0;
    newMovement[MOTOR_A].totalSteps = abs(stepsA *2);
    newMovement[MOTOR_A].actualSteps = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_A].velocityUs = vel2us(outputMotors.motorVelPercent);
    newMovement[MOTOR_A].flagRunning = false;

    newMovement[MOTOR_B].dir = stepsB > 0;
    newMovement[MOTOR_B].totalSteps = abs(stepsB *2);
    newMovement[MOTOR_B].actualSteps = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_B].velocityUs = vel2us(outputMotors.motorVelPercent);
    newMovement[MOTOR_B].flagRunning = false;

    newMovement[MOTOR_C].dir = stepsC > 0;
    newMovement[MOTOR_C].totalSteps = abs(stepsC *2);
    newMovement[MOTOR_C].actualSteps = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
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