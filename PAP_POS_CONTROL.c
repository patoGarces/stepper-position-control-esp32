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

gptimer_handle_t    handleTimerA = NULL;
gptimer_handle_t    handleTimerB = NULL;
gptimer_handle_t    handleTimerC = NULL;
gptimer_handle_t    handleTimerRamp = NULL;

// TODO: bug: a alta velocidad el motor de mayor recorrido no se mueve


static void generateStep( uint8_t indexMotor );
static uint16_t vel2us( uint8_t velInPercent);
static void reloadAlarm( uint8_t indexMotor, uint16_t velocity );
static void rampHandler( void );
static void calculateRamp( void );


static bool IRAM_ATTR timerInterrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {

    if( timer == handleTimerA ){
        generateStep(MOTOR_A); 
    }
    else if( timer == handleTimerB ){
        generateStep(MOTOR_B);
    }
    else if( timer == handleTimerC ){
        generateStep(MOTOR_C);
    }
    else{
        if( outputMotors.motorsControl[MOTOR_A].flagRunning){
            rampHandler();
        }
    }
    return  false;
}

static void generateStep( uint8_t indexMotor ){

    if( outputMotors.motorsControl[indexMotor].flagRunning ){

        if(outputMotors.motorsControl[indexMotor].contMotor < outputMotors.motorsControl[indexMotor].stepsMotor){

            outputMotors.motorsControl[indexMotor].contMotor++;
            gpio_set_level(outputMotors.motorsGpio.motors[indexMotor].stepPin,outputMotors.motorsControl[indexMotor].flagToggle);
            outputMotors.motorsControl[indexMotor].flagToggle = !outputMotors.motorsControl[indexMotor].flagToggle;  
        }
        else{ 
            outputMotors.motorsControl[indexMotor].flagRunning = false;
        }
    }
}

// ActualVel	Arranca de 1%
// targetVel	Depende de la interpolacion
// AcelMax	(velMax-velMin)*2 / deltaT
	
// Columna t	actualCont
// Columna Acel	(acelMax*period)/deltaT
// columna vel	vel_inic + (accelMax*actualCont*actualCont*0,5)/deltaT 


static void calculateRamp( void ){
    uint8_t indexMotor=0;
    float difVelocity=0;

    rampMotors.rampTotalTime = outputMotors.motorsControl[indexMotor].velocityUs * outputMotors.motorsControl[indexMotor].stepsMotor;
    rampMotors.rampUpDownTime =  rampMotors.rampTotalTime * VAL_RAMP_PERCENT;
    rampMotors.constantRampTime = rampMotors.rampTotalTime * ( 1.00 - (2*VAL_RAMP_PERCENT)); 

    rampMotors.period = rampMotors.rampUpDownTime / RAMP_DEFINITION;
    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "Calculo constantes rampa-> totalTime: %ld, t1: %ld, t2: %ld",rampMotors.rampTotalTime,rampMotors.rampUpDownTime, rampMotors.constantRampTime );

    for( indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){ 

        rampMotors.individualRamp[indexMotor].actualVelUs = vel2us(1);    
        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "outputMotores velocityUs: %ld",outputMotors.motorsControl[indexMotor].velocityUs);
        
        rampMotors.individualRamp[indexMotor].targetVelUs = outputMotors.motorsControl[indexMotor].velocityUs;
        difVelocity = rampMotors.individualRamp[indexMotor].targetVelUs- rampMotors.individualRamp[indexMotor].actualVelUs;

        reloadAlarm( indexMotor, rampMotors.individualRamp[indexMotor].actualVelUs );

        rampMotors.individualRamp[indexMotor].accelMax = (difVelocity*2.00) / rampMotors.rampUpDownTime;

        printf("accelMax -> difVelocity: %f, t1: %ld,accelMax: %f\n",difVelocity,rampMotors.rampUpDownTime, rampMotors.individualRamp[indexMotor].accelMax);
        
        // rampMotors.individualRamp[indexMotor].dxdt = ceil(difVelocity / RAMP_DEFINITION) ;

        // if( !rampMotors.individualRamp[indexMotor].dxdt ){
        //     rampMotors.individualRamp[indexMotor].dxdt = 1;
        // }

        // if(rampMotors.period == 0){
        //     rampMotors.period = 1;
        //     ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"ERROR PERIODO DE RAMPA MUY CHICO");
        // }
        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "targetUs: %ld,actualUs: %ld, periodo: %ld",rampMotors.individualRamp[indexMotor].targetVelUs , rampMotors.individualRamp[indexMotor].actualVelUs,rampMotors.period);
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
                            rampMotors.individualRamp[indexMotor].actualVelUs = vel2us(1) + ( rampMotors.individualRamp[indexMotor].accelMax*rampMotors.actualTime*rampMotors.actualTime*0.5)/(float)rampMotors.rampUpDownTime; 
                            reloadAlarm( indexMotor, rampMotors.individualRamp[indexMotor].actualVelUs ); 
                        }
                    } 
                }
                else{
                    rampMotors.stateRamp = RAMP_MESETA;
                    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"STATE RAMP -> RAMP MESETA: %ld,vel0: %ld,vel1: %ld,vel2: %ld",rampMotors.actualTime,rampMotors.individualRamp[0].actualVelUs,rampMotors.individualRamp[1].actualVelUs,rampMotors.individualRamp[2].actualVelUs); // TODO: Eliminaar
                    gpio_set_level(22,0);
                    for(indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){
                        rampMotors.individualRamp[indexMotor].actualVelUs = rampMotors.individualRamp[indexMotor].targetVelUs;
                        reloadAlarm( indexMotor, rampMotors.individualRamp[indexMotor].actualVelUs );
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
                            rampMotors.individualRamp[indexMotor].actualVelUs =  vel2us(1) - ( rampMotors.individualRamp[indexMotor].accelMax*rampMotors.actualTime*rampMotors.actualTime*0.5)/rampMotors.rampUpDownTime;
                            reloadAlarm( indexMotor, rampMotors.individualRamp[indexMotor].actualVelUs );    
                        }
                    }
                }
                else{
                    rampMotors.stateRamp = RAMP_END;
                    ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL,"STATE RAMP -> RAMP END : %ld,vel0: %ld,vel1: %ld,vel2: %ld",rampMotors.actualTime,rampMotors.individualRamp[0].actualVelUs,rampMotors.individualRamp[1].actualVelUs,rampMotors.individualRamp[2].actualVelUs); // TODO: Eliminaar
                    gpio_set_level(22,0);
                    for(indexMotor=0;indexMotor<CANT_MOTORS;indexMotor++){
                        rampMotors.individualRamp[indexMotor].actualVelUs = vel2us(1);
                        reloadAlarm( indexMotor, rampMotors.individualRamp[indexMotor].actualVelUs );
                    }
                }
            break;
        }
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
        ESP_DRAM_LOGE(PAP_POS_CONTROL_TAL, "INTERPOLATION: Max: %ld steps, act: %ld steps,vel: %ldus",maxSteps,outputMotors.motorsControl[indexMotor].stepsMotor,outputMotors.motorsControl[indexMotor].velocityUs);
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

                #ifndef AVOID_ACCEL_RAMP
                    calculateRamp();
                #else
                    reloadAlarm( MOTOR_A, outputMotors.motorsControl[MOTOR_A].velocityUs );
                    reloadAlarm( MOTOR_B, outputMotors.motorsControl[MOTOR_B].velocityUs );
                    reloadAlarm( MOTOR_C, outputMotors.motorsControl[MOTOR_C].velocityUs );
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

    #ifndef AVOID_ACCEL_RAMP
        ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimerRamp));
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleTimerRamp,&newCallback,NULL));
        ESP_ERROR_CHECK(gptimer_enable(handleTimerRamp));
        ESP_ERROR_CHECK(gptimer_start(handleTimerRamp));
    #endif

     gptimer_alarm_config_t alarm_config = {
            .alarm_count = PERIOD_RAMP_HANDLER,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
    gptimer_set_alarm_action(handleTimerRamp, &alarm_config);

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

void moveAxis(int32_t stepsA,int32_t stepsB,int32_t stepsC){
    
    motor_control_t newMovement[3];

    newMovement[MOTOR_A].dir = stepsA > 0;
    newMovement[MOTOR_A].stepsMotor = abs(stepsA *2);
    newMovement[MOTOR_A].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_A].velocityUs = vel2us(outputMotors.motorVelPercent);
    newMovement[MOTOR_A].flagRunning = false;

    newMovement[MOTOR_B].dir = stepsB > 0;
    newMovement[MOTOR_B].stepsMotor = abs(stepsB *2);
    newMovement[MOTOR_B].contMotor = 0;               // Util si quiero hacer movimientos relativos,cargando steps anteriores..
    newMovement[MOTOR_B].velocityUs = vel2us(outputMotors.motorVelPercent);
    newMovement[MOTOR_B].flagRunning = false;

    newMovement[MOTOR_C].dir = stepsC > 0;
    newMovement[MOTOR_C].stepsMotor = abs(stepsC *2);
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