# Componente para control de motores paso a paso con interpolación lineal para ESP32 SDK-IDF

Este componente proporciona funcionalidades avanzadas para el control de motores paso a paso, incorporando interpolación lineal para un movimiento suave y preciso. Está diseñado para ser utilizado con la plataforma ESP32 SDK-IDF.

## Funciones Disponibles

### Inicialización

```c
void initMotors(pap_position_control_config_t config);
```

- Inicializa los motores con la configuración proporcionada.

### Descripción de la Estructura de Inicialización (`pap_position_control_config_t`)

La estructura `pap_position_control_config_t` se utiliza para configurar los parámetros necesarios antes de inicializar los motores. Aquí hay un ejemplo de cómo se podría utilizar:

```c
pap_position_control_config_t motorConfig = {
    .relativePos = 1,                           // Posición relativa activada
    .motorsGpio = {
        .enablePin = 5,                         // Pin para habilitar los motores
        .motors = {
            { .stepPin = 2, .dirPin = 4 },      // Configuración del motor Q1
            { .stepPin = 14, .dirPin = 12 },    // Configuración del motor Q2
            { .stepPin = 27, .dirPin = 26 },    // Configuración del motor Q3
        }
    },
    .endOfTravelsGpio = {
        .pinSensor = { 18, 19, 21 }             // Pines de sensores de final de recorrido
    },
    .safetyLimits = {
        .safetyLimit = { 500, 700, 1000 }       // Límites de seguridad para los motores
    },
    .callbackErrorPointer = &errorCallbackFunction   // Puntero a la función de manejo de errores
};

initMotors(motorConfig);                        // Inicialización de los motores con la configuración creada
```


### Movimiento del Eje

```c
void moveAxis(int32_t stepsQ1, int32_t stepsQ2, int32_t stepsQ3);
```

- Mueve los ejes la cantidad especificada de pasos para cada motor.

### Velocidad

```c
void setVel(uint8_t velocity);
uint8_t getVelPercent(void);
```

- Establece el porcentaje de velocidad de los motores.
- Obtiene el porcentaje de velocidad actual.

### Control de Habilitación

```c
void setEnableMotors(void);
void setDisableMotors(void);
```

- Habilita y deshabilita los motores.

### Estado y Posición

```c
uint8_t areMotorsMoving(void);
void resetAbsPosition(void);
absolute_position_t getAbsPosition(void);
```

- Verifica si los motores están en movimiento.
- Resetea la posición absoluta.
- Obtiene la posición absoluta actual.

### Otras Funciones

```c
void autoHome(void);
void stopEmergency(void);
```

- Realiza el procedimiento de auto-home.
- Detiene de emergencia todos los motores.

## Configuración de Defines Principales

### Velocidades y Precisión

- `MIN_VELOCITY_US`: Velocidad mínima en microsegundos.
- `MAX_VELOCITY_US`: Velocidad máxima en microsegundos.
- `VEL_PERCENT_DEFAULT`: Velocidad por defecto al iniciar los motores.

### Definición de Rampa

- `RAMP_DEFINITION`: Puntos de definición de la velocidad en la rampa.
- `BASE_PERIOD_TIMER`: Periodo base del temporizador.
- `RAMP_MAX_ACCEL`: Aceleración máxima en la rampa.
- `VAL_RAMP_PERCENT`: Porcentaje de la rampa.

### Otros Ajustes

- `AVOID_ACCEL_RAMP`: Define si se debe evitar la aceleración en la rampa.
- `MOTOR_ENABLE`, `MOTOR_DISABLE`: Estados de habilitación y deshabilitación de los motores.
- `CANT_MOTORS`: Cantidad de motores.
- `QUEUE_MOVES_LENGTH`: Longitud de la cola de movimientos.
- `DELAY_BETWEEN_MOVES`: Retraso entre movimientos en microsegundos.
- `INTERPOLATION_PRECISION`: Precisión de la interpolación.

Estos defines permiten ajustar diferentes parámetros del control de motores y la interpolación. Modifica estos valores según las necesidades específicas de tu aplicación.


## Estructuras y Enums

### Enumeraciones

- `enum motor_name`: Enumeración para identificar los motores.
- `enum ramp_state`: Estados de la rampa de movimiento.
- `enum callback_error_responses`: Respuestas de error para los callbacks.

### Estructuras

- `output_motor_pins_t`: Pines de salida para los motores.
- `input_sensor_pins_t`: Pines de sensores de entrada.
- `output_motors_pins_t`: Configuración de pines de salida para los motores.
- `safety_limits_t`: Límites de seguridad.
- `pap_position_control_config_t`: Configuración de control de posición PAP.
- `motor_control_t`: Control del motor.
- `motors_control_t`: Control de varios motores.
- `individual_ramp_t`: Rampa individual.
- `control_ramp_t`: Control de rampa.
- `absolute_position_t`: Posición absoluta.

Para obtener detalles sobre cómo utilizar cada función o estructura, consulta el código fuente y los comentarios asociados.
