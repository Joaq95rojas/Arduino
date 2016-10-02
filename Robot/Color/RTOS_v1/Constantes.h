#ifndef _CONSTANTES_H_
#define _CONSTANTES_H_

//////////////////////////////////////////////////////////////////////////////////
// INCLUDES
//
#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <Streaming.h>
#include <Wire.h>

//////////////////////////////////////////////////////////////////////////////////
// DEFINES
//
#define  DATOS_FINAL_MS    (180 / portTICK_PERIOD_MS)
#define  PROCESAMIENTO_MS  (50 / portTICK_PERIOD_MS)
#define  INIT_COLOR_MS     (200 / portTICK_PERIOD_MS)
#define  MOTORES_MS        (500 / portTICK_PERIOD_MS)

#define S0     45
#define S1     47
#define S2     49
#define S3     51
#define OUT    2

#define PIN_MOTOR_D1 6
#define PIN_MOTOR_D2 7
#define PIN_MOTOR_D3 8 
#define PIN_MOTOR_D4 9
#define PIN_MOTOR_T1 10 
#define PIN_MOTOR_T2 11
#define PIN_MOTOR_T3 12 
#define PIN_MOTOR_T4 13

#define BIT_PIN(i)  digitalPinToBitMask(i)
#define OUT_PIN(i)  portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)  portModeRegister(digitalPinToPort(i))
#define IN_PIN(i)  *portInputRegister(digitalPinToPort(i))

#define ENTRADA 0
#define SALIDA  1

#define STACK_COLOR        128
#define STACK_CALLBACK     512
#define STACK_COUNT        256
#define STACK_INIT         256
#define STACK_MOTOR        256
#define PRIORIDAD_COLOR    tskIDLE_PRIORITY + 2
#define PRIORIDAD_CALLBACK tskIDLE_PRIORITY + 2
#define PRIORIDAD_COUNT    tskIDLE_PRIORITY + 2
#define PRIORIDAD_INIT     tskIDLE_PRIORITY + 2
#define PRIORIDAD_MOTOR    tskIDLE_PRIORITY + 2

#define AMARILLO  0
#define BLANCO    1
#define ROJO      2
#define AZUL      3

#endif
