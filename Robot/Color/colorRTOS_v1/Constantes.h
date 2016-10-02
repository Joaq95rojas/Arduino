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
#define S0     45
#define S1     47
#define S2     49
#define S3     51
#define OUT    2

#define BIT_PIN(i)  digitalPinToBitMask(i)
#define OUT_PIN(i)  portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)  portModeRegister(digitalPinToPort(i))
#define IN_PIN(i)  *portInputRegister(digitalPinToPort(i))

#define ENTRADA 0
#define SALIDA  1

#define STACK_COLOR        512
#define STACK_CALLBACK     768
#define STACK_COUNT        512
#define STACK_INIT        512
#define PRIORIDAD_COLOR    tskIDLE_PRIORITY + 2
#define PRIORIDAD_CALLBACK tskIDLE_PRIORITY + 2
#define PRIORIDAD_COUNT    tskIDLE_PRIORITY + 2
#define PRIORIDAD_INIT    tskIDLE_PRIORITY + 2

#define AMARILLO  0
#define BLANCO    1
#define ROJO      2
#define AZUL      3

#endif
