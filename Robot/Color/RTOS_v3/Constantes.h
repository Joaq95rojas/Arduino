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
#define  DATOS_FINAL_MS    (300 / portTICK_PERIOD_MS)  // 500
#define  PROCESAMIENTO_MS  (80 / portTICK_PERIOD_MS)  // 110
#define  INIT_COLOR_MS     (500 / portTICK_PERIOD_MS)  // 500
#define  TIEMPO_SERVO_MS   (80 / portTICK_PERIOD_MS)    // 100

#define ANGULO_MAXIMO_DEG     250
#define ANGULO_MINIMO_DEG     150
#define PASO_MINIMO_DEG       5
#define DISTANCIA_MAXIMA_CM   5

#define S0     45
#define S1     47
#define S2     49
#define S3     51
#define OUT    19

#define STACK_COLOR        300
#define STACK_CALLBACK     256
#define STACK_COUNT        300
#define STACK_INIT         256
#define STACK_HCSR04       300
#define PRIORIDAD_COLOR    tskIDLE_PRIORITY + 2
#define PRIORIDAD_CALLBACK tskIDLE_PRIORITY + 1
#define PRIORIDAD_COUNT    tskIDLE_PRIORITY + 1
#define PRIORIDAD_INIT     tskIDLE_PRIORITY + 1
#define PRIORIDAD_HCSR04   tskIDLE_PRIORITY + 2

#define PIN_ECHO  4

#define AMARILLO  0
#define BLANCO    1
#define ROJO      2
#define AZUL      3

#define COND_AMARILLO  (R > 2*B) && (G>1.5*B)
#define COND_BLANCO    (R>200) && (G>200) && (B>200)
#define COND_ROJO      (R>2*G) && (R>2*B)
#define COND_AZUL      (B>R) && (B>G)

#endif
