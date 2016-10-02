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

#define  DATOS_FINAL_MS    (300 / portTICK_PERIOD_MS)
#define  PROCESAMIENTO_MS  (80 / portTICK_PERIOD_MS)
#define  INIT_COLOR_MS     (250 / portTICK_PERIOD_MS)
#define  TIEMPO_SERVO_MS   ( 80 / portTICK_PERIOD_MS)

#define ANGULO_MAXIMO_DEG     250
#define ANGULO_MINIMO_DEG     150
#define PASO_MINIMO           5
#define DISTANCIA_MAXIMA_CM   5

#define STACK_COLOR        350
#define STACK_CALLBACK     256
#define STACK_COUNT        256
#define STACK_INIT         256
#define STACK_HCSR04       350  //128
#define STACK_DETECTION    128
#define STACK_MPU          256
#define PRIORIDAD_COLOR      tskIDLE_PRIORITY + 3
#define PRIORIDAD_CALLBACK   tskIDLE_PRIORITY + 2
#define PRIORIDAD_COUNT      tskIDLE_PRIORITY + 2
#define PRIORIDAD_INIT       tskIDLE_PRIORITY + 2    
#define PRIORIDAD_HCSR04     tskIDLE_PRIORITY + 3
#define PRIORIDAD_DETECTION  tskIDLE_PRIORITY + 3
#define PRIORIDAD_MPU        tskIDLE_PRIORITY + 2

#define AMARILLO  0
#define BLANCO    1
#define ROJO      2
#define AZUL      3

#define RAD_2_DEG   57.2958

#define COND_AMARILLO  (R > 2*B) && (G>1.5*B)
#define COND_BLANCO    (R>200) && (G>200) && (B>200)
#define COND_ROJO      (R>2*G) && (R>2*B)
#define COND_AZUL      (B>R) && (B>G)

#endif
