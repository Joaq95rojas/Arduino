#ifndef _CONSTANTES_H_
#define _CONSTANTES_H_

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////
// DEFINES

//#define __LCD_SHIELD__	// Comentar la línea en caso de usar un LCD 20x4

#define ENTRADA	0
#define SALIDA	1

#define BIT_PIN(i)  digitalPinToBitMask(i)
#define OUT_PIN(i)  portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)	portModeRegister(digitalPinToPort(i))
#define IN_PIN(i)	*portInputRegister(digitalPinToPort(i))

#define PERIODO_INT_MS          ( 1000 / portTICK_PERIOD_MS )
#define TIEMPO_RUIDO_BOTON_MS   ( 250 / portTICK_PERIOD_MS )
#define TIEMPO_RUIDO_SENSOR_MS  ( 50 / portTICK_PERIOD_MS )
#define TIEMPO_EMERG_BOTON_MS	( 100 / portTICK_PERIOD_MS )
#define EMERGENCIA		2 	// D2
#define INT_EMERG		0	// INT0
#define INT_PIN         19	// D19
#define INT_NUM         4 	// INT4
#define PIN_SENSOR      18	// D18
#define PIN_SENSOR_INT  5 	// INT5
#define PIN_MOTOR       30	// D30

// Botones a identificar
#define btnNETAS          53
#define btnSETUP          23
#define btnREPR           25
#define btnPARADAxPROC    27
#define btnPAUSA          51
// LEDs asociados
#define ledNETAS          32
#define ledSETUP          34
#define ledREPR           38
#define ledPARADAxPROC    36
#define ledPAUSA          40

// Constantes para tareas
#define PRIORIDAD_HORA			tskIDLE_PRIORITY + 2
#define PRIORIDAD_BOTONES		tskIDLE_PRIORITY + 2
#define PRIORIDAD_ASSIGN		tskIDLE_PRIORITY + 1
#define PRIORIDAD_BALANCIN		tskIDLE_PRIORITY + 2
#define PRIORIDAD_EMERGENCIA	tskIDLE_PRIORITY + 3

#define STACK_HORA			configMINIMAL_STACK_SIZE*4
#define STACK_BOTONES		1024
#define STACK_ASSIGN		2048
#define STACK_BALANCIN		1024
#define STACK_EMERGENCIA	1024

#define CONVERTIR_HORA(a)	a.minutos/60.0

struct T{
      volatile uint8_t segundos;
      volatile uint8_t minutos;
      volatile double tiempoConvertido;
};

#endif