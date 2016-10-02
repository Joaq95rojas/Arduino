#ifndef _CONSTANTES_H_
#define _CONSTANTES_H_

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
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

#define VELOCIDAD_SERIAL	9600
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

// Especificaciones del Queue Set
#define BUTTON_QUEUE_LENGTH			4
#define COMUNICACION_QUEUE_LENGTH	5
#define EMERGENCIA_SEM_LENGTH		1 	// Único valor posible
#define GOLPES_QUEUE_LENGTH			1
#define BUTTON_QUEUE_SIZE		sizeof( double)
#define COMUNICACION_QUEUE_SIZE	sizeof(uint16_t)
#define GOLPES_QUEUE_SIZE		sizeof(uint16_t)
#define COMBINED_LENGTH			( COMUNICACION_QUEUE_LENGTH + \
								  GOLPES_QUEUE_LENGTH )

// Constantes para tareas
#define PRIORIDAD_HORA			tskIDLE_PRIORITY + 2
#define PRIORIDAD_BOTONES		tskIDLE_PRIORITY + 2
#define PRIORIDAD_ASSIGN		tskIDLE_PRIORITY + 1
#define PRIORIDAD_BALANCIN		tskIDLE_PRIORITY + 2
#define PRIORIDAD_EMERGENCIA	tskIDLE_PRIORITY + 3
// -------------------------------------------------
#define PRIORIDAD_CTRLGRAL		tskIDLE_PRIORITY + 2
#define PRIORIDAD_COMUNICACION	tskIDLE_PRIORITY + 2

#define STACK_HORA			configMINIMAL_STACK_SIZE*4
#define STACK_BOTONES		1024//configMINIMAL_STACK_SIZE
#define STACK_ASSIGN		2048//configMINIMAL_STACK_SIZE
#define STACK_BALANCIN		1024//configMINIMAL_STACK_SIZE
#define STACK_EMERGENCIA	configMINIMAL_STACK_SIZE*2
// ---------------------------------------------------------------------
#define STACK_CTRLGRAL		4096//configMINIMAL_STACK_SIZE
#define STACK_COMUNICACION	1024//configMINIMAL_STACK_SIZE

struct T{
      volatile uint8_t segundos;
      volatile uint8_t minutos;
      double tiempoConvertido;
};
/*
// Cadenas con los mensajes que se guardan en MEMORIA FLASH.
const char msg0[]  PROGMEM = "xQueueRec ERROR ";
const char msg1[]  PROGMEM = "Seleccione tarea";
const char msg2[]  PROGMEM = " *** ERROR ***  ";
const char msg3[]  PROGMEM = "                ";
const char msg4[]  PROGMEM = "                    ";
const char msg5[]  PROGMEM = "0";
const char msg6[]  PROGMEM = ":";
const char msg7[]  PROGMEM = "Piezas: ";
const char msg8[]  PROGMEM = " / ";
const char msg9[]  PROGMEM = "xQueueSend ERROR";
const char msg10[] PROGMEM = "    PAUSADO    ";
const char msg11[] PROGMEM = "Produccion";
const char msg12[] PROGMEM = "Setup";
const char msg13[] PROGMEM = "Reproceso";
const char msg14[] PROGMEM = "PxProceso";
const char msg15[] PROGMEM = "Hora: ";
const char msg16[] PROGMEM = "Presione INICIO";
const char msg17[] PROGMEM = " ERROR BALANCIN ";
const char msg18[] PROGMEM = "Tarea finalizada";
const char msg19[] PROGMEM = "Tiempo Conv: ";
const char msg20[] PROGMEM = "***EMERGENCIA***";
const char msg21[] PROGMEM = " QueueSet ERROR ";
const char msg22[] PROGMEM = "ERROR DE COMUNIC";*/

#endif