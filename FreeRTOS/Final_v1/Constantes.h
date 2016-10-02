#ifndef _CONSTANTES_H_
#define _CONSTANTES_H_

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <Streaming.h>
#include <Wire.h>
#include "Wifi.h"

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
#define TIEMPO_RUIDO_SENSOR_MS  ( 10 / portTICK_PERIOD_MS )
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
#define TIEMPOS_QUEUE_LENGTH		5
#define BUTTON_QUEUE_SIZE		sizeof( int8_t )	// tipoTarea
#define COMUNICACION_QUEUE_SIZE	sizeof(uint16_t)
#define GOLPES_QUEUE_SIZE		sizeof(uint16_t)
#define TIEMPOS_QUEUE_SIZE		sizeof( double )
// En COMBINED_LENGTH se colocan todas las colas que irán al Set.
// NO TIENEN QUE IR BUTTON_QUEUE!!
#define COMBINED_LENGTH			( COMUNICACION_QUEUE_LENGTH + \
								  TIEMPOS_QUEUE_LENGTH 	)

// Constantes para tareas
#define PRIORIDAD_HORA			tskIDLE_PRIORITY + 2
#define PRIORIDAD_BOTONES		tskIDLE_PRIORITY + 2
#define PRIORIDAD_ASSIGN		tskIDLE_PRIORITY + 2
#define PRIORIDAD_BALANCIN		tskIDLE_PRIORITY + 3
#define PRIORIDAD_EMERGENCIA	tskIDLE_PRIORITY + 4
// -------------------------------------------------
#define PRIORIDAD_CTRLGRAL		tskIDLE_PRIORITY + 3
#define PRIORIDAD_COMUNICACION	tskIDLE_PRIORITY + 2

// NO SER RATA CON LA ASIGNACIÓN DE MEMORIA DURANTE LAS PRUEBAS
#define STACK_HORA			256
#define STACK_BOTONES		512//configMINIMAL_STACK_SIZE
#define STACK_ASSIGN		512//configMINIMAL_STACK_SIZE
#define STACK_BALANCIN		256//configMINIMAL_STACK_SIZE
#define STACK_EMERGENCIA	256
// ---------------------------------------------------------------------
#define STACK_CTRLGRAL		512
#define STACK_COMUNICACION	512

// Para las posición del LCD en setCursor()
#define TASK_CURSOR			0
#define INSTR_CURSOR		1
#define ERROR_CURSOR		2
#define INFO_CURSOR			2
#define HORA_CURSOR			3

struct T{
      volatile uint8_t segundos;
      volatile uint8_t minutos;
};


#endif
