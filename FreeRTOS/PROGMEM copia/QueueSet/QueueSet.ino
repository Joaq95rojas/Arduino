////////////////////////////////////////////////////////////////////
// QueueSet.ino
// ----------------------------------------------------------------
//	* "Blocking on Multiple RTOS Objects"
//	* Queue sets can contain queues and semaphores, which together
// 	  are known as queue set members.
//	* 
//
///////////////////////////////////////////////////////////////////
// INCLUDES

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>		// Para poder usar la memoria Flash y
								// ahorrar memoria SRAM.
#include <Wire.h>
#include <Streaming.h>

#include "Constantes.h"

/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

QueueSetHandle_t xQueueSet;
SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore, xEmergenciaSemaphore;
QueueHandle_t xButtonQueue, xComunicacionQueue, xEmergenciaSemQueue, \
				xGolpesQueue, xTiemposQueue;

volatile int8_t tipoTarea = 0;
bool progPausado = true; // El programa comienza pausado
bool tareaEnProceso = false;
bool initBalancin = false;

#ifdef __LCD_SHIELD__
	LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD Keypad Shield
#else
	LiquidCrystal lcd(9, 8, 6, 5, 4, 3);
#endif

#include "Funciones.h"
#include "Handlers.h"
#include "Inicializar.h"

/////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL - LOOP()

void loop() {}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////