////////////////////////////////////////////////////////////////////
// TaskCom.ino
// ----------------------------------------------------------------
//	* Habrán 3 tareas:
//		- vEmergenciaTask()
//		- vComunicacionTask()
//		- vControlGeneralTask()
//	* Las dos primeras se comunicarán con la restante a través de
//	  un set de colas de mensajes usando al puerto serie.
//
///////////////////////////////////////////////////////////////////
// INCLUDES

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <Wire.h>

#include "Constantes.h"

/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

QueueSetHandle_t xQueueSet;
SemaphoreHandle_t xEmergenciaSemaphore;
QueueHandle_t xComunicacionQueue, xEmergenciaSemQueue;

volatile uint8_t tipoTarea = 0;
bool progPausado = true; // El programa comienza pausado
bool tareaEnProceso = false;
bool initBalancin = false;

#ifdef __LCD_SHIELD__
	LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD Keypad Shield
#else
	LiquidCrystal lcd(9, 8, 6, 5, 4, 3);
#endif

struct T t_neto;

#include "Funciones.h"
#include "Handlers.h"
#include "Inicializar.h"

/////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL - LOOP()

void loop() {}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////