/////////////////////////////////////////////////////////////////
// Completo.ino
// --------------------------------------------------------------
//	* Se harán las siguientes mejoras respecto a contPiezas.ino:
//    - Se agregará el apagado de la motor trifásico cuando se
//      complete el conteo de las piezas a fabricar.
//    - Se optimizaron algunas funciones como digitalWrite().
//    - Agregar tarea que atienda al botón de Emergencia.
//    - Modularizar el código. 
//
/////////////////////////////////////////////////////////////////
// INCLUDES

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <Wire.h>

#include "Constantes.h"

/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore, xEmergenciaSemaphore;
QueueHandle_t xButtonQueue;

// xLcdQueue será usada por todas las tareas que deseen escribir el LCD.

volatile uint8_t tipoTarea = 0, tipoTareaAnterior = 0;
boolean lastStateLED = false;
boolean progPausado = true; // El programa comienza pausado
boolean initBalancin = false;
volatile uint16_t contGolpes = 0;
uint16_t golpesBalancinTotal = 20;

#ifdef __LCD_SHIELD__
	LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD Keypad Shield
#else
	LiquidCrystal lcd(9, 8, 6, 5, 4, 3);
#endif

struct T t_neto,t_setup,t_pxproc,t_repr,t_pausa,t_hora;

#include "Funciones.h"
#include "Handlers.h"
#include "Inicializar.h"

/////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL - LOOP()

void loop() {}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////
