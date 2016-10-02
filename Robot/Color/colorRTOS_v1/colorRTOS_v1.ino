//////////////////////////////////////////////////////////////////////////////////////
// RTOS_v1.ino
// **********************************************************************************
// - Implementa lo mismo que colorLento_v5.ino solo que usa FreeRTOS.

#include <Streaming.h>
#include <Wire.h>
#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"

#include "Constantes.h"

int   g_count = 0;    // count the frequecy
int   g_array[3];     // store the RGB value
int   g_flag = 0;     // filter of RGB queue
float g_SF[3];        // save the RGB Scale factor

int color, count = 0;
bool initColor = true;

SemaphoreHandle_t xColorSemaphore;

#include "Funciones.h"
#include "Inicializar.h"

///////////////////////////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL
//

void loop(){}  

// Fin del programa
///////////////////////////////////////////////////////////////////////////////////////
