//////////////////////////////////////////////////////////////////////////////////////
// RTOS_v2.ino
// **********************************************************************************
// - Se agrega el uso del sensor ultrasónico (modelo: Completo_v1_sin_color.ino).

#include <Streaming.h>
#include <Servo.h> 
#include <Wire.h>
#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"

#include "Constantes.h"

int   g_count = 0;    // count the frequecy
int   g_array[3];     // store the RGB value
int   g_flag = 0;     // filter of RGB queue
float g_SF[3];        // save the RGB Scale factor

bool enLaMeta = false;  // Si detecta blanco y enLaMeta=true, es porque llego a destino.
                        // Esta variable se activa cuando se detecta AMARILLO.
int velRobot = 0, color = BLANCO, count = 0;
bool initColor = true, inicio = false, meta = false, movPermitido = false;

Servo miServo;

SemaphoreHandle_t xColorSemaphore;

#include "Funciones.h"
#include "Inicializar.h"

///////////////////////////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL
//

void loop(){}  

// Fin del programa
///////////////////////////////////////////////////////////////////////////////////////
