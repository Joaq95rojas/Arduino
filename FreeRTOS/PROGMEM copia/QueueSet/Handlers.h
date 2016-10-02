#ifndef _TAREAS_H_
#define _TAREAS_H_

#include "Constantes.h"

extern SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore, xEmergenciaSemaphore;

/////////////////////////////////////////////////////////////////
// HANDLERS

void  vInterruptHandler( void )
{
    signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xButtonSemaphore , \
    	(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();
}


void  vSensorHandler( void )
{
    signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSensorSemaphore , \
    	(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();
}


void  vEmergenciaHandler( void )
{
    signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xEmergenciaSemaphore , \
    	(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();
}


#endif