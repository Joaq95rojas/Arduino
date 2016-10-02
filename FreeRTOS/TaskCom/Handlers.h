#ifndef _TAREAS_H_
#define _TAREAS_H_

#include "Constantes.h"

extern SemaphoreHandle_t xEmergenciaSemaphore;

/////////////////////////////////////////////////////////////////
// HANDLERS

void  vEmergenciaHandler( void )
{
    signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xEmergenciaSemaphore , \
    	(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();
}


#endif