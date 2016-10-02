#ifndef __TOUCHLIB_H__
#define __TOUCHLIB_H__

#include "constantes.h"

extern TaskHandle_t xPantallaHandle;

void vTouchTask(void* parametros)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	UTouch myTouch(6, 5, 4, 3, 2);

	myTouch.InitTouch();
    myTouch.setPrecision(PREC_EXTREME);

    _bln_debug_println(F(">touch-task"));

	for ( ; ; ) {
		if ( myTouch.dataAvailable() )
		{
			myTouch.read();

			Posicion posicion{myTouch.getX(), myTouch.getY()};

			if ( posicion.x != -1 && posicion.y != -1 ) {
				xQueueOverwrite(xTouchQueue, &posicion);
				xTaskNotify(xPantallaHandle, FLAG_PANTALLA_TOUCH, eSetBits);

				while(myTouch.dataAvailable()){ vTaskDelay(DELAY_TAREA_TOUCH_OK); }
			}
			_bln_debug_print(F(">touch-task=")); _bln_debug_println(uxTaskGetStackHighWaterMark(NULL));
		}
		vTaskDelayUntil(&xLastWakeTime, DELAY_TAREA_TOUCH);
	}
}

#endif