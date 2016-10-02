#ifndef __RFIDLIB_H__
#define __RFIDLIB_H__

#include <AVRFreeRTOS.h>
#include <utility/task.h>
#include <SoftwareSerial.h>

#define PIN_RFID_TX        	   13
#define PIN_RFID_RX        	   12 	// No se usa...

extern TaskHandle_t xRFIDHandle;
extern TaskHandle_t xPantallaHandle;
extern QueueHandle_t xTarjetaRFIDQueue;


void vTarjetaRFIDTask(void* parametros) {
	TickType_t xLastWakeTime = xTaskGetTickCount();

	SoftwareSerial RFID(PIN_RFID_TX, PIN_RFID_RX);

	bool leer = false;
	bool bSupervisor;
	uint32_t flags = 0UL;

	RFID.begin(9600);
	RFID.flush();
	while( RFID.available() ) { RFID.read(); }

	_bln_debug_println(F(">tarjeta-rfid-task"));

	for ( ; ; ) {
		xTaskNotifyWait(0UL, 0xFFFFFFFFUL, &flags, 0);

		if ( flags & FLAG_RFID_LEER ) {
			leer = true;
			while( RFID.available() ) { RFID.read(); }
			bSupervisor = false;
		}

		if (flags & FLAG_RFID_LEER_SUPERVISOR) {
			leer = true;
			while( RFID.available() ) { RFID.read(); }
			bSupervisor = true;
		}

		if ( leer && RFID.available() >= LONGITUD_TARJETA_RFID ) {

			xTaskNotify(xPantallaHandle, FLAG_PANTALLA_LEYENDO_RFID, eSetBits);

			leer = false;
			TarjetaRFID tarjeta;

			for ( uint8_t i = 0; i < LONGITUD_TARJETA_RFID; i++ ) {
				tarjeta.codigo[i] = RFID.read();
			}

			_bln_debug_print(F("CODIGO TARJETA= "));
			for (int i = 0; i < LONGITUD_TARJETA_RFID; i++) {
				_bln_debug_print(tarjeta.codigo[i]);
			}
			_bln_debug_println(F(""));

			xQueueSend(xTarjetaRFIDQueue, &tarjeta, ESPERA_COLA);
			if(!bSupervisor)	xTaskNotify(xControlHandle, FLAG_CONTROL_TARJETA_RFID, eSetBits);
			else				xTaskNotify(xControlHandle, FLAG_CONTROL_TARJETA_RFID_SUPERVISOR, eSetBits);

			RFID.flush();

			while( RFID.available() ) { RFID.read(); }

			_bln_debug_print(F(">tarjeta-rfid-task=")); _bln_debug_println(uxTaskGetStackHighWaterMark(NULL));
		}

		vTaskDelayUntil(&xLastWakeTime, DELAY_TAREA_RFID);
	}
}


#endif