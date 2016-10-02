
/*
 * TODO:
 *
 * Corregir recepcion de "NombreOperario".
 *
 * También enviar cantidad de VARIANTES máxima para dimensionar el vector.
 *
 * CANTIDAD MÁXIMA DE CARACTERES EN EL NOMBRE DEL OPERARIO = 25.
 *
 * Implementar caracteres de control ASCII para el envío de datos.
 *
 * Se necesitan mMensajeMQTT[160]???
 *
 * NOTA: No hace falta mandar nombre del operario o ID máquina porque se publica
 * 		 en el tópico correspondiente.
 *
 * Entrar a modo de booteo si se mantiene [presionada una tecla][Poner botón escondido externo].
 *
 * #######################################################
 * ESTE PROGRAMA AGREGA LAS PUBLICACIONES Y SUSCRIPCIONES.
 *
 * [P] -> placas/andif-empaque-1/conectado
 *
 * [P] -> placas/andif-empaque-1/tagRFID
 * [S] placas/andif-empaque-1/accion/respuestaRFID 			-> [NombreOperario] / 0
 * [S] -> placas/andif-empaque-1/accion/cantTotalArticulos	-> 24
 *
 * [P] -> placas/andif-empaque-1/Pedidos/Articulos
 * [S] placas/andif-empaque-1/accion/respuestaPedidos -> 101;202;203;209;218;501
 * [S]												  -> 504;515;530;535;544;550
 * [S]												  -> 555;590;850;851;852;853
 * [S] 												  -> 854;855;856;857;858;860
 *
 * [P] -> placas/andif-empaque-1/Pedidos/Lista_Articulo
 * [S] placas/andif-empaque-1/accion/respuestaListaArticulo -> 101 AZUL;101 DOR;101 DOR COMB x2;101 IG;101 PLAT
 * [S] 														-> 501;501 C-C;501 COMB x2;501 DOR;501 IG
 * [S]														-> 504;504 COMB x2;504 IG;504 DOR
 * [S]														-> 515;515 COMB x2;515 IG
 * [S]														-> 544 DER;544 IZQ
 * [S]														-> 850;850 COMB x2
 * [S]														-> 852;852 IG;852 FR
 * [S]														-> 853 DER;853 IZQ
 * [S]														-> 854 DER;854 IZQ
 * [S]														-> 855;855 DOR;855 IG
 * [S]														-> 857;857 F16;857 IG;857/40;857/40 F16;857/40 IG
 * [S]														-> 858;858 AZUL;858 AZUL FC;858 IG
 * [S]														-> 860;860 IG
 *
 * [P] -> placas/andif-empaque-1/LlamarSupervisor
 * [S] placas/andif-empaque-1/accion/respuestaSupervisor  	->
 *
 * [P] -> placas/andif-empaque-1/Pedidos/confirmacionSupervisor
 * [S] "placas/andif-empaque-1/accion/confirmacionSupervisor"	-> [1 / 0]
 *
 * [P] -> placas/andif-empaque-1/Fin_Tarea
 * [S] -> placas/andif-empaque-1/Fin_Tarea_ACK
 *
 */

// Include
//

#include <AVRFreeRTOS.h>
#include <utility/task.h>
#include <utility/timers.h>
#include <utility/semphr.h>
#include <utility/queue.h>

#include <espduino.h>
#include <mqtt.h>
#include <UTFT.h>
#include <UTouch.h>

#include "constantes.h"
#include "util.h"
#include "comunicacionmqtt.h"
#include "TimerLib.h"
#include "PantallaLib.h"
#include "RFIDLib.h"
#include "TouchLib.h"

#include <SoftwareSerial.h>

// Variables globales
//

SemaphoreHandle_t xMutexArticulos = NULL;
SemaphoreHandle_t xMutexTiempos = NULL;
SemaphoreHandle_t xMutexOperario = NULL;

TaskHandle_t xComunicacionHandle = NULL;
TaskHandle_t xRFIDHandle = NULL;
TaskHandle_t xControlHandle = NULL;
TaskHandle_t xPantallaHandle = NULL;

QueueHandle_t xComunicacionRespuestasCola = NULL;
QueueHandle_t xListaArticuloQueue = NULL;
QueueHandle_t xTarjetaRFIDQueue = NULL;
QueueHandle_t xuIntQueue = NULL;
QueueHandle_t xTouchQueue = NULL;
QueueHandle_t xBoolQueue = NULL;
QueueHandle_t xArticulosQueue = NULL;
QueueHandle_t xInfoQueue = NULL;

extern TimerHandle_t xInfoHandle;

char articulos[CANTIDAD_ARTICULOS_POR_PANTALLA][LONGITUD_ARTICULO];
char NombreOperario[LONGITUD_NOMBRE_OPERARIO] = "";
char ArticuloEmpaque[LONGITUD_LISTA_ARTICULO] = "";

uint8_t TipoTarea = SIN_TAREA;
uint8_t CantidadTotalArticulos = 0;
uint8_t CantidadPantallasArticulos = 0;
uint16_t CT = 0;

// Funciones
//

void setup()
{
	#ifdef DEBUG
		debugSerial.begin(19200);
	#endif

	esp8266Serial.begin(19200);

	_bln_debug_println(F("Iniciado"));

	xMutexArticulos = xSemaphoreCreateMutex();
	xMutexTiempos   = xSemaphoreCreateMutex();
	xMutexOperario = xSemaphoreCreateMutex();

	xHoraHandle = xTimerCreate(NULL, PERIODO_TIMER, pdTRUE, NULL, vHoraTimer);
	xInfoHandle = xTimerCreate(NULL, PERIODO_INFO, pdTRUE, NULL, vInfoTimer);
	xTaskCreate(vTouchTask, NULL, STACK_TOUCH, NULL, PRIORIDAD_TOUCH, NULL);
	xTaskCreate(vPantallaTask, NULL, STACK_PANTALLA, NULL, PRIORIDAD_PANTALLA, &xPantallaHandle);
	xTaskCreate(vComunicacionTask, NULL, STACK_COMUNICACION, NULL, PRIORIDAD_COMUNICACION, &xComunicacionHandle);
	xTaskCreate(vTarjetaRFIDTask, NULL, STACK_RFID, NULL, PRIORIDAD_RFID, &xRFIDHandle);
	xTaskCreate(vControlTask, NULL, STACK_CONTROL, NULL, PRIORIDAD_CONTROL, &xControlHandle);

	xTouchQueue = xQueueCreate(1, sizeof(Posicion));
	xComunicacionRespuestasCola = xQueueCreate(1, sizeof(void*));
	xTarjetaRFIDQueue = xQueueCreate(1, sizeof(TarjetaRFID));
	xuIntQueue = xQueueCreate(1, sizeof(uint8_t));
	xListaArticuloQueue = xQueueCreate(1, sizeof(Lista));
	xBoolQueue = xQueueCreate(1, sizeof(bool));
	xArticulosQueue = xQueueCreate(1, sizeof(char*[CANTIDAD_ARTICULOS_POR_PANTALLA]));
	xInfoQueue = xQueueCreate(1, sizeof(struct sInfo));

	vTaskStartScheduler();
}


void loop() {}


// Funciones
//


void vComunicacionTask(void* parametros) {

	#ifdef DEBUG
		ESP esp = ESP(&esp8266Serial, &debugSerial, PIN_WIFI);
	#else
		ESP esp = ESP(&esp8266Serial, PIN_WIFI);
	#endif

	MQTT mqtt = MQTT(&esp);
	uint32_t flags = 0UL;
	void* respuesta;
	bool Booteo = true;

	ComunicacionMQTT comunicacion = ComunicacionMQTT(&esp, &mqtt);

	comunicacion.setComunicacionHandle(xComunicacionHandle);
	comunicacion.setControlHandle(xControlHandle);
	comunicacion.setBoolQueue(xBoolQueue);
	comunicacion.setArticulosQueue(xArticulosQueue);
	comunicacion.setListaArticuloQueue(xListaArticuloQueue);
	comunicacion.setInfoQueue(xInfoQueue);
	comunicacion.setMutexOperario(xMutexOperario);

	comunicacion.setCallbackMQTTConectado(callbackMQTTConectado);
	comunicacion.setCallbackMQTTDesconectado(callbackMQTTDesconectado);
	comunicacion.setCallbackMQTTPublicado(callbackMQTTPublicado);
	comunicacion.setCallbackMQTTDatos(callbackMQTTDatos);
	comunicacion.setCallbackEstadoWifi(callbackEstadoWifi);

	_bln_debug_println(F(">comunicacion-task"));

	for ( ; ; ) {
		comunicacion.procesar();

		xTaskNotifyWait(0UL, 0xFFFFFFFFUL, &flags, 0);

		// Conectado.
		if ( flags & FLAG_COMUNICACION_MQTT_CONECTADO ) {
			_bln_debug_println(F("FLAG_COMUNICACION_MQTT_CONECTADO"));

			comunicacion.marcarMQTTComoConectado();
			comunicacion.configurarSuscripciones();

			if (Booteo) {
				Booteo = false;
				xTaskNotify(xPantallaHandle, FLAG_PANTALLA_RFID, eSetBits);
				xTaskNotify(xRFIDHandle, FLAG_RFID_LEER, eSetBits);
				// CUANDO BOOTEA, COMIENZA LA CUENTA
				xTimerStart(xInfoHandle,0);
			}
		}

		// Desconectado.
		if ( flags & FLAG_COMUNICACION_MQTT_DESCONECTADO ) {
			_bln_debug_println(F("FLAG_COMUNICACION_MQTT_DESCONECTADO"));
			comunicacion.marcarMQTTComoDesconectado();
		}

		// Publicado.
		if ( flags & FLAG_COMUNICACION_PUBLICADO ) {
			_bln_debug_println(F("FLAG_COMUNICACION_PUBLICADO"));
			//
		}

		// Recibiendo datos.
		if ( flags & FLAG_COMUNICACION_DATOS ) {
			_bln_debug_println(F("FLAG_COMUNICACION_DATOS"));

			if ( xQueueReceive(xComunicacionRespuestasCola, &respuesta, 0) == pdTRUE ) {

				comunicacion.obtenerDatosMQTT(respuesta);
			}
		}

		// Estado WIFI.
		if ( flags & FLAG_COMUNICACION_ESTADO_WIFI ) {

			if ( xQueueReceive(xComunicacionRespuestasCola, &respuesta, 0) == pdTRUE ) {
				comunicacion.leerEstadoWifi(respuesta);
			}
		}

		// Wifi desconectado.
		if ( flags & FLAG_COMUNICACION_WIFI_DESCONECTADO ) {
			_bln_debug_println(F("FLAG_COMUNICACION_WIFI_DESCONECTADO"));
			//
		}

		// Enviar tiempo.
		if ( flags & FLAG_COMUNICACION_ENVIAR_TIEMPO ) {
			_bln_debug_println(F("FLAG_COMUNICACION_ENVIAR_TIEMPO"));
			//
		}

		// Verificar Conexión de usuario RFID válida
		if (flags & FLAG_COMUNICACION_CONEXION_RFID) {
			_bln_debug_println(F("FLAG_COMUNICACION_CONEXION_RFID"));

			TarjetaRFID tarjeta;

			if ( xQueueReceive(xTarjetaRFIDQueue, &tarjeta, 0) == pdTRUE ) {
				comunicacion.publicarTagRFID(&tarjeta,false);
			}
		}

		// Pedir lista de artículos
		if ( flags & FLAG_COMUNICACION_PEDIR_ARTICULOS ) {
			_bln_debug_println(F("FLAG_COMUNICACION_PEDIR_ARTICULOS"));

			uint8_t PedidoLista = 0;

			if ( xQueueReceive(xuIntQueue, &PedidoLista, 0) == pdTRUE ){
				comunicacion.pedirArticulos( PedidoLista );
			}
		}

		// Enviar solicitud para recibir variantes de un artículo
		if ( flags & FLAG_COMUNICACION_LISTA_ARTICULO ) {
			_bln_debug_println(F("FLAG_COMUNICACION_LISTA_ARTICULO"));

			uint8_t I;
			char S[LONGITUD_ARTICULO];

			if ( xQueueReceive(xuIntQueue, &I, 0) == pdTRUE ) {

				xSemaphoreTake(xMutexArticulos, portMAX_DELAY);
				snprintf_P(S, sizeof(S), PSTR("%s"), articulos[I-1]);
				xSemaphoreGive(xMutexArticulos);

				comunicacion.pedirListaArticulo(S);
			}
		}

		// Aviso de llamada al supervisor
		if (flags & FLAG_COMUNICACION_LLAMADA_SUPERVISOR) {
			_bln_debug_println(F("FLAG_COMUNICACION_LLAMADA_SUPERVISOR"));
			comunicacion.llamarSupervisor();
		}

		if (flags & FLAG_COMUNICACION_PEDIDO_SUPERVISOR) {
			_bln_debug_println(F("FLAG_COMUNICACION_PEDIDO_SUPERVISOR"));

			TarjetaRFID tarjeta;

			if ( xQueueReceive(xTarjetaRFIDQueue, &tarjeta, 0) == pdTRUE ) {
				comunicacion.publicarTagRFID(&tarjeta,true);
			}
		}

		// Envia los datos de la tarea cada 30 segundos
		if (flags & FLAG_COMUNICACION_ENVIAR_INFO) {
			_bln_debug_println(F("FLAG_COMUNICACION_ENVIAR_INFO"));
			struct sInfo sI;
			xSemaphoreTake(xMutexTiempos,portMAX_DELAY);
			sI.T[0] = tProd;
			sI.T[1] = tSetup;
			sI.T[2] = tPxP;
			sI.T[3] = tReproc;
			sI.TipoTarea = TipoTarea;
			xSemaphoreGive(xMutexTiempos);
			strcpy(sI.Art, ArticuloEmpaque);
			sI.CT = CT;
			xQueueSend(xInfoQueue, &sI, ESPERA_COLA);
			comunicacion.EnviarInfo();
		}

		if ( flags != 0UL ) {
			_bln_debug_print(F(">comunicacion-Task=")); _bln_debug_println(uxTaskGetStackHighWaterMark(NULL));
		}

		vTaskDelay(DELAY_TAREA_COMUNICACION);
	}
}


void callbackMQTTConectado(void* response) {
	xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_MQTT_CONECTADO, eSetBits);
}


void callbackMQTTDesconectado(void* response) {
	xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_MQTT_DESCONECTADO, eSetBits);

	xTaskNotify(xRFIDHandle, FLAG_RFID_LEER, eSetBits);
}


void callbackMQTTPublicado(void* response) {
	xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_PUBLICADO, eSetBits);
}


void callbackMQTTDatos(void* response) {
	xQueueOverwrite(xComunicacionRespuestasCola, &response);
	xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_DATOS, eSetBits);
}


void callbackEstadoWifi(void* response) {
	xQueueOverwrite(xComunicacionRespuestasCola, &response);
	xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_ESTADO_WIFI, eSetBits);
}



void vControlTask(void* parametros) {
	uint32_t flags = 0UL;
	uint8_t contador_articulos = 0;

	_bln_debug_println(F(">control-task"));

	xTaskNotify(xPantallaHandle, FLAG_PANTALLA_CONECTANDO_WIFI, eSetBits);

	for ( ; ; ) {
		xTaskNotifyWait(0UL, 0xFFFFFFFFUL, &flags, portMAX_DELAY);

		// Tarjeta RFID
		if ( flags & FLAG_CONTROL_TARJETA_RFID ) {
			_bln_debug_println(F("FLAG_CONTROL_TARJETA_RFID"));
			// NO PUEDO MANDAR, TODAVIA, LA NOTIFICACION DEL RFID.
			// DEBO SUSCRIBIRME A LA APROBACION DEL WIFI.
			xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_CONEXION_RFID, eSetBits);
		}

		if (flags & FLAG_CONTROL_TARJETA_RFID_SUPERVISOR) {
			_bln_debug_println(F("FLAG_CONTROL_TARJETA_RFID_SUPERVISOR"));
			xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_PEDIDO_SUPERVISOR, eSetBits);
		}

		// Pedir artículos (Llega por: RFID o Botón +)
		if ( flags & FLAG_CONTROL_PEDIR_ARTICULOS ) {
			_bln_debug_println(F("FLAG_CONTROL_PEDIR_ARTICULOS"));

			// Envía el número de lista a recibir
			xQueueSend(xuIntQueue, &contador_articulos, ESPERA_COLA);
			xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_PEDIR_ARTICULOS, eSetBits);
		}

		// Vuelve a la misma pantalla de antes
		if ( flags & FLAG_CONTROL_PEDIR_ARTICULOS_BACK ) {
			_bln_debug_println(F("FLAG_CONTROL_PEDIR_ARTICULOS_BACK"));

			// Envía el número de lista a recibir
			contador_articulos--;
			xQueueSend(xuIntQueue, &contador_articulos, ESPERA_COLA);
			xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_PEDIR_ARTICULOS, eSetBits);
		}

		// Recibir datos de los articulos a imprimir
		if (flags & FLAG_CONTROL_RECIBIR_ARTICULOS) {
			_bln_debug_println(F("FLAG_CONTROL_RECIBIR_ARTICULOS"));

			char *S[CANTIDAD_ARTICULOS_POR_PANTALLA];

			if ( xQueueReceive(xArticulosQueue, &S, 0) == pdTRUE ) {

				xSemaphoreTake(xMutexArticulos, portMAX_DELAY);

				for (int i = 0; i < CANTIDAD_ARTICULOS_POR_PANTALLA; i++) {
					articulos[i][0] = '\0';
					strncat(articulos[i], S[i], sizeof(articulos[i])-1);
				}

				_bln_debug_print(F("ARTICULOS >> "));
				for (int i = 0; i < CANTIDAD_ARTICULOS_POR_PANTALLA; i++) {
					_bln_debug_print(articulos[i]);
					_bln_debug_print(F(" "));
				}
				_bln_debug_println(F(""));

				xSemaphoreGive(xMutexArticulos);
			}
			if ((++contador_articulos) >= CantidadPantallasArticulos)
				contador_articulos = 0;

			xTaskNotify(xPantallaHandle, FLAG_PANTALLA_ARTICULOS, eSetBits);
		}

		// Recepción de Lista de Artículo Seleccionado
		if ( flags & FLAG_CONTROL_ARTICULO_SELECCIONADO ) {
			_bln_debug_println(F("FLAG_CONTROL_ARTICULO_SELECCIONADO"));
			xTaskNotify(xPantallaHandle, FLAG_PANTALLA_LISTA_ARTICULO, eSetBits);
		}

		// Hora.
		if ( flags & FLAG_CONTROL_HORA ) {
			_bln_debug_println(F("FLAG_CONTROL_HORA"));
			xTaskNotify(xPantallaHandle, FLAG_PANTALLA_HORA, eSetBits);
		}

		// Llamada al supervisor
		if (flags & FLAG_CONTROL_LLAMADA_SUPERVISOR) {
			_bln_debug_println(F("FLAG_CONTROL_LLAMADA_SUPERVISOR"));
			// ENVIAR A COMUNICACION AVISO PARA QUE LEVANTE LA LLAMADA AL SUPERVISOR
			xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_LLAMADA_SUPERVISOR, eSetBits);
		}

		// Aviso de recepción del supervisor
		if (flags & FLAG_CONTROL_RESPUESTA_SUPERVISOR) {
			_bln_debug_println(F("FLAG_CONTROL_RESPUESTA_SUPERVISOR"));
			xTaskNotify(xPantallaHandle, FLAG_PANTALLA_LLAMADA_SUPERVISOR, eSetBits);
		}

		if (flags & FLAG_CONTROL_LEER_RFID) {
			_bln_debug_println(F("FLAG_CONTROL_LEER_RFID"));
			xTaskNotify(xRFIDHandle, FLAG_RFID_LEER, eSetBits);
			xTaskNotify(xPantallaHandle, FLAG_PANTALLA_RFID, eSetBits);
		}

		if ( flags & FLAG_CONTROL_RESPUESTA_RFID ) {
			_bln_debug_println(F("FLAG_CONTROL_RESPUESTA_RFID"));

			bool Respuesta = false;

			if ( xQueueReceive(xBoolQueue, &Respuesta, 0) == pdTRUE ) {

				if (Respuesta) {
					contador_articulos = 0;
					xTaskNotify(xControlHandle, FLAG_CONTROL_PEDIR_ARTICULOS, eSetBits);
				}
				else {
					_bln_debug_println(F("Tarjeta RFID invalida"));
					// Vuelve a la pantalla del RFID - Espera una tarjeta RFID válida
					xTaskNotify(xPantallaHandle, FLAG_PANTALLA_INFO_ACTIVIDAD, eSetBits);
					xTaskNotify(xRFIDHandle, FLAG_RFID_LEER, eSetBits);
				}
			}
		}

		// Cambia el estado del proceso en pantalla del Supervisor
		if (flags & FLAG_CONTROL_FIN_TAREA) {
			_bln_debug_println(F("FLAG_CONTROL_FIN_TAREA"));
			xTaskNotify(xPantallaHandle, FLAG_PANTALLA_RESUMEN_ACTIVIDAD, eSetBits);
            xTaskNotify(xRFIDHandle, FLAG_RFID_LEER_SUPERVISOR, eSetBits);
		}

		if (flags & FLAG_CONTROL_ARTICULO_BORRAR) {
			_bln_debug_println(F("FLAG_CONTROL_ARTICULO_BORRAR"));
			contador_articulos = 0;
			xSemaphoreTake(xMutexTiempos, portMAX_DELAY);
			tProd = 0;
			tSetup = 0;
			tPxP = 0;
			tReproc = 0;
			NombreOperario[0] = '\0';
			strncat_P(NombreOperario, PSTR("                         "), sizeof(NombreOperario)-1);
			xSemaphoreGive(xMutexTiempos);
			CT = 0;
		}

		if (flags & FLAG_CONTROL_CONFIMACION_SUPERVISOR) {
			_bln_debug_println(F("FLAG_CONTROL_CONFIMACION_SUPERVISOR"));
			bool RespuestaSupervisor;
			if (xQueueReceive(xBoolQueue, &RespuestaSupervisor, 0) == pdTRUE) {
				if (RespuestaSupervisor) {
					// Si recibe un '1', el supervisor aprobó la planilla.
					xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_ENVIAR_INFO, eSetBits);
					xTaskNotify(xControlHandle, FLAG_CONTROL_LEER_RFID, eSetBits);
					// TODO: Activar Timer - Si no se recibe el FLAG_CONTROL_RESPUESTA_RESUMEN,
					// volver a mandar el mensaje.
				}
				else {
					// Borrar "Leyendo RFID" y poner "Tarjeta Rechazada".
					vTaskDelay(100/portTICK_PERIOD_MS);
					xTaskNotify(xPantallaHandle, FLAG_PANTALLA_INFO_ACTIVIDAD, eSetBits);
					xTaskNotify(xRFIDHandle, FLAG_RFID_LEER_SUPERVISOR, eSetBits);
				}
			}
		}
/*
		if (flags & FLAG_CONTROL_RESPUESTA_RESUMEN) {
			_bln_debug_println(F("FLAG_CONTROL_RESPUESTA_RESUMEN"));
			xTaskNotify(xControlHandle, FLAG_CONTROL_ARTICULO_BORRAR, eSetBits);
			xTaskNotify(xControlHandle, FLAG_CONTROL_LEER_RFID, eSetBits);
		}
*/
		_bln_debug_print(F(">control-task=")); _bln_debug_println(uxTaskGetStackHighWaterMark(NULL));
	}
}



