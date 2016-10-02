#include "constantes.h"
#include "comunicacionmqtt.h"

#include <AVRFreeRTOS.h>
#include <utility/semphr.h>
#include <utility/task.h>
#include <espduino.h>
#include <mqtt.h>
#include <Arduino.h>

#include "util.h"


#define MQTT_CHAR_DATOS_INICIO    2
#define MQTT_CHAR_DATOS_SEPARADOR 30
#define MQTT_CHAR_DATOS_FIN       3
#define MQTT_LONGITUD_ID_MENSAJE  17
#define MQTT_DEFAULT_QOS          1
#define MQTT_KEEPALIVE            5

#define INTERVALO_MAXIMO_ULTIMA_CONEXION 1000000
#define ESPERA_ACTIVACION_WIFI           3000
#define DELAY_ACTIVACION_WIFI            500/portTICK_PERIOD_MS
#define DELAY_VERIFICACION_WIFI          500/portTICK_PERIOD_MS

static const char error0[] PROGMEM = "ERROR";
static const char error1[] PROGMEM = "MENSAJE_INVALIDO";

static const char* const lista_errores[ComunicacionMQTT::ERROR_MQTT_MENSAJE_INVALIDO+1] PROGMEM = {
	error0, error1
};


ComunicacionMQTT::ComunicacionMQTT(ESP* esp, MQTT* mqtt) {
	mEsp = esp;
	mMQTT = mqtt;

	mControlHandle = NULL;
	mComunicacionHandle = NULL;
	mArticulosQueue = NULL;
	mListaArticuloQueue = NULL;
	mInfoQueue = NULL;
	mMutexOperario = NULL;

	mTopicMQTT[0] = '\0';
	mMensajeMQTT[0] = '\0';
	mIdMensajeMQTT[0] = '\0';
	mBufferCampo[0] = '\0';
	mWifiConectado = false;
	mMQTTConectado = false;
	mEsperandoConexionWifi = false;
	mEsperandoConexionMQTT = false;
	mUltimaConexion = 0UL;

	Util::configPin(PIN_WIFI, OUTPUT);
}


void ComunicacionMQTT::procesar() {
	uint32_t tiempo_actual = millis();

	mEsp->process();

	verificarConexion();
}


void ComunicacionMQTT::configurarYConectarWifi() {
	char mqtt_usuario[31] = "";
	bool ready = false;
	uint32_t tiempo_inicio = millis();

	_bln_debug_println(F("___>activando-wifi"));

	mEsp->enable();
	vTaskDelay(DELAY_ACTIVACION_WIFI);
	mEsp->reset();
	vTaskDelay(DELAY_ACTIVACION_WIFI);

	while( !ready && (millis() - tiempo_inicio <= ESPERA_ACTIVACION_WIFI) ) {
		ready = mEsp->ready();

		if ( !ready ) {
			vTaskDelay(DELAY_VERIFICACION_WIFI);
		}
	}

 	if ( !ready ) {
 		_bln_debug_println(F("___>no-se-pudo-activar-wifi"));

 		return;
 	}

 	_bln_debug_println(F("___>wifi-activado"));

 	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));

 	if ( !mMQTT->begin(mqtt_usuario, mqtt_usuario, MQTT_PASSWORD, MQTT_KEEPALIVE, 1) ) {
		return;
	}

	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT), PSTR("placas/%s/conectado"), mqtt_usuario);
	mMQTT->lwt(mTopicMQTT, "0", 0, 1);

	mMQTT->connectedCb.attach(mCallbackMQTTConectado);
	mMQTT->disconnectedCb.attach(mCallbackMQTTDesconectado);
	mMQTT->publishedCb.attach(mCallbackMQTTPublicado);
	mMQTT->dataCb.attach(mCallbackMQTTDatos);
	mEsp->wifiCb.attach(mCallbackEstadoWifi);

	mEsp->wifiConnect(WIFI_SSID, WIFI_PASSWORD);

	mUltimaConexion = millis();

	mEsperandoConexionWifi = true;
}


void ComunicacionMQTT::configurarSuscripciones() {
	char mqtt_usuario[31] = "";

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));

	// SUSCRIPCIONES

	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT), PSTR("placas/%s/accion/+"), mqtt_usuario);
	mMQTT->subscribe(mTopicMQTT, MQTT_DEFAULT_QOS);

	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT), PSTR("placas/%s/ping"), mqtt_usuario);
	mMQTT->subscribe(mTopicMQTT, MQTT_DEFAULT_QOS);

	// PUBLICACIÓN

	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT), PSTR("placas/%s/conectado"), mqtt_usuario);
	mMQTT->publish(mTopicMQTT, "1", MQTT_DEFAULT_QOS, 1);
}


void ComunicacionMQTT::obtenerDatosMQTT(void* response) {
	RESPONSE res(response);
	char* accion;
	ComunicacionMQTT::ErrorMQTT codigo_error = ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;

	res.popString(mTopicMQTT, sizeof(mTopicMQTT));
	res.popString(mMensajeMQTT, sizeof(mMensajeMQTT));

	_bln_debug_print(F("topic="));
	_bln_debug_println(mTopicMQTT);
	_bln_debug_print(F("msg="));
	_bln_debug_println(mMensajeMQTT);

	accion = strrchr(mTopicMQTT, '/');

	if ( accion != NULL ) {
		accion++;
	}

	if ( accion == NULL ) {
		return;
	}

	mUltimaConexion = millis();

	if ( strcmp_P(accion, PSTR("ping")) == 0) {
		//
	}
	else if ( !mensajeServidorValido(mMensajeMQTT) ) {
		codigo_error = ComunicacionMQTT::ERROR_MQTT_MENSAJE_INVALIDO;
	}
	else if ( strcmp_P(accion, PSTR("conectar")) == 0) {
//		codigo_error = accionConectarControlador();
	}
	else if ( strcmp_P(accion, PSTR("asignarTarea")) == 0 ) {
//		codigo_error = accionAsignarTarea();
	}
	else if ( strcmp_P(accion, PSTR("estado")) == 0 ) {
//		codigo_error = accionActualizarEstado();
	}
	else if ( strcmp_P(accion, PSTR("respuestaRFID")) == 0 ) {
		codigo_error = accionRespuestaRFID(&mMensajeMQTT[0]);
	}
	else if (strcmp_P(accion, PSTR("cantTotalArticulos")) == 0) {
		codigo_error = accionCantTotalArticulos(&mMensajeMQTT[0]);
	}
	else if ( strcmp_P(accion, PSTR("respuestaPedidos")) == 0) {
		codigo_error = accionRespuestaPedidos(&mMensajeMQTT[0]);
	}
	else if (strcmp_P(accion, PSTR("respuestaListaArticulo")) == 0) {
		codigo_error = accionRespuestaListaArticulo(&mMensajeMQTT[0]);
	}
	else if ( strcmp_P(accion, PSTR("finalizarTarea")) == 0) {
//		codigo_error = accionFinalizarTarea();
	}
	else if ( strcmp_P(accion, PSTR("confirmarTarea")) == 0 ) {
//		codigo_error = accionConfirmarTarea();
	}
	else if (strcmp_P(accion, PSTR("respuestaSupervisor")) == 0) {
		codigo_error = accionRespuestaSupervisor();
	}
	else if (strcmp_P(accion, PSTR("confirmacionSupervisor")) == 0) {
		codigo_error = accionConfirmacionSupervisor(&mMensajeMQTT[0]);
	}
	else if (strcmp_P(accion, PSTR("respuestaResumen")) == 0) {
		codigo_error = accionRespuestaResumen(&mMensajeMQTT[0]);
	}

	if ( codigo_error != 0 ) {
		accionErrorAccion(codigo_error);
	}
}

bool ComunicacionMQTT::mensajeServidorValido(const char* datos) {
	return true;
}


void ComunicacionMQTT::accionErrorAccion(const ComunicacionMQTT::ErrorMQTT& codigo_error) {
	char mensaje_error[17] = "";

	mIdMensajeMQTT[0] = '\0';

	strncat_P(mTopicMQTT, PSTR("/respuesta"), sizeof(mTopicMQTT)-1);
	strncat(mIdMensajeMQTT, mMensajeMQTT+1, MQTT_LONGITUD_ID_MENSAJE);

	strncat_P(
		mensaje_error,
		(char*)pgm_read_word(&(lista_errores[codigo_error-1])),
		sizeof(mensaje_error)-1
	);

	snprintf_P(
		mMensajeMQTT,
		sizeof(mMensajeMQTT),
		PSTR("%c%17s%c%16s%c"),
		MQTT_CHAR_DATOS_INICIO,
		mIdMensajeMQTT,
		MQTT_CHAR_DATOS_SEPARADOR,
		mensaje_error,
		MQTT_CHAR_DATOS_FIN
	);
	_bln_debug_print(F(">ErrorAccion= ")); _bln_debug_println(mMensajeMQTT);
	mMQTT->publish(mTopicMQTT, mMensajeMQTT, MQTT_DEFAULT_QOS, 0);
}


void ComunicacionMQTT::leerEstadoWifi(void* response) {
	uint32_t status;
	RESPONSE res(response);

	if ( res.getArgc() == 1 ) {
		res.popArgs( (uint8_t*)&status, 4 );

		if ( status == STATION_GOT_IP ) {
			mUltimaConexion = millis();
			mEsperandoConexionWifi = false;
			mWifiConectado = true;
		} else {
			mWifiConectado = false;
		}
	}
}


void ComunicacionMQTT::verificarConexion() {
	if ( (mEsperandoConexionWifi || mEsperandoConexionMQTT || mWifiConectado || mMQTTConectado) &&
		 (millis() - mUltimaConexion > INTERVALO_MAXIMO_ULTIMA_CONEXION) )
	{
		if ( mWifiConectado || mMQTTConectado ) {
			xTaskNotify(mComunicacionHandle, FLAG_COMUNICACION_WIFI_DESCONECTADO, eSetBits);
		}

		mEsperandoConexionWifi = false;
		mEsperandoConexionMQTT = false;
		mWifiConectado = false;
		mMQTTConectado = false;
	}

	if ( mWifiConectado && !mEsperandoConexionMQTT && !mMQTTConectado ) {
		mEsperandoConexionMQTT = true;
		mMQTT->connect(MQTT_SERVIDOR, MQTT_PUERTO, MQTT_SEGURO);
	}
	else if ( !mEsperandoConexionWifi && !mEsperandoConexionMQTT &&
			 (!mWifiConectado || !mMQTTConectado) )
	{
		vTaskPrioritySet(mComunicacionHandle, 0);

		if ( mMQTTConectado ) {
			mMQTT->disconnect();
		}

		configurarYConectarWifi();

		vTaskPrioritySet(mComunicacionHandle, PRIORIDAD_COMUNICACION);
	}
}


void ComunicacionMQTT::enviarTiempo(const uint32_t& tiempo) {/*
	char mqtt_usuario[31] = "";

	if ( !mMQTTConectado ) {
		return;
	}

	mUltimoIntentoConexion = millis();

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));

	snprintf_P(
		mTopicMQTT, sizeof(mTopicMQTT),
		PSTR("placas/%s/tiempo"), mqtt_usuario
	);

	snprintf_P(
		mMensajeMQTT,
		sizeof(mMensajeMQTT),
		PSTR("%c%.7lu%c"),
		MQTT_CHAR_DATOS_INICIO,
		tiempo,
		MQTT_CHAR_DATOS_FIN
	);

	mMQTT->publish(mTopicMQTT, mMensajeMQTT, MQTT_DEFAULT_QOS, 0);*/
}


void ComunicacionMQTT::publicarTagRFID(TarjetaRFID* tarjeta, bool bSupervisor)
{
	char mqtt_usuario[31] = "";

	mMensajeMQTT[0] = '\0';

	if ( !mMQTTConectado ) {
		return;
	}

	mUltimoIntentoConexion = millis();

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));

	if(!bSupervisor)	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT),PSTR("placas/%s/tagRFID"), mqtt_usuario);
	else				snprintf_P(mTopicMQTT, sizeof(mTopicMQTT),PSTR("placas/%s/Pedidos/confirmacionSupervisor"), mqtt_usuario);

	for(int i=0;i<LONGITUD_TARJETA_RFID;i++)
		snprintf_P( mMensajeMQTT, sizeof(mMensajeMQTT), PSTR("%s%d"), mMensajeMQTT,(*tarjeta).codigo[i]);

	mMQTT->publish(mTopicMQTT, mMensajeMQTT, MQTT_DEFAULT_QOS, 0);
}


void ComunicacionMQTT::pedirArticulos(uint32_t P)
{
	char mqtt_usuario[31] = "";

	if ( !mMQTTConectado ) {
		return;
	}

	mUltimoIntentoConexion = millis();

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));

	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT),PSTR("placas/%s/Pedidos/Articulos"), mqtt_usuario);

	snprintf_P( mMensajeMQTT, sizeof(mMensajeMQTT), PSTR("%u"), P);
	_bln_debug_print(F("Pedir Articulos > ")); _bln_debug_println(mMensajeMQTT);

	mMQTT->publish(mTopicMQTT, mMensajeMQTT, MQTT_DEFAULT_QOS, 0);
}


void ComunicacionMQTT::pedirListaArticulo(char* S)
{
	char mqtt_usuario[31] = "";

	if ( !mMQTTConectado ) {
		return;
	}

	mUltimoIntentoConexion = millis();

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));
	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT),PSTR("placas/%s/Pedidos/Lista_Articulo"), mqtt_usuario);
	snprintf_P( mMensajeMQTT, sizeof(mMensajeMQTT), PSTR("%s"), S);

	mMQTT->publish(mTopicMQTT, mMensajeMQTT, MQTT_DEFAULT_QOS, 0);
}


void ComunicacionMQTT::llamarSupervisor(void)
{
	char mqtt_usuario[31] = "";

	_bln_debug_println(F(">> Llamando al Supervisor..."));

	if ( !mMQTTConectado ) {
		return;
	}

	mUltimoIntentoConexion = millis();

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));
	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT),PSTR("placas/%s/LlamarSupervisor"), mqtt_usuario);

	mMQTT->publish(mTopicMQTT, "1", MQTT_DEFAULT_QOS, 0);
}


void ComunicacionMQTT::EnviarInfo(void)
{
	char mqtt_usuario[31] = "";

	if ( !mMQTTConectado ) {
		return;
	}

	mUltimoIntentoConexion = millis();

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));
	snprintf_P(mTopicMQTT, sizeof(mTopicMQTT),PSTR("placas/%s/InfoTarea"), mqtt_usuario);

	struct sInfo sI;

	if( xQueueReceive(mInfoQueue,&sI,0) == pdPASS ) {
		char S[sizeof(sI)+1];
		// Pasa la estructura a char*
		snprintf_P(S, sizeof(S), PSTR("%d;%d;%d;%d;%d;%s;%d"), sI.T[0], sI.T[1], sI.T[2], sI.T[3], sI.TipoTarea, sI.Art, sI.CT);
		mMQTT->publish(mTopicMQTT, S, MQTT_DEFAULT_QOS, 0);
	}
}


ComunicacionMQTT::ErrorMQTT ComunicacionMQTT::accionRespuestaRFID(char* mMje)
{
	extern char NombreOperario[LONGITUD_NOMBRE_OPERARIO];
	bool RespuestaRFID = false;

	if( strcmp_P(mMje,PSTR("0")) ) {
		// Se detectó una persona válida
		RespuestaRFID = true;
		xTaskNotify(mControlHandle, FLAG_CONTROL_ARTICULO_BORRAR, eSetBits);
		xSemaphoreTake(mMutexOperario, portMAX_DELAY);
		snprintf_P(NombreOperario, sizeof(NombreOperario), PSTR("%s"), mMje);
		xSemaphoreGive(mMutexOperario);
	}
	xQueueOverwrite(mBoolQueue, &RespuestaRFID);
	xTaskNotify(mControlHandle, FLAG_CONTROL_RESPUESTA_RFID, eSetBits);

	return ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;
}

ComunicacionMQTT::ErrorMQTT ComunicacionMQTT::accionCantTotalArticulos(char* mMje)
{
	extern uint8_t CantidadTotalArticulos;
	extern uint8_t CantidadPantallasArticulos;

	CantidadTotalArticulos = atoi(mMje);
	CantidadPantallasArticulos = CantidadTotalArticulos / CANTIDAD_ARTICULOS_POR_PANTALLA;
	// Si la división da con resto, se necesita una pantalla más.
	if(CantidadTotalArticulos % CANTIDAD_ARTICULOS_POR_PANTALLA) CantidadPantallasArticulos++;

//	xTaskNotify(mControlHandle, FLAG_CONTROL_CANT_ARTICULOS, eSetBits);

	return ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;
}



ComunicacionMQTT::ErrorMQTT ComunicacionMQTT::accionRespuestaPedidos(char* S)
{
	char* s[CANTIDAD_ARTICULOS_POR_PANTALLA];

	s[0] = strtok(S,";");
	for(uint8_t i=1; i < CANTIDAD_ARTICULOS_POR_PANTALLA; i++) {
		s[i] = strtok(NULL,";");
	}

	xQueueOverwrite(mArticulosQueue, &s);
	xTaskNotify(mControlHandle, FLAG_CONTROL_RECIBIR_ARTICULOS, eSetBits);

	return ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;
}

ComunicacionMQTT::ErrorMQTT ComunicacionMQTT::accionRespuestaListaArticulo(char* S)
{
	Lista Lst;
	Lst.T = 1;	// Considera la asignación de Lst.L[0]

	Lst.L[0] = strtok(S,";");
	for(int i = 1; i < CANTIDAD_VARIANTES_ARTICULO; i++) {
		Lst.L[i] = strtok(NULL,";");
		if(strcmp_P(Lst.L[i],PSTR("")) != 0) Lst.T++;
	}

	_bln_debug_print(F("LISTA >> "));
	for (int i = 0; i < CANTIDAD_VARIANTES_ARTICULO; i++) {
		_bln_debug_print(Lst.L[i]);
		_bln_debug_print(F(" "));
	}
	_bln_debug_println(F(""));

	xQueueOverwrite(mListaArticuloQueue, &Lst);
	xTaskNotify(mControlHandle, FLAG_CONTROL_ARTICULO_SELECCIONADO, eSetBits);

	return ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;
}


ComunicacionMQTT::ErrorMQTT ComunicacionMQTT::accionRespuestaSupervisor(void)
{
	xTaskNotify(mControlHandle, FLAG_CONTROL_RESPUESTA_SUPERVISOR, eSetBits);
	return ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;
}

ComunicacionMQTT::ErrorMQTT ComunicacionMQTT::accionConfirmacionSupervisor(char* mMje)
{
	bool RespuestaSupervisor = false;

	if( strcmp_P(mMje,PSTR("0")) ) RespuestaSupervisor = true;

	xQueueOverwrite(mBoolQueue, &RespuestaSupervisor);
	xTaskNotify(mControlHandle, FLAG_CONTROL_CONFIMACION_SUPERVISOR, eSetBits);

	return ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;
}

ComunicacionMQTT::ErrorMQTT ComunicacionMQTT::accionRespuestaResumen(char* mMje)
{/*
	if(!strcmp_P(mMje, PSTR("1"))) {
		xTaskNotify(mControlHandle, FLAG_CONTROL_RESPUESTA_RESUMEN, eSetBits);
	}
	else {

	}
*/
	return ComunicacionMQTT::ERROR_MQTT_SIN_ERROR;
}