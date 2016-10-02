#ifndef __COMUNICACION_MQTT_H__
#define __COMUNICACION_MQTT_H__

// Include
//

#include <stdint.h>
#include <Arduino.h>

// Clase
//

class ESP;
class MQTT;
typedef void * TaskHandle_t;
typedef void * QueueHandle_t;
typedef void * SemaphoreHandle_t;

struct TarjetaRFID {
	uint8_t codigo[LONGITUD_TARJETA_RFID];
};

struct Lista {
	char* L[CANTIDAD_VARIANTES_ARTICULO];
	uint8_t T;
};


class ComunicacionMQTT {

public:

	enum ErrorMQTT {
		ERROR_MQTT_SIN_ERROR,
		ERROR_MQTT_ERROR,
		ERROR_MQTT_MENSAJE_INVALIDO
	};

	// Métodos públicos.
	//

	ComunicacionMQTT(ESP*, MQTT*);

	void procesar();

	void configurarYConectarWifi();

	void configurarSuscripciones();

	void marcarMQTTComoConectado() {
		mEsperandoConexionMQTT = false;
		mMQTTConectado = true;
		mUltimaConexion = millis();
	}

	void marcarMQTTComoDesconectado() {
		mEsperandoConexionMQTT = false;
		mMQTTConectado = false;
	}

	void obtenerDatosMQTT(void*);

	void leerEstadoWifi(void*);

	void enviarTiempo(const uint32_t&);

	bool mensajeServidorValido(const char*);

	void accionErrorAccion(const ErrorMQTT&);

	void publicarTagRFID(TarjetaRFID*,bool);

	void pedirArticulos(uint32_t);

	void pedirListaArticulo(char*);

	void llamarSupervisor(void);

	void EnviarInfo(void);

	ErrorMQTT accionRespuestaRFID(char*);

	ErrorMQTT accionRespuestaPedidos(char*);

	ErrorMQTT accionRespuestaListaArticulo(char*);

	ErrorMQTT accionRespuestaSupervisor(void);

	ErrorMQTT accionCantTotalArticulos(char*);

	ErrorMQTT accionConfirmacionSupervisor(char*);

	ErrorMQTT accionRespuestaResumen(char*);

public:

	// Métodos accesores/modificadores.
	//

	ESP* getEsp() const {
		return mEsp;
	}

	void setEsp(ESP* esp) {
		mEsp = esp;
	}

	MQTT* getMQTT() const {
		return mMQTT;
	}

	void setMQTT(MQTT* mqtt) {
		mMQTT = mqtt;
	}

	TaskHandle_t getControlHandle() const {
		return mControlHandle;
	}

	void setControlHandle(TaskHandle_t handle) {
		mControlHandle = handle;
	}

	TaskHandle_t getComunicacionHandle() const {
		return mComunicacionHandle;
	}

	void setComunicacionHandle(TaskHandle_t handle) {
		mComunicacionHandle = handle;
	}

	QueueHandle_t getBoolQueue() const {
		return mBoolQueue;
	}

	void setBoolQueue(QueueHandle_t cola) {
		mBoolQueue = cola;
	}

	QueueHandle_t getArticulosQueue() const {
		return mArticulosQueue;
	}

	void setArticulosQueue(QueueHandle_t cola) {
		mArticulosQueue = cola;
	}

	QueueHandle_t getListaArticuloQueue() const {
		return mListaArticuloQueue;
	}

	void setListaArticuloQueue(QueueHandle_t cola) {
		mListaArticuloQueue = cola;
	}

	QueueHandle_t getInfoQueue() const {
		return mInfoQueue;
	}

	void setInfoQueue(QueueHandle_t cola) {
		mInfoQueue = cola;
	}

	SemaphoreHandle_t getMutexOperario() const {
		return mMutexOperario;
	}

	void setMutexOperario(SemaphoreHandle_t semaforo) {
		mMutexOperario = semaforo;
	}

	bool getWifiConectado() const {
		return mWifiConectado;
	}

	void setWifiConectado(const bool& valor) {
		mWifiConectado = valor;
	}

	bool getMQTTConectado() const {
		return mMQTTConectado;
	}

	void setMQTTConectado(const bool& valor) {
		mMQTTConectado = valor;
	}

	bool getEsperandoConexionWifi() const {
		return mEsperandoConexionWifi;
	}

	void setEsperandoConexionWifi(const bool& valor) {
		mEsperandoConexionWifi = valor;
	}

	bool getEsperandoConexionMQTT() const {
		return mEsperandoConexionMQTT;
	}

	void setEsperandoConexionMQTT(const bool& valor) {
		mEsperandoConexionMQTT = valor;
	}

	uint32_t getUltimaConexion() const {
		return mUltimaConexion;
	}

	void setUltimaConexion(const uint32_t& valor) {
		mUltimaConexion = valor;
	}

	void setCallbackMQTTConectado( void (*callback)(void*) ) {
		mCallbackMQTTConectado = callback;
	}

	void setCallbackMQTTDesconectado( void (*callback)(void*) ) {
		mCallbackMQTTDesconectado = callback;
	}

	void setCallbackMQTTPublicado( void (*callback)(void*) ) {
		mCallbackMQTTPublicado = callback;
	}

	void setCallbackMQTTDatos( void (*callback)(void*) ) {
		mCallbackMQTTDatos = callback;
	}

	void setCallbackEstadoWifi( void (*callback)(void*) ) {
		mCallbackEstadoWifi = callback;
	}

private:

	// Métodos privados
	//

	void verificarConexion();

private:

	// Propiedades privadas.
	//

	ESP* mEsp;
	MQTT* mMQTT;
	TaskHandle_t mControlHandle;
	TaskHandle_t mComunicacionHandle;
	QueueHandle_t mBoolQueue;
	QueueHandle_t mArticulosQueue;
	QueueHandle_t mListaArticuloQueue;
	QueueHandle_t mInfoQueue;
	SemaphoreHandle_t mMutexOperario;
	char mTopicMQTT[70];
	char mMensajeMQTT[160];
	char mIdMensajeMQTT[20];
	char mBufferCampo[32];
	bool mWifiConectado;
	bool mMQTTConectado;
	bool mEsperandoConexionWifi;
	bool mEsperandoConexionMQTT;
	uint32_t mUltimaConexion;
	uint32_t mUltimoIntentoConexion;
	void (*mCallbackMQTTConectado)(void*);
	void (*mCallbackMQTTDesconectado)(void*);
	void (*mCallbackMQTTPublicado)(void*);
	void (*mCallbackMQTTDatos)(void*);
	void (*mCallbackEstadoWifi)(void*);

};

#endif
