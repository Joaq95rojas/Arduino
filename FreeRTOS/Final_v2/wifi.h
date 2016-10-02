/** @file wifi.h
 *
 */

#ifndef __WIFI_H__
#define __WIFI_H__



// Include
//

#include <avr/pgmspace.h>
#include "include/ArduinoJson/JsonObject.hpp"


// Constantes
//

#define WIFI_BUFFER_DATOS 351
#define WIFI_TAMANO_BUFFER_CADENAS 63
#define WIFI_TAMANO_BUFFER_JSON 255
#define WIFI_INTERVALO_VERIFICAR 5000

#define debugSerial Serial
#define esp8266Serial Serial2

// Prototipos clases usadas
//

class ESP8266;
class ESP8266Client;
class ESP8266Server;
enum ESP8266CommandStatus;


// Estructuras
//


enum TiposRespuestasHTTP {
	HTTP_NINGUNO = 0,
    HTTP_RESPUESTA,
    HTTP_CONSULTA,
};


class RespuestaHTTP {
	public:
		RespuestaHTTP() {
			this->inicializar();
		}

		~RespuestaHTTP() {
		}

		void inicializar() {
			this->tipo = HTTP_NINGUNO;
			this->error = true;
			strcpy(this->datos, "");
		}

		TiposRespuestasHTTP tipo = HTTP_NINGUNO;
		bool error = true;
		char datos[WIFI_BUFFER_DATOS];
};


#define _s(a) getRamString(PSTR(a))

class Stream;

char* getRamString(PGM_P pString);
bool empiezaCon(const char *pre, const char *str);
bool leerCadenaHasta(char* cadena, int tamano, char limite, Stream* stream);

// Clases
//


/**
 *
 */
class Wifi {
	public:
		// Metódos públicos
		//

		/**
		 *
		 */
		Wifi();

		/**
		 *
		 */
		bool conectar();

		/**
		 *
		 */
		ESP8266Client* clientesDisponibles();

		/**
		 *
		 */
		bool conectarConServidor(const IPAddress& ip, const int& puerto);

		/**
		 *
		 */
		bool conectarConServidor(const char* ip, const int& puerto);

		/**
		 *
		 */
		bool desconectarServidor();

		/**
		 *
		 */
		bool enviar(const char* datos);

		/**
		 *
		 */
		bool obtenerDatosRespuestaHTTP(ESP8266Client* cliente, RespuestaHTTP* datos);

		/**
		 *
		 */
		bool reiniciar();

		/**
		 *
		 */
		bool inicializar();

		/**
		 *
		 */
		bool obtenerCadenaConsultaHTTP(char* cadena_consulta, int tamano,
										const char* metodo, const char* host,
										const char* url, const char* datos);

		/**
		 *
		 */
		bool obtenerCadenaRespuestaHTTP(char* cadena_consulta, int tamano,
										const char* codigo_http, const char* datos);

		/**
		 *
		 */
		bool enviarConsulta(const char* metodo, const char* host, const char* url, const char* datos);

		/**
		 *
		 */
		bool enviarConsultaJSON(const char* metodo, const char* host, const char* url, JsonObject& json);

		/**
		 *
		 */
		bool enviarRespuesta(ESP8266Client* cliente, const char* codigo_http, const char* datos);

		/**
		 *
		 */
		bool enviarRespuestaJSON(ESP8266Client* cliente, const char* codigo_http, JsonObject& json);

		/**
		 *
		 */
		void verificarConexion(const char* ip);

		/**
		 *
		 */
		void limpiarBuffer();
	protected:
		// Métodos privados
		//


		/**
		 *
		 */
		void inicializarPropiedades();

		/**
		 *
		 */
		bool establecerTimeout(const int& tiempo);

		/**
		 *
		 */
		bool inicializarModo();

		/**
		 *
		 */
		bool establecerModoMultiplesConexiones(const bool& modo);

		/**
		 *
		 */
		bool conectarRed();

		/**
		 *
		 */
		bool arrancarServidor();

		/**
		 *
		 */
		bool establecerTimeoutServidor(const int& tiempo);

		/**
		 *
		 */
		bool estadoOk(const bool& estado) const;

		/**
		 *
		 */
		bool estadoOk(const ESP8266CommandStatus& estado) const;

		/**
		 *
		 */
		bool tiempoCumplido(const unsigned long& tiempo_inicio, const int& duracion) const;
	public:
		// Métodos accesores
		//

		/**
		 *
		 */
		ESP8266* getESP8266() const {
			return wifi;
		}

		/**
		 *
		 */
		ESP8266Server* getESP8266Server() const {
			return server;
		}

	protected:
		// Propiedades privadas.
		//

		//
		ESP8266* wifi;
		//
		ESP8266Server* server;
};

#endif
