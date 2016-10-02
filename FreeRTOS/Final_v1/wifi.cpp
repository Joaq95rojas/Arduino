// Include
//

#include <Arduino.h>
#include <ESP8266.h>
#include <ESP8266Client.h>
#include <ESP8266Server.h>
#include <ArduinoJson.h>

#include "wifi.h"


// Constantes
//


#define WIFI_CANTIDAD_REDES 1
#define WIFI_TIMEOUT_DEFAULT 5000
#define WIFI_DELAY_ERROR 1500
#define WIFI_DELAY_ENTRE_COMANDOS 100
#define WIFI_PUERTO_SERVIDOR 80
#define WIFI_TIMEOUT_DATOS 1000


// Variables globales
//


static const char ssid0[] PROGMEM = "AN-DIF 1 | BUFFALO";
static const char password0[] PROGMEM = "cerradura2012";

static const char* const lista_redes[WIFI_CANTIDAD_REDES][2] PROGMEM = {
	{ssid0, password0}
};


// Métodos públicos
//


/**
 *
 */
Wifi::Wifi() {
	this->inicializarPropiedades();
}


/**
 *
 */
bool Wifi::conectar() {
	if ( !this->inicializar() ) {
		return false;
	}

	delay(WIFI_DELAY_ENTRE_COMANDOS);

	if ( !this->establecerTimeout(WIFI_TIMEOUT_DEFAULT) ) {
		return false;
	}

	if ( !this->inicializarModo() ) {
		return false;

	delay(WIFI_DELAY_ENTRE_COMANDOS);
	}

	if ( !this->establecerModoMultiplesConexiones(true) ) {
		return false;
	}

	delay(WIFI_DELAY_ENTRE_COMANDOS);

	if ( !this->conectarRed() ) {
		return false;
	}

	delay(WIFI_DELAY_ENTRE_COMANDOS);

	if ( !this->arrancarServidor() ) {
		return false;
	}

	delay(WIFI_DELAY_ENTRE_COMANDOS);

	if ( !this->establecerTimeoutServidor(WIFI_TIMEOUT_DEFAULT) ) {
		return false;
	}

	delay(WIFI_DELAY_ENTRE_COMANDOS);

	this->limpiarBuffer();

	return true;
}


/**
 *
 */
ESP8266Client* Wifi::clientesDisponibles() {
	return this->server->available();
}


/**
 *
 */
bool Wifi::conectarConServidor(const IPAddress& ip, const int& puerto) {
	ESP8266CommandStatus estado_comando;
	unsigned long tiempo_actual = millis();

	do {
		debugSerial.println(F("Conectando con servidor."));
		estado_comando = this->wifi->connect(ESP8266_PROTOCOL_TCP, ip, puerto);

		if ( !this->estadoOk(estado_comando) ) {
			debugSerial.println(F("No se pudo conectar con el servidor."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado_comando) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado_comando);
}


/**
 *
 */
bool Wifi::conectarConServidor(const char* ip, const int& puerto) {
	ESP8266CommandStatus estado_comando;
	unsigned long tiempo_actual = millis();

	do {
		debugSerial.println(F("Conectando con servidor."));
		estado_comando = this->wifi->connect(ESP8266_PROTOCOL_TCP, ip, puerto);

		if ( !this->estadoOk(estado_comando) ) {
			debugSerial.println(F("No se pudo conectar con el servidor."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado_comando) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado_comando);
}


/**
 *
 */
bool Wifi::desconectarServidor() {
	ESP8266CommandStatus estado_comando;
	unsigned long tiempo_actual = millis();

	debugSerial.println(F("Desconectando del servidor."));
	estado_comando = this->wifi->close(5);

	if ( !this->estadoOk(estado_comando) || estado_comando != ESP8266_COMMAND_NO_LINK ) {
		debugSerial.println(F("No se pudo desconectar del servidor."));
		delay(WIFI_DELAY_ERROR);
	}

	return this->estadoOk(estado_comando) || estado_comando == ESP8266_COMMAND_NO_LINK;
}


/**
 *
 */
bool Wifi::enviar(const char* datos) {
	debugSerial.println(F("Enviando consulta."));
	debugSerial.println(datos);

	this->wifi->send(datos);
}


/**
 *
 */
bool Wifi::obtenerDatosRespuestaHTTP(ESP8266Client* cliente, RespuestaHTTP* datos_respuesta) {
	int longitud_datos = 0;
	int tamano_cadena = 0;
	unsigned long tiempo_comienzo = 0;
	char caracter;
	bool body_encontrado = false;
	char* puntero_longitud;
	int bytes_leidos = 0;

	datos_respuesta->inicializar();

	debugSerial.print(F("[cliente="));
	debugSerial.print(this->wifi->getId());
	debugSerial.println(F("]"));

	do {
		caracter = cliente->peek();

		if ( caracter != -1 &&
			 caracter != 'P' &&
			 caracter != 'H' &&
			 caracter != 'G' )
		{
			cliente->read();
		} else {
			break;
		}
	} while ( caracter != -1 );

	leerCadenaHasta(datos_respuesta->datos, WIFI_BUFFER_DATOS, '\n', cliente);
	/*strcpy(datos_respuesta->datos, "");
	bytes_leidos = cliente->readBytesUntil('\n', datos_respuesta->datos, WIFI_BUFFER_DATOS);
	datos_respuesta->datos[bytes_leidos] = '\0';*/

	if ( strlen(datos_respuesta->datos) <= 0 ) {
		return false;
	}

	if ( empiezaCon(_s("HTTP/1.1 "), datos_respuesta->datos) ) {
		debugSerial.println(F("[RESPUESTA]"));
		datos_respuesta->tipo = HTTP_RESPUESTA;
	}
	else if ( empiezaCon(_s("GET "), datos_respuesta->datos) ||
			  empiezaCon(_s("POST "), datos_respuesta->datos) )
	{
		debugSerial.println(F("[CONSULTA]"));
		datos_respuesta->tipo = HTTP_CONSULTA;
	} else {
		return false;
	}

	tiempo_comienzo = millis();

	while ( longitud_datos == 0 && !this->tiempoCumplido(tiempo_comienzo, WIFI_TIMEOUT_DATOS) ) {
		leerCadenaHasta(datos_respuesta->datos, WIFI_BUFFER_DATOS, '\n', cliente);
		/*strcpy(datos_respuesta->datos, "");
		bytes_leidos = cliente->readBytesUntil('\n', datos_respuesta->datos, WIFI_BUFFER_DATOS);
		datos_respuesta->datos[bytes_leidos] = '\0';*/

		if ( strlen(datos_respuesta->datos) <= 0 ) {
			break;
		}
debugSerial.print(F("cadena__="));
debugSerial.println(datos_respuesta->datos);
		if ( empiezaCon(_s("Content-Length"), datos_respuesta->datos) ) {
			puntero_longitud = datos_respuesta->datos + 16;
			longitud_datos = strtol(puntero_longitud, NULL, 10);
			break;
		}
	}

	debugSerial.print(F("longitud_datos = "));
	debugSerial.println(longitud_datos);

	if ( longitud_datos <= 0 ) {
		return false;
	}

	tiempo_comienzo = millis();

	while ( !body_encontrado && !this->tiempoCumplido(tiempo_comienzo, WIFI_TIMEOUT_DATOS) ) {
		leerCadenaHasta(datos_respuesta->datos, WIFI_BUFFER_DATOS, '\n', cliente);
		/*strcpy(datos_respuesta->datos, "");
		bytes_leidos = cliente->readBytesUntil('\n', datos_respuesta->datos, WIFI_BUFFER_DATOS);
		datos_respuesta->datos[bytes_leidos] = '\0';*/

		if ( datos_respuesta->datos[0] == 13 ) {
			debugSerial.println(F("\\r encontrado."));
			body_encontrado = true;
		}
	}

	if ( !body_encontrado ) {
		return false;
	}

	if ( longitud_datos > WIFI_BUFFER_DATOS ) {
		longitud_datos = WIFI_BUFFER_DATOS;
	}

	tiempo_comienzo = millis();

	strcpy(datos_respuesta->datos, "");
	bytes_leidos = cliente->readBytes(datos_respuesta->datos, longitud_datos);
	datos_respuesta->datos[bytes_leidos] = '\0';

	debugSerial.print(F("DATOS="));
	debugSerial.println(datos_respuesta->datos);

	if ( strlen(datos_respuesta->datos) > 0 ) {
		datos_respuesta->error = false;
	}

	return true;
}


/**
 *
 */
bool Wifi::reiniciar() {
	bool estado = false;
	unsigned long tiempo_actual = millis();

	do {
		debugSerial.println(F("Reiniciando."));
		estado = this->wifi->restart();

		if ( !this->estadoOk(estado) ) {
			debugSerial.println(F("No se pudo reiniciar."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado);
}


/**
 *
 */
bool Wifi::inicializar() {
	bool estado = false;
	unsigned long tiempo_actual = millis();

	do {
		debugSerial.println(F("Inicializando."));
		estado = this->wifi->begin();

		if ( !this->estadoOk(estado) ) {
			debugSerial.println(F("No se pudo inicializar."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado);
}


/**
 *
 */
bool Wifi::obtenerCadenaConsultaHTTP(char* cadena_consulta, int tamano, const char* metodo, const char* host, const char* url, const char* datos) {
	int tamano_datos = 0;

	strcpy(cadena_consulta, "");
	strcat(cadena_consulta, metodo);
	strcat(cadena_consulta, " ");
	strcat(cadena_consulta, url);
	strcat(cadena_consulta, _s(" HTTP/1.1\r\n"));

	strcat(cadena_consulta, _s("Server: prueba-placa-wifi/0.1\r\n"));
	strcat(cadena_consulta, _s("Content-Type: application/json\r\n"));

	strcat(cadena_consulta, _s("Host: "));
	strcat(cadena_consulta, host);
	strcat(cadena_consulta, _s("\r\n"));

	//strcat(cadena_consulta, _s("Keep-Alive: timeout=5, max=100\r\n"));
	//strcat(cadena_consulta, _s("Connection: Keep-Alive\r\n"));
	strcat(cadena_consulta, _s("Connection: close\r\n"));

	tamano_datos = strlen(datos);

	if ( tamano_datos > 0 ){
      sprintf(cadena_consulta, _s("%sContent-Length: %d\r\n"), cadena_consulta, strlen(datos));
    }

    strcat(cadena_consulta, _s("\r\n"));

    if ( tamano_datos > 0 ) {
    	strcat(cadena_consulta, datos);
    }

    return true;
}


/**
 *
 */
bool Wifi::obtenerCadenaRespuestaHTTP(char* cadena_consulta, int tamano,
									  const char* codigo_http, const char* datos)
{
	int tamano_datos = 0;

	strcpy(cadena_consulta, _s("HTTP/1.1 "));
	strcat(cadena_consulta, codigo_http);
	strcat(cadena_consulta, _s("\r\n"));

	strcat(cadena_consulta, _s("Server: prueba-placa-wifi/0.1\r\n"));
	strcat(cadena_consulta, _s("Content-Type: application/json\r\n"));

	/*strcat(cadena_consulta, _s("Keep-Alive: timeout=5, max=100\r\n"));
	strcat(cadena_consulta, _s("Connection: Keep-Alive\r\n"));*/
	strcat(cadena_consulta, _s("Connection: close\r\n"));

	tamano_datos = strlen(datos);

	if ( tamano_datos > 0 ){
      sprintf(cadena_consulta, _s("%sContent-Length: %d\r\n"), cadena_consulta, strlen(datos));
    }

    strcat(cadena_consulta, _s("\r\n"));

    if ( tamano_datos > 0 ) {
    	strcat(cadena_consulta, datos);
    }

    return true;
}


/**
 *
 */
bool Wifi::enviarConsulta(const char* metodo, const char* host, const char* url, const char* datos) {
	static char cadena_consulta[WIFI_BUFFER_DATOS] = "";

	this->obtenerCadenaConsultaHTTP(
		cadena_consulta,
		WIFI_BUFFER_DATOS,
		metodo,
		host,
		url,
		datos
	);

	debugSerial.print(F("enviado_consulta="));
	debugSerial.println(cadena_consulta);

	return this->enviar(cadena_consulta);
}


/**
 *
 */
bool Wifi::enviarConsultaJSON(const char* metodo, const char* host, const char* url, JsonObject& json) {
	static char buffer[WIFI_BUFFER_DATOS] = "";

	json.printTo(buffer, WIFI_BUFFER_DATOS);

	this->enviarConsulta(metodo, host, url, buffer);
}


/**
 *
 */
bool Wifi::enviarRespuesta(ESP8266Client* cliente, const char* codigo_http, const char* datos) {
	static char cadena_consulta[WIFI_BUFFER_DATOS] = "";

	this->obtenerCadenaRespuestaHTTP(
		cadena_consulta,
		WIFI_BUFFER_DATOS,
		codigo_http,
		datos
	);

	debugSerial.print(F("enviado_respuesta="));
	debugSerial.println(cadena_consulta);

	cliente->write(cadena_consulta);
	cliente->flush();

	return true;
}


/**
 *
 */
bool Wifi::enviarRespuestaJSON(ESP8266Client* cliente, const char* codigo_http, JsonObject& json) {
	static char buffer[WIFI_BUFFER_DATOS] = "";

	json.printTo(buffer, WIFI_BUFFER_DATOS);

	this->enviarRespuesta(cliente, codigo_http, buffer);
}


/**
 *
 */
void Wifi::verificarConexion(const char* ip) {
	ESP8266CommandStatus estado_comando;
	int id = 4;

	debugSerial.println(F("Verificando conexión..."));

	estado_comando = this->wifi->connect(
		id,
		ESP8266_PROTOCOL_TCP,
		ip,
		81
	);

	if ( estado_comando == ESP8266_COMMAND_BUSY_SEND ||
		 estado_comando == ESP8266_COMMAND_BUSY_PROCESS ||
		 estado_comando == ESP8266_COMMAND_OK ||
		 estado_comando == ESP8266_COMMAND_NO_CHANGE ||
 		 estado_comando == ESP8266_COMMAND_ALREADY_CONNECTED )
	{
		debugSerial.println(F("Conexión ok..."));

		this->wifi->close(id);
	} else {
		debugSerial.println(F("Reconectando..."));

		//this->reiniciar();
		this->conectar();
	}
}


/**
 *
 */
void Wifi::limpiarBuffer() {
	this->wifi->clear();
}


// Métodos privados
//


/**
 *
 */
void Wifi::inicializarPropiedades() {
	this->wifi = new ESP8266(esp8266Serial);
	this->server = new ESP8266Server(*this->wifi, WIFI_PUERTO_SERVIDOR);
}


/**
 *
 */
bool Wifi::establecerTimeout(const int& tiempo) {
	debugSerial.println(F("Estableciendo timeout."));
	this->wifi->setTimeout(tiempo);

	return true;
}


/**
 *
 */
bool Wifi::inicializarModo() {
	ESP8266CommandStatus estado_comando;
	unsigned long tiempo_actual = millis();

	do {
		debugSerial.println(F("Estableciendo modo."));
		estado_comando = this->wifi->setMode(ESP8266_WIFI_BOTH);

		if ( !this->estadoOk(estado_comando) ) {
			debugSerial.println(F("No se pudo establecer el modo."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado_comando) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado_comando);
}


/**
 *
 */
bool Wifi::establecerModoMultiplesConexiones(const bool& modo) {
	ESP8266CommandStatus estado_comando;
	unsigned long tiempo_actual = millis();

	do {
		debugSerial.println(F("Estableciendo multiples conexiones."));
		estado_comando = this->wifi->setMultipleConnections(modo);

		if ( !this->estadoOk(estado_comando) ) {
			debugSerial.println(F("No se pudo establecer el mode de multiples conexiones."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado_comando) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado_comando);
}


/**
 *
 */
bool Wifi::conectarRed() {
	int indice_red = 0;
	ESP8266CommandStatus estado_comando;
	char ssid[30] = "";
	char password[30] = "";
	unsigned long tiempo_actual = millis();

	do {
		strcpy_P(ssid, lista_redes[indice_red][0]);
		strcpy_P(password, lista_redes[indice_red][1]);

		debugSerial.println(F("Conectandose a la red."));
		estado_comando = this->wifi->joinAP(ssid, password);

		if ( !this->estadoOk(estado_comando) ) {
			indice_red++;

			if ( indice_red >= WIFI_CANTIDAD_REDES ) {
				indice_red = 0;
			}

			debugSerial.println(F("No se pudo conectar a la red."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado_comando) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado_comando);
}


/**
 *
 */
bool Wifi::arrancarServidor() {
	debugSerial.println(F("Arrancando servidor."));
	this->server->begin();

	return true;
}


/**
 *
 */
bool Wifi::establecerTimeoutServidor(const int& tiempo) {
	ESP8266CommandStatus estado_comando;
	unsigned long tiempo_actual = millis();

	do {
		debugSerial.println(F("Estableciendo timeout servidor."));
		estado_comando = this->wifi->setServerTimeout(10);

		if ( !this->estadoOk(estado_comando) ) {
			debugSerial.println(F("No se pudo establecer el timeout del servidor."));
			delay(WIFI_DELAY_ERROR);
		}
	} while ( !this->estadoOk(estado_comando) &&
			  !this->tiempoCumplido(tiempo_actual, WIFI_TIMEOUT_DEFAULT) );

	return this->estadoOk(estado_comando);
}


/**
 *
 */
bool Wifi::estadoOk(const bool& estado) const {
	return estado;
}


/**
 *
 */
bool Wifi::estadoOk(const ESP8266CommandStatus& estado) const {
	if ( estado == ESP8266_COMMAND_OK ||
		 estado == ESP8266_COMMAND_NO_CHANGE ||
		 estado == ESP8266_COMMAND_ALREADY_CONNECTED )
	{
		return true;
	} else {
		return false;
	}
}


/**
 *
 */
bool Wifi::tiempoCumplido(const unsigned long& tiempo_inicio, const int& duracion) const {
	if ( millis() - tiempo_inicio > duracion ) {
		return true;
	} else {
		return false;
	}
}
