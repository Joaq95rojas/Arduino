#include <espduino.h>
#include <mqtt.h>
#include "eventosMQTT.h"

#define DELAY_ACTIVACION_WIFI 500
#define DELAY_VERIFICACION_WIFI 500
#define ESPERA_ACTIVACION_WIFI 3000


void setup() {
	char mqtt_usuario[31] = "";
	bool ready = false;
	uint32_t tiempo_inicio = millis();

	#ifdef DEBUG
		debugSerial.begin(19200);
	#endif
	esp8266Serial.begin(19200);

	esp.enable();
	delay(DELAY_ACTIVACION_WIFI);
	esp.reset();
	delay(DELAY_ACTIVACION_WIFI);

	while( !ready && (millis() - tiempo_inicio <= ESPERA_ACTIVACION_WIFI) ) {
		ready = esp.ready();

		if ( !ready ) {
			delay(DELAY_VERIFICACION_WIFI);
		}
	}

 	if ( !ready ) {
 		return;
 	}

 	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));

 	if ( !mqtt.begin(mqtt_usuario, mqtt_usuario, MQTT_PASSWORD, MQTT_KEEPALIVE, 1) ) {
                #ifdef DEBUG
		  debugSerial.println(F("ARDUINO: fail to setup mqtt"));
                #endif
                while(1);
	}

        // Last Will Testament (LWT) me avisa cuando se desconecta un cliente
        // mqtt.lwt("/lwt", "offline");
        
        // Configuración de eventos MQTT
	mqtt.connectedCb.attach(mqttConectado);
	mqtt.disconnectedCb.attach(mqttDesconectado);
	mqtt.publishedCb.attach(mqttPublicado);
	mqtt.dataCb.attach(mqttDatos);
	

        // Setup WiFi
        #ifdef DEBUG
          debugSerial.println(F("ARDUINO: setting up WiFi..."));
        #endif
        esp.wifiCb.attach(estadoWiFi);
	esp.wifiConnect(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
	esp.process();
}
