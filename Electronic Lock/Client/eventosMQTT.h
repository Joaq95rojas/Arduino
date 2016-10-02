#ifndef __EVENTOSMQTT_H__
#define __EVENTOSMQTT_H__

#define DEBUG    // DESCOMENTADO, FUNCIONA ÚNICAMENTE EN LA ARDUINO MEGA2560

#ifdef DEBUG
	#define debugSerial Serial
	#define esp8266Serial Serial3
#else
	#define esp8266Serial Serial
#endif

//////////////////////////////////////////////////////////// MQTT CONNECT
//
#define WIFI_SSID         "ANDIF-AP"                                // USUARIO (opc.)
#define WIFI_PASSWORD     "cerradura2015"                       // CONTRASEÑA (opc.)
#define MQTT_USUARIO      "andif-C1"                    // CLIENT ID
#define MQTT_PASSWORD     "d86c8a487e5a856603f69636bff5830c"
#define MQTT_SERVIDOR     "192.168.1.144"                      // IP DE LA PC
#define MQTT_PUERTO       1883
#define MQTT_SEGURO       false                                    // CLEAN SESSION
#define MQTT_DEFAULT_QOS  1                                  // LAST WILL QUALITY OF SERVICE (QOS)
// " It allows to notify other clients, when a client disconnects ungracefully"
#define MQTT_KEEPALIVE    5                                    // KEEP ALIVE
///////////////////////////////////////////////////////////////////////////

#define PIN_WIFI 15  // RX3

#ifdef DEBUG
	ESP esp = ESP(&esp8266Serial, &debugSerial, PIN_WIFI);
#else
	ESP esp = ESP(&esp8266Serial, PIN_WIFI);
#endif

MQTT mqtt = MQTT(&esp);

bool mqtt_conectado = false;

void mqttConectado(void* response) {
	char mqtt_usuario[31] = "";
	char topic_mqtt[70] = "";

	strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));

	#ifdef DEBUG
          debugSerial.println(F("MQTT_CONECTADO"));
	#endif

	snprintf_P(topic_mqtt, sizeof(topic_mqtt), PSTR("cerraduras/%s/tag-resp"), mqtt_usuario);  // MQTT TOPIC: "cerraduras/'andif-cerradura-1'/tag"
	mqtt.subscribe(topic_mqtt, 0);  // AL CONECTARSE, SE SUSCRIBE A topic_mqtt
//  mqtt.publish("cerraduras/andif-C1/tag-resp","Dato");
}

void mqttDesconectado(void* response) {
	#ifdef DEBUG
		debugSerial.println(F("MQTT_DESCONECTADO"));
	#endif
}

void mqttPublicado(void* response) {
	#ifdef DEBUG
		debugSerial.println(F("MQTT_PUBLICADO"));
	#endif
}

void mqttDatos(void* response) {
      RESPONSE res(response);

      String topic_mqtt = res.popString();
      String mensaje_mqtt = res.popString();
      
      #ifdef DEBUG
		debugSerial.print(F(">> MQTT_MENSAJE RECIBIDO: "));
                debugSerial.println(mensaje_mqtt);
      #endif
      if(mensaje_mqtt == "0"){
        #ifdef DEBUG
  		debugSerial.println(F("Abriendo Puerta.."));
         #endif      
       digitalWrite(LED_VERDE,HIGH);
       digitalWrite(LED_ROJO,LOW);
       digitalWrite(CERRADURA,LOW);
       delay(1500);                  // Mantiene la cerradura activada durante 2,5 seg
       digitalWrite(CERRADURA,HIGH);
       digitalWrite(LED_VERDE,LOW);
       digitalWrite(LED_ROJO,HIGH);
    }
    else if(mensaje_mqtt == "-1") Serial.println("ELSE IF -1");
    else if(mensaje_mqtt == "-2") Serial.println("ELSE IF -2");
    else Serial.println("ELSE");
}

void estadoWiFi(void* response) {
	uint32_t status;
	RESPONSE res(response);

	#ifdef DEBUG
	  debugSerial.println(F("ESTADO_WIFI"));
	#endif

	if ( res.getArgc() == 1 ) {
		
          res.popArgs( (uint8_t*)&status, 4 );

	  if ( status == STATION_GOT_IP ){ 
                #ifdef DEBUG
                  debugSerial.println(F("WIFI CONECTADO"));
                #endif
		if ( !mqtt_conectado )  mqtt.connect(MQTT_SERVIDOR, MQTT_PUERTO, MQTT_SEGURO);
          }
	}
}

#endif
