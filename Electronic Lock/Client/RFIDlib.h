#ifndef __RFIDLIB_H__
#define __RFIDLIB_H__

#define ESPERA_LECTURA 3000

#define LED_ROJO  11
#define LED_VERDE 10
#define CERRADURA 9

#include "eventosMQTT.h"

SoftwareSerial RFID(13, 12); // CONECTAR TX (PIN-1.1) A PIN-13 DE ARDUINO

uint8_t rfidCard[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int i = -1;

void hardwareInit(){
  pinMode(LED_ROJO,OUTPUT);
  pinMode(LED_VERDE,OUTPUT);
  pinMode(CERRADURA,OUTPUT);
  digitalWrite(CERRADURA,HIGH);
  RFID.begin(9600); 
  RFID.flush();  
  digitalWrite(LED_ROJO,HIGH);
  digitalWrite(LED_VERDE,LOW);
}

void limpiarDatosRFID(){
    RFID.flush();
    while( RFID.available() ) RFID.read();
}

///////////////////////////////////////////////////////
// LEER TARJETA RFID
//
void leerTarjetaRFID(){
  unsigned long tiempo_actual = millis();
  static unsigned long ultima_lectura = 0;
  
  if ( (tiempo_actual - ultima_lectura) < ESPERA_LECTURA ) {
      limpiarDatosRFID();
      return;
  }
  
  if (RFID.available() >= 14) 
  {
    char mqtt_usuario[31] = "";
    char topic_mqtt[70] = "";
    char tagRFID[30] = "";
    
    strcpy_P(mqtt_usuario, PSTR(MQTT_USUARIO));
    snprintf_P(topic_mqtt, sizeof(topic_mqtt), PSTR("cerraduras/%s/tag"), mqtt_usuario);
    
    for(int j=0;j<14;j++)  rfidCard[j]= RFID.read();
     
     #ifdef DEBUG
         Serial << ":: ";
         for(int j=0;j<14;j++) Serial << rfidCard[j] << " ";
         Serial << endl;
     #endif
     
     limpiarDatosRFID();
     ultima_lectura = tiempo_actual;
     
     for(int k=0;k<14;k++)  snprintf(tagRFID,30,"%s%d",tagRFID,rfidCard[k]);
     mqtt.publish(topic_mqtt,tagRFID);
  }
}

#endif
