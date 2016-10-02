/*
   Prueba de lectura del PLC y activación de botones externos.

   Si se acciona un pulsador (21) -> se prende un LED (13).
   Si se activa por software el pulsador (21) -> se prende el LED (13).

   El pulsador (21) se activa por software cada 5 segundos.
   El pulsador (21) se detecta por interrupción de hardware en ambos flancos.
*/

#include <avr/interrupt.h>
#include <avr/io.h>
#include <Streaming.h>

#define PULSADOR 21  // Pin 21 (SCL) -> INT2
#define LED      13
#define nINT     digitalPinToInterrupt(PULSADOR)

void setup(void)
{
  pinMode(PULSADOR, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  attachInterrupt(nINT, handler, CHANGE);
  Serial.begin(19200);
  Serial << "Serial Start" << endl;
  Serial << "############" << endl;

  TCCR2B = 0x00;
  TCNT2  = 130;
  TIFR2  = 0x00;
  TIMSK2 = 0x01;
  TCCR2A = 0x00;
  TCCR2B = 0x05;  // Enable Timer 2
}

void loop(void) { }

void handler(void)
{
  if( digitalRead(PULSADOR) ) {
      // Prendo
      Serial << "ON" << endl;
      digitalWrite(LED, HIGH);
  }
  else {
    Serial << "OFF" << endl;
    digitalWrite(LED, LOW);
  }
}

uint16_t C = 0, S = 0;

ISR(TIMER2_OVF_vect)  // 1 ms
{
  C++;
  if(C >= 1000) {  // 1 segundo
    C = 0;
    if(++S >= 5) {
      S = 0;
      Serial << "LED ON" << endl;
      digitalWrite(LED, HIGH);
    }
  }
  TCNT2 = 130;
  TIFR2 = 0x00;
}

