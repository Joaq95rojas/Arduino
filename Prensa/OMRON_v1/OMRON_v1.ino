#include <Streaming.h>
#define TIEMPO_MS  0

int pin = 13;
volatile int state = LOW;
uint16_t count = 0;

void setup()
{
  Serial.begin(57600);
  Serial << "CUENTA" << endl;
  Serial << "###############" << endl;
  Serial << endl;
  pinMode(pin, OUTPUT);
  attachInterrupt(4, blink, CHANGE);
}

void loop()
{
  digitalWrite(pin, state);
}

void blink()
{
  delay(TIEMPO_MS);
//  if(digitalRead(19)){
    state = !state;
//    if(state)  
    Serial << count++ << endl;
//  }
}
