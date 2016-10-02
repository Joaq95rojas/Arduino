#include <LiquidCrystal.h>
#include <Streaming.h>
#include <Wire.h>

#define INT_SCL 2    // INT2: Pin 21 (SCL)

#define PIN_B   3   // Pin 3

uint16_t contPulsos = 0;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup()
{
  pinMode(PIN_B, INPUT);
  attachInterrupt(INT_SCL, handler, RISING);
  Serial.begin(19200);
  Serial<< F("Serial OK") << endl;
  lcd.begin(16, 2);
  lcd.setCursor(0,1);
  lcd << F("Cont: ");
}

void loop() {
  Serial << contPulsos << endl;
  lcd.setCursor(6,1);
  lcd << contPulsos;
}

void handler(void)
{
  delay(300);
//  if(digitalRead(21)) 
contPulsos++;
  
} // Fin de handlerCanalA()
