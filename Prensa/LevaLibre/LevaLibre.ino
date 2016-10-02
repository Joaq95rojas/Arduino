#include<Streaming.h>
#include <LiquidCrystal.h>
#include<Wire.h>

#define pin  13

int cont = 0;
volatile int state = LOW;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup()
{
  pinMode(pin, OUTPUT);
  pinMode(19, INPUT);
  attachInterrupt(4, blink, RISING);
  Serial.begin(57600);
  Serial<< F("Serial OK") << endl;
  lcd.begin(16, 2);
  lcd.setCursor(0,1);
  lcd << F("Cont: ");
}

void loop()
{
  digitalWrite(pin, state);
}

void blink()
{
  state = !state;
  Serial << cont++ << endl;
  lcd.setCursor(6,1);
  lcd << cont;
}
