#include <avr/io.h>

#define SUP  250
#define INF  150

void setup() 
{
  // Set OC3 as output (pin PE5)
  DDRE |= 1<<PE5;
  
  
  // Timer Counter Control Register (TCCR3 - Timer 3)
  // Digital Pin 3 = OC3C
  // PWM output: Non inverted
  // Mode: Fast PWM
  // Timer clock: f_CLK/1024
  // 
//  TCCR3C|=(1<<WGM30)|(1<<WGM31)|(1<<COM3C1)|(1<<CS32)|(1<<CS31)|(1<<CS30);
  TCCR3A |= (1<<WGM30)|(1<<WGM31)|(1<<COM3C1);
  TCCR3B |= (1<<CS30);
}

void loop() 
{
  for(uint8_t i = INF; i<SUP; i++)
  {
    OCR3C = i;
    delayMicroseconds(2000);
  }
//  delay(100);
  for(uint8_t i = SUP; i>INF; i--)
  {
    OCR3C = i;
    delayMicroseconds(2000);
  }
//  delay(100);
}
