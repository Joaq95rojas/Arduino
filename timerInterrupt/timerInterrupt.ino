/*
 * CONFIGURACIÓN DEL TIMER USANDO REGISTROS
 * 
 * TCNT2: Valor de interrupción (130 -> 1ms)
 * TCCR2B: 0x00=Apagado. 0x05=Encendido.
 * 
 * https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/
 */

#include <avr/interrupt.h>  
#include <avr/io.h>
#include <Streaming.h>

unsigned int toggle = 0;  //used to keep the state of the LED
unsigned int count = 0;   //used to keep count of how many interrupts were fired
unsigned int segundos = 0;

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) 
{
  count++;               //Increments the interrupt counter
  if(count >= 1000)
  {
    toggle = ~toggle;    //toggles the LED state
    count = 0;           //Resets the interrupt counter
    Serial << ++segundos << endl;
    if(segundos == 5) TCCR2B = 0x00;
    digitalWrite(13,toggle);
  }
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};  

void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(19200);

  //Setup Timer2 to fire every 1ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
  TCCR2B = 0x00;        //Timer2 Control Reg B: Timer Prescaler set to 128
}

void loop() 
{
    delay(3000);
    TCCR2B = 0x05;
    delay(5000);
    TCCR2B = 0x00;
}
