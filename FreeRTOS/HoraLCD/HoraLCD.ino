/* FreeRTOS.org includes. */
#include "FreeRTOS_AVR.h"
#include <LiquidCrystal.h>
#include <Wire.h>
#include "basic_io_avr.h"

///////////////////////////////////////////////////////////////
#include <Streaming.h>
// Versión mejorar con la librería nueva de Liquid Crystal y
// la de Streaming.

#define PIN_LED  13

/* The task functions. */
void vHoraTask( void *pvParameters );
void actualizarHora(void);
void imprimirHora(void);
void toggleLED(void);

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

uint8_t hora = 0, minutos = 0, segundos = 0;
boolean lastStateLED = false;

/*-----------------------------------------------------------*/

void setup( void )
{
  Serial.begin(9600);
  
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd << "Hora:";
  Serial.println("HORA:");

  /* Create one instance of the periodic task at priority 2. */
  xTaskCreate( vHoraTask, "Task Hora", 1024, NULL, 1, NULL );

  /* Start the scheduler so our tasks start executing. */
  vTaskStartScheduler();

  for( ;; );
//  return 0;
}
/*-----------------------------------------------------------*/

void vHoraTask( void *pvParameters )
{
TickType_t xLastWakeTime;

  /* The xLastWakeTime variable needs to be initialized with the current tick
  count.  Note that this is the only time we access this variable.  From this
  point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
  API function. */
  xLastWakeTime = xTaskGetTickCount();

  /* As per most tasks, this task is implemented in an infinite loop. */
  for( ;; )
  {
    segundos++;
    actualizarHora();
    toggleLED();
    
    noInterrupts();
    
    imprimirHora();
    
    interrupts();

    /* We want this task to execute exactly every 1 second. */
    vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );
  }
}
//------------------------------------------------------------------------------
void loop() {}

void actualizarHora (void)
{
  if( segundos >= 60 )  
  { 
      segundos = 0; 
      minutos++; 
  }
  if( minutos >= 60 )   
  { 
      minutos = 0;
      hora ++; 
  }
}

void imprimirHora(void)
{
  lcd.setCursor(0,1);
  
  if( hora < 10 ) {lcd << "0";  Serial << "0";}
  Serial << hora << ":";
  lcd    << hora << ":";
  
  if( minutos < 10 ) {lcd <<"0"; Serial << "0";}
  Serial << minutos << ":";
  lcd    << minutos << ":";
  
  if( segundos < 10 ) {lcd << "0"; Serial << "0";}
  Serial << segundos ;
  lcd    << segundos ;
}

void toggleLED(void)
{
  lastStateLED = ~lastStateLED;
  digitalWrite(PIN_LED,lastStateLED);
}
