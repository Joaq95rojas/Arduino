/* FreeRTOS.org includes. */
#include "FreeRTOS_AVR.h"
//#include "task.h"

/* Demo includes. */
#include "basic_io_avr.h"

#define PIN_LED  13

/* The task functions. */
void vHoraTask( void *pvParameters );
void actualizarHora(void);
void imprimirHora(void);
void toggleLED(void);

uint8_t hora = 0, minutos = 0, segundos = 0;
boolean lastStateLED = false;

/*-----------------------------------------------------------*/

void setup( void )
{
  Serial.begin(9600);

  /* Create one instance of the periodic task at priority 2. */
  xTaskCreate( vHoraTask, "Task Hora", 512, NULL, 1, NULL );

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
  if( hora < 10 ) Serial.print("0");
  Serial.print( hora ); Serial.print(":");
  if( minutos < 10 ) Serial.print("0");
  Serial.print( minutos ); Serial.print(":");
  if( segundos < 10 ) Serial.print("0");
  Serial.println( segundos );
}

void toggleLED(void)
{
  lastStateLED = ~lastStateLED;
  digitalWrite(PIN_LED,lastStateLED);
}
