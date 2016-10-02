#ifndef _INICIALIZAR_H_
#define _INICIALIZAR_H_

#include "Funciones.h"

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void initPins(void)
{  
  configPin(PIN_MOTOR,SALIDA);   APAGAR_MOTOR();
  configPin(btnNETAS,ENTRADA);
  configPin(btnSETUP,ENTRADA);
  configPin(btnREPR,ENTRADA);
  configPin(btnPAUSA,ENTRADA);
  configPin(btnPARADAxPROC,ENTRADA);

  configPin(ledNETAS,SALIDA);        APAGAR_LED(ledNETAS);
  configPin(ledSETUP,SALIDA);        APAGAR_LED(ledSETUP);
  configPin(ledREPR,SALIDA);         APAGAR_LED(ledREPR);
  configPin(ledPAUSA,SALIDA);        PRENDER_LED(ledPAUSA);
  configPin(ledPARADAxPROC,SALIDA);  APAGAR_LED(ledPARADAxPROC);

}	// Fin de initPins()


void initLCD(void)
{
  #ifdef __LCD_SHIELD__
    lcd.begin(16, 2);
  #else
    lcd.begin(20, 4);
  #endif
//    lcd.print(getMsgString(1)); // "Seleccione tarea"

} // Fin de initLCD()


void initSemaphore(void)
{
  vSemaphoreCreateBinary( xButtonSemaphore );
  vSemaphoreCreateBinary( xSensorSemaphore );
  vSemaphoreCreateBinary( xEmergenciaSemaphore );

  // Semáforo para la cola de mensajes
  xEmergenciaSemQueue = xSemaphoreCreateBinary(); 

} // Fin de initSemaphore()


void initQueue(void)
{
  xQueueSet = xQueueCreateSet( COMBINED_LENGTH);

  xButtonQueue = xQueueCreate( BUTTON_QUEUE_LENGTH, BUTTON_QUEUE_SIZE );
  xComunicacionQueue = xQueueCreate( COMUNICACION_QUEUE_LENGTH, \
                                              COMUNICACION_QUEUE_SIZE );
  xGolpesQueue = xQueueCreate( GOLPES_QUEUE_LENGTH, GOLPES_QUEUE_SIZE );
  xTiemposQueue = xQueueCreate( TIEMPOS_QUEUE_LENGTH, TIEMPOS_QUEUE_SIZE );

} // Fin de initQueue()


void initInterrupts(void)
{
  configPin(INT_PIN, ENTRADA);
  configPin(PIN_SENSOR,ENTRADA);
  configPin(EMERGENCIA, ENTRADA);

  attachInterrupt(INT_EMERG, vEmergenciaHandler, RISING);

} // Fin de initInterrupts()


void createTasks(void)
{/*
  xTaskCreate( 
              vHoraTask,
              "Hora",
              STACK_HORA,
              NULL,
              PRIORIDAD_HORA,
              NULL
            );
*/
  // Tarea que atenderá las interrupciones de vInterruptHandler
  xTaskCreate( 
              vBotonesTask,
              "TaskInt",
              STACK_BOTONES,
              NULL,
              PRIORIDAD_BOTONES,
              NULL
            );
/*
  // Se encarga de seleccionar la operación correcta e imprimirla en el LCD
  // Se comunicará con vBotonesTask mediante una cola de mensajes.
  xTaskCreate( 
              vTaskAssign,
              "OpAssign",
              STACK_ASSIGN,
              NULL,
              PRIORIDAD_ASSIGN,
              NULL
            );  

  // Tarea que cuenta los golpes del balancín
  xTaskCreate( 
              vgolpesBalancinTask,
              "Balancin",
              STACK_BALANCIN,
              NULL,
              PRIORIDAD_BALANCIN,
              NULL
            );

  // Tarea de emergencia
  xTaskCreate(
              vEmergenciaTask,
              "Emergencia",
              STACK_EMERGENCIA,
              NULL,
              PRIORIDAD_EMERGENCIA,
              NULL
            );
*/
  // Tarea de control general (global) de las demás tareas
  xTaskCreate(
              vControlGeneralTask,
              "CtrlGral",
              STACK_CTRLGRAL,
              NULL,
              PRIORIDAD_CTRLGRAL,
              NULL
            );

  // Tarea de comunicación con el servidor
  xTaskCreate(
              vComunicacionTask,
              "Comunicacion",
              STACK_COMUNICACION,
              NULL,
              PRIORIDAD_COMUNICACION,
              NULL
            );  

}	// Fin de createTasks()


void setup( void )
{
  initPins();
  INIT_SERIAL();
//  Serial.begin(9600);
//  Serial.println("INIT_SERIAL");
  initLCD();
  initSemaphore();
  initQueue();
  initInterrupts();

  // USAR configASSERT() !!!

  if( (xButtonSemaphore  != NULL)  && (xButtonQueue != NULL)\
	&& (xSensorSemaphore != NULL)    && (xEmergenciaSemaphore != NULL)\
  && (xComunicacionQueue != NULL)  && (xEmergenciaSemQueue != NULL)\
  && (xGolpesQueue != NULL)        && (xQueueSet != NULL) )
  {
    addToSet();
    createTasks();
    vTaskStartScheduler();  // Ejecuta el Scheduler
  } 
  else  PRINT_ERROR(" *** ERROR *** ");
  //PRINT_ERROR(getMsgString(2)); // " *** ERROR ***  "

  for( ;; );

}	// Fin de setup()


#endif