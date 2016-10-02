#ifndef _INICIALIZAR_H_
#define _INICIALIZAR_H_

#include "Funciones.h"

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void initPins(void)
{  

}	// Fin de initPins()


void initLCD(void)
{
  #ifdef __LCD_SHIELD__
    lcd.begin(16, 2);
  #else
    lcd.begin(20, 4);
  #endif
    lcd.setCursor(0,3);
    lcd.print("LCD OK");

} // Fin de initLCD()


void initSemaphore(void)
{
  vSemaphoreCreateBinary( xEmergenciaSemaphore );

  // Semáforo para la cola de mensajes
  xEmergenciaSemQueue = xSemaphoreCreateBinary(); 

} // Fin de initSemaphore()


void initQueue(void)
{
  xQueueSet = xQueueCreateSet( COMBINED_LENGTH);

  xComunicacionQueue = xQueueCreate( COMUNICACION_QUEUE_LENGTH , \
                                              COMUNICACION_QUEUE_SIZE );

} // Fin de initQueue()


void initInterrupts(void)
{
  configPin(EMERGENCIA, ENTRADA);

  attachInterrupt(INT_EMERG, vEmergenciaHandler, RISING);

} // Fin de initInterrupts()


void createTasks(void)
{
  // Tarea de emergencia
  xTaskCreate(
              vEmergenciaTask,
              "Emergencia",
              STACK_EMERGENCIA,
              NULL,
              PRIORIDAD_EMERGENCIA,
              NULL
            );

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
  Serial.begin(9600);
  Serial.println("INIT_SERIAL");
  initLCD();
  initSemaphore();
  initQueue();
  initInterrupts();

  // USAR configASSERT() !!!

    addToSet();
    createTasks();

    vTaskStartScheduler();  // Ejecuta el Scheduler

  for( ;; );

}	// Fin de setup()


#endif