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

  configPin(PIN_SENSOR,ENTRADA);

}	// Fin de initPins()


void initSerial(void)
{
    Serial.begin(9600);
    Serial.println("************************************");
}	


void initLCD(void)
{
  #ifdef __LCD_SHIELD__
    lcd.begin(16, 2);
  #else
    lcd.begin(20, 4);
  #endif
    lcd.setCursor(0,1);
    lcd.print("Seleccione tarea");
}


void initSemaphore(void)
{
  vSemaphoreCreateBinary( xButtonSemaphore );
  vSemaphoreCreateBinary( xSensorSemaphore );
  vSemaphoreCreateBinary( xEmergenciaSemaphore );
}


void initQueue(void)
{
  xButtonQueue = xQueueCreate( 5 , sizeof(uint8_t) );
}


void initInterrupts(void)
{
   configPin(INT_PIN, INPUT);
   configPin(EMERGENCIA, INPUT);

   attachInterrupt(INT_NUM, vInterruptHandler, RISING);
   attachInterrupt(INT_EMERG, vEmergenciaHandler, RISING);
}


void createTasks(void)
{
  xTaskCreate( 
              vHoraTask,
              "Hora",
              STACK_HORA,
              NULL,
              PRIORIDAD_HORA,
              NULL
            );

  // Tarea que atenderá las interrupciones de vInterruptHandler
  xTaskCreate( 
              vBotonesTask,
              "TaskInt",
              STACK_BOTONES,
              NULL,
              PRIORIDAD_BOTONES,
              NULL
            );

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

}	// Fin de createTasks()


void setup( void )
{
  initPins();
  initVariables();
  initSerial();
  initLCD();
  initSemaphore();
  initQueue();
  initInterrupts();

  if( (xButtonSemaphore  != NULL)  && (xButtonQueue != NULL)\
	&& (xSensorSemaphore != NULL)  && (xEmergenciaSemaphore != NULL) )
  {
     createTasks();
     vTaskStartScheduler();  // Ejecuta el Scheduler
  } 
  else  PRINT_ERROR("*** ERROR ***");

  for( ;; );

}	// Fin de setup()


#endif