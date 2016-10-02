#ifndef _INICIALIZAR_H_
#define _INICIALIZAR_H_

#include "Funciones.h"
#include "wifi.h"

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void initPins(void)
{
  configPin(PIN_MOTOR,SALIDA);     APAGAR_MOTOR();

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

  configPin(PIN_RESET_WIFI,SALIDA);

}	// Fin de initPins()


void initLCD(void)
{
  #ifdef __LCD_SHIELD__
    lcd.begin(16, 2);
  #else
    lcd.begin(20, 4);
  #endif

} // Fin de initLCD()


void initWifi(void)
{
//  XY(0,INFO_CURSOR); lcd << "Iniciando WiFi";
  esp8266Serial.begin(9600);

  _digWrite(PIN_RESET_WIFI, HIGH);

  while( !wifi.reiniciar() )
  {
    Serial << "No se pudo reiniciar el Wifi." << endl;
    BORRAR_LINEA_LCD(ERROR_CURSOR);
    PRINT_ERROR("Wifi ERROR");
  }

  while( !wifi.conectar() )
  {
    Serial << "No se pudo conectar a la red wifi." << endl;

    while ( !wifi.reiniciar() ) Serial << "No se pudo reiniciar." << endl;
  }

} // Fin de initWifi()


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
  //INIT_BALANCIN();

} // Fin de initInterrupts()


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
  initLCD();
  initWifi();
  initSemaphore();
  initQueue();

  initInterrupts();

  // USAR configASSERT() !!!

  if( (xButtonSemaphore  != NULL)  && (xButtonQueue != NULL)\
	&& (xSensorSemaphore != NULL)    && (xEmergenciaSemaphore != NULL)\
  && (xComunicacionQueue != NULL)  && (xEmergenciaSemQueue != NULL)\
  && (xGolpesQueue != NULL)        && (xQueueSet != NULL) )
  {
Serial.println("ASD"); Serial.flush(); delay(50);

    addToSet();
    createTasks();
    vTaskStartScheduler();  // Ejecuta el Scheduler
  }
  else  PRINT_ERROR(" *** ERROR *** ");

  for( ;; );

}	// Fin de setup()


#endif
