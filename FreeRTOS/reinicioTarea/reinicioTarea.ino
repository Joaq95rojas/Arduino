/////////////////////////////////////////////////////////////////
// reinicioTarea.ino
// --------------------------------------------------------------
// * 
/////////////////////////////////////////////////////////////////
// INCLUDES

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <Streaming.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////
// DEFINES

#define PERIODO_INT_MS          ( 1000 / portTICK_PERIOD_MS )
#define PIN_LED   13

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES


/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

uint8_t segundos = 0;
TaskHandle_t pxHoraHandle;
//byte varGlobal1,varGlobal2;

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES

static void vHoraTask( void *pvParameters );
static void vTaskAssign( void *pvParameters );

static void initSerial(void);
static void createTasks(void);

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void setup( void )
{
  initSerial();
  createTasks();
  vTaskStartScheduler();  // Ejecuta el Scheduler
  for( ;; );
}


static void initSerial(void)
{
    Serial.begin(115200);
    Serial << F("************************************") << endl;
}


static void createTasks(void)
{
  // Esta tarea generará una interrupción por software periódica
  xTaskCreate( 
              vHoraTask,      
              "Hora",                  
              configMINIMAL_STACK_SIZE*4, 
              NULL,                      
              1,  
              &pxHoraHandle  
            );

    xTaskCreate( 
                vTaskAssign,
                "OpAssign",
                configMINIMAL_STACK_SIZE*4,
                NULL,
                1,
                NULL
              );  
}

/////////////////////////////////////////////////////////////////
// TAREAS

static void vHoraTask( void *pvParameters )
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();/*
#define portSAVE_CONTEXT()
  asm volatile(
        "LDS  r26, pxCurrentTCB  \n\t"
        "LDS  r27, pxCurrentTCB+1"
  );
  register byte address1 asm("r26");
  register byte address2 asm("r27");
#undef portSAVE_CONTEXT()
  Serial << _HEX(address1) << " " << _HEX(address2) << endl;
  varGlobal1 = address1;
  varGlobal2 = address2;*/
  Serial << F("Fuera del for()...") << endl;
  for( ;; )
  {   
    Serial << segundos++ << endl;
    vTaskDelayUntil( &xLastWakeTime, PERIODO_INT_MS );
  }
}


static void vTaskAssign( void *pvParameters )
{
   for ( ;; )
  {
    vTaskDelay(4000/portTICK_PERIOD_MS);
//    vTaskSuspend(pxHoraHandle);
    vTaskDelete(pxHoraHandle);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    xTaskCreate(vHoraTask,"Hora",configMINIMAL_STACK_SIZE*4,NULL,1,&pxHoraHandle);
/*#define portRESTORE_CONTEXT()
    asm volatile(
        "LDS  r26, pxCurrentTCB  \n\t"
        "LDS  r27, pxCurrentTCB+1"
  );
  register uint16_t address1 asm("r26");
  register uint16_t address2 asm("r27");
#undef portRESTORE_CONTEXT()*/
  }    
}

/////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL - LOOP()

void loop() {}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////
