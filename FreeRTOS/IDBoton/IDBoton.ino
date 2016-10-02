/******************************
*   QUE TIMER USA MILLIS()????
*   Y CUAL USA EL SCHEDULER ??
*******************************/

/////////////////////////////////////////////////////////////////
// IdentificarBoton.ino
// --------------------------------------------------------------
// * Espera por una tarea en ejecución, la identifica y la muestra
//   por el LCD.
// * Simultáneamente muestra la hora.

/////////////////////////////////////////////////////////////////
// INCLUDE

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////
// DEFINE

#define PERIODO_INT_MS          ( 1000 / portTICK_PERIOD_MS )
#define TIEMPO_RUIDO_BOTON_MS   ( 250 / portTICK_PERIOD_MS )
#define INT_PIN   19
#define INT_NUM    4
#define PIN_LED   13
#define COL_LCD   16
#define FILA_LCD   2

// Botones a identificar
#define btnNETAS          53
#define btnSETUP          23
#define btnREPR           25
#define btnPARADAxPROC    27
// LEDs asociados
#define ledNETAS          32
#define ledSETUP          34
#define ledREPR           38
#define ledPARADAxPROC    36

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES

#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i);\
                            for(int j=0;j<COL_LCD;j++)  lcd.print(" ");
#define PRENDER_LED(i)  digitalWrite(i,HIGH);

/////////////////////////////////////////////////////////////////
// DECLARACIONES DE FUNCIONES

static void vHoraTask( void *pvParameters );
static void vInterruptHandler( void );
static void vBotonesTask(void *pvParameters);
       void actualizarHora(void);
       void imprimirHora(void);
       uint8_t leerBotones(void);
       void toggleLED(void);
       void initSerial(void);
       void initLCD(void);
       void initInterrupts(void);
       void initSemaphore(void);
       void createTasks(void);

/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD Keypad Shield

// Sincroniza vBotonesTask con vInterruptHandler
SemaphoreHandle_t xButtonSemaphore ;   // SEMÁFORO BINARIO

uint8_t hora = 0, minutos = 0, segundos = 0;
boolean lastStateLED = false;

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void setup( void )
{
  initPins();
  initSerial();
  initLCD();
  initSemaphore();
  initInterrupts();

  if( xButtonSemaphore  != NULL )
  {
     createTasks();
     vTaskStartScheduler();  // Ejecuta el Scheduler
  }

  for( ;; );
}

void initPins(void)
{  
    pinMode(btnNETAS,INPUT);
    pinMode(btnSETUP,INPUT);
    pinMode(btnREPR,INPUT);
    pinMode(btnPARADAxPROC,INPUT);

    pinMode(ledNETAS, OUTPUT);        PRENDER_LED(ledNETAS);
    pinMode(ledSETUP, OUTPUT);        PRENDER_LED(ledSETUP);
    pinMode(ledREPR, OUTPUT);         PRENDER_LED(ledREPR);
    pinMode(ledPARADAxPROC, OUTPUT);  PRENDER_LED(ledPARADAxPROC);
}

void initSerial(void)
{
    Serial.begin(9600);
    Serial.println("************************************");
}

void initLCD(void)
{
    lcd.begin(16, 2);
    lcd.setCursor(0,0);
    lcd.print("Hora: ");
}

void initSemaphore(void)
{
  vSemaphoreCreateBinary( xButtonSemaphore );
}

void initInterrupts(void)
{
     pinMode(INT_PIN, INPUT);
     attachInterrupt(INT_NUM, vInterruptHandler, RISING);
}

void createTasks(void)
{
  // Esta tarea generará una interrupción por software periódica
  xTaskCreate( 
              vHoraTask,                   // Puntero a la tarea. 
                                           // No debe retornar.
              "Hora",                      // Nombre descriptivo de la tarea.
              configMINIMAL_STACK_SIZE*4,  // Cant. de var. que la pila puede tener.
              NULL,                        // Puntero a parámetros.
              1,                           // Prioridad de la tarea.
              NULL                         // pvCreatedTask
            );

  // Tarea que atenderá las interrupciones de vInterruptHandler
  xTaskCreate( 
              vBotonesTask,              // Puntero a la tarea. 
                                           // No debe retornar.
              "TaskInt",                   // Nombre descriptivo de la tarea.
              configMINIMAL_STACK_SIZE*4,  // Cant. de var. que la pila puede tener.
              NULL,                        // Puntero a parámetros.
              3,                           // Prioridad de la tarea.
              NULL                         // pvCreatedTask
            );
}

/////////////////////////////////////////////////////////////////
// TAREAS

static void vHoraTask( void *pvParameters )
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    segundos++;
    actualizarHora();
//    toggleLED();
    imprimirHora(); // Deshabilita las interrupciones automaticamente 
    
    // La tarea se ejecuta exactamente cada 100 ms.
    vTaskDelayUntil( &xLastWakeTime, PERIODO_INT_MS );
  }
}

static void vBotonesTask( void *pvParameters)
{
  TickType_t xLastWakeTime_2;
  // Lo primero que debe hacerse es tomar el semáforo.
  xSemaphoreTake( xButtonSemaphore , 0 );

  for( ;; )
  {
    // El semáforo se utiliza para esperar por el evento.
    xSemaphoreTake( xButtonSemaphore , portMAX_DELAY );
    // Para llegar aquí, el evento debe haber ocurrido.

    uint8_t tareaLeidaAnterior = leerBotones();

    xLastWakeTime_2 = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime_2, TIEMPO_RUIDO_BOTON_MS );

    uint8_t tareaLeida = leerBotones();

    if ( tareaLeidaAnterior == tareaLeida && (tareaLeida != 0) )
    {
      noInterrupts();
      BORRAR_LINEA_LCD(1);
      lcd.setCursor(0,1);
      switch( tareaLeida )
      {
        case 53:  lcd.print("Produccion");
                  break;
        case 27:  lcd.print("Parada x Proceso");
                  break;
        case 23:  lcd.print("Setup");
                  break;
        case 25:  lcd.print("Reproceso");
                  break;
      }
      interrupts();
    }
  }
}

/////////////////////////////////////////////////////////////////
// HANDLERS

static void  vInterruptHandler( void )
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    // Se entrega el semáforo para poder desbloquear la tarea.
    xSemaphoreGiveFromISR( xButtonSemaphore , (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken == pdTRUE )
    {
      // Si se activa una tarea de menor prioridad se fuerza un
      // cambio de contexto para asegurarse que vuelva a la 
      // anterior tarea.
      vPortYield();
    }
}

/////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL - LOOP()

void loop() {}

/////////////////////////////////////////////////////////////////
// FUNCIONES CREADAS

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
  if( hora >= 24 )  hora = 0; // Esta línea no es necesaria
}

void imprimirHora(void)
{
  noInterrupts();
  lcd.setCursor(6,0);
  if( hora < 10 ) lcd.print("0");
  lcd.print( hora ); lcd.print(":");
  if( minutos < 10 ) lcd.print("0");
  lcd.print( minutos ); lcd.print(":");
  if( segundos < 10 ) lcd.print("0");
  lcd.print( segundos );
  interrupts();
}

void toggleLED(void)
{
  lastStateLED = ~lastStateLED;
  digitalWrite(PIN_LED,lastStateLED);
}

uint8_t leerBotones(void)
{
  for (;;)
  {
    // Una vez ocurrida la interrupción lee qué botón la generó.
    if(digitalRead(btnNETAS))       return (uint8_t)btnNETAS;
    if(digitalRead(btnPARADAxPROC)) return (uint8_t)btnPARADAxPROC;
    if(digitalRead(btnSETUP))       return (uint8_t)btnSETUP;
    if(digitalRead(btnREPR))        return (uint8_t)btnREPR;

    vTaskDelay(20);
  }
  return 0;   // Nunca debería llegar acá
}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////