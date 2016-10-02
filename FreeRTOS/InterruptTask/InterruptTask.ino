/////////////////////////////////////////////////////////////////
// InterruptTask.ino
// --------------------------------------------------------------
// * Activa un Handler cuando se pone en '1' en el pin 19 (INT 4) 
//   y activa una tarea que avisa por puerto serie.
// * Imprime la hora en el LCD
// --------------------------------------------------------------
//  Basado en el EJEMPLO 12 del FreeRTOS Book.

/////////////////////////////////////////////////////////////////
// INCLUDE

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////
// DEFINE

#define INT_PIN   19
#define INT_NUM    4
#define PIN_LED   13

/////////////////////////////////////////////////////////////////
// DECLARACIONES DE FUNCIONES

static void vHoraTask( void *pvParameters );
static void vInterruptHandler( void );
static void vTaskInterrupt(void *pvParameters);
       void actualizarHora(void);
       void imprimirHora(void);
       void toggleLED(void);
       void initSerial(void);
       void initLCD(void);
       void initInterrupts(void);
       void initSemaphore(void);
       void createTasks(void);

/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD Keypad Shield

// Sincroniza vTaskInterrupt con vInterruptHandler
SemaphoreHandle_t xBinarySemaphore;   // SEMÁFORO BINARIO

volatile uint8_t cont = 0;   // Cuenta la cantidad de veces que se ingresó
                    // a vInterruptHandler()
uint8_t hora = 0, minutos = 0, segundos = 0;
boolean lastStateLED = false;

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void setup( void )
{
  initSerial();
  initLCD();
  initSemaphore();
  initInterrupts();

  if( xBinarySemaphore != NULL )
  {
     createTasks();
     vTaskStartScheduler();  // Ejecuta el Scheduler
  }

  for( ;; );
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
  vSemaphoreCreateBinary( xBinarySemaphore );
}

void initInterrupts(void)
{
     pinMode(INT_PIN, INPUT);
     attachInterrupt(INT_NUM, vInterruptHandler, RISING);
  //  EICRB = 0x03;   // Rising Edge
  //  EIMSK = 0x10;   // INT4
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
              vTaskInterrupt,              // Puntero a la tarea. 
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
    
    noInterrupts();
    imprimirHora();
    interrupts();

    // La tarea se ejecuta exactamente cada 1 segundo.
    vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );
  }
}

static void vTaskInterrupt( void *pvParameters)
{
  // Lo primero que debe hacerse es tomar el semáforo.
  xSemaphoreTake( xBinarySemaphore, 0);

  for( ;; )
  {
    // El semáforo se utiliza para esperar por el evento.
    xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
    // Para llegar aquí, el evento debe haber ocurrido.
//    portENTER_CRITICAL();
    noInterrupts();
    lcd.setCursor(0,1);
    lcd.print(cont++);
//    portEXIT_CRITICAL();
    interrupts();
  }
}

/////////////////////////////////////////////////////////////////
// HANDLERS

static void  vInterruptHandler( void )
//ISR( INT4_vect)
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    // Se entrega el semáforo para poder desbloquear la tarea.
    xSemaphoreGiveFromISR( xBinarySemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken == pdTRUE )
    {
      // Si se activa una tarea de menor prioridad se fuerza un
      // cambio de contexto para asegurarse que vuelva a la 
      // anterior tarea.
      vPortYield();
//    taskYIELD();
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
  lcd.setCursor(6,0);
  if( hora < 10 ) lcd.print("0");
  lcd.print( hora ); lcd.print(":");
  if( minutos < 10 ) lcd.print("0");
  lcd.print( minutos ); lcd.print(":");
  if( segundos < 10 ) lcd.print("0");
  lcd.print( segundos );
}

void toggleLED(void)
{
  lastStateLED = ~lastStateLED;
  digitalWrite(PIN_LED,lastStateLED);
}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////