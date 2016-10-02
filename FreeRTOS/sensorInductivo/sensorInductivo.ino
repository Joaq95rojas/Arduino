/////////////////////////////////////////////////////////////////
// SensorInductivo.ino
// --------------------------------------------------------------
// * Similar al programa InterruptTask.ino, solo que aplicado a 
//   contar las piezas del balancin.
//
/////////////////////////////////////////////////////////////////
// INCLUDE

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////
// DEFINE

#define PIN_SENSOR        18
#define PIN_SENSOR_INT    5
#define PIN_LED           13

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES

#define DEINIT_BALANCIN() detachInterrupt(PIN_SENSOR_INT);initBalancin = false;
#define LEER_SENSOR() digitalRead(PIN_SENSOR);
#define TIEMPO_RUIDO_SENSOR_MS  ( 50 / portTICK_PERIOD_MS )
#define BORRAR_LCD()  lcd.setCursor(0,0);lcd.print("                ");\
                      lcd.setCursor(0,1);lcd.print("                ");
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                ");

/////////////////////////////////////////////////////////////////
// DECLARACIONES DE FUNCIONES

static void vHoraTask( void *pvParameters );
static void vSensorHandler( void );
static void vIntHandler( void );
static void vTaskAssign( void *pvParameters );
static void vgolpesBalancinTask(void *pvParameters);
static void vTareaEjemplo( void *pvParameters );
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

SemaphoreHandle_t xSensorSemaphore,xEjemploSemaphore;   // SEMÁFORO BINARIO

volatile uint8_t tipoTarea = 0, tipoTareaAnterior = 0;
uint8_t minutos = 0, segundos = 0;
boolean lastStateLED = false;
volatile uint16_t contGolpes = 0;
uint16_t golpesBalancinTotal = 100;
boolean progPausado = true; // El programa comienza pausado
boolean initBalancin = false;

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void setup( void )
{
  pinMode(PIN_SENSOR, INPUT);
  initSerial();
  initLCD();
  initSemaphore();
  initInterrupts();

  if( xSensorSemaphore != NULL )
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
    lcd.setCursor(0,1);
    lcd.print("Hora: ");
}

void initSemaphore(void)
{
  vSemaphoreCreateBinary( xSensorSemaphore );
  vSemaphoreCreateBinary( xEjemploSemaphore );
}

void initInterrupts(void)
{
    attachInterrupt(PIN_SENSOR_INT, vSensorHandler, RISING);\
    initBalancin = true;
    attachInterrupt(4,vIntHandler,RISING);
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
              tskIDLE_PRIORITY+2,    //1                       // Prioridad de la tarea.
              NULL                         // pvCreatedTask
            );

  // Tarea que atenderá las interrupciones de vSensorHandler
  xTaskCreate( 
              vgolpesBalancinTask,           
                                           
              "Balancin",                  
              configMINIMAL_STACK_SIZE*2,  
              NULL,                       
              tskIDLE_PRIORITY+2,       //2                  
              NULL                    
            );

    xTaskCreate( 
              vTareaEjemplo,           
                                           
              "Ejemplo",                  
              configMINIMAL_STACK_SIZE*2,  
              NULL,                       
              tskIDLE_PRIORITY+2,       //3                  
              NULL                    
            );

        xTaskCreate( 
                vTaskAssign,
                "OpAssign",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY+1,      //2
                NULL
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
    
    imprimirHora();

    vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );
  }
}

static void vgolpesBalancinTask( void *pvParameters)
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;

  xSemaphoreTake( xSensorSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xSensorSemaphore , portMAX_DELAY );

    uint8_t sensorLeidoAnterior = LEER_SENSOR();
    Serial.print("LEE SENSOR 1: "); Serial.println(sensorLeidoAnterior);

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_SENSOR_MS );

    uint8_t sensorLeido = LEER_SENSOR();
    Serial.print("LEE SENSOR 2: "); Serial.println(sensorLeido);

    if ( (sensorLeidoAnterior == HIGH) && (sensorLeido == HIGH) )
      if( contGolpes < golpesBalancinTotal )
      {
        contGolpes++;
        mostrarPiezasLCD();
      }
      if( contGolpes >= golpesBalancinTotal )
      {
        BORRAR_LINEA_LCD(0);
        lcd.setCursor(0,0);
        lcd.print("Tarea finalizada");
        progPausado = true;
        tipoTarea = 0;
        DEINIT_BALANCIN();
      }
  }
  if( xStatus != pdPASS )
  {
    lcd.setCursor(0,0);
    lcd.print("xQueueSend ERROR");
  }
}

static void vTareaEjemplo( void *pvParameters )
{
  TickType_t xLastWakeTime;

  xSemaphoreTake( xEjemploSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xEjemploSemaphore , portMAX_DELAY );

  lcd.setCursor(10,1);
  lcd.print("*");
  Serial.println("X");
  }
}

static void vTaskAssign( void *pvParameters )
{
    for(;;);
} // Fin de vTaskAssign()

/////////////////////////////////////////////////////////////////
// HANDLERS

static void  vSensorHandler( void )
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( xSensorSemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken == pdTRUE )
        vPortYield();
}

static void  vIntHandler( void )
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( xEjemploSemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken == pdTRUE )
        vPortYield();
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
}

void imprimirHora(void)
{
  taskENTER_CRITICAL();
  lcd.setCursor(6,1);
  if( minutos < 10 ) lcd.print("0");
  lcd.print( minutos ); lcd.print(":");
  if( segundos < 10 ) lcd.print("0");
  lcd.print( segundos );
  taskEXIT_CRITICAL();
}

void toggleLED(void)
{
  lastStateLED = ~lastStateLED;
  digitalWrite(PIN_LED,lastStateLED);
}

static void mostrarPiezasLCD( void )
{
  BORRAR_LINEA_LCD(0);
  lcd.setCursor(0,0);
  lcd.print("Piezas: ");
  lcd.print(contGolpes);
  lcd.print(" / ");
  lcd.print(golpesBalancinTotal);
}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////