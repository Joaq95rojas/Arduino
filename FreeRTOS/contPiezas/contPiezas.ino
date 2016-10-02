/////////////////////////////////////////////////////////////////
// ContPiezas.ino
// --------------------------------------------------------------
// * Cuando tipoTarea = btnNETAS:
//    - Espera a que se detecte el primer golpe para comenzar a 
//      contar la hora.
//    - Se crea la tarea vgolpesBalancinTask() para detectar dichos
//      golpes del balancín.
//
/////////////////////////////////////////////////////////////////
// INCLUDES

#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <LiquidCrystal.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////
// DEFINES

#define LCD_SHIELD  true    // EN CASO DE USAR UN LCD 20x4 PONER EN "false"!

#define PERIODO_INT_MS          ( 1000 / portTICK_PERIOD_MS )
#define TIEMPO_RUIDO_BOTON_MS   ( 250 / portTICK_PERIOD_MS )
#define TIEMPO_RUIDO_SENSOR_MS  ( 50 / portTICK_PERIOD_MS )
#define INT_PIN         19
#define INT_NUM         4
#define PIN_SENSOR      18
#define PIN_SENSOR_INT  5

// Botones a identificar
#define btnNETAS          53
#define btnSETUP          23
#define btnREPR           25
#define btnPARADAxPROC    27
#define btnPAUSA          51
// LEDs asociados
#define ledNETAS          32
#define ledSETUP          34
#define ledREPR           38
#define ledPARADAxPROC    36
#define ledPAUSA          40

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES

#ifdef LCD_SHIELD
#define BORRAR_LCD()  lcd.setCursor(0,0);lcd.print("                ");\
                      lcd.setCursor(0,1);lcd.print("                ");
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                ");
#elif
#define BORRAR_LCD()  for(int j=0;j<4;j++)\
                      lcd.setCursor(0,j);lcd.print("                    ");
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                    ");
#endif      

#define PRENDER_LED(i)  digitalWrite(i,HIGH);
#define APAGAR_LED(i)   digitalWrite(i,LOW);

#define LEER_SENSOR() digitalRead(PIN_SENSOR);

#define INIT_BALANCIN() attachInterrupt(PIN_SENSOR_INT, vSensorHandler, RISING);\
                        initBalancin = true;
#define DEINIT_BALANCIN() detachInterrupt(PIN_SENSOR_INT);initBalancin = false;

/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore;
QueueHandle_t xButtonQueue;

volatile uint8_t tipoTarea = 0, tipoTareaAnterior = 0;
boolean lastStateLED = false;
boolean progPausado = true; // El programa comienza pausado
boolean initBalancin = false;
volatile uint16_t contGolpes = 0;
uint16_t golpesBalancinTotal = 20;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD Keypad Shield

struct T{
      volatile uint8_t segundos;
      volatile uint8_t minutos;
      volatile double tiempoConvertido;
} t_neto,t_setup,t_pxproc,t_repr,t_pausa,t_hora;  // Ver la posib. de hacerlas locales

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES

static void vHoraTask( void *pvParameters );
static void vInterruptHandler( void );
static void vBotonesTask( void *pvParameters );
static void vTaskAssign( void *pvParameters );
static void vgolpesBalancinTask( void *pvParameters );
static void vSensorHandler( void );

uint8_t leerBotones(void);
static void initSerial(void);
static void initLCD(void);
static void initInterrupts(void);
static void initSemaphore(void);
static void createTasks(void);
static void cambiarLEDs(uint8_t);
static void onExcLED(uint8_t);
static void mostrarPiezasLCD(void);
static void initVariables(void);

const struct T *actualizarHora(struct T* pHora);
static void imprimirHora(const struct T* pHora);

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void setup( void )
{
  initVariables();
  initPins();
  initSerial();
  initLCD();
  initSemaphore();
  initQueue();
  initInterrupts();

  if( (xButtonSemaphore  != NULL)  && (xButtonQueue != NULL) && (xSensorSemaphore != NULL))
  {
     createTasks();
     vTaskStartScheduler();  // Ejecuta el Scheduler
  } 
  else
  {
    lcd.setCursor(0,0);
    lcd.print("*** ERROR ***");
  }
  for( ;; );
}

static void initPins(void)
{  
    pinMode(btnNETAS, INPUT);
    pinMode(btnSETUP, INPUT);
    pinMode(btnREPR, INPUT);
    pinMode(btnPAUSA, INPUT);
    pinMode(btnPARADAxPROC, INPUT);

    pinMode(ledNETAS, OUTPUT);        APAGAR_LED(ledNETAS);
    pinMode(ledSETUP, OUTPUT);        APAGAR_LED(ledSETUP);
    pinMode(ledREPR, OUTPUT);         APAGAR_LED(ledREPR);
    pinMode(ledPAUSA, OUTPUT);        PRENDER_LED(ledPAUSA);
    pinMode(ledPARADAxPROC, OUTPUT);  APAGAR_LED(ledPARADAxPROC);

    pinMode(PIN_SENSOR, INPUT);
}

static void initSerial(void)
{
    Serial.begin(9600);
    Serial.println("************************************");
}

static void initLCD(void)
{
    lcd.begin(16, 2);
    lcd.setCursor(0,1);
    lcd.print("Seleccione tarea");
}

static void initSemaphore(void)
{
  vSemaphoreCreateBinary( xButtonSemaphore );
  vSemaphoreCreateBinary( xSensorSemaphore );
}

static void initQueue(void)
{
  xButtonQueue = xQueueCreate( 5 , sizeof(uint8_t) );
}

static void initInterrupts(void)
{
     pinMode(INT_PIN, INPUT);
     attachInterrupt(INT_NUM, vInterruptHandler, RISING);

//     attachInterrupt(PIN_SENSOR_INT, vSensorHandler, RISING);
}

static void createTasks(void)
{
  // Esta tarea generará una interrupción por software periódica
  xTaskCreate( 
              vHoraTask,                   // Puntero a la tarea
                                           // No debe retornar
              "Hora",                      // Nombre descriptivo de la tarea
              configMINIMAL_STACK_SIZE*4,  // Cant. de var. que la pila puede tener
              NULL,                        // Puntero a parámetros
              tskIDLE_PRIORITY+2,          // Prioridad de la tarea
              NULL                         // pvCreatedTask
            );

  // Tarea que atenderá las interrupciones de vInterruptHandler
  xTaskCreate( 
              vBotonesTask,
              "TaskInt",
              1024,
              NULL,
              tskIDLE_PRIORITY+2,
              NULL
            );

  // Se encarga de seleccionar la operación correcta e imprimirla en el LCD
  // Se comunicará con vBotonesTask mediante una cola de mensajes.
    xTaskCreate( 
                vTaskAssign,
                "OpAssign",
                2048,
                NULL,
                tskIDLE_PRIORITY+1,
                NULL
              );  

  // Tarea que cuenta los golpes del balancín
    xTaskCreate( 
            vgolpesBalancinTask,
            "Balancin",
            1024,
            NULL,
            tskIDLE_PRIORITY+2,
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
    t_hora.segundos++;
    actualizarHora(&t_hora);

    if( progPausado == false)
    {
      switch( tipoTarea )
      {
        case btnNETAS:        if((contGolpes >= 1)&&(contGolpes<golpesBalancinTotal)) 
                                  t_neto.segundos++;
                              imprimirHora(actualizarHora(&t_neto));
                              break;
        case btnSETUP:        t_setup.segundos++;
                              imprimirHora(actualizarHora(&t_setup));
                              break;
        case btnREPR:         t_repr.segundos++;
                              imprimirHora(actualizarHora(&t_setup));
                              break;
        case btnPARADAxPROC:  t_pxproc.segundos++;
                              imprimirHora(actualizarHora(&t_pxproc));
                              break;
      }
    }
    vTaskDelayUntil( &xLastWakeTime, PERIODO_INT_MS );
  }
}

static void vBotonesTask( void *pvParameters)
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;

  xSemaphoreTake( xButtonSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xButtonSemaphore , portMAX_DELAY );

    uint8_t tareaLeidaAnterior = leerBotones();

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_BOTON_MS );

    uint8_t tareaLeida = leerBotones();

    if ( tareaLeidaAnterior == tareaLeida && (tareaLeida != 0) )
        xStatus = xQueueSend(xButtonQueue,&tareaLeida,(TickType_t)0);
  }
  if( xStatus != pdPASS )
  {
    lcd.setCursor(0,0);
    lcd.print("xQueueSend ERROR");
  }
}

static void vTaskAssign( void *pvParameters )
{
  uint8_t qTipoTarea;

  if( xButtonQueue != 0 )   // Se ejecuta si la cola fue creada 
  {                         // correctamente
    for ( ;; )
    {
      if ( xQueueReceive(xButtonQueue,&qTipoTarea,portMAX_DELAY) )
      {
        if ( qTipoTarea == btnPAUSA )
        {
          if( progPausado == false )
          {
            progPausado = true;
            tipoTareaAnterior = tipoTarea;
            BORRAR_LINEA_LCD(0);
            lcd.setCursor(0,0);
            lcd.print("PAUSADO");
            lcd.setCursor(0,1);
            lcd.print("Seleccione tarea");
          }
          else
          {
            progPausado = false;
            tipoTarea = tipoTareaAnterior;
            PRENDER_LED(ledPAUSA);
            BORRAR_LCD();
            lcd.setCursor(0,0);
            switch( tipoTarea )
            {
              case btnNETAS:        lcd.print("Produccion");
                                    if( initBalancin == false ) INIT_BALANCIN();
                                    break;
              case btnSETUP:        lcd.print("Setup");
                                    break;
              case btnREPR:         lcd.print("Reproceso");
                                    break;
              case btnPARADAxPROC:  lcd.print("PxProceso");
                                    break;
             }
             lcd.setCursor(0,1);
             lcd.print("Hora: ");
          }
        }
        else if ( progPausado == true )
        {
          if( qTipoTarea != btnPAUSA ) tipoTareaAnterior = qTipoTarea;
          cambiarLEDs(qTipoTarea);
          BORRAR_LINEA_LCD(1);
          lcd.setCursor(0,1);
          lcd.print("Presione INICIO");
        }
      }
    }    
  }
} // Fin de vTaskAssign()

static void vgolpesBalancinTask( void *pvParameters )
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;

  xSemaphoreTake( xSensorSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xSensorSemaphore , portMAX_DELAY );

    uint8_t sensorLeidoAnterior = LEER_SENSOR();

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_SENSOR_MS );

    uint8_t sensorLeido = LEER_SENSOR();

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
} // Fin de vgolpesBalancinTask()

/////////////////////////////////////////////////////////////////
// HANDLERS

static void  vInterruptHandler( void )
{
    signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xButtonSemaphore , (signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();
}

static void  vSensorHandler( void )
{
    signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSensorSemaphore , (signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();
}

/////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL - LOOP()

void loop() {}

/////////////////////////////////////////////////////////////////
// FUNCIONES CREADAS

const struct T *actualizarHora (struct T* pHora)
{
  if( pHora->segundos >= 60 )  
  { 
      pHora->segundos = 0; 
      pHora->minutos++; 
  }
  return pHora;
}

static void imprimirHora(const struct T* pHora)
{
  taskENTER_CRITICAL();
  lcd.setCursor(6,1);
  if( pHora->minutos < 10 ) lcd.print("0");
  lcd.print( pHora->minutos ); lcd.print(":");
  if( pHora->segundos < 10 ) lcd.print("0");
  lcd.print( pHora->segundos );
  taskEXIT_CRITICAL();
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
    if(digitalRead(btnPAUSA))       return (uint8_t)btnPAUSA;

    vTaskDelay(20);
  }
  return 0;   // Nunca debería llegar acá
}

static void onExcLED(uint8_t tareaLED)
{
  switch( tareaLED )
  {
    case ledNETAS:        PRENDER_LED(ledNETAS);
                          APAGAR_LED(ledSETUP);APAGAR_LED(ledREPR);
                          APAGAR_LED(ledPARADAxPROC);APAGAR_LED(ledPAUSA);
                          break;
    case ledSETUP:        PRENDER_LED(ledSETUP);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledREPR);
                          APAGAR_LED(ledPARADAxPROC);APAGAR_LED(ledPAUSA);
                          break;
    case ledREPR:         PRENDER_LED(ledREPR);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledSETUP);
                          APAGAR_LED(ledPARADAxPROC);APAGAR_LED(ledPAUSA);
                          break;
    case ledPARADAxPROC:  PRENDER_LED(ledPARADAxPROC);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledSETUP);
                          APAGAR_LED(ledREPR);APAGAR_LED(ledPAUSA);
                          break;
    case ledPAUSA:        PRENDER_LED(ledPAUSA);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledSETUP);
                          APAGAR_LED(ledREPR);APAGAR_LED(ledPARADAxPROC);
                          break;
  }
}

static void cambiarLEDs(uint8_t tareaLED)
{
  switch( tareaLED )
  {
    case btnNETAS:        onExcLED(ledNETAS);
                          break;
    case btnSETUP:        onExcLED(ledSETUP);
                          break;
    case btnREPR:         onExcLED(ledREPR);
                          break;
    case btnPARADAxPROC:  onExcLED(ledPARADAxPROC);
                          break;
    case btnPAUSA:        onExcLED(ledPAUSA);
                          break;
  }
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

static void initVariables( void )
{
  t_neto.segundos = 0;
  t_neto.minutos = 0;
  t_setup.segundos = 0;
  t_setup.minutos = 0;
  t_pxproc.segundos = 0;
  t_pxproc.minutos = 0;
  t_repr.segundos = 0;
  t_repr.minutos = 0;
  t_pausa.segundos = 0;
  t_pausa.minutos = 0;
  t_hora.segundos = 0;
  t_hora.minutos = 0;
}

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////