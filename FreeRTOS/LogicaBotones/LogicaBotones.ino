/////////////////////////////////////////////////////////////////
// LogicaBotones.ino
// --------------------------------------------------------------
// * Se crea una "tarea de asignación de operac.": vTaskAssign() 
//   que:
//    - Espera a que se aprete una de las 4 tareas principales.
//    - Luego espera a que se aprete "Pausa/Inicio" para confirmar
//      la operación en el LCD.
// * vBotonesTask() le avisará a vTaskAssign() a través de una 
//   cola de mensajes.
// --------------------------------------------------------------
// La tarea de la cola de mensajes se basa en el ejemplo 10 
// del FreeRtosBook.
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
#define INT_PIN   19
#define INT_NUM    4
#define PIN_LED   13

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
#define BORRAR_LCD()  lcd.setCursor(0,0);lcd.print("                    ");\
                      lcd.setCursor(0,1);lcd.print("                    ");
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                    ");
#endif      

#define PRENDER_LED(i)  digitalWrite(i,HIGH);
#define APAGAR_LED(i)   digitalWrite(i,LOW);

/////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES

// Sincroniza vBotonesTask con vInterruptHandler
SemaphoreHandle_t xButtonSemaphore ;   // SEMÁFORO BINARIO
QueueHandle_t xButtonQueue;            // COLA DE MENSAJES

volatile uint8_t tipoTarea = 0, tipoTareaAnterior = 0;
boolean lastStateLED = false;
boolean progPausado = true; // El programa comienza pausado

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

uint8_t leerBotones(void);
static void toggleLED(void);
static void initSerial(void);
static void initLCD(void);
static void initInterrupts(void);
static void initSemaphore(void);
static void createTasks(void);
static void cambiarLEDs(uint8_t);
static void onExcLED(uint8_t);

const struct T *actualizarHora(struct T* pHora);
static void imprimirHora(const struct T* pHora);

/////////////////////////////////////////////////////////////////
// INICIALIZACIONES - SETUP()

void setup( void )
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

  initPins();
  initSerial();
  initLCD();
  initSemaphore();
  initQueue();
  initInterrupts();

  if( (xButtonSemaphore  != NULL)  && (xButtonQueue != NULL) )
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
}

static void initQueue(void)
{
  // La cola de mensajes puede contener hasta 5 paquetes del 
  // tamaño de "tareaLeida".
  xButtonQueue = xQueueCreate( 5 , sizeof(uint8_t) );
}

static void initInterrupts(void)
{
     pinMode(INT_PIN, INPUT);
     attachInterrupt(INT_NUM, vInterruptHandler, RISING);
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
              1,                           // Prioridad de la tarea
              NULL                         // pvCreatedTask
            );

  // Tarea que atenderá las interrupciones de vInterruptHandler
  xTaskCreate( 
              vBotonesTask,
              "TaskInt",
              configMINIMAL_STACK_SIZE*4,
              NULL,
              3,
              NULL
            );

  // Se encarga de seleccionar la operación correcta e imprimirla en el LCD
  // Se comunicará con vBotonesTask mediante una cola de mensajes.
    xTaskCreate( 
                vTaskAssign,
                "OpAssign",
                configMINIMAL_STACK_SIZE*4,
                NULL,
                3,
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
    Serial.print("T: ");
    Serial.println(tipoTarea);

    if( progPausado == false)
    {
      switch( tipoTarea )
      {
        case btnNETAS:        t_neto.segundos++;
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
  // Lo primero que debe hacerse es tomar el semáforo.
  xSemaphoreTake( xButtonSemaphore , 0 );

  for( ;; )
  {
    // El semáforo se utiliza para esperar por el evento.
    xSemaphoreTake( xButtonSemaphore , portMAX_DELAY );
    // Para llegar aquí, el evento debe haber ocurrido.

    uint8_t tareaLeidaAnterior = leerBotones();

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_BOTON_MS );

    uint8_t tareaLeida = leerBotones();

    if ( tareaLeidaAnterior == tareaLeida && (tareaLeida != 0) )
        // Envía la tarea leída a la cola de mensajes
//        tipoTarea = tareaLeida;
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
            cambiarLEDs(btnPAUSA);
            BORRAR_LCD();
            lcd.setCursor(0,0);
            switch( tipoTarea )
            {
              case btnNETAS:        lcd.print("Produccion");
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
//            lcd.print("EJEC");
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
}

/////////////////////////////////////////////////////////////////
// HANDLERS

static void  vInterruptHandler( void )
{
    signed portBASE_TYPE xHigherPriorityTaskWoken;
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
  noInterrupts();
  lcd.setCursor(6,1);
  if( pHora->minutos < 10 ) lcd.print("0");
  lcd.print( pHora->minutos ); lcd.print(":");
  if( pHora->segundos < 10 ) lcd.print("0");
  lcd.print( pHora->segundos );
  interrupts();
}

static void toggleLED(void)
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

// FIN DEL PROGRAMA
/////////////////////////////////////////////////////////////////