#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

#include "Constantes.h"

extern int g_count, g_array[3], g_flag;
extern float g_SF[3];
extern bool initColor;
extern int color, count;

extern SemaphoreHandle_t xColorSemaphore;

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES
//
void configPin( uint8_t, uint8_t );
void _digWrite( uint8_t, uint8_t );

void TSC_FilterColor(int, int);

void initTSC(void);
void initSemaphore(void);
void createTasks( void );


//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES MACRO
//
#define INIT_SERIAL()  Serial.begin(9600);  Serial << "* INICIADO *" << endl;

//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES PROPIAS
//

void configPin( uint8_t pNum, uint8_t modo )
{
  volatile uint8_t *reg, *out;

  reg = REG_PIN(pNum);
  out = OUT_PIN(pNum);
  uint8_t oldSREG = SREG;
  cli();
  if ( modo ) *reg |= BIT_PIN(pNum);  // Salida
  else
  {
    *reg &= ~BIT_PIN(pNum);   // Entrada
    *out &= ~BIT_PIN(pNum);
  }
  SREG = oldSREG;

}  // Fin de configPin()


void _digWrite( uint8_t pNum, uint8_t statePin )
{
  volatile uint8_t* out = OUT_PIN(pNum);

  if ( statePin )  *out |= BIT_PIN(pNum);
  else            *out &= ~BIT_PIN(pNum);

}  // Fin de _digWrite()

///////////////////////////////////////////////////////////////////////////////////

void TSC_FilterColor(int Level01, int Level02)
{
  // Select the filter color
  if (Level01 != 0)  Level01 = HIGH;
  if (Level02 != 0)  Level02 = HIGH;

  _digWrite(S2, Level01);
  _digWrite(S3, Level02);

}  // Fin de TSC_FilterColor()

//////////////////////////////////////////////////////////////////////////////////
// TAREAS
//

void vColorTask( void *pvParameters )
{
  int R,G,B;

  for( ;; )
  {
    while( initColor == false )
    {
      g_flag = 0;
  
      R = int(g_array[0] * g_SF[0]);
      G = int(g_array[1] * g_SF[1]);
      B = int(g_array[2] * g_SF[2]);
  
      if( (R > 2*B) && (G>1.5*B) )  // Color AMARILLO
      {
         color = AMARILLO;
         Serial << "AMARILLO" << endl;
      }
      else if( (R>200) && (G>200) && (B>200) ) {  color = BLANCO;Serial << "BLANCO" << endl; }
      else if( (R>2*G) && (R>2*B) ){  color = ROJO;Serial << "ROJO" << endl; }
      else if( (B>R) && (B>G) )    {  color = AZUL;Serial << "AZUL" << endl; }
      else Serial << "COLOR No identificado." << endl;

     vTaskDelay( DATOS_FINAL_MS );
    }
    vPortYield();
  }

}  // Fin de vColorTask()


void vTaskCallback(void *pvParameters)
{
  for ( ;; )
  {
    switch (g_flag)
    {
      case 0:  
        g_count = 0;
        g_flag ++;
        TSC_FilterColor(LOW, LOW);
        break;
      case 1:  //Serial << "g_array[0]= " << g_count << endl;
        g_array[0] = g_count;
        g_count = 0;
        g_flag ++;
        TSC_FilterColor(HIGH, HIGH);
        break;
      case 2:  //Serial << "g_array[1]= " << g_count << endl;
        g_array[1] = g_count;
        g_count = 0;
        g_flag ++;
        TSC_FilterColor(LOW, HIGH);
        break;
      case 3: //Serial << "g_array[2]= " << g_count << endl;
        g_array[2] = g_count;
        g_count = 0;
        g_flag ++;
        TSC_FilterColor(HIGH, LOW);
        break;
      default:  g_count = 0;
        break;
    }
    vTaskDelay( PROCESAMIENTO_MS );
  }

}  // Fin de TSC_Callback()


void vTaskCount(void *pvParameters)
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;

  xSemaphoreTake( xColorSemaphore , 0 );

  for ( ;; )
  {
    xSemaphoreTake( xColorSemaphore , portMAX_DELAY );
    g_count ++;
  }

}  // Fin de vTaskCount()


void vInitColor ( void *pvParameters )
{
  for( ;; )
  {
    if ( initColor == true )
    {
      vTaskDelay(INIT_COLOR_MS);
  
      g_SF[0] = 255.0 / g_array[0];    //R Scale factor
      g_SF[1] = 255.0 / g_array[1];   //G Scale factor
      g_SF[2] = 255.0 / g_array[2];   //B Scale factor
  
      Serial << "SF[0]= " << g_SF[0] << endl;
      Serial << "SF[1]= " << g_SF[1] << endl;
      Serial << "SF[2]= " << g_SF[2] << endl;
      initColor = false;
  //    taskEXIT_CRITICAL();
    }
    vPortYield();
  }
}

/////////////////////////////////////////////////////////////////
// HANDLERS

void vCountHandler( void )
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR( xColorSemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

  if ( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();

}  // Fin de vCountHandler()

#endif
