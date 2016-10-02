#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

#include "Constantes.h"

extern int g_count, g_array[3], g_flag;
extern float g_SF[3];
extern bool enCamino, initColor, inicio, meta, movPermitido;
extern int velRobot, color, count;

extern SemaphoreHandle_t xColorSemaphore;
extern TaskHandle_t xInitColorHandle, xColorHandle, xHCSR04Handle, xCallbackHandle;
extern TaskHandle_t xColorCountHandle;

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES
//
void configPin( uint8_t, uint8_t );
void _digWrite( uint8_t, uint8_t );

void cambiarVelocidad(int);
long lecturaHCSR04(void);

void initAll(void);
void initTSC(void);
void xInitRTOS(void);
void createTasks( void );

//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES MACRO Y "EN LINEA"
//
#define  APAGAR(x,y)  x &= ~(1 << y)
#define PRENDER(x,y)  x |= (1 << y)
#define ENTRADA(x,y)  x &= ~(1 << y)
#define SALIDA(x,y)   x |= (1 << y)

#define sbi(x,y)  x |= 1 << y

//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES PROPIAS
//

void cambiarVelocidad( void )
{
  if( velRobot >= 255 ) velRobot = 255;
  if( velRobot <= 0 )   velRobot = 0;  
  
  APAGAR(PORTH,3);    APAGAR(PORTH,6);
  APAGAR(PORTB,5);    APAGAR(PORTB,6);
  
  if( velRobot == 255 )
  {
    PRENDER(PORTH,4);    PRENDER(PORTH,5);
    PRENDER(PORTB,4);    PRENDER(PORTB,7);
  }
  else if( !velRobot )
  {
    APAGAR(PORTH,4);    APAGAR(PORTH,5);
    APAGAR(PORTB,4);    APAGAR(PORTB,7);
  }
  else
  {
    // analogWrite( 7, velMotor );  // Motor D2 (PD 7)
    sbi(TCCR4A, COM4B1);
    OCR4B = velRobot;
    // analogWrite( 8, velMotor );
    sbi(TCCR4A, COM4C1);
    OCR4C = velRobot;
    // analogWrite( 10, velMotor );    // Motor T1 (PD 10)
    sbi(TCCR2A, COM2A1);
    OCR2A = velRobot;
    // analogWrite( 13, velMotor );
    sbi(TCCR0A, COM0A1);
    OCR0A = velRobot;
  }
  Serial << velRobot << endl;
  
}  // Fin de cambiarVelocidad()


long lecturaHCSR04(void)
{
  long tiempoEco, distMedida;

  APAGAR(PORTE,3);        // Trigger -> LOW
  delayMicroseconds(2);
  PRENDER(PORTE,3);       // Trigger -> HIGH
  delayMicroseconds(10);
  APAGAR(PORTE,3);

  tiempoEco = pulseIn(PIN_ECHO, HIGH);
  distMedida = tiempoEco / 58.2;

  return distMedida;

} // Fin de lecturaHCSR04()


//////////////////////////////////////////////////////////////////////////////////
// TAREAS
//

void vColorTask( void *pvParameters )
{
  int R,G,B;
  
  vTaskSuspend(xInitColorHandle);
  
  for( ;; )
  {
   g_flag = 0;
  
    R = int(g_array[0] * g_SF[0]);
    G = int(g_array[1] * g_SF[1]);
    B = int(g_array[2] * g_SF[2]);
    
//    Serial << R << F(" : ") << G << F(" : ") << B << endl;

    if(movPermitido)
    {
      if( COND_AMARILLO )
      {
        enCamino = true;
        color = AMARILLO;
        Serial << F("Amarillo") << endl;
        velRobot += 40;
      }
      else if( COND_BLANCO )  // Color BLANCO
      {
        if( inicio )
        {
          if( enCamino && !meta )
          {
            Serial << F("META") << endl;
            meta = true;
            vTaskSuspend(xHCSR04Handle);
           }
           color = BLANCO;
           velRobot -= 80;
        }  
        else
        { 
          Serial << F("INICIO") << endl; 
          inicio = true;
          vTaskResume(xHCSR04Handle);
        }
      }
      else if( COND_ROJO )
      {
        color = ROJO;
        Serial << F("Color Rojo") << endl;
        velRobot -= 130;
      }  
      else if( COND_AZUL )
      {
        color = AZUL;
        Serial << F("Color Azul") << endl;
        velRobot -= 40;
      }   
      else Serial << F("Color no detectado.") << endl;
    }
    else  velRobot = 0;
    
    cambiarVelocidad();  
   
    vTaskDelay( DATOS_FINAL_MS );
  }
  
}  // Fin de vColorTask()


void vCallbackTask(void *pvParameters)
{
  for( ;; )
  {
//    Serial << g_count << endl;
    switch (g_flag)
    {
      case 0:  
        g_count = 0;
        g_flag ++;
        APAGAR(PORTL,PL0);  // TSC_FilterColor(LOW, LOW);
        APAGAR(PORTB,PB2);
        break;
      case 1: 
        g_array[0] = g_count;
        g_count = 0;
        g_flag ++;
        PRENDER(PORTL,PL0);  // TSC_FilterColor(HIGH, HIGH);
        PRENDER(PORTB,PB2);
        break;
      case 2: 
        g_array[1] = g_count;
        g_count = 0;
        g_flag ++;
        APAGAR(PORTL,PL0);  // TSC_FilterColor(LOW, HIGH);
        PRENDER(PORTB,PB2);
        break;
      case 3:
        g_array[2] = g_count;
        g_count = 0;
        g_flag ++;
        PRENDER(PORTL,PL0);  // TSC_FilterColor(HIGH, LOW);
        APAGAR(PORTB,PB2);
        break;
      default:  g_count = 0;
        break;
    }
    vTaskDelay( PROCESAMIENTO_MS );
  }

}  // Fin de TSC_Callback()


void vColorCountTask(void *pvParameters)
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;

  xSemaphoreTake( xColorSemaphore , 0 );
  
  for( ;; )
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
    }
    vTaskResume(xColorHandle);
  }

}  // Fin de vInitColor()


void vUltrasonicoTask( void *pvParameters )
{
  uint8_t pos = 90;
  bool direccionServo = true;
  
  for( ;; )
  {
    if( pos <= ANGULO_MAXIMO_DEG && movPermitido && inicio && (color==AMARILLO))
    {
      if( direccionServo && (pos <=(ANGULO_MAXIMO_DEG-PASO_MINIMO_DEG)) ) 
          pos+=PASO_MINIMO_DEG;
      else if( !direccionServo && pos >= ANGULO_MINIMO_DEG ) 
          pos-=PASO_MINIMO_DEG;
      else direccionServo = !direccionServo;
    }
    OCR3C = pos; 

    if( (lecturaHCSR04() >= DISTANCIA_MAXIMA_CM) && !meta ) movPermitido = true;
    else  movPermitido = false;
   
    vTaskDelay(TIEMPO_SERVO_MS);
  }
  
}  // Fin de vUltrasonicoTask()


/////////////////////////////////////////////////////////////////
// HANDLERS
//

void vColorCountHandler( void )
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( xColorSemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();

}  // Fin de vCountHandler()

#endif
