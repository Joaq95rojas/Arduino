#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

#include "Constantes.h"

extern int g_count, g_array[3], g_flag;
extern float g_SF[3];
extern bool enLaMeta, initColor, inicio, meta, movPermitido;
extern int velRobot, color, count;

extern Servo miServo;

extern SemaphoreHandle_t xColorSemaphore;

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES
//
void configPin( uint8_t, uint8_t );
void _digWrite( uint8_t, uint8_t );

void TSC_FilterColor(int, int);
void cambiarVelocidad(int);
long lecturaHCSR04(void);

void initTSC(void);
void initSemaphore(void);
void createTasks( void );


//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES MACRO Y "EN LINEA"
//
inline void initSerial(void) { Serial.begin(9600);  Serial << "* INICIADO *" << endl; }
inline void apagarMotor(uint16_t M) { _digWrite(M, LOW); }


//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES PROPIAS
//

void configPin( uint8_t pNum, uint8_t modo )
{
  volatile uint8_t *reg,*out;

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

  if( statePin )  *out |= BIT_PIN(pNum);
  else            *out &= ~BIT_PIN(pNum);

}  // Fin de _digWrite()


////////////////////////////////////////////////////////////////////////////////////////////

void TSC_FilterColor(int Level01, int Level02)
{
  // Select the filter color
  if(Level01 != 0)  Level01 = HIGH;
  if(Level02 != 0)  Level02 = HIGH;
 
  _digWrite(S2, Level01); 
  _digWrite(S3, Level02); 

}  // Fin de TSC_FilterColor()


void cambiarVelocidad( int velMotor)
{
  _digWrite( PIN_MOTOR_D1, LOW );
  analogWrite( PIN_MOTOR_D2, velMotor );
  analogWrite( PIN_MOTOR_D3, velMotor );
  _digWrite( PIN_MOTOR_D4, LOW );
  analogWrite( PIN_MOTOR_T1, velMotor );
  _digWrite( PIN_MOTOR_T2, LOW );
  _digWrite( PIN_MOTOR_T3, LOW );
  analogWrite( PIN_MOTOR_T4, velMotor );
//  Serial << velRobot << endl;
  
}  // Fin de cambiarVelocidad()


long lecturaHCSR04(void)
{
  long tiempoEco, distMedida;
  
  _digWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  _digWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  _digWrite(PIN_TRIG, LOW);

  tiempoEco = pulseIn(PIN_ECHO, HIGH);
  distMedida = (tiempoEco/2) / 29.1;

  return distMedida;

} // Fin de lecturaHCSR04()


//////////////////////////////////////////////////////////////////////////////////
// TAREAS
//

void vColorTask( void *pvParameters )
{
  int R,G,B;
  
  for( ;; )
  {
    while( !initColor )
    {
      g_flag = 0;
    
      R = int(g_array[0] * g_SF[0]);
      G = int(g_array[1] * g_SF[1]);
      B = int(g_array[2] * g_SF[2]);
    
      if( (R > 2*B) && (G>1.5*B) )
      {
        enLaMeta = true;
        color = AMARILLO;
      }
      else if( (R>200) && (G>200) && (B>200) )  // Color BLANCO
      {
        if( enLaMeta && !meta && inicio )
        {
          Serial << "META" << endl;
          meta = true;
          color = BLANCO;
        }  
        else if( !inicio )
        { 
          Serial << "INICIO" << endl; 
          inicio = true;
          movPermitido = true;
        }
      }
      else if( (R>2*G) && (R>2*B) )  color = ROJO;
      else if( (B>R) && (B>G) )      color = AZUL;
     
      vTaskDelay( DATOS_FINAL_MS ); 
    }
    vPortYield();
  }
  
}  // Fin de vColorTask()


void vMotoresTask( void *pvParameters )
{
  for( ;; )
  {
    if( movPermitido )
    {
      switch( color )
      {
        case AMARILLO: // Avanzar con aumento progresivo de velocidad
               velRobot += 40;
               if( velRobot >= 255 ) velRobot = 255;
               cambiarVelocidad(velRobot);
               break;
        case BLANCO: // Frenar motores suavemente
              velRobot -= 80;
              if( velRobot <= 0 )  velRobot = 0;
              cambiarVelocidad(velRobot);
              break;
        case ROJO:// Frenar motores bruscamente
              velRobot -= 130;
              if( velRobot <= 0 )  velRobot = 0;
              cambiarVelocidad(velRobot);
              break;
        case AZUL: // Disminuir la velocidad progresivamente
              velRobot -= 40;
              if( velRobot <= 0 )  velRobot = 0;
              cambiarVelocidad(velRobot);
              break;
      }
    }
    else    // if (movPermitido)
    { 
      velRobot = 0;
      cambiarVelocidad(velRobot);  
    }
    
    vTaskDelay(MOTORES_MS);
  }  // for( ;; )
  
}  // Fin de vMotoresTask()


void vTaskCallback(void *pvParameters)
{
  for( ;; )
  {
    switch (g_flag)
    {
      case 0:  
        g_count = 0;
        g_flag ++;
        TSC_FilterColor(LOW, LOW);
        break;
      case 1: 
        g_array[0] = g_count;
        g_count = 0;
        g_flag ++;
        TSC_FilterColor(HIGH, HIGH);
        break;
      case 2: 
        g_array[1] = g_count;
        g_count = 0;
        g_flag ++;
        TSC_FilterColor(LOW, HIGH);
        break;
      case 3:
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
    vPortYield();
  }

}  // Fin de vInitColor()


void vUltrasonicoTask( void *pvParameters )
{
  int16_t pos = 90;
  bool direccionServo = true;
  
  for( ;; )
  {
    if( pos <= ANGULO_MAXIMO_DEG )
    {
      if( direccionServo && (pos <=(ANGULO_MAXIMO_DEG-PASO_MINIMO_DEG)) && movPermitido) pos+=PASO_MINIMO_DEG;
      else if( !direccionServo && pos >= ANGULO_MINIMO_DEG && movPermitido ) pos-=PASO_MINIMO_DEG;
      else direccionServo = !direccionServo;
    }
    else  direccionServo = !direccionServo;
    miServo.write(pos); 

    if( (lecturaHCSR04() >= DISTANCIA_MAXIMA_CM) && !meta ) movPermitido = true;
    else  movPermitido = false;
    
    vTaskDelay(TIEMPO_SERVO_MS);
  }
  
}  // Fin de vUltrasonicoTask()


/////////////////////////////////////////////////////////////////
// HANDLERS

void vCountHandler( void )
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( xColorSemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();

}  // Fin de vCountHandler()

#endif
