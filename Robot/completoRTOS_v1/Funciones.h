#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

#include "Constantes.h"

/////////////////////////////////////////////////////////////////
// VARIABLES EXTERNAS
//

extern uint16_t g_flag, g_count, g_array[3];
extern float g_SF[3];
extern int16_t ax, ay, az, gx, gy, gz;
extern uint8_t mpuIntStatus, fifoBuffer[128], velRobot, color;
extern uint16_t packetSize, fifoCount;
extern bool yawEstable, primeraMedicion;
extern bool movPermitido, inicio, meta, enCamino;
extern volatile bool mpuInterrupt;
extern float Yaw, yawAnterior, yawActual;
extern Quaternion q;

extern unsigned long last_read_time;
extern float last_x_angle, last_gyro_x_angle, last_y_angle, last_gyro_y_angle, last_z_angle, last_gyro_z_angle;
extern float base_x_gyro, base_x_accel, base_y_gyro, base_y_accel, base_z_gyro, base_z_accel;
extern float GYRO_FACTOR;

extern MPU6050 mpu;

extern SemaphoreHandle_t xColorSemaphore, xMPUSemaphore;
extern TaskHandle_t xInitColorHandle, xColorHandle, xHCSR04Handle, xMotoresHandle;
extern TaskHandle_t xDetectionHandle, xMPUHandle, xCallbackHandle, xColorCountHandle;

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES
//
void initIMU(void);
void initTSC(void);
void initSerial(void);
void xInitRTOS(void);
void initMotores(void);
void initUltrasonico(void);
void createTasks(void);
void initAll(void);

void processAngleData(void);
void calibrateSensors(void);
void cambiarVelocidad(void);
void setLastAngle(unsigned long, float, float, float, float, float, float);

void vColorCountHandler(void);
void dmpDataReady(void);

//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES MACRO Y "EN LINEA"
//
#define APAGAR(x,y)   x &= ~(1 << y);
#define PRENDER(x,y)  x |= (1 << y);
#define ENTRADA(x,y)  x &= ~(1 << y);
#define SALIDA(x,y)   x |= (1 << y);
#define sbi(x,y)      x |= 1 << y

inline unsigned long get_last_time() { return last_read_time; }
inline float get_last_x_angle() { return last_x_angle; }
inline float get_last_y_angle() { return last_y_angle; }
inline float get_last_z_angle() { return last_z_angle; }
inline float get_last_gyro_x_angle() { return last_gyro_x_angle; }
inline float get_last_gyro_y_angle() { return last_gyro_y_angle; }
inline float get_last_gyro_z_angle() { return last_gyro_z_angle; }

//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES PROPIAS
//

void processAngleData(void)
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
       unsigned long t_now = millis();

       float gyro_x = (gx - base_x_gyro) / GYRO_FACTOR;
       float gyro_y = (gy - base_y_gyro) / GYRO_FACTOR;
       float gyro_z = (gz - base_z_gyro) / GYRO_FACTOR;
       float accel_x = ax;
       float accel_y = ay;
       float accel_z = az;

       float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RAD_2_DEG;
       float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RAD_2_DEG;
       float accel_angle_z = 0;

       float dt = (t_now - get_last_time()) / 1000.0;
       float gyro_angle_x = gyro_x * dt + get_last_x_angle();
       float gyro_angle_y = gyro_y * dt + get_last_y_angle();
       float gyro_angle_z = gyro_z * dt + get_last_z_angle();

       float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
       float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
       float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();

       const float alpha = 0.96;
       float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
       float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
       float angle_z = gyro_angle_z;

       setLastAngle(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
       
}  // Fin de processAngleData()


void calibrateSensors(void)
{
  const int num_readings = 50;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  for (int i = 0; i < num_readings; i++)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;

}  // Fin de calibrateSensors()


void setLastAngle(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;

}  // Fin de setLastAngle()


void cambiarVelocidad(void)
{ 
  APAGAR(PORTH,3);    APAGAR(PORTH,6);
  APAGAR(PORTB,5);    APAGAR(PORTB,6);
  
  if( velRobot >= 255 )
  {
    velRobot = 255;
    PRENDER(PORTH,4);    PRENDER(PORTH,5);
    PRENDER(PORTB,4);    PRENDER(PORTB,7);
  }
  else if( velRobot <= 0 )
  {
    velRobot = 0;
    APAGAR(PORTH,4);    APAGAR(PORTH,5);
    APAGAR(PORTB,4);    APAGAR(PORTB,7);
  }
  else
  {
    sbi(TCCR4A, COM4B1);  OCR4B = velRobot;
    sbi(TCCR4A, COM4C1);  OCR4C = velRobot;
    sbi(TCCR2A, COM2A1);  OCR2A = velRobot;
    sbi(TCCR0A, COM0A1);  OCR0A = velRobot;
  }
//  Serial << velRobot << endl;
  
}  // Fin de cambiarVelocidad()


long lecturaHCSR04(void)
{
  long tiempoEco, distMedida;
  
  APAGAR(PORTE,3);      // Trigger -> LOW
  delayMicroseconds(2);
  PRENDER(PORTE,3);     // Trigger -> HIGH
  delayMicroseconds(10);
  APAGAR(PORTE,3);

  tiempoEco = pulseIn(4, HIGH);    // ECHO (DP 4)
  distMedida = tiempoEco / 58.2;

  return distMedida;

} // Fin de lecturaHCSR04()


//////////////////////////////////////////////////////////////////////////////////
// TAREAS
//

void vTaskCallback(void *pvParameters)
{
  for( ;; )
  {
    switch (g_flag)
    {
      case 0:  
        g_count = 0;
        g_flag ++;
        APAGAR(PORTL,PL0);
        APAGAR(PORTB,PB2);
        break;
      case 1: 
        g_array[0] = g_count;
        g_count = 0;
        g_flag ++;
        PRENDER(PORTL,PL0);
        PRENDER(PORTB,PB2);
        break;
      case 2: 
        g_array[1] = g_count;
        g_count = 0;
        g_flag ++;
        APAGAR(PORTL,PL0);
        PRENDER(PORTB,PB2);
        break;
      case 3:
        g_array[2] = g_count;
        g_count = 0;
        g_flag ++;
        PRENDER(PORTL,PL0);
        APAGAR(PORTB,PB2);
        break;
      default:  g_count = 0;
        break;
    }
    vTaskDelay( PROCESAMIENTO_MS );
  }

}  // Fin de vTaskCallback()

// ========================================================================================

void vTaskColorCount(void *pvParameters)
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
    vTaskDelay(INIT_COLOR_MS);

    g_SF[0] = 255.0 / g_array[0];    // R Scale factor
    g_SF[1] = 255.0 / g_array[1];    // G Scale factor
    g_SF[2] = 255.0 / g_array[2];    // B Scale factor

    Serial << F("SF => ") << g_SF[0] << F(" ") << g_SF[1] << F(" ") << g_SF[2] << endl;
    Serial << F("Color OK") << endl;
    delayMicroseconds(200);
//    vTaskPrioritySet(xColorHandle, tskIDLE_PRIORITY+3); 
    vTaskResume(xColorHandle);
  }

}  // Fin de vInitColor()


void vColorCountTask( void *pvParameters )
{
  int R,G,B;
//  vTaskPrioritySet(xInitColorHandle, tskIDLE_PRIORITY);
  vTaskSuspend(xInitColorHandle);
  
  for( ;; )
  {
    g_flag = 0;
  
    R = int(g_array[0] * g_SF[0]);
    G = int(g_array[1] * g_SF[1]);
    B = int(g_array[2] * g_SF[2]);
    
    if(!meta) Serial << R << " : " << G << " : " << B << endl;
    
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
          delayMicroseconds(200);
          vTaskResume(xHCSR04Handle);
        }
      }
      else if( COND_ROJO )
      {  
        color = ROJO;
        velRobot -= 130;
        Serial << F("Rojo") << endl;
      }
      else if( COND_AZUL )
      {
        color = AZUL;
        velRobot -= 40;
        Serial << F("Azul") << endl;
      }
      else Serial << F("Color no detectado") << endl;
    }
    else velRobot = 0;
    
    cambiarVelocidad();
   
    vTaskDelay( DATOS_FINAL_MS ); 
  } 
  
}  // Fin de vColorCountTask()


void vMPU6050Task( void *pvParameters )
{
   uint8_t diffCont = 0;
   // turn on the DMP, now that it's ready
   I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,true);
   attachInterrupt(0, dmpDataReady, RISING);  // enable Arduino interrupt detection
   mpuIntStatus = mpu.getIntStatus();
   uint8_t FS_SEL = 0;
   uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
   GYRO_FACTOR = 131.0/(FS_SEL + 1);
   uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
   packetSize = mpu.dmpGetFIFOPacketSize();    
   calibrateSensors();
   setLastAngle(millis(), 0, 0, 0, 0, 0, 0);
   Serial << F("MPU OK") << endl;
   ////////////////////////////////////////////////////////////// 
   for(;;)
   {
     unsigned long t_now = millis();
     while( !mpuInterrupt && fifoCount < packetSize)  processAngleData();
     mpuInterrupt = false;
     mpuIntStatus = mpu.getIntStatus();

     fifoCount = mpu.getFIFOCount();

     if((mpuIntStatus & 0x10) || fifoCount == 1024)
     {
       I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
       Serial << F("FIFO overflow") << endl;
     }
     else if(mpuIntStatus & 0x02)
     {
       while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
       I2Cdev::readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_R_W, packetSize, fifoBuffer);
       fifoCount -= packetSize;

       mpu.dmpGetQuaternion(&q, fifoBuffer);
       
       Yaw = RAD_2_DEG*atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);
       
       if (primeraMedicion == false)
       {
         yawActual = Yaw;
         primeraMedicion = true;
         Serial << F("Estabilizando IMU...") << endl;
       }
       else
       {
         yawAnterior = yawActual;
         yawActual = Yaw;
         
         if(yawEstable == false)
         {
           if( fabs(yawActual - yawAnterior) <=  0.01)  diffCont++;
//           Serial << yawActual << endl;
           if( diffCont >= 5 )
           {
             yawEstable = true;
             Serial << F("IMU ESTABILIZADO") << endl;
             vTaskResume(xCallbackHandle);
             vTaskResume(xColorCountHandle);
             vTaskResume(xInitColorHandle);
             Serial << "Tareas resumidas correctamente." << endl;
           }  
         }
       }

       if ( yawEstable == true )
       {
         // Detectar variaciones y corregirlas
         // Serial << Yaw << endl;
       }
     }
   }

}  // Fin de vMPU6050Task().


void vMpuDetectionTask(void *pvParameters)
{
   portBASE_TYPE xStatus;
   xSemaphoreTake( xMPUSemaphore , 0 );

   for ( ;; )
   {
     xSemaphoreTake( xMPUSemaphore , portMAX_DELAY );
     mpuInterrupt = true;
   }

}  // Fin de vMpuDetectionTask()


void vUltrasonicoTask( void *pvParameters )
{
  int16_t pos = 90;
  bool direccionServo = true;
  
  for( ;; )
  {
    if( pos <= ANGULO_MAXIMO_DEG && inicio && (color==AMARILLO) )
    {
      if( direccionServo && (pos <=(ANGULO_MAXIMO_DEG-PASO_MINIMO)) ) 
          pos+=PASO_MINIMO;
      else if( !direccionServo && pos >= ANGULO_MINIMO_DEG ) 
          pos-=PASO_MINIMO;
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


void dmpDataReady(void)
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( xMPUSemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );
  if ( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();

}  // Fin de dmpDataReady()

#endif
