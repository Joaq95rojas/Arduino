///////////////////////////////////////////////////////////////////////////////////////
// betaRTOS_v2.ino
// ------------------------------------------------------------------------------------
// - Se agrega la tarea periódica que estabiliza la medición.
// - Se calcula el ángulo de Yaw.
//

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Streaming.h>
#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"

#include "defines.h"

#define RAD_2_DEG  57.2958
uint8_t diffCont = 0;
float diff = 0; 

MPU6050 mpu;
SemaphoreHandle_t xMPUSemaphore;
TaskHandle_t xIntHandle, xMPUHandle;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[128];

unsigned int count = 0;
bool yawEstable = false;
bool primeraMedicion = false;
float yawAnterior = 0, yawActual = 0;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float Yaw;

unsigned long last_read_time;
float         last_x_angle;
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void setLastAngle(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {
  return last_read_time;
}
inline float get_last_x_angle() {
  return last_x_angle;
}
inline float get_last_y_angle() {
  return last_y_angle;
}
inline float get_last_z_angle() {
  return last_z_angle;
}
inline float get_last_gyro_x_angle() {
  return last_gyro_x_angle;
}
inline float get_last_gyro_y_angle() {
  return last_gyro_y_angle;
}
inline float get_last_gyro_z_angle() {
  return last_gyro_z_angle;
}

float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

float    GYRO_FACTOR;

int16_t ax, ay, az;
int16_t gx, gy, gz;

char dataOut[256];

volatile bool mpuInterrupt = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpDataReady(void)
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( xMPUSemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );
  if ( xHigherPriorityTaskWoken == pdTRUE )  vPortYield();
}

void vDetectionTask(void *pvParameters)
{
   portBASE_TYPE xStatus;
   xSemaphoreTake( xMPUSemaphore , 0 );

   for ( ;; )
   {
     xSemaphoreTake( xMPUSemaphore , portMAX_DELAY );
     mpuInterrupt = true;
   }

}  // Fin de vDetectionTask()

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================

void calibrate_sensors()
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
}

////////////////////////////////////////////////////////////////////
// TIMER HANDLER
//
void vInterruptTask(void *pvParameters)
{/*
   TickType_t xLastWakeTime;
   uint8_t diffCont = 0;
   float diff = 0;
   long segundos = 0;

   for ( ;; )
   {
     xLastWakeTime = xTaskGetTickCount();
     if(yawEstable == false)
     {
       segundos++;
       Serial << segundos << endl;
       if( primeraMedicion == true )
       {
         diff = fabs(yawActual -  yawAnterior);
         if( diff <=  0.01)  diffCont++;
         if( diffCont >= 10 )
         {
           yawEstable = true;
           Serial << F("BETA ESTABLE EN [") << segundos << F(" s]") << endl;
           diffCont = 0;
         } 
       }
     }
     vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
   }*/

}  // Fin de vInterruptTask()

////////////////////////////////////////////////////////////////////////////////////

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Wire.begin();
  Serial.begin(57600); Serial << F("Serial OK") << endl;
  vSemaphoreCreateBinary( xMPUSemaphore );
  
  I2Cdev::writeBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);  // Set Clock Source
  I2Cdev::writeBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH,MPU6050_GYRO_FS_250);
  I2Cdev::writeBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT,false); 
  
  mpu.dmpInitialize();
  ////////////////////////////////////////////////////////////////////////////////  
  xTaskCreate(vDetectionTask, "Detection", 128, NULL, 2, NULL);
  xTaskCreate(vMPU6050Task, "MPU6050", 256, NULL, 1, NULL);
  /////////////////////////////////////////////////////////////////////////////////
  vTaskStartScheduler();
  Serial << F("Memoria RAM insuficiente") << endl;
  for (;;);
}

////////////////////////////////////////////////////////////////////


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

////////////////////////////////////////////////////////////////////
// MAIN PROGRAM LOOP
//

void vMPU6050Task( void *pvParameters )
{
   // turn on the DMP, now that it's ready
   I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,true);
   attachInterrupt(0, dmpDataReady, RISING);  // enable Arduino interrupt detection
   mpuIntStatus = mpu.getIntStatus();
   uint8_t FS_SEL = 0;
   uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
   GYRO_FACTOR = 131.0/(FS_SEL + 1);
   uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
   packetSize = mpu.dmpGetFIFOPacketSize();    
   calibrate_sensors();
   setLastAngle(millis(), 0, 0, 0, 0, 0, 0);
   Serial << F("Init Successful") << endl;
//   vTaskResume(xIntHandle);
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
       // dmpGetGravity():
/*       v.x = 2 * (q.x*q.z - q.w*q.y);
       v.y = 2 * (q.w*q.x + q.y*q.z);
       v.z = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;*/
       
       // Los parámetros anteriores se utilizan para calcular el Pitch y el Roll
       
       // dmpGetYawPitchRoll()
       Yaw = RAD_2_DEG*atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);
       
       if (primeraMedicion == false)
       {
         yawActual = Yaw;
         primeraMedicion = true;
         Serial << "Estabilizando angulo de Yaw" << endl;
       }
       else
       {
         yawAnterior = yawActual;
         yawActual = Yaw;
         
         diff = fabs(yawActual -  yawAnterior);
         if( diff <=  0.01)  diffCont++;
         Serial << diff << endl;
         if( diffCont >= 10 )
         {
           yawEstable = true;
           //Serial << F("BETA ESTABLE EN [") << segundos << F(" s]") << endl;
           Serial << "Angulo estable" << endl;
           diffCont = 0;
         } 
       }

       if ( yawEstable == true )  Serial << Yaw << endl;
     }
   }

}  // Fin de vMPU6050Task()

void loop() {}
