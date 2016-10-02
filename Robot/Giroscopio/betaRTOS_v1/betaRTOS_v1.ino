#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Streaming.h>
#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"

#include "defines.h"

MPU6050 mpu;
SemaphoreHandle_t xMPUSemaphore;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[128];  // = 128

unsigned int count = 0;
bool betaEstable = false;
bool primeraMedicion = false;
float betaAnterior = 0, betaActual = 0;
long segundos = 0;
float diff = 0;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float beta;

unsigned long last_read_time;
float         last_x_angle;
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
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
float    ACCEL_FACTOR;

int16_t ax, ay, az;
int16_t gx, gy, gz;

char dataOut[256];

volatile bool initMPU = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt2 = false;

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
   bool mpuInterrupt = true;
   xSemaphoreTake( xMPUSemaphore , 0 );

   for ( ;; )
   {
     xSemaphoreTake( xMPUSemaphore , portMAX_DELAY );
     mpuInterrupt2 = true;
   }

}  // Fin de vDetectionTask()

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================

void calibrate_sensors()
{
  const int num_readings = 10;
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

// ================================================================
// ===                      TIMER 2 HANDLER                     ===
// ================================================================

void vInterruptTask(void *pvParameters)
{/*
   TickType_t xLastWakeTime;

   for ( ;; )
   {
     xLastWakeTime = xTaskGetTickCount();
     if (betaEstable == false)
     {
       segundos++;
       Serial << segundos << endl;
       if ( primeraMedicion == true )
       {
         diff = fabs(betaActual -  betaAnterior);
         if ( diff <=  0.01)
         {
           betaEstable = true;
           Serial << F("BETA ESTABLE EN [") << segundos << F(" s]") << endl;
         } 
         Serial << "D: " << diff << endl;
       }
     }
     vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
   }
*/
}  // Fin de vInterruptTask()

////////////////////////////////////////////////////////////////////////////////////

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
uint8_t bank;
const uint8_t devAddr = MPU6050_DEFAULT_ADDRESS;
  
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
 /* 
    I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);  // Trigger a full device reset.
    delay(30); // wait after reset

    I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false);  // disable sleep mode

    // get MPU hardware revision
    bank = 0x10;
    bank &= 0x1F;
    bank |= 0x20;
    bank |= 0x40;
    I2Cdev::writeByte(devAddr, MPU6050_RA_BANK_SEL, bank);  // BANK_SEL register
    I2Cdev::writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, 0x06);  // MEM_START_ADDR register
/////////////////////////////////   
  uint8_t hwRevision = mpu.readMemoryByte();
////////////////////////////////
    I2Cdev::writeByte(devAddr, MPU6050_RA_BANK_SEL, 0);

    // check OTP bank valid
    uint8_t otpValid = mpu.getOTPBankValid();
  
    // get X/Y/Z gyro offsets
    int8_t xgOffsetTC = mpu.getXGyroOffsetTC();
    int8_t ygOffsetTC = mpu.getYGyroOffsetTC();
    int8_t zgOffsetTC = mpu.getZGyroOffsetTC();
  
    // setup weird slave stuff (?)
    I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR, 0x7F);  // Set the I2C address of the specified slave (0)
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, false);
    I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR, 0x68);
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
    delay(20);

    I2Cdev::writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_ZGYRO);
    I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, 0x12);
/////////////////////////////////////////
    I2Cdev::writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, 4); // 1khz / (1 + n) ; Default n=4 (200 Hz)
/////////////////////////////////////////
    I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, MPU6050_EXT_SYNC_TEMP_OUT_L);
    I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
    I2Cdev::writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_2000);
    I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_1, 0x03);
    I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_2, 0x00);
    I2Cdev::writeBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, false);
    I2Cdev::writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, xgOffsetTC);
    I2Cdev::writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, ygOffsetTC);
    I2Cdev::writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, zgOffsetTC);

    uint8_t dmpUpdate[16], j;
    uint16_t pos = 0;
    for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
    mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
    mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);

    uint16_t fifoCount = mpu.getFIFOCount();
    uint8_t fifoBuffer[128];

    I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, fifoCount, fifoBuffer);
    I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_THR, 2);
    I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_THR, 156);
    I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_DUR, 80);
    I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_DUR, 0);
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, true);
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, true);
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);

    for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
    mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
    for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
    mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
    for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
    mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
    while ((fifoCount = mpu.getFIFOCount()) < 3);

    I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, fifoCount, fifoBuffer);
    uint8_t mpuIntStatus = mpu.getIntStatus();
    for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
    mpu.readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
    while ((fifoCount = mpu.getFIFOCount()) < 3);
    I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, fifoCount, fifoBuffer);
    mpuIntStatus = mpu.getIntStatus();
    for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
    mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, false);

    mpu.dmpPacketSize = 42;
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
    mpu.getIntStatus();
  */
////////////////////////////////////////////////////////////////////////////////  
  xTaskCreate(vDetectionTask, "Detection", 300, NULL, 3, NULL);
//  xTaskCreate(vInterruptTask, "Periodical", 256, NULL, 2, NULL);
  xTaskCreate(vMPU6050Task, "MPU6050", 600, NULL, 2, NULL);
/////////////////////////////////////////////////////////////////////////////////
  vTaskStartScheduler();
  Serial << F("Memoria RAM insuficiente") << endl;
  for (;;);
}

////////////////////////////////////////////////////////////////////
// MAIN PROGRAM LOOP
//

void vMPU6050Task( void *pvParameters )
{
   bool mpuInterrupt;
   const uint8_t devAddr = MPU6050_DEFAULT_ADDRESS;
   const float RADIANS_TO_DEGREES = 57.2958;
   
  // turn on the DMP, now that it's ready
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,true);

  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;
  uint8_t FS_SEL = 0;
  uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
  GYRO_FACTOR = 131.0/(FS_SEL + 1);
  uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
  packetSize = mpu.dmpGetFIFOPacketSize();    
  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
   
   Serial << F("Init Successful") << endl;
   
   for(;;)
   {
     if (!dmpReady) vPortYield();
     unsigned long t_now = millis();
      while( !mpuInterrupt2 && fifoCount < packetSize)
     {
       mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
       unsigned long t_now = millis();

       float gyro_x = (gx - base_x_gyro) / GYRO_FACTOR;
       float gyro_y = (gy - base_y_gyro) / GYRO_FACTOR;
       float gyro_z = (gz - base_z_gyro) / GYRO_FACTOR;
       float accel_x = ax;
       float accel_y = ay;
       float accel_z = az;

       float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
       float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
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

       set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
     }
     mpuInterrupt2 = false;
     mpuIntStatus = mpu.getIntStatus();

     fifoCount = mpu.getFIFOCount();

     if ((mpuIntStatus & 0x10) || fifoCount == 1024)
     {
       I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
       Serial.println(F("FIFO overflow!"));
     }
     else if (mpuIntStatus & 0x02)
     {
       while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
       I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, packetSize, fifoBuffer);
       fifoCount -= packetSize;

       mpu.dmpGetQuaternion(&q, fifoBuffer);
       beta = RADIANS_TO_DEGREES * atan2(2*q.x*q.y-2*q.w*q.z, 2*q.w*q.w+2*q.x*q.x-1);   // psi
/*
       if (primeraMedicion == false)
       {
         betaActual = beta;
         primeraMedicion = true;
       }
       else
       {
         betaAnterior = betaActual;
         betaActual = beta;
       }

       if ( betaEstable == true )  Serial << beta << endl;*/
       Serial << beta << endl;
     }
   }

}  // Fin de vMPU6050Task()

void loop() {}
