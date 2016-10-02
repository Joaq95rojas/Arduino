#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <Streaming.h>

MPU6050 mpu;

#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus; 
uint16_t packetSize; 
uint16_t fifoCount; 
uint8_t fifoBuffer[128];

unsigned int count = 0;
bool betaEstable = false;
bool primeraMedicion = false;
float betaAnterior = 0, betaActual = 0;
long segundos = 0;
float diff = 0;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];

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

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

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

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;
void dmpDataReady() 
{
    mpuInterrupt = true;
}

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================

void calibrate_sensors() 
{
  int num_readings = 30;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
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

ISR(TIMER2_OVF_vect)   //Timer2 Overflow Interrupt Vector, called every 1ms
{
  count++;               //Increments the interrupt counter
  if(count >= 1000)
  {
    count = 0;           //Resets the interrupt counter

    if(betaEstable == false)
    {
      segundos++;
      if( primeraMedicion == true )
      {
        diff = fabs(betaActual -  betaAnterior);
//        Serial << diff << endl;
        if( diff <=  0.01) 
        {
          betaEstable = true;
          Serial << "BETA ESTABLE EN [" << segundos << " s]" << endl;
        }
      }
    } 
  }
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};  

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
    Wire.begin();
    Serial.begin(57600);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) 
    {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        uint8_t FS_SEL = 0;
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
        Serial.print("FS_SEL = ");
        Serial.println(READ_FS_SEL);
        GYRO_FACTOR = 131,0 / pow (2 , FS_SEL);
        uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
        Serial.print("AFS_SEL = ");
        Serial.println(READ_AFS_SEL);
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    // get calibration values for sensors
    calibrate_sensors();
    set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);

    //Setup Timer2 to fire every 1ms
    TCCR2B = 0x00;        //Disbale Timer2 while we set it up
    TCNT2  = 130;         //Reset Timer Count to 130 out of 255
    TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
    TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
    TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
    const float RADIANS_TO_DEGREES = 57.2958;
    if (!dmpReady) return;
    unsigned long t_now = millis();
    while (!mpuInterrupt && fifoCount < packetSize) 
    {    
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        unsigned long t_now = millis();
          
        float gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
        float accel_x = ax; 
        float accel_y = ay; 
        float accel_z = az; 
 
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();     
        
        const float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;
        
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);       
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) 
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        
        if(primeraMedicion == false)
        {
          betaActual = euler[0]*RADIANS_TO_DEGREES;
          primeraMedicion = true;
        }
        else
        {
          betaAnterior = betaActual;
          betaActual = euler[0]*RADIANS_TO_DEGREES;
        }

        if( betaEstable == true )
        {
//          Serial << betaActual << endl;
      //    Serial << "Euler: " << _FLOAT(euler[1]*RADIANS_TO_DEGREES,2) << endl;
          Serial << "EULER: " << _FLOAT(euler[0]*RADIANS_TO_DEGREES,2) << ":" << _FLOAT(euler[1]*RADIANS_TO_DEGREES,2) << ":" << _FLOAT(euler[2]*RADIANS_TO_DEGREES,2) << endl;

        }
    }
}
