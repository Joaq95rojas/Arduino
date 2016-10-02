// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include <avr/interrupt.h>  
#include <avr/io.h>
#include <Streaming.h>
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Mega 2560, this is digital I/O 
   pin 2.
 * ========================================================================= */

#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

unsigned int count = 0;   //used to keep count of how many interrupts were fired
bool yawEstable = false;
bool primeraMedicion = false;
float yawAnterior = 0, yawActual = 0;
long segundos = 0;
float diff = 0;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
float yaw_values[3];

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
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

//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Buffer for data output
char dataOut[256];
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
// Simple calibration - just average first few readings to subtract
// from the later data
void calibrate_sensors() 
{
  int       num_readings = 10;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
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
//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) 
{
  count++;               //Increments the interrupt counter
  if(count >= 1000)
  {
    count = 0;           //Resets the interrupt counter
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    if(yawEstable == false)
    {
      segundos++;
      if( primeraMedicion == true )
      {
        diff = fabs(yawActual -  yawAnterior);
        if( diff <=  0.01) 
        {
          yawEstable = true;
          Serial << "YAW ESTABILIZADO [" << segundos << " s]" << endl;
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
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(57600);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Set the full scale range of the gyro
        uint8_t FS_SEL = 0;
        //mpu.setFullScaleGyroRange(FS_SEL);

        // get default full scale value of gyro - may have changed from default
        // function call returns values between 0 and 3
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
        Serial.print("FS_SEL = ");
        Serial.println(READ_FS_SEL);
        //GYRO_FACTOR = 131.0/(FS_SEL + 1);
        GYRO_FACTOR = 131,0 / pow (2 , FS_SEL);

        // get default full scale value of accelerometer - may not be default value.  
        // Accelerometer scale factor doesn't reall matter as it divides out
        uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
        Serial.print("AFS_SEL = ");
        Serial.println(READ_AFS_SEL);
        //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
        
        // Set the full scale range of the accelerometer
        //uint8_t AFS_SEL = 0;
        //mpu.setFullScaleAccelRange(AFS_SEL);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    yaw_values[0] = 0; yaw_values[1] = 400; yaw_values[2] = -400;
    
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
    const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    unsigned long t_now = millis();

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) 
    {    
        // Keep calculating the values of the complementary filter angles for comparison with DMP here
        // Read the raw accel/gyro values from the MPU-6050
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
              
        // Get time of last raw data read
        unsigned long t_now = millis();
          
        // Remove offsets and scale gyro data  
        float gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
        float accel_x = ax; // - base_x_accel;
        float accel_y = ay; // - base_y_accel;
        float accel_z = az; // - base_z_accel;
        
              
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();     
        
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        const float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);       
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // Obtain Euler angles from buffer
        //mpu.dmpGetQuaternion(&q, fifoBuffer);
        //mpu.dmpGetEuler(euler, &q);
        
        // Obtain YPR angles from buffer
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if(primeraMedicion == false)
        {
          yawActual = ypr[0]*RADIANS_TO_DEGREES;
          primeraMedicion = true;
        }
        else
        {
          yawAnterior = yawActual;
          yawActual = ypr[0]*RADIANS_TO_DEGREES;
        }

        if( yawEstable == true )
        {
//          Serial << yawActual << endl;
          Serial.print("CMP:");
         Serial.print(get_last_x_angle(), 2);
         Serial.print(":");
         Serial.print(get_last_y_angle(), 2);
         Serial.print(":");
         Serial.println(-get_last_z_angle(), 2);
         Serial.print("DMP:");
         Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
         Serial.print(":");
         Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
         Serial.print(":");
         Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);
        }
  /*
       // Output complementary data and DMP data to the serial port.  The signs on the data needed to be
       // fudged to get the angle direction correct.
       Serial.print("CMP:");
       Serial.print(get_last_x_angle(), 2);
       Serial.print(":");
       Serial.print(get_last_y_angle(), 2);
       Serial.print(":");
       Serial.println(-get_last_z_angle(), 2);
       Serial.print("DMP:");
       Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
       Serial.print(":");
       Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
       Serial.print(":");
       Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);*/
    }
}