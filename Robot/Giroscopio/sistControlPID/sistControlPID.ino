#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <Streaming.h>

MPU6050 mpu;

#define ESTABILIDAD 0.05
#define TIEMPO_ESTABILIDAD_FINAL  5     // [segundos]
#define THRESHOLD_LINEA 0.1

#define ENTRADA 0
#define SALIDA  1

#define PIN_MOTOR_D1 6
#define PIN_MOTOR_D2 7
#define PIN_MOTOR_D3 8 
#define PIN_MOTOR_D4 9
#define PIN_MOTOR_T1 10 
#define PIN_MOTOR_T2 11
#define PIN_MOTOR_T3 12 
#define PIN_MOTOR_T4 13

#define BIT_PIN(i)  digitalPinToBitMask(i)
#define OUT_PIN(i)  portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)  portModeRegister(digitalPinToPort(i))
#define IN_PIN(i) *portInputRegister(digitalPinToPort(i))

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

unsigned int count = 0;   //used to keep count of how many interrupts were fired
bool yawEstable = false;
bool primeraMedicion = true;
bool motoresEncendidos = true;  // Los motores comienzan encendidos
bool initPID = false;
float yawAnterior = 0, yawActual = 0;
uint16_t segundos = 0, segEstabilizado = 0, minutos;
float diff = 0;
float errorDireccion = 0, errorAcumulado = 0;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

float Setpoint;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7

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
    segundos++;

    if(yawEstable == false)
    {
      

      if( primeraMedicion == false )
      {
        diff = fabs(yawActual -  yawAnterior);
        if( diff <=  ESTABILIDAD ) 
        {
          yawEstable = true;
          segEstabilizado = segundos;
          Serial << "YAW ESTABILIZADO [" << segEstabilizado << " s]" << endl;
        }
      }
    }
    else  if( (segundos - segEstabilizado) >= TIEMPO_ESTABILIDAD_FINAL )  robotAdelante();  
  }
  actualizarHora();
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};  

// ================================================================
// ===                FUNCIONES AGREGADAS                       ===
// ================================================================

void robotAdelante( void )
{ 
  if ( motoresEncendidos == false )
  {
    analogWrite(PIN_MOTOR_D1, LOW);
    analogWrite(PIN_MOTOR_D2, HIGH); 
    analogWrite(PIN_MOTOR_D3, HIGH); 
    analogWrite(PIN_MOTOR_D4, LOW);

    analogWrite(PIN_MOTOR_T1, HIGH);
    analogWrite(PIN_MOTOR_T2, LOW);
    analogWrite(PIN_MOTOR_T3, LOW);
    analogWrite(PIN_MOTOR_T4, HIGH);

    motoresEncendidos = true;
    Serial << "Encendiendo motores del robot" << endl;
  }
  
} // Fin de robotAdelante()

void girarRobotCW( int velMotor )
{
  analogWrite( PIN_MOTOR_D3, velMotor );
  _digWrite( PIN_MOTOR_D4, LOW );
  analogWrite( PIN_MOTOR_T4, velMotor );
  _digWrite( PIN_MOTOR_T3, LOW );

  _digWrite( PIN_MOTOR_D1, LOW );
  _digWrite( PIN_MOTOR_D2, LOW );
  _digWrite( PIN_MOTOR_T1, LOW );
  _digWrite( PIN_MOTOR_T2, LOW );

}  // Fin de girarRobotCW()

void girarRobotCCW( int velMotor )
{
  analogWrite( PIN_MOTOR_D2, velMotor );
  _digWrite( PIN_MOTOR_D1, LOW );
  analogWrite( PIN_MOTOR_T1, velMotor );
  _digWrite( PIN_MOTOR_T2, LOW );

  _digWrite( PIN_MOTOR_D3, LOW );
  _digWrite( PIN_MOTOR_D4, LOW );
  _digWrite( PIN_MOTOR_T3, LOW );
  _digWrite( PIN_MOTOR_T4, LOW );

}  // Fin de girarRobotCCW()


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

} // Fin de configPin()

void _digWrite( uint8_t pNum, uint8_t statePin )
{
  volatile uint8_t* out = OUT_PIN(pNum);

  if( statePin )  *out |= BIT_PIN(pNum);
  else            *out &= ~BIT_PIN(pNum);

} // Fin de _digWrite()

void pararRobot( void )
{ 
  if ( motoresEncendidos == true )
  {
    _digWrite(PIN_MOTOR_D1, LOW);
    _digWrite(PIN_MOTOR_D2, LOW);
    _digWrite(PIN_MOTOR_D3, LOW);
    _digWrite(PIN_MOTOR_D4, LOW);

    _digWrite(PIN_MOTOR_T1, LOW);
    _digWrite(PIN_MOTOR_T2, LOW);
    _digWrite(PIN_MOTOR_T3, LOW);
    _digWrite(PIN_MOTOR_T4, LOW);

    motoresEncendidos = false;
  }

} // Fin de pararRobot()

void actualizarHora( void )
{
  if( segundos >= 60 )
  {
    minutos++;
    segundos = 0;
  }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
  configPin( PIN_MOTOR_D1, SALIDA );
  configPin( PIN_MOTOR_D2, SALIDA );
  configPin( PIN_MOTOR_D3, SALIDA );
  configPin( PIN_MOTOR_D4, SALIDA );

  configPin( PIN_MOTOR_T1, SALIDA );
  configPin( PIN_MOTOR_T2, SALIDA );
  configPin( PIN_MOTOR_T3, SALIDA );
  configPin( PIN_MOTOR_T4, SALIDA );

  pararRobot();

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
      GYRO_FACTOR = 131.0/(FS_SEL + 1);
      //GYRO_FACTOR = 131,0 / pow (2 , FS_SEL);

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

  Serial << "Espere mientras se estabiliza el angulo de Yaw." << endl;
  
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
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        
        // Obtain YPR angles from buffer
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if(primeraMedicion == true)
        {
          yawActual = ypr[0]*RADIANS_TO_DEGREES;
          primeraMedicion = false;
        }
        else
        {
          yawAnterior = yawActual;
          yawActual = ypr[0]*RADIANS_TO_DEGREES;
        }

        if( yawEstable == true && motoresEncendidos == true)
        {
          if( initPID == false )
          {
            //initialize the variables we're linked to
            Setpoint = yawActual;   // En grados!!

            Serial << "SETPOINT: " << Setpoint << endl;
            Serial << " ---------" << endl;

            initPID = true;
          }
          
//          Serial << yawActual << endl;
//          Serial << "E: " << (yawActual - Setpoint) << endl;

          errorDireccion = yawActual - Setpoint;
          errorAcumulado += errorDireccion;
          Serial << "EA: " << errorAcumulado << endl;

          if ( errorDireccion > THRESHOLD_LINEA )
          {
            girarRobotCCW(255);  
          }
          else if(errorDireccion < (-THRESHOLD_LINEA))
          {
            girarRobotCW(255);
          }
          else robotAdelante();

//          Serial << "CMP:" << _FLOAT(get_last_x_angle(),2) << ":" << _FLOAT(get_last_y_angle(),2) << ":" << _FLOAT(-get_last_z_angle(),2) << endl;
//          Serial << "DMP:" << _FLOAT(ypr[2]*RADIANS_TO_DEGREES,2 << ":" << _FLOAT(-ypr[1]*RADIANS_TO_DEGREES,2) << ":" << _FLOAT(ypr[0]*RADIANS_TO_DEGREES,2) << endl;
        }
    }
}