////////////////////////////////////////////////////////////////////////////////////////
// completoRTOS_v1.ino
// ------------------------------------------------------------------------------------
// - Inicializa el giróscopo y el sensor de color.
//

#include <Streaming.h>
#include <Servo.h> 
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include "I2Cdev.h"

#include "Constantes.h"

uint16_t g_flag = 0, g_count = 0, g_array[3];
float g_SF[3];
int16_t ax, ay, az, gx, gy, gz;
uint8_t mpuIntStatus, fifoBuffer[128], velRobot = 0, color = BLANCO;
uint16_t packetSize, fifoCount;;
bool yawEstable = false, primeraMedicion = false;
bool movPermitido = true, inicio = false, meta = false, enCamino = false;;
volatile bool mpuInterrupt = false;
float Yaw, yawAnterior = 0, yawActual = 0;;
Quaternion q;

unsigned long last_read_time;
float last_x_angle, last_gyro_x_angle;
float last_y_angle, last_gyro_y_angle;
float last_z_angle, last_gyro_z_angle;
float base_x_gyro = 0, base_x_accel = 0;
float base_y_gyro = 0, base_y_accel = 0;
float base_z_gyro = 0, base_z_accel = 0;
float GYRO_FACTOR;

MPU6050 mpu;

SemaphoreHandle_t xColorSemaphore, xMPUSemaphore;
TaskHandle_t xInitColorHandle, xColorHandle, xHCSR04Handle, xMotoresHandle;
TaskHandle_t xDetectionHandle, xMPUHandle, xCallbackHandle, xColorCountHandle;

#include "Funciones.h"
#include "Inicializar.h"

///////////////////////////////////////////////////////////////////////////////////////
// LAZO PRINCIPAL
//

void loop(){}  

// Fin del programa
///////////////////////////////////////////////////////////////////////////////////////
