#ifndef _INICIALIZAR_H_
#define _INICIALIZAR_H_

#include "Funciones.h"

//////////////////////////////////////////////////////////////////////////////////
// INICIALIZACION
//

void initTSC(void)
{
  // Init TSC230 and setting Frequency.
  SALIDA(DDRL,PL4);  // configPin(S0, SALIDA);
  SALIDA(DDRL,PL2);  // configPin(S1, SALIDA);
  SALIDA(DDRL,PL0);  // configPin(S2, SALIDA);
  SALIDA(DDRB,PB2);  // configPin(S3, SALIDA);
  ENTRADA(DDRD,PD2);  // configPin(OUT, ENTRADA);
  
  // OUTPUT FREQUENCY SCALING 2%
  APAGAR(PORTL,4);   // S0: DP 45
  PRENDER(PORTL,2);  // S1: DP 47 

}  // Fin de TSC_Init()


void xInitRTOS(void)
{
  vSemaphoreCreateBinary( xColorSemaphore );
  vSemaphoreCreateBinary( xMPUSemaphore );

}  // Fin de initSemaphore()


void initMotores(void)
{
  SALIDA(DDRH,PH3);  APAGAR(PORTH,3);  // DP 6
  SALIDA(DDRH,PH4);  APAGAR(PORTH,4);  // DP 7
  SALIDA(DDRH,PH5);  APAGAR(PORTH,5);  // DP 8
  SALIDA(DDRH,PH6);  APAGAR(PORTH,6);  // DP 9

  SALIDA(DDRB,PB4);  APAGAR(PORTB,4);  // DP 10
  SALIDA(DDRB,PB5);  APAGAR(PORTB,5);  // DP 11
  SALIDA(DDRB,PB6);  APAGAR(PORTB,6);  // DP 12
  SALIDA(DDRB,PB7);  APAGAR(PORTB,7);  // DP 13

}  // Fin de initMotores()


void initIMU(void)
{
  I2Cdev::writeBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);  // Set Clock Source
  I2Cdev::writeBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH,MPU6050_GYRO_FS_250);
  I2Cdev::writeBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT,false); 
  
  mpu.dmpInitialize();
  
  Serial << "IMU Pre-Init OK" << endl;

}  // Fin de initIMU()


// ======================================================================================


void initUltrasonico(void) 
{
  ENTRADA(DDRG,PG5);    // configPin(PIN_ECHO, ENTRADA);
  SALIDA(DDRE,PE3);     // configPin(PIN_TRIG, SALIDA);
  
  DDRE |= 1<<PE5;      // Pin 3 as PWM Output
  // Timer Counter Control Register (TCCR3 - Timer 3)
  TCCR3A |= (1<<WGM30)|(1<<WGM31)|(1<<COM3C1);
  TCCR3B |= (1<<CS30);
  OCR3C = 200;        // Posición inicial del servo
        
}  // Fin de initUltrasonico()


inline void initSerial(void) { 
        Serial.begin(57600);  
        Serial << F("** Serial OK **") << endl; 

}  // Fin de initSerial()


// =======================================================================================

void createTasks( void )
{
  	xTaskCreate(
                    vColorCountTask,
		    "Colores",
		    STACK_COLOR,
		    NULL,
		    PRIORIDAD_COLOR,
		    &xColorHandle
		   );
  
	xTaskCreate(
                    vTaskCallback,
		    "Callback",
		    STACK_CALLBACK,
		    NULL,
		    PRIORIDAD_CALLBACK,
		    &xCallbackHandle
		   );

	xTaskCreate(
                    vTaskColorCount,
		    "ColorCount",
		    STACK_CALLBACK,
		    NULL,
		    PRIORIDAD_CALLBACK,
		    &xColorCountHandle
		   );

        xTaskCreate(
                    vInitColor,
		    "InitColor",
		    STACK_INIT,
		    NULL,
		    PRIORIDAD_INIT,
		    &xInitColorHandle
		   );

        xTaskCreate(
                    vUltrasonicoTask,
                    "HCSR04",
                    STACK_HCSR04,
                    NULL,
                    PRIORIDAD_HCSR04,
                    &xHCSR04Handle
                  );

        xTaskCreate(
                    vMpuDetectionTask, 
                    "Detection", 
                    STACK_DETECTION, 
                    NULL, 
                    PRIORIDAD_DETECTION, 
                    &xDetectionHandle
                   );
                   
        xTaskCreate(
                    vMPU6050Task, 
                    "MPU6050", 
                    STACK_MPU, 
                    NULL, 
                    PRIORIDAD_MPU, 
                    &xMPUHandle
                   );

}	// Fin de createTasks()


void initAll(void)
{
  initMotores();
  Wire.begin();
  initTSC();
  initSerial();
  xInitRTOS();
  initUltrasonico();
  
  attachInterrupt(4, vColorCountHandler, RISING);  

}  // Fin de initAll()


//////////////////////////////////////////////////////////////////////////////////
// SETUP()
//
void setup()
{
  initAll();
  
  createTasks();
  vTaskSuspend(xColorHandle);
  vTaskSuspend(xCallbackHandle);
  vTaskSuspend(xColorCountHandle);
  vTaskSuspend(xInitColorHandle);
  vTaskSuspend(xHCSR04Handle);
  initIMU();
  vTaskStartScheduler();
  Serial << F("Memoria RAM insuficiente") << endl;
  for( ;; );

}  // Fin de setup()


#endif
