#ifndef _INICIALIZAR_H_
#define _INICIALIZAR_H_

#include "Funciones.h"

//////////////////////////////////////////////////////////////////////////////////
// INICIALIZACION
//

void initTSC(void)
{
  // Init TSC230 and setting Frequency.
  configPin(S0, OUTPUT);
  configPin(S1, OUTPUT);
  configPin(S2, OUTPUT);
  configPin(S3, OUTPUT);
  configPin(OUT, INPUT);
  
  configPin(13,OUTPUT);
  _digWrite(13,LOW);
 
  _digWrite(S0, LOW);  // OUTPUT FREQUENCY SCALING 2%
  _digWrite(S1, HIGH); 

}  // Fin de TSC_Init()


void initSemaphore(void)
{
  vSemaphoreCreateBinary( xColorSemaphore );

}  // Fin de initSemaphore()


void initMotores(void)
{
  configPin( PIN_MOTOR_D1, SALIDA );  apagarMotor(PIN_MOTOR_D1);
  configPin( PIN_MOTOR_D2, SALIDA );  apagarMotor(PIN_MOTOR_D2);
  configPin( PIN_MOTOR_D3, SALIDA );  apagarMotor(PIN_MOTOR_D3);
  configPin( PIN_MOTOR_D4, SALIDA );  apagarMotor(PIN_MOTOR_D4);

  configPin( PIN_MOTOR_T1, SALIDA );  apagarMotor(PIN_MOTOR_T1);
  configPin( PIN_MOTOR_T2, SALIDA );  apagarMotor(PIN_MOTOR_T2);
  configPin( PIN_MOTOR_T3, SALIDA );  apagarMotor(PIN_MOTOR_T3);
  configPin( PIN_MOTOR_T4, SALIDA );  apagarMotor(PIN_MOTOR_T4);

}  // Fin de initMotores()


inline void initUltrasonico(void) {
        configPin(PIN_TRIG, SALIDA);
        configPin(PIN_ECHO, ENTRADA);
        miServo.attach(PIN_SERVO);
        miServo.write(90);            // Se posiciona inicialmente en 90 grados.
        
}  // Fin de initUltrasonico()


void createTasks( void )
{
	xTaskCreate(
                    vColorTask,
		    "Colores",
		    STACK_COLOR,
		    NULL,
		    PRIORIDAD_COLOR,
		    NULL
		   );

	xTaskCreate(
                    vMotoresTask,
		    "Motores",
		    STACK_MOTOR,
		    NULL,
		    PRIORIDAD_MOTOR,
		    NULL
		   );

	xTaskCreate(
                    vTaskCallback,
		    "Callback",
		    STACK_CALLBACK,
		    NULL,
		    PRIORIDAD_CALLBACK,
		    NULL
		   );

	xTaskCreate(
                    vTaskCount,
		    "Callback",
		    STACK_CALLBACK,
		    NULL,
		    PRIORIDAD_CALLBACK,
		    NULL
		   );

        xTaskCreate(
                    vInitColor,
		    "InitColor",
		    STACK_INIT,
		    NULL,
		    PRIORIDAD_INIT,
		    NULL
		   );

        xTaskCreate(
                    vUltrasonicoTask,
                    "HCSR04",
                    STACK_HCSR04,
                    NULL,
                    PRIORIDAD_HCSR04,
                    NULL
                  );

}	// Fin de createTasks()


//////////////////////////////////////////////////////////////////////////////////
// SETUP()
//
void setup()
{
  initTSC();
  initSerial();
  initSemaphore();
  initMotores();
  initUltrasonico();
  
  vSemaphoreCreateBinary( xColorSemaphore );
  
  attachInterrupt(0, vCountHandler, RISING);  
  
  createTasks();
  vTaskStartScheduler();
 
  for( ;; );

}  // Fin de setup()


#endif
