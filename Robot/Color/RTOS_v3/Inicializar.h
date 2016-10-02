#ifndef _INICIALIZAR_H_
#define _INICIALIZAR_H_

#include "Funciones.h"

//////////////////////////////////////////////////////////////////////////////////
// INICIALIZACION
//

inline void initSerial(void) { Serial.begin(57600);  Serial << "* INICIADO *" << endl; }


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


void initUltrasonico(void) 
{
  ENTRADA(DDRG,PG5);    // configPin(PIN_ECHO, ENTRADA);
  SALIDA(DDRE,PE3);     // configPin(PIN_TRIG, SALIDA);
  
  DDRE |= 1<<PE5;    // Set OC3 as output (pin PE5)
  // Timer Counter Control Register (TCCR3 - Timer 3)
  TCCR3A |= (1<<WGM30)|(1<<WGM31)|(1<<COM3C1);
  TCCR3B |= (1<<CS30);
  OCR3C = 200;
        
}  // Fin de initUltrasonico()


void createTasks( void )
{
	xTaskCreate(
                    vColorTask,
		    "Colores",
		    STACK_COLOR,
		    NULL,
		    PRIORIDAD_COLOR,
		    &xColorHandle
		   );

	xTaskCreate(
                    vCallbackTask,
		    "Callback",
		    STACK_CALLBACK,
		    NULL,
		    PRIORIDAD_CALLBACK,
		    &xCallbackHandle
		   );

	xTaskCreate(
                    vColorCountTask,
		    "Callback",
		    STACK_COUNT,
		    NULL,
		    PRIORIDAD_COUNT,
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

}	// Fin de createTasks()


void initAll(void)
{
  initTSC();
  initSerial();
  xInitRTOS();
  initMotores();
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
  vTaskSuspend(xHCSR04Handle);
  vTaskStartScheduler();
 
  for( ;; );

}  // Fin de setup()


#endif
