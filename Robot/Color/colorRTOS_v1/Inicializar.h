#ifndef _INICIALIZAR_H_
#define _INICIALIZAR_H_

#include "Funciones.h"

//////////////////////////////////////////////////////////////////////////////////
// INICIALIZACION
//
/*
void TSC_Init(void)
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

}  // Fin de TSC_Init()*/


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
                    vTaskCallback,
		    "Callback",
		    STACK_CALLBACK,
		    NULL,
		    PRIORIDAD_CALLBACK,
		    NULL
		   );

	xTaskCreate(
                    vTaskCount,
		    "Count",
		    STACK_COUNT,
		    NULL,
		    PRIORIDAD_COUNT,
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

}	// Fin de createTasks()


//////////////////////////////////////////////////////////////////////////////////
// SETUP()
//
void setup()
{
//  TSC_Init();
  /////////////////////////////////////////////////////
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
  //////////////////////////////////////////////////////

  Serial.begin(9600);  Serial << "* INICIADO *" << endl;
  
  vSemaphoreCreateBinary( xColorSemaphore );
  
  attachInterrupt(0, vCountHandler, RISING);  
  
  createTasks();
  vTaskStartScheduler();
 
  for( ;; );

}  // Fin de setup()


#endif
