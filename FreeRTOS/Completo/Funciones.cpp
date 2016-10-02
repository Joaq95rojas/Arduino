#include <Arduino.h>
#include "Funciones.h"

/////////////////////////////////////////////////////////////////
// FUNCIONES CREADAS

const struct T *actualizarHora (struct T* pHora)
{
  if( pHora->segundos >= 60 )  
  { 
      pHora->segundos = 0; 
      pHora->minutos++; 
  }
  return pHora;
}


void initVariables( void )
{
  t_neto.segundos = 0;
  t_neto.minutos = 0;
  t_setup.segundos = 0;
  t_setup.minutos = 0;
  t_pxproc.segundos = 0;
  t_pxproc.minutos = 0;
  t_repr.segundos = 0;
  t_repr.minutos = 0;
  t_pausa.segundos = 0;
  t_pausa.minutos = 0;
  t_hora.segundos = 0;
  t_hora.minutos = 0;
}


void imprimirHora(const struct T* pHora)
{
	taskENTER_CRITICAL();
	lcd.setCursor(6,1);
	if( pHora->minutos < 10 ) lcd.print("0");
	lcd.print( pHora->minutos ); lcd.print(":");
	if( pHora->segundos < 10 ) lcd.print("0");
	lcd.print( pHora->segundos );
	taskEXIT_CRITICAL();
}


uint8_t leerBotones(void)
{
  for (;;)
  {
    // Una vez ocurrida la interrupción lee qué botón la generó.
    if(_digRead(btnNETAS))       return (uint8_t)btnNETAS;
    if(_digRead(btnPARADAxPROC)) return (uint8_t)btnPARADAxPROC;
    if(_digRead(btnSETUP))       return (uint8_t)btnSETUP;
    if(_digRead(btnREPR))        return (uint8_t)btnREPR;
    if(_digRead(btnPAUSA))       return (uint8_t)btnPAUSA;

    vTaskDelay(20);
  }
  return 0;   // Nunca debería llegar acá
}


void onExcLED(uint8_t tareaLED)
{
  switch( tareaLED )
  {
    case ledNETAS:        PRENDER_LED(ledNETAS);
                          APAGAR_LED(ledSETUP);APAGAR_LED(ledREPR);
                          APAGAR_LED(ledPARADAxPROC);APAGAR_LED(ledPAUSA);
                          break;
    case ledSETUP:        PRENDER_LED(ledSETUP);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledREPR);
                          APAGAR_LED(ledPARADAxPROC);APAGAR_LED(ledPAUSA);
                          break;
    case ledREPR:         PRENDER_LED(ledREPR);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledSETUP);
                          APAGAR_LED(ledPARADAxPROC);APAGAR_LED(ledPAUSA);
                          break;
    case ledPARADAxPROC:  PRENDER_LED(ledPARADAxPROC);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledSETUP);
                          APAGAR_LED(ledREPR);APAGAR_LED(ledPAUSA);
                          break;
    case ledPAUSA:        PRENDER_LED(ledPAUSA);
                          APAGAR_LED(ledNETAS);APAGAR_LED(ledSETUP);
                          APAGAR_LED(ledREPR);APAGAR_LED(ledPARADAxPROC);
                          break;
  }
}	// Fin de onExcLED()


void cambiarLEDs(uint8_t tareaLED)
{
  switch( tareaLED )
  {
    case btnNETAS:        onExcLED(ledNETAS);
                          break;
    case btnSETUP:        onExcLED(ledSETUP);
                          break;
    case btnREPR:         onExcLED(ledREPR);
                          break;
    case btnPARADAxPROC:  onExcLED(ledPARADAxPROC);
                          break;
    case btnPAUSA:        onExcLED(ledPAUSA);
                          break;
  }
}


void mostrarPiezasLCD( void )
{
  BORRAR_LINEA_LCD(0);
	lcd.setCursor(0,0);
  lcd.print("Piezas: "); 
  lcd.print(contGolpes);
  lcd.print(" / ");
  lcd.print(golpesBalancinTotal);
}


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
}

void _digWrite( uint8_t pNum, uint8_t statePin )
{
  volatile uint8_t* out = OUT_PIN(pNum);

  if( statePin )  *out |= BIT_PIN(pNum);
  else            *out &= ~BIT_PIN(pNum);
}


boolean _digRead(uint8_t pNum)
{
  if ( IN_PIN(pNum) & BIT_PIN(pNum) ) return HIGH;
  return LOW;
}

/////////////////////////////////////////////////////////////////
// TAREAS

void vHoraTask( void *pvParameters )
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    t_hora.segundos++;
    actualizarHora(&t_hora);

    if( progPausado == false)
    {
      switch( tipoTarea )
      {
        case btnNETAS:        
        	if( ( contGolpes >= 1 ) && ( contGolpes < golpesBalancinTotal ) ) 
                t_neto.segundos++;
                imprimirHora(actualizarHora(&t_neto));
                break;
        case btnSETUP:
                t_setup.segundos++;
                imprimirHora(actualizarHora(&t_setup));
                break;
        case btnREPR:
                t_repr.segundos++;
                imprimirHora(actualizarHora(&t_setup));
                break;
        case btnPARADAxPROC:
        		    t_pxproc.segundos++;
                imprimirHora(actualizarHora(&t_pxproc));
                break;
      }
    }
    vTaskDelayUntil( &xLastWakeTime, PERIODO_INT_MS );
  }

}	// Fin de vHoraTask()


void vBotonesTask( void *pvParameters)
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;

  xSemaphoreTake( xButtonSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xButtonSemaphore , portMAX_DELAY );

    uint8_t tareaLeidaAnterior = leerBotones();

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_BOTON_MS );

    uint8_t tareaLeida = leerBotones();

    if ( tareaLeidaAnterior == tareaLeida && (tareaLeida != 0) )
        xStatus = xQueueSend(xButtonQueue,&tareaLeida,(TickType_t)0);
  }
  if( xStatus != pdPASS )	PRINT_ERROR("xQueueSend ERROR");

} // Fin de vBotonesTask()


void vTaskAssign( void *pvParameters )
{
  uint8_t qTipoTarea;

  if( xButtonQueue != 0 )   // Se ejecuta si la cola fue creada 
  {                         // correctamente
    for ( ;; )
    {
      if ( xQueueReceive(xButtonQueue,&qTipoTarea,portMAX_DELAY) )
      {
        if ( qTipoTarea == btnPAUSA )
        {
          if( progPausado == false )
          {
            progPausado = true;
            tipoTareaAnterior = tipoTarea;
            BORRAR_LINEA_LCD(0);
            lcd.setCursor(0,0);
            lcd.print("PAUSADO");
            lcd.setCursor(0,1);
            lcd.print("Seleccione tarea");
          }
          else
          {
            progPausado = false;
            tipoTarea = tipoTareaAnterior;
            PRENDER_LED(ledPAUSA);
            BORRAR_LCD();
            lcd.setCursor(0,0);
            switch( tipoTarea )
            {
              case btnNETAS:        lcd.print("Produccion");
                                    if( initBalancin == false ) INIT_BALANCIN();
                                    break;
              case btnSETUP:        lcd.print("Setup");
                                    break;
              case btnREPR:         lcd.print("Reproceso");
                                    break;
              case btnPARADAxPROC:  lcd.print("PxProceso");
                                    break;
            }
            lcd.setCursor(0,1);
            lcd.print("Hora: ");
          }
        }
        else if ( progPausado == true )
        {
          if( qTipoTarea != btnPAUSA ) tipoTareaAnterior = qTipoTarea;
          cambiarLEDs(qTipoTarea);
          BORRAR_LINEA_LCD(1);
          lcd.setCursor(0,1);
          lcd.print("Presione INICIO");
        }
      }
    }    
  }

} // Fin de vTaskAssign()


void vgolpesBalancinTask( void *pvParameters )
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;

  xSemaphoreTake( xSensorSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xSensorSemaphore , portMAX_DELAY );

    uint8_t sensorLeidoAnterior = LEER_SENSOR();

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_SENSOR_MS );

    uint8_t sensorLeido = LEER_SENSOR();

    if ( (sensorLeidoAnterior == HIGH) && (sensorLeido == HIGH) )
    {
	  if( contGolpes < golpesBalancinTotal )
      {
        contGolpes++;
        mostrarPiezasLCD();
      }
      if( contGolpes >= golpesBalancinTotal )
      {
        APAGAR_MOTOR();
        BORRAR_LINEA_LCD(0);
        lcd.setCursor(0,0);
        lcd.print("Tarea finalizada");
        progPausado = true;
        tipoTarea = 0;
        DEINIT_BALANCIN();
        /*
         * DESHABILITAR BOTONES Y ENVIAR INFORME
         */
        t_neto.tiempoConvertido = CONVERTIR_HORA(t_neto);
        lcd.setCursor(0,2);
        lcd.print("Tiempo Conv: "); lcd.print(t_neto.tiempoConvertido);
      }
    }
  }
  if( xStatus != pdPASS )
  {
    PRINT_ERROR("xQueueSend ERROR");
  }

} // Fin de vgolpesBalancinTask()


void vEmergenciaTask( void *pvParameters)
{
  TickType_t xLastWakeTime;

  xSemaphoreTake( xEmergenciaSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xEmergenciaSemaphore , portMAX_DELAY );
    Serial.println("EMERGENCIA");

    uint8_t botonLeidoAnterior = _digRead(EMERGENCIA);

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_EMERG_BOTON_MS );

    uint8_t botonLeido = _digRead(EMERGENCIA);

    if ( botonLeidoAnterior == botonLeido )
    {
      // No se comprueba ruido ni se usa una cola de mensajes
      APAGAR_MOTOR();
      // ENVIAR SEÑAL APAGADO EMERGENCIA
      tipoTarea = EMERGENCIA;
      BORRAR_LCD();
      PRINT_ERROR("***EMERGENCIA***");
    }
  }

} // Fin de vEmergenciaTask()