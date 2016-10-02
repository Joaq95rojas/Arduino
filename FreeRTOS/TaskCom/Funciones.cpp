#include <Arduino.h>
#include "Funciones.h"

/////////////////////////////////////////////////////////////////
// FUNCIONES CREADAS

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

} // Fin de _digWrite()


bool _digRead(uint8_t pNum)
{
  if ( IN_PIN(pNum) & BIT_PIN(pNum) ) return HIGH;
  return LOW;

} // Fin de _digRead()

void addToSet( void )
{
  xQueueAddToSet( (QueueSetMemberHandle_t) xComunicacionQueue , xQueueSet );
  xQueueAddToSet( (QueueSetMemberHandle_t) xEmergenciaSemQueue , xQueueSet );

} // Fin de addToSet()


/////////////////////////////////////////////////////////////////
// TAREAS


void vEmergenciaTask( void *pvParameters)
{
  TickType_t xLastWakeTime;

  xSemaphoreTake( xEmergenciaSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xEmergenciaSemaphore , portMAX_DELAY );

    uint8_t botonLeidoAnterior = _digRead(EMERGENCIA);

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_EMERG_BOTON_MS );

    uint8_t botonLeido = _digRead(EMERGENCIA);

    if ( botonLeidoAnterior == botonLeido )
    {
      // ENVIAR SEÑAL APAGADO EMERGENCIA
//      xQueueSend();
      tipoTarea = EMERGENCIA;
      BORRAR_LCD();
      Serial.println("***EMERGENCIA***");
    }
  }

} // Fin de vEmergenciaTask()

void vControlGeneralTask( void *pvParameters )
{
  uint16_t data = 0;
  double tiempoProduccion = 0;
  QueueSetMemberHandle_t xActivatedMember;

  for( ;; )
  {
    Serial.println("Esperando al comm");
    xActivatedMember = xQueueSelectFromSet( xQueueSet , portMAX_DELAY );
    Serial.print("Recibido: ");
    if ( xActivatedMember == xComunicacionQueue )
    {
      
      xQueueReceive(xComunicacionQueue, &data,0);

      Serial.println(data);
    }
    else if( xActivatedMember == xEmergenciaSemQueue )
    {
      Serial.println("EmergQueue");
      xSemaphoreTake( xActivatedMember, 0 );
      Serial.println("EMERGENCIA");
    }
    else Serial.println(" QueueSet ERROR ");
  }

} // Fin de vControlGeneralTask()


void vComunicacionTask( void *pvParameters )
{
  portBASE_TYPE xStatus;
  uint16_t vL  = 20;

  for( ;; )
  {
    vTaskDelay(5000/portTICK_PERIOD_MS);

    xStatus = xQueueSend( xComunicacionQueue, &vL, 0 );

    Serial.print("Dato enviado: "); Serial.println(vL);

    if( xStatus != pdPASS ) Serial.println("QueueSend ERROR");
    vTaskDelay(portMAX_DELAY);
  }

} // Fin de vComunicacionTask()