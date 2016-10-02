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

} // Fin de actualizarHora()


void imprimirHora(const struct T* pHora)
{
	taskENTER_CRITICAL();
	lcd.setCursor(6,1);
	if( pHora->minutos < 10 ) lcd.print("0");    // "0"
	lcd.print( pHora->minutos ); lcd.print(":"); // ":"
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


void mostrarPiezasLCD( uint16_t contGolpes, uint16_t golpesTotales )
{
  BORRAR_LINEA_LCD(0);
	lcd.setCursor(0,0);
  lcd.print("Piezas: "); // "Piezas: "
  lcd.print(contGolpes);
  lcd.print(" / "); // " / "
  lcd.print(golpesTotales);
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

} // Fin de _digWrite()


bool _digRead(uint8_t pNum)
{
  if ( IN_PIN(pNum) & BIT_PIN(pNum) ) return HIGH;
  return LOW;

} // Fin de _digRead()

void addToSet( void ) // Cant. de colas en el Set = 4
{
  xQueueAddToSet( (QueueSetMemberHandle_t) xEmergenciaSemQueue, xQueueSet );
  xQueueAddToSet( (QueueSetMemberHandle_t) xComunicacionQueue, xQueueSet );
  xQueueAddToSet( (QueueSetMemberHandle_t) xTiemposQueue, xQueueSet );
  xQueueAddToSet( (QueueSetMemberHandle_t) xGolpesQueue, xQueueSet );

} // Fin de addToSet()

/*
char* getMsgString(int16_t i) 
{
  ////////////////////////////////////////////////////
  // Lee un mensaje desde la memoria del programa
  //
  // @param: indice Número de mensaje
  //
  // @return: Puntero al buffer que contiene la cadena
  //
  #ifdef __LCD_SHIELD__
    static char prg_buffer[16] = "";
  #else
    static char prg_buffer[20] = "";
  #endif
  // "prg_buffer[]" se usa para guardar cadenas que se leen desde
  // la memoria del programa.

  const char* const msg_list[] PROGMEM = {
    msg0 , msg1 , msg2 , msg3 , msg4 , msg5 , msg6 , msg7 ,
    msg8 , msg9 , msg10, msg11, msg12, msg13, msg14, msg15,
    msg16, msg17, msg18, msg19, msg20, msg21, msg22
  };

  return strcpy_P(prg_buffer, (char*)pgm_read_word(&(msg_list[i])));

} // Fin de getMsgString()*/

/////////////////////////////////////////////////////////////////
// TAREAS

void vHoraTask( void *pvParameters )
{/*
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  double tiempoConvertido;
  struct T t_prod, t_setup, t_pxproc, t_repr, t_pausa, t_hora;

  INIT_VARIABLES(t_prod);
  INIT_VARIABLES(t_setup);
  INIT_VARIABLES(t_pxproc);
  INIT_VARIABLES(t_repr);
  INIT_VARIABLES(t_pausa);
  INIT_VARIABLES(t_hora);
  
  for( ;; )
  {
    Serial.println(t_hora.segundos);
    t_hora.segundos++;
    actualizarHora(&t_hora);

    if( progPausado == false)
    {
      switch( tipoTarea )
      {
        case btnNETAS:        
        	if( tareaEnProceso == true ) 
                t_prod.segundos++;
                imprimirHora(actualizarHora(&t_prod));
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
    if ( tareaEnProceso == false && tipoTarea == -1)
    {
      // Convierte los tiempos y se los manda a vControlGeneralTask()
      tiempoConvertido = CONVERTIR_HORA(t_prod.minutos);
      xQueueSend( xTiemposQueue, &tiempoConvertido, 0 );
    }

    vTaskDelayUntil( &xLastWakeTime, PERIODO_INT_MS );
  }*/

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
  // PRINT_ERROR(getMsgString(9)); // "xQueueSend ERROR"

} // Fin de vBotonesTask()


void vTaskAssign( void *pvParameters )
{
  int8_t qTipoTarea, tipoTareaAnterior = 0;

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
//          lcd.print(getMsgString(10));  // "   PAUSADO   "
          lcd.print("   PAUSADO   ");
          lcd.setCursor(0,1);
//          lcd.print(getMsgString(1));
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
            case btnNETAS:        //lcd.print(getMsgString(11));  // "Produccion"
                                  lcd.print("Produccion");
                                  if( initBalancin == false ) INIT_BALANCIN();
                                  break;
            case btnSETUP:        //lcd.print(getMsgString(12));  // "Setup"
                                  lcd.print("Setup");
                                  break;
            case btnREPR:         //lcd.print(getMsgString(13));  // "Reproceso"
                                  lcd.print("Reproceso");
                                  break;
            case btnPARADAxPROC:  //lcd.print(getMsgString(14));  // "PxProceso"
                                  lcd.print("PxProceso");
                                  break;
          }
          lcd.setCursor(0,1);
//          lcd.print(getMsgString(15));  // "Hora: "
          lcd.print("Hora: ");
        }
      }
      else if ( progPausado == true )
      {
        if( qTipoTarea != btnPAUSA ) tipoTareaAnterior = qTipoTarea;
        cambiarLEDs(qTipoTarea);
        BORRAR_LINEA_LCD(1);
        lcd.setCursor(0,1);
//        lcd.print(getMsgString(16));  // "Presione INICIO"
        lcd.print("Presione INICIO");
      }
    }
  }    

} // Fin de vTaskAssign()


void vgolpesBalancinTask( void *pvParameters )
{/*
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;
  uint16_t contGolpes = 0, golpesBalancinTotal = 0;

  xSemaphoreTake( xSensorSemaphore , 0 );

  for( ;; )
  {
    // Espera indefinidamente por el la cantidad de golpes a efectuar.
    xQueueReceive( xGolpesQueue, &golpesBalancinTotal, portMAX_DELAY );
 
    if( golpesBalancinTotal <= 0 ) { PRINT_ERROR(" ERROR BALANCIN ");}
      // PRINT_ERROR(getMsgString(17)); 
    else
    {
      // Se "despierta" con cada golpe de la máquina
      xSemaphoreTake( xSensorSemaphore , portMAX_DELAY );

      uint8_t sensorLeidoAnterior = LEER_SENSOR();

      xLastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_SENSOR_MS );

      uint8_t sensorLeido = LEER_SENSOR();

      if ( (sensorLeidoAnterior == HIGH) && (sensorLeido == HIGH) )
      {
      if( contGolpes < golpesBalancinTotal )
      {
        tareaEnProceso = true;
        mostrarPiezasLCD( contGolpes++, golpesBalancinTotal );
      }
        
        if( contGolpes >= golpesBalancinTotal )
        {
          APAGAR_MOTOR();
          BORRAR_LINEA_LCD(0);
          lcd.setCursor(0,0);
//          lcd.print(getMsgString(18));  // "Tarea finalizada"
          lcd.print("Tarea finalizada");
          tareaEnProceso = false;
          progPausado = true;
          tipoTarea = -1;
          DEINIT_BALANCIN();
          //
          // DESHABILITAR BOTONES (LO TIENE QUE HACER vControlGeneralTask()!!)
          // Y ENVIAR INFORME.
          // 
           golpesBalancinTotal = 0; 
           // Reinicia el contador a la espera de una nueva asignación
        }
      }
    }
  }
  if( xStatus != pdPASS ) PRINT_ERROR("xQueueSend ERROR");
  // PRINT_ERROR(getMsgString(9));
*/
} // Fin de vgolpesBalancinTask()


void vEmergenciaTask( void *pvParameters)
{/*
  TickType_t xLastWakeTime;

  xSemaphoreTake( xEmergenciaSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xEmergenciaSemaphore , portMAX_DELAY );
//    PRINT_ERROR(getMsgString(20)); // "EMERGENCIA"
    PRINT_ERROR("EMERGENCIA");

    uint8_t botonLeidoAnterior = _digRead(EMERGENCIA);

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_EMERG_BOTON_MS );

    uint8_t botonLeido = _digRead(EMERGENCIA);

    if ( botonLeidoAnterior == botonLeido )
    {
      // No se comprueba ruido ni se usa una cola de mensajes
      APAGAR_MOTOR();
      // ENVIAR SEÑAL APAGADO EMERGENCIA
//      xQueueSend();
      tipoTarea = EMERGENCIA;
      BORRAR_LCD();
//      PRINT_ERROR(getMsgString(20));  // "***EMERGENCIA***"
      PRINT_ERROR("***EMERGENCIA***");
    }
  }*/

} // Fin de vEmergenciaTask()

void vControlGeneralTask( void *pvParameters )
{
  uint16_t golpesBalancinTotal = 0;
  double tiempoProduccion = 0;
  QueueSetMemberHandle_t xActivatedMember;

  // La tarea se queda esperando por alguno de los dos eventos:
  //  - Botón de emergencia( vEmergenciaTask() ) 
  //  - Comunicación del Servidor ( vComunicacionTask() )
  lcd.setCursor(0,1);
  lcd.print("Esperando Server");
  
  for( ;; )
  {
    // Se queda bloqueada hasta que aparezca un dato en el Set
    xActivatedMember = xQueueSelectFromSet( xQueueSet , portMAX_DELAY );
    // Ante una llegada, es necesario indentificar de quién lo recibió.
    BORRAR_LINEA_LCD(1);
    lcd.setCursor(0,1);
    lcd.print("Mensaje Recibido");

    if ( xActivatedMember == xComunicacionQueue )
    {
      // Recibe la cantidad de golpes a efectuar por el balancín
      // de vComunicacionTask().
      xQueueReceive( xComunicacionQueue, &golpesBalancinTotal, 0 );
      // Se habilita vInterruptHandler().
      if( golpesBalancinTotal >= 0 )
      {
        BORRAR_LINEA_LCD(0);
        lcd.setCursor(0,0);
        lcd.print("Golpes: ");  lcd.print(golpesBalancinTotal);
        INIT_BOTONES();
        // Se le notifica a vgolpesBalancinTask() el valor.
        xQueueSend( xGolpesQueue, &golpesBalancinTotal, 0 );
      }
      else  //PRINT_ERROR(getMsgString(0)); // "xQueueRec ERROR "
        PRINT_ERROR("xQueueRec ERROR");
    }
    else if( xActivatedMember == xEmergenciaSemQueue )
    {
      // Recibo notificación de emergencia.
      xSemaphoreTake( xEmergenciaSemQueue, 0 );
      // El motor fue apagado por vEmergenciaTask().
      // INHABILITAR TODOS LOS CONTROLES HASTA QUE SE RECIBA UNA
      // ORDEN DE ACTIVACIÓN DEL SERVIDOR CENTRAL.
      DEINIT_BOTONES();
    }
    else if( xActivatedMember == xTiemposQueue )
    {
      // Se ha finalizado la tarea de "Producción".
      // vControlGeneralTask() debe recibir todos los datos del
      // proceso para enviarlos a vComunicacionTask().
      xQueueReceive( xTiemposQueue, &tiempoProduccion, 0);
      xQueueSend( xComunicacionQueue, &tiempoProduccion, 0);
    }
    else  PRINT_ERROR(" QueueSet ERROR ");
    //PRINT_ERROR(getMsgString(21));  // " QueueSet ERROR "
  }

} // Fin de vControlGeneralTask()


void vComunicacionTask( void *pvParameters )
{
  portBASE_TYPE xStatus;
  uint16_t golpesBalancinTotal  = 25;

  for( ;; )
  {
    vTaskDelay( 5000/portTICK_PERIOD_MS );  // Tiempo de espera de prueba

    if( golpesBalancinTotal <= 0) { PRINT_ERROR("ERROR DE COMUNIC");}
      // PRINT_ERROR(getMsgString(22)); 
    else
    {
      // Manda ACK.
//      Serial.write(golpesBalancinTotal+1);

      // Debe avisar a vControlGeneralTask() usando una cola de mensajes.
      xStatus = xQueueSend( xComunicacionQueue, &golpesBalancinTotal, 0 );
      if( xStatus != pdPASS ) PRINT_ERROR("QueueSend ERROR");
      // PRINT_ERROR(getMsgString(9)); // "QueueSend ERROR"

      vTaskDelay( portMAX_DELAY );
    }
  }

} // Fin de vComunicacionTask()