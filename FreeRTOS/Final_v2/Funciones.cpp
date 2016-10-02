#include <Arduino.h>
#include <ESP8266Client.h>
#include <ESP8266Server.h>
#include <ArduinoJson.h>
#include "Funciones.h"
#include "wifi.h"

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
	XY( 6, HORA_CURSOR );
  if( pHora->minutos < 10 ) lcd << "0";
	lcd << pHora->minutos << ":";
	if( pHora->segundos < 10 ) lcd << "0";
	lcd << pHora->segundos;
	taskEXIT_CRITICAL();

} // Fin de imprimirHora()


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

} // Fin de cambiarLEDs()


void mostrarPiezasLCD( uint16_t contGolpes, uint16_t golpesTotales )
{
  if( tareaEnProceso == false )
  {
    BORRAR_LINEA_LCD( HORA_CURSOR );
    XY( 0, HORA_CURSOR ); lcd << "Piezas: " << contGolpes << " / " << golpesTotales;
  }
  else
  {
    BORRAR_LINEA_LCD( INFO_CURSOR );
    XY( 0, INFO_CURSOR ); lcd << "Piezas: " << contGolpes << " / " << golpesTotales;
  }

} // Fin de mostrarPiezasLCD()


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


bool _digRead(uint8_t pNum)
{
  if ( IN_PIN(pNum) & BIT_PIN(pNum) ) return HIGH;
  return LOW;

} // Fin de _digRead()

void addToSet( void ) // Cant. de colas en el Set = 4
{
//  xQueueAddToSet( (QueueSetMemberHandle_t) xEmergenciaSemQueue, xQueueSet );
  xQueueAddToSet( (QueueSetMemberHandle_t) xComunicacionQueue, xQueueSet );
  xQueueAddToSet( (QueueSetMemberHandle_t) xTiemposQueue, xQueueSet );

} // Fin de addToSet()


void PRENDER_MOTOR(void)
{
  if( motorApagado == true )
  {
    _digWrite(PIN_MOTOR, LOW);
    motorApagado = false;
  }

} // Fin de PRENDER_MOTOR()


void APAGAR_MOTOR(void)
{
  if( motorApagado == false )
  {
    _digWrite(PIN_MOTOR, HIGH);
    motorApagado = true;
    Serial << "Motor apagado" << endl;
  }

} // Fin de APAGAR_MOTOR()


char* getRamString(PGM_P pString)
{
  static char prg_buffer[WIFI_TAMANO_BUFFER_CADENAS] = "";

  return strcpy_P(prg_buffer, pString);

} // Fin de getRamString()


bool empiezaCon(const char *pre, const char *str)
{
    size_t lenpre = strlen(pre), lenstr = strlen(str);
    return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;

} // Fin de empiezaCon()


bool leerCadenaHasta(char* cadena, int tamano, char limite, Stream* stream)
{
  strcpy(cadena, "");
  int bytes_leidos = stream->readBytesUntil(limite, cadena, tamano);
  cadena[bytes_leidos] = '\0';

  return true;

} // Fin de leerCadenaHasta()


/////////////////////////////////////////////////////////////////
// TAREAS

void vHoraTask( void *pvParameters )
{
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
    Serial << t_hora.segundos << endl;
    t_hora.segundos++;
    actualizarHora(&t_hora);

    if( tipoTarea != -1 && tipoTarea != EMERGENCIA )
    {
      switch( tipoTareaAnterior )
      {
        case btnNETAS:
        	if( (tareaEnProceso == true) && (progPausado == false) )  t_prod.segundos++;
                imprimirHora(actualizarHora(&t_prod));
                break;
        case btnSETUP:
                if( progPausado == false) t_setup.segundos++;
                imprimirHora(actualizarHora(&t_setup));
                break;
        case btnREPR:
                if( progPausado == false) t_repr.segundos++;
                imprimirHora(actualizarHora(&t_repr));
                break;
        case btnPARADAxPROC:
        		    if( progPausado == false) t_pxproc.segundos++;
                imprimirHora(actualizarHora(&t_pxproc));
                break;
      }
    }
    if ( tareaEnProceso == false && tipoTarea == -1)
    {
      // Convierte los tiempos y se los manda a vControlGeneralTask()
      tiempoConvertido = CONVERTIR_HORA(t_prod.minutos);
//      xQueueSend( xTiemposQueue, &tiempoConvertido, 0 );
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
    {
      xStatus = xQueueSend(xButtonQueue,&tareaLeida,(TickType_t)0);
      Serial << "Boton Leido: " << tareaLeida << endl;
    }

  }
  if( xStatus != pdPASS )
    {
      PRINT_ERROR("xQueueSend ERROR");
      Serial << "ERROR leyendo botones" << endl;
    }

} // Fin de vBotonesTask()


void vTaskAssign( void *pvParameters )
{
  int8_t qTipoTarea;

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
taskENTER_CRITICAL();
          APAGAR_MOTOR();
          BORRAR_LINEA_LCD( TASK_CURSOR );
          XY( 0, TASK_CURSOR );  lcd << "Prog PAUSADO   ";
          Serial << "Pausado" << endl;
          XY( 0, INSTR_CURSOR );  lcd << "Seleccione tarea";
          DEINIT_BALANCIN();
taskEXIT_CRITICAL();
        }
        else
        {
          progPausado = false;
          tipoTarea = tipoTareaAnterior;
          PRENDER_LED(ledPAUSA);
          taskENTER_CRITICAL();
          BORRAR_LCD();
          XY( 0, TASK_CURSOR );
          switch( tipoTarea )
          {
            case btnNETAS:        lcd << "Produccion";
                                  PRENDER_MOTOR();
                                  if( initBalancin == false ) INIT_BALANCIN();
                                  break;
            case btnSETUP:        lcd << "Setup";
                                  PRENDER_MOTOR();
                                  break;
            case btnREPR:         lcd << "Reproceso";
                                  PRENDER_MOTOR();
                                  break;
            case btnPARADAxPROC:  lcd << "PxProceso";
                                  APAGAR_MOTOR();
                                  break;
          }
          XY( 0, HORA_CURSOR ); lcd << "Hora: ";
          taskEXIT_CRITICAL();
        }
      }
      else if ( progPausado == true )
      {
        if( qTipoTarea != btnPAUSA )
        {
          tipoTareaAnterior = qTipoTarea;
          DEINIT_BALANCIN();
          BORRAR_LINEA_LCD( TASK_CURSOR );
          XY( 0, TASK_CURSOR );
          switch( qTipoTarea )
          {
            case btnNETAS:        lcd << "Produccion";
                                  break;
            case btnSETUP:        lcd << "Setup";
                                  break;
            case btnREPR:         lcd << "Reproceso";
                                  break;
            case btnPARADAxPROC:  lcd << "PxProceso";
                                  break;
          }
        }
        APAGAR_MOTOR();
        cambiarLEDs(qTipoTarea);
        BORRAR_LINEA_LCD( INSTR_CURSOR );
        XY( 0, INSTR_CURSOR );  lcd << "Presione INICIO";
      }
    }
  }

} // Fin de vTaskAssign()


void vgolpesBalancinTask( void *pvParameters )
{
  portBASE_TYPE xStatus;
  TickType_t xLastWakeTime;
  uint16_t contGolpes = 0, golpesBalancinTotal = 0;

  xSemaphoreTake( xSensorSemaphore , 0 );

  for( ;; )
  {
    //Serial << "BALANCIN" << endl;
    // Espera indefinidamente por el la cantidad de golpes a efectuar.
    while( golpesBalancinTotal <= 0 )
    {
      xQueueReceive( xGolpesQueue, &golpesBalancinTotal, portMAX_DELAY );
    }
    Serial << "GB: " << golpesBalancinTotal << endl;

    if( golpesBalancinTotal <= 0 ) { PRINT_ERROR(" ERROR BALANCIN ");}
    else
    {
      // Se "despierta" con cada golpe de la máquina
      xSemaphoreTake( xSensorSemaphore , portMAX_DELAY );
      uint8_t sensorLeidoAnterior = LEER_SENSOR();
      Serial << "S1: " << sensorLeidoAnterior << endl;

      xLastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(  &xLastWakeTime, TIEMPO_RUIDO_SENSOR_MS );

      uint8_t sensorLeido = LEER_SENSOR();
      Serial << "S2: " << sensorLeido << endl;

      if ( (sensorLeidoAnterior == HIGH) && (sensorLeido == HIGH) )
      {
      if( contGolpes < golpesBalancinTotal )
      {
        tareaEnProceso = true;
        contGolpes++;
        mostrarPiezasLCD( contGolpes, golpesBalancinTotal );
      }

        if( contGolpes >= golpesBalancinTotal )
        {
taskENTER_CRITICAL();
          APAGAR_MOTOR();
          DEINIT_BALANCIN();
          DEINIT_BOTONES();
          tareaEnProceso = false;
          progPausado = true;
          tipoTarea = -1;
          BORRAR_LCD();
          XY( 0, INFO_CURSOR );  lcd << "Tarea finalizada";
          mostrarPiezasLCD( contGolpes, golpesBalancinTotal );
taskEXIT_CRITICAL();
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

} // Fin de vgolpesBalancinTask()


void vEmergenciaTask( void *pvParameters)
{
  TickType_t xLastWakeTime;
  bool avisoEmerg = false;

  xSemaphoreTake( xEmergenciaSemaphore , 0 );

  for( ;; )
  {
    xSemaphoreTake( xEmergenciaSemaphore , portMAX_DELAY );

    uint8_t botonLeidoAnterior = _digRead(EMERGENCIA);

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(  &xLastWakeTime, TIEMPO_EMERG_BOTON_MS );

    uint8_t botonLeido = _digRead(EMERGENCIA);

    if ( botonLeidoAnterior == botonLeido && (botonLeido == HIGH) )
    {
      taskENTER_CRITICAL();
      APAGAR_MOTOR();
      DEINIT_BOTONES();
      tipoTarea = EMERGENCIA;
      avisoEmerg = true;
      BORRAR_LCD();
      PRINT_ERROR("***EMERGENCIA***");
      // Envía señal de apagado de emergencia
      xQueueSend( xEmergenciaSemQueue, &avisoEmerg, 0 );
      taskEXIT_CRITICAL();
    }
  }

} // Fin de vEmergenciaTask()

void vControlGeneralTask( void *pvParameters )
{
  uint16_t golpesBalancinTotal = 0;
  double tiempoProduccion = 0;
  bool avisoEmerg = false;
  QueueSetMemberHandle_t xActivatedMember;

  // La tarea se queda esperando por alguno de los dos eventos:
  //  - Botón de emergencia( vEmergenciaTask() )
  //  - Comunicación del Servidor ( vComunicacionTask() )
  XY( 0, INFO_CURSOR );  lcd << "Esperando Servidor";
  //Serial << "Esperando Servidor" << endl;

  for( ;; )
  {
    // Se queda bloqueada hasta que aparezca un dato en el Set
    xActivatedMember = xQueueSelectFromSet( xQueueSet , portMAX_DELAY );
    // Ante una llegada, es necesario indentificar de quién lo recibió.

    if(xActivatedMember)
    {
      BORRAR_LINEA_LCD( INFO_CURSOR );
      XY( 0, INFO_CURSOR );  lcd << "Mensaje recibido";
      Serial << "Mensaje Recibido" << endl;
    }

    if ( xActivatedMember == xComunicacionQueue )
    {
      // Recibe la cantidad de golpes a efectuar por el balancín
      // de vComunicacionTask().
      xQueueReceive( xComunicacionQueue, &golpesBalancinTotal, 0 );
      // Se habilita vInterruptHandler().
      if( golpesBalancinTotal >= 0 )
      {
        BORRAR_LINEA_LCD( INFO_CURSOR );
        XY( 0, INFO_CURSOR );  lcd << "Golpes: " << golpesBalancinTotal;

        Serial << "Golpes: " << golpesBalancinTotal << endl;
        INIT_BOTONES();
        // Se le notifica a vgolpesBalancinTask() el valor.
        xQueueSend( xGolpesQueue, &golpesBalancinTotal, 0 );
        XY( 0, INSTR_CURSOR );  lcd << "Seleccione tarea";
      }
    }
    else if( xActivatedMember == xEmergenciaSemQueue )
    {
      // Recibo notificación de emergencia.
      xQueueReceive( xEmergenciaSemQueue, &avisoEmerg, 0 );
      Serial << "AVISO EMERG: " << avisoEmerg << endl;
      // El motor fue apagado por vEmergenciaTask().
      // INHABILITAR TODOS LOS CONTROLES HASTA QUE SE RECIBA UNA
      // ORDEN DE ACTIVACIÓN DEL SERVIDOR CENTRAL.
      if( avisoEmerg == true )
      {
//taskENTER_CRITICAL();

//taskEXIT_CRITICAL();
      }
    }
    else if( xActivatedMember == xTiemposQueue )
    {
      Serial << "xTiemposQueue" << endl;
      // Se ha finalizado la tarea de "Producción".
      // vControlGeneralTask() debe recibir todos los datos del
      // proceso para enviarlos a vComunicacionTask().
      xQueueReceive( xTiemposQueue, &tiempoProduccion, 0);
      Serial << "Enviar Tiempos a vComunicacionTask() para el servidor." << endl;
//      xQueueSend( xComunicacionQueue, &tiempoProduccion, 0);
    }
  }

} // Fin de vControlGeneralTask()


void vComunicacionTask( void *pvParameters )
{
  portBASE_TYPE xStatus;
  uint16_t golpesBalancinTotal  = 0;
  static ESP8266Client* cliente;

  Serial << "[verificando1]" << endl; // Evita que el programa se tilde de manera improvista.

  for( ;; )
  {
    //Serial << "[verificando2]" << endl;
    //taskENTER_CRITICAL();
    if ( (cliente = wifi.clientesDisponibles()) && cliente->available() )
    {
      Serial << "[cliente-disponible]" << endl;

      BORRAR_LINEA_LCD(INFO_CURSOR);
      XY(0,INFO_CURSOR); lcd << "Wifi Conectado";

      if ( wifi.obtenerDatosRespuestaHTTP(cliente, &respuesta) ) {
         StaticJsonBuffer<WIFI_TAMANO_BUFFER_JSON> jsonBuffer;
         JsonObject& json = jsonBuffer.parseObject(respuesta.datos);
         golpesBalancinTotal=json["datos"]["cantidad_fabricar"];
         Serial.print(F("cantidad="));
         Serial.println(golpesBalancinTotal);

         if( golpesBalancinTotal <= 0) { PRINT_ERROR("ERROR DE COMUNIC");}
    else
    {
      // Manda ACK.

      // Debe avisar a vControlGeneralTask() usando una cola de mensajes.
      xStatus = xQueueSend( xComunicacionQueue, &golpesBalancinTotal, 0 );

      //for( ;; );
    }
       } else {
        Serial.println(F("no se pudo interpretar"));
       }
    }
    //taskEXIT_CRITICAL();

    //for(;;);

    //vTaskDelay( 5000/portTICK_PERIOD_MS );  // Tiempo de espera de prueba


  }

} // Fin de vComunicacionTask()
