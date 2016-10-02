#ifndef __TIMERLIB_H__
#define __TIMERLIB_H__

#include <AVRFreeRTOS.h>
#include <utility/task.h>
#include <utility/timers.h>

// Usar uint16_t alcanza para 18 horas de trabajo.
uint16_t tProd, tSetup, tPxP, tReproc;

extern SemaphoreHandle_t xMutexTiempos;
extern TaskHandle_t xControlHandle;
extern TaskHandle_t xComunicacionHandle;
static TimerHandle_t xHoraHandle;
static TimerHandle_t xInfoHandle;

extern uint8_t TipoTarea;

inline void TimerON (void){ xTimerStart(xHoraHandle,0); }
inline void TimerOFF(void){ xTimerStop( xHoraHandle,0); }


void vHoraTimer(TimerHandle_t xTimer)
{
	_bln_debug_print(F(">hora-task=")); _bln_debug_println(uxTaskGetStackHighWaterMark(NULL));
  xTaskNotify(xControlHandle, FLAG_CONTROL_HORA, eSetBits);
}

void vInfoTimer(TimerHandle_t xInfo)
{
  xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_ENVIAR_INFO, eSetBits);
}

void imprimirHora(UTFT& myGLCD, bool bSupervisor)
{
  // PARA QUE NO HAYA PROBLEMAS, PRIMERO DEBO OBTENER LOS COLORES QUE SE
  // ESTAN UTILIZANDO, Y DESPUES, LOS RETORNO A SU VALOR ORIGINAL.

    char Tiempo[9];

    uint8_t Color = myGLCD.getColor();
    uint8_t BackColor = myGLCD.getBackColor();

    bSupervisor? myGLCD.setColor(WINDOWS_9) : myGLCD.setColor(BLANCO_2);
    myGLCD.setBackColor(COLOR_FONDO_PANTALLA);

    // (TIEMPO  )(10)HH:MM:SS

    xSemaphoreTake(xMutexTiempos, portMAX_DELAY);

    if( TipoTarea == PRODUCCION ) {
        uint16_t X = tProd % 3600;
        snprintf(Tiempo,sizeof Tiempo, "%02u:%02hu:%02hu", (uint16_t)(tProd/3600), (uint8_t)(X/60), (uint8_t)(X % 60));
        myGLCD.print(Tiempo, (myGLCD.getFontXsize()+1)*10, Y_TIEMPO);
    }
    else if( TipoTarea == SETUP ) {
        uint16_t X = tSetup % 3600;
        snprintf(Tiempo,sizeof Tiempo, "%02u:%02hu:%02hu", (uint16_t)(tSetup/3600), (uint8_t)(X/60), (uint8_t)(X % 60));
        myGLCD.print(Tiempo, (myGLCD.getFontXsize()+1)*10, Y_TIEMPO);
    }
    else if( TipoTarea == PARADA_PROCESO ) {
        uint16_t X = tPxP % 3600;
        snprintf(Tiempo,sizeof Tiempo, "%02u:%02hu:%02hu", (uint16_t)(tPxP/3600), (uint8_t)(X/60), (uint8_t)(X % 60));
        myGLCD.print(Tiempo, (myGLCD.getFontXsize()+1)*10, Y_TIEMPO);
    }
    else if( TipoTarea == REPROCESO ) {
        uint16_t X = tReproc % 3600;
        snprintf(Tiempo,sizeof Tiempo, "%02u:%02hu:%02hu", (uint16_t)(tReproc/3600), (uint16_t)(X/60), (uint16_t)(X % 60));
        myGLCD.print(Tiempo, (myGLCD.getFontXsize()+1)*10, Y_TIEMPO);
    }
    else  myGLCD.print("-       ",(myGLCD.getFontXsize()+1)*10, Y_TIEMPO);

    xSemaphoreGive(xMutexTiempos);

    myGLCD.setColor(Color);
    myGLCD.setBackColor(BackColor);
}

void borrarTiempos(void)
{
  xSemaphoreTake(xMutexTiempos, portMAX_DELAY);
  tProd   = 0;
  tSetup  = 0;
  tPxP    = 0;
  tReproc = 0;
  xSemaphoreGive(xMutexTiempos);
}


void actualizarTiempos(UTFT& myGLCD)
{
  xSemaphoreTake(xMutexTiempos, portMAX_DELAY);

  switch(TipoTarea)
  {
    case PRODUCCION:
              ++tProd;
              break;
      case SETUP:
              ++tSetup;
              break;
      case PARADA_PROCESO:
              ++tPxP;
              break;
      case REPROCESO:
              ++tReproc;
              break;
  }
  xSemaphoreGive(xMutexTiempos);
  imprimirHora(myGLCD,false);
}

#endif