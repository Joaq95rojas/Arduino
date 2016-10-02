#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

#include "Constantes.h"

extern LiquidCrystal lcd;
extern bool progPausado;
extern bool tareaEnProceso;
extern volatile int8_t tipoTarea, tipoTareaAnterior;
extern uint16_t golpesBalancinTotal;
extern QueueSetHandle_t xQueueSet;
extern SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore, xEmergenciaSemaphore;
extern QueueHandle_t xButtonQueue, xComunicacionQueue, xEmergenciaSemQueue, \
					xGolpesQueue, xTiemposQueue;
extern bool initBalancin;
extern bool motorApagado;

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES

#define	XY(x,y)				lcd.setCursor(x,y)
#define INIT_SERIAL()		Serial.begin(VELOCIDAD_SERIAL); \
							Serial << "INIT_SERIAL" << endl;
#define CONVERTIR_HORA(h)	h/60.0
#define INIT_VARIABLES(t)	t.segundos = 0; t.minutos = 0

#ifdef __LCD_SHIELD__
#define BORRAR_LCD()  	XY(0,0); lcd << "                " << "                ";
#define BORRAR_LINEA_LCD(i) 	XY(0,i); lcd <<"                ";
#else
#define BORRAR_LCD()  for(int j=0;j<4;j++){ XY(0,j); lcd << "                    "; }
#define BORRAR_LINEA_LCD(i) XY(0,i); lcd << "                    ";
#endif
#define PRINT_ERROR(c) 		XY(0,ERROR_CURSOR); lcd << c;

#define PRENDER_LED(i)    _digWrite(i,HIGH)
#define APAGAR_LED(i)     _digWrite(i,LOW)

#define LEER_SENSOR() _digRead(PIN_SENSOR)

#define INIT_BALANCIN() attachInterrupt(PIN_SENSOR_INT, vSensorHandler, RISING);\
                        initBalancin = true
#define DEINIT_BALANCIN() detachInterrupt(PIN_SENSOR_INT);initBalancin = false
// -------------------------------------------------------------------------------
#define INIT_BOTONES()	 {attachInterrupt(INT_NUM, vInterruptHandler, RISING);\	
                    		Serial << "INIT_BOTONES" << endl;} 
#define DEINIT_BOTONES()  detachInterrupt(INT_NUM)

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES

void vHoraTask( void *pvParameters );
void vBotonesTask( void *pvParameters );
void vTaskAssign( void *pvParameters );
void vgolpesBalancinTask( void *pvParameters );
void vEmergenciaTask( void *pvParameters );
// ---------------------------------------------
void vControlGeneralTask( void *pvParameters );
void vComunicacionTask( void *pvParameters );

void vEmergenciaHandler( void );
void vInterruptHandler( void );
void vSensorHandler( void );

uint8_t leerBotones(void);
void initSerial(void);
void initLCD(void);
void initInterrupts(void);
void initSemaphore(void);
void createTasks(void);
void cambiarLEDs(uint8_t);
void onExcLED(uint8_t);
void mostrarPiezasLCD(uint16_t,uint16_t);
void configPin(uint8_t,uint8_t);
void _digWrite(uint8_t,uint8_t);	// Función de escritura optimizada
bool _digRead(uint8_t);
void addToSet(void);
void APAGAR_MOTOR(void);
void PRENDER_MOTOR(void);

const struct T *actualizarHora(struct T* pHora);
void imprimirHora(const struct T* pHora);

#endif