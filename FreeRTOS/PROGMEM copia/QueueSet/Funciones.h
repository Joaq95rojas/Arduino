#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

#include "Constantes.h"

extern LiquidCrystal lcd;
extern bool progPausado;
extern bool tareaEnProceso;
extern volatile int8_t tipoTarea;
extern uint16_t golpesBalancinTotal;
extern QueueSetHandle_t xQueueSet;
extern SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore, xEmergenciaSemaphore;
extern QueueHandle_t xButtonQueue, xComunicacionQueue, xEmergenciaSemQueue, \
					xGolpesQueue, xTiemposQueue;
extern bool initBalancin;

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES

#define INIT_SERIAL()		Serial.begin(VELOCIDAD_SERIAL);Serial.println("INIT_SERIAL");
#define CONVERTIR_HORA(h)	h/60.0
#define INIT_VARIABLES(t)	t.segundos = 0; t.minutos = 0
/*
#ifdef __LCD_SHIELD__
#define BORRAR_LCD()  lcd.setCursor(0,0); lcd.print(getMsgString(3));\
                      lcd.setCursor(0,1); lcd.print(getMsgString(3));
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print(getMsgString(3));
#else
#define BORRAR_LCD()  for(int j=0;j<4;j++){\
                        lcd.setCursor(0,j); lcd.print(getMsgString(4)); }
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print(getMsgString(4))
#endif*/
#ifdef __LCD_SHIELD__
#define BORRAR_LCD()  lcd.setCursor(0,0); lcd.print("                ");\
                      lcd.setCursor(0,1); lcd.print("                ");
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                ");
#else
#define BORRAR_LCD()  for(int j=0;j<4;j++){\
                        lcd.setCursor(0,j); lcd.print("                    "); }
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                    ")
#endif
#define PRINT_ERROR(c) lcd.setCursor(0,0); lcd.print(c)

#define PRENDER_LED(i)    _digWrite(i,HIGH)
#define APAGAR_LED(i)     _digWrite(i,LOW)
#define PRENDER_MOTOR()   _digWrite(PIN_MOTOR, LOW)
#define APAGAR_MOTOR()    _digWrite(PIN_MOTOR, HIGH)

#define LEER_SENSOR() _digRead(PIN_SENSOR)

#define INIT_BALANCIN() attachInterrupt(PIN_SENSOR_INT, vSensorHandler, RISING);\
                        initBalancin = true
#define DEINIT_BALANCIN() detachInterrupt(PIN_SENSOR_INT);initBalancin = false
// -------------------------------------------------------------------------------
#define INIT_BOTONES()	  attachInterrupt(INT_NUM, vInterruptHandler, RISING)	
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
char* getMsgString(int16_t) ;

const struct T *actualizarHora(struct T* pHora);
void imprimirHora(const struct T* pHora);

#endif