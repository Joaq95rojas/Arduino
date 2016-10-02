#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

//#include <string>
#include "Constantes.h"

extern struct T t_neto; 
extern LiquidCrystal lcd;
extern bool progPausado;
extern bool tareaEnProceso;
extern volatile uint8_t tipoTarea;
extern uint16_t golpesBalancinTotal;
extern QueueSetHandle_t xQueueSet;
extern SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore, xEmergenciaSemaphore;
extern QueueHandle_t xButtonQueue, xComunicacionQueue, xEmergenciaSemQueue, \
					xGolpesQueue;
extern bool initBalancin;

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES

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

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES

void vEmergenciaTask( void *pvParameters );
// ---------------------------------------------
void vControlGeneralTask( void *pvParameters );
void vComunicacionTask( void *pvParameters );

void vEmergenciaHandler( void );

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

#endif