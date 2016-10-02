#ifndef _FUNCIONES_H_
#define _FUNCIONES_H_

#include "Constantes.h"

extern struct T t_neto,t_setup,t_pxproc,t_repr,t_pausa,t_hora; 
extern LiquidCrystal lcd;
extern boolean progPausado;
extern volatile uint8_t tipoTarea;
extern volatile uint8_t tipoTareaAnterior;
extern volatile uint16_t contGolpes;
extern uint16_t golpesBalancinTotal;
extern SemaphoreHandle_t xButtonSemaphore, xSensorSemaphore, xEmergenciaSemaphore;
extern QueueHandle_t xButtonQueue;
extern boolean initBalancin;

/////////////////////////////////////////////////////////////////
// MACRO FUNCIONES

#ifdef __LCD_SHIELD__
#define BORRAR_LCD()  lcd.setCursor(0,0); lcd.print("                ");\
                      lcd.setCursor(0,1); lcd.print("                ");
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                ");
#else
#define BORRAR_LCD()  for(int j=0;j<4;j++){\
                        lcd.setCursor(0,j); lcd.print("                    "); }
#define BORRAR_LINEA_LCD(i) lcd.setCursor(0,i); lcd.print("                    ");
#endif
#define PRINT_ERROR(cadena) lcd.setCursor(0,0); lcd.print(cadena);

#define PRENDER_LED(i)    _digWrite(i,HIGH);
#define APAGAR_LED(i)     _digWrite(i,LOW);
#define PRENDER_MOTOR()   _digWrite(PIN_MOTOR, LOW)
#define APAGAR_MOTOR()    _digWrite(PIN_MOTOR, HIGH)

#define LEER_SENSOR() _digRead(PIN_SENSOR);

#define INIT_BALANCIN() attachInterrupt(PIN_SENSOR_INT, vSensorHandler, RISING);\
                        initBalancin = true;
#define DEINIT_BALANCIN() detachInterrupt(PIN_SENSOR_INT);initBalancin = false;

/////////////////////////////////////////////////////////////////
// DECLARACIÓN DE FUNCIONES

void vHoraTask( void *pvParameters );
void vBotonesTask( void *pvParameters );
void vTaskAssign( void *pvParameters );
void vgolpesBalancinTask( void *pvParameters );
void vEmergenciaTask( void *pvParameters );

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
void mostrarPiezasLCD(void);
void initVariables(void);
void configPin(uint8_t,uint8_t);
void _digWrite(uint8_t,uint8_t);	// Función de escritura optimizada
boolean _digRead(uint8_t);

const struct T *actualizarHora(struct T* pHora);
void imprimirHora(const struct T* pHora);

#endif