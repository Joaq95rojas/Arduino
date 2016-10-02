#ifndef __CONSTANTES_H__
#define __CONSTANTES_H__

#include <AVRFreeRTOS.h>
#include <utility/timers.h>

#include <utility/task.h>
#include <utility/timers.h>
#include <utility/semphr.h>
#include <utility/queue.h>

// Constantes
//

#define DEBUG 1

#define CONTROLADOR 	"Controlador de Empaque v7"

#define WIFI_SSID     "ANDIF-AP"
#define WIFI_PASSWORD "cerradura2015"
#define MQTT_USUARIO  "andif-empaque-1"
#define MQTT_PASSWORD "6fc39098a05f2051970c91c45b1d81ec"
#define MQTT_SERVIDOR "192.168.1.144"

#define MQTT_PUERTO   1883
#define MQTT_SEGURO   false

#define debugSerial   Serial
#define esp8266Serial Serial3

#define LONGITUD_TARJETA_RFID 		14
#define LONGITUD_NOMBRE_OPERARIO	25

#define PERIODO_TIMER 	1000.0/(1000.0/(F_CPU/16411.35225))
#define PERIODO_INFO	30 * PERIODO_TIMER

#define ESPERA_COLA     			100/portTICK_PERIOD_MS

#define DELAY_TAREA_TOUCH        	20/portTICK_PERIOD_MS
#define DELAY_TAREA_TOUCH_OK		50/portTICK_PERIOD_MS
#define DELAY_TAREA_COMUNICACION 	50/portTICK_PERIOD_MS
#define DELAY_TAREA_RFID 		 	20/portTICK_PERIOD_MS
#define DELAY_TAREA_CONTROL      	30/portTICK_PERIOD_MS
#define DELAY_TAREA_PANTALLA	 	20/portTICK_PERIOD_MS

#define STACK_PANTALLA     650
#define STACK_TOUCH        400
#define STACK_RFID         700
#define STACK_COMUNICACION 1700
#define STACK_CONTROL      950

#define PRIORIDAD_TOUCH        0
#define PRIORIDAD_RFID         0
#define PRIORIDAD_PANTALLA     1
#define PRIORIDAD_COMUNICACION 2
#define PRIORIDAD_CONTROL      3

#define FLAG_COMUNICACION_MQTT_CONECTADO    	0x01
#define FLAG_COMUNICACION_MQTT_DESCONECTADO 	0x02
#define FLAG_COMUNICACION_PUBLICADO         	0x04
#define FLAG_COMUNICACION_DATOS             	0x08
#define FLAG_COMUNICACION_ESTADO_WIFI       	0x10
#define FLAG_COMUNICACION_WIFI_DESCONECTADO 	0x20
#define FLAG_COMUNICACION_ENVIAR_TIEMPO     	0x40
#define FLAG_COMUNICACION_CONEXION_RFID			0x80
#define FLAG_COMUNICACION_PEDIR_ARTICULOS		0x0100
#define FLAG_COMUNICACION_LISTA_ARTICULO		0x0200
#define FLAG_COMUNICACION_LLAMADA_SUPERVISOR	0x0400
#define FLAG_COMUNICACION_PEDIDO_SUPERVISOR		0x0800
#define FLAG_COMUNICACION_ENVIAR_INFO			0x1000

#define FLAG_CONTROL_TARJETA_RFID 	 			0x01
#define FLAG_CONTROL_PEDIR_ARTICULOS			0x02
#define FLAG_CONTROL_ARTICULO_SELECCIONADO		0x04
#define FLAG_CONTROL_HORA						0x08
#define FLAG_CONTROL_LLAMADA_SUPERVISOR			0x10
#define FLAG_CONTROL_LEER_RFID					0x20
#define FLAG_CONTROL_RESPUESTA_RFID				0x40
#define FLAG_CONTROL_RECIBIR_ARTICULOS			0x80
#define FLAG_CONTROL_RESPUESTA_SUPERVISOR		0x0100
#define FLAG_CONTROL_FIN_TAREA					0x0200
#define FLAG_CONTROL_PEDIR_ARTICULOS_BACK		0x0400
#define FLAG_CONTROL_ARTICULO_BORRAR			0x0800
#define FLAG_CONTROL_TARJETA_RFID_SUPERVISOR	0x1000
#define FLAG_CONTROL_CONFIMACION_SUPERVISOR		0x2000
#define FLAG_CONTROL_RESPUESTA_RESUMEN			0x4000

#define FLAG_PANTALLA_TOUCH						0x01
#define FLAG_PANTALLA_RFID						0x02
#define FLAG_PANTALLA_ARTICULOS					0x04
#define FLAG_PANTALLA_LISTA_ARTICULO			0x08
#define FLAG_PANTALLA_HORA						0x10
#define FLAG_PANTALLA_LLAMADA_SUPERVISOR		0x20
#define FLAG_PANTALLA_CONECTANDO_WIFI			0x40
#define FLAG_PANTALLA_RFID_SUPERVISOR			0x80
#define FLAG_PANTALLA_LEYENDO_RFID				0x0100
#define FLAG_PANTALLA_RESUMEN_ACTIVIDAD			0x0200
#define FLAG_PANTALLA_MENU_EJECUCION			0x0400
#define FLAG_PANTALLA_INFO_ACTIVIDAD			0x0800

#define FLAG_RFID_LEER 							0x01
#define FLAG_RFID_LEER_SUPERVISOR				0x02

#define PIN_WIFI 			   15

#define pRFID 				0x01
#define pARTICULOS 			0x02
#define pLISTA_ARTICULOS	0x04
#define pEJECUCION			0x08
#define pFIN_TAREA			0x10
#define pRESUMEN_ACTIVIDAD  0x20
#define pTECLADO 			0x40

#define SIN_TAREA			0
#define PRODUCCION 			1
#define SETUP				2
#define PARADA_PROCESO		3
#define REPROCESO			4
#define LLAMADA_SUPERVISOR 	5

#define CANTIDAD_ARTICULOS_POR_PANTALLA	6
#define CANTIDAD_VARIANTES_ARTICULO		6 	// (857) -> SI ES MAYOR DEBE MODIFICARSE EL CODIGO!!!
#define CANTIDAD_EMBALAJE_TOTAL			6
#define LONGITUD_ARTICULO 				4
#define LONGITUD_LISTA_ARTICULO			12 	// 501 COMB x2

#define PANTALLA_X						799
#define PANTALLA_Y 						479

/*
 * COLORES UTILIZADOS EN LA PANTALLA
 */

#define AZUL_1							12, 80, 137
#define BLANCO                  		255, 255, 255
#define BLANCO_2  						237, 237, 237
#define BLANCO_3  						249, 237, 224
#define BLANCO_4						193, 218, 213
#define CIAN                      		27, 161, 226
#define NEGRO							0, 0, 0
#define ROJO_1							150, 47, 37		// Rojo oscuro
#define ROJO_2							161, 50, 41		// Rojo claro
#define VERDE_CLARO						182, 198, 195
#define VERDE_OSCURO 					0, 100, 0
#define VERDE_OSCURO_2					20, 78, 67
#define VERDE_OLIVA_OSCURO				85, 107, 47
#define COLOR_FONDO_PANTALLA      		45, 45, 45
#define COLOR_FONDO_ARTICULOS     		24, 101, 85
#define COLOR_LISTA_ARTICULOS     		39, 120, 105
#define COLOR_LLAMADA_SUPERVISOR  		45, 146, 70
#define COLOR_BOTON_GENERAL            	9, 92, 148
#define COLOR_BOTON_SELECCIONADO       	3, 149, 249
#define COLOR_BOTON_FIN_TAREA          	255, 51, 102
#define COLOR_BOTON_INICIO_PAUSA       	79, 164, 53
#define COLOR_BOTON_LLAMAR_SUPERVISOR 	169, 175, 79
#define COLOR_BOTONES_TECLADO			48, 47, 55
#define WINDOWS_0   					155, 205, 254     // Celeste
#define WINDOWS_1   					87, 109, 205      // Violeta
#define WINDOWS_2   					64, 115, 254      // Azul
#define WINDOWS_3   					64, 153, 224      // Azul claro
#define WINDOWS_4   					64, 178, 204      // Azul verdoso
#define WINDOWS_5   					64, 196, 174      // Verde claro
#define WINDOWS_6   					64, 193, 127      // Verde
#define WINDOWS_7   					154, 203, 63      // Mostaza
#define WINDOWS_8   					252, 232, 64      // Amarillo
#define WINDOWS_9   					248, 146, 86      // Naranja
#define WINDOWS_10  					224, 103, 106     // Rojo claro
#define WINDOWS_11  					237, 76, 89       // Rojo
#define WINDOWS_12  					215, 181, 202     // Rosa
#define WINDOWS_13  					188, 59, 139      // Violeta
#define WINDOWS_14  					125, 115, 165     // Petroleo
#define WINDOWS_15  					135, 145, 136     // Oro

#define X1_TARJETA      0
#define Y1_TARJETA      44
#define dX_TARJETA      399
#define dY_TARJETA      PANTALLA_Y
#define dART        	200
#define ART_X11     	50
#define ART_X12     	260
#define ART_X13     	470
#define ART_Y11     	50
#define ART_Y12     	260
#define ART_X21     	(ART_X11 + dART)
#define ART_X22     	(ART_X12 + dART)
#define ART_X23     	(ART_X13 + dART)
#define ART_Y21     	(ART_Y11 + dART)
#define ART_Y22     	(ART_Y12 + dART)
#define DSV_X       	45
#define DSV_Y       	30
#define X1_TXT      	(ART_X11 + ART_X21)/2 - DSV_X
#define Y1_TXT      	(ART_Y11 + ART_Y21)/2 - DSV_Y
#define X2_TXT      	(ART_X12 + ART_X22)/2 - DSV_X
#define Y2_TXT      	(ART_Y11 + ART_Y21)/2 - DSV_Y
#define X3_TXT      	(ART_X13 + ART_X23)/2 - DSV_X
#define Y3_TXT      	(ART_Y11 + ART_Y21)/2 - DSV_Y
#define X4_TXT      	(ART_X11 + ART_X21)/2 - DSV_X
#define Y4_TXT      	(ART_Y12 + ART_Y22)/2 - DSV_Y
#define X5_TXT      	(ART_X12 + ART_X22)/2 - DSV_X
#define Y5_TXT      	(ART_Y12 + ART_Y22)/2 - DSV_Y
#define X6_TXT      	(ART_X13 + ART_X23)/2 - DSV_X
#define Y6_TXT      	(ART_Y12 + ART_Y22)/2 - DSV_Y
#define dY2_LISTA  		50
#define dY1_LISTA  		60
#define X1_LISTA   		50
#define X2_LISTA		550
#define Y_LISTA   		35
#define ART1 			1
#define ART2 			2
#define ART3 			3
#define ART4 			4
#define ART5 			5
#define ART6 			6
#define RECUADRO 		4
#define GROSOR  		3

#define TAMANIO_BOTON  			  100
#define MAS_Y2         			  ART_Y22
#define MAS_X1         			  (ART_X23 + 10)
#define MAS_Y1         			  (MAS_Y2 - TAMANIO_BOTON)
#define MAS_X2         			  (MAS_X1 + TAMANIO_BOTON)

/*
 * MACROS DEL MENU PRINCIPAL
 */

// Separación entre botones
// Con este valor (8) se obtiene una distancia entera (no decimal) en 'x' e 'y'.
#define DIST_SEPARACION   8
 // Ancho botones de tarea
#define ANCHO_HORIZONTAL  ((PANTALLA_X + 1 - DIST_SEPARACION*5)/4)//((PANTALLA_X - DIST_SEPARACION) /4) - DIST_SEPARACION
 // Ancho de botones de tarea
#define ANCHO_VERTICAL	  ((PANTALLA_Y +1 - DIST_SEPARACION*5)/4)//((PANTALLA_Y - DIST_SEPARACION) /4) - DIST_SEPARACION
#define PROD_I   		  DIST_SEPARACION	// Comienzo en 'x' de botón Producción
#define PROD_D   		  (PROD_I + ANCHO_HORIZONTAL)
#define SETUP_I  		  (PROD_D + DIST_SEPARACION)
#define SETUP_D  		  (SETUP_I + ANCHO_HORIZONTAL)
#define PXP_I    		  (SETUP_D + DIST_SEPARACION)
#define PXP_D    		  (PXP_I + ANCHO_HORIZONTAL)
#define REPR_I   		  (PXP_D + DIST_SEPARACION)
#define REPR_D   		  (REPR_I + ANCHO_HORIZONTAL)
#define LLS_X   		  (PROD_I + ANCHO_HORIZONTAL/2)-20
#define LLS_Y   		  (IP_D + ANCHO_VERTICAL/2)-20

#define X_DATOS     	  10
#define Y_ARTICULO  	  30
#define Y_TIEMPO    	  60
#define Y_OPERARIO  	  90

#define X1  			  REPR_I	   	// ANCHO DE LOS BOTONES DE LA DERECHA:
#define X2  			  REPR_D		// (IP, SUPERVISOR Y FIN_TAREA)
#define Y2  			  PANTALLA_Y - DIST_SEPARACION
#define Y1  			  Y2 - ANCHO_VERTICAL

#define IP_U          	  DIST_SEPARACION			// Separación VERTICAL ENTRE BOTONES
#define IP_D          	  (IP_U + ANCHO_VERTICAL)
#define SUPERVISOR_U  	  (IP_D + DIST_SEPARACION)
#define SUPERVISOR_D  	  (SUPERVISOR_U + ANCHO_VERTICAL)
#define FIN_TAREA_U   	  (SUPERVISOR_D + DIST_SEPARACION)
#define FIN_TAREA_D   	  (FIN_TAREA_U + ANCHO_VERTICAL)

#define TEXTO_INFO_EJECUCION	320
#define TEXTO_INFO_H1			110
#define TEXTO_INFO_H2			100

#define dBOTON					120
#define TY11					130
#define TY12					(TY11-10+dBOTON)
#define TY21					250
#define TY22					(TY21-10+dBOTON)
#define TX0						50
#define TXF 					(TX0-10+dBOTON)
#define ESP_Y0					(260+dBOTON)
#define TXF_BORRAR				(TXF+dBOTON)
#define TXF_OK					(TXF_BORRAR+3*dBOTON)
#define ESP_YF 					470
#define TX0_OK					(TX0+2*dBOTON)
#define TXT_NUM_1				175
#define TXT_NUM_2				295

#define POSICION_INICIAL		300
#define POS_Y_NUMEROS_TECLADO	45

#define X_RECUADRO				X2_LISTA + 25

#define INFO_TXT_X      150
#define POS_X_MIN		500

struct Posicion {
	int16_t x;
	int16_t y;
};

struct sInfo {
	uint16_t T[4];
	uint8_t TipoTarea;
	char Art[LONGITUD_LISTA_ARTICULO];
	uint16_t CT;
};

#endif
