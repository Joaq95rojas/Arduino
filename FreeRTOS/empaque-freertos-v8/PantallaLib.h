#ifndef __PANTALLALIB_H__
#define __PANTALLALIB_H__

#include "constantes.h"

extern char articulos[CANTIDAD_ARTICULOS_POR_PANTALLA][LONGITUD_ARTICULO];
extern char NombreOperario[LONGITUD_NOMBRE_OPERARIO];
extern char ArticuloEmpaque[LONGITUD_LISTA_ARTICULO];

extern TaskHandle_t xComunicacionHandle;
extern TaskHandle_t xPantallaHandle;
extern QueueHandle_t xListaArticuloQueue;
extern QueueHandle_t xTouchQueue;
extern QueueHandle_t xuIntQueue;
extern SemaphoreHandle_t xMutexArticulos;
extern SemaphoreHandle_t xMutexOperario;

extern uint16_t llamada_icon[0x2400];

extern uint8_t Ubuntu_Mono_32[];
extern uint8_t Ubuntu_Mono_64[];

extern uint16_t CT;

void configurarPantalla(UTFT& myGLCD)
{
  myGLCD.InitLCD();
  myGLCD.setFont(Ubuntu_Mono_32);

  myGLCD.clrScr();

  myGLCD.setColor(200,200,200);
  myGLCD.setBackColor(NEGRO);
}

void dibujarBotonAtras(UTFT& myGLCD)
{
  // Botón de retroceso
  myGLCD.setColor(VERDE_OSCURO_2);
  myGLCD.fillRoundRect(MAS_X1,MAS_Y1,MAS_X2, MAS_Y2);
  myGLCD.setColor(VERDE_CLARO);
  myGLCD.setBackColor(VERDE_OSCURO_2);
  for(int i=0; i<40; i++)
    myGLCD.drawLine(MAS_X2-30-i,MAS_Y1+10+i,MAS_X2-30-i,MAS_Y2-10-i);
}

void mostrarPantallaRFID(UTFT& myGLCD, bool bSupervisor)
{
	char buffer[19] = "";
  uint16_t PosY = myGLCD.getFontYsize()+10;

	myGLCD.fillScr(COLOR_FONDO_PANTALLA);

  myGLCD.setColor(AZUL_1);
  myGLCD.fillRect(0,0,PANTALLA_X,PosY);

  myGLCD.setColor(BLANCO_4);
  myGLCD.setBackColor(AZUL_1);

  buffer[0] = '\0';
	if(bSupervisor) strncat_P(buffer, PSTR("Confirmar cantidad"), sizeof(buffer)-1);
  else            strncat_P(buffer, PSTR("Inicie tarea nueva"), sizeof(buffer)-1);
  myGLCD.print(buffer, RIGHT, 5);
/*
  myGLCD.setColor(COLOR_BOTONES_TECLADO);
  myGLCD.fillRoundRect(X1_TARJETA, Y1_TARJETA, dX_TARJETA, dY_TARJETA);
*/
  myGLCD.setColor(BLANCO_4);
  myGLCD.setBackColor(COLOR_BOTONES_TECLADO);
//////////////////////////////////
  buffer[0] = '\0';
//  strncat_P(buffer, PSTR("COLOQUE AQUI"), sizeof(buffer)-1);
  strncat_P(buffer,  PSTR("Coloque su tarjeta RFID"), sizeof buffer);
//  myGLCD.print(buffer, 120, 200);
  myGLCD.print(buffer, CENTER, 150);

  buffer[0] = '\0';
//  strncat_P(buffer, PSTR("SU TARJETA RFID"), sizeof(buffer)-1);
  strncat_P(buffer, PSTR("en el lugar indicado", sizeof buffer));
//  myGLCD.print(buffer, 100, 250);
  myGLCD.print(buffer, CENTER, 200);
//////////////////////////
  /*
  for(int i=-1;i<2;i++){
      myGLCD.drawLine(0,PosY+i,PANTALLA_X,PosY+i);
      myGLCD.drawLine(dX_TARJETA+1+i,PosY,dX_TARJETA+1+i,PANTALLA_Y);
  }*/
}


void mostrarPantallaArticulos(UTFT& myGLCD) {

	char buffer[23] = "";
  uint16_t PosY = myGLCD.getFontYsize() + 10;

	myGLCD.fillScr(COLOR_FONDO_PANTALLA);
	myGLCD.setColor(AZUL_1);
  myGLCD.fillRect(0, 0, PANTALLA_X, PosY);
  myGLCD.setBackColor(AZUL_1);
  myGLCD.setColor(BLANCO_4);

  for(int i=-1; i<2; i++) myGLCD.drawLine(0, PosY+i, PANTALLA_X, PosY+i);

	buffer[0] = '\0';
  strncat_P(buffer, PSTR("Seleccione un articulo"), sizeof(buffer)-1);
	myGLCD.print(buffer, 15, 5);

  xSemaphoreTake(xMutexArticulos, portMAX_DELAY);

	if(strcmp(articulos[0],""))  myGLCD.setColor(WINDOWS_3);
  else                         myGLCD.setColor(COLOR_FONDO_PANTALLA);
  myGLCD.fillRoundRect(ART_X11, ART_Y11, ART_X21, ART_Y21);

	if(strcmp(articulos[1],""))  myGLCD.setColor(WINDOWS_7);
  else                         myGLCD.setColor(COLOR_FONDO_PANTALLA);
  myGLCD.fillRoundRect(ART_X12, ART_Y11, ART_X22, ART_Y21);

	if(strcmp(articulos[2],""))  myGLCD.setColor(WINDOWS_14);
  else                         myGLCD.setColor(COLOR_FONDO_PANTALLA);
  myGLCD.fillRoundRect(ART_X13, ART_Y11, ART_X23, ART_Y21);

	if(strcmp(articulos[3],""
    ))  myGLCD.setColor(CIAN);
  else                         myGLCD.setColor(COLOR_FONDO_PANTALLA);
  myGLCD.fillRoundRect(ART_X11, ART_Y12, ART_X21, ART_Y22);

	if(strcmp(articulos[4],""))  myGLCD.setColor(WINDOWS_5);
  else                         myGLCD.setColor(COLOR_FONDO_PANTALLA);
  myGLCD.fillRoundRect(ART_X12, ART_Y12, ART_X22, ART_Y22);

	if(strcmp(articulos[5],""))  myGLCD.setColor(WINDOWS_10);
  else                         myGLCD.setColor(COLOR_FONDO_PANTALLA);
  myGLCD.fillRoundRect(ART_X13, ART_Y12, ART_X23, ART_Y22);

  xSemaphoreGive(xMutexArticulos);

	myGLCD.setColor(BLANCO_4);

	uint8_t *Font = myGLCD.getFont();
	myGLCD.setFont(Ubuntu_Mono_64);

  xSemaphoreTake(xMutexArticulos, portMAX_DELAY);

  myGLCD.setBackColor(WINDOWS_3);
  if(strcmp(articulos[0],""))   myGLCD.print(articulos[0], X1_TXT, Y1_TXT);
	myGLCD.setBackColor(WINDOWS_7);
	if(strcmp(articulos[1],""))   myGLCD.print(articulos[1], X2_TXT, Y2_TXT);
  myGLCD.setBackColor(WINDOWS_14);
  if(strcmp(articulos[2],""))   myGLCD.print(articulos[2], X3_TXT, Y3_TXT);
  myGLCD.setBackColor(CIAN);
  if(strcmp(articulos[3],""))   myGLCD.print(articulos[3], X4_TXT, Y4_TXT);
  myGLCD.setBackColor(WINDOWS_5);
  if(strcmp(articulos[4],""))   myGLCD.print(articulos[4], X5_TXT, Y5_TXT);
  myGLCD.setBackColor(WINDOWS_10);
  if(strcmp(articulos[5],""))   myGLCD.print(articulos[5], X6_TXT, Y6_TXT);

	xSemaphoreGive(xMutexArticulos);

  // IMPRIME EL SIMBOLO 'MAS'
	myGLCD.setColor(WINDOWS_9);
  myGLCD.fillRoundRect(MAS_X1,MAS_Y1,MAS_X2, MAS_Y2);
  myGLCD.setColor(BLANCO_4);
  myGLCD.setBackColor(WINDOWS_9);
	myGLCD.print("+", MAS_X1+34,MAS_Y1+18);

	myGLCD.setFont(Font);
}


void mostrarPantallaMasArticulos(UTFT& myGLCD) {

	uint8_t *Font = myGLCD.getFont();
	myGLCD.setFont(Ubuntu_Mono_64);

	myGLCD.setColor(BLANCO_4);

	xSemaphoreTake(xMutexArticulos, portMAX_DELAY);

  myGLCD.setBackColor(WINDOWS_3);
  if(strcmp(articulos[0],""))   myGLCD.print(articulos[0], X1_TXT, Y1_TXT);
	else{
    myGLCD.setColor(COLOR_FONDO_PANTALLA);
    myGLCD.fillRoundRect(ART_X11, ART_Y11, ART_X21, ART_Y21);
    myGLCD.setColor(BLANCO_4);
  }

	myGLCD.setBackColor(WINDOWS_7);
  if(strcmp(articulos[1],""))   myGLCD.print(articulos[1], X2_TXT, Y2_TXT);
	else{
    myGLCD.setColor(COLOR_FONDO_PANTALLA);
    myGLCD.fillRoundRect(ART_X12, ART_Y11, ART_X22, ART_Y21);
    myGLCD.setColor(BLANCO_4);
  }

	myGLCD.setBackColor(WINDOWS_14);
  if(strcmp(articulos[2],""))   myGLCD.print(articulos[2], X3_TXT, Y3_TXT);
	else{
    myGLCD.setColor(COLOR_FONDO_PANTALLA);
    myGLCD.fillRoundRect(ART_X13, ART_Y11, ART_X23, ART_Y21);
    myGLCD.setColor(BLANCO_4);
  }

	myGLCD.setBackColor(CIAN);
  if(strcmp(articulos[3],""))   myGLCD.print(articulos[3], X4_TXT, Y4_TXT);
	else{
    myGLCD.setColor(COLOR_FONDO_PANTALLA);
    myGLCD.fillRoundRect(ART_X11, ART_Y12, ART_X21, ART_Y22);
    myGLCD.setColor(BLANCO_4);
  }

	myGLCD.setBackColor(WINDOWS_5);
  if(strcmp(articulos[4],""))   myGLCD.print(articulos[4], X5_TXT, Y5_TXT);
	else{
    myGLCD.setColor(COLOR_FONDO_PANTALLA);
    myGLCD.fillRoundRect(ART_X12, ART_Y12, ART_X22, ART_Y22);
    myGLCD.setColor(BLANCO_4);
  }

	myGLCD.setBackColor(WINDOWS_10);
  if(strcmp(articulos[5],""))   myGLCD.print(articulos[5], X6_TXT, Y6_TXT);
	else{
    myGLCD.setColor(COLOR_FONDO_PANTALLA);
    myGLCD.fillRoundRect(ART_X13, ART_Y12, ART_X23, ART_Y22);
    myGLCD.setColor(BLANCO_4);
  }

	xSemaphoreGive(xMutexArticulos);

	myGLCD.setFont(Font);
}

void imprimirListaArticulo(UTFT& myGLCD, Lista Lst)
{
  myGLCD.fillScr(COLOR_FONDO_ARTICULOS);
  myGLCD.setColor(COLOR_LISTA_ARTICULOS);

  for (int i = 0; i < Lst.T; i++) {
    int I = Y_LISTA + dY1_LISTA * i;
    myGLCD.fillRect(X1_LISTA, I, X2_LISTA, I + dY2_LISTA);
  }
  // Recuadro de la derecha
  myGLCD.fillRect(X_RECUADRO,0,PANTALLA_X,PANTALLA_Y);

  myGLCD.setBackColor(COLOR_LISTA_ARTICULOS);
  myGLCD.setColor(BLANCO_4);

  // Texto informativo
  char buffer[12];
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Seleccione"), sizeof(buffer)-1);
  myGLCD.print(buffer, RIGHT, 5);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("el tipo de"), sizeof(buffer)-1);
  myGLCD.print(buffer, RIGHT, 10 + myGLCD.getFontYsize());
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("articulo"), sizeof(buffer)-1);
  myGLCD.print(buffer, RIGHT, 15 + 2 * myGLCD.getFontYsize());

  // Lineas
  for(int i=-1; i<2; i++) myGLCD.drawLine(X_RECUADRO+i,0,X_RECUADRO+i,PANTALLA_Y);

  // Texto artículos
  for (int i = 0; i < Lst.T; i++) {
    int I = Y_LISTA + dY1_LISTA * i;
    myGLCD.print(Lst.L[i], X1_LISTA + 5, I + 10);
  }

  dibujarBotonAtras(myGLCD);
}


void imprimirTarea(UTFT& myGLCD, uint8_t T, bool Seleccionado)
{
  if (Seleccionado)  myGLCD.setColor(COLOR_BOTON_SELECCIONADO);
  else               myGLCD.setColor(COLOR_BOTON_GENERAL);

  switch (T) {
    case  PRODUCCION:
      myGLCD.fillRoundRect(PROD_I, Y1, PROD_D, Y2);
      break;
    case SETUP:
      myGLCD.fillRoundRect(SETUP_I, Y1, SETUP_D, Y2);
      break;
    case PARADA_PROCESO:
      myGLCD.fillRoundRect(PXP_I, Y1, PXP_D, Y2);
      break;
    case REPROCESO:
      myGLCD.fillRoundRect(REPR_I, Y1, REPR_D, Y2);
      break;
    case SIN_TAREA:
      break;
    default:
      myGLCD.fillRoundRect(PROD_I, Y1, PROD_D, Y2);
      myGLCD.fillRoundRect(SETUP_I, Y1, SETUP_D, Y2);
      myGLCD.fillRoundRect(PXP_I, Y1, PXP_D, Y2);
      myGLCD.fillRoundRect(REPR_I, Y1, REPR_D, Y2);
      break;
  }
  myGLCD.setColor(BLANCO_4);

  if (Seleccionado)  myGLCD.setBackColor(COLOR_BOTON_SELECCIONADO);
  else               myGLCD.setBackColor(COLOR_BOTON_GENERAL);

  char buffer[11];
  buffer[0] = '\0';
  uint16_t PosY = Y2 - (ANCHO_VERTICAL + myGLCD.getFontYsize())/2;

  if( T == PRODUCCION ) {
    strncat_P(buffer, PSTR("PRODUCCION"), sizeof(buffer)-1);
    uint16_t PosX = PROD_I + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
    myGLCD.print(buffer, PosX, PosY);
  }
  else if( T == SETUP ) {
    strncat_P(buffer, PSTR("SETUP"), sizeof(buffer)-1);
    uint16_t PosX = SETUP_I + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
    myGLCD.print(buffer, PosX, PosY);
  }
  else if( T == PARADA_PROCESO ) {
    strncat_P(buffer, PSTR("PARADA"), sizeof(buffer)-1);
    uint16_t PosX = PXP_I + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
    myGLCD.print(buffer, PosX, PosY-30);
    buffer[0] = '\0';
    strncat_P(buffer, PSTR("POR"), sizeof(buffer)-1);
    PosX = PXP_I + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
    myGLCD.print(buffer, PosX, PosY);
    buffer[0] = '\0';
    strncat_P(buffer, PSTR("PROCESO"), sizeof(buffer)-1);
    PosX = PXP_I + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
    myGLCD.print(buffer, PosX, PosY+30);
  }
  else if( T == REPROCESO ) {
    strncat_P(buffer, PSTR("REPROCESO"), sizeof(buffer)-1);
    uint16_t PosX = REPR_I + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
    myGLCD.print(buffer, PosX, PosY);
  }
}


void imprimirMenuPrincipal(UTFT& myGLCD,char* Articulo)
{
  char buffer[23];

  myGLCD.fillScr(COLOR_FONDO_PANTALLA);      // FONDO DE PANTALLA

  myGLCD.setColor(BLANCO_4);                      // COLOR DE PALABRAS
  myGLCD.setBackColor(COLOR_FONDO_PANTALLA);    // COLOR DE FONDO DE PALABRAS
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("ARTICULO  "), sizeof(buffer)-1);
  myGLCD.print(buffer, X_DATOS, Y_ARTICULO);  myGLCD.print(Articulo, (myGLCD.getFontXsize()+1)*10, Y_ARTICULO);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("TIEMPO    -"), sizeof(buffer)-1);
  myGLCD.print(buffer, X_DATOS, Y_TIEMPO);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("OPERARIO  "), sizeof(buffer)-1);
  myGLCD.print(buffer, X_DATOS, Y_OPERARIO);
  xSemaphoreTake(xMutexOperario, portMAX_DELAY);
  myGLCD.print(NombreOperario, (myGLCD.getFontXsize() + 1) * 10, Y_OPERARIO);
  xSemaphoreGive(xMutexOperario);

  // TEXTO DE AYUDA EN PANTALLA
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Seleccione una tarea"), sizeof(buffer)-1);
  myGLCD.print(buffer,TEXTO_INFO_H1,TEXTO_INFO_EJECUCION-(myGLCD.getFontYsize() + 5));
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Luego, presione Inicio"), sizeof(buffer)-1);
  myGLCD.print(buffer,TEXTO_INFO_H2,TEXTO_INFO_EJECUCION);
  // Botones de la derecha (especiales)
  // -> Botones
  myGLCD.setColor(COLOR_BOTON_INICIO_PAUSA);
  myGLCD.fillRoundRect(X1, IP_U, X2, IP_D);                  // INICIO/PAUSA
  myGLCD.setColor(COLOR_BOTON_LLAMAR_SUPERVISOR);
  myGLCD.fillRoundRect(X1, SUPERVISOR_U, X2, SUPERVISOR_D);  // LLAMAR AL SUPERVISOR
  myGLCD.setColor(COLOR_BOTON_FIN_TAREA);
  myGLCD.fillRoundRect(X1, FIN_TAREA_U, X2, FIN_TAREA_D);    // FINALIZAR TAREA
  // ->Texto
  uint16_t PosX;
  uint16_t PosY;
  myGLCD.setColor(BLANCO_4);
  myGLCD.setBackColor(COLOR_BOTON_INICIO_PAUSA);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("INICIO"), sizeof(buffer)-1);
  PosX = X1 + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
  PosY = IP_U + ANCHO_VERTICAL/2 - myGLCD.getFontYsize();
  myGLCD.print(buffer, PosX, PosY);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("PAUSA"), sizeof(buffer)-1);
  PosX = X1 + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
  PosY += (myGLCD.getFontYsize() + 5);
  myGLCD.print(buffer, PosX, PosY);
  myGLCD.setBackColor(COLOR_BOTON_LLAMAR_SUPERVISOR);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("LLAMAR AL"), sizeof(buffer)-1);
  PosX = X1 + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
  PosY = SUPERVISOR_U + ANCHO_VERTICAL/2 - myGLCD.getFontYsize();
  myGLCD.print(buffer, PosX, PosY);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("SUPERVISOR"), sizeof(buffer)-1);
  PosX = X1 + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
  PosY += (myGLCD.getFontYsize() + 5);
  myGLCD.print(buffer, PosX, PosY);
  myGLCD.setBackColor(COLOR_BOTON_FIN_TAREA);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("FINALIZAR"), sizeof(buffer)-1);
  PosX = X1 + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
  PosY = FIN_TAREA_U + ANCHO_VERTICAL/2 - myGLCD.getFontYsize();
  myGLCD.print(buffer, PosX, PosY);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("TAREA"), sizeof(buffer)-1);
  PosX = X1 + (ANCHO_HORIZONTAL - myGLCD.getFontXsize()*strlen(buffer))/2;
  PosY += (myGLCD.getFontYsize() + 5);
  myGLCD.print(buffer, PosX, PosY);

  imprimirTarea(myGLCD, PRODUCCION, false);
  imprimirTarea(myGLCD, SETUP, false);
  imprimirTarea(myGLCD, PARADA_PROCESO, false);
  imprimirTarea(myGLCD, REPROCESO, false);
}


void remarcarTarea(UTFT& myGLCD, uint8_t TareaActual, bool Remarcar)
{
  if (Remarcar == true)  myGLCD.setColor(BLANCO_4);
  else                   myGLCD.setColor(COLOR_FONDO_PANTALLA);

  for(int i=1 ; i<GROSOR ; i++)
  {
    switch (TareaActual)
    {
      case  PRODUCCION:
          myGLCD.drawRoundRect(PROD_I-i, Y1-i, PROD_D+i, Y2+i);
          break;
      case SETUP:
          myGLCD.drawRoundRect(SETUP_I-i, Y1-i, SETUP_D+i, Y2+i);
          break;
      case PARADA_PROCESO:
          myGLCD.drawRoundRect(PXP_I-i, Y1-i, PXP_D+i, Y2+i);
          break;
      case REPROCESO:
          myGLCD.drawRoundRect(REPR_I-i, Y1-i, REPR_D+i, Y2+i);
          break;
      case SIN_TAREA:
          myGLCD.drawRoundRect(PROD_I-i, Y1-i, PROD_D+i, Y2+i);
          myGLCD.drawRoundRect(SETUP_I-i, Y1-i, SETUP_D+i, Y2+i);
          myGLCD.drawRoundRect(PXP_I-i, Y1-i, PXP_D+i, Y2+i);
          myGLCD.drawRoundRect(REPR_I-i, Y1-i, REPR_D+i, Y2+i);
          break;
    }
  }
}


void imprimirTeclado(UTFT& myGLCD)
{
  char buffer[32];
  buffer[0] = '\0';

  myGLCD.fillScr(NEGRO);

  // DIBUJA LOS BOTONES
  myGLCD.setColor(COLOR_BOTONES_TECLADO);
  for(int i=0; i<5; i++) {
    myGLCD.fillRoundRect (TX0+(i*dBOTON), TY11, TXF+(i*dBOTON), TY12);
    myGLCD.fillRoundRect (TX0+(i*dBOTON), TY21, TXF+(i*dBOTON), TY22);
  }
  myGLCD.setColor(WINDOWS_0);
  myGLCD.fillRoundRect(TX0, ESP_Y0, TXF_BORRAR, ESP_YF);   // BOTON BORRAR
  myGLCD.setColor(WINDOWS_9);
  myGLCD.fillRoundRect(TX0_OK, ESP_Y0, TXF_OK, ESP_YF);   // BOTON OK

  // DIBUJA LOS BORDES DE LOS BOTONES  Y LAS LEYENDAS
  myGLCD.setColor(BLANCO_4);
  myGLCD.setBackColor(COLOR_BOTONES_TECLADO);
  for(int i=0; i<5; i++) {
    myGLCD.drawRoundRect (TX0+(i*dBOTON), TY11, TXF+(i*dBOTON), TY12);
    myGLCD.printNumI(i+1, ((i+1)*dBOTON)-17, TXT_NUM_1);
    myGLCD.drawRoundRect (TX0+(i*dBOTON), TY21, TXF+(i*dBOTON), TY22);
    if (i<4)  myGLCD.printNumI(i+6, ((i+1)*dBOTON)-17, TXT_NUM_2);
  }
  strncat_P(buffer, PSTR("0"), sizeof(buffer)-1);
  myGLCD.print(buffer, 5*dBOTON-17, 295);
  myGLCD.drawRoundRect(TX0, ESP_Y0, TXF_BORRAR, ESP_YF);   // BOTON BORRAR
  myGLCD.drawRoundRect(TX0_OK, ESP_Y0, TXF_OK, ESP_YF);   // BOTON OK

  // TEXTO DE INFORMACIÓN
  myGLCD.setBackColor(NEGRO);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Ingrese la cantidad empaquetada"), sizeof(buffer)-1);
  myGLCD.print(buffer, 10, 10);

  myGLCD.setBackColor(WINDOWS_0);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Borrar"), sizeof(buffer)-1);
  myGLCD.print(buffer, 110, 470-60);

  myGLCD.setBackColor(WINDOWS_9);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Confirmar"), sizeof(buffer)-1);
  myGLCD.print(buffer, 40+3*dBOTON, 470-60);

  dibujarBotonAtras(myGLCD);
}


void imprimirNumero(UTFT& myGLCD, uint16_t& Pos, int8_t n)
{
  // Imprime el número presionado en el teclado
  char N[2];
  uint8_t *Font = myGLCD.getFont();
  myGLCD.setFont(Ubuntu_Mono_64);
  if(n>0) {
    if(n == 10) snprintf_P(N,sizeof N, PSTR("0"));
    else        snprintf(N,sizeof N, "%01d", n);
  }
  else{
    snprintf(N,sizeof N, "%s"," ");
    if(Pos >= myGLCD.getFontXsize()) Pos-=myGLCD.getFontXsize();
    if(Pos < POSICION_INICIAL) Pos = POSICION_INICIAL;
  }

  myGLCD.setBackColor(NEGRO);
  myGLCD.setColor(WINDOWS_9);
  myGLCD.print(N,Pos,POS_Y_NUMEROS_TECLADO);

  if(n > 0)   Pos+=myGLCD.getFontXsize();

  myGLCD.setFont(Font);
}

void borrarNumero(UTFT& myGLCD, uint16_t& Pos)
{
  // Borra los números ingresados
  imprimirNumero(myGLCD,Pos,-1);
}

void imprimirPantallaConectandoWiFi(UTFT& myGLCD)
{
  char buffer[30];
  buffer[0] = '\0';

  uint16_t PosY   = PANTALLA_Y - 5 - myGLCD.getFontYsize();
  uint16_t PosY_2 = PosY - 5 - myGLCD.getFontYsize();

  myGLCD.fillScr(COLOR_FONDO_PANTALLA);
  myGLCD.setBackColor(COLOR_FONDO_PANTALLA);
  // Recuadro azul
  myGLCD.setColor(AZUL_1);
  myGLCD.fillRect(0, PosY_2, PANTALLA_X, PANTALLA_Y);
  myGLCD.setColor(BLANCO_4);

  strncat_P(buffer, PSTR("Conectando a la red WiFi"), sizeof(buffer)-1);
  myGLCD.print(buffer,CENTER,240);
  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Aguarde un momento, por favor"), sizeof(buffer)-1);
  myGLCD.print(buffer,CENTER,300);

  myGLCD.setBackColor(AZUL_1);

  buffer[0] = '\0';
  strncat_P(buffer, PSTR(CONTROLADOR), sizeof(buffer)-1);
  myGLCD.print(buffer, RIGHT, PosY_2);

  buffer[0] = '\0';
  strncat_P(buffer, PSTR("Depto. de Ingenieria"), sizeof(buffer)-1);
  myGLCD.print(buffer, RIGHT, PosY);

  for(int i=-1; i<2; i++) myGLCD.drawLine(0, PosY_2+i, PANTALLA_X, PosY_2+i);
}


void vPantallaTask(void* parametros) {
  uint8_t TipoPantalla = pRFID;
  uint8_t TipoTareaAnterior = SIN_TAREA, TipoTareaPresel = SIN_TAREA;
  bool Time = false;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  UTFT myGLCD(ITDB50, 38, 39, 40, 41);
  configurarPantalla(myGLCD);

    uint32_t flags = 0UL;

    _bln_debug_println(F(">pantalla-task"));

  for ( ; ; ) {
    xTaskNotifyWait(0UL, 0xFFFFFFFFUL, &flags, portMAX_DELAY);

    Lista Lst;

    // Touch.
    if ( flags & FLAG_PANTALLA_TOUCH && TipoTarea != LLAMADA_SUPERVISOR) {
      // Entra si no se está llamando al supervisor

      Posicion posicion;

      if ( xQueueReceive(xTouchQueue, &posicion, 0 ) == pdTRUE ) {

        _bln_debug_print(F("("));
        _bln_debug_print(posicion.x);
        _bln_debug_print(F(";"));
        _bln_debug_print(posicion.y);
        _bln_debug_println(F(")"));

        if ( TipoPantalla == pARTICULOS ) {

          uint8_t ArticuloSeleccionado = 0;

          if (posicion.y >= ART_Y11 && posicion.y <= ART_Y21){
            if (posicion.x >= ART_X11 && posicion.x <= ART_X21){
              _bln_debug_println(F("Articulo 1 seleccionado."));
              xSemaphoreTake(xMutexArticulos, portMAX_DELAY);
              if(strcmp(articulos[0],""))  ArticuloSeleccionado = ART1;
              xSemaphoreGive(xMutexArticulos);
            }
            if (posicion.x >= ART_X12 && posicion.x <= ART_X22){
              _bln_debug_println(F("Articulo 2 seleccionado."));
              xSemaphoreTake(xMutexArticulos, portMAX_DELAY);
              if(strcmp(articulos[1],""))  ArticuloSeleccionado = ART2;
              xSemaphoreGive(xMutexArticulos);
            }
            if (posicion.x >= ART_X13 && posicion.x <= ART_X23){
              _bln_debug_println(F("Articulo 3 seleccionado."));
              xSemaphoreTake(xMutexArticulos, portMAX_DELAY);
              if(strcmp(articulos[2],""))  ArticuloSeleccionado = ART3;
              xSemaphoreGive(xMutexArticulos);
            }
          }
          else if (posicion.y >= ART_Y12 && posicion.y <= ART_Y22){
            if (posicion.x >= ART_X11 && posicion.x <= ART_X21){
              _bln_debug_println(F("Articulo 4 seleccionado."));
              xSemaphoreTake(xMutexArticulos, portMAX_DELAY);
              if(strcmp(articulos[3],""))  ArticuloSeleccionado = ART4;
              xSemaphoreGive(xMutexArticulos);
            }
            if (posicion.x >= ART_X12 && posicion.x <= ART_X22){
              _bln_debug_println(F("Articulo 5 seleccionado."));
              xSemaphoreTake(xMutexArticulos, portMAX_DELAY);
              if(strcmp(articulos[4],""))  ArticuloSeleccionado = ART5;
              xSemaphoreGive(xMutexArticulos);
            }
            if (posicion.x >= ART_X13 && posicion.x <= ART_X23){
              _bln_debug_println(F("Articulo 6 seleccionado."));
              xSemaphoreTake(xMutexArticulos, portMAX_DELAY);
              if(strcmp(articulos[5],""))  ArticuloSeleccionado = ART6;
              xSemaphoreGive(xMutexArticulos);
            }
          }

          if ( ArticuloSeleccionado != 0 )
          {
            // Solo si se ha seleccionado un artículo
            TipoPantalla = pLISTA_ARTICULOS;
            xQueueSend(xuIntQueue, &ArticuloSeleccionado, ESPERA_COLA);
            xTaskNotify(xComunicacionHandle, FLAG_COMUNICACION_LISTA_ARTICULO, eSetBits);
          }

          if (posicion.x >= MAS_X1 && posicion.x <= MAS_X2 &&
              posicion.y >= MAS_Y1 && posicion.y <= MAS_Y2) {
            // Botón '+'
            _bln_debug_println(F("Boton '+' Articulos"));
            TipoPantalla = pARTICULOS;
            xTaskNotify(xControlHandle, FLAG_CONTROL_PEDIR_ARTICULOS, eSetBits);
          }
        }
        else if ( TipoPantalla == pLISTA_ARTICULOS )
        {
          for (int i = 0; i < Lst.T; i++) {
                int I = Y_LISTA + dY1_LISTA * i;
                if (posicion.y >= I && posicion.y <= I + dY2_LISTA) {
                  if (posicion.x >= X1_LISTA && posicion.x <= 700) {
                    TipoPantalla = pEJECUCION;
                    strcpy(ArticuloEmpaque, Lst.L[i]);
                    xTaskNotify(xPantallaHandle, FLAG_PANTALLA_MENU_EJECUCION, eSetBits);
                  }
                }
          }

          if (posicion.x >= MAS_X1 && posicion.x <= MAS_X2 &&
              posicion.y >= MAS_Y1 && posicion.y <= MAS_Y2) {
              // Botón de retroceso
              xTaskNotify(xControlHandle, FLAG_CONTROL_PEDIR_ARTICULOS_BACK, eSetBits);
          }
        }
        else if ( TipoPantalla == pEJECUCION )
        {
          if ( TipoTarea == SIN_TAREA )
          {
                if (posicion.y >= Y1 && posicion.y <= Y2)      // Botones inferiores
                {
                  imprimirTarea(myGLCD, TipoTareaAnterior, false);     // APAGA LA TAREA ACTIVA

                  if (posicion.x >= PROD_I && posicion.x <= PROD_D) {
                    TipoTareaAnterior = TipoTareaPresel;
                    TipoTareaPresel = PRODUCCION;
                  }
                  if (posicion.x >= SETUP_I && posicion.x <= SETUP_D) {
                    TipoTareaAnterior = TipoTareaPresel;
                    TipoTareaPresel = SETUP;
                  }
                  if (posicion.x >= PXP_I && posicion.x <= PXP_D) {
                    TipoTareaAnterior = TipoTareaPresel;
                    TipoTareaPresel = PARADA_PROCESO;
                  }
                  if (posicion.x >= REPR_I && posicion.x <= REPR_D) {
                    TipoTareaAnterior = TipoTareaPresel;
                    TipoTareaPresel = REPROCESO;
                  }
                }
                imprimirTarea(myGLCD, TipoTareaPresel, true);
                imprimirTarea(myGLCD, TipoTareaAnterior, false);
            }
            if (posicion.x >= X1 && posicion.x <= X2) {

                if (posicion.y >= IP_U && posicion.y <= IP_D)
                {
                  // INICIO - PAUSA

                  if (TipoTareaPresel != SIN_TAREA) {

                    if (!Time) {
                      // Se inicia la Tarea
                        Time = true;
                        xSemaphoreTake(xMutexTiempos, portMAX_DELAY);
                        TipoTarea = TipoTareaPresel;
                        xSemaphoreGive(xMutexTiempos);
                        remarcarTarea(myGLCD, TipoTarea, true);
                        TimerON();
                      }
                      else {
                        // Se pausa la tarea
                        Time = false;
                        TimerOFF();
                        imprimirTarea(myGLCD, TipoTarea, false);
                        remarcarTarea(myGLCD, TipoTarea, false);
                        TipoTareaAnterior = TipoTarea;
                        xSemaphoreTake(xMutexTiempos, portMAX_DELAY);
                        TipoTarea = SIN_TAREA;
                        xSemaphoreGive(xMutexTiempos);
                        TipoTareaPresel = SIN_TAREA;
                        imprimirHora(myGLCD, false);
                      }
                  }
                }
                if (posicion.y >= SUPERVISOR_U && posicion.y <= SUPERVISOR_D) {
                    // LLAMADA AL SUPERVISOR

                    if (TipoTarea != SIN_TAREA )
                    {
                      TimerOFF();
                      Time = false;
                      remarcarTarea(myGLCD, TipoTarea, false);
                      // No interesa conocer cuál fue la última tarea
                      TipoTareaAnterior = SIN_TAREA;
                      TipoTareaPresel = TipoTarea;
                      imprimirHora(myGLCD, true);
                    }

                    xSemaphoreTake(xMutexTiempos, portMAX_DELAY);
                    TipoTarea = LLAMADA_SUPERVISOR;
                    xSemaphoreGive(xMutexTiempos);

                    myGLCD.drawBitmap(LLS_X-7, LLS_Y, 96, 96, llamada_icon);
                    myGLCD.setColor(COLOR_LLAMADA_SUPERVISOR);
                    myGLCD.fillRoundRect(LLS_X + 100, LLS_Y, LLS_X + 460, LLS_Y + 96);
                    myGLCD.setColor(BLANCO_4);
                    myGLCD.setBackColor(COLOR_LLAMADA_SUPERVISOR);
                    myGLCD.print("LLAMANDO AL SUPERVISOR", LLS_X + 105, LLS_Y + 35);
                    myGLCD.setColor(WINDOWS_9);

                    for(int i=0;i<RECUADRO;i++)
                        myGLCD.drawRoundRect(LLS_X+100-i, LLS_Y-i, LLS_X+460+i, LLS_Y+96+i);

                    xTaskNotify(xControlHandle, FLAG_CONTROL_LLAMADA_SUPERVISOR, eSetBits);
                }

                if (posicion.y >= FIN_TAREA_U && posicion.y <= FIN_TAREA_D) {
                  // FINALIZAR TAREA
                  if( TipoTarea != SIN_TAREA ) {
                    TimerOFF();
                    Time = false;
                    // No interesa conocer cuál fue la última tarea
                    TipoTareaAnterior = SIN_TAREA;
                    TipoTareaPresel = SIN_TAREA;
                    xSemaphoreTake(xMutexTiempos, portMAX_DELAY);
                    TipoTarea = SIN_TAREA;
                    xSemaphoreGive(xMutexTiempos);
                  }
                  // Deja de enviar periódicamente los tiempos.
                  xTimerStop(xInfoHandle,0);
                  TipoPantalla = pTECLADO;
                  imprimirTeclado(myGLCD);
                }
            }
        }
        else if(TipoPantalla == pTECLADO)
        {
          static uint16_t pos = POSICION_INICIAL;
          static char EmbalajeTotal[CANTIDAD_EMBALAJE_TOTAL] = "";   // Valor máximo = 99.999 artículos
          static bool ErrorMostrado = false;
          char* TextoAdvertencia_1 = "Debe ingresar la";
          char* TextoAdvertencia_2 = "cantidad embalada";

          if( strlen(EmbalajeTotal) < CANTIDAD_EMBALAJE_TOTAL )
          {
            if( posicion.y >= TY11 && posicion.y <= TY12 ) {
              for (int i=0; i<5; i++)
                if ( posicion.x >= (TX0+i*dBOTON) && posicion.x <= (TXF+i*dBOTON) ) {
                    // BOTON i+1
                    if(ErrorMostrado) {
                      uint8_t I = (strlen(TextoAdvertencia_2)/2) + 1;
                      for (int i=0; i<I; i++) {
                          uint16_t P = pos + (i+1) * 32;
                          borrarNumero(myGLCD, P);
                      }
                      ErrorMostrado = false;
                    }
                    char C[2];
                    C[0] = '\0';
                    snprintf(C,sizeof C, "%01d", i+1);
                    strncat(EmbalajeTotal, C, sizeof C);
                    imprimirNumero(myGLCD,pos,i+1);
                }
            }
            else if ( posicion.y >= TY21 && posicion.y <= TY22 ) {
              for (int i=0; i<5; i++)
                if ( posicion.x >= (TX0+i*dBOTON) && posicion.x <= (TXF+i*dBOTON) ) {
                    // BOTON i+6

                  if( ((i+6) != 10) | (strlen(EmbalajeTotal) > 0) ) {
                    // Evita que se escriba un '0' al principio
                    // TABLA DE VERDAD
                    //
                    //    != 10   |   strlen > 0
                    //      F     |       F       |   F
                    //      F     |       V       |   V
                    //      V     |       F       |   V
                    //      V     |       V       |   V
                    //
                    if(ErrorMostrado) {
                      uint8_t I = (strlen(TextoAdvertencia_2)/2) + 1;
                      for (int i=0; i<I; i++) {
                          uint16_t P = pos + (i+1) * 32;
                          borrarNumero(myGLCD,P);
                      }
                      ErrorMostrado = false;
                    }
                    char C[2];
                    C[0] = '\0';
                    if((i+6) == 10) snprintf_P(C,sizeof C, PSTR("0"), C);
                    else            snprintf(C,sizeof C, "%01d", i+6);
                    strncat(EmbalajeTotal, C, sizeof C);
                    imprimirNumero(myGLCD,pos,i+6);
                  }
                }
            }
          }
          if ( posicion.y >= ESP_Y0 && posicion.y <= ESP_YF ) {
            if ( posicion.x >= TX0 && posicion.x <= TXF_BORRAR ) {
                // BOTON BORRAR
                //
                char C[2];
                C[0] = '\0';
                snprintf_P(C,sizeof C, "%s",PSTR(" "));
                EmbalajeTotal[strlen(EmbalajeTotal)-1] = '\0';
                if(!ErrorMostrado)  borrarNumero(myGLCD,pos);
            }
            if ( posicion.x >= TX0_OK && posicion.x <= TXF_OK ) {
              // BOTON CONFIRMAR TAREA
              if (strcmp_P(EmbalajeTotal,PSTR("")) == 0 ) {
                  // Muestra "Debe ingresar la cantidad embalada" en pantalla
                  myGLCD.setBackColor(NEGRO);
                  myGLCD.setColor(ROJO_2);
                  myGLCD.print(TextoAdvertencia_1, POSICION_INICIAL, POS_Y_NUMEROS_TECLADO);
                  myGLCD.print(TextoAdvertencia_2, POSICION_INICIAL, POS_Y_NUMEROS_TECLADO+32);
                  ErrorMostrado = true;
              }
              else {
                TipoPantalla = pRESUMEN_ACTIVIDAD;
                // Guarda EmbalajeTotal en una variable global
                CT = atoi(EmbalajeTotal);
                // Borra EmbalajeTotal
                EmbalajeTotal[0] = '\0';
                for(uint8_t i=1; i<CANTIDAD_EMBALAJE_TOTAL; i++)  EmbalajeTotal[i] = ' ';
                pos = POSICION_INICIAL;
                xTaskNotify(xControlHandle, FLAG_CONTROL_FIN_TAREA, eSetBits);
              }
            }
          }
          if (posicion.x >= MAS_X1 && posicion.x <= MAS_X2 &&
              posicion.y >= MAS_Y1 && posicion.y <= MAS_Y2) {
              // Botón de retroceso a Menu principal de Ejecución
              TipoPantalla = pEJECUCION;
              // Borra EmbalajeTotal
              EmbalajeTotal[0] = '\0';
              for(uint8_t i=1; i<CANTIDAD_EMBALAJE_TOTAL; i++)  EmbalajeTotal[i] = ' ';
              pos = POSICION_INICIAL;
              xTaskNotify(xPantallaHandle, FLAG_PANTALLA_MENU_EJECUCION, eSetBits);
          }
        }
        else if(TipoPantalla == pRESUMEN_ACTIVIDAD) {
          if (posicion.x >= MAS_X1 && posicion.x <= MAS_X2 &&
              posicion.y >= MAS_Y1 && posicion.y <= MAS_Y2) {
              // Botón de retroceso al teclado
              TipoPantalla = pTECLADO;
              imprimirTeclado(myGLCD);
          }
        }
      }
    }

    // Pantalla RFID
    if ( flags & FLAG_PANTALLA_RFID ) {
      _bln_debug_println(F("FLAG_PANTALLA_RFID"));
      TipoPantalla = pRFID;
      mostrarPantallaRFID(myGLCD,false);
    }

    // Articulos
    if ( flags & FLAG_PANTALLA_ARTICULOS ) {
      _bln_debug_println(F("FLAG_PANTALLA_ARTICULOS"));

      if ( TipoPantalla != pARTICULOS ) {
        TipoPantalla = pARTICULOS;

        mostrarPantallaArticulos(myGLCD);
      } else {
        mostrarPantallaMasArticulos(myGLCD);
      }
    }

    if (flags & FLAG_PANTALLA_LISTA_ARTICULO) {

      _bln_debug_println(F("FLAG_PANTALLA_LISTA_ARTICULO"));

      if ( xQueueReceive(xListaArticuloQueue, &Lst, 0 ) == pdTRUE ) {
        TipoPantalla = pLISTA_ARTICULOS;
        if( Lst.T > 1 ){
          // Si hay más de dos variantes
          imprimirListaArticulo(myGLCD, Lst);
        }
        else{
          // Pasa directamente al menú de ejecución
          TipoPantalla = pEJECUCION;
          strcpy(ArticuloEmpaque, Lst.L[0]);
          xTaskNotify(xPantallaHandle, FLAG_PANTALLA_MENU_EJECUCION, eSetBits);
        }
      }
    }

    if (flags & FLAG_PANTALLA_HORA) {
      _bln_debug_println(F("FLAG_PANTALLA_HORA"));
      actualizarTiempos(myGLCD);
    }

    if (flags & FLAG_PANTALLA_LLAMADA_SUPERVISOR) {
      _bln_debug_println(F("FLAG_PANTALLA_LLAMADA_SUPERVISOR"));

      // SE APAGA LA LLAMADA AL SUPERVISOR
      myGLCD.setColor(COLOR_FONDO_PANTALLA);
      myGLCD.fillRect(LLS_X-7-RECUADRO, LLS_Y-RECUADRO, LLS_X+460+RECUADRO, LLS_Y+96+RECUADRO);
      TipoTarea = TipoTareaPresel;  // Vuelve a la tarea preseleccionada
    }

    // Pantalla inicial - Avisa que se está conectando
    if (flags & FLAG_PANTALLA_CONECTANDO_WIFI)
    {
        _bln_debug_println(F("FLAG_PANTALLA_CONECTANDO_WIFI"));
        imprimirPantallaConectandoWiFi(myGLCD);
    }


    // Confirmación del supervisor
    if (flags & FLAG_PANTALLA_RFID_SUPERVISOR)
    {
      _bln_debug_println(F("FLAG_PANTALLA_RFID_SUPERVISOR"));
  //    borrarTiempos();
      TipoPantalla = pRFID;
      mostrarPantallaRFID(myGLCD,true);
    }

    // Avisa en pantalla que se está leyendo la tarjeta y
    // esperando la respuesta del servidor
    if (flags & FLAG_PANTALLA_LEYENDO_RFID)
    {
      char buffer[21];
      myGLCD.setBackColor(COLOR_FONDO_PANTALLA);
      myGLCD.setColor(BLANCO_4);
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("Leyendo Tarjeta RFID"), sizeof(buffer)-1);

      if (TipoPantalla == pRFID)  myGLCD.print(buffer, 450, 200);
      else                        myGLCD.print(buffer, CENTER, 400);
    }

    //
    if (flags & FLAG_PANTALLA_RESUMEN_ACTIVIDAD)
    {
      char buffer[27];
      _bln_debug_println(F("FLAG_PANTALLA_RESUMEN_ACTIVIDAD"));
      myGLCD.fillScr(COLOR_FONDO_PANTALLA);
      myGLCD.setBackColor(COLOR_FONDO_PANTALLA);
      myGLCD.setColor(BLANCO_4);

      buffer[0] = '\0';
      strncat_P(buffer, PSTR("Resumen de la actividad"), sizeof(buffer)-1);
      uint16_t PosY = 30;
      myGLCD.print(buffer, CENTER, PosY);

      ////////////////////////////////////////
      // INFORMACION SOBRE LA TAREA:
      //
      // - Nombre del Operario    :: Articulo
      // - Producción: TIEMPO
      // - Setup:      TIEMPO
      // - ...
      //
      // Cantidad Total Empaquetada
      myGLCD.setColor(WINDOWS_9);

      // NOMBRE OPERARIO
      buffer[0] = '\0';
      xSemaphoreTake(xMutexOperario, portMAX_DELAY);
      strncat(buffer, NombreOperario, sizeof(buffer)-1);
      xSemaphoreGive(xMutexOperario);
      PosY += (myGLCD.getFontYsize() + 50);
      myGLCD.print(buffer, INFO_TXT_X, PosY);
      // ARTICULO
      buffer[0] = '\0';
      strncat(buffer, ArticuloEmpaque, sizeof(buffer)-1);
      xSemaphoreTake(xMutexOperario, portMAX_DELAY);
      uint16_t PosColumnaX = 250 + (strlen(NombreOperario) * myGLCD.getFontXsize()) + 40;
      xSemaphoreGive(xMutexOperario);
      if (PosColumnaX < POS_X_MIN)  PosColumnaX = POS_X_MIN;
      myGLCD.print(buffer, PosColumnaX, PosY);
      // PRODUCCION
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("- Produccion"), sizeof(buffer)-1);
      PosY += (myGLCD.getFontYsize() + 5);
      myGLCD.print(buffer, INFO_TXT_X, PosY);
      // TIEMPO PRODUCCION
      buffer[0] = '\0';
      xSemaphoreTake(xMutexTiempos, portMAX_DELAY);
      uint16_t X = tProd % 3600;
      snprintf(buffer,sizeof buffer, "%02u:%02hu:%02hu", (uint16_t)(tProd/3600), (uint8_t)(X/60), (uint8_t)(X % 60));
      myGLCD.print(buffer, PosColumnaX, PosY);
      // SETUP
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("- Setup"), sizeof(buffer)-1);
      PosY += (myGLCD.getFontYsize() + 5);
      myGLCD.print(buffer, INFO_TXT_X, PosY);
      // TIEMPO SETUP
      buffer[0] = '\0';
      X = tSetup % 3600;
      snprintf(buffer,sizeof buffer, "%02u:%02hu:%02hu", (uint16_t)(tSetup/3600), (uint8_t)(X/60), (uint8_t)(X % 60));
      myGLCD.print(buffer, PosColumnaX, PosY);
      // PARADA POR PROCESO
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("- Parada por Proceso"), sizeof(buffer)-1);
      PosY += (myGLCD.getFontYsize() + 5);
      myGLCD.print(buffer, INFO_TXT_X, PosY);
      // TIEMPO PARADA POR PROCESO
      buffer[0] = '\0';
      X = tPxP % 3600;
      snprintf(buffer,sizeof buffer, "%02u:%02hu:%02hu", (uint16_t)(tPxP/3600), (uint8_t)(X/60), (uint8_t)(X % 60));
      myGLCD.print(buffer, PosColumnaX, PosY);
      // REPROCESO
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("- Reproceso"), sizeof(buffer)-1);
      PosY += (myGLCD.getFontYsize() + 5);
      myGLCD.print(buffer, INFO_TXT_X, PosY);
      // TIEMPO REPROCESO
      buffer[0] = '\0';
      X = tReproc % 3600;
      snprintf(buffer,sizeof buffer, "%02u:%02hu:%02hu", (uint16_t)(tReproc/3600), (uint16_t)(X/60), (uint16_t)(X % 60));
      myGLCD.print(buffer, PosColumnaX, PosY);
      xSemaphoreGive(xMutexTiempos);
      // INFO CANTIDAD EMPAQUETADA
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("Cantidad empaquetada:"), sizeof(buffer)-1);
      PosY += (myGLCD.getFontYsize() + 5);
      myGLCD.print(buffer, INFO_TXT_X, PosY);
      // CANTIDAD EMPAQUETADA
      buffer[0] = '\0';
      snprintf(buffer, sizeof buffer, "%u", CT);
      myGLCD.print(buffer, PosColumnaX, PosY);
      ////////////////////////////////////////
      myGLCD.setColor(BLANCO_4);
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("Coloque su tarjeta RFID"), sizeof(buffer)-1);
      PosY += (myGLCD.getFontYsize() + 25);
      myGLCD.print(buffer, CENTER, PosY);

      buffer[0] = '\0';
      strncat_P(buffer, PSTR("para confirmar la cantidad"), sizeof(buffer)-1);
      myGLCD.print(buffer, CENTER, PosY + myGLCD.getFontYsize());

      dibujarBotonAtras(myGLCD);
    }

    // Imprime el menú principal de ejecución
    if (flags & FLAG_PANTALLA_MENU_EJECUCION)
    {
      _bln_debug_println(F("FLAG_PANTALLA_MENU_EJECUCION"));
      imprimirMenuPrincipal(myGLCD, ArticuloEmpaque);
    }

    // Cambia la información del texto en la pantalla de resumen de actividad
    if (flags & FLAG_PANTALLA_INFO_ACTIVIDAD)
    {
      _bln_debug_println(F("FLAG_PANTALLA_INFO_ACTIVIDAD"));
      char buffer[22];
      buffer[0] = '\0';
      strncat_P(buffer, PSTR("  Tarjeta rechazada  "), sizeof(buffer)-1);
      if (TipoPantalla == pRFID)  myGLCD.print(buffer, 450, 200);
      else                        myGLCD.print(buffer, CENTER, 400);
    }

// ###########################################################################

    _bln_debug_print(F(">pantalla-task=")); _bln_debug_println(uxTaskGetStackHighWaterMark(NULL));
    vTaskDelayUntil(&xLastWakeTime, DELAY_TAREA_PANTALLA);
  }
}

#endif

