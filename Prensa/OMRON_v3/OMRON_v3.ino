//////////////////////////////////////////////////////////////////////////
// Lee los botones de movimiento (arriba y abajo) de la prensa para saber
// la dirección de movimiento
//

#include <Streaming.h>

#define PIN_ABAJO    //XX
#define PIN_ARRIBA   //XX

uint16_t count = 0;
bool stringComplete = false;
float ref;
int8_t dir;

void setup()
{
  Serial.begin(57600);
  Serial << "CUENTA" << endl;
  Serial << "###############" << endl;
  Serial << endl;
}

void loop()
{
  ///////////////////////////////
  // Ingreso de posición inicial
  //
  Serial << "Ingrese la posicion inicial XX.XX [mm]" << endl;
  Serial << "Luego, presione ENTER" << endl;
  while( !Serial.available() );

  ref = readSerial();
  Serial << "Posicion: " << ref << endl;
  
  ///////////////////////////
  // Lectura del movimiento
  //
  attachInterrupt(4, omronHandler, RISING);
  attachInterrupt(5, dirHandler, RISING);
  
  while(1);
}
//##############################################################
void omronHandler()
{
  delayMicroseconds(250);
  if( digitalRead(19) )
  {  
    count++;
    // 1C / 9 ppm + REF = 0.9 mm + ref [mm]
    Serial << count / 9.0 + dir * ref << " mm" << endl;
  }
}
//##############################################################
float readSerial() 
{
  float divisor = 1.0;
  uint8_t cantDecimales = 0;
  bool punto = false;
  String S = "";
  
  while( Serial.available() ) 
  {
    char inChar = (char) Serial.read(); 
    
    if(inChar == '.')  punto = true;
    else  S += inChar;
    if( punto )  cantDecimales++;
    delay(50);
  }
  if(cantDecimales) cantDecimales--;
  for(int i=0;i<cantDecimales;i++)  divisor = divisor * 10.0;
  
  return S.toFloat()/divisor;
}

void dirHandler()
{
  delay(50);
  //////////////////
  if(digitalRead(PIN_ARRIBA))  dir = 1;
  else if(digitalRead(PIN_ABAJO))  dir = -1;
  else Serial << "### Error de Lectura ###" << endl;
  
}
