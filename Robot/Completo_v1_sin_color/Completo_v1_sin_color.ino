//////////////////////////////////////////////////////////////////////////////////
// Completo_v1_sin_color.ino
// ------------------------------------------------------------------------------
// * El robot se mueve hacia adelante hasta que, con el sensor ultrasónico, detecte
//   un obstáculo. Ante esto, se quedará quieto hasta que tenga el camino libre.
// * También se agregó el LDR.
//

//////////////////////////////////////////////////////////////////////////////////
// INCLUDES
//
#include <Streaming.h>
#include <Servo.h> 

//////////////////////////////////////////////////////////////////////////////////
// DEFINES
//
#define ENTRADA 0
#define SALIDA  1

#define PIN_LDR   0
#define PIN_SERVO 3
#define PIN_ECHO  4
#define PIN_TRIG  5
#define PIN_LED   13

#define PIN_MOTOR_D1 6
#define PIN_MOTOR_D2 7
#define PIN_MOTOR_D3 8 
#define PIN_MOTOR_D4 9
#define PIN_MOTOR_T1 10 
#define PIN_MOTOR_T2 11
#define PIN_MOTOR_T3 12 
#define PIN_MOTOR_T4 13

#define DISTANCIA_MINIMA  16
#define OSCURIDAD_MAXIMA  500

#define BIT_PIN(i)  digitalPinToBitMask(i)
#define OUT_PIN(i)  portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)  portModeRegister(digitalPinToPort(i))
#define IN_PIN(i) *portInputRegister(digitalPinToPort(i))

//////////////////////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES
//
Servo miServo;

//////////////////////////////////////////////////////////////////////////////////
// FUNCIONES PROPIAS
//
void robotAdelante()
{ 
  _digWrite(PIN_MOTOR_T4, HIGH);
  _digWrite(PIN_MOTOR_T3, LOW); 
  _digWrite(PIN_MOTOR_T1, HIGH);
  _digWrite(PIN_MOTOR_T2, LOW);
  
  _digWrite(PIN_MOTOR_D1, LOW);
  _digWrite(PIN_MOTOR_D2, HIGH); 
  _digWrite(PIN_MOTOR_D3, HIGH); 
  _digWrite(PIN_MOTOR_D4, LOW);
  
} // Fin de robotAdelante()

void robotAtras()
{ 
  _digWrite(PIN_MOTOR_T1, LOW);
  _digWrite(PIN_MOTOR_T2, HIGH);
  _digWrite(PIN_MOTOR_T3, HIGH);
  _digWrite(PIN_MOTOR_T4, LOW);
  
  _digWrite(PIN_MOTOR_D1, HIGH);
  _digWrite(PIN_MOTOR_D2, LOW);
  _digWrite(PIN_MOTOR_D3, LOW);
  _digWrite(PIN_MOTOR_D4, HIGH);
  
} // Fin de robotAtras()

void pararRobot( void )
{ 
  _digWrite(PIN_MOTOR_T1, LOW);
  _digWrite(PIN_MOTOR_T4, LOW);
  _digWrite(PIN_MOTOR_T3, LOW);
  _digWrite(PIN_MOTOR_T2, LOW);
  
  _digWrite(PIN_MOTOR_D1, LOW);
  _digWrite(PIN_MOTOR_D4, LOW);
  _digWrite(PIN_MOTOR_D3, LOW);
  _digWrite(PIN_MOTOR_D2, LOW);
  
  do{ }while( lecturaHCSR04() < DISTANCIA_MINIMA );

} // Fin de pararRobot()

long lecturaHCSR04()
{
  long tiempoEco, distMedida;
  _digWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  _digWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  _digWrite(PIN_TRIG, LOW);

  tiempoEco = pulseIn(PIN_ECHO, HIGH);
  distMedida = (tiempoEco/2) / 29.1;

  return distMedida;

} // Fin de lecturaHCSR04()

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

//////////////////////////////////////////////////////////////////////////////////
// INICIALIZACIÓN
//
void setup()
{
  Serial.begin (9600);

  configPin( PIN_TRIG, SALIDA );
  configPin( PIN_ECHO, ENTRADA );
  configPin( PIN_MOTOR_D2, SALIDA );
  configPin( PIN_MOTOR_D1, SALIDA );
  configPin( PIN_MOTOR_D3, SALIDA );
  configPin( PIN_MOTOR_D4, SALIDA );

  configPin( PIN_MOTOR_T1, SALIDA );
  configPin( PIN_MOTOR_T2, SALIDA );
  configPin( PIN_MOTOR_T3, SALIDA );
  configPin( PIN_MOTOR_T4, SALIDA );

  configPin( PIN_LED, OUTPUT );
  _digWrite( PIN_LED, LOW );
  
  miServo.attach(PIN_SERVO);
  miServo.write(90);        // Se posiciona inicialmente en 90 grados.

} // Fin de setup()

//////////////////////////////////////////////////////////////////////////////////
// FUNCIÓN PRINCIPAL
//
void loop()
{
  int16_t pos;
  uint16_t luzLeida;
  
  for(pos = 0; pos <= 180; pos += 1)
  {
    miServo.write(pos);
    delay(15);

    if( lecturaHCSR04() < DISTANCIA_MINIMA ) { pararRobot(); }
    else robotAdelante();

    luzLeida = analogRead( PIN_LDR );
    if( luzLeida > OSCURIDAD_MAXIMA )
    {
      _digWrite( PIN_LED, HIGH );
      Serial <<  "Luz encendida (" << luzLeida << ")" << endl;
    }
    else _digWrite( PIN_LED, LOW );
  } 
  
  for(pos = 180; pos>=0; pos-=1)
  {                                
    miServo.write(pos);
    delay(15);

    if( lecturaHCSR04() < DISTANCIA_MINIMA ) { pararRobot(); }
    else robotAdelante();

    luzLeida = analogRead( PIN_LDR );
    if( luzLeida > OSCURIDAD_MAXIMA )
    {
      _digWrite( PIN_LED, HIGH );
      Serial <<  "Luz encendida (" << luzLeida << ")" << endl;
    }
    else _digWrite( PIN_LED, LOW );
  }

} // Fin de loop()


