#include <Streaming.h>

#define ENTRADA 0
#define SALIDA  1

#define PIN_MOTOR_D1 6
#define PIN_MOTOR_D2 7
#define PIN_MOTOR_D3 8 
#define PIN_MOTOR_D4 9
#define PIN_MOTOR_T1 10 
#define PIN_MOTOR_T2 11
#define PIN_MOTOR_T3 12 
#define PIN_MOTOR_T4 13

#define BIT_PIN(i)  digitalPinToBitMask(i)
#define OUT_PIN(i)  portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)  portModeRegister(digitalPinToPort(i))
#define IN_PIN(i) *portInputRegister(digitalPinToPort(i))
;
void girarRobotCW( int velMotor )
{
  analogWrite( PIN_MOTOR_D3, velMotor );
  _digWrite( PIN_MOTOR_D4, LOW );
  analogWrite( PIN_MOTOR_T4, velMotor );
  _digWrite( PIN_MOTOR_T3, LOW );

  _digWrite( PIN_MOTOR_D1, LOW );
  _digWrite( PIN_MOTOR_D2, LOW );
  _digWrite( PIN_MOTOR_T1, LOW );
  _digWrite( PIN_MOTOR_T2, LOW );

}  // Fin de girarRobotCW()

void girarRobotCCW( int velMotor )
{
  analogWrite( PIN_MOTOR_D2, velMotor );
  _digWrite( PIN_MOTOR_D1, LOW );
  analogWrite( PIN_MOTOR_T1, velMotor );
  _digWrite( PIN_MOTOR_T2, LOW );

  _digWrite( PIN_MOTOR_D3, LOW );
  _digWrite( PIN_MOTOR_D4, LOW );
  _digWrite( PIN_MOTOR_T3, LOW );
  _digWrite( PIN_MOTOR_T4, LOW );

}  // Fin de girarRobotCCW()

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

void setup() 
{
  configPin( PIN_MOTOR_D1, SALIDA );
  configPin( PIN_MOTOR_D2, SALIDA );
  configPin( PIN_MOTOR_D3, SALIDA );
  configPin( PIN_MOTOR_D4, SALIDA );

  configPin( PIN_MOTOR_T1, SALIDA );
  configPin( PIN_MOTOR_T2, SALIDA );
  configPin( PIN_MOTOR_T3, SALIDA );
  configPin( PIN_MOTOR_T4, SALIDA );
}

void loop() 
{
  for (int vel = 255; vel >= 0; --vel)
  {
    girarRobotCCW(vel);
    delay(100);
  }
  for (int vel = 255; vel >= 0; --vel)
  {
    girarRobotCW(vel);
    delay(100);
  }

}
