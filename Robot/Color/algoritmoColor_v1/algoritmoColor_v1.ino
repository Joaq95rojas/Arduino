//////////////////////////////////////////////////////////////////////////////////////
// algoritmoColor_v1.ino
// **********************************************************************************
// - El programa comienza y finaliza en una base BLANCA.
// - Si detecta AMARILLO, el robot avanza.
// - Si detecta AZUL, el robot disminuye su velocidad progresivamente.
// - Si detecta ROJO, el robot se detiene.

#include <TimerOne.h>
#include <Streaming.h>

#define S0     45
#define S1     47
#define S2     49
#define S3     51
#define OUT    2

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
#define IN_PIN(i)  *portInputRegister(digitalPinToPort(i))

#define ENTRADA 0
#define SALIDA  1

int   g_count = 0;    // count the frequecy
int   g_array[3];     // store the RGB value
int   g_flag = 0;     // filter of RGB queue
float g_SF[3];        // save the RGB Scale factor

bool enLaMeta = false;  // Si detecta blanco y enLaMeta=true, es porque llego a destino.
                        // Esta variable se activa cuando se detecta AMARILLO.
int velRobot = 0;

//////////////////////////////////////////////////////////////////////////////////////
// FUNCIONES CREADAS
//

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

}  // Fin de configPin()


void _digWrite( uint8_t pNum, uint8_t statePin )
{
  volatile uint8_t* out = OUT_PIN(pNum);

  if( statePin )  *out |= BIT_PIN(pNum);
  else            *out &= ~BIT_PIN(pNum);

}  // Fin de _digWrite()

void TSC_Count()
{
  g_count ++;
}


void TSC_Init()
{
  // Init TSC230 and setting Frequency.
  configPin(S0, OUTPUT);
  configPin(S1, OUTPUT);
  configPin(S2, OUTPUT);
  configPin(S3, OUTPUT);
  configPin(OUT, INPUT);
  
  configPin(13,OUTPUT);
  _digWrite(13,LOW);
 
  _digWrite(S0, LOW);  // OUTPUT FREQUENCY SCALING 2%
  _digWrite(S1, HIGH); 

}  // Fin de TSC_Init()

 
void TSC_FilterColor(int Level01, int Level02)
{
  // Select the filter color
  if(Level01 != 0)  Level01 = HIGH;
  if(Level02 != 0)  Level02 = HIGH;
 
  _digWrite(S2, Level01); 
  _digWrite(S3, Level02); 

}  // Fin de TSC_FilterColor()


void TSC_Callback()
{
  switch(g_flag)
  {
    case 0:  TSC_WB(LOW, LOW);              // Filter without Red
             break;
    case 1:  g_array[0] = g_count;
             TSC_WB(HIGH, HIGH);            // Filter without Green
             break;
    case 2:  g_array[1] = g_count;
             TSC_WB(LOW, HIGH);             // Filter without Blue
             break; 
    case 3:  g_array[2] = g_count;
             TSC_WB(HIGH, LOW);             // Clear(no filter)   
             break;
   default:  g_count = 0;
             break;
  }

}  // Fin de TSC_Callback()
 
void TSC_WB(int Level0, int Level1)      // White Balance
{
  g_count = 0;
  g_flag ++;
  TSC_FilterColor(Level0, Level1);
  Timer1.setPeriod(40000);             // Periodo de 40 ms

}  // Fin de TSC_WB()

void cambiarVelocidad( int velMotor)
{
  _digWrite( PIN_MOTOR_D1, LOW );
  analogWrite( PIN_MOTOR_D2, velMotor );
  analogWrite( PIN_MOTOR_D3, velMotor );
  _digWrite( PIN_MOTOR_D4, LOW );
  analogWrite( PIN_MOTOR_T1, velMotor );
  _digWrite( PIN_MOTOR_T2, LOW );
  _digWrite( PIN_MOTOR_T3, LOW );
  analogWrite( PIN_MOTOR_T4, velMotor );
  Serial << velRobot << endl;
  
}  // Fin de cambiarVelocidad()
///////////////////////////////////////////////////////////////////////////////////////
// FUNCIONES DE ARDUINO
//

void setup()
{
  TSC_Init();
  Serial.begin(9600);
  Timer1.initialize();
  Timer1.attachInterrupt(TSC_Callback); 
  attachInterrupt(0, TSC_Count, RISING);  
 
  delay(1200);
 
  for(int i=0; i<3; i++)  Serial << g_array[i] << endl;
 
  g_SF[0] = 255.0/ g_array[0];     //R Scale factor
  g_SF[1] = 255.0/ g_array[1] ;    //G Scale factor
  g_SF[2] = 255.0/ g_array[2] ;    //B Scale factor
 
  Serial << g_SF[0] << endl;
  Serial << g_SF[1] << endl;
  Serial << g_SF[2] << endl;
  
  configPin( PIN_MOTOR_D1, SALIDA );
  configPin( PIN_MOTOR_D2, SALIDA );
  configPin( PIN_MOTOR_D3, SALIDA );
  configPin( PIN_MOTOR_D4, SALIDA );

  configPin( PIN_MOTOR_T1, SALIDA );
  configPin( PIN_MOTOR_T2, SALIDA );
  configPin( PIN_MOTOR_T3, SALIDA );
  configPin( PIN_MOTOR_T4, SALIDA );

}  // Fin de setup()
 
void loop()
{
  int R,G,B;
  g_flag = 0;

  R = int(g_array[0] * g_SF[0]);
  G = int(g_array[1] * g_SF[1]);
  B = int(g_array[2] * g_SF[2]);

  if( (R > 2*B) && (G>1.5*B) )  // Color AMARILLO
  {
     enLaMeta = true;
     // Avanzar con aumento progresivo de velocidad
     velRobot += 40;
     if( velRobot >= 255 ) velRobot = 255;
     cambiarVelocidad(velRobot);
  }
  else if( (R>200) && (G>200) && (B>200) )  // Color BLANCO
  {
    if( enLaMeta )
    {
      Serial << "LLEGADA" << endl;
      // Frenar motores suavemente
      velRobot -= 80;
      if( velRobot <= 0 )  velRobot = 0;
      cambiarVelocidad(velRobot);
    }  
    else Serial << "SALIDA" << endl; 
  }
  else if( (R>2*G) && (R>2*B) )  // Color ROJO
  {
    // Frenar motores bruscamente
    velRobot -= 130;
    if( velRobot <= 0 )  velRobot = 0;
    cambiarVelocidad(velRobot);
  }
  else if( (B>R) && (B>G) )  // Color AZUL
  {
    // Disminuir la velocidad progresivamente
    velRobot -= 40;
    if( velRobot <= 0 )  velRobot = 0;
    cambiarVelocidad(velRobot);
  }
  
  delay(250);

}  // Fin de loop()
