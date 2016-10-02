#include <Streaming.h>
#include <avr/interrupt.h>  
#include <avr/io.h>

#define S0     45
#define S1     47
#define S2     49
#define S3     51
#define OUT    2

#define BIT_PIN(i)  digitalPinToBitMask(i)
#define OUT_PIN(i)  portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)  portModeRegister(digitalPinToPort(i))
#define IN_PIN(i)  *portInputRegister(digitalPinToPort(i))

int   g_count = 0;    // count the frequecy
int   g_array[3];     // store the RGB value
int   g_flag = 0;     // filter of RGB queue
float g_SF[3];        // save the RGB Scale factor

int count = 0;

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
}

void _digWrite( uint8_t pNum, uint8_t statePin )
{
  volatile uint8_t* out = OUT_PIN(pNum);

  if( statePin )  *out |= BIT_PIN(pNum);
  else            *out &= ~BIT_PIN(pNum);
}

void TSC_Count()
{
  g_count ++;
}

// Init TSC230 and setting Frequency.
void TSC_Init()
{
  configPin(S0, OUTPUT);
  configPin(S1, OUTPUT);
  configPin(S2, OUTPUT);
  configPin(S3, OUTPUT);
  configPin(OUT, INPUT);
  
  configPin(13,OUTPUT);
  _digWrite(13,LOW);
 
  _digWrite(S0, LOW);  // OUTPUT FREQUENCY SCALING 2%
  _digWrite(S1, HIGH); 
}

// Select the filter color 
void TSC_FilterColor(int Level01, int Level02)
{
  if(Level01 != 0)  Level01 = HIGH;
  if(Level02 != 0)  Level02 = HIGH;
 
  _digWrite(S2, Level01); 
  _digWrite(S3, Level02); 
}

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) 
{
  count++;           // Increments the interrupt counter
  if(count >= 110)    //// 120 ////
  {
    switch(g_flag)
    {
      case 0: 
           TSC_WB(LOW, LOW);              //Filter without Red
           break;
      case 1:
           g_array[0] = g_count;
           TSC_WB(HIGH, HIGH);            //Filter without Green
           break;
      case 2:
           g_array[1] = g_count;
           TSC_WB(LOW, HIGH);             //Filter without Blue
           break;
      case 3:
           g_array[2] = g_count;
           TSC_WB(HIGH, LOW);             //Clear(no filter)   
           break;
     default:
           g_count = 0;
           break;
    }
    count = 0;
  }
  TCNT2 = 130;           // Reset Timer to 130 out of 255
  TIFR2 = 0x00;          // Timer2 INT Flag Reg: Clear Timer Overflow Flag
}
 
void TSC_WB(int Level0, int Level1)      // White Balance
{
  g_count = 0;
  g_flag ++;
  TSC_FilterColor(Level0, Level1);
}
 
void setup()
{
  TSC_Init();
  Serial.begin(9600);
  
  //Setup Timer2 to fire every 1ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
  
  attachInterrupt(0, TSC_Count, RISING);  
 
  delay(1500);   //// 1500 ////
 
  for(int i=0; i<3; i++)  Serial << g_array[i] << endl;
 
  g_SF[0] = 255.0/ g_array[0];     //R Scale factor
  g_SF[1] = 255.0/ g_array[1] ;    //G Scale factor
  g_SF[2] = 255.0/ g_array[2] ;    //B Scale factor
 
  Serial << g_SF[0] << endl;
  Serial << g_SF[1] << endl;
  Serial << g_SF[2] << endl;
}
 
void loop()
{
  int Color[3];
  g_flag = 0;
  Serial << " " << endl;
  for(int i=0; i<3; i++)
  {
     Color[i] = int(g_array[i] * g_SF[i]);
     Serial << Color[i] << "  "; 
  }
     
  if( (Color[0] > 2* Color[2]) && (Color[1]>1.5*Color[2]) )
  {
     Serial << "<-- Color Amarillo";
     _digWrite(13,HIGH);
  }
  else _digWrite(13,LOW);
  
  delay(350);  //// 350 ////
}
