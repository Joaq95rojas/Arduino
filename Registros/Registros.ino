#define PIN_LED   13

#define BIT_LED(i)   digitalPinToBitMask(i)
#define OUT_LED(i)   portOutputRegister(digitalPinToPort(i))
#define REG_LED(i)   portModeRegister(digitalPinToPort(i))

unsigned long old_time = 0;

void setup() 
{
  Serial.begin(9600);
  old_time = millis();

  volatile uint8_t *reg,*out;
  
  reg = REG_LED(PIN_LED);
  out = OUT_LED(PIN_LED);
  uint8_t oldSREG = SREG;
  cli();
  *reg |= BIT_LED(PIN_LED);
  SREG = oldSREG;   
}

void loop() 
{
  volatile uint8_t* led = OUT_LED(PIN_LED);
  
  *led |= BIT_LED(PIN_LED);
  delay(1000);    
  *led &= ~BIT_LED(PIN_LED);
  delay(1000);     

  Serial.println(millis()-old_time);
}
