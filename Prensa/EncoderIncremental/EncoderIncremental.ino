////////////////////////////////////////////////////////////////////////
// Programa que cuenta las vueltas de un encoder rotativo incremental
// que tiene dos señales: Canal A y Canal B.
//

#include <LiquidCrystal.h>
#include <Streaming.h>
#include <Wire.h>

#define READ(port,pin) (PIN ## port & (1<<pin))

#define PPR (360/360)
#define INT_A 2
#define PIN_A 21
#define PIN_B 18  // Pin 3

uint16_t contGrados = 0;
uint16_t ref=0;
uint16_t contVueltas=1;
bool primeraVez = true;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup()
{
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  // El programa debe mirar la lectura de B cuando A esté con flanco
  // decreciente.
 
  Serial.begin(57600);
  Serial<< F("Serial OK") << endl;
  lcd.begin(16, 2);
  lcd.setCursor(0,1);
  lcd << F("Cont: ");
  lcd.setCursor(0,0);
  lcd << F("Vueltas: ");
 attachInterrupt(INT_A, handlerCanalA, FALLING);
}

void loop() { 
  int16_t diff = ref-contGrados;
  if(diff<0) diff=(-diff);
  Serial << "diff: " <<diff<< endl;
  if((diff%PPR)==0 && diff!=0 && diff>=PPR && !primeraVez)
  {
    contVueltas++;
    contGrados = 0;
    ref = 0;
    primeraVez=true;
//    if((contVueltas % 90)==0)
 //   {
      lcd.setCursor(9,0);
      lcd << contVueltas << "   ";
   // }
    if(contVueltas==360) contVueltas = 0;
  }
}
/////////////////////////////////////////////////////////////////////////
void handlerCanalA(void)
{
  /////////////////////////////////////
  // Leer canal B.
  //
  READ(D,3)? contGrados++ : contGrados--;

  if(primeraVez)
  {
//    contGrados=0;
//    ref = contGrados;
    primeraVez = false;
//    Serial<<"REF: "<<ref<<endl;
  }
  /*
  Serial << contGrados << endl;
  lcd.setCursor(6,1);
  lcd << contGrados;
  */

} // Fin de handlerCanalA()
