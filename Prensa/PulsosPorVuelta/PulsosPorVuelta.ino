////////////////////////////////////////////////////////////////
// Programa que cuenta la cantidad de pulsos (grados mecánicos)
// que lee el encoder en una revolución (360° eléctricos).
// ------------------------------------------------------------
// Cuando lee el flanco creciente de la leva libre, comienza a
// contar los pulsos hasta el próximo flanco, donde resetea el
// contador.
//

//#include <LiquidCrystal.h>
#include <Streaming.h>
//#include <Wire.h>

// Shaft Encoder
#define PIN_A     18//19
#define INT_A 		digitalPinToInterrupt(PIN_A)
#define PIN_B 		5//3
// Leva Libre
#define PIN_LEVA	19//18
#define INT_LEVA  digitalPinToInterrupt(PIN_LEVA)

uint16_t contGrados = 0;

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup()
{
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(PIN_LEVA, INPUT);
  attachInterrupt(INT_A, handlerCanalA, FALLING);
  attachInterrupt(INT_LEVA, handlerLevaLibre, RISING);
  Serial.begin(19200);
  Serial<< F("Serial OK") << endl;
//  lcd.begin(16, 2);
//  lcd.setCursor(0,1);
//  lcd << F("Cont: ");
}

void loop() {}

void handlerCanalA(void)
{
  /////////////////////////////////////
  // Leer canal B.
  //

  if( !digitalRead(PIN_B) )
  {
    // Incrementa el contador de grados
    ++contGrados;
  }
  else{
    // Decrementa el contador de grados
    --contGrados;
  }
  if(contGrados>=360) {
    contGrados = 0;
  }
  if(contGrados==0){
      Serial << "-> LEVA LIBRE EN EL PUNTO MUERTO SUPERIOR" << endl;
  }
  //Serial << contGrados << endl;
//  lcd.setCursor(6,1);
//  lcd << contGrados;

} // Fin de handlerCanalA()

void handlerLevaLibre (void)
{
	contGrados = 0;

}	// Fin de handlerLevaLibre()
