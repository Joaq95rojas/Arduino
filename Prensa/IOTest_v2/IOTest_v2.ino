/*
 * Prueba del hardware IO para las prensas.
 *
 * Si se acciona un pulsador (21) -> se envía por serie la lectura.
 * 
 * En el caso del botón de emergencia, cuando se presiona, se ejecuta un timer de 
 * 5 segundos. Cuando pasa ese tiempo, se activa la parada desde la Arduino.
 */

 // TODO: 1) El programa no lee la parada. -> LM358 mal colocado.                           OK
 //       2) El botòn de emergencia no es reconocido por la prensa. Aunque son 2
 //       conectados en paralelo, sólo a uno deja de reconocerlo. -> Colocar LM358          OK
 //       3) Lo ideal serìa frenar el motor cuando ángulo=0º para automàtico o
 //       después de 5 segundos para manual.                                                PROBAR
 //       4) Cuando se presiona EMERGENCIA, y despuès MARCHA, no se activa bien
 //       la interrupciòn de la LEVA_LIBRE.
 //       5) Apenas apreta MARCHA se detecta un PMI. (Limpiar interrupciones pendientes?)   PROBAR
 //       EIFR  |= (1<<INTF1) -> Después de attachInterrupt().
 //       En la Arduino MEGA2560:
 //       Header pin 2 (INT0) use EIFR = (1 << INTF4);
//        Header pin 3 (INT1) use EIFR = (1 << INTF5);
//        Header pin 21 (INT2) use EIFR = (1 << INTF0);
//        Header pin 20 (INT3) use EIFR = (1 << INTF1);
//        Header pin 19 (INT4) use EIFR = (1 << INTF2);
//        Header pin 18 (INT5) use EIFR = (1 << INTF3);
//        6)AGREGAR CIRCUITO PARA LECTURA DE ANGULO Y FRENAR CUANDO LA PRENSA LLEGUE A 0°.
//        7) El programa detecta que se presiona el botòn de EMERGENCIA constantemente luego
//        de presionar el botòn de MARCHA.

/*
 * CÓDIGO DE COLORES DEL ENCODER INCREMENTAL:
 * 
 * ROJO:   24V
 * NEGRO:  GND
 * VERDE:  A   // E3
 * BLANCO: B   // E4
 * AZUL:   Z   // E5
 */
 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Streaming.h>

// Pulsadores
#define bPAUSAR               9
#define bEMERGENCIA           8
#define bMARCHA               7
#define bPARADA               6
// Interrupción de pulsadores
#define PULSADOR              21      // Pin 21 (SCL) -> INT2
#define nINT_P                digitalPinToInterrupt(PULSADOR)
#define TIEMPO_DEBOUNCE_MS    100
// Leva Libre
#define LEVA_LIBRE            19
#define nINT_LL               digitalPinToInterrupt(LEVA_LIBRE)
#define TIEMPO_LEVA_LIBRE_MS  10
// Contador de ángulos
#define PIN_CANAL_A           18
#define INT_CANAL_A           digitalPinToInterrupt(PIN_CANAL_A)
#define PIN_CANAL_B           5
// Timer
#define OFF                   false
#define ON                    true
// Estados de la tarea
#define eMARCHA               1
#define ePARADA               2
#define eEMERGENCIA           3

void inline habTimer2(bool B) {B? TCCR2B = 0x05 : TCCR2B = 0x00;};
void inline pararPrensa(bool B) {B? digitalWrite(bPAUSAR, LOW):digitalWrite(bPAUSAR, HIGH);}
void inline habContAngulos() {attachInterrupt(INT_CANAL_A, handlerLevaCanalA, FALLING);};
void inline desHabContAngulos() {detachInterrupt(INT_CANAL_A);};
void inline habLevaLibre() {attachInterrupt(nINT_LL, LevaLibre, RISING);habContAngulos();};
void inline desHabLevaLibre() {detachInterrupt(nINT_LL);desHabContAngulos();};

bool esperarPMS = false;
uint8_t estado = ePARADA;
uint16_t contGrados = 0;

void setup(void)
{
  // Pulsadores
  pinMode(bEMERGENCIA, INPUT);
  pinMode(bMARCHA, INPUT);
  pinMode(bPARADA, INPUT);
  pinMode(bPAUSAR, OUTPUT);
  pararPrensa(OFF);
  // Contador de ángulos
  pinMode(PIN_CANAL_A, INPUT);
  pinMode(PIN_CANAL_B, INPUT);
  // Interrupciones
  pinMode(PULSADOR, INPUT);
  pinMode(LEVA_LIBRE, INPUT);
  attachInterrupt(nINT_P, handler, RISING);
  // Serial Test
  Serial.begin(19200);
  Serial << "Serial Start" << endl;
  Serial << "############" << endl;
  // Timer
  initTimer2(OFF);
  //initTimer2(ON);
}

void loop(void) 
{
  if(esperarPMS && contGrados == 0) {
    pararPrensa(ON);
    esperarPMS = false;
  }
  
}

void handler(void)
{
  delay(TIEMPO_DEBOUNCE_MS);
  if( digitalRead(PULSADOR) ) {
      // Se presionó un botón. Detecto cuál fue:
      if(digitalRead(bEMERGENCIA)) {
        Serial << "--- EMERGENCIA ON  ---" << endl;
        estado = eEMERGENCIA;
        habTimer2(ON);  // Enable Timer 2
      }
      else if(digitalRead(bMARCHA)) {
        Serial << "-> Marcha" << endl;
        habLevaLibre();
        estado = eMARCHA;
      }
      else if(digitalRead(bPARADA)) {
        Serial << "-> Parada" << endl;
        estado = ePARADA;
        desHabLevaLibre();
      }
      else {
        Serial << "ERROR: Lectura" << endl;
      }
  }
}

uint16_t C = 0, S = 0;

ISR(TIMER2_OVF_vect)  // 1 ms
{
  if(++C >= 1000) {  // 1 segundo
    C = 0;
    ++S;
    if(S >= 4 && estado == ePARADA) {
      S = 0;
      esperarPMS = true;
      Serial << "Prensa parada por el Supervisor." << endl;
    }
    if(S >= 5 && estado == eEMERGENCIA) {
      S = 0;
      Serial << "--- EMERGENCIA OFF ---" << endl;
      habTimer2(OFF);  // Disable Timer 2
      estado = eMARCHA;
    }
  }
  TCNT2 = 130;
  TIFR2 = 0x00;
}

uint16_t PuntoMuertoInferior = 0;

void LevaLibre(void) 
{
  /*
  // PUNTO MUERTO INFERIOR
  // Lectura de una revolución de la leva libre (= 1 golpe)
  delay(TIEMPO_LEVA_LIBRE_MS);
  if(digitalRead(LEVA_LIBRE)) {
    Serial << "PMI: " << ++PuntoMuertoInferior << endl;
    if( PuntoMuertoInferior == 5 ) {
      estado = ePARADA;
      Serial << "Tarea terminada." << endl;
      // Esperar 2 segundos -> Lanzar timer o delay()
      habTimer2(ON);
    }
  }
  */
  contGrados = 0;
}

void initTimer2(bool B)
{
  TCCR2B = 0x00;
  TCNT2  = 130;
  TIFR2  = 0x00;
  TIMSK2 = 0x01;
  TCCR2A = 0x00;
  habTimer2(B);
}

void handlerLevaCanalA(void)
{
  // Leer Canal B
  if( digitalRead(PIN_CANAL_B) ) {
    ++contGrados;
  }
  else {
    --contGrados;
  }
  //Serial << "G: " << contGrados << endl;
}

