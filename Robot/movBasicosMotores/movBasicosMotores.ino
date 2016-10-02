/* 
 Ejemplo de control de motor DC usando modulo L298
 http://electronilab.co/tienda/driver-dual-para-motores-full-bridge-l298n/
 
 El programa activa el motor en un sentido por 4 segundos, 
 para el motor por 500 ms, activa el motor en sentido inverso por 4 segundos 
 y se detiene por 5 segundos. Luego repite la acción indefinidamente.
*/
#define PIN_MOTOR_D1 6
#define PIN_MOTOR_D2 7
#define PIN_MOTOR_D3 8 
#define PIN_MOTOR_D4 9
#define PIN_MOTOR_T1 10 
#define PIN_MOTOR_T2 11
#define PIN_MOTOR_T3 12 
#define PIN_MOTOR_T4 13


void ADELANTE()
{ 
  digitalWrite (PIN_MOTOR_T1, HIGH);
  digitalWrite (PIN_MOTOR_T2, LOW); 
  digitalWrite (PIN_MOTOR_T3, LOW);
  digitalWrite (PIN_MOTOR_T4, HIGH);
   
  digitalWrite(PIN_MOTOR_D1, LOW);
  digitalWrite(PIN_MOTOR_D2, HIGH); 
  digitalWrite(PIN_MOTOR_D3, HIGH); 
  digitalWrite(PIN_MOTOR_D4, LOW);
}

void ATRAS()
{ 
  digitalWrite (PIN_MOTOR_T1, LOW);
  digitalWrite (PIN_MOTOR_T2, HIGH);
  digitalWrite (PIN_MOTOR_T3, HIGH);
  digitalWrite (PIN_MOTOR_T4, LOW);

  digitalWrite(PIN_MOTOR_D1, HIGH);
  digitalWrite(PIN_MOTOR_D2, LOW); 
  digitalWrite(PIN_MOTOR_D3, LOW); 
  digitalWrite(PIN_MOTOR_D4, HIGH); 

}

void setup()
{
  pinMode (PIN_MOTOR_T1, OUTPUT);  
  pinMode (PIN_MOTOR_T2, OUTPUT);  
  pinMode (PIN_MOTOR_T3, OUTPUT);  
  pinMode (PIN_MOTOR_T4, OUTPUT);  
  
  pinMode (PIN_MOTOR_D1, OUTPUT);  
  pinMode (PIN_MOTOR_D2, OUTPUT);  
  pinMode (PIN_MOTOR_D3, OUTPUT);  
  pinMode (PIN_MOTOR_D4, OUTPUT);   
}

void loop()
{
 // ADELANTE();
 digitalWrite(PIN_MOTOR_T1, HIGH);
 digitalWrite(PIN_MOTOR_T2, LOW);
 digitalWrite(PIN_MOTOR_T3, LOW);
 digitalWrite(PIN_MOTOR_T4, LOW);
 delay(50);
 digitalWrite(PIN_MOTOR_T1, LOW);
 digitalWrite(PIN_MOTOR_T2, HIGH);
 digitalWrite(PIN_MOTOR_T3, LOW);
 digitalWrite(PIN_MOTOR_T4, LOW);
 delay(50);
 digitalWrite(PIN_MOTOR_T1, LOW);
 digitalWrite(PIN_MOTOR_T2, LOW);
 digitalWrite(PIN_MOTOR_T3, HIGH);
 digitalWrite(PIN_MOTOR_T4, LOW);
 delay(50);
 digitalWrite(PIN_MOTOR_T1, LOW);
 digitalWrite(PIN_MOTOR_T2, LOW);
 digitalWrite(PIN_MOTOR_T3, LOW);
 digitalWrite(PIN_MOTOR_T4, HIGH);
 delay(50);
}
