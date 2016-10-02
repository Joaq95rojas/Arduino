/*
 El programa activa el motor en un sentido por 4 segundos, 
 para el motor por 500 ms, activa el motor en sentido inverso por 4 segundos 
 y se detiene por 5 segundos. Luego repite la acción indefinidamente.
*/
#include <Servo.h> 

#define trigPin 5
#define echoPin 4
#define servo 3
long distancia_minima= 16;

int TRA1 = 10; 
int TRA2 = 11;
int TRA3 = 12; 
int TRA4 = 13;

int DEL1 = 6; 
int DEL2 = 7;
int DEL3 = 8; 
int DEL4 = 9;

Servo myservo;
int pos = 90;

void ADELANTE()
{ 
  digitalWrite (TRA4, HIGH);
  digitalWrite (TRA3, LOW); 
  digitalWrite (TRA1, HIGH);
  digitalWrite (TRA2, LOW);
  
  digitalWrite (DEL4, LOW);
  digitalWrite (DEL3, HIGH); 
  digitalWrite (DEL1, LOW);
  digitalWrite (DEL2, HIGH); 
}

void ATRAS()
{ digitalWrite (TRA1, LOW);
  digitalWrite (TRA4, LOW);
  digitalWrite (TRA3, HIGH);
  digitalWrite (TRA2, HIGH);
  
  digitalWrite (DEL1, HIGH);
  digitalWrite (DEL4, HIGH);
  digitalWrite (DEL3, LOW);
  digitalWrite (DEL2, LOW);
}

void PARAR()
{ digitalWrite (TRA1, LOW);
  digitalWrite (TRA4, LOW);
  digitalWrite (TRA3, LOW);
  digitalWrite (TRA2, LOW);
  
  digitalWrite (DEL1, LOW);
  digitalWrite (DEL4, LOW);
  digitalWrite (DEL3, LOW);
  digitalWrite (DEL2, LOW);
  
  do{
    }while(LECTURA_HCSR04() < distancia_minima);
    
   
}

long LECTURA_HCSR04()
{
    long duration, distance;
   digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}
void setup()
{
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode (TRA4, OUTPUT);    // Input4 conectada al pin 4 
  pinMode (TRA3, OUTPUT);    // Input3 conectada al pin 5
  pinMode (TRA2, OUTPUT);    // Input4 conectada al pin 4 
  pinMode (TRA1, OUTPUT);    // Input3 conectada al pin 5
  pinMode (DEL4, OUTPUT);    // Input4 conectada al pin 4 
  pinMode (DEL3, OUTPUT);    // Input3 conectada al pin 5
  pinMode (DEL2, OUTPUT);    // Input4 conectada al pin 4 
  pinMode (DEL1, OUTPUT);    // Input3 conectada al pin 5
  myservo.attach(servo);  // attaches the servo on pin 9 to the servo object 
}
void loop()
{
   for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);       // waits 15ms for the servo to reach the position 
    
      if (LECTURA_HCSR04() < distancia_minima) {  // This is where the LED On/Off happens
   //Serial.println(distance);
  // Serial.println("PARAR | OBSTACULO DETECTADO");
   PARAR();
   
   
}
  else {
      // Serial.println(distance);
       ADELANTE();
  }
  
  
} 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
   // LECTURA_HCSR04();
      if (LECTURA_HCSR04() < distancia_minima) {  // This is where the LED On/Off happens
   //Serial.println(distance);
   Serial.println("PARAR | OBSTACULO DETECTADO");
   PARAR();
      }
  else {
      // Serial.println(distance);
       ADELANTE();
      }
    
 
  } 

 
}


