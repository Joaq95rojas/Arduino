// SE MEJORA EL ALGORITMO DE MOVIMIENTO DEL SERVO.

#include <Streaming.h>
#include <Servo.h> 

#define trigPin 5
#define echoPin 4
#define servo 3

#define ANGULO_MAXIMO_GRADOS  140
#define ANGULO_MINIMO_GRADOS  70
#define PASO_MINIMO_GRADOS    5
#define TIEMPO_SERVO_MS       10
#define DISTANCIA_MAXIMA_CM   15

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
bool direccionServo = true;

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
    }while(LECTURA_HCSR04() < DISTANCIA_MAXIMA_CM);
    
   
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
  if( pos <= ANGULO_MAXIMO_GRADOS )
  {
    if( direccionServo && pos<=(ANGULO_MAXIMO_GRADOS-PASO_MINIMO_GRADOS) ) pos+=PASO_MINIMO_GRADOS;
    else if( !direccionServo && pos >= ANGULO_MINIMO_GRADOS) pos-=PASO_MINIMO_GRADOS;
    else direccionServo = !direccionServo;
  }
  else direccionServo = !direccionServo;
  Serial << "GIRO: " << direccionServo << endl;
  myservo.write(pos);  Serial << "pos: " << pos << endl;
  delay(15);

  if( LECTURA_HCSR04() < DISTANCIA_MAXIMA_CM ) PARAR();
  else  ADELANTE();
  
  delay(TIEMPO_SERVO_MS);

}


