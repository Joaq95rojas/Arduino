#include <Streaming.h>
#include <Servo.h>
#include <Wire.h>
#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"

#define REFRESH_INTERVAL    200000    // Timer Interval of PWM Servo function (200 ms)

#define trigPin 5
#define echoPin 4
#define servo 3

#define ANGULO_MAXIMO_GRADOS  140
#define ANGULO_MINIMO_GRADOS  70
#define PASO_MINIMO_GRADOS    5
#define TIEMPO_SERVO_MS       50/portTICK_PERIOD_MS
#define DISTANCIA_MAXIMA_CM   15

Servo myservo;
int pos = 90;
bool direccionServo = true;

long LECTURA_HCSR04()
{
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}
void setup()
{
  Serial.begin (9600); Serial << "** Serial **" << endl;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo.attach(servo);  // attaches the servo on pin 9 to the servo object
  xTaskCreate(vUltrasonicoTask, "HCSR04", 768, NULL, 2, NULL);
  vTaskStartScheduler();
  for(;;);
}

void loop() {}

void vUltrasonicoTask( void *pvParameters )
{
  bool prueba;
  for (;;)
  {
    //taskENTER_CRITICAL();
    if ( pos <= ANGULO_MAXIMO_GRADOS )
    {
      if ( direccionServo && (pos <= (ANGULO_MAXIMO_GRADOS - PASO_MINIMO_GRADOS)) ) pos += PASO_MINIMO_GRADOS;
      else if ( !direccionServo && pos >= ANGULO_MINIMO_GRADOS) pos -= PASO_MINIMO_GRADOS;
      else direccionServo = !direccionServo;
    }
//    else direccionServo = !direccionServo;
//    taskENTER_CRITICAL();
    myservo.write(pos);
//    taskEXIT_CRITICAL();
    Serial << "pos: " << pos << endl;
    // delay(15);
    prueba = LECTURA_HCSR04() < DISTANCIA_MAXIMA_CM;
    Serial << "prueba: " << prueba << endl;
    /*
    if ( LECTURA_HCSR04() < DISTANCIA_MAXIMA_CM ) {}// PARAR ROBOT
    else {} // MOVER ROBOT*/
    //taskEXIT_CRITICAL();
    vTaskDelay(TIEMPO_SERVO_MS);
  }
}  //  Fin de vUltrasonicoTask()
