#include <Servo.h>

#define PIN_IR    0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 450
#define _DUTY_NEU 1550
#define _DUTY_MAX 2450

#define _DIST_MIN 100.0
#define _DIST_MAX 250.0

#define EMA_ALPHA 0.25
#define LOOP_INTERVAL 40

Servo myservo;
unsigned long last_loop_time = 0;

float dist_filtered;
float dist_ema = _DIST_MIN;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  Serial.begin(1000000);
}

void loop()
{
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;

  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  a_value = analogRead(PIN_IR);
  dist_raw = (6762.0 / (a_value - 9.0) - 4.0) * 10.0 - 60.0;

  if (dist_raw > _DIST_MAX) {
    dist_filtered = _DIST_MAX;
    digitalWrite(PIN_LED, HIGH);
  } 
  else if (dist_raw < _DIST_MIN) {
    dist_filtered = _DIST_MIN;
    digitalWrite(PIN_LED, HIGH);
  } 
  else {
    dist_filtered = dist_raw;
    digitalWrite(PIN_LED, LOW);
  }

  dist_ema = dist_filtered * EMA_ALPHA + dist_ema * (1 - EMA_ALPHA);
  duty = _DUTY_MIN + ((dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN)) * (_DUTY_MAX - _DUTY_MIN);
  myservo.writeMicroseconds(duty);

  Serial.print("IR:");           Serial.print(a_value);
  Serial.print(", dist_raw:");   Serial.print(dist_raw);
  Serial.print(", ema:");        Serial.print(dist_ema);
  Serial.print(", duty:");       Serial.print(duty);
  Serial.println("");
}