#include <ESP32Servo.h>

#define SERVO_PIN 14

Servo servo;

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  servo.write(0);
}

void loop() {
  for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    servo.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }

  for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
    servo.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }

  /* // Putar ke 0°
  servo.write(0);
  Serial.println(0);
  delay(1000);

  // Putar ke 90°
  servo.write(90);
  Serial.println(90);
  delay(1000);

  // Putar ke 180°
  servo.write(180);
  Serial.println(180);
  delay(1000);

  // Putar ke 90°
  servo.write(90);
  Serial.println(90);
  delay(1000); */
}