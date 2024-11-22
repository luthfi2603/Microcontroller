#define BUZZER_PIN 23

void setup() {
  Serial.begin(115200);
}

void loop() {
  tone(BUZZER_PIN, 400);

  delay(100);
}