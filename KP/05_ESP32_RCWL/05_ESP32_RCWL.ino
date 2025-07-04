#define RCWL_PIN 23
#define LED_PIN 2

void setup() {
  Serial.begin(115200);
  pinMode(RCWL_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
}

int8_t motion = LOW, state = LOW;

void loop() {
  motion = digitalRead(RCWL_PIN);

  if (motion == HIGH) { // Ada gerakan
    if (state == LOW) {
      Serial.println("Gerakan terdeteksi!");
      digitalWrite(LED_PIN, HIGH);
      state = HIGH;
    }
  } else { // Tidak ada gerakan
    if (state == HIGH) {
      Serial.println("Tidak ada gerakan");
      digitalWrite(LED_PIN, LOW);
      state = LOW;
    }
  }
}