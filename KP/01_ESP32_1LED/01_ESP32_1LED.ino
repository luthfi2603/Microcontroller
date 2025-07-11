#define LED_PIN 2
#define INTERVAL 1000

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

bool state = true;
uint32_t previousMillis = 0, currentMillis;

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    if (state) {
      digitalWrite(LED_PIN, HIGH);
      state = false;
    } else {
      digitalWrite(LED_PIN, LOW);
      state = true;
    }
  }
}