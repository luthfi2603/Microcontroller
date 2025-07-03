#define LED_PIN 2
#define INTERVAL 1000

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

bool state = true;
uint32_t previousMilis = 0, currentMilis;

void loop() {
  currentMilis = millis();
  if (currentMilis - previousMilis >= INTERVAL) {
    previousMilis = currentMilis;

    if (state) {
      digitalWrite(LED_PIN, HIGH);
      state = false;
    } else {
      digitalWrite(LED_PIN, LOW);
      state = true;
    }
  }
}