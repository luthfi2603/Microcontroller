constexpr uint8_t LED_PIN = 2;
constexpr uint16_t ON_INTERVAL = 2000;
constexpr uint16_t OFF_INTERVAL = 500;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

bool state = true;
uint32_t lastLEDTime = 0;

void loop() {
  uint32_t currentTime = millis();
  uint16_t currentInterval = state ? ON_INTERVAL : OFF_INTERVAL;

  if (currentTime - lastLEDTime >= currentInterval) {
    lastLEDTime += currentInterval;
    state = !state;
    digitalWrite(LED_PIN, state ? HIGH : LOW);
  }
}