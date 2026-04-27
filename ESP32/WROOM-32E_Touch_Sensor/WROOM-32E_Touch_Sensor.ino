#define TOUCH_PIN 4

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  uint32_t touchValue = touchRead(TOUCH_PIN);
  
  Serial.print("--------\nTouch value: ");
  Serial.println(touchValue);
  delay(500);
}