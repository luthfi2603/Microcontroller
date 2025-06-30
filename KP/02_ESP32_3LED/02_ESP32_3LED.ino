#define LED_PIN_1 2
#define LED_PIN_2 4
#define LED_PIN_3 16

void setup() {
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN_1, HIGH);
  delay(300);
  digitalWrite(LED_PIN_1, LOW);
  delay(300);
  digitalWrite(LED_PIN_2, HIGH);
  delay(300);
  digitalWrite(LED_PIN_2, LOW);
  delay(300);
  digitalWrite(LED_PIN_3, HIGH);
  delay(300);
  digitalWrite(LED_PIN_3, LOW);
  delay(300);
}