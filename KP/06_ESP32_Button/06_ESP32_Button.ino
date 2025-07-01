#define BUTTON_PIN 19
#define LED_PIN 4

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
}

bool reading, lastButtonState = HIGH, buttonState = HIGH;
uint32_t lastDebounceTime = 0;

void loop() {
  // Ambil status tombol
  reading = digitalRead(BUTTON_PIN);

  // Jika status button berubah (karena noise atau tekanan), reset timer debounce
  if (reading != lastButtonState) {
    Serial.print("STATUS : ");
    Serial.println(reading);
    lastDebounceTime = millis();
  }

  // Cek apakah sudah melewati waktu debounce
  if ((millis() - lastDebounceTime) > 50) {
    // Jika status button sudah stabil
    if (reading != buttonState) {
      buttonState = reading;

      // Hanya ubah status LED jika button ditekan (LOW)
      if (buttonState == LOW) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    }
  }

  // Simpan status button terakhir
  lastButtonState = reading;
}