#define BUTTON_PIN 19
#define LED_PIN 4
#define RELAY_PIN 18

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Agar relay mati pada awalnya
}

bool reading, lastButtonState = HIGH, buttonState = HIGH, relayState = false, lastRelayState = false;
uint32_t lastDebounceTime = 0;

void loop() {
  // Ambil status tombol
  reading = digitalRead(BUTTON_PIN);

  // Jika status button berubah (karena noise atau tekanan), reset timer debounce
  if (reading != lastButtonState) {
    Serial.print("BUTTON STATUS: ");
    Serial.println(reading);
    lastDebounceTime = millis();
  }

  // Cek apakah sudah melewati waktu debounce
  if ((millis() - lastDebounceTime) > 50) {
    // Jika status button sudah stabil
    if (reading != buttonState) {
      buttonState = reading;

      // Hanya ubah status relayState jika tombol ditekan
      if (buttonState == LOW) {
        relayState = !relayState;
      }
    }
  }

  // Hanya jalankan digitalWrite jika relayState berubah
  if (relayState != lastRelayState) {
    Serial.print("RELAY STATE CHANGED TO: ");
    Serial.println(relayState);

    digitalWrite(LED_PIN, relayState ? HIGH : LOW);
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);

    lastRelayState = relayState; // Update state terakhir
    Serial.print("LAST RELAY STATE: ");
    Serial.println(lastRelayState);
  }

  // Simpan status button terakhir
  lastButtonState = reading;
}