#include <DHT.h>

#define DHT_PIN 13
#define DHT_TYPE DHT22
#define LED_PIN 2
#define INTERVAL 1000

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  dht.begin();
}

uint32_t previousMilis = 0, currentMilis;
float humidity, temperature;

void loop() {
  currentMilis = millis();
  if (currentMilis - previousMilis >= INTERVAL) {
    previousMilis = currentMilis;

    humidity = dht.readHumidity(); // Membaca kelembaban
    temperature = dht.readTemperature(); // Membaca suhu dalam Celcius

    // Cek apakah pembacaan berhasil
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Gagal membaca dari sensor DHT!");
      return;
    }

    if (temperature >= 35) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    Serial.print("Kelembaban: ");
    Serial.print(humidity);
    Serial.print("%\t");
    Serial.print("Suhu: ");
    Serial.print(temperature);
    Serial.println("°C");
  }
}