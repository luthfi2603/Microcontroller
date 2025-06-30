#include <DHT.h>

#define DHT_PIN 15
#define DHTTYPE DHT22
#define LED_PIN 2

DHT dht(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  // pinMode(DHT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  dht.begin();
}

uint32_t previousMilis = 0, currentMilis;
uint16_t interval = 1000;
float humidity, temperature;

void loop() {
  currentMilis = millis();
  if (currentMilis - previousMilis >= interval) {
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