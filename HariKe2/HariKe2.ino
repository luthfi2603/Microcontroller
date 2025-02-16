#include <ESP8266WiFi.h>
#include <WifiClient.h>
#include <DHT.h>

#define DHT_PIN 4
#define MQ_PIN_AO A0
#define MQ_PIN_DO D5
#define BUZZER_PIN 16
#define LED_PIN 5
#define DHTTYPE DHT11
#define WIFI_SSID "KOST TENTREM LOBBY"
#define WIFI_PASSWORD "meteorakalambaka18"

DHT dht(DHT_PIN, DHTTYPE);

void setup() {
    Serial.begin(9600);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(DHT_PIN, INPUT);
    pinMode(MQ_PIN_AO, INPUT);
    pinMode(MQ_PIN_DO, INPUT);

    digitalWrite(BUZZER_PIN, LOW);

    dht.begin();
}

unsigned long previousMilis = 0, currentMilis;
const long interval = 1000;

void loop() {
    currentMilis = millis();
    if (currentMilis - previousMilis >= interval) {
        previousMilis = currentMilis;

        float humidity = dht.readHumidity(); // Membaca kelembaban
        float temperature = dht.readTemperature(); // Membaca suhu dalam Celcius

        // Cek apakah pembacaan berhasil
        if (isnan(humidity) || isnan(temperature)) {
            Serial.println("Gagal membaca dari sensor DHT!");
            return;
        }

        Serial.print("Kelembaban: ");
        Serial.print(humidity);
        Serial.print("%\t");
        Serial.print("Suhu: ");
        Serial.print(temperature);
        Serial.println("°C");

        if (temperature >= 50) {
            digitalWrite(BUZZER_PIN, HIGH);
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(LED_PIN, LOW);
        }

        int gasValue = analogRead(MQ_PIN_AO);  // Baca nilai analog
        Serial.print("Gas Value: ");
        Serial.println(gasValue);  // Cetak ke Serial Monitor

        int gasStatus = digitalRead(MQ_PIN_DO);  // Baca status gas (0 atau 1)
        if (gasStatus == LOW) {
            Serial.println("⚠ Gas terdeteksi!");  
        } else {
            Serial.println("✅ Gas normal");
        }

        Serial.println();
    }
}