#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME280.h>

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BME_ADDRESS 0x77

#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18

#define INTERVAL 1000

// OLED display (SH1106G 128x64)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// BME280 sensor
Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);

  // Inisialisasi I2C custom
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  // OLED
  Serial.println("Inisialisasi OLED...");
  display.begin(SCREEN_ADDRESS, true);
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  // BME280
  Serial.println("Mendeteksi BME280...");
  if (!bme.begin(BME_ADDRESS)) {  // Ganti ke 0x77 jika 0x76 tidak terdeteksi
    Serial.println("Sensor BME280 tidak ditemukan! Cek koneksi");
    while (1);
  }
  Serial.println("BME280 ditemukan");

  // Tampilan awal
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("BME280 Ready...");
  display.display();
  delay(1000);
}

uint32_t previousMillis = 0, currentMillis;
float temperature, humidity, pressure;

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;  // hPa

    // Tampilkan di Serial Monitor
    Serial.print("Suhu: ");
    Serial.print(temperature);
    Serial.print(" C, Kelembaban: ");
    Serial.print(humidity);
    Serial.print(" %, Tekanan: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    // Tampilkan di OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Temp: ");
    display.print(temperature);
    display.println(" C");
    display.print("Hum:  ");
    display.print(humidity);
    display.println(" %");
    display.print("Pres: ");
    display.print(pressure);
    display.println(" hPa");
    display.display();
  }
}