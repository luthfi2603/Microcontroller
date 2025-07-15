#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME280.h>

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BME_ADDRESS 0x77

#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18

#define LORA_NSS 10
#define LORA_BUSY 4
#define LORA_RST 5
#define LORA_DIO1 1

#define INTERVAL 5000

// OLED display (SH1106G 128x64)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

SX1262 loRa = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

Adafruit_BME280 bme;

int8_t state;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  Serial.println("Initializing OLED...");
  display.begin(SCREEN_ADDRESS, true);
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  Serial.println("Initializing LoRa...");
  state = loRa.begin();
  if (state == RADIOLIB_ERR_NONE) {
    loRa.setFrequency(923.2);         // untuk Indonesia gunakan 923.0 MHz
    loRa.setBandwidth(125.0);         // BW = 125 kHz
    loRa.setSpreadingFactor(7);       // SF7
    loRa.setCodingRate(5);            // CR 4/5
    loRa.setOutputPower(22);          // 22 dBm, maksimal power T-Beam

    Serial.println("LoRa init success!");
  } else {
    Serial.print("LoRa init failed, code: ");
    Serial.println(state);
    while (true) { delay(10); }
  }

  // BME280
  Serial.println("Detect BME280...");
  if (!bme.begin(BME_ADDRESS)) {  // Ganti ke 0x77 jika 0x76 tidak terdeteksi
    Serial.println("Sensor BME280 not found! Check connection");
    while (true) { delay(10); }
  }
  Serial.println("BME280 detected");

  // Tampilan awal
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("LoRa Transmitter Ready...");
  display.println("BME280 Ready...");
  display.display();
  delay(1000);
}

uint32_t previousMillis = 0, currentMillis;
float temperature, humidity, pressure;
String message;

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;  // hPa

    message = String("{\n  \"temp\": ") + temperature + ",\n  \"hum\": " + humidity + ",\n  \"press\": " + pressure + "\n}";
    // message = "Hello World";

    Serial.println("----------------\nTransmit:");
    Serial.println(message);

    state = loRa.transmit(message);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("Transmission success!");

      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Transmission success!");
      display.print(message);
      display.display();
    } else {
      Serial.print("Transmission Failed, code: ");
      Serial.println(state);

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Transmission Failed, code: ");
      display.println(state);
      display.display();
    }
  }
}