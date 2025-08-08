#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME280.h>
#include <esp_sleep.h>

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

#define INTERVAL 600000  // 10 menit

// OLED display (SH1106G 128x64)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
SX1262 loRa = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
Adafruit_BME280 bme;

int8_t state;
float temperature, humidity, pressure;
String message;

void setup() {
  // Serial.begin(115200);
  delay(1000);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  // OLED
  // Serial.println("Initializing OLED...");
  if (display.begin(SCREEN_ADDRESS, true)) {
    // Serial.println("OLED init success!");
  } else {
    // Serial.println("OLED init failed!");
  }
  // display.ssd1306_command(SH110X_DISPLAYON);
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();

  // LoRa
  // Serial.println("Initializing LoRa...");
  state = loRa.begin();
  if (state == RADIOLIB_ERR_NONE) {
    loRa.setFrequency(923.2);         // untuk Indonesia gunakan 923.0 MHz
    loRa.setBandwidth(125.0);         // BW = 125 kHz
    loRa.setSpreadingFactor(7);       // SF7
    loRa.setCodingRate(5);            // CR 4/5
    loRa.setOutputPower(22);          // 22 dBm, maksimal power T-Beam
    // Serial.println("LoRa init success!");
  } else {
    // Serial.print("LoRa init failed, code: ");
    // Serial.println(state);
    while (true) { delay(10); }
  }

  // BME280
  // Serial.println("Detecting BME280...");
  if (!bme.begin(BME_ADDRESS)) {
    // Serial.println("Sensor BME280 not found! Check connection");
    while (true) { delay(10); }
  }
  // Serial.println("BME280 detected");

  // Ambil data sensor
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

  // Buat pesan
  message = String("{\"id\":1,") + "\"temp\":" + temperature + ",\"hum\":" + humidity + ",\"press\":" + pressure + "}";

  // Serial.println("----------------\nTransmit:");
  // Serial.println(message);

  // Kirim data via LoRa
  state = loRa.transmit(message);
  if (state == RADIOLIB_ERR_NONE) {
    // Serial.println("Transmission success!");

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Transmission success!");
    display.print(message);
    display.display();
  } else {
    // Serial.print("Transmission failed, code: ");
    // Serial.println(state);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Transmission failed!");
    display.print("Error code: " + String(state));
    display.display();
  }

  delay(3000);  // Biar sempat ditampilkan
  // Serial.println("Entering deep sleep for 10 minutes...");

  // Set timer
  esp_sleep_enable_timer_wakeup(INTERVAL * 1000ULL);  // dalam mikrodetik
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Sleep for 10 min...");
  display.display();

  delay(5000);  // Agar ada delay untuk tampilkan OLED dan upload program baru
  display.clearDisplay();
  display.display();
  // display.ssd1306_command(SH110X_DISPLAYOFF);
  esp_deep_sleep_start();
}

void loop() {}