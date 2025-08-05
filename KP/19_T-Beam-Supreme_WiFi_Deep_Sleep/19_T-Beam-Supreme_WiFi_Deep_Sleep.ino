#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_BME280.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BME_ADDRESS 0x77

#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18

#define INTERVAL 60000  // 60 detik

// WiFi
#define WIFI_SSID "USUNETA-IOT-DLCBLT4"
#define WIFI_PASSWORD "IoTDLCBLt4?"

// MQTT Broker
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_CLIENT_ID ""
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_PORT 1883
#define MQTT_TOPIC_TELE_PUB "v1/devices/me/telemetry" // Untuk publish

WiFiClient espClient;
PubSubClient client(espClient);

// OLED display (SH1106G 128x64)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
Adafruit_BME280 bme;

float temperature, humidity, pressure;
String message;

void setup() {
  Serial.begin(115200);
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

  // Connecting to a WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }
  // Serial.println();
  // Serial.println("Connected to the Wi-Fi network");

  // Connecting to a MQTT Broker
  client.setServer(MQTT_BROKER, MQTT_PORT);

  while (!client.connected()) {
    // Serial.print("The client ");
    // Serial.print(MQTT_CLIENT_ID);
    // Serial.println(" connecting to the MQTT Broker");
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      // Serial.println("MQTT Broker connected");
    } else {
      // Serial.print("Failed with state ");
      // Serial.print(client.state());
      // Serial.println(" try again in 5 seconds");
      delay(5000);
    }
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
  message = String("{\"temp\":") + temperature + ",\"hum\":" + humidity + ",\"press\":" + pressure + "}";

  // Serial.println("----------------\nTransmit:");
  // Serial.println(message);

  // Kirim data via WiFi
  if (client.publish(MQTT_TOPIC_TELE_PUB, message.c_str())) {
    // Serial.println("Transmission success!");

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Transmission success!");
    display.print(message);
    display.display();
  } else {
    // Serial.print("Transmission failed, code: ");
    // Serial.println(client.state());

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Transmission failed!");
    display.print("Error code: " + String(client.state()));
    display.display();
  }

  delay(3000);  // Biar sempat ditampilkan
  // Serial.println("Entering deep sleep for 60 seconds...");

  // Set timer untuk bangun setelah 60 detik
  esp_sleep_enable_timer_wakeup(INTERVAL * 1000ULL);  // dalam mikrodetik
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Sleep for 60s...");
  display.display();

  delay(5000);  // Agar ada delay untuk tampilkan OLED dan upload program baru
  display.clearDisplay();
  display.display();
  // display.ssd1306_command(SH110X_DISPLAYOFF);
  esp_deep_sleep_start();
}

void loop() {}