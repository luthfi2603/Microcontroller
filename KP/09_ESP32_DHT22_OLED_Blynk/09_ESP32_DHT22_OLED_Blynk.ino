#define BLYNK_TEMPLATE_ID "TMPL6exAg2JQx"
#define BLYNK_TEMPLATE_NAME "Sensor"
#define BLYNK_AUTH_TOKEN "mZ7HW-JvWPCc1N746xWchZnknhS0J1AD"

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#define DHT_PIN 13
#define DHT_TYPE DHT22
#define LED_PIN 2
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22
#define VIRTUAL_PIN_TEMP V4
#define VIRTUAL_PIN_HUM V5
#define INTERVAL 1000

#define SCREEN_ADDRESS 0x3C 
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define WIFI_SSID "USUNETA-IOT-DLCBLT4"
#define WIFI_PASSWORD "IoTDLCBLt4?"

DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  dht.begin();

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED SSD1306 tidak terdeteksi. Periksa wiring & alamat I2C!");
    while (true); // berhenti di sini
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
}

uint32_t previousMillis = 0, currentMillis;
float humidity, temperature;

void loop() {
  Blynk.run();
  currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

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

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Temperature: ");
    display.print(temperature);
    display.println("C");
    display.print("Humidity: ");
    display.print(humidity);
    display.println("%");
    display.display();

    Blynk.virtualWrite(VIRTUAL_PIN_TEMP, temperature);
    Blynk.virtualWrite(VIRTUAL_PIN_HUM, humidity);
  }
}