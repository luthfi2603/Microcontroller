#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DHT_PIN 13
#define DHT_TYPE DHT22
#define LED_PIN 2
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22
#define INTERVAL 1000

#define SCREEN_ADDRESS 0x3C 
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

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
}

uint32_t previousMillis = 0, currentMillis;
float humidity, temperature;

void loop() {
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
  }
}