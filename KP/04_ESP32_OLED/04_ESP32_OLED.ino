#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Ubah jika OLED-mu pakai alamat I2C selain 0x3C
#define SCREEN_ADDRESS 0x3C 
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// Default ESP32 I2C pins (SDA = GPIO21, SCL = GPIO22)
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

void setup() {
  // Inisialisasi I2C
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  // Inisialisasi display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.begin(9600);
    Serial.println("OLED SSD1306 tidak terdeteksi. Periksa wiring & alamat I2C!");
    while (true); // berhenti di sini
  }

  display.clearDisplay();      // bersihkan buffer
  display.setTextSize(1);      // ukuran font (1–8)
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);     // (x, y) dalam pixel
  display.println("Hello World!");
  display.display();           // kirim buffer ke layar
}

void loop() {
  // nothing to do here
}