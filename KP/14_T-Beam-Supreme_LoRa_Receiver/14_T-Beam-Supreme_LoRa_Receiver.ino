#include <RadioLib.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18

#define LORA_NSS 10
#define LORA_BUSY 4
#define LORA_RST 5
#define LORA_DIO1 1

// OLED display (SH1106G 128x64)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

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
  state = lora.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init success!");
  } else {
    Serial.print("LoRa init failed, code: ");
    Serial.print(state);
    while (true);
  }

  // Tampilan awal
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("LoRa Receiver Ready...");
  display.display();
  delay(1000);
}

String message;

void loop() {
  state = lora.receive(message);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("----------------\nMessage received: ");
    Serial.println(message);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Reception success!");
    display.print(message);
    display.display();
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.println("Timeout...");

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Timeout...");
    display.display();
  } else {
    Serial.print("Receive error, code: ");
    Serial.println(state);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Receive error, code: ");
    display.print(state);
    display.display();
  }

  delay(1000);
}