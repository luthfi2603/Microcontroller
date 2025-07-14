#include <RadioLib.h>
#include <Wire.h>
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

volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

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
    lora.setFrequency(923.0);         // untuk Indonesia gunakan 923.0 MHz
    lora.setBandwidth(125.0);         // BW = 125 kHz
    lora.setSpreadingFactor(7);       // SF7
    lora.setCodingRate(5);            // CR 4/5
    lora.setOutputPower(22);          // 22 dBm, maksimal power T-Beam

    Serial.println("LoRa init success!");

    /* while (lora.startReceive() != RADIOLIB_ERR_NONE) {
      delay(500);
      Serial.print(".");
      } */
     
    lora.setPacketReceivedAction(setFlag); // Set callback, kalok ada yang diterima
    
    Serial.println(F("LoRa Starting to listen..."));
    state = lora.startReceive(); // Buat jadi mode reciever
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("Success!"));
    } else {
      Serial.print(F("Failed, code: "));
      Serial.print(state);
      while (true) { delay(10); }
    }
  } else {
    Serial.print("LoRa init failed, code: ");
    Serial.print(state);
    while (true) { delay(10); }
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
  // if (lora.available()) {
  if (receivedFlag) {
    receivedFlag = false;
    state = lora.readData(message);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("----------------\nMessage received: ");
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
  }
}