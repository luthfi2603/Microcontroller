#include <RadioLib.h>

#define LORA_CS 5
#define LORA_IRQ 2
#define LORA_RST 14

#define INTERVAL 1000
/**
 * MOSI 23
 * MISO 19
 * VSCLK 18
 * CS 5
 * GPIO 2
 * HSPICLK 12
 */
// SX1276 loRa = new Module(LORA_CS, LORA_IRQ, LORA_RST);

int8_t state;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing LoRa...");
  /* state = loRa.begin();
  if (state == RADIOLIB_ERR_NONE) {
    loRa.setFrequency(923.2);         // untuk Indonesia gunakan 923.0 MHz
    loRa.setBandwidth(125.0);         // BW = 125 kHz
    loRa.setSpreadingFactor(7);       // SF7
    loRa.setCodingRate(5);            // CR 4/5
    loRa.setOutputPower(22);          // 22 dBm, maksimal power T-Beam */

    Serial.println("LoRa init success!");
  /* } else {
    Serial.print("LoRa init failed, code: ");
    Serial.println(state);
    while (true) { delay(10); }
  } */
}

uint32_t previousMillis = 0, currentMillis;
String message;

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    message = "Hello World!";

    Serial.println("----------------\nTransmit:");
    Serial.println(message);

    /* state = loRa.transmit(message);
    if (state == RADIOLIB_ERR_NONE) { */
      Serial.println("Transmission success!");
    /* } else {
      Serial.print("Transmission failed, code: ");
      Serial.println(state);
    } */
  }
}