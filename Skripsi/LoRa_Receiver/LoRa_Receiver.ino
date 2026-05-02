#include <RadioLib.h>

constexpr const uint8_t LORA_CS = 5;
constexpr const uint8_t LORA_IRQ = 2;
constexpr const uint8_t LORA_RST = 14;

Module loRaModule(LORA_CS, LORA_IRQ, LORA_RST);
SX1276 loRa(&loRaModule);

// volatile untuk CPU cek langsung ke SRAM
volatile bool receivedFlag = false;

// Untuk ISR
#if defined(ESP8266) || defined(ESP32)
  IRAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

int8_t state;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\r\nInitializing LoRa..."));
  state = loRa.begin();
  if (state == RADIOLIB_ERR_NONE) {
    loRa.setFrequency(923.2);         // untuk Indonesia gunakan 923.0 MHz
    loRa.setBandwidth(125.0);         // BW = 125 kHz
    loRa.setSpreadingFactor(7);       // SF7
    loRa.setCodingRate(5);            // CR 4/5
    loRa.setOutputPower(22);          // 22 dBm, maksimal power

    Serial.println(F("LoRa init success!"));

    loRa.setPacketReceivedAction(setFlag); // Set callback, kalau ada yang diterima
    
    Serial.println(F("LoRa starting to listen..."));
    state = loRa.startReceive(); // Buat jadi mode receiver
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("Success!\nWaiting for packets..."));
    } else {
      Serial.print(F("Failed, code: "));
      Serial.print(state);
      while (true) { delay(10); }
    }
  } else {
    Serial.print(F("LoRa init failed, code: "));
    Serial.print(state);
    while (true) { delay(10); }
  }
}

String message;

void loop() {
  if (receivedFlag) {
    receivedFlag = false;
    state = loRa.readData(message);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("----------------\nMessage received: "));
      Serial.println(message);
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      Serial.println(F("Timeout..."));
    } else {
      Serial.print(F("Receive error, code: "));
      Serial.println(state);
    }
  }
}