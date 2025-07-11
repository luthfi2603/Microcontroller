#include <RadioLib.h>

#define LORA_NSS 10
#define LORA_BUSY 4
#define LORA_RST 5
#define LORA_DIO1 1

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

int state;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing LoRa...");
  state = lora.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init sukses!");
  } else {
    Serial.print("LoRa init gagal, kode: ");
    Serial.println(state);
    while (true);
  }
}

String message;

void loop() {
  state = lora.receive(message);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("Pesan diterima: ");
    Serial.println(message);
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.println("Timeout...");
  } else {
    Serial.print("Receive error, code: ");
    Serial.println(state);
  }

  delay(1000);
}