#include <RadioLib.h>

#define LORA_NSS      10
#define LORA_BUSY     8
#define LORA_RST      9
#define LORA_DIO1     3

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

void loop() {
  String message = "Hello from T-Beam Supreme!";
  Serial.print("Mengirim: ");
  Serial.println(message);

  state = lora.transmit(message);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Pengiriman sukses!");
  } else {
    Serial.print("Gagal kirim, kode: ");
    Serial.println(state);
  }

  delay(5000);
}