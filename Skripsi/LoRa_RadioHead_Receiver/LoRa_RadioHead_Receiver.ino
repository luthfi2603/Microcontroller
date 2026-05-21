#include <RH_RF95.h>
#include <RHReliableDatagram.h>

constexpr const uint8_t LORA_CS = 5;
constexpr const uint8_t LORA_IRQ = 4;
constexpr const uint8_t LORA_RST = 26;

constexpr const uint8_t NODE_ID = 0; // Alamat ID node ini

// Instansiasi driver RF95 (SX1276)
RH_RF95 rf95(LORA_CS, LORA_IRQ);

// Instansiasi manager pengiriman andal (Reliable Datagram)
RHReliableDatagram manager(rf95, NODE_ID);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("\r\nInitializing LoRa with RadioHead..."));

  // Reset manual pada chip sebelum inisialisasi
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  delay(10);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // Inisialisasi Manager
  if (!manager.init()) {
    Serial.println(F("Lora init failed! Check SPI cable (MISO, MOSI, SCK, CS)!"));
    while (1) { delay(1000); } // Berhenti di sini jika gagal
  }
  
  Serial.println(F("LoRa init success!"));

  // Konfigurasi Frekuensi (Sesuaikan dengan aturan regulasi Indonesia)
  if (!rf95.setFrequency(923.2)) {
    Serial.println(F("Set frequency failed!"));
    while (1) { delay(1000); }
  }

  // Konfigurasi Power (Maksimal 23 dBm untuk SX1276)
  rf95.setTxPower(5, false);
  
  Serial.println(F("LoRa starting to listen..."));
}

void loop() {
  // Cek apakah ada paket data yang terbang di udara
  if (manager.available()) {
    // Siapkan wadah (buffer) untuk menampung pesan yang masuk
    // RH_RF95_MAX_MESSAGE_LEN adalah batas bawaan dari library (biasanya 251 byte)
    uint8_t dataBuffer[RH_RF95_MAX_MESSAGE_LEN + 1];
    uint8_t dataLength = RH_RF95_MAX_MESSAGE_LEN;
    uint8_t fromAddress; // Variabel untuk menyimpan info "Pesan ini dari siapa?"

    // Tangkap data dan otomatis kirimkan tanda terima (ACK) ke pengirim
    if (manager.recvfromAck(dataBuffer, &dataLength, &fromAddress)) {
      dataBuffer[dataLength] = '\0';
      const char *message = (char*)dataBuffer; // Byte jadi c-string

      // Ambil nilai RSSI (Kekuatan Sinyal)
      int16_t rssi = rf95.lastRssi();

      Serial.print(F("----------------\r\nMessage received from ID: "));
      Serial.println(fromAddress);
      Serial.println(F("Message:"));
      Serial.println(message);
      Serial.print(F("RSSI: "));
      Serial.print(rssi);
      Serial.println(F(" dBm"));
      Serial.println(F("Automatically send ACK reply to sending node..."));
      Serial.println(F("LoRa starting to listen again..."));
    }
  }
}