#include <RH_RF95.h>
#include <RHReliableDatagram.h>

constexpr const uint32_t INTERVAL = 5000;

constexpr const uint8_t LORA_CS = 5;
constexpr const uint8_t LORA_IRQ = 4;
constexpr const uint8_t LORA_RST = 26;

constexpr const uint8_t NODE_ID = 2; // Alamat ID node ini
constexpr const uint8_t RECEIVER_ID = 1; // Alamat ID tujuan

// Instansiasi driver RF95 (SX1276)
RH_RF95 rf95(LORA_CS, LORA_IRQ);

// Instansiasi manager pengiriman andal (Reliable Datagram)
RHReliableDatagram manager(rf95, NODE_ID);

// Prototipe fungsi
// void messageTypeCast(const char *message, uint8_t *typeCastedMessage, size_t maxLen);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("\r\nInitializing LoRa with RadioHead..."));

  // Praktik Terbaik untuk ESP32 + SX1276:
  // Lakukan reset manual pada chip sebelum inisialisasi agar terhindar dari error chip tidak terbaca
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  delay(10);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // Inisialisasi Manager (otomatis menginisialisasi driver rf95 juga)
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
  
  manager.setRetries(3); // Ubah jumlah pengiriman ulang (default = 3)
  manager.setTimeout(200); // Waktu tunggu ACK dalam milidetik (default = 200)
}

void loop() {
  static uint32_t lastTransmitTime = 0;
  uint32_t currentTime = millis();

  if (currentTime - lastTransmitTime >= INTERVAL) {
    lastTransmitTime = currentTime;

    // Data yang akan dikirim
    // const char *message = "Labore fugiat laboris tempor laboris anim sint fugiat elit labore reprehenderit. Ea quis occaecat ipsum deserunt incididunt. Consequat consequat duis elit nulla mollit non officia quis enim magna est. Deserunt mollit est voluptate officia aliqua. Cupidatat ea incididunt.";
    const char *message = "{\"Gateway\":[{\"acc_dyn\":0,\"ratio_sta_lta\":0,\"roll\":0,\"pitch\":0,\"gyro_x\":0,\"gyro_y\":0,\"gyro_z\":0}]}";
    
    /* uint8_t typeCastedMessage[256];
    messageTypeCast(message, typeCastedMessage, sizeof(typeCastedMessage)); */

    size_t msgLen = strlen(message) + 1; // Tentukan panjang pesan asli (+1 untuk \0)

    Serial.println(F("----------------"));

    if (msgLen > RH_RF95_MAX_MESSAGE_LEN) {
      Serial.println(F("Buffer overflow, string cut! Truncating to 251 bytes"));
      msgLen = RH_RF95_MAX_MESSAGE_LEN; // Paksa panjang maksimal 251 (termasuk \0)
    }

    /* uint8_t typeCastedMessage[msgLen]; // Siapkan wadah sesuai ukuran pesan yang mau dikirim
    memcpy(typeCastedMessage, message, msgLen); // Proses pengkopian instan di register CPU
    typeCastedMessage[msgLen - 1] = '\0'; // Kalau data terpotong, char terakhir harus \0 */

    Serial.print(F("Transmit message to ID: "));
    Serial.println(RECEIVER_ID);
    Serial.println(F("Message:"));
    Serial.println(/* (char*)typeCastedMessage */message);

    // Kirim data dan tunggu tanda terima (ACK)
    // Fungsi sendtoWait akan me-return true hanya jika ACK berhasil diterima dari penerima
    if (manager.sendtoWait(/* typeCastedMessage */(uint8_t*)message, /* strlen(message) */msgLen, RECEIVER_ID)) {
      Serial.println(F("Transmission success! Validated by receiver!"));
    } else {
      Serial.println(F("Transmission failed! There is no ACK response!"));
    }
  }
}

/* void messageTypeCast(const char *message, uint8_t *typeCastedMessage, size_t maxLen) {
  for (size_t i = 0; i < strlen(message); i++) {
    if (i > maxLen - 1) {
      Serial.println(F("Buffer overflow, string cut!"));
      break; 
    }

    typeCastedMessage[i] = (uint8_t)message[i];

    if (i == strlen(message) - 1) {
      // Null Terminator
      typeCastedMessage[i] = '\0';
    }
  }
} */