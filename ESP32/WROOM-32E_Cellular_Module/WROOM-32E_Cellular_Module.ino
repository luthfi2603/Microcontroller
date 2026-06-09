// Definisikan pin Hardware Serial 2 milik ESP32
constexpr const uint8_t RXD2 = 16;
constexpr const uint8_t TXD2 = 17;

void setup() {
  // 1. Jalur komunikasi dari ESP32 ke Laptop (Serial Monitor)
  Serial.begin(115200);
  
  // 2. Jalur komunikasi dari ESP32 ke Modul Seluler (Hardware UART2)
  // Format: Serial2.begin(BaudRate, SERIAL_8N1, RX_Pin, TX_Pin);
  // Kita coba 115200 dulu karena banyak modul baru (termasuk 4G) memakai baud rate ini.
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  delay(2000);
  Serial.println("===========================================");
  Serial.println("  Sistem Interogasi Modul Universal Siap!  ");
  Serial.println("  Ketik 'AT' dan tekan Enter.              ");
  Serial.println("===========================================");
}

void loop() {
  // Jika Modul mengirim balasan, teruskan tampilannya ke Layar Laptop
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  
  // Jika Kamu mengetik di Layar Laptop, teruskan perintahnya ke Modul
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
}