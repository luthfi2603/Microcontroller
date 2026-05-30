#include <ArduinoJson.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// Deklarasi variabel global
constexpr const uint8_t MPU_ADDRESS = 0x68; // MPU-6050 I2C address
constexpr const uint8_t MPU_SAMPLING_INTERVAL = 10; // 10 ms

constexpr const uint16_t LORA_TX_INTERVAL = 5000; // 5 s

constexpr const uint8_t LORA_CS = 5;
constexpr const uint8_t LORA_IRQ = 4;
constexpr const uint8_t LORA_RST = 26;
constexpr const uint16_t LORA_CHECK_INTERVAL = 1000; // 1 s
constexpr const char *NODE_NAME = "Node 2";
constexpr const uint8_t NODE_ID = 2; // Alamat ID node ini
constexpr const uint8_t RECEIVER_ID = 1; // Alamat ID tujuan

constexpr const float RAD_TO_DEG_F = 57.29577951f; // 180 / PI(3.14)
constexpr const float ACCEL_SCALE = 1.0f / 16384.0f; // / 16384
constexpr const float GYRO_SCALE  = 1.0f / 131.0f; // / 131
constexpr const uint8_t STA_WINDOW = 100; // 1 detik (100 sampel)
constexpr const uint16_t LTA_WINDOW = 2000; // 20 detik (2000 sampel)

float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
bool mpuConnectionState, lastMpuConnectionState = true;
uint32_t currentGyroTime;

RH_RF95 rf95(LORA_CS, LORA_IRQ);
RHReliableDatagram manager(rf95, NODE_ID);

// Prototipe fungsi
void calculateIMUError();
bool evaluateSafetyThresholds(const char *nodeName, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ);
void buildJsonPayload(char *outputBuffer, size_t maxLen, const char *nodeName, float accDyn, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ);
void loRaTransmit(const char *jsonPayload);

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Inisialisasi LoRa SX1276 (sama dengan RF95)
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
  
  manager.setRetries(3); // Ubah jumlah pengiriman ulang (default = 3)
  manager.setTimeout(200); // Waktu tunggu ACK dalam milidetik (default = 200)

  Serial.println(F("LoRa starting to listen..."));

  // Inisialisasi sensor MPU-6050
  Serial.println(F("----------------\r\nInitializing MPU-6050..."));
  Wire.begin();                                   // Initialize comunication
  Wire.beginTransmission(MPU_ADDRESS);            // Start communication with MPU6050 // MPU_ADDRESS=0x68
  Wire.write(0x6B);                               // Talk to the register 6B
  Wire.write(0x00);                               // Make reset - place a 0 into the 6B register
  uint8_t statusI2C = Wire.endTransmission(true); // End the transmission
  if (statusI2C != 0) {
    Serial.print(F("MPU-6050 init failed because of I2C problem!, Code: "));
    Serial.println(statusI2C);
    while (1) { delay(1000); } // Hentikan program tanpa membuat Watchdog Crash
  }
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x75); // Tanya KTP (Register WHO_AM_I)
  Wire.endTransmission(false);
  
  if (Wire.requestFrom(MPU_ADDRESS, 1, true) == 1) {
    uint8_t whoAmI = Wire.read();
    
    if (whoAmI == 0x68 || whoAmI == 0x70) {
      Serial.println(F("MPU-6050 init success!"));
    } else {
      Serial.print(F("MPU-6050 is not connected (0x68 || 0x70)!, Register: 0x"));
      Serial.println(whoAmI, HEX);
      while (1) { delay(1000); }
    }
  } else {
    Serial.println(F("Failed to read register WHO_AM_I!"));
    while (1) { delay(1000); }
  }

  // Hitung error sensor MPU-6050
  calculateIMUError();
  delay(20);

  currentGyroTime = millis();
}

void loop() {
  uint32_t currentTime = millis();

  static uint32_t lastLoRaCheckTime = 0;
  
  // Cek konektivitas LoRa
  if (currentTime - lastLoRaCheckTime >= LORA_CHECK_INTERVAL) {
    // Jalankan perintah read register bawaan RadioHead (Register 0x42 adalah RegAfcOf)
    uint8_t version = rf95.spiRead(0x42);
    
    // Jika mengembalikan 0x00 atau 0xFF, kemungkinan besar koneksi SPI ke chip LoRa putus
    if (version != 0x12) {
      Serial.println(F("----------------\r\nLoRa disconnected! Attempting to reinit..."));
      
      digitalWrite(LORA_RST, HIGH);
      delay(10);
      digitalWrite(LORA_RST, LOW);
      delay(10);
      digitalWrite(LORA_RST, HIGH);
      delay(10);

      if (manager.init() && rf95.setFrequency(923.2)) {
        rf95.setTxPower(5, false);
        Serial.println(F("LoRa reinit success!"));
      } else {
        Serial.println(F("LoRa reinit failed!"));
      }
    }
    
    lastLoRaCheckTime = currentTime;
  }

  static char pendingLoRaPayload[252] = ""; // Wadah sementara untuk string JSON LoRa
  static bool hasPendingLoRaData = false; // Bendera penanda ada data LoRa yang antre

  // Receive data LoRa
  if (manager.available()) {
    // Siapkan wadah (buffer) untuk menampung pesan yang masuk
    uint8_t dataBuffer[RH_RF95_MAX_MESSAGE_LEN + 1]; // RH_RF95_MAX_MESSAGE_LEN adalah batas bawaan dari library (biasanya 251 byte)
    uint8_t dataLength = RH_RF95_MAX_MESSAGE_LEN;
    uint8_t fromAddress;

    // Tangkap data dan otomatis kirimkan tanda terima (ACK) ke pengirim
    if (manager.recvfromAck(dataBuffer, &dataLength, &fromAddress)) {
      dataBuffer[dataLength] = '\0';
      const char *loRaPayload = (char*)dataBuffer; // Byte jadi c-string

      // Ambil nilai RSSI (Kekuatan Sinyal)
      int16_t rssi = rf95.lastRssi();

      Serial.print(F("----------------\r\nMessage received from ID: "));
      Serial.println(fromAddress);
      Serial.println(F("Message:"));
      Serial.println(loRaPayload);
      Serial.print(F("RSSI: "));
      Serial.print(rssi);
      Serial.println(F(" dBm"));
      Serial.println(F("Automatically send ACK reply to sending node..."));

      // Parsing JSON string
      JsonDocument json;
      DeserializationError error = deserializeJson(json, loRaPayload);

      if (error) {
        Serial.print(F("Failed to parsing JSON LoRa!, Error: "));
        Serial.println(error.f_str());
      } else {
        strncpy(pendingLoRaPayload, loRaPayload, sizeof(pendingLoRaPayload));
        hasPendingLoRaData = true; // Angkat bendera
      }
      
      Serial.println(F("LoRa starting to listen again..."));
    }
  }

  static float accDyn;
  static float gyroX, gyroY, gyroZ;
  static float roll = 0.0f, pitch = 0.0f/* , yaw = 0.0f */;
  static float staLtaRatio = 0.0f;
  static uint32_t lastSampleTime = 0, lastTxTime = 0;

  // Pembacaan sensor MPU-6050
  if (currentTime - lastSampleTime >= MPU_SAMPLING_INTERVAL) {
    float accAngleX = 0.0f, accAngleY = 0.0f;

    // Read acceleromter data
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDRESS, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
      // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
      float accX = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // X-axis value
      float accY = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // Y-axis value
      float accZ = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // Z-axis value
      // Calculating Roll and Pitch from the accelerometer data
      accAngleX = (atan2f(accY, sqrtf((accX * accX) + (accZ * accZ))) * RAD_TO_DEG_F) - accErrorX; // accErrorX, See the calculateIMUError() custom function for more details
      accAngleY = (atan2f(-accX, sqrtf((accY * accY) + (accZ * accZ))) * RAD_TO_DEG_F) - accErrorY; // accErrorY

      float accRes = sqrtf((accX * accX) + (accY * accY) + (accZ * accZ)) * 9.81f;
      accDyn = fabsf(accRes - 9.81f);

      // Variabel untuk STA/LTA
      static float staBuffer[STA_WINDOW] = {0};
      static float ltaBuffer[LTA_WINDOW] = {0};
      static uint8_t staIndex = 0;
      static uint16_t ltaIndex = 0;
      static float staSum = 0.0f, ltaSum = 0.0f;
      static uint16_t sampleCount = 0;

      // STA/LTA
      // STA (Short-Term Average)
      staSum -= staBuffer[staIndex];          // Buang riwayat paling tua
      staBuffer[staIndex] = accDyn;           // Masukkan getaran terbaru
      staSum += staBuffer[staIndex];          // Tambahkan ke total sum
      staIndex = (staIndex + 1) % STA_WINDOW; // Putar laci index (0-99)
      float sta = staSum / STA_WINDOW;        // Rata-rata

      // LTA (Long-Term Average)
      ltaSum -= ltaBuffer[ltaIndex];
      ltaBuffer[ltaIndex] = accDyn;
      ltaSum += ltaBuffer[ltaIndex];
      ltaIndex = (ltaIndex + 1) % LTA_WINDOW;
      float lta = ltaSum / LTA_WINDOW;

      // Rasio STA/LTA
      if (sampleCount < LTA_WINDOW) {
        // Tunggu 20 detik pertama sampai laci LTA penuh
        sampleCount++;
      } else {
        // Laci sudah penuh, hitung rasio dengan aman
        if (lta > 0.001f) { // Mencegah dibagi dengan 0
          staLtaRatio = sta / lta;
        } else {
          staLtaRatio = 0.0f;
        }
      }

      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;
    }

    // Read gyroscope data
    uint32_t lastGyroTime = currentGyroTime; // Previous time is stored before the actual time read
    currentGyroTime = millis(); // Current time actual time read
    float elapsedTime = (currentGyroTime - lastGyroTime) * 0.001f; // Divide by 1000 to get seconds
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDRESS, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
      gyroX = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // For a 250 deg/s range we have to divide first the raw value by 131.0, according to the datasheet
      gyroY = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
      gyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
      // Correct the outputs with the calculated error values
      gyroX = gyroX - gyroErrorX; // gyroErrorX
      gyroY = gyroY - gyroErrorY; // gyroErrorY
      gyroZ = gyroZ - gyroErrorZ; // gyroErrorZ
      // Complementary filter - combine acceleromter and gyro angle values
      // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
      roll = 0.96f * (roll + (gyroX * elapsedTime)) + 0.04f * accAngleX; // deg/s * s = deg
      pitch = 0.96f * (pitch + (gyroY * elapsedTime)) + 0.04f * accAngleY;
      // yaw = yaw + gyroZ * elapsedTime;

      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;
    }

    bool dangerState = evaluateSafetyThresholds(NODE_NAME, staLtaRatio, roll, pitch, gyroX, gyroY, gyroZ);
    static bool lastDangerState = false;

    // Penerusan LoRa payload dari node sebelumnya
    if (hasPendingLoRaData) {
      if (dangerState && !lastDangerState) { // Kalau node ini melewati threshold dan ada LoRa payload yang diterima
        JsonDocument json;
        deserializeJson(json, pendingLoRaPayload);
        JsonObject nodeData = json[NODE_NAME].add<JsonObject>();

        nodeData["acc_dyn"] = round(accDyn * 100.0f) / 100.0f;
        nodeData["sta_lta_ratio"] = round(staLtaRatio * 100.0f) / 100.0f;
        nodeData["roll"] = round(roll * 100.0f) / 100.0f;
        nodeData["pitch"] = round(pitch * 100.0f) / 100.0f;
        nodeData["gyro_x"] = round(gyroX * 100.0f) / 100.0f;
        nodeData["gyro_y"] = round(gyroY * 100.0f) / 100.0f;
        nodeData["gyro_z"] = round(gyroZ * 100.0f) / 100.0f;

        // Cek secara virtual ukuran payload jika digabung
        size_t estimatedSize = measureJson(json);

        if (estimatedSize <= RH_RF95_MAX_MESSAGE_LEN) { // Aman! Ukuran muat, jadikan satu string dan kirim!
          char jsonPayload[256];
          serializeJson(json, jsonPayload);

          Serial.println(F("----------------\r\nLoRa JSON payload merged!"));
          loRaTransmit(jsonPayload);
        } else {
          // Terlalu besar! Batalkan penggabungan, kirim berurutan secara utuh!
          Serial.println(F("----------------\r\nMerged JSON exceeds 251 bytes! Splitting transmission..."));
          
          // Meneruskan LoRa payload
          loRaTransmit(pendingLoRaPayload); 
          
          // Transmit data node ini
          char jsonPayload[256];
          buildJsonPayload(jsonPayload, sizeof(jsonPayload), NODE_NAME, accDyn, staLtaRatio, roll, pitch, gyroX, gyroY, gyroZ);

          loRaTransmit(jsonPayload); 
        }
        
        lastTxTime = currentTime;
      } else { // Kalau node ini tidak melewati threshold hanya meneruskan LoRa payload
        loRaTransmit(pendingLoRaPayload);
      }

      hasPendingLoRaData = false;
    } else if (dangerState && !lastDangerState) { // Kalau node ini melewati threshold tapi tidak ada LoRa payload yang diterima
      char jsonPayload[256];
      buildJsonPayload(jsonPayload, sizeof(jsonPayload), NODE_NAME, accDyn, staLtaRatio, roll, pitch, gyroX, gyroY, gyroZ);

      loRaTransmit(jsonPayload);

      lastTxTime = currentTime;
    }

    lastDangerState = dangerState;
    lastSampleTime = currentTime;
  }

  // Kirim data ke ThingsBoard melalui MQTT
  if (currentTime - lastTxTime >= LORA_TX_INTERVAL) {
    char jsonPayload[256];
    buildJsonPayload(jsonPayload, sizeof(jsonPayload), NODE_NAME, accDyn, staLtaRatio, roll, pitch, gyroX, gyroY, gyroZ);

    loRaTransmit(jsonPayload);

    lastTxTime = currentTime;
  }

  // Cek konektivitas sensor MPU-6050
  checkMPUConnectivity();
}

void checkMPUConnectivity() {
  if (mpuConnectionState != lastMpuConnectionState) {
    if (!mpuConnectionState) { // Hanya kalau false/disconnected
      Serial.println(F("----------------\r\nSensor MPU-6050 disconnected!"));
    } else {
      Serial.println(F("----------------\r\nSensor MPU-6050 reconnected!"));
    }

    lastMpuConnectionState = mpuConnectionState;
  }
}

void calculateIMUError() {
  uint8_t c = 0;
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDRESS, 6, true) == 6) {
      float accX = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE;
      float accY = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE;
      float accZ = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE;
      // Sum all readings
      accErrorX = accErrorX + (atan2f(accY, sqrtf((accX * accX) + (accZ * accZ))) * RAD_TO_DEG_F);
      accErrorY = accErrorY + (atan2f(-accX, sqrtf((accY * accY) + (accZ * accZ))) * RAD_TO_DEG_F);

      c++;
      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;

      delay(10);
    }

    checkMPUConnectivity();
  }

  // Divide the sum by 200 to get the error value
  accErrorX = accErrorX * 0.005f;
  accErrorY = accErrorY * 0.005f;
  c = 0;
  lastMpuConnectionState = true;

  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDRESS, 6, true) == 6) {
      float gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
      float gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
      float gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
      // Sum all readings
      gyroErrorX = gyroErrorX + (gyroX * GYRO_SCALE);
      gyroErrorY = gyroErrorY + (gyroY * GYRO_SCALE);
      gyroErrorZ = gyroErrorZ + (gyroZ * GYRO_SCALE);

      c++;
      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;

      delay(10);
    }

    checkMPUConnectivity();
  }

  //Divide the sum by 200 to get the error value
  gyroErrorX = gyroErrorX * 0.005f;
  gyroErrorY = gyroErrorY * 0.005f;
  gyroErrorZ = gyroErrorZ * 0.005f;
  lastMpuConnectionState = true;

  // Print the error values on the Serial Monitor
  Serial.print(F("----------------\r\nAccErrorX: "));
  Serial.println(accErrorX);
  Serial.print(F("AccErrorY: "));
  Serial.println(accErrorY);
  Serial.print(F("GyroErrorX: "));
  Serial.println(gyroErrorX);
  Serial.print(F("GyroErrorY: "));
  Serial.println(gyroErrorY);
  Serial.print(F("GyroErrorZ: "));
  Serial.println(gyroErrorZ);
}

void buildJsonPayload(char *outputBuffer, size_t maxLen, const char *nodeName, float accDyn, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ) {
  JsonDocument json;
  JsonObject nodeData = json[nodeName].add<JsonObject>();

  nodeData["acc_dyn"] = round(accDyn * 100.0f) / 100.0f;
  nodeData["sta_lta_ratio"] = round(staLtaRatio * 100.0f) / 100.0f;
  nodeData["roll"] = round(roll * 100.0f) / 100.0f;
  nodeData["pitch"] = round(pitch * 100.0f) / 100.0f;
  nodeData["gyro_x"] = round(gyroX * 100.0f) / 100.0f;
  nodeData["gyro_y"] = round(gyroY * 100.0f) / 100.0f;
  nodeData["gyro_z"] = round(gyroZ * 100.0f) / 100.0f;

  serializeJson(json, outputBuffer, maxLen);
}

void loRaTransmit(const char *jsonPayload) {
  size_t msgLen = strlen(jsonPayload) + 1; // Tentukan panjang pesan asli (+1 untuk \0)

  Serial.println(F("----------------"));

  if (msgLen > RH_RF95_MAX_MESSAGE_LEN) {
    Serial.println(F("Buffer overflow, string cut! Truncating to 251 bytes"));
    msgLen = RH_RF95_MAX_MESSAGE_LEN; // Paksa panjang maksimal 251 (termasuk \0)
  }

  Serial.print(F("Transmit message to ID: "));
  Serial.println(RECEIVER_ID);
  Serial.println(F("Message:"));
  Serial.println(jsonPayload);

  // Kirim data dan tunggu tanda terima (ACK)
  // Fungsi sendtoWait akan me-return true hanya jika ACK berhasil diterima dari penerima
  if (manager.sendtoWait((uint8_t*)jsonPayload, msgLen, RECEIVER_ID)) {
    Serial.println(F("Transmission success! Validated by receiver!"));
  } else {
    Serial.println(F("Transmission failed! There is no ACK response!"));
  }
}

bool evaluateSafetyThresholds(const char *nodeName, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ) {
  float absRoll = fabsf(roll);
  float absPitch = fabsf(pitch);
  bool isDangerDetected = false;

  // Cek threshold gempa
  if (staLtaRatio > 3.0f) {
    isDangerDetected = true;
  }

  // Cek threshold perubahan sudut kemiringan tanah
  if (absRoll > 5.0f || absPitch > 5.0f) {
    isDangerDetected = true;
  } else if ((absRoll >= 2.0f && absRoll <= 5.0f) || (absPitch >= 2.0f && absPitch <= 5.0f)) {
    isDangerDetected = true;
  }

  // Cek threshold kecepatan sudut
  if (fabsf(gyroX) > 10.0f || fabsf(gyroY) > 10.0f || fabsf(gyroZ) > 10.0f) {
    isDangerDetected = true;
  }

  return isDangerDetected;
}