#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include "secrets.h"

// Deklarasi variabel global
constexpr const uint16_t WIFI_RECONNECT_INTERVAL = 5000; // 5s

constexpr const char *MQTT_TOPIC_GATEWAY_TELE_PUB = "v1/gateway/telemetry";
constexpr const uint16_t MQTT_RECONNECT_INTERVAL = 5000; // 5s
constexpr const uint16_t MQTT_PUBLISH_INTERVAL = 2000; // 2s

constexpr const uint8_t MPU_ADDRESS = 0x68; // MPU-6050 I2C address
constexpr const uint8_t MPU_SAMPLING_INTERVAL = 10; // 10ms

constexpr const uint8_t LORA_CS = 5;
constexpr const uint8_t LORA_IRQ = 4;
constexpr const uint8_t LORA_RST = 26;
constexpr const uint16_t LORA_CHECK_INTERVAL = 1000; // 1s
constexpr const uint8_t NODE_ID = 0; // Alamat ID node ini

constexpr const float RAD_TO_DEG_F = 57.29577951f; // 180 / PI(3.14)
constexpr const float ACCEL_SCALE = 1.0f / 16384.0f; // / 16384
constexpr const float GYRO_SCALE  = 1.0f / 131.0f; // / 131
constexpr const uint8_t STA_WINDOW = 100; // 1 detik (100 sampel)
constexpr const uint16_t LTA_WINDOW = 2000; // 20 detik (2000 sampel)

float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
bool mpuConnectionState, lastMpuConnectionState = true;
uint32_t currentGyroTime;
// volatile bool earthquakeFlag = false, angleAlertFlag = false, angleDangerFlag = false, angularVelocityFlag = false;

WiFiClient wiFiClient;
WiFiClientSecure secureWiFiClient;
PubSubClient mqttClient(wiFiClient);
RH_RF95 rf95(LORA_CS, LORA_IRQ);
RHReliableDatagram manager(rf95, NODE_ID);

// Gembok pelindung tabrakan antar Core
SemaphoreHandle_t alertMutex;

// Prototipe fungsi
void telegramTask(void *pvParameters);
void urlEncode(const char *str, char *encodedStr, size_t maxLen);
void sendMessageToTelegram(const char *message);
bool evaluateSafetyThresholds(const char *nodeName, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ);
void mqttReconnect();
void buildJsonPayload(char *outputBuffer, size_t maxLen, const char *nodeName, float accDyn, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ);
void mqttPublishToThingsBoard(const char *jsonPayload);
void checkMPUConnectivity();
void calculateIMUError();

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi LoRa SX1276 (sama dengan RF95)
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

  // Inisialisasi sensor MPU-6050
  Serial.println(F("----------------\r\nInitializing MPU-6050..."));
  Wire.begin();                                   // Initialize comunication
  Wire.beginTransmission(MPU_ADDRESS);                // Start communication with MPU6050 // MPU_ADDRESS=0x68
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

  // Menghubungkan ke jaringan Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print(F("----------------\r\nConnecting to Wi-Fi"));
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("."));

    delay(500);
  }
  Serial.println(F("\r\nConnected to the Wi-Fi network!"));

  secureWiFiClient.setInsecure(); // Bypass verifikasi sertifikat SSL

  // Menghubungkan ke MQTT Broker
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  while (!mqttClient.connected()) {
    Serial.println(F("----------------\r\nConnecting to MQTT Broker..."));
    if (mqttClient.connect("LandSlidEtect_Gateway", GATEWAY_ACCESS_TOKEN, "")) {
      Serial.println(F("Connected to MQTT Broker!"));
    } else {
      Serial.print(F("Failed with state "));
      Serial.print(mqttClient.state());
      Serial.println(F(" try again in 5 seconds!"));

      delay(5000);
    }
  }

  // Hitung error sensor MPU-6050
  calculateIMUError();
  delay(20);

  // Membuat gembok RTOS
  alertMutex = xSemaphoreCreateMutex();

  // Bangunkan Core 0
  xTaskCreatePinnedToCore(
    telegramTask,   // Nama fungsi Task yang dipanggil
    "TelegramTask", // Nama alias untuk debugging
    10240,          // Ukuran RAM khusus untuk Task ini
    NULL,           // Parameter (kosongkan)
    1,              // Prioritas (1 = Standar)
    NULL,           // Task Handle (kosongkan)
    0               // Pin ke Core 0
  );

  currentGyroTime = millis();

  // sendMessageToTelegram("Halo dari ESP32!\nフリーナちゃんとビビアンくん");
}

void loop() {
  uint32_t currentTime = millis();
  static bool lastWiFiState = true;

  // Cek konektivitas Wi-Fi dan MQTT
  bool wiFiState = (WiFi.status() == WL_CONNECTED);

  if (!wiFiState) { // Kalau disconnected
    static uint32_t lastWiFiReconnectTime = 0;

    if (currentTime - lastWiFiReconnectTime >= WIFI_RECONNECT_INTERVAL) {
      Serial.println(F("----------------\r\nWi-Fi disconnected! Attempting to reconnect..."));
      WiFi.reconnect();
      
      lastWiFiReconnectTime = currentTime;
    }
  } else {
    if (!lastWiFiState) {
      Serial.println(F("----------------\r\nWi-Fi reconnect success!"));
    }

    if (!mqttClient.connected()) {
      static uint32_t lastMqttReconnectTime = 0;

      if (currentTime - lastMqttReconnectTime >= MQTT_RECONNECT_INTERVAL) {
        mqttReconnect();
        
        lastMqttReconnectTime = currentTime;
      }
    } else { // Kalau tidak ada masalah
      mqttClient.loop();
    }
  }

  lastWiFiState = wiFiState;

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
        Serial.println(F("----------------\r\nLoRa reinit success!"));
      } else {
        Serial.println(F("----------------\r\nLoRa reinit failed!"));
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

      // Parsing JSON string
      JsonDocument json;
      DeserializationError error = deserializeJson(json, loRaPayload);

      Serial.println(F("----------------"));
      if (error) {
        Serial.print(F("Failed to parsing JSON LoRa!, Error: "));
        Serial.println(error.f_str());
      } else {
        strncpy(pendingLoRaPayload, loRaPayload, sizeof(pendingLoRaPayload));
        hasPendingLoRaData = true; // Angkat bendera

        mqttPublishToThingsBoard(loRaPayload);
      }

      // Ambil nilai RSSI (Kekuatan Sinyal)
      int16_t rssi = rf95.lastRssi();

      Serial.print(F("Message received from ID: "));
      Serial.println(fromAddress);
      Serial.println(F("Message:"));
      Serial.println(loRaPayload);
      Serial.print(F("RSSI: "));
      Serial.print(rssi);
      Serial.println(F(" dBm"));
      Serial.println(F("Automatically send ACK reply to sending node..."));
      Serial.println(F("LoRa starting to listen again..."));
    }
  }

  static float accDyn;
  static float gyroX, gyroY, gyroZ;
  static float roll = 0.0f, pitch = 0.0f/* , yaw = 0.0f */;
  static float staLtaRatio = 0.0f;
  static uint32_t lastSampleTime = 0, lastPublishTime = 0;

  // Pembacaan sensor MPU-6050
  if (currentTime - lastSampleTime >= MPU_SAMPLING_INTERVAL) {
    static float accAngleX = 0.0f, accAngleY = 0.0f;

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
    static uint32_t lastGyroTime = currentGyroTime; // Previous time is stored before the actual time read
    currentGyroTime = millis(); // Current time actual time read
    float elapsedTime = (currentGyroTime - lastGyroTime) * 0.001f; // Divide by 1000 to get seconds
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDRESS, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
      gyroX = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
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

    bool dangerState = false;

    // Minta izin memegang gembok (Tunggu maksimal sampai diizinkan)
    if (xSemaphoreTake(alertMutex, portMAX_DELAY) == pdTRUE) {
      dangerState = evaluateSafetyThresholds("Gateway", staLtaRatio, roll, pitch, gyroX, gyroY, gyroZ);

      if (hasPendingLoRaData) {
        JsonDocument json;
        deserializeJson(json, pendingLoRaPayload);
        JsonObject root = json.as<JsonObject>();

        for (JsonPair kv : root) {
          const char *nodeName = kv.key().c_str(); // Mengambil key atau nama "Node 1" atau "Node 2"
          JsonArray dataArray = kv.value().as<JsonArray>();
          JsonObject data = dataArray[0];

          float extStaLtaRatio = data["sta_lta_ratio"];
          float extRoll = data["roll"];
          float extPitch = data["pitch"];
          float extGx = data["gyro_x"];
          float extGy = data["gyro_y"];
          float extGz = data["gyro_z"];

          // Hakim mengevaluasi data dari Node LoRa
          evaluateSafetyThresholds(nodeName, extStaLtaRatio, extRoll, extPitch, extGx, extGy, extGz);
        }

        hasPendingLoRaData = false;
      }

      // Kembalikan gembok! (Sangat penting agar Core 0 tidak membeku)
      xSemaphoreGive(alertMutex);
    }

    static bool lastDangerState = false;

    if (dangerState && !lastDangerState) {
      char jsonPayload[256];
      buildJsonPayload(jsonPayload, sizeof(jsonPayload), "Gateway", accDyn, staLtaRatio, roll, pitch, gyroX, gyroY, gyroZ);

      mqttPublishToThingsBoard(jsonPayload);

      lastPublishTime = currentTime;
    }

    lastDangerState = dangerState;
    lastSampleTime = currentTime;
  }

  // Kirim data ke ThingsBoard melalui MQTT
  if (currentTime - lastPublishTime >= MQTT_PUBLISH_INTERVAL) {
    char jsonPayload[256];
    buildJsonPayload(jsonPayload, sizeof(jsonPayload), "Gateway", accDyn, staLtaRatio, roll, pitch, gyroX, gyroY, gyroZ);

    mqttPublishToThingsBoard(jsonPayload);

    lastPublishTime = currentTime;
  }

  // Cek konektivitas sensor MPU-6050
  checkMPUConnectivity();
}

void urlEncode(const char *str, char *encodedStr, size_t maxLen) {
  size_t encodedIdx = 0;

  for (size_t i = 0; i < strlen(str); i++) {
    // Cegah buffer overflow dengan pastikan wadah masih muat untuk 3 karakter ("%XX") + 1 karakter penutup ('\0')
    if (encodedIdx + 3 >= maxLen - 1) {
      Serial.println(F("Buffer overflow, string cut!"));
      break; 
    }

    uint8_t c = (uint8_t)str[i];

    // Menurut RFC 3986, huruf, angka, dan 4 simbol ini tidak boleh di-encode
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encodedStr[encodedIdx++] = c;
    } else { // Sisanya (termasuk spasi dan simbol unik), ubah ke format %HEX
      snprintf(&encodedStr[encodedIdx], 4, "%%%02X", c);
      encodedIdx += 3;
    }
  }

  // Null Terminator
  encodedStr[encodedIdx] = '\0';
}

void sendMessageToTelegram(const char *message) {
  char encodedMsg[512];
  urlEncode(message, encodedMsg, sizeof(encodedMsg));

  char apiUrl[1024];
  snprintf(apiUrl, sizeof(apiUrl),
           "https://api.telegram.org/bot%s/sendMessage?chat_id=%s&text=%s",
           TELEGRAM_BOT_API_TOKEN, CHAT_ID, encodedMsg);
  
  Serial.println(F("----------------\r\nTrying request to: "));
  Serial.println(apiUrl);

  HTTPClient http;
  http.begin(secureWiFiClient, apiUrl);

  int16_t httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print(F("----------------\r\nHTTP Response Code: "));
    Serial.println(httpResponseCode);

    // (Catatan: http.getString() masih menggunakan String, tapi tidak apa-apa untuk 
    // respons sekali lewat. Jika ingin 100% C-String, bisa pakai http.getStream())
    String payload = http.getString();
    Serial.println(F("Server Response: "));
    Serial.println(payload);
  } else {
    Serial.print(F("Error Code: "));
    Serial.println(httpResponseCode);
  }

  http.end();
}

void mqttReconnect() {
  Serial.println(F("----------------\r\nMQTT disconnected! Reconnecting to MQTT Broker..."));
  if (mqttClient.connect("LandSlidEtect_Gateway", GATEWAY_ACCESS_TOKEN, "")) {
    Serial.println(F("Reconnected to MQTT Broker!"));
  } else {
    Serial.print(F("Failed with state "));
    Serial.print(mqttClient.state());
    Serial.println(F(" try again in 5 seconds!"));
  }
}

void buildJsonPayload(char *outputBuffer, size_t maxLen, const char *nodeName, float accDyn, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ) {
  JsonDocument json;
  JsonObject gatewayData = json[nodeName].add<JsonObject>();

  gatewayData["acc_dyn"] = round(accDyn * 100.0f) / 100.0f;
  gatewayData["sta_lta_ratio"] = round(staLtaRatio * 100.0f) / 100.0f;
  gatewayData["roll"] = round(roll * 100.0f) / 100.0f;
  gatewayData["pitch"] = round(pitch * 100.0f) / 100.0f;
  gatewayData["gyro_x"] = round(gyroX * 100.0f) / 100.0f;
  gatewayData["gyro_y"] = round(gyroY * 100.0f) / 100.0f;
  gatewayData["gyro_z"] = round(gyroZ * 100.0f) / 100.0f;

  serializeJson(json, outputBuffer, maxLen);
}

void mqttPublishToThingsBoard(const char *jsonPayload) {
  // Cek apakah Wi-Fi dan MQTT masih terhubung
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    Serial.println(F("----------------\r\nPublish:"));
    Serial.println(jsonPayload);
    
    mqttClient.publish(MQTT_TOPIC_GATEWAY_TELE_PUB, jsonPayload);
  } else {
    Serial.println(F("----------------\r\nFailed to publish! Wi-Fi or MQTT Broker disconnected!"));
  }
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

char alertEarthquakeNodes[128] = "";
char dangerAngleNodes[128] = "";
char alertAngleNodes[128] = "";
char dangerVelocityNodes[128] = "";

bool evaluateSafetyThresholds(const char *nodeName, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ) {
  float absRoll = fabsf(roll);
  float absPitch = fabsf(pitch);
  bool isDangerDetected = false;
  
  // Cek threshold gempa
  if (staLtaRatio > 3.0f) {
    isDangerDetected = true;
    // Jika nama node belum ada di dalam daftar (strstr mengembalikan NULL)
    if (strstr(alertEarthquakeNodes, nodeName) == NULL) {
      // Cek apakah wadah masih muat
      if (strlen(alertEarthquakeNodes) + strlen(nodeName) + 3 <= sizeof(alertEarthquakeNodes)) {
        strcat(alertEarthquakeNodes, nodeName);
        strcat(alertEarthquakeNodes, ", ");
      }
    }
  }

  // Cek threshold perubahan sudut kemiringan tanah
  if (absRoll > 5.0f || absPitch > 5.0f) {
    isDangerDetected = true;
    if (strstr(dangerAngleNodes, nodeName) == NULL) {
      if (strlen(dangerAngleNodes) + strlen(nodeName) + 3 <= sizeof(dangerAngleNodes)) {
        strcat(dangerAngleNodes, nodeName);
        strcat(dangerAngleNodes, ", ");
      }
    }
  } else if ((absRoll >= 2.0f && absRoll <= 5.0f) || (absPitch >= 2.0f && absPitch <= 5.0f)) {
    isDangerDetected = true;
    if (strstr(alertAngleNodes, nodeName) == NULL) {
      if (strlen(alertAngleNodes) + strlen(nodeName) + 3 <= sizeof(alertAngleNodes)) {
        strcat(alertAngleNodes, nodeName);
        strcat(alertAngleNodes, ", ");
      }
    }
  }

  // Cek threshold kecepatan sudut
  if (fabsf(gyroX) > 10.0f || fabsf(gyroY) > 10.0f || fabsf(gyroZ) > 10.0f) {
    isDangerDetected = true;
    if (strstr(dangerVelocityNodes, nodeName) == NULL) {
      if (strlen(dangerVelocityNodes) + strlen(nodeName) + 3 <= sizeof(dangerVelocityNodes)) {
        strcat(dangerVelocityNodes, nodeName);
        strcat(dangerVelocityNodes, ", ");
      }
    }
  }

  return isDangerDetected;
}

// RTOS Core 0 untuk menangani pengiriman pesan ke telegram
void telegramTask(void *pvParameters) {
  const uint16_t TELEGRAM_COOLDOWN = 60000; // Jeda 1 menit
  uint32_t lastEarthquakeTelegramSentTime = 0, lastAngleAlertTelegramSentTime = 0, lastAngleDangerTelegramSentTime = 0, lastAngularVelocityTelegramSentTime = 0;

  for (;;) { // Looping abadi khusus Core 0
    uint32_t currentTaskTime = millis();

    if (WiFi.status() == WL_CONNECTED) { // Kalau terhubung ke internet
      char localAlertEarthquakeNodes[128] = "";
      char localDangerAngleNodes[128] = "";
      char localAlertAngleNodes[128] = "";
      char localDangerVelocityNodes[128] = "";

      // Pegang gembok
      if (xSemaphoreTake(alertMutex, portMAX_DELAY) == pdTRUE) {
        // Jika ada isi, copy ke lokal, lalu kosongkan yang global (\0 adalah Null Terminator)
        if (strlen(alertEarthquakeNodes) > 0) {
          strcpy(localAlertEarthquakeNodes, alertEarthquakeNodes);
          alertEarthquakeNodes[0] = '\0'; 
        }
        if (strlen(dangerAngleNodes) > 0) {
          strcpy(localDangerAngleNodes, dangerAngleNodes);
          dangerAngleNodes[0] = '\0';
        }
        if (strlen(alertAngleNodes) > 0) {
          strcpy(localAlertAngleNodes, alertAngleNodes);
          alertAngleNodes[0] = '\0';
        }
        if (strlen(dangerVelocityNodes) > 0) {
          strcpy(localDangerVelocityNodes, dangerVelocityNodes);
          dangerVelocityNodes[0] = '\0';
        }
        
        xSemaphoreGive(alertMutex); // Buka gembok secepatnya!
      }

      if (strlen(localAlertEarthquakeNodes) > 0) {
        if (currentTaskTime - lastEarthquakeTelegramSentTime >= TELEGRAM_COOLDOWN || lastEarthquakeTelegramSentTime == 0) {
          char msg[256];
          snprintf(msg, sizeof(msg), "https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n⚠️ Waspada!\nTerdeteksi getaran tanah yang berasal dari gempa di titik: %sberpotensi longsor!", localAlertEarthquakeNodes);
          
          Serial.println(F("----------------\r\n[Core 0] Alert, Earthquake! Sending message to Telegram..."));
          sendMessageToTelegram(msg);
          
          lastEarthquakeTelegramSentTime = currentTaskTime;
        }
      }
      if (strlen(localAlertAngleNodes) > 0) {
        if (currentTaskTime - lastAngleAlertTelegramSentTime >= TELEGRAM_COOLDOWN || lastAngleAlertTelegramSentTime == 0) {
          char msg[256];
          snprintf(msg, sizeof(msg), "https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n⚠️ Waspada!\nTerdeteksi perubahan sudut kemiringan tanah di titik: %sberpotensi longsor!", localAlertAngleNodes);
          
          Serial.println(F("----------------\r\n[Core 0] Alert, The land is getting sloping! Sending message to Telegram..."));
          sendMessageToTelegram(msg);
          
          lastAngleAlertTelegramSentTime = currentTaskTime;
        }
      }
      if (strlen(localDangerAngleNodes) > 0) {
        if (currentTaskTime - lastAngleDangerTelegramSentTime >= TELEGRAM_COOLDOWN || lastAngleDangerTelegramSentTime == 0) {
          char msg[256];
          snprintf(msg, sizeof(msg), "https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n🚨 Bahaya!\nTerdeteksi perubahan sudut kemiringan tanah semakin tinggi di titik: %skemungkinan besar longsor!", localDangerAngleNodes);
          
          Serial.println(F("----------------\r\n[Core 0] Danger, The land is getting more sloping! Sending message to Telegram..."));
          sendMessageToTelegram(msg);
          
          lastAngleDangerTelegramSentTime = currentTaskTime;
        }
      }
      if (strlen(localDangerVelocityNodes) > 0) {
        if (currentTaskTime - lastAngularVelocityTelegramSentTime >= TELEGRAM_COOLDOWN || lastAngularVelocityTelegramSentTime == 0) {
          char msg[256];
          snprintf(msg, sizeof(msg), "https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n🚨 Bahaya!\nTerdeteksi alat pemantauan jatuh di titik: %sindikasi terjadinya tanah longsor, periksa lereng segera!", localDangerVelocityNodes);
          
          Serial.println(F("----------------\r\n[Core 0] Danger, The landslide! Sending message to Telegram..."));
          sendMessageToTelegram(msg);
          
          lastAngularVelocityTelegramSentTime = currentTaskTime;
        }
      }
    }

    // Istirahatkan Core 0 selama 100ms agar tidak panas (Watchdog friendly)
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}