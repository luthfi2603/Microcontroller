// #define TINY_GSM_DEBUG Serial
#define TINY_GSM_YIELD() { delay(2); }
#define TINY_GSM_RX_BUFFER 2048
#define TINY_GSM_MODEM_SIM800
// #include <StreamDebugger.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include "secrets.h"

// Deklarasi variabel global
constexpr const uint8_t RXD2 = 16;
constexpr const uint8_t TXD2 = 17;
constexpr const uint16_t NETWORK_4G_RECONNECT_INTERVAL = 5000; // 5 s

constexpr const char *MQTT_TOPIC_GATEWAY_TELE_PUB = "v1/gateway/telemetry";
constexpr const uint16_t MQTT_RECONNECT_INTERVAL = 5000; // 5 s
constexpr const uint16_t MQTT_PUBLISH_INTERVAL = 5000; // 5 s

constexpr const uint32_t TELEGRAM_COOLDOWN = 60000; // 60 s

constexpr const uint8_t MPU_ADDRESS = 0x68; // MPU-6050 I2C address
constexpr const uint8_t MPU_SAMPLING_INTERVAL = 10; // 10 ms

constexpr const uint8_t LORA_CS = 5;
constexpr const uint8_t LORA_IRQ = 4;
constexpr const uint8_t LORA_RST = 26;
constexpr const uint16_t LORA_CHECK_INTERVAL = 5000; // 5 s
constexpr const uint8_t NODE_ID = 0; // Alamat ID node ini

constexpr const float RAD_TO_DEG_F = 57.29577951f; // 180 / PI(3.14)
constexpr const float ACCEL_SCALE = 1.0f / 16384.0f; // / 16384
constexpr const float GYRO_SCALE  = 1.0f / 131.0f; // / 131
constexpr const uint8_t STA_WINDOW = 100; // 1 detik (100 sampel)
constexpr const uint16_t LTA_WINDOW = 2000; // 20 detik (2000 sampel)

float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
bool mpuConnectionState, lastMpuConnectionState = true;
uint32_t currentGyroTime;
volatile bool network4GState = true;

/* StreamDebugger debugger(Serial2, Serial);
TinyGsm modem(debugger); */
TinyGsm modem(Serial2);
TinyGsmClient cellularClient(modem, 0);
TinyGsmClientSecure secureCellularClient(modem, 1); // Default bypass verifikasi sertifikat SSL
PubSubClient mqttClient(cellularClient);
RH_RF95 rf95(LORA_CS, LORA_IRQ);
RHReliableDatagram manager(rf95, NODE_ID);
SemaphoreHandle_t alertMutex; // Gembok pelindung tabrakan antar core
SemaphoreHandle_t modemMutex;

// Prototipe fungsi
void init4G();
void initLoRa();
void initMPU();
void calculateIMUError();
void checkMPUConnectivity();
void initMQTT();
void mqttReconnect();
void urlEncode(const char *str, char *encodedStr, size_t maxLen);
void sendMessageToTelegram(const char *message);
void buildJsonPayload(char *outputBuffer, size_t maxLen, const char *nodeName, float accDyn, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ);
void mqttPublishToThingsBoard(const char *jsonPayload);
bool evaluateSafetyThresholds(const char *nodeName, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ);
void telegramTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Inisialisasi jalur hardware serial 2 menuju modul 4G
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(3000);

  // Inisialisasi jaringan seluler 4G LTE menggunakan modul Air780E
  init4G();

  // Inisialisasi LoRa SX1276
  initLoRa();

  // Inisialisasi sensor MPU-6050
  initMPU();

  // Hitung error sensor MPU-6050
  calculateIMUError();
  delay(20);

  // Menghubungkan ke MQTT Broker
  initMQTT();

  // Membuat gembok RTOS
  alertMutex = xSemaphoreCreateMutex();
  modemMutex = xSemaphoreCreateMutex();

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

  // Cek konektivitas 4G LTE dan MQTT
  if (!network4GState) { // Kalau disconnected
    static uint32_t lastNetwork4GReconnectTime = 0;

    if (currentTime - lastNetwork4GReconnectTime >= NETWORK_4G_RECONNECT_INTERVAL) {
      if (xSemaphoreTake(modemMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println(F("----------------"));

        if (!modem.isNetworkConnected()) {
          Serial.println(F("4G LTE disconnected! Waiting for cellular signal..."));
        } else {
          if (!modem.isGprsConnected()) {
            Serial.println(F("4G LTE disconnected! Reconnecting GPRS..."));

            if (modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
              Serial.println(F("4G LTE reconnect success!"));
              network4GState = true;
            } else {
              Serial.println(F("4G LTE reconnect failed!"));
            }
          } else {
            network4GState = true;
          }
        }

        xSemaphoreGive(modemMutex);
      }

      lastNetwork4GReconnectTime = currentTime;
    }
  } else {
    if (xSemaphoreTake(modemMutex, portMAX_DELAY) == pdTRUE) {
      if (!mqttClient.connected()) {
        static uint32_t lastMqttReconnectTime = 0;

        if (currentTime - lastMqttReconnectTime >= MQTT_RECONNECT_INTERVAL) {
          mqttReconnect();
          
          lastMqttReconnectTime = currentTime;
        }
      } else { // Kalau tidak ada masalah
        mqttClient.loop();
      }

      xSemaphoreGive(modemMutex);
    }
  }

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
      Serial.println(F("LoRa starting to listen again..."));

      // Parsing JSON string
      JsonDocument json;
      DeserializationError error = deserializeJson(json, loRaPayload);

      if (error) {
        Serial.print(F("Failed to parsing JSON LoRa!, Error: "));
        Serial.println(error.f_str());
      } else {
        strncpy(pendingLoRaPayload, loRaPayload, sizeof(pendingLoRaPayload));
        hasPendingLoRaData = true; // Angkat bendera

        mqttPublishToThingsBoard(loRaPayload);
      }
    }
  }

  static float accDyn;
  static float gyroX, gyroY, gyroZ;
  static float roll = 0.0f, pitch = 0.0f/* , yaw = 0.0f */;
  static float staLtaRatio = 0.0f;
  static uint32_t lastSampleTime = 0, lastPublishTime = 0;

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

    if (dangerState && !lastDangerState) { // Publish data gateway hanya jika melewati threshold
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

void init4G() {
  Serial.println(F("\r\nInitializing 4G LTE modem..."));

  Serial.println(F("Checking modem AT response..."));
  if (!modem.testAT()) {
    Serial.println(F("Failed! Modem not responding to AT!"));
    while (1) { delay(1000); }
  }
  Serial.println(F("OK! Modem is awake!"));

  // Matikan fitur Echo (Pantulan Teks) secara manual agar library tidak bingung
  Serial2.print("ATE0\r\n"); 
  delay(100);

  // Blokir SMS agar tidak mengganggu jalur UART
  Serial.println(F("Disabling SMS notifications..."));
  Serial2.print("AT+CNMI=0,0,0,0,0\r\n");
  delay(100);

  // Bersihkan jalur UART
  while (Serial2.available()) {
    Serial2.read();
  }

  Serial.println(F("Waiting for network registration..."));
  if (!modem.waitForNetwork(60000L)) {
    Serial.println(F("Network registration failed! Check antenna or SIM status!"));
    while (1) { delay(1000); }
  }
  Serial.println(F("Registered to cellular network!"));

  Serial.print(F("Connecting to GPRS APN: "));
  Serial.println(APN);
  if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    Serial.println(F("GPRS connection failed!"));
    while (1) { delay(1000); }
  }
  Serial.println(F("GPRS connected! 4G LTE is active!"));
}

void initLoRa() {
  Serial.println(F("----------------\r\nInitializing LoRa with RadioHead..."));

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

void initMPU() {
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

void initMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(64, 320);

  while (!mqttClient.connected()) {
    Serial.println(F("----------------\r\nConnecting to MQTT Broker..."));
    if (mqttClient.connect(MQTT_CLIENT_ID, GATEWAY_ACCESS_TOKEN, "")) {
      Serial.println(F("Connected to MQTT Broker!"));
    } else {
      Serial.print(F("Failed with state "));
      Serial.print(mqttClient.state());
      Serial.println(F(" try again in 5 seconds!"));

      delay(5000);
    }
  }
}

void mqttReconnect() {
  Serial.println(F("----------------\r\nMQTT Broker disconnected! Reconnecting to MQTT Broker..."));

  if (mqttClient.connect(MQTT_CLIENT_ID, GATEWAY_ACCESS_TOKEN, "")) {
    Serial.println(F("Reconnected to MQTT Broker!"));
  } else {
    Serial.print(F("Failed with state "));
    Serial.print(mqttClient.state());
    Serial.println(F(" try again in 5 seconds!"));
  }
}

void urlEncode(const char *str, char *encodedStr, size_t maxLen) {
  size_t encodedIdx = 0;

  Serial.println(F("----------------"));

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
  char encodedMsg[1024];
  urlEncode(message, encodedMsg, sizeof(encodedMsg));

  char apiPath[1536];
  snprintf(apiPath, sizeof(apiPath),
           "/bot%s/sendMessage?chat_id=%s&text=%s",
           TELEGRAM_BOT_API_TOKEN, CHAT_ID, encodedMsg);
  
  Serial.println(F("Trying request to:"));
  Serial.print(F("https://api.telegram.org"));
  Serial.println(apiPath);

  if (xSemaphoreTake(modemMutex, portMAX_DELAY) == pdTRUE) {
    HttpClient http(secureCellularClient, "api.telegram.org", 443);
    http.connectionKeepAlive();
    http.setHttpResponseTimeout(5000);

    int result = http.get(apiPath);

    Serial.print(F("----------------\r\nGET result code: "));
    Serial.println(result);

    int httpResponseStatusCode = http.responseStatusCode();

    if (httpResponseStatusCode > 0) {
      Serial.print(F("HTTP response status code: "));
      Serial.println(httpResponseStatusCode);

      String response = http.responseBody();

      Serial.println(F("Server response:"));
      Serial.println(response);
    } else {
      Serial.print(F("HTTP error code: "));
      Serial.println(httpResponseStatusCode);

      network4GState = false; // Kalau gagal request
    }

    http.stop();

    /* if (secureCellularClient.connect("api.telegram.org", 443)) {
      Serial.print("connected=");
      Serial.println(secureCellularClient.connected());
      Serial.print("available=");
      Serial.println(secureCellularClient.available());
      
      secureCellularClient.print(
        String("GET ") + apiPath + " HTTP/1.1\r\n"
        "Host: api.telegram.org\r\n\r\n"
      );

      uint32_t start = millis();

      while (millis() - start < 10000) {
        int avail = secureCellularClient.available();

        if (avail > 0) {
          Serial.print("avail=");
          Serial.println(avail);
        }

        while (secureCellularClient.available()) {
          char c = secureCellularClient.read();
          Serial.write(c);
          start = millis();
        }

        if (!secureCellularClient.connected()) {
          Serial.println("DISCONNECTED");
          break;
        }

        delay(100);
      }

      secureCellularClient.stop();
    } */

    xSemaphoreGive(modemMutex);
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
  if (xSemaphoreTake(modemMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(F("----------------\r\nPublish:"));
    Serial.println(jsonPayload);

    // Cek apakah 4G LTE dan MQTT masih terhubung
    if (network4GState && mqttClient.connected()) {
        if (!mqttClient.publish(MQTT_TOPIC_GATEWAY_TELE_PUB, jsonPayload)) {
          Serial.println(F("Failed to publish! 4G LTE disconnected!"));

          network4GState = false;
        }
    } else {
      Serial.println(F("Failed to publish! 4G LTE or MQTT Broker disconnected!"));
    }

    xSemaphoreGive(modemMutex);
  }
}

struct NodeCooldown {
  const char *name;
  const char *mapUrl;
  uint32_t lastAlertEqTime;
  uint32_t lastDangerAngleTime;
  uint32_t lastAlertAngleTime;
  uint32_t lastDangerVelTime;
};

NodeCooldown nodeTimers[3] = {
  {"Gateway", "https://www.google.com/maps?q=3.603849926081428,98.66822999261507", 0, 0, 0, 0},
  {"Node 1", "https://www.google.com/maps?q=3.603852901277159,98.66777711800978", 0, 0, 0, 0},
  {"Node 2", "https://www.google.com/maps?q=3.603858320647524,98.66732609471924", 0, 0, 0, 0}
};

char alertEarthquakeNodes[256] = "";
char dangerAngleNodes[256] = "";
char alertAngleNodes[256] = "";
char dangerVelocityNodes[256] = "";

bool evaluateSafetyThresholds(const char *nodeName, float staLtaRatio, float roll, float pitch, float gyroX, float gyroY, float gyroZ) {
  uint32_t currentTime = millis();
  float absRoll = fabsf(roll);
  float absPitch = fabsf(pitch);
  bool isDangerDetected = false;

  int8_t idx = -1;
  for (uint8_t i = 0; i < 3; i++) {
    if (strcmp(nodeTimers[i].name, nodeName) == 0) {
      idx = i;
      break;
    }
  }

  // Jika nama node aneh/tidak terdaftar, abaikan
  if (idx == -1) return false;
  
  // Cek threshold gempa
  if (staLtaRatio > 3.0f) {
    isDangerDetected = true;

    // Cek cooldown khusus node ini dan hanya masukkan ke variabel global jika sudah lewat 1 menit
    if (currentTime - nodeTimers[idx].lastAlertEqTime >= TELEGRAM_COOLDOWN || nodeTimers[idx].lastAlertEqTime == 0) {
      // Jika nama node belum ada di dalam daftar (strstr mengembalikan NULL)
      if (strstr(alertEarthquakeNodes, nodeName) == NULL) {
        char tempBuffer[78];
        // Rakit format baris baru: Enter -> Dash -> Nama Node -> Titik Dua -> URL
        snprintf(tempBuffer, sizeof(tempBuffer), "\n- %s: %s", nodeName, nodeTimers[idx].mapUrl);
        
        // Cek apakah wadah 256 byte global masih muat
        if (strlen(alertEarthquakeNodes) + strlen(tempBuffer) < sizeof(alertEarthquakeNodes)) {
          strcat(alertEarthquakeNodes, tempBuffer);
        }
      }

      nodeTimers[idx].lastAlertEqTime = currentTime; // Update waktu lapor node ini
    }
  }

  // Cek threshold perubahan sudut kemiringan tanah
  if (absRoll > 5.0f || absPitch > 5.0f) {
    isDangerDetected = true;

    if (currentTime - nodeTimers[idx].lastDangerAngleTime >= TELEGRAM_COOLDOWN || nodeTimers[idx].lastDangerAngleTime == 0) {
      if (strstr(dangerAngleNodes, nodeName) == NULL) {
        char tempBuffer[78];
        snprintf(tempBuffer, sizeof(tempBuffer), "\n- %s: %s", nodeName, nodeTimers[idx].mapUrl);
        
        if (strlen(dangerAngleNodes) + strlen(tempBuffer) < sizeof(dangerAngleNodes)) {
          strcat(dangerAngleNodes, tempBuffer);
        }
      }

      nodeTimers[idx].lastDangerAngleTime = currentTime;
    }
  } else if ((absRoll >= 2.0f && absRoll <= 5.0f) || (absPitch >= 2.0f && absPitch <= 5.0f)) {
    isDangerDetected = true;

    if (currentTime - nodeTimers[idx].lastAlertAngleTime >= TELEGRAM_COOLDOWN || nodeTimers[idx].lastAlertAngleTime == 0) {
      if (strstr(alertAngleNodes, nodeName) == NULL) {
        char tempBuffer[78];
        snprintf(tempBuffer, sizeof(tempBuffer), "\n- %s: %s", nodeName, nodeTimers[idx].mapUrl);
        
        if (strlen(alertAngleNodes) + strlen(tempBuffer) < sizeof(alertAngleNodes)) {
          strcat(alertAngleNodes, tempBuffer);
        }
      }

      nodeTimers[idx].lastAlertAngleTime = currentTime;
    }
  }

  // Cek threshold kecepatan sudut
  if (fabsf(gyroX) > 10.0f || fabsf(gyroY) > 10.0f || fabsf(gyroZ) > 10.0f) {
    isDangerDetected = true;

    if (currentTime - nodeTimers[idx].lastDangerVelTime >= TELEGRAM_COOLDOWN || nodeTimers[idx].lastDangerVelTime == 0) {
      if (strstr(dangerVelocityNodes, nodeName) == NULL) {
        char tempBuffer[78];
        snprintf(tempBuffer, sizeof(tempBuffer), "\n- %s: %s", nodeName, nodeTimers[idx].mapUrl);
        
        if (strlen(dangerVelocityNodes) + strlen(tempBuffer) < sizeof(dangerVelocityNodes)) {
          strcat(dangerVelocityNodes, tempBuffer);
        }
      }

      nodeTimers[idx].lastDangerVelTime = currentTime;
    }
  }

  return isDangerDetected;
}

// RTOS Core 0 untuk menangani pengiriman pesan ke telegram
void telegramTask(void *pvParameters) {
  for (;;) { // Looping abadi khusus Core 0
    if (network4GState) { // Kalau terhubung ke internet
      char localAlertEarthquakeNodes[256] = "";
      char localDangerAngleNodes[256] = "";
      char localAlertAngleNodes[256] = "";
      char localDangerVelocityNodes[256] = "";

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

      // Cek dan kirim pesan ke telegram
      if (strlen(localAlertEarthquakeNodes) > 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "⚠️ Waspada!\nTerdeteksi getaran tanah yang berasal dari gempa berpotensi longsor di titik berikut:%s", localAlertEarthquakeNodes);

        Serial.println(F("----------------\r\n[Core 0] Alert, Earthquake! Sending message to Telegram..."));
        sendMessageToTelegram(msg);
      }
      if (strlen(localAlertAngleNodes) > 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "⚠️ Waspada!\nTerdeteksi perubahan sudut kemiringan tanah berpotensi longsor di titik berikut:%s", localAlertAngleNodes);

        Serial.println(F("----------------\r\n[Core 0] Alert, The land is getting sloping! Sending message to Telegram..."));
        sendMessageToTelegram(msg);
      }
      if (strlen(localDangerAngleNodes) > 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "🚨 Bahaya!\nTerdeteksi perubahan sudut kemiringan tanah semakin tinggi berkemungkinan besar longsor di titik berikut:%s", localDangerAngleNodes);

        Serial.println(F("----------------\r\n[Core 0] Danger, The land is getting more sloping! Sending message to Telegram..."));
        sendMessageToTelegram(msg);
      }
      if (strlen(localDangerVelocityNodes) > 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "🚨 Bahaya!\nTerdeteksi alat pemantauan jatuh di titik berikut:%s\nIndikasi terjadinya tanah longsor, periksa lereng segera!", localDangerVelocityNodes);

        Serial.println(F("----------------\r\n[Core 0] Danger, The landslide! Sending message to Telegram..."));
        sendMessageToTelegram(msg);
      }
    }

    // Istirahatkan Core 0 selama 100ms agar tidak panas (Watchdog friendly)
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}