#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include "secrets.h"

// Deklarasi variabel global
constexpr const char* MQTT_TOPIC_GATEWAY_TELE_PUB = "v1/gateway/telemetry";

constexpr const uint8_t MPU_ADDRESS = 0x68; // MPU-6050 I2C address
constexpr const float RAD_TO_DEG_F = 57.29577951f; // 180 / PI(3.14)
constexpr const float ACCEL_SCALE = 1.0f / 16384.0f; // / 16384
constexpr const float GYRO_SCALE  = 1.0f / 131.0f; // / 131
constexpr const uint8_t STA_WINDOW = 100; // 1 detik (100 sampel)
constexpr const uint16_t LTA_WINDOW = 2000; // 20 detik (2000 sampel)

float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
bool mpuConnectionState, lastMpuConnectionState = true;
uint32_t currentGyroTime;
volatile bool earthquakeFlag = false, angleAlertFlag = false, angleDangerFlag = false, angularVelocityFlag = false;

WiFiClient wiFiClient;
WiFiClientSecure secureWiFiClient;
PubSubClient mqttClient(wiFiClient);

// Prototipe fungsi
void telegramTask(void *pvParameters);
void urlEncode(const char *str, char *encodedStr, size_t maxLen);
void sendMessageToTelegram(const char *message);
void mqttReconnect();
void calculateIMUError();

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi sensor MPU-6050
  Serial.println(F("\r\nInitializing MPU-6050..."));
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

  // Bangunkan Core 0
  xTaskCreatePinnedToCore(
    telegramTask,   // Nama fungsi Task yang dipanggil
    "TelegramTask", // Nama alias untuk debugging
    10240,          // Ukuran RAM khusus untuk Task ini (4KB cukup untuk HTTP/JSON)
    NULL,           // Parameter (kosongkan)
    1,              // Prioritas (1 = Standar)
    NULL,           // Task Handle (kosongkan)
    0               // Pin ke Core 0
  );

  currentGyroTime = millis();

  // sendMessageToTelegram("Halo dari ESP32!\nフリーナちゃんとビビアンくん");
}

void loop() {
  static uint32_t lastGyroTime, lastSampleTime = 0;
  static float accAngleX = 0.0f, accAngleY = 0.0f;
  static float gyroX, gyroY, gyroZ;
  static float accDyn;
  static float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

  // Variabel untuk STA/LTA
  static float staBuffer[STA_WINDOW] = {0};
  static float ltaBuffer[LTA_WINDOW] = {0};
  static uint8_t staIndex = 0;
  static uint16_t ltaIndex = 0;
  static float staSum = 0.0f, ltaSum = 0.0f;
  static float ratioStaLta = 0.0f;
  static uint16_t sampleCount = 0;

  uint32_t currentTime = millis();
  static uint32_t lastMqttReconnectTime = 0, lastPublishTime = 0;

  if (!mqttClient.connected()) {
    if (currentTime - lastMqttReconnectTime >= 5000) {
      mqttReconnect();
      
      lastMqttReconnectTime = currentTime;
    }
  } else {
    mqttClient.loop();
  }

  if (currentTime - lastSampleTime >= 10) {
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
          ratioStaLta = sta / lta;
        } else {
          ratioStaLta = 0.0f;
        }
      }

      // Cek threshold gempa
      if (ratioStaLta > 3.0f) {
        earthquakeFlag = true;
      }

      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;
    }

    // Read gyroscope data
    lastGyroTime = currentGyroTime;        // Previous time is stored before the actual time read
    currentGyroTime = millis();            // Current time actual time read
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
      yaw = yaw + gyroZ * elapsedTime;

      float absRoll = fabsf(roll);
      float absPitch = fabsf(pitch);

      // Cek threshold perubahan sudut kemiringan
      if (absRoll > 5.0f || absPitch > 5.0f) {
        angleDangerFlag = true;
      } else if ((absRoll >= 2.0f && absRoll <= 5.0f) || (absPitch >= 2.0f && absPitch <= 5.0f)) {
        angleAlertFlag = true;
      }

      // Cek threshold kecepatan sudut
      if (fabsf(gyroX) > 10.0f || fabsf(gyroY) > 10.0f || fabsf(gyroZ) > 10.0f) {
        angularVelocityFlag = true;
      }

      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;
    }

    lastSampleTime = currentTime;
  }

  if (mqttClient.connected() && (currentTime - lastPublishTime >= 2000)) {
    JsonDocument json;

    /* // Parsing JSON string
    DeserializationError error = deserializeJson(json, loRaPayload);

    if (error) {
      Serial.print(F("Failed to parsing JSON LoRa!, Error: "));
      Serial.println(error.f_str());
    } else {
      float nilaiAx = json["ax"];
      float nilaiAy = json["ay"];
      const char* statusGempa = json["status_gempa"];
    } */

    JsonObject gatewayData = json["Gateway"].add<JsonObject>();

    gatewayData["acc_dyn"] = round(accDyn * 100.0f) / 100.0f;
    gatewayData["ratio_sta_lta"] = round(ratioStaLta * 100.0f) / 100.0f;
    gatewayData["roll"] = round(roll * 100.0f) / 100.0f;
    gatewayData["pitch"] = round(pitch * 100.0f) / 100.0f;
    gatewayData["gyro_x"] = round(gyroX * 100.0f) / 100.0f;
    gatewayData["gyro_y"] = round(gyroY * 100.0f) / 100.0f;
    gatewayData["gyro_z"] = round(gyroZ * 100.0f) / 100.0f;

    char payload[256];
    serializeJson(json, payload);
    
    Serial.println(F("----------------\r\nPublish:"));
    Serial.println(payload);
    mqttClient.publish(MQTT_TOPIC_GATEWAY_TELE_PUB, payload);

    lastPublishTime = currentTime;
  }

  if (mpuConnectionState != lastMpuConnectionState) {
    if (!mpuConnectionState) { // Hanya kalau false/disconnected
      Serial.println(F("Sensor MPU-6050 disconnected!"));
    } else {
      Serial.println(F("Sensor MPU-6050 reconnected!"));
    }

    lastMpuConnectionState = mpuConnectionState;
  }
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
  char encodedMsg[256];
  urlEncode(message, encodedMsg, sizeof(encodedMsg));

  char apiUrl[512];
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
  Serial.println(F("----------------\r\nReconnecting to MQTT Broker..."));
  if (mqttClient.connect("LandSlidEtect_Gateway", GATEWAY_ACCESS_TOKEN, "")) {
    Serial.println(F("Reonnected to MQTT Broker!"));
  } else {
    Serial.print(F("Failed with state "));
    Serial.print(mqttClient.state());
    Serial.println(F(" try again in 5 seconds!"));
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

    if (mpuConnectionState != lastMpuConnectionState) {
      if (!mpuConnectionState) {
        Serial.println(F("Sensor MPU-6050 disconnected!"));
      } else {
        Serial.println(F("Sensor MPU-6050 reconnected!"));
      }

      lastMpuConnectionState = mpuConnectionState;
    }
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

    if (mpuConnectionState != lastMpuConnectionState) {
      if (!mpuConnectionState) {
        Serial.println(F("Sensor MPU-6050 disconnected!"));
      } else {
        Serial.println(F("Sensor MPU-6050 reconnected!"));
      }

      lastMpuConnectionState = mpuConnectionState;
    }
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

// RTOS Core 0 untuk menangani pengiriman pesan ke telegram
void telegramTask(void *pvParameters) {
  const uint16_t TELEGRAM_COOLDOWN = 60000; // Jeda 1 menit
  uint32_t lastEarthquakeTelegramSentTime = 0, lastAngleAlertTelegramSentTime = 0, lastAngleDangerTelegramSentTime = 0, lastAngularVelocityTelegramSentTime = 0;

  for (;;) { // Looping abadi khusus Core 0
    uint32_t currentTaskTime = millis();

    if (earthquakeFlag) {
      // Cek Cooldown
      if (currentTaskTime - lastEarthquakeTelegramSentTime >= TELEGRAM_COOLDOWN || lastEarthquakeTelegramSentTime == 0) {
        Serial.println(F("----------------\r\n[Core 0] Waspada Gempa! Kirim Telegram..."));
        
        sendMessageToTelegram("https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n⚠️ Waspada!\nTerdeteksi getaran tanah yang berasal dari gempa, berpotensi longsor!");
        
        lastEarthquakeTelegramSentTime = currentTaskTime;
      }
      
      earthquakeFlag = false;
    }
    if (angleAlertFlag) {
      // Cek Cooldown
      if (currentTaskTime - lastAngleAlertTelegramSentTime >= TELEGRAM_COOLDOWN || lastAngleAlertTelegramSentTime == 0) {
        Serial.println(F("----------------\r\n[Core 0] Waspada Tanah Tambah Miring! Kirim Telegram..."));
        
        sendMessageToTelegram("https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n⚠️ Waspada!\nTerdeteksi perubahan sudut kemiringan tanah, berpotensi longsor!");
        
        lastAngleAlertTelegramSentTime = currentTaskTime;
      }
      
      angleAlertFlag = false;
    }
    if (angleDangerFlag) {
      // Cek Cooldown
      if (currentTaskTime - lastAngleDangerTelegramSentTime >= TELEGRAM_COOLDOWN || lastAngleDangerTelegramSentTime == 0) {
        Serial.println(F("----------------\r\n[Core 0] Bahaya Tanah Tambah Miring! Kirim Telegram..."));
        
        sendMessageToTelegram("https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n🚨 Bahaya!\nTerdeteksi perubahan sudut kemiringan tanah, kemungkinan besar longsor!");
        
        lastAngleDangerTelegramSentTime = currentTaskTime;
      }
      
      angleDangerFlag = false;
    }
    if (angularVelocityFlag) {
      // Cek Cooldown
      if (currentTaskTime - lastAngularVelocityTelegramSentTime >= TELEGRAM_COOLDOWN || lastAngularVelocityTelegramSentTime == 0) {
        Serial.println(F("----------------\r\n[Core 0] Bahaya Tanah Longsor! Kirim Telegram..."));
        
        sendMessageToTelegram("https://www.google.com/maps?q=3.603849926081428,98.66822999261507\n\n🚨 Bahaya!\nTerdeteksi alat pemantauan jatuh, indikasi terjadinya tanah longsor, periksa lereng segera!");
        
        lastAngularVelocityTelegramSentTime = currentTaskTime;
      }
      
      angularVelocityFlag = false;
    }

    // Istirahatkan Core 0 selama 100ms agar tidak panas (Watchdog friendly)
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}