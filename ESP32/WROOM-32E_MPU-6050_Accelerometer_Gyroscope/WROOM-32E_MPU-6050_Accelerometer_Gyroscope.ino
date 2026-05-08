/**
 * Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial by Dejan, https://howtomechatronics.com
 */
#include <Wire.h>

constexpr const uint8_t MPU_PIN = 0x68; // MPU-6050 I2C address
constexpr const float RAD_TO_DEG_F = 57.29577951f; // 180 / PI(3.14)
constexpr const float ACCEL_SCALE = 1.0f / 16384.0f; // / 16384
constexpr const float GYRO_SCALE  = 1.0f / 131.0f; // / 131
constexpr const uint8_t STA_WINDOW = 100; // 1 detik (100 sampel)
constexpr const uint16_t LTA_WINDOW = 2000; // 20 detik (2000 sampel)

float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
bool mpuConnectionState, lastMpuConnectionState = true;
uint32_t gyroCurrentTime;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\r\nInitializing MPU-6050..."));
  Wire.begin();                                   // Initialize comunication
  Wire.beginTransmission(MPU_PIN);                // Start communication with MPU6050 // MPU_PIN=0x68
  Wire.write(0x6B);                               // Talk to the register 6B
  Wire.write(0x00);                               // Make reset - place a 0 into the 6B register
  uint8_t statusI2C = Wire.endTransmission(true); // End the transmission
  if (statusI2C != 0) {
    Serial.print(F("MPU-6050 init failed because of I2C problem!, Code: "));
    Serial.println(statusI2C);
    while (1) { delay(1000); } // Hentikan program tanpa membuat Watchdog Crash
  }
  Wire.beginTransmission(MPU_PIN);
  Wire.write(0x75); // Tanya KTP (Register WHO_AM_I)
  Wire.endTransmission(false);
  
  if (Wire.requestFrom(MPU_PIN, 1, true) == 1) {
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
  /* // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_PIN);
  Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU_PIN);
  Wire.write(0x1B);                  // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                  // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20); */
  // Call this function if you need to get the IMU error values for your module
  calculateIMUError();
  delay(20);
  gyroCurrentTime = millis();
}

void loop() {
  static uint32_t previousTime, pollingTime = 0, lastSampleTime = 0;
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

  if (currentTime - lastSampleTime >= 10) {
    lastSampleTime = currentTime;

    // === Read acceleromter data === //
    Wire.beginTransmission(MPU_PIN);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_PIN, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
      // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
      float accX = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // X-axis value
      float accY = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // Y-axis value
      float accZ = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // Z-axis value
      // Calculating Roll and Pitch from the accelerometer data
      accAngleX = (atan2f(accY, sqrtf((accX * accX) + (accZ * accZ))) * RAD_TO_DEG_F) - accErrorX; // accErrorX ~(0.58) See the calculateIMUError() custom function for more details
      accAngleY = (atan2f(-accX, sqrtf((accY * accY) + (accZ * accZ))) * RAD_TO_DEG_F) - accErrorY; // accErrorY ~(-1.58)

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

      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;
    }

    // === Read gyroscope data === //
    previousTime = gyroCurrentTime;        // Previous time is stored before the actual time read
    gyroCurrentTime = millis();            // Current time actual time read
    float elapsedTime = (gyroCurrentTime - previousTime) * 0.001f; // Divide by 1000 to get seconds
    Wire.beginTransmission(MPU_PIN);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_PIN, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
      gyroX = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
      gyroY = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
      gyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
      // Correct the outputs with the calculated error values
      gyroX = gyroX - gyroErrorX; // gyroErrorX ~(-0.56)
      gyroY = gyroY - gyroErrorY; // gyroErrorY ~(2)
      gyroZ = gyroZ - gyroErrorZ; // gyroErrorZ ~ (-0.8)
      // Complementary filter - combine acceleromter and gyro angle values
      // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
      roll = 0.96f * (roll + (gyroX * elapsedTime)) + 0.04f * accAngleX; // deg/s * s = deg
      pitch = 0.96f * (pitch + (gyroY * elapsedTime)) + 0.04f * accAngleY;
      yaw = yaw + gyroZ * elapsedTime;

      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;
    }
  }

  // Print the values on the serial monitor
  if (currentTime - pollingTime >= 1000) {
    pollingTime = currentTime;

    Serial.print(F("{\r\n  \"vibration\": {\r\n    \"acc_dyn\": "));
    Serial.print(accDyn);
    Serial.print(F(",\r\n    \"ratio_sta_lta\": "));
    Serial.print(ratioStaLta);
    Serial.print(F("\r\n  },\r\n  \"angle\": {\r\n    \"roll\": "));
    Serial.print(roll);
    Serial.print(F(",\r\n    \"pitch\": "));
    Serial.print(pitch);
    Serial.print(F(",\r\n    \"yaw\": "));
    Serial.print(yaw);
    Serial.print(F("\r\n  },\r\n  \"angular_velocity\": {\r\n    \"gyro_x\": "));
    Serial.print(gyroX);
    Serial.print(F(",\r\n    \"gyro_y\": "));
    Serial.print(gyroY);
    Serial.print(F(",\r\n    \"gyro_z\": "));
    Serial.print(gyroZ);
    Serial.print(F("\r\n  }\r\n}\r\n"));
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

void calculateIMUError() {
  uint8_t c = 0;
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_PIN);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_PIN, 6, true) == 6) {
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
    Wire.beginTransmission(MPU_PIN);
    Wire.write(0x43);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_PIN, 6, true) == 6) {
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
  Serial.print(F("accErrorY: "));
  Serial.println(accErrorY);
  Serial.print(F("gyroErrorX: "));
  Serial.println(gyroErrorX);
  Serial.print(F("gyroErrorY: "));
  Serial.println(gyroErrorY);
  Serial.print(F("gyroErrorZ: "));
  Serial.println(gyroErrorZ);
  Serial.println(F("----------------"));
}