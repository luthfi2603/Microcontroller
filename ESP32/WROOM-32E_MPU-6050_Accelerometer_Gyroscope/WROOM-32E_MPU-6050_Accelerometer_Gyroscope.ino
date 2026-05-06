/**
 * Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial by Dejan, https://howtomechatronics.com
 */
#include <Wire.h>

constexpr const uint8_t MPU_PIN = 0x68; // MPU-6050 I2C address
constexpr const float RAD_TO_DEG_F = 57.29577951f; // 180 / PI(3.14)
constexpr const float ACCEL_SCALE = 1.0f / 16384.0f;
constexpr const float GYRO_SCALE  = 1.0f / 131.0f;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY;
float roll = 0, pitch = 0, yaw = 0;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime;
uint32_t currentTime, previousTime, pollingTime = 0;
bool mpuConnectionState, lastMpuConnectionState = true;

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
  calculate_IMU_error();
  delay(20);

  currentTime = millis();
}

void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU_PIN);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_PIN, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
    // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // X-axis value
    AccY = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // Y-axis value
    AccZ = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE; // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan2f(AccY, sqrtf((AccX * AccX) + (AccZ * AccZ))) * RAD_TO_DEG_F) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error() custom function for more details
    accAngleY = (atan2f(-AccX, sqrtf((AccY * AccY) + (AccZ * AccZ))) * RAD_TO_DEG_F) - AccErrorY; // AccErrorY ~(-1.58)

    mpuConnectionState = true;
  } else {
    mpuConnectionState = false;
  }

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) * 0.001f; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU_PIN);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_PIN, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
    GyroX = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    GyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    // Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
    GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
    GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
    // Complementary filter - combine acceleromter and gyro angle values
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    roll  = 0.96f * (roll  + (GyroX * elapsedTime)) + 0.04f * accAngleX; // deg/s * s = deg
    pitch = 0.96f * (pitch + (GyroY * elapsedTime)) + 0.04f * accAngleY;
    yaw =  yaw + GyroZ * elapsedTime;

    mpuConnectionState = true;
  } else {
    mpuConnectionState = false;
  }

  // Print the values on the serial monitor
  if (currentTime - pollingTime >= 1000) {
    pollingTime = currentTime;

    Serial.print(roll);
    Serial.print(F("/"));
    Serial.print(pitch);
    Serial.print(F("/"));
    Serial.println(yaw);
  }

  if (mpuConnectionState != lastMpuConnectionState) {
    if (!mpuConnectionState) { // Hanya kalau false/disconnected
      Serial.println(F("Sensor MPU-6050 disconnected!"));
    }

    lastMpuConnectionState = mpuConnectionState;
  }
}

void calculate_IMU_error() {
  uint8_t c = 0;
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_PIN);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_PIN, 6, true) == 6) {
      AccX = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE;
      AccY = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE;
      AccZ = (int16_t)(Wire.read() << 8 | Wire.read()) * ACCEL_SCALE;
      // Sum all readings
      AccErrorX = AccErrorX + (atan2f(AccY, sqrtf((AccX * AccX) + (AccZ * AccZ))) * RAD_TO_DEG_F);
      AccErrorY = AccErrorY + (atan2f(-AccX, sqrtf((AccY * AccY) + (AccZ * AccZ))) * RAD_TO_DEG_F);

      c++;
      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;

      delay(10);
    }

    if (mpuConnectionState != lastMpuConnectionState) {
      if (!mpuConnectionState) {
        Serial.println(F("Sensor MPU-6050 disconnected!"));
      }

      lastMpuConnectionState = mpuConnectionState;
    }
  }

  // Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX * 0.005f;
  AccErrorY = AccErrorY * 0.005f;
  c = 0;
  lastMpuConnectionState = true;

  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_PIN);
    Wire.write(0x43);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_PIN, 6, true) == 6) {
      GyroX = (int16_t)(Wire.read() << 8 | Wire.read());
      GyroY = (int16_t)(Wire.read() << 8 | Wire.read());
      GyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
      // Sum all readings
      GyroErrorX = GyroErrorX + (GyroX * GYRO_SCALE);
      GyroErrorY = GyroErrorY + (GyroY * GYRO_SCALE);
      GyroErrorZ = GyroErrorZ + (GyroZ * GYRO_SCALE);

      c++;
      mpuConnectionState = true;
    } else {
      mpuConnectionState = false;

      delay(10);
    }

    if (mpuConnectionState != lastMpuConnectionState) {
      if (!mpuConnectionState) {
        Serial.println(F("Sensor MPU-6050 disconnected!"));
      }

      lastMpuConnectionState = mpuConnectionState;
    }
  }

  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX * 0.005f;
  GyroErrorY = GyroErrorY * 0.005f;
  GyroErrorZ = GyroErrorZ * 0.005f;
  lastMpuConnectionState = true;

  // Print the error values on the Serial Monitor
  Serial.print(F("----------------\r\nAccErrorX: "));
  Serial.println(AccErrorX);
  Serial.print(F("AccErrorY: "));
  Serial.println(AccErrorY);
  Serial.print(F("GyroErrorX: "));
  Serial.println(GyroErrorX);
  Serial.print(F("GyroErrorY: "));
  Serial.println(GyroErrorY);
  Serial.print(F("GyroErrorZ: "));
  Serial.println(GyroErrorZ);
  Serial.println(F("----------------"));
}