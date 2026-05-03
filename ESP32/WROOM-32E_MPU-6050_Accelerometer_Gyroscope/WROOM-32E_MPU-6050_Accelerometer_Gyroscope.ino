/**
 * Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial by Dejan, https://howtomechatronics.com
 */
#include <Wire.h>

constexpr const uint8_t MPU_PIN = 0x68; // MPU-6050 I2C address
constexpr const float RAD_TO_DEG_F = 57.29577951f; // 180 / PI(3.14)

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime;
uint32_t currentTime, previousTime, pollingTime = 0;
uint8_t c = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_PIN);   // Start communication with MPU6050 // MPU_PIN=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // End the transmission
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
    AccX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f; // X-axis value
    AccY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f; // Y-axis value
    AccZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f; // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan2f(AccY, sqrtf((AccX * AccX) + (AccZ * AccZ))) * RAD_TO_DEG_F) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error() custom function for more details
    accAngleY = (atan2f(-AccX, sqrtf((AccY * AccY) + (AccZ * AccZ))) * RAD_TO_DEG_F) - AccErrorY; // AccErrorY ~(-1.58)
  } else {
    Serial.println(F("Sensor MPU-6050 disconnected!"));
  }

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000.0f; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU_PIN);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_PIN, 6, true) == 6) { // Read 6 registers total, each axis value is stored in 2 registers
    GyroX = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
    GyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
    // Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
    GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
    GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw =  yaw + GyroZ * elapsedTime;
    // Complementary filter - combine acceleromter and gyro angle values
    roll = 0.96f * gyroAngleX + 0.04f * accAngleX;
    pitch = 0.96f * gyroAngleY + 0.04f * accAngleY;
  } else {
    Serial.println(F("Sensor MPU-6050 disconnected!"));
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
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_PIN);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_PIN, 6, true) == 6) {
      AccX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
      AccY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
      AccZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
      // Sum all readings
      AccErrorX = AccErrorX + (atan2f(AccY, sqrtf((AccX * AccX) + (AccZ * AccZ))) * RAD_TO_DEG_F);
      AccErrorY = AccErrorY + (atan2f(-AccX, sqrtf((AccY * AccY) + (AccZ * AccZ))) * RAD_TO_DEG_F);

      c++;
    } else {
      Serial.println(F("Sensor MPU-6050 disconnected!"));

      delay(10);
    }
  }

  // Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200.0f;
  AccErrorY = AccErrorY / 200.0f;
  c = 0;

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
      GyroErrorX = GyroErrorX + (GyroX / 131.0f);
      GyroErrorY = GyroErrorY + (GyroY / 131.0f);
      GyroErrorZ = GyroErrorZ + (GyroZ / 131.0f);

      c++;
    } else {
      Serial.println(F("Sensor MPU-6050 disconnected!"));

      delay(10);
    }
  }

  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200.0f;
  GyroErrorY = GyroErrorY / 200.0f;
  GyroErrorZ = GyroErrorZ / 200.0f;

  // Print the error values on the Serial Monitor
  Serial.print(F("AccErrorX: "));
  Serial.println(AccErrorX);
  Serial.print(F("AccErrorY: "));
  Serial.println(AccErrorY);
  Serial.print(F("GyroErrorX: "));
  Serial.println(GyroErrorX);
  Serial.print(F("GyroErrorY: "));
  Serial.println(GyroErrorY);
  Serial.print(F("GyroErrorZ: "));
  Serial.println(GyroErrorZ);
}