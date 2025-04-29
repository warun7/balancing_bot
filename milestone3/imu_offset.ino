#include <Wire.h>
const int MPU6050_ADDR = 0x68;

// IMU Variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll = 0, pitch = 0;
unsigned long prevTime;
const float GYRO_SENSITIVITY = 131.0;
const float ALPHA = 0.98;
float gyroX_offset = 0, gyroY_offset = 0;
float pitch_offset = 0;
bool calibrationDone = false;
const int CALIBRATION_SAMPLES = 10000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  calibrateGyro();
  measurePitchOffset();
  prevTime = micros();
}

void loop() {
  if (!calibrationDone) return;
  
  readMPU6050();
  calculateAngles();
  printCalibratedAngles();
  delay(10);
}

void measurePitchOffset() {
  Serial.println("=== IMU CALIBRATION ===");
  Serial.println("Keep robot PERFECTLY STILL and UPRIGHT");
  Serial.print("Taking ");
  Serial.print(CALIBRATION_SAMPLES);
  Serial.println(" samples...\n");
  
  unsigned long startTime = millis();
  float sum = 0;
  float min_pitch = 1000, max_pitch = -1000; // Track min/max
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readMPU6050();
    calculateAngles();
    sum += pitch;
    
    // Update min/max
    if (pitch < min_pitch) min_pitch = pitch;
    if (pitch > max_pitch) max_pitch = pitch;
    
    // Display progress every 1000 samples
    if (i % 1000 == 0) {
      float current_offset = sum / (i + 1);
      Serial.print("Sample ");
      Serial.print(i);
      Serial.print("/");
      Serial.print(CALIBRATION_SAMPLES);
      Serial.print(" | Current offset: ");
      Serial.print(current_offset, 4);
      Serial.print("° | Range: [");
      Serial.print(min_pitch, 2);
      Serial.print("° to ");
      Serial.print(max_pitch, 2);
      Serial.println("°]");
    }
    delay(1);
  }
  
  pitch_offset = sum / CALIBRATION_SAMPLES;
  unsigned long duration = (millis() - startTime) / 1000;
  
  Serial.println("\n=== CALIBRATION REPORT ===");
  Serial.print("Samples: ");
  Serial.println(CALIBRATION_SAMPLES);
  Serial.print("Duration: ");
  Serial.print(duration);
  Serial.println(" seconds");
  Serial.print("Final Pitch Offset: ");
  Serial.print(pitch_offset, 4);
  Serial.println("°");
  Serial.print("Pitch Range Observed: [");
  Serial.print(min_pitch, 2);
  Serial.print("° to ");
  Serial.print(max_pitch, 2);
  Serial.println("°]");
  Serial.println("\nLive data starting now...");
  calibrationDone = true;
}

void calibrateGyro() {
  const int GYRO_SAMPLES = 500;
  Serial.println("Calibrating gyro...");
  
  float sumX = 0, sumY = 0;
  for (int i = 0; i < GYRO_SAMPLES; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // GYRO_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 4, true); // Read X and Y
    
    sumX += (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
    sumY += (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
    delay(2);
  }
  
  gyroX_offset = sumX / GYRO_SAMPLES;
  gyroY_offset = sumY / GYRO_SAMPLES;
  Serial.println("Gyro calibration complete");
}

void readMPU6050() {
  // Read accelerometer (6 bytes)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  accelX = (Wire.read() << 8 | Wire.read()) / 16384.0; // ±2g range
  accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  // Read gyroscope (6 bytes)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  gyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY - gyroX_offset;
  gyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY - gyroY_offset;
  gyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
}

void calculateAngles() {
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;
  
  // Accelerometer angles
  float accelRoll = atan2(accelY, accelZ) * 180 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180 / PI;
  
  // Complementary filter
  roll = ALPHA * (roll + gyroX * dt) + (1 - ALPHA) * accelRoll;
  pitch = ALPHA * (pitch + gyroY * dt) + (1 - ALPHA) * accelPitch;
}

void printCalibratedAngles() {
  Serial.print("Roll: ");
  Serial.print(roll, 2); // 2 decimal places
  Serial.print("° | Pitch: ");
  Serial.print(pitch - pitch_offset, 2);
  Serial.println("°");
}
