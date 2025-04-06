#include <Wire.h>

// MPU6050 configuration
const int MPU6050_ADDR = 0x68;
const float GYRO_SENSITIVITY = 131.0; // 131 LSB/(°/s) for ±250°/s range
const float ACCEL_SENSITIVITY = 16384.0; // 16384 LSB/g for ±2g range
const float ALPHA = 0.98; // Complementary filter coefficient

// Sensor variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float gyroX_offset = 0, gyroY_offset = 0;

// Angle measurements for each method
float accel_roll = 0, accel_pitch = 0;
float gyro_roll = 0, gyro_pitch = 0;
float comp_roll = 0, comp_pitch = 0;

// Timing variables
unsigned long currentTime;
unsigned long prevTime;
const unsigned long LOOP_TIME = 10; // 10ms fixed loop time

// Test variables
bool testStarted = false;
unsigned long testStartTime = 0;
const unsigned long TEST_DURATION = 5000; // 5 seconds in milliseconds

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up MPU6050
  Wire.endTransmission(true);
  
  // Configure gyroscope range (±250°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  // Configure accelerometer range (±2g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  // Calibrate gyroscope
  calibrateGyro();
  
  // Initialize timing
  prevTime = millis();
  
  // Print CSV header
  Serial.println("Time,Method,Roll,Pitch");
  
  // Wait for serial input to start test
  Serial.println("Send any character to start the step response test");
  while (!Serial.available()) {
    // Wait for input
  }
  
  // Clear input buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  Serial.println("Prepare for step response test in 3 seconds...");
  delay(3000);
  
  // Start test
  testStarted = true;
  testStartTime = millis();
}

void loop() {
  // Record start time of this loop iteration
  unsigned long loopStartTime = millis();
  
  // Read sensor data
  readMPU6050();
  
  // Calculate angles using all three methods
  calculateAngles();
  
  // If test is running, print data
  if (testStarted) {
    unsigned long elapsedTime = millis() - testStartTime;
    
    if (elapsedTime <= TEST_DURATION) {
      // Print data in CSV format for easier plotting
      // Format: Time(ms),Method,Roll,Pitch
      
      // Accelerometer data
      Serial.print(elapsedTime);
      Serial.print(",ACCEL,");
      Serial.print(accel_roll, 3);
      Serial.print(",");
      Serial.println(accel_pitch, 3);
      
      // Gyroscope data
      Serial.print(elapsedTime);
      Serial.print(",GYRO,");
      Serial.print(gyro_roll, 3);
      Serial.print(",");
      Serial.println(gyro_pitch, 3);
      
      // Complementary filter data
      Serial.print(elapsedTime);
      Serial.print(",COMP,");
      Serial.print(comp_roll, 3);
      Serial.print(",");
      Serial.println(comp_pitch, 3);
    } else if (elapsedTime > TEST_DURATION && elapsedTime < TEST_DURATION + 1000) {
      // End test after duration
      Serial.println("Test complete!");
      testStarted = false;
    }
  }
  
  // Enforce fixed loop time as mentioned in the hint
  while (millis() - loopStartTime < LOOP_TIME) {
    // Wait until loop time is reached
  }
}

void calibrateGyro() {
  Serial.println("Calibrating gyroscope...");
  int numReadings = 500;
  float sumX = 0, sumY = 0;
  
  for (int i = 0; i < numReadings; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    sumX += (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
    sumY += (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
    delay(2);
  }
  
  gyroX_offset = sumX / numReadings;
  gyroY_offset = sumY / numReadings;
  
  Serial.println("Calibration complete!");
  Serial.print("Gyro X offset: ");
  Serial.print(gyroX_offset);
  Serial.print(" | Gyro Y offset: ");
  Serial.println(gyroY_offset);
}

void readMPU6050() {
  // Read accelerometer data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  accelX = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
  accelY = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
  accelZ = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
  
  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);  // Starting register for gyroscope
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  gyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY - gyroX_offset;
  gyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY - gyroY_offset;
  gyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
}

void calculateAngles() {
  // Get time difference
  currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
  prevTime = currentTime;
  
  // 1. Calculate angles from accelerometer
  accel_roll = atan2(accelY, accelZ) * 180 / PI;
  accel_pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
  
  // 2. Calculate angles from gyroscope (integration)
  gyro_roll += gyroX * dt;
  gyro_pitch += gyroY * dt;
  
  // 3. Calculate angles using complementary filter
  comp_roll = ALPHA * (comp_roll + gyroX * dt) + (1 - ALPHA) * accel_roll;
  comp_pitch = ALPHA * (comp_pitch + gyroY * dt) + (1 - ALPHA) * accel_pitch;
}