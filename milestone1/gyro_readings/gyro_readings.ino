#include <Wire.h>
const int MPU6050_ADDR = 0x68;
float gyroX, gyroY, gyroZ;
float roll = 0, pitch = 0;
unsigned long prevTime;
const float GYRO_SENSITIVITY = 131.0; // 131 LSB/(°/s) for ±250°/s range

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // Power management register
    Wire.write(0);     // Wake up MPU6050
    Wire.endTransmission(true);
    
    prevTime = millis(); // Initialize time tracking
}

void loop() {
    readGyroscope();
    calculateAngles();
    printAngles();
    delay(100);  // Adjust delay as needed
}

void readGyroscope() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // Starting register for gyroscope
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    // Read raw gyroscope data
    gyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
    gyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
    gyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
}

void calculateAngles() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;
    
    // Integrate gyroscope data to estimate angles
    roll += gyroX * dt;
    pitch += gyroY * dt;
}

void printAngles() {
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("° | Pitch: ");
    Serial.print(pitch);
    Serial.println("°");
}
