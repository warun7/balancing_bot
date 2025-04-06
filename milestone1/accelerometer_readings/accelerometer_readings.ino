#include <Wire.h>
const int MPU6050_ADDR = 0x68;
float accX, accY, accZ;
float roll, pitch;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // Power management register
    Wire.write(0);     // Wake up MPU6050
    Wire.endTransmission(true);
}

void loop() {
    readAccelerometer();
    calculateAngles();
    printAngles();
    delay(500);  // Adjust delay as needed
}

void readAccelerometer() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // Starting register for accelerometer
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    // Read raw accelerometer data
    accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void calculateAngles() {
    roll = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
    pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;
}

void printAngles() {
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("° | Pitch: ");
    Serial.print(pitch);
    Serial.println("°");
}
