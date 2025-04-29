#include <Wire.h>

const int leftDirPin = 2;
const int leftStepPin = 3;
const int rightDirPin = 4;
const int rightStepPin = 5;
const int stepsPerRev = 6400; // 200 steps/rev * 32 microsteps

const int MPU_ADDR = 0x68;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float pitch = 0, roll = 0;
unsigned long prevTime;
const float GYRO_SENSITIVITY = 131.0; // ±250°/s range
const float ALPHA = 0.98; // Complementary filter coefficient

const bool LEFT_FORWARD = HIGH;  
const bool RIGHT_FORWARD = LOW;  

void setup() {
  // Motor Setup
  pinMode(leftDirPin, OUTPUT);
  pinMode(leftStepPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  
  // IMU Setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up IMU
  Wire.endTransmission(true);
  
  Serial.begin(115200);
  Serial.println("System Ready");
  Serial.println("Enter commands as: <degrees> <direction>");
  Serial.println("Example: '57 1' = turn 57° right");
  Serial.println("          '159 2' = turn 159° left");
  prevTime = micros();
}

void loop() {
  if (Serial.available()) {
    float degrees = Serial.parseFloat();
    int direction = Serial.parseInt();
    
    if (degrees > 0 && (direction == 1 || direction == 2)) {
      turnRobot(degrees, direction);
    } else {
      Serial.println("Invalid input! Use: <degrees> <1 or 2>");
    }
  }
  
  readIMU();
  calculateAngles();
  
  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("°\tRoll: ");
  Serial.print(roll, 1);
  Serial.println("°");
  
  delay(100); // Reduce serial spam
}

void turnRobot(float degrees, int turnDirection) {
  
  int steps = degrees * stepsPerRev / 360;
  

  if (turnDirection == 1) { // Right turn
    digitalWrite(leftDirPin, LEFT_FORWARD);    
    digitalWrite(rightDirPin, !RIGHT_FORWARD); 
  } else { // Left turn
    digitalWrite(leftDirPin, !LEFT_FORWARD);   // Left backward
    digitalWrite(rightDirPin, RIGHT_FORWARD);  // Right forward
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(leftStepPin, HIGH);
    digitalWrite(rightStepPin, HIGH);
    delayMicroseconds(500); // Controls speed
    digitalWrite(leftStepPin, LOW);
    digitalWrite(rightStepPin, LOW);
    delayMicroseconds(500);
  }
  
  Serial.print("Turned ");
  Serial.print(degrees);
  Serial.print("° ");
  Serial.println(turnDirection == 1 ? "right" : "left");
}

void readIMU() {
  // Read accelerometer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  
  accelX = (Wire.read()<<8|Wire.read()) / 16384.0;
  accelY = (Wire.read()<<8|Wire.read()) / 16384.0;
  accelZ = (Wire.read()<<8|Wire.read()) / 16384.0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  
  gyroX = (Wire.read()<<8|Wire.read()) / GYRO_SENSITIVITY;
  gyroY = (Wire.read()<<8|Wire.read()) / GYRO_SENSITIVITY;
  gyroZ = (Wire.read()<<8|Wire.read()) / GYRO_SENSITIVITY;
}

void calculateAngles() {
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;
  
  
  float accelRoll = atan2(accelY, accelZ) * 180 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180 / PI;
  
  // Complementary filter
  roll = ALPHA * (roll + gyroX * dt) + (1 - ALPHA) * accelRoll;
  pitch = ALPHA * (pitch + gyroY * dt) + (1 - ALPHA) * accelPitch;
}
