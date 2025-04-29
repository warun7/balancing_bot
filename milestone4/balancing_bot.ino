#include <AccelStepper.h>
#include <Wire.h>

// PID variables
double setpoint = 3;    // Target pitch (desired angle)
double input, output;
double Kp = 50.0, Ki = 0.2, Kd =  1;// PID constants
double lastInput = 0;   // Previous input for derivative calculation
double integral = 0;    // Integral sum

// Time-related variables
unsigned long lastTime = 0;
unsigned long sampleTime = 10; // PID loop sample time in milliseconds

const int MPU6050_ADDR = 0x68;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll = 0, pitch = 0;
unsigned long prevTime;
const float GYRO_SENSITIVITY = 131.0; // 131 LSB/(°/s) for ±250°/s range
const float ALPHA = 0.98; // Complementary filter coefficient
float gyroX_offset = 0, gyroY_offset = 0; // Gyro drift offsets

#define dirPin1 6
#define stepPin1 7
#define dirPin2 8
#define stepPin2 9

#define motorInterfaceType 1

#define STEPS_PER_REVOLUTION 800
#define ANGLE 360

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

float speed = 18000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up MPU6050
  Wire.endTransmission(true);
  
  calibrateGyro(); // Measure gyro drift
  prevTime = micros(); // Initialize time tracking with micros

  stepper.setMaxSpeed(30000);
  stepper2.setMaxSpeed(30000);
}

void loop() {
  readMPU6050();
  calculateAngles();
  printAngles();  

  // Use the pitch as the input to the PID controller
  input = pitch;

  // PID control loop
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= sampleTime) {
    // Calculate PID
    double error = setpoint - input;
    integral += error * (currentTime - lastTime) / 1000.0; // sum of errors (integral term)
    double derivative = (input - lastInput) / (currentTime - lastTime); // rate of change (derivative term)
    
    // PID output calculation
    output = Kp * error + Ki * integral + Kd * derivative;

    // Store current input for next derivative calculation
    lastInput = input;

    // Update lastTime for next sample
    lastTime = currentTime;
  }

  // Dynamically adjust speed based on pitch value
 if (pitch < 0) {
  // Forward motion (negative pitch), speed range capped at -30000
  stepper.setSpeed(map(pitch, -90, 0, -15000, -30000) + output);
  stepper2.setSpeed(-(map(pitch, -90, 0, -15000, -30000) - output));
} else {
  // Backward motion (positive pitch), speed range capped at 30000
  stepper.setSpeed(map(pitch, 0, 90, 15000, 30000) + output);
  stepper2.setSpeed(-(map(pitch, 0, 90, 15000, 30000) - output));
}


  // Run the motors
  stepper.runSpeed();
  stepper2.runSpeed();
}

void calibrateGyro() {
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
}

void readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    // Read raw accelerometer data
    accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    // Read raw gyroscope data and apply offset correction
    gyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY - gyroX_offset;
    gyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY - gyroY_offset;
    gyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
}

void calculateAngles() {
    unsigned long currentTime = micros();
    float dt = (currentTime - prevTime) / 1000000.0; // Convert to seconds
    prevTime = currentTime;
    
    // Calculate roll and pitch from accelerometer
    float accelRoll = atan2(accelY, accelZ) * 180 / PI;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
    
    // Integrate gyroscope data
    roll = ALPHA * (roll + gyroX * dt) + (1 - ALPHA) * accelRoll;
    pitch = ALPHA * (pitch + gyroY * dt) + (1 - ALPHA) * accelPitch;
}

void printAngles() {
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("° | Pitch: ");
    Serial.print(pitch);
    Serial.println("°");
}

