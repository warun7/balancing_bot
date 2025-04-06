#include <AccelStepper.h>

// Motor interface type (1 = external driver)
#define motorInterfaceType 1

// Pins
const int dirPin = 2;
const int stepPin = 3;

// Create motor instance
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// Microstepping setup (32x)
const int microsteps = 32;
const int stepsPerRev = 200 * microsteps; // 6400 steps/rev

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial connection
  
  // Motor parameters
  stepper.setMaxSpeed(1000);     // Steps per second
  stepper.setAcceleration(800);  // Steps per second²
  
  printInstructions();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processInput(input);
  }
}

void processInput(String input) {
  input.trim();
  
  // Parse angle and direction
  int spaceIdx = input.indexOf(' ');
  if (spaceIdx == -1) {
    Serial.println("ERROR: Use 'ANGLE DIRECTION' (e.g., 90 CW)");
    return;
  }
  
  float angle = input.substring(0, spaceIdx).toFloat();
  String dir = input.substring(spaceIdx + 1);
  dir.toUpperCase();

  // Validation
  if (angle <= 0 || angle > 3600) {
    Serial.println("ERROR: Angle must be 0.1° to 3600°");
    return;
  }
  
  if (dir != "CW" && dir != "CCW") {
    Serial.println("ERROR: Direction must be CW or CCW");
    return;
  }

  // Convert angle to steps
  long targetSteps = (angle / 360.0) * stepsPerRev;
  if (dir == "CCW") targetSteps *= -1;
  
  // Move motor
  stepper.move(targetSteps);
  Serial.print("Moving ");
  Serial.print(angle);
  Serial.print("° ");
  Serial.println(dir);
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    printProgress(stepper);
  }
  
  Serial.println("Done!\n");
}

void printProgress(AccelStepper &motor) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { // Update every 200ms
    float progress = 100 * (1 - (abs(motor.distanceToGo()) / (float)abs(motor.targetPosition())));
    Serial.print("Progress: ");
    Serial.print(progress, 1);
    Serial.println("%");
    lastPrint = millis();
  }
}

void printInstructions() {
  Serial.println("\n==== ACCELSTEPPER CONTROLLER ====");
  Serial.println("Format: ANGLE DIRECTION");
  Serial.println("Examples:");
  Serial.println("  90 CW    - 90° clockwise");
  Serial.println("  180 CCW  - Half turn counter-clockwise");
  Serial.println("  720 CW   - 2 full rotations");
  Serial.println("Settings:");
  Serial.print("  Microstepping: 1/"); Serial.println(microsteps);
  Serial.print("  Max Speed: "); Serial.print(stepper.maxSpeed()*60/stepsPerRev); Serial.println(" RPM");
  Serial.print("  Acceleration: "); Serial.print(stepper.acceleration()*3600/stepsPerRev); Serial.println(" °/s²");
  Serial.println("==============================\n");
}