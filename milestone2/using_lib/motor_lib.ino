#include <AccelStepper.h>

const int stepPin = 3;
const int dirPin = 2;

// Motor setup (200 steps/rev × 32 microsteps = 6400 steps/rev)
const int stepsPerRevolution = 6400; 
AccelStepper stepper(1, stepPin, dirPin); 

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(1000);     // 1000 steps/sec (~9.4 RPM)
  stepper.setAcceleration(800);  // 800 steps/sec^2
  Serial.println("Enter degrees and direction (e.g., '90 CW'):");
}

void loop() {
  if (Serial.available()) {
    float degrees = Serial.parseFloat();
    String dir = Serial.readStringUntil('\n');
    dir.trim();
    dir.toUpperCase();

    // Validate input
    if (degrees <= 0 || degrees > 3600) {
      Serial.println("Error: Enter 0.1° to 3600°");
      return;
    }
    if (dir != "CW" && dir != "CCW") {
      Serial.println("Error: Use CW or CCW");
      return;
    }

    long target = degrees * stepsPerRevolution / 360;
    if (dir == "CCW") target = -target;
    
    stepper.move(target); 
    
    while (stepper.run()) { 
    }
    
    Serial.println("Done!");
  }
}
