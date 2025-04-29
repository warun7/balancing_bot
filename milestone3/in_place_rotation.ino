#include <Arduino.h>
// TURN X RIGHT/LEFT -> TURN 120 LEFT OR TURN 3598 RIGHT
// Basic pin setup for motors
int leftDirPin = 2;
int leftStepPin = 3;
int rightDirPin = 4;
int rightStepPin = 5;
int stepsPerRev = 6400;

void setup() {
  pinMode(leftDirPin, OUTPUT);
  pinMode(leftStepPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Type: TURN 90 RIGHT or TURN 90 LEFT");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    
    if (input.startsWith("TURN")) {
      int firstSpace = input.indexOf(' ');
      int lastSpace = input.lastIndexOf(' ');
      
      if (firstSpace != -1 && lastSpace != -1) {
        float degrees = input.substring(firstSpace + 1, lastSpace).toFloat();
        String direction = input.substring(lastSpace + 1);
        
        if (degrees > 0 && degrees <= 3600) {
          if (direction == "LEFT" || direction == "RIGHT") {
            turnRobot(degrees, direction);
            Serial.println("Turn complete!");
          } else {
            Serial.println("Please type LEFT or RIGHT only");
          }
        } else {
          Serial.println("Please use degrees between 0.1 and 3600");
        }
      }
    } else {
      Serial.println("Start with the word TURN");
    }
  }
}

// Makes the robot turn left or right by moving both motors
void turnRobot(float degrees, String direction) {
  int steps = degrees * stepsPerRev / 360;
  
  digitalWrite(leftDirPin, (direction == "RIGHT") ? LOW : HIGH);
  digitalWrite(rightDirPin, (direction == "RIGHT") ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(leftStepPin, HIGH);
    digitalWrite(rightStepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(leftStepPin, LOW);
    digitalWrite(rightStepPin, LOW);
    delayMicroseconds(500);
  }
}
