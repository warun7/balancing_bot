// Pin definitions
const int dirPin = 2;    // Direction pin
const int stepPin = 3;   // Step pin
const int stepsPerRev = 6400; // 200 steps * 32 microsteps

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Enter angle and direction (e.g., '90 CW'):");
}

void loop() {
  if (Serial.available()) {
    // Read input
    float angle = Serial.parseFloat();
    String dir = Serial.readStringUntil('\n');
    dir.trim();
    dir.toUpperCase();

    // Validate input
    if (angle <= 0 || angle > 3600) {
      Serial.println("Error: Angle must be 0.1° to 3600°");
      return;
    }

    if (dir != "CW" && dir != "CCW") {
      Serial.println("Error: Use 'CW' or 'CCW' for direction");
      return;
    }

    // Move motor
    moveStepper(angle, dir);
    Serial.println("Ready for next command");
  }
}

void moveStepper(float degrees, String direction) {
  int steps = degrees * stepsPerRev / 360;
  digitalWrite(dirPin, direction == "CW" ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);

  }
  Serial.println("Done");
}
