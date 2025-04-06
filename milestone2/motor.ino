#define directionPin 2
#define stepPin 3
#define stepsPerRevolution 6400  // 32x microstepping (200 steps * 32)

void setup() {
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial); // Wait for serial connection
  printInstructions();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Parse input
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("ERROR: Invalid format. Use: ANGLE DIRECTION (e.g., 90 CW)");
      return;
    }
    
    float angle = input.substring(0, spaceIndex).toFloat();
    String dir = input.substring(spaceIndex + 1);
    dir.toUpperCase();
    
    // Validate input
    if (angle <= 0 || angle > 3600) {  // Limit to 10 full rotations max
      Serial.println("ERROR: Angle must be between 0.1° and 3600°");
      return;
    }
    
    if (dir != "CW" && dir != "CCW") {
      Serial.println("ERROR: Direction must be CW or CCW");
      return;
    }
    
    // Execute rotation
    rotateMotor(angle, dir);
    Serial.println("Ready for next command...");
  }
}

void rotateMotor(float degrees, String direction) {
  int steps = (degrees / 360.0) * stepsPerRevolution;
  digitalWrite(directionPin, direction == "CW" ? HIGH : LOW);
  
  Serial.print("Rotating ");
  Serial.print(degrees);
  Serial.print("° ");
  Serial.println(direction);
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
    
    // Print progress every 10%
    if (i % (steps/10 + 1) == 0) {  // +1 to avoid division by zero
      Serial.print("Progress: ");
      Serial.print((i * 100) / steps);
      Serial.println("%");
    }
  }
  Serial.println("Done!");
}

void printInstructions() {
  Serial.println("\n=== STEPPER MOTOR CONTROLLER ===");
  Serial.println("Enter commands in the format: ANGLE DIRECTION");
  Serial.println("Examples:");
  Serial.println("  90 CW    - Rotate 90° clockwise");
  Serial.println("  45 CCW   - Rotate 45° counter-clockwise");
  Serial.println("  180 CW   - Rotate half a turn");
  Serial.println("  3600 CCW - Rotate 10 full turns counter-clockwise");
  Serial.println("Note: Max 3600° (10 rotations) per command");
  Serial.println("===============================\n");
}