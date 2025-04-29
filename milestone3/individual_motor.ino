const int leftDirPin = 2;
const int leftStepPin = 3;
const int rightDirPin = 4;
const int rightStepPin = 5;
const int stepsPerRev = 6400;

void setup() {
  pinMode(leftDirPin, OUTPUT);
  pinMode(leftStepPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Easy Motor Control Ready!");
  Serial.println("Enter commands as: (left,right)");
  Serial.println("Examples:");
  Serial.println("(90,-90) - Turn right");
  Serial.println("(180,180) - Move forward");
  Serial.println("(-45,-45) - Move backward");
}

void loop() {
  if (Serial.available()) {
    while (Serial.read() != '(') {}
    
    int leftDeg = Serial.parseInt();
    while (Serial.read() != ',') {}
    int rightDeg = Serial.parseInt();
    
    moveMotor(leftStepPin, leftDirPin, leftDeg);
    moveMotor(rightStepPin, rightDirPin, rightDeg);
    
    Serial.print("Moved Left:");
    Serial.print(leftDeg);
    Serial.print(" Right:");
    Serial.println(rightDeg);
  }
}

void moveMotor(int stepPin, int dirPin, int degrees) {
  digitalWrite(dirPin, degrees < 0 ? HIGH : LOW);
  int steps = abs(degrees) * stepsPerRev / 360;
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}
