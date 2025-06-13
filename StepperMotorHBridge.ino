// ----------------------------------------------------------------
// Objective:
// Control a bipolar stepper motor using two full H-bridge channels
// ----------------------------------------------------------------

// Typical pin definitions for a TB6612FNG or L298N dual H-bridge
#define AIN1 4
#define AIN2 5
#define PWMA 9
#define BIN1 6
#define BIN2 7
#define PWMB 10
#define STBY 8

// Global variables
double travelDistance = 4; // [cm]
double gearDiameter = 3; // [cm]
int gearBoxRatio = 19;
int stepRev = 60; // Guessed on this one, PLZ check soon

void setup() {
  // Set up important pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH); 
  analogWrite(PWMA, 255);   // Set max speed for A
  analogWrite(PWMB, 255);   // Set max speed for B
}

void loop() {
  // Find the amount of steps needed
  int steps = findStepsNeeded();

  // Rotate the motor clockwise
  stepMotorCW(steps); 
  delay(2000);

  // Rotate the motor counter clockwise
  stepMotorCCW(steps);  
  delay(2000);
}

int findStepsNeeded(void) {
  // Find how many output shaft revolutions needed for travel distance wanted
  int pitch = gearDiameter * 3.14159;
  
  // Find amount of output shaft revolutions needed
  double revolutions = travelDistance / pitch;

  // Convert the distance (cm) to steps
  int steps = revolutions * gearBoxRatio * stepRev;

  return steps;
}

void stepMotorCW(int steps) {
  for (int i = 1; i < steps; i++) {
    
    int stepDelay = 1;  // Adjust speed if you want

    // Step 1
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);

    // Step 2
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);

    // Step 3
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);

    // Step 4
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);
  }
}

void stepMotorCCW(int steps) {
  for (int i = 1; i < steps; i++) {
    
    int stepDelay = 1;  // Adjust speed if you want

    // Step 1
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);

    // Step 2
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);

    // Step 3
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);

    // Step 4
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);
  }
}


