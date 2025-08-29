// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// Battery swapping code
// Jamie Henson & Abhi
// Updated as of Aug 28th, 2025
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

// Define global pins
#define AIN1 4
#define AIN2 5
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define STBY 8 
#define PWMB 10
#define HALL_SENSOR_1_PIN 11
#define HALL_SENSOR_2_PIN 12
#define SWITCH_PIN 13  // Digital input from FC AUX/PWM channel (for CH7 passthrough)

// Define global variables for Hall Effect sensors
int hallSensor1Value = 0;
int hallSensor2Value = 0;

// Global variables to track motor and remote state
double travelDistance = 4; // [cm]
double gearDiameter = 3; // [cm]
int gearBoxRatio = 19;
int stepRev = 70; // Calculated number ??? <--- Check before flight
unsigned long startTime = 0;
bool motorActivated = false;

// Global variables for docking
unsigned long dockedStartTime = 0;
bool dockingConfirmed = false;

// Find how many output shaft revolutions needed for travel distance wanted
int pitch = gearDiameter * 3.14159;
  
// Find amount of output shaft revolutions needed
double revolutions = travelDistance / pitch;

// Convert the distance (cm) to steps
int steps = revolutions * gearBoxRatio * stepRev;

int lastState = 0; // 0=OFF, 1=CW, -1=CCW

void setup() {
  // Set up baud rate
  Serial.begin(9600);

  // Begin the timer
  unsigned long currentTime = millis();

  // Set up pins
  pinMode(HALL_SENSOR_1_PIN, INPUT);
  pinMode(HALL_SENSOR_2_PIN, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH); 
  analogWrite(PWMA, 255);   // Set max speed for A
  analogWrite(PWMB, 255);   // Set max speed for B

  // Down here is for testing 
  // |+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
  // delay(3000);

  // // Rotate the motor clockwise
  // stepMotorCW(); 
  // Serial.println("CW");
  // delay(5000);

  // // Rotate the motor counter clockwise
  // stepMotorCCW(); 
  // Serial.println("CCW");
  // delay(2000);
  // |+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
}

void loop() {
  // Read and store sensor data value (0 = contact and 1 = no contact)
  hallSensor1Value = readHallDebounced(HALL_SENSOR_1_PIN);
  hallSensor2Value = readHallDebounced(HALL_SENSOR_2_PIN);
  Serial.println("Sensor 1 value = " + String(hallSensor1Value));
  Serial.println("Sensor 2 value = " + String(hallSensor2Value));
  
  if(hallSensor1Value == 0 && hallSensor2Value == 0){
    if(!dockingConfirmed){
        dockedStartTime = millis();
        dockingConfirmed = true;
    } 
    else if(millis() - dockedStartTime >= 3000){
        // 3 seconds passed, start swap
        startBatterySwap();
    }
  } 
  else {
    dockingConfirmed = false; // reset if contact lost
  }
}

int readHallDebounced(int pin) {
    int val = digitalRead(pin);
    delay(5);  // 5 ms debounce
    int val2 = digitalRead(pin);
    return (val == val2) ? val : 1; // Default to "no contact" if bouncing
}

void startBatterySwap() {
  // Read RC PWM
  int pwm = pulseIn(SWITCH_PIN, HIGH, 25000); 

  int currentState = 0;
  if (pwm > 1800) {
    currentState = 1;  // CW
  } else if (pwm < 1200) {
    currentState = -1; // CCW
  } else {
    currentState = 0;  // Neutral
  }

  if (currentState != lastState) {
    if (currentState == 1) {
      Serial.println("CW");
      stepMotorCW();
    } else if (currentState == -1) {
      Serial.println("CCW");
      stepMotorCCW();
    } else {
      Serial.println("OFF");
    }
    lastState = currentState;  // Update last known state
  }
  delay(100);  // Basic debounce
}

void stepMotorCW() {
  int stepDelay = 3; // Adjust speed if you want
  for (int i = 0; i < steps; i++) {
    // Step a
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);

    // Step b
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);

    // Step c
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);

    // Step d
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);
  }
}

void stepMotorCCW() {
  int stepDelay = 3;  // Adjust speed if you want
  for (int i = 0; i < steps; i++) {
    // Step a
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);

    // Step b
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    delay(stepDelay);

    // Step c
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);

    // Step d
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    delay(stepDelay);
  }
}
