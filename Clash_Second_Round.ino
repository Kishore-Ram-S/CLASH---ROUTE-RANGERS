
// CLASH PHASE 2


#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>

// --- COMPONENT OBJECTS ---
Servo servo_lift;
Servo servo_grip;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
QTRSensors qtr;

// --- PIN DEFINITIONS ---
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

#define PIN_CALIBRATE 13
#define PIN_IR 12        
#define PIN_SERVO_LIFT 10
#define PIN_SERVO_GRIP 11

// --- MOTOR SETUP ---
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// --- PID CONSTANTS ---
float Kp = 0.09;
float Ki = 0.0002; 
float Kd = 0.45; 
int P, I, D, lastError = 0;

// --- SPEED SETTINGS ---
const uint8_t MAX_SPEED = 100;
const uint8_t BASE_SPEED = 50;
const uint8_t TURN_SPEED = 70;

// --- SERVO VARIABLES ---
int pos_lift = 0;
int pos_grip = 0;
bool boxLifted = false;

// --- BOX COUNTERS ---
int redCount = 3;
int blueCount = 3;
int greenCount = 1;

// --- STATE MACHINE ---
enum MissionState {
  IDLE,
  NAV_TO_SCAN,      // Phase 1: Blind Driving
  SCAN_UTURN,       // Transition: The 180 Spin
  HUNTING,          // Phase 2: Active Search & Rescue
  AVOIDING,         
  DELIVERING,       
  DROP_OFF,         
  PARK              
};

MissionState currentState = IDLE;
int heldBoxColor = -1; // 0=Red, 1=Green, 2=Blue

// --- SENSOR VARIABLES ---
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
const int BLACK_THRESH = 600;    

// --- GAP CHAIN VARIABLES ---
bool wasOnLine = true;        
int gapCount = 0;             
unsigned long lastGapTime = 0;
const unsigned long GAP_CHAIN_TIMEOUT = 2000; 
const int BROKEN_LINE_THRESHOLD = 3; 

// --- TURN COOLDOWN ---
unsigned long lastTurnTime = 0;

// --- FUNCTION PROTOTYPES ---
void stopRobot(); 
void runPID_WithGapCounting();
void runPID_Normal();
void checkObject();
int getPreciseColor();
void performUTurn();
void nudgeAround();
void checkIntersectionForDelivery();
void turnLeft();
void turnRight();
void lift(); 
void drop(); 
void resetServos();

void setup() {
  Serial.begin(9600);
  
  // 1. Hardware Init
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A3, A2, A1, A0}, SensorCount);

  pinMode(PIN_CALIBRATE, INPUT);
  pinMode(PIN_IR, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  servo_lift.attach(PIN_SERVO_LIFT);
  servo_grip.attach(PIN_SERVO_GRIP);
  resetServos(); 

  // 2. Color Sensor Init
  if (tcs.begin()) {
    Serial.println("TCS34725 Found.");
    tcs.setInterrupt(false); 
  } else {
    Serial.println("ERR: No Color Sensor");
    while(1); 
  }

  // 3. Calibration
  Serial.println("Waiting for Calibration (Press Button)...");
  while (digitalRead(PIN_CALIBRATE) == LOW) {} 
  
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrating...");
  for (int i = 0; i < 250; i++) {
    qtr.calibrate();
    if(i < 60 || i > 180) { motor1.drive(40); motor2.drive(-40); }
    else { motor1.drive(-40); motor2.drive(40); }
    delay(20);
  }
  stopRobot();
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Ready. Press to Start.");
  
  while (digitalRead(PIN_CALIBRATE) == LOW) {} 
  delay(1000);
  
  // Start Mission
  currentState = NAV_TO_SCAN;
  gapCount = 0; 
}

void loop() {
  switch (currentState) {
    case IDLE:
      stopRobot();
      break;

    case NAV_TO_SCAN:
      // --- PHASE 1: IGNORE BOXES ---
      runPID_WithGapCounting(); 
      
      // Check for Broken Lines (End of Scan)
      if (gapCount >= BROKEN_LINE_THRESHOLD) {
         Serial.println("Phase 1 Complete.");
         motor1.drive(BASE_SPEED); motor2.drive(BASE_SPEED);
         delay(400); 
         stopRobot();
         currentState = SCAN_UTURN;
      }
      break;

    case SCAN_UTURN:
      performUTurn(); 
      currentState = HUNTING;
      break;

    case HUNTING:
      // --- PHASE 2: SEARCH & RESCUE ---
      runPID_Normal(); 
      checkObject(); 
      break;

    case AVOIDING:
      nudgeAround();
      currentState = HUNTING;
      break;

    case DELIVERING:
      runPID_Normal();
      checkIntersectionForDelivery();
      break;

    case DROP_OFF:
      drop(); 
      Serial.println("Resuming Hunt.");
      performUTurn(); 
      currentState = HUNTING; 
      break;

    case PARK:
      stopRobot();
      Serial.println("MISSION COMPLETE");
      while(1); 
      break;
  }
}

// =============================================================
// LOGIC FUNCTIONS
// =============================================================

// --- PID PHASE 1 ---
void runPID_WithGapCounting() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  bool currentlyOnLine = false;
  for(int i=0; i<SensorCount; i++) {
    if(sensorValues[i] > BLACK_THRESH) currentlyOnLine = true;
  }

  // GAP COUNTING
  if (!wasOnLine && currentlyOnLine) {
    unsigned long timeBetweenGaps = millis() - lastGapTime;
    if (timeBetweenGaps > GAP_CHAIN_TIMEOUT) {
       gapCount = 1; 
       Serial.println("Gap Chain Reset");
    } else {
       gapCount++; 
       Serial.print("Gap: "); Serial.println(gapCount);
    }
    lastGapTime = millis(); 
  }
  wasOnLine = currentlyOnLine; 

  // MOVE
  if (!currentlyOnLine) {
    motor1.drive(BASE_SPEED); motor2.drive(BASE_SPEED);
    return; 
  }
  int error = position - 2500;
  P = error; I += error; D = error - lastError; lastError = error;
  I = constrain(I, -1000, 1000);
  int motorSpeed = P * Kp + I * Ki + D * Kd;
  int speedA = constrain(BASE_SPEED + motorSpeed, 0, MAX_SPEED);
  int speedB = constrain(BASE_SPEED - motorSpeed, 0, MAX_SPEED);
  motor1.drive(speedA); motor2.drive(speedB);
}

// --- PID PHASE 2 ---
void runPID_Normal() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;
  P = error; I += error; D = error - lastError; lastError = error;
  I = constrain(I, -1000, 1000);
  int motorSpeed = P * Kp + I * Ki + D * Kd;
  int speedA = constrain(BASE_SPEED + motorSpeed, 0, MAX_SPEED);
  int speedB = constrain(BASE_SPEED - motorSpeed, 0, MAX_SPEED);
  motor1.drive(speedA); motor2.drive(speedB);
}

// --- OBJECT DETECTION (INSTANT GRAB) ---
void checkObject() {
  if (digitalRead(PIN_IR) == 0) {
    // STOP INSTANTLY
    stopRobot();
    delay(500); // Wait for potential wobbling to stop
    
    Serial.println("Object Detected. Scanning...");
    int color = getPreciseColor(); 
    
    // --- GREEN (AVOID) ---
    if (color == 1) { 
        nudgeAround(); 
        currentState = HUNTING;
    } 
    // --- RED OR BLUE (TARGET) ---
    else if (color == 0 || color == 2) { 
      
      bool pickUp = false;
      if (color == 0 && redCount > 0) {
         Serial.println("RED TARGET");
         redCount--;
         pickUp = true;
      } else if (color == 2 && blueCount > 0) {
         Serial.println("BLUE TARGET");
         blueCount--;
         pickUp = true;
      }

      if (pickUp) {
        // --- INSTANT GRAB SEQUENCE ---
        
        // 1. GRAB IMMEDIATELY (No forward motion)
        lift(); 
        
        // 2. LOADED U-TURN (Spin 180 with box)
        Serial.println("Performing Loaded U-Turn...");
        performUTurn(); 
        
        heldBoxColor = color;
        currentState = DELIVERING; 
      } else {
        // Known color but we don't need it (count 0)
        nudgeAround();
      }
    }
    else {
      // Noise
      motor1.drive(-40); motor2.drive(-40); delay(200); 
    }
  }
}

// --- COLOR SENSOR ---
int getPreciseColor() {
  long rTot = 0, gTot = 0, bTot = 0;
  for(int i=0; i<3; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    rTot += r; gTot += g; bTot += b;
    delay(50);
  }
  long r = rTot/3; long g = gTot/3; long b = bTot/3;
  if (g > r && g > b) return 1; // Green
  if (r > g * 1.5 && r > b) return 0; // Red
  if (b > r && b > g) return 2; // Blue
  return 3; 
}

// --- MANEUVERS ---
void performUTurn() {
  motor1.drive(TURN_SPEED);
  motor2.drive(-TURN_SPEED);
  delay(600); 
  while(true) {
    qtr.readLineBlack(sensorValues);
    if(sensorValues[2] > BLACK_THRESH || sensorValues[3] > BLACK_THRESH) break;
  }
  stopRobot();
  delay(200);
}

void nudgeAround() {
  motor1.drive(-40); motor2.drive(-40); delay(300);
  motor1.drive(60); motor2.drive(-60); delay(400);
  motor1.drive(70); motor2.drive(40); delay(800);
  motor1.drive(-60); motor2.drive(60); delay(400);
  while(true) {
    motor1.drive(40); motor2.drive(40);
    qtr.readLineBlack(sensorValues);
    if(sensorValues[2] > BLACK_THRESH || sensorValues[3] > BLACK_THRESH) break;
  }
  stopRobot();
}

void checkIntersectionForDelivery() {
  bool extremeLeft = sensorValues[0] > BLACK_THRESH;
  bool extremeRight = sensorValues[5] > BLACK_THRESH;
  
  if ((extremeLeft || extremeRight) && (millis() - lastTurnTime > 500)) {
    stopRobot();
    delay(50);
    qtr.readLineBlack(sensorValues);
    if (sensorValues[0] > BLACK_THRESH || sensorValues[5] > BLACK_THRESH) {
      if (heldBoxColor == 0) turnLeft();
      else if (heldBoxColor == 2) turnRight();
      lastTurnTime = millis();
      currentState = DROP_OFF;
    }
  }
}

void turnLeft() {
  motor1.drive(50); motor2.drive(50); delay(200); 
  motor1.drive(-TURN_SPEED); motor2.drive(TURN_SPEED); delay(300); 
  while(sensorValues[2] < BLACK_THRESH) { qtr.readLineBlack(sensorValues); } 
  stopRobot();
}

void turnRight() {
  motor1.drive(50); motor2.drive(50); delay(200); 
  motor1.drive(TURN_SPEED); motor2.drive(-TURN_SPEED); delay(300); 
  while(sensorValues[2] < BLACK_THRESH) { qtr.readLineBlack(sensorValues); } 
  stopRobot();
}

// --- CUSTOM SERVO FUNCTIONS ---

void lift() {
    Serial.println("ACTION: Lifting Box...");
    for (pos_lift = 160; pos_lift >= 55; pos_lift -= 1) { servo_lift.write(pos_lift); delay(15); }
    for (pos_grip = 90; pos_grip >= 20; pos_grip -= 1) { servo_grip.write(pos_grip); delay(15); }
    for (pos_lift = 55; pos_lift <= 160; pos_lift += 1) { servo_lift.write(pos_lift); delay(15); }
    boxLifted = true; 
    Serial.println(">> BOX SECURED.");
}

void drop() {
    Serial.println("ACTION: Dropping Box...");
    for (pos_lift = 160; pos_lift >= 55; pos_lift -= 1) { servo_lift.write(pos_lift); delay(15); }
    for (pos_grip = 20; pos_grip <= 90; pos_grip += 1) { servo_grip.write(pos_grip); delay(15); }
    for (pos_lift = 55; pos_lift <= 160; pos_lift += 1) { servo_lift.write(pos_lift); delay(15); }
    Serial.println(">> MISSION COMPLETE.");
}

void resetServos() {
  servo_lift.write(160); // Up
  servo_grip.write(90);  // Open
  boxLifted = false;
}

void stopRobot() {
  motor1.brake();
  motor2.brake();
}