//CLASH PHASE_1 FINAL

#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>

Servo servo_lift;
Servo servo_grip;

// --- COLOR SENSOR ---
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_60X);

char data;
int pos_lift = 0;
int pos_grip = 0;

// QTR Sensor
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// PID
float Kp = 0.09;
float Ki = 0.0001;  
float Kd = 0.33;
int P, I, D, lastError = 0;

// --- MOTOR SETTINGS ---
int TURN_SPEED = 50; 
int TURN_SPEEDr = -50;
int TURN_ALIGN_DELAY = 300; 

// *** NEW: POST-TURN REVERSE SETTINGS ***
// Time to drive backward after a hard turn to catch the strip
const int REVERSE_AFTER_TURN_MS = 350; 
const int REVERSE_SPEED = 50; // Safe speed for reversing

// Timers
const int IR_TURN_IGNORE = 0;   
const int TURN_COOLDOWN_TIME = 1500;
const int STARTUP_GRACE_PERIOD = 0; 

const int BLACK_THRESHOLD = 700;

// SPEED
uint8_t maxspeeda = 90;
uint8_t maxspeedb = 90;
uint8_t basespeeda = 40;
uint8_t basespeedb = 40;

// Color Counting & Mission Flags
int countRed = 0;
int countGreen = 0;
int countBlue = 0;
boolean ir = false; 

boolean all_black = false;       
boolean printedAllBlack = false; 
boolean boxLifted = false;       

// Memory
unsigned long lastTurnTime = 0;
unsigned long missionStartTime = 0;

// Pins
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

int buttoncalibrate = 13;
int ir_pin = 12;

const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Ramp
uint8_t currentSpeedA = 0;
uint8_t currentSpeedB = 0;
const uint8_t rampStep = 5;
const uint16_t rampDelay = 15;
unsigned long lastRampTime = 0;

boolean onoff = false;
unsigned long lastWhiteTime = 0;
const unsigned long WHITE_TIMEOUT = 50;

void setup() 
{
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A6,A7}, SensorCount);

    Serial.begin(9600);
    Serial.println("--- SYSTEM BOOTING ---");

    if (tcs.begin()) {
        Serial.println("Color Sensor Found.");
    } else {
        Serial.println("ERROR: No TCS34725 found");
    }

    delay(500);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(buttoncalibrate, INPUT);
    pinMode(ir_pin, INPUT); 

    servo_lift.attach(10);
    servo_grip.attach(11);

    servo_lift.write(160);
    servo_grip.write(90);

    boolean Ok = false;
    Serial.println("Waiting for Calibration...");

    while (!Ok) 
    {
        if (digitalRead(buttoncalibrate) == HIGH) 
        {
            delay(1000);
            calibration();
            Ok = true;
        }
    }

    brake(motor1, motor2);
    Serial.println("Ready. Send 'S' to start.");
}

void calibration() 
{
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("CALIBRATING (360 SPIN)...");
    for (uint16_t i = 0; i < 200; i++) 
    {
        motor1.drive(30);   
        motor2.drive(-30);  
        qtr.calibrate();    
        delay(20);
    }
    brake(motor1, motor2);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Calibration Done.");
}

void loop() 
{
    if (Serial.available()) 
    {
        data = Serial.read();

        if (data == 'i') Kp += 0.005;
        else if (data == 'd') Kp -= 0.005;
        else if (data == 'j') Ki += 0.00005;
        else if (data == 'e') Ki -= 0.00005;
        else if (data == 'k') Kd += 0.01;
        else if (data == 'f') Kd -= 0.01;
        else if (data == 'l') { maxspeeda += 25; maxspeedb += 25; basespeeda += 25; basespeedb += 25; } 
        else if (data == 'g') { maxspeeda -= 25; maxspeedb -= 25; basespeeda -= 25; basespeedb -= 25; } 
        else if (data == 's') { 
            onoff = false; P = I = D = 0; 
            all_black = false; printedAllBlack = false; boxLifted = false; 
            countRed = 0; countGreen = 0; countBlue = 0; 
            Serial.println("STOPPED (Mission Reset).");
        } 
        else if (data == 'S') { 
            onoff = true; currentSpeedA = 0; currentSpeedB = 0; lastRampTime = millis(); 
            missionStartTime = millis(); 
            Serial.println("STARTED.");
        } 
        else if (data == 't') TURN_SPEED += 10;
        else if (data == 'T') TURN_SPEED -= 10;
    }

    if (digitalRead(buttoncalibrate) == HIGH) 
    {
        onoff = !onoff;
        delay(500);
        if (onoff) { 
            currentSpeedA = 0; currentSpeedB = 0; lastRampTime = millis(); 
            missionStartTime = millis(); 
            Serial.println("Button Start.");
        }
    }

    if (onoff) 
    {
        if (millis() - lastRampTime >= rampDelay) 
        {
            lastRampTime = millis();
            if (currentSpeedA < basespeeda) currentSpeedA += rampStep;
            if (currentSpeedB < basespeedb) currentSpeedB += rampStep;
        }

        PID_control(currentSpeedA, currentSpeedB);
    } 
    else 
    {
        brake(motor1, motor2);
    }
}

// --- HELPER FUNCTIONS ---

String getColorName(int c) {
    if (c == 0) return "RED";
    if (c == 1) return "GREEN";
    if (c == 2) return "BLUE";
    return "UNKNOWN";
}

int getColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 100) return 3; 
  if (r > g && r > b) return 0;
  else if (g > r && g > b) return 1;
  else if (b > r && b > g) return 2;
  else return 3;
}

void forward_brake(int posa, int posb) { motor1.drive(posa); motor2.drive(posb); }

int max_1(int &r, int &g, int &b) {
    if (r > g && r > b) return 0;
    else if (g > r && g > b) return 1;
    else if (b > r && b > g) return 2;
    return 0; 
}

void lift() {
    Serial.println("ACTION: Lifting Box...");
    for (pos_lift = 160; pos_lift >= 55; pos_lift -= 1) { servo_lift.write(pos_lift); delay(15); }
    for (pos_grip = 90; pos_grip >= 28; pos_grip -= 1) { servo_grip.write(pos_grip); delay(15); }
    for (pos_lift = 55; pos_lift <= 160; pos_lift += 1) { servo_lift.write(pos_lift); delay(15); }
    
    boxLifted = true; 
    Serial.println(">> BOX SECURED. Heading to Drop Zone.");
}

void drop() {
    Serial.println("ACTION: Dropping Box...");
    for (pos_lift = 160; pos_lift >= 55; pos_lift -= 1) { servo_lift.write(pos_lift); delay(15); }
    for (pos_grip = 28; pos_grip <= 90; pos_grip += 1) { servo_grip.write(pos_grip); delay(15); }
    for (pos_lift = 55; pos_lift <= 160; pos_lift += 1) { servo_lift.write(pos_lift); delay(15); }
    Serial.println(">> MISSION COMPLETE.");
}

// --- HARD TURN FUNCTIONS WITH REVERSE ---
void turnLeftBlocking() {
    Serial.println(">> DETECTED LEFT... Aligning...");
    
    // 1. Kick forward
    motor1.drive(40); motor2.drive(40);
    delay(TURN_ALIGN_DELAY); 

    // 2. Safety Check (Crossing)
    qtr.readLineBlack(sensorValues);
    if (sensorValues[5] > BLACK_THRESHOLD) {
        Serial.println(">> CROSSING DETECTED (Abort Turn)");
        motor1.drive(40); motor2.drive(40);
        delay(200);
        return; 
    }

    // 3. Spin
    motor1.drive(-TURN_SPEED); motor2.drive(TURN_SPEED);
    delay(200); 
    
    // 4. Catch Line
    while(1) {
        qtr.readLineBlack(sensorValues);
        if (sensorValues[1] > BLACK_THRESHOLD || sensorValues[2] > BLACK_THRESHOLD || 
            sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD) {
            break;
        }
    }

    // 5. Active Brake
    motor1.drive(50); motor2.drive(-50); 
    delay(50);
    brake(motor1, motor2);
    delay(200); // Fully stop before reversing

    // 6. *** REVERSE ALIGNMENT ***
    Serial.println(">> REVERSING to Catch Strip...");
    motor1.drive(-REVERSE_SPEED); 
    motor2.drive(-REVERSE_SPEED/2);
    delay(REVERSE_AFTER_TURN_MS); 
    
    brake(motor1, motor2);
    delay(100);

    Serial.println(">> TURN COMPLETE. Resuming PID.");
    lastTurnTime = millis(); 
}

void turnRightBlocking() {
    Serial.println(">> DETECTED RIGHT... Aligning...");
    
    // 1. Kick forward
    motor1.drive(40); motor2.drive(40);
    delay(TURN_ALIGN_DELAY); 

    // 2. Safety Check (Crossing)
    qtr.readLineBlack(sensorValues);
    if (sensorValues[0] > BLACK_THRESHOLD) {
        Serial.println(">> CROSSING DETECTED (Abort Turn)");
        motor1.drive(40); motor2.drive(40);
        delay(200);
        return; 
    }

    // 3. Spin
    motor1.drive(TURN_SPEED); motor2.drive(-TURN_SPEED);
    delay(200); 
    
    // 4. Catch Line
    while(1) {
        qtr.readLineBlack(sensorValues);
        if (sensorValues[1] > BLACK_THRESHOLD || sensorValues[2] > BLACK_THRESHOLD || 
            sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD) {
            break;
        }
    }

    // 5. Active Brake
    motor1.drive(-50); motor2.drive(50); 
    delay(50);
    brake(motor1, motor2);
    delay(200);

    // 6. *** REVERSE ALIGNMENT ***
    Serial.println(">> REVERSING to Catch Strip...");
    motor1.drive(-REVERSE_SPEED/2); 
    motor2.drive(-REVERSE_SPEED);
    delay(REVERSE_AFTER_TURN_MS); 
    
    brake(motor1, motor2);
    delay(100);

    Serial.println(">> TURN COMPLETE. Resuming PID.");
    lastTurnTime = millis(); 
}

void before_all_black () {
    // STARTUP SAFETY
    if (millis() - missionStartTime < STARTUP_GRACE_PERIOD) return;
    
    // IR IGNORE removed (Detects immediately after reverse)

    if (digitalRead(ir_pin) == 0 && ir==false) { 
        brake(motor1,motor2);
        delay(1000); 
        ir=true; 
        int color = getColor(); 
        if (color == 0) countRed++;
        else if (color == 1) countGreen++;
        else if (color == 2) countBlue++;
        
        Serial.print("Detected: "); Serial.println(getColorName(color));
        Serial.print("R:"); Serial.print(countRed); Serial.print(" G:"); Serial.print(countGreen); Serial.print(" B:"); Serial.println(countBlue);
    }
    else if (digitalRead(ir_pin) == 1 && ir==true) {
        ir = false;
    }
}

void after_all_black () {
    if (boxLifted) {
         // Drive forward until All Black is detected
        while (true) {
            motor1.drive(60);  // Adjust speed if needed (stable speed)
            motor2.drive(60); 

            qtr.readLineBlack(sensorValues); // Read sensors

            // Check if ALL sensors are seeing black
            bool allSensorsBlack = true;
            for (uint8_t i = 0; i < SensorCount; i++) {
                if (sensorValues[i] < BLACK_THRESHOLD) {
                    allSensorsBlack = false;
                    break;
                }
            }

            // If all are black, Stop and Exit this loop
            if (allSensorsBlack) {
                brake(motor1, motor2);
                delay(100); // Small pause to stabilize
                break;
            }
        }
            
    }
    if (millis() - missionStartTime < STARTUP_GRACE_PERIOD) return;

    if (digitalRead(ir_pin) == 0 && ir==false) { 
        brake(motor1,motor2);
        delay(1000); 
        ir=true; 
        int color = getColor();
        int maximum = max_1(countRed,countGreen,countBlue);
        if (color == maximum) {
            Serial.println(">>> MATCH! <<<");
            lift();
        } else {
            Serial.println(">>> NO MATCH. Ignoring. <<<");
        }
        delay(1000); 
    }
    else if (digitalRead(ir_pin) == 1) {
        ir = false;
    }
}

void PID_control(uint8_t baseA, uint8_t baseB) 
{
    uint16_t position = qtr.readLineBlack(sensorValues);

    // *** NEW: DROP LOGIC ***
    // Checks if the box is lifted and we see the Final All-Black Line
    if (boxLifted && (millis() - lastTurnTime > TURN_COOLDOWN_TIME)) 
    {
        bool isDropZone = true;
        for (uint8_t i = 0; i < SensorCount; i++) {
            if (sensorValues[i] <= BLACK_THRESHOLD) {
                isDropZone = false;
                break;
            }
        }

        if (isDropZone) {
            Serial.println("!!! DROP ZONE DETECTED !!!");
            brake(motor1, motor2);
            delay(500);
            drop();
            onoff = false; P = I = D = 0; 
            all_black = false; printedAllBlack = false; boxLifted = false; 
            countRed = 0; countGreen = 0; countBlue = 0; 
            Serial.println("STOPPED (Mission Reset).");
        }
    }
    // ************************

    // --- CHECK FOR HARD TURNS ---
    if (millis() - lastTurnTime > TURN_COOLDOWN_TIME) {
        
        bool extremeLeft = (sensorValues[0] > BLACK_THRESHOLD);
        bool extremeRight = (sensorValues[5] > BLACK_THRESHOLD);

        if (extremeLeft && extremeRight) {
             // Crossing Protection
             Serial.println(">> CROSSING: Forcing Straight");
             position = 2500; 
             lastTurnTime = millis(); 
        }
        else if (extremeLeft) {
            turnLeftBlocking();
            lastError = 0; 
            return; 
        }
        else if (extremeRight) {
            turnRightBlocking();
            lastError = 0;
            return; 
        }
    }

    // --- CHECK FOR ALL BLACK (First Pass / Hunting Mode Switch) ---
    if ((!all_black) && (millis() - lastTurnTime > 750)) {
        bool tempAllBlack = true;
        for (uint8_t i = 0; i < SensorCount; i++) {
            if (sensorValues[i] <= BLACK_THRESHOLD) {
                tempAllBlack = false;
                break;
            }
        }
        if (tempAllBlack) {
            all_black = true; 
            Serial.println("!!! FIRST ALL BLACK PASSED -> HUNTING MODE !!!");
        }
    }

    // --- PID CALCULATION ---
    int error = position - 2500;
    P = error;
    I += error;
    D = error - lastError;
    lastError = error;

    int motorspeed = P * Kp + I * Ki + D * Kd;
    int motorspeeda = baseA + motorspeed;
    int motorspeedb = baseB - motorspeed;

    motorspeeda = constrain(motorspeeda, 0, maxspeeda);
    motorspeedb = constrain(motorspeedb, 0, maxspeedb);
    
    // --- WHITE GAP CHECK ---
    bool allWhite = true;
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > BLACK_THRESHOLD) allWhite = false;
    }

    if (allWhite) {
        forward_brake(baseA, baseB);
    } 
    else {
        forward_brake(motorspeeda, motorspeedb);
    }

    // --- MODE EXECUTION ---
    if (!all_black) {
        before_all_black(); 
    }
    else {
        after_all_black(); 
    }
}