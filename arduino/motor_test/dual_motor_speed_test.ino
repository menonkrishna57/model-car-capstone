/**
 * dual_motor_speed_test.ino
 * =============================================================================
 * Dual Motor & PWM Speed Control Diagnostic Sketch
 * 
 * Hardware Setup (L298N Dual H-Bridge & Arduino Uno):
 * ---------------------------------------------------
 *   LEFT MOTOR (Channel A):
 *     - ENA: Pin D5  (PWM Speed: 0 - 255)
 *     - IN1: Pin D8  (Direction bit 1: HIGH = Forward)
 *     - IN2: Pin D9  (Direction bit 2: HIGH = Reverse)
 * 
 *   RIGHT MOTOR (Channel B):
 *     - ENB: Pin D6  (PWM Speed: 0 - 255)
 *     - IN3: Pin D10 (Direction bit 1: HIGH = Forward)
 *     - IN4: Pin D11 (Direction bit 2: HIGH = Reverse)
 * 
 * Test Features:
 * --------------
 *   1. Automated Diagnostic Sequence:
 *      - Left motor individual direction and speed check
 *      - Right motor individual direction and speed check
 *      - Both motors synchronized forward/reverse
 *      - Smooth PWM speed ramping (0 -> 255 -> 0) in steps
 *      - Differential turning (spin left / spin right)
 *   2. Interactive Serial Monitor Commands (at 115200 baud):
 *      - "TEST"        : Run the full automated test suite
 *      - "RAMP"        : Test PWM speed gradient on both motors
 *      - "L <speed>"   : Drive left motor (-255 to 255, negative = reverse)
 *      - "R <speed>"   : Drive right motor (-255 to 255, negative = reverse)
 *      - "BOTH <speed>": Drive both motors (-255 to 255)
 *      - "STOP"        : Immediately stop all motors
 * =============================================================================
 */

// Pin Definitions (Matches pins.h single source of truth)
const int PIN_ENA = 5;   // Left Motor PWM (Timer0/OC0B)
const int PIN_IN1 = 8;   // Left Motor Dir 1
const int PIN_IN2 = 9;   // Left Motor Dir 2

const int PIN_ENB = 6;   // Right Motor PWM (Timer0/OC0A)
const int PIN_IN3 = 10;  // Right Motor Dir 1
const int PIN_IN4 = 11;  // Right Motor Dir 2

// Helper functions for motor control
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, speed);
  } else if (speed < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -speed);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, speed);
  } else if (speed < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENB, -speed);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, 0);
  }
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

void printBanner() {
  Serial.println(F("\n=================================================="));
  Serial.println(F("     DUAL MOTOR & SPEED TEST DIAGNOSTIC           "));
  Serial.println(F("=================================================="));
  Serial.println(F("Pins: Left Motor (ENA=5, IN1=8, IN2=9)"));
  Serial.println(F("      Right Motor (ENB=6, IN3=10, IN4=11)"));
  Serial.println(F("Commands (Serial Monitor at 115200 baud):"));
  Serial.println(F("  TEST         - Run automated test cycle"));
  Serial.println(F("  RAMP         - Run PWM speed ramp sweep (0-255)"));
  Serial.println(F("  L <speed>    - Set left motor speed (-255 to 255)"));
  Serial.println(F("  R <speed>    - Set right motor speed (-255 to 255)"));
  Serial.println(F("  BOTH <speed> - Set both motors speed (-255 to 255)"));
  Serial.println(F("  STOP         - Stop all motors immediately"));
  Serial.println(F("==================================================\n"));
}

void runAutomatedTest() {
  Serial.println(F(">>> STARTING AUTOMATED MOTOR TEST CYCLE <<<"));
  Serial.println(F("CAUTION: Ensure wheels are elevated off ground!\n"));

  // -------------------------------------------------------------
  // Test 1: Left Motor Solo
  // -------------------------------------------------------------
  Serial.println(F("[1/5] Testing LEFT MOTOR:"));
  Serial.println(F("  -> Left Forward @ PWM 130 (Medium)"));
  setLeftMotor(130);
  setRightMotor(0);
  delay(1500);

  Serial.println(F("  -> Left Forward @ PWM 220 (High)"));
  setLeftMotor(220);
  delay(1500);

  Serial.println(F("  -> Left STOP"));
  stopMotors();
  delay(800);

  Serial.println(F("  -> Left Reverse @ PWM 150"));
  setLeftMotor(-150);
  delay(1500);

  Serial.println(F("  -> Left STOP"));
  stopMotors();
  delay(800);

  // -------------------------------------------------------------
  // Test 2: Right Motor Solo
  // -------------------------------------------------------------
  Serial.println(F("\n[2/5] Testing RIGHT MOTOR:"));
  Serial.println(F("  -> Right Forward @ PWM 130 (Medium)"));
  setLeftMotor(0);
  setRightMotor(130);
  delay(1500);

  Serial.println(F("  -> Right Forward @ PWM 220 (High)"));
  setRightMotor(220);
  delay(1500);

  Serial.println(F("  -> Right STOP"));
  stopMotors();
  delay(800);

  Serial.println(F("  -> Right Reverse @ PWM 150"));
  setRightMotor(-150);
  delay(1500);

  Serial.println(F("  -> Right STOP"));
  stopMotors();
  delay(800);

  // -------------------------------------------------------------
  // Test 3: Both Motors Synchronized
  // -------------------------------------------------------------
  Serial.println(F("\n[3/5] Testing BOTH MOTORS:"));
  Serial.println(F("  -> Both Forward @ PWM 140"));
  setLeftMotor(140);
  setRightMotor(140);
  delay(2000);

  Serial.println(F("  -> Both Forward @ PWM 240 (Fast)"));
  setLeftMotor(240);
  setRightMotor(240);
  delay(1500);

  Serial.println(F("  -> Both STOP"));
  stopMotors();
  delay(1000);

  Serial.println(F("  -> Both Reverse @ PWM 140"));
  setLeftMotor(-140);
  setRightMotor(-140);
  delay(2000);

  Serial.println(F("  -> Both STOP"));
  stopMotors();
  delay(1000);

  // -------------------------------------------------------------
  // Test 4: Differential Turn (Skid Steer)
  // -------------------------------------------------------------
  Serial.println(F("\n[4/5] Testing DIFFERENTIAL TURNS:"));
  Serial.println(F("  -> Turn RIGHT in place (Left FWD 150, Right REV 150)"));
  setLeftMotor(150);
  setRightMotor(-150);
  delay(1500);

  Serial.println(F("  -> STOP"));
  stopMotors();
  delay(800);

  Serial.println(F("  -> Turn LEFT in place (Left REV 150, Right FWD 150)"));
  setLeftMotor(-150);
  setRightMotor(150);
  delay(1500);

  Serial.println(F("  -> STOP"));
  stopMotors();
  delay(1000);

  // -------------------------------------------------------------
  // Test 5: Speed Ramping / PWM Sweep
  // -------------------------------------------------------------
  Serial.println(F("\n[5/5] Testing PWM SPEED RAMPING (0 -> 255 -> 0):"));
  runSpeedRamp();

  Serial.println(F("\n>>> AUTOMATED TEST COMPLETE! <<<\n"));
}

void runSpeedRamp() {
  Serial.println(F("  -> Ramping UP speed from 0 to 255..."));
  for (int spd = 0; spd <= 255; spd += 25) {
    setLeftMotor(spd);
    setRightMotor(spd);
    Serial.print(F("    PWM Speed: "));
    Serial.println(spd);
    delay(250);
  }
  delay(1000);

  Serial.println(F("  -> Ramping DOWN speed from 255 to 0..."));
  for (int spd = 255; spd >= 0; spd -= 25) {
    setLeftMotor(spd);
    setRightMotor(spd);
    Serial.print(F("    PWM Speed: "));
    Serial.println(spd);
    delay(250);
  }
  stopMotors();
  Serial.println(F("  -> Ramp complete. Motors stopped."));
}

void parseSerialCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd == "TEST") {
    runAutomatedTest();
  } else if (cmd == "RAMP") {
    runSpeedRamp();
  } else if (cmd == "STOP") {
    stopMotors();
    Serial.println(F("[ACK] STOP - All motors halted."));
  } else if (cmd.startsWith("L ")) {
    int spd = cmd.substring(2).toInt();
    setLeftMotor(spd);
    Serial.print(F("[ACK] Left motor speed set to: "));
    Serial.println(spd);
  } else if (cmd.startsWith("R ")) {
    int spd = cmd.substring(2).toInt();
    setRightMotor(spd);
    Serial.print(F("[ACK] Right motor speed set to: "));
    Serial.println(spd);
  } else if (cmd.startsWith("BOTH ")) {
    int spd = cmd.substring(5).toInt();
    setLeftMotor(spd);
    setRightMotor(spd);
    Serial.print(F("[ACK] Both motors speed set to: "));
    Serial.println(spd);
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
    Serial.println(F("Valid: TEST, RAMP, STOP, L <spd>, R <spd>, BOTH <spd>"));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1000); // Wait briefly for USB Serial

  // Configure Left Motor Pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  // Configure Right Motor Pins
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Ensure all motors are off initially
  stopMotors();

  printBanner();

  // Run the test cycle once on startup
  runAutomatedTest();
}

void loop() {
  // Listen for commands in Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseSerialCommand(input);
  }
}
