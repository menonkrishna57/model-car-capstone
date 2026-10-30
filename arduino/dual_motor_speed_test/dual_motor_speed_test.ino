/**
 * dual_motor_speed_test.ino
 * =============================================================================
 * Dual Motor & PWM Speed Control Diagnostic Sketch
 * 
 * Hardware Setup (L298N Dual H-Bridge & Arduino Uno):
 * ---------------------------------------------------
 *   LEFT MOTOR (Channel B):
 *     - PWM (ENB): Pin D6  (PWM Speed: 0 - 255)
 *     - IN3:       Pin D10 (Direction bit 1: HIGH = Forward)
 *     - IN4:       Pin D11 (Direction bit 2: HIGH = Reverse)
 * 
 *   RIGHT MOTOR (Channel A):
 *     - PWM (ENA): Pin D5  (PWM Speed: 0 - 255)
 *     - IN1:       Pin D8  (Direction bit 1: HIGH = Forward)
 *     - IN2:       Pin D9  (Direction bit 2: HIGH = Reverse)
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

// Pin Definitions (Matches physical chassis wiring and pins.h):
// Left Motor Pair  -> L298N Channel B: ENB = D6, IN3 = D10, IN4 = D11
// Right Motor Pair -> L298N Channel A: ENA = D5, IN1 = D8,  IN2 = D9
const int PIN_LEFT_PWM  = 6;   // L298N ENB (Timer0/OC0A)
const int PIN_LEFT_IN1  = 10;  // L298N IN3 (Dir bit 1)
const int PIN_LEFT_IN2  = 11;  // L298N IN4 (Dir bit 2)

const int PIN_RIGHT_PWM = 5;   // L298N ENA (Timer0/OC0B)
const int PIN_RIGHT_IN1 = 8;   // L298N IN1 (Dir bit 1)
const int PIN_RIGHT_IN2 = 9;   // L298N IN2 (Dir bit 2)

// Helper functions for motor control
// Native forward polarity: Left (IN3=LOW, IN4=HIGH), Right (IN1=LOW, IN2=HIGH)
// Native reverse polarity: Left (IN3=HIGH, IN4=LOW), Right (IN1=HIGH, IN2=LOW)
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0 && speed < 180) speed = 180;
  if (speed < 0 && speed > -180) speed = -180;
  if (speed > 0) {
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, HIGH);
    analogWrite(PIN_LEFT_PWM, speed);
  } else if (speed < 0) {
    digitalWrite(PIN_LEFT_IN1, HIGH);
    digitalWrite(PIN_LEFT_IN2, LOW);
    analogWrite(PIN_LEFT_PWM, -speed);
  } else {
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, LOW);
    analogWrite(PIN_LEFT_PWM, 0);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0 && speed < 180) speed = 180;
  if (speed < 0 && speed > -180) speed = -180;
  if (speed > 0) {
    digitalWrite(PIN_RIGHT_IN1, LOW);
    digitalWrite(PIN_RIGHT_IN2, HIGH);
    analogWrite(PIN_RIGHT_PWM, speed);
  } else if (speed < 0) {
    digitalWrite(PIN_RIGHT_IN1, HIGH);
    digitalWrite(PIN_RIGHT_IN2, LOW);
    analogWrite(PIN_RIGHT_PWM, -speed);
  } else {
    digitalWrite(PIN_RIGHT_IN1, LOW);
    digitalWrite(PIN_RIGHT_IN2, LOW);
    analogWrite(PIN_RIGHT_PWM, 0);
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
  Serial.println(F("Pins: Left Motor  [Ch.B] (PWM=6, IN3=10, IN4=11)"));
  Serial.println(F("      Right Motor [Ch.A] (PWM=5, IN1=8,  IN2=9)"));
  Serial.println(F("Polarity: Forward is Left(IN3=L, IN4=H), Right(IN1=L, IN2=H)"));
  Serial.println(F("STATUS: DISARMED at startup (motors stopped at 0 PWM)"));
  Serial.println(F("NOTICE FOR KRISHNA: Elevate wheels before testing!"));
  Serial.println(F("Commands (Serial Monitor at 115200 baud):"));
  Serial.println(F("  TEST         - Run automated test cycle"));
  Serial.println(F("  RAMP         - Run PWM speed ramp sweep"));
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
  Serial.println(F("  -> Left Forward @ PWM 180 (Medium)"));
  setLeftMotor(180);
  setRightMotor(0);
  delay(1500);

  Serial.println(F("  -> Left Forward @ PWM 220 (High)"));
  setLeftMotor(220);
  delay(1500);

  Serial.println(F("  -> Left STOP"));
  stopMotors();
  delay(800);

  Serial.println(F("  -> Left Reverse @ PWM 185"));
  setLeftMotor(-185);
  delay(1500);

  Serial.println(F("  -> Left STOP"));
  stopMotors();
  delay(800);

  // -------------------------------------------------------------
  // Test 2: Right Motor Solo
  // -------------------------------------------------------------
  Serial.println(F("\n[2/5] Testing RIGHT MOTOR:"));
  Serial.println(F("  -> Right Forward @ PWM 180 (Medium)"));
  setLeftMotor(0);
  setRightMotor(180);
  delay(1500);

  Serial.println(F("  -> Right Forward @ PWM 220 (High)"));
  setRightMotor(220);
  delay(1500);

  Serial.println(F("  -> Right STOP"));
  stopMotors();
  delay(800);

  Serial.println(F("  -> Right Reverse @ PWM 185"));
  setRightMotor(-185);
  delay(1500);

  Serial.println(F("  -> Right STOP"));
  stopMotors();
  delay(800);

  // -------------------------------------------------------------
  // Test 3: Both Motors Synchronized
  // -------------------------------------------------------------
  Serial.println(F("\n[3/5] Testing BOTH MOTORS:"));
  Serial.println(F("  -> Both Forward @ PWM 180"));
  setLeftMotor(180);
  setRightMotor(180);
  delay(2000);

  Serial.println(F("  -> Both Forward @ PWM 240 (Fast)"));
  setLeftMotor(240);
  setRightMotor(240);
  delay(1500);

  Serial.println(F("  -> Both STOP"));
  stopMotors();
  delay(1000);

  Serial.println(F("  -> Both Reverse @ PWM 185"));
  setLeftMotor(-185);
  setRightMotor(-185);
  delay(2000);

  Serial.println(F("  -> Both STOP"));
  stopMotors();
  delay(1000);

  // -------------------------------------------------------------
  // Test 4: Differential Turn (Skid Steer)
  // -------------------------------------------------------------
  Serial.println(F("\n[4/5] Testing DIFFERENTIAL TURNS:"));
  Serial.println(F("  -> Turn RIGHT in place (Left FWD 190, Right REV 190)"));
  setLeftMotor(190);
  setRightMotor(-190);
  delay(1500);

  Serial.println(F("  -> STOP"));
  stopMotors();
  delay(800);

  Serial.println(F("  -> Turn LEFT in place (Left REV 190, Right FWD 190)"));
  setLeftMotor(-190);
  setRightMotor(190);
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

  // Configure Left Motor Pins (Channel B)
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);

  // Configure Right Motor Pins (Channel A)
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);

  // Ensure all motors are off initially (disarmed startup per R2)
  stopMotors();

  printBanner();
  Serial.println(F("[DISARMED] Motors safe and stationary. Send 'TEST' to run diagnostic cycle."));
}

void loop() {
  // Listen for commands in Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseSerialCommand(input);
  }
}
