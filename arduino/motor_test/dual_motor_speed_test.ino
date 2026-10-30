// =============================================================================
// 4WD SKID-STEER OBSTACLE AVOIDANCE (VERIFIED HARDWARE CONFIG)
// - Left Sensor : Trig = Pin 7,  Echo = Pin 13
// - Right Sensor: Trig = Pin 12, Echo = Pin 4
// - L298N Motors: ENA = Pin 5, IN1 = Pin 8,  IN2 = Pin 9
//                 ENB = Pin 6, IN3 = Pin 10, IN4 = Pin 11
//
// POLARITY: INVERT_DIR = true (fixes reversed TT motor wiring)
// =============================================================================

// ---------- Pins ----------
#define LEFT_TRIG_PIN  7
#define LEFT_ECHO_PIN  13
#define RIGHT_TRIG_PIN 12
#define RIGHT_ECHO_PIN 4

#define ENA 5
#define IN1 8
#define IN2 9
#define ENB 6
#define IN3 10
#define IN4 11

// ---------- Tuning ----------
const bool          INVERT_DIR      = false;  // Corrected native motor wiring
const bool          SWAP_SIDES      = false;  // true if left/right sides swapped
const unsigned int  OBSTACLE_CM     = 25;     // Detection threshold (cm)
const uint8_t       SPEED_FORWARD   = 200;    // Forward torque (>= 180)
const uint8_t       SPEED_TURN      = 210;    // Pivot turn torque (>= 180)
const uint8_t       SPEED_REVERSE   = 190;    // Backing up torque (>= 180)
const unsigned long LOOP_MS         = 60;     // Sensor loop interval
const unsigned long REVERSE_MS      = 400;    // Backup duration (ms)
const unsigned long BOTH_TURN_MS    = 450;    // Turn duration after backup (ms)
const unsigned long SENSOR_TIMEOUT  = 25000;  // 25ms timeout (~4m)
const uint8_t       CONFIRM_READS   = 2;      // Debounce noisy readings

uint8_t leftHits = 0, rightHits = 0;
bool isStopped = true; // Initialized in disarmed state at startup per Requirement R2

// ---------- Low-level motor control ----------
// Channel A: Right Motor (ENA=5, IN1=8, IN2=9). Forward: IN1=LOW, IN2=HIGH; Reverse: IN1=HIGH, IN2=LOW
void driveA(int s) {
  if (INVERT_DIR) s = -s;
  if (s > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (s < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(s));
}

// Channel B: Left Motor (ENB=6, IN3=10, IN4=11). Forward: IN3=LOW, IN4=HIGH; Reverse: IN3=HIGH, IN4=LOW
void driveB(int s) {
  if (INVERT_DIR) s = -s;
  if (s > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (s < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(s));
}

void setLeft(int speed) {
  int s = constrain(speed, -255, 255);
  if (SWAP_SIDES) { driveA(s); } else { driveB(s); }
}

void setRight(int speed) {
  int s = constrain(speed, -255, 255);
  if (SWAP_SIDES) { driveB(s); } else { driveA(s); }
}

// ---------- Movements ----------
void forward()   { setLeft(SPEED_FORWARD);  setRight(SPEED_FORWARD); }
void backward()  { setLeft(-SPEED_REVERSE); setRight(-SPEED_REVERSE); }
void stopAll()   { setLeft(0); setRight(0); }
void turnLeft()  { setLeft(-SPEED_TURN); setRight(SPEED_TURN); }
void turnRight() { setLeft(SPEED_TURN);  setRight(-SPEED_TURN); }

// ---------- Ultrasonic ----------
unsigned int readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, SENSOR_TIMEOUT);
  if (us == 0) return 999;
  return us / 58;
}

void setup() {
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopAll();
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=======================================================");
  Serial.println("  OBSTACLE AVOIDANCE SYSTEM READY (DISARMED at startup)");
  Serial.println("  Polarity: Forward is Left(IN3=L, IN4=H), Right(IN1=L, IN2=H)");
  Serial.println("  NOTICE FOR KRISHNA: Elevate wheels before testing!");
  Serial.println("  Type 'g' to ARM and GO, 's' to STOP / DISARM");
  Serial.println("=======================================================");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      isStopped = true;
      stopAll();
      Serial.println("\n🛑 >>> [STOP] MOTORS HALTED <<<");
    } else if (cmd == 'g' || cmd == 'G') {
      isStopped = false;
      Serial.println("\n🟢 >>> [RESUME] RUNNING <<<");
    }
  }

  if (isStopped) {
    stopAll();
    delay(50);
    return;
  }

  unsigned int dL = readCm(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  delay(25);
  unsigned int dR = readCm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  leftHits  = (dL < OBSTACLE_CM) ? min(leftHits + 1, 255)  : 0;
  rightHits = (dR < OBSTACLE_CM) ? min(rightHits + 1, 255) : 0;
  bool leftBlocked  = leftHits  >= CONFIRM_READS;
  bool rightBlocked = rightHits >= CONFIRM_READS;

  Serial.print("L: "); Serial.print(dL);
  Serial.print(" cm | R: "); Serial.print(dR);
  Serial.print(" cm  --> ");

  if (leftBlocked && rightBlocked) {
    Serial.println("🛑 BOTH BLOCKED: Backing up & turning");
    backward();
    delay(REVERSE_MS);
    if (dL >= dR) turnLeft(); else turnRight();
    delay(BOTH_TURN_MS);
    leftHits = rightHits = 0;
  } else if (rightBlocked) {
    Serial.println("↩️ OBSTACLE RIGHT: Steering LEFT");
    turnLeft();
  } else if (leftBlocked) {
    Serial.println("↪️ OBSTACLE LEFT: Steering RIGHT");
    turnRight();
  } else {
    Serial.println("🟢 CLEAR: Driving FORWARD");
    forward();
  }

  delay(LOOP_MS);
}
