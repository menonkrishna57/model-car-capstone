// --- Motor Pins (L298N) ---
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
const int ENA = 5;
const int ENB = 6;

// --- Ultrasonic Sensor Pins ---
const int TRIG_PIN = 2; // Front Ultrasonic Trigger
const int ECHO_PIN = 3; // Front Ultrasonic Echo

bool isStopped = true; // Initialized in disarmed state at startup per Requirement R2

void setup() {
  Serial.begin(115200);

  // Motor pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize motors stopped with zero PWM
  stopMotors();
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Ultrasonic pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println(F("=========================================="));
  Serial.println(F("Motor + Ultrasonic Safety Test Ready (DISARMED at startup)"));
  Serial.println(F("Direction Inversion Applied:"));
  Serial.println(F("  Forward: Right (IN1=LOW, IN2=HIGH), Left (IN3=LOW, IN4=HIGH)"));
  Serial.println(F("  Reverse: Right (IN1=HIGH, IN2=LOW), Left (IN3=HIGH, IN4=LOW)"));
  Serial.println(F("NOTICE FOR KRISHNA: Test on physical hardware with wheels elevated!"));
  Serial.println(F("Type 'g' in Serial Monitor to ARM and DRIVE FORWARD (180 PWM)."));
  Serial.println(F("Type 'r' in Serial Monitor to DRIVE REVERSE (180 PWM)."));
  Serial.println(F("Type 's' in Serial Monitor to STOP / DISARM."));
  Serial.println(F("=========================================="));
}

void loop() {
  // --- Check Serial Monitor for Manual Commands ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      isStopped = true;
      stopMotors();
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      Serial.println(F("[MANUAL OVERRIDE] Motors STOPPED / DISARMED by user."));
    } else if (cmd == 'g' || cmd == 'G') {
      isStopped = false;
      analogWrite(ENA, 180);
      analogWrite(ENB, 180);
      Serial.println(F("[RESUME] Motors ARMED / enabled at 180 PWM."));
    } else if (cmd == 'r' || cmd == 'R') {
      isStopped = false;
      analogWrite(ENA, 180);
      analogWrite(ENB, 180);
      driveReverse();
      Serial.println(F("[REVERSE] Driving REVERSE at 180 PWM."));
    }
  }

  // If user manually stopped the car or car is disarmed, do not spin motors
  if (isStopped) {
    stopMotors();
    delay(50);
    return;
  }

  // --- 1. Trigger Ultrasonic Pulse ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // --- 2. Measure Echo Pulse (cm) ---
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  long distance = duration * 0.034 / 2;

  // --- 3. Obstacle Check (< 25 cm) ---
  if (distance > 0 && distance < 25) {
    // Obstacle detected! Auto-Brake
    stopMotors();
    Serial.print(F("[OBSTACLE BLOCKED] Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm -> MOTORS STOPPED!"));
  } else {
    // Clear path: Drive forward
    driveForward();
  }

  delay(60); // Small 60ms delay between readings
}

void driveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void driveReverse() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
