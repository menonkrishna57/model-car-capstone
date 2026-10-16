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

bool isStopped = false; // Flag for manual stop from Serial Monitor

void setup() {
  Serial.begin(115200);

  // Motor pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set motor speed (0-255)
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);

  Serial.println("==========================================");
  Serial.println("Motor + Ultrasonic Safety Test Ready!");
  Serial.println("Type 's' in Serial Monitor to STOP.");
  Serial.println("Type 'g' in Serial Monitor to GO again.");
  Serial.println("==========================================");
}

void loop() {
  // --- Check Serial Monitor for Manual Commands ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      isStopped = true;
      stopMotors();
      Serial.println("[MANUAL OVERRIDE] Motors STOPPED by user.");
    } else if (cmd == 'g' || cmd == 'G') {
      isStopped = false;
      Serial.println("[RESUME] Motors re-enabled.");
    }
  }

  // If user manually stopped the car, do not spin motors
  if (isStopped) {
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
    Serial.print("[OBSTACLE BLOCKED] Distance: ");
    Serial.print(distance);
    Serial.println(" cm -> MOTORS STOPPED!");
  } else {
    // Clear path: Drive forward
    driveForward();
  }

  delay(60); // Small 60ms delay between readings
}

void driveForward() {
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
