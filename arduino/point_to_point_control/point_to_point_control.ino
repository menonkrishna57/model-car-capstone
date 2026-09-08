// =============================================================================
// point_to_point_control.ino — Turn-then-drive state machine for Arduino Uno
// Hardware: Arduino Uno + L298N (skid-steer 4WD)
// Communicates with Raspberry Pi 5 over Serial (9600 baud)
// Protocol: arduino/PROTOCOL.md
// =============================================================================

#include "../lib/pins.h"
#include "../lib/protocol.h"

enum CarState {
  STATE_IDLE,
  STATE_DRIVING,
  STATE_TURNING,
  STATE_REVERSING
};

CarState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long stateDuration = 0;

void setMotors(int leftSpeed, int rightSpeed) {
  // Left side direction & PWM
  if (leftSpeed > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    leftSpeed = -leftSpeed;
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  analogWrite(PIN_ENA, constrain(leftSpeed, 0, 255));

  // Right side direction & PWM
  if (rightSpeed > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    rightSpeed = -rightSpeed;
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  }
  analogWrite(PIN_ENB, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
  setMotors(0, 0);
  currentState = STATE_IDLE;
  Serial.print(TELEM_PREFIX);
  Serial.println("STATE=IDLE");
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  stopMotors();

  Serial.print(TELEM_PREFIX);
  Serial.println("READY Uno-ATmega328P");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("PING")) {
    Serial.print(ACK_PREFIX);
    Serial.println("PONG");
  } else if (cmd.startsWith("STOP")) {
    stopMotors();
    Serial.print(ACK_PREFIX);
    Serial.println("STOP");
  } else if (cmd.startsWith("DRIVE")) {
    // Format: DRIVE <speed> <duration_ms>
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0) {
      int speed = cmd.substring(firstSpace + 1, secondSpace).toInt();
      unsigned long duration = cmd.substring(secondSpace + 1).toInt();
      setMotors(speed, speed);
      currentState = STATE_DRIVING;
      stateStartTime = millis();
      stateDuration = duration;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
    }
  } else if (cmd.startsWith("REV")) {
    // Format: REV <speed> <duration_ms>
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0) {
      int speed = cmd.substring(firstSpace + 1, secondSpace).toInt();
      unsigned long duration = cmd.substring(secondSpace + 1).toInt();
      setMotors(-speed, -speed);
      currentState = STATE_REVERSING;
      stateStartTime = millis();
      stateDuration = duration;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
    }
  } else if (cmd.startsWith("TURN")) {
    // Format: TURN <angle_deg> <speed> (skid-steer)
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0) {
      int angle = cmd.substring(firstSpace + 1, secondSpace).toInt();
      int speed = cmd.substring(secondSpace + 1).toInt();
      // Rough skid turn: positive angle = turn right (left fwd, right rev)
      if (angle > 0) {
        setMotors(speed, -speed);
      } else {
        setMotors(-speed, speed);
      }
      currentState = STATE_TURNING;
      stateStartTime = millis();
      // Calibrated duration placeholder: abs(angle) * 10 ms
      stateDuration = (unsigned long)abs(angle) * 10;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
    }
  }
}

void loop() {
  // Check for serial commands from Pi 5
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // Handle timed state transitions
  if (currentState != STATE_IDLE) {
    if (millis() - stateStartTime >= stateDuration) {
      Serial.print(TELEM_PREFIX);
      Serial.println("DONE ACTION");
      stopMotors();
    }
  }
}
