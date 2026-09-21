// =============================================================================
// point_to_point_control.ino — Skid-Steer Control with Ultrasonic Avoidance
// Hardware: Arduino Uno (ATmega328P) + L298N (4WD skid-steer) + 2× HC-SR04P
// Communicates with Raspberry Pi 5 over Serial (9600 baud)
// Protocol: arduino/PROTOCOL.md
//
// Priority tiers (highest wins):
//   Tier 1 — Pi Camera Latch (PI_STOP / PI_GO)
//   Tier 2 — Local Ultrasonic Avoidance (reflex)
//   Tier 3 — Normal Commands (DRIVE / REV / TURN / STOP)
// =============================================================================

#include "../lib/pins.h"
#include "../lib/protocol.h"

// =============================================================================
// State Machine
// =============================================================================

enum CarState {
  STATE_IDLE,
  STATE_DRIVING,
  STATE_TURNING,
  STATE_REVERSING,
  STATE_AVOIDING,       // Local ultrasonic avoidance in progress
  STATE_PI_STOPPED      // Latched stop from Pi camera — frozen until PI_GO
};

// Sub-states within STATE_AVOIDING (reverse → turn → resume)
enum AvoidPhase {
  AVOID_REVERSE,        // Phase 1: back up
  AVOID_TURN,           // Phase 2: skid-turn away from obstacle
  AVOID_RESUME          // Phase 3: resume forward briefly
};

CarState currentState = STATE_IDLE;
unsigned long stateStartTime  = 0;
unsigned long stateDuration   = 0;

// Avoidance sub-state tracking
AvoidPhase   avoidPhase       = AVOID_REVERSE;
unsigned long avoidPhaseStart = 0;
int          avoidTurnDir     = 0;  // +1 = turn right, -1 = turn left

// Saved state before avoidance (to know what speed to resume at)
int savedDriveSpeed = 0;

// Pi avoid hint: +1 = right, -1 = left, 0 = no hint (Arduino decides locally)
int piAvoidHint = 0;

// Ultrasonic polling timer
unsigned long lastUsPollTime = 0;
float usLeftDistance  = 999.0f;
float usRightDistance = 999.0f;

// =============================================================================
// Motor Control (skid-steer L298N)
// =============================================================================

// Sonar polling
#define SONAR_INTERVAL_MS 50
unsigned long lastSonarTime = 0;

// Helper to read distance in cm (with 20ms timeout ~ 3.4m max)
long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return 100; // Timeout, assume clear (100cm)
  return duration * 0.034 / 2;
}

void pollSonar() {
  long dLeft = readDistanceCM(PIN_TRIG_LEFT, PIN_ECHO_LEFT);
  long dCenter = readDistanceCM(PIN_TRIG_CENTER, PIN_ECHO_CENTER);
  long dRight = readDistanceCM(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
  
  Serial.print(TELEM_PREFIX);
  Serial.print("SONAR ");
  Serial.print(dLeft);
  Serial.print(" ");
  Serial.print(dCenter);
  Serial.print(" ");
  Serial.println(dRight);
}

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
}

// Transition to IDLE and broadcast state
void goIdle() {
  stopMotors();
  currentState = STATE_IDLE;
  savedDriveSpeed = 0;
  Serial.print(TELEM_PREFIX);
  Serial.println("STATE=IDLE SPEED=0");
}

// =============================================================================
// Ultrasonic Distance Reading
// =============================================================================

float readUltrasonicCm(int trigPin, int echoPin) {
  // Send 10µs trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse width (timeout = US_TIMEOUT_US)
  unsigned long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);

  if (duration == 0) {
    // Timeout — no echo received, treat as max range (no obstacle)
    return 999.0f;
  }

  // Speed of sound ≈ 0.0343 cm/µs, round trip → divide by 2
  return (float)duration * 0.0343f / 2.0f;
}

// =============================================================================
// Avoidance Maneuver (Tier 2 — Spine Reflex)
// =============================================================================

void startAvoidance(int turnDirection) {
  // turnDirection: +1 = turn right (obstacle on left), -1 = turn left (obstacle on right)

  // If Pi has provided a directional hint, use it instead
  if (piAvoidHint != 0) {
    turnDirection = piAvoidHint;
    piAvoidHint = 0;  // Consume the hint (one-shot)
  }

  avoidTurnDir = turnDirection;
  avoidPhase = AVOID_REVERSE;
  avoidPhaseStart = millis();
  currentState = STATE_AVOIDING;

  // Phase 1: Reverse
  setMotors(-AVOID_SPEED, -AVOID_SPEED);

  // Broadcast which direction we're avoiding toward
  Serial.print(TELEM_PREFIX);
  Serial.print("AVOID ");
  Serial.println(avoidTurnDir > 0 ? "RIGHT" : "LEFT");
}

void updateAvoidance() {
  unsigned long elapsed = millis() - avoidPhaseStart;

  switch (avoidPhase) {
    case AVOID_REVERSE:
      if (elapsed >= AVOID_REVERSE_MS) {
        // Transition to Phase 2: Skid-turn
        avoidPhase = AVOID_TURN;
        avoidPhaseStart = millis();
        if (avoidTurnDir > 0) {
          // Turn right: left forward, right reverse
          setMotors(AVOID_SPEED, -AVOID_SPEED);
        } else {
          // Turn left: left reverse, right forward
          setMotors(-AVOID_SPEED, AVOID_SPEED);
        }
      }
      break;

    case AVOID_TURN:
      if (elapsed >= AVOID_TURN_MS) {
        // Transition to Phase 3: Resume forward driving
        avoidPhase = AVOID_RESUME;
        int resumeSpeed = (savedDriveSpeed > 0) ? savedDriveSpeed : AVOID_SPEED;
        setMotors(resumeSpeed, resumeSpeed);
        currentState = STATE_DRIVING;
        stateStartTime = millis();
        // Give 2 seconds of forward travel after avoidance
        stateDuration = 2000;
        Serial.print(TELEM_PREFIX);
        Serial.println("DONE AVOID");
      }
      break;

    default:
      break;
  }
}

// =============================================================================
// Command Handler (Pi 5 → Arduino)
// =============================================================================

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // ── Tier 1: Pi Camera Latch Commands ──────────────────────────────────────

  if (cmd.startsWith("PI_STOP")) {
    // Latched stop — overrides EVERYTHING, even mid-avoidance
    stopMotors();
    currentState = STATE_PI_STOPPED;
    savedDriveSpeed = 0;
    Serial.print(ACK_PREFIX);
    Serial.println("PI_STOP");
    Serial.print(TELEM_PREFIX);
    Serial.println("STATE=PI_STOPPED");
    return;
  }

  if (cmd.startsWith("PI_GO")) {
    // Release Pi latch — return to idle (clean slate)
    if (currentState == STATE_PI_STOPPED) {
      goIdle();
      Serial.print(ACK_PREFIX);
      Serial.println("PI_GO");
    } else {
      // Not in PI_STOPPED state, still acknowledge
      Serial.print(ACK_PREFIX);
      Serial.println("PI_GO IGNORED (not PI_STOPPED)");
    }
    return;
  }

  if (cmd.startsWith("PI_AVOID")) {
    // Directional hint for next avoidance maneuver
    // Format: PI_AVOID LEFT  or  PI_AVOID RIGHT
    if (cmd.indexOf("LEFT") >= 0) {
      piAvoidHint = -1;
    } else if (cmd.indexOf("RIGHT") >= 0) {
      piAvoidHint = 1;
    }
    Serial.print(ACK_PREFIX);
    Serial.println(cmd);
    return;
  }

  // ── Block all Tier 3 commands while Pi latch is active ────────────────────

  if (currentState == STATE_PI_STOPPED) {
    Serial.print(ACK_PREFIX);
    Serial.println("BLOCKED (PI_STOPPED)");
    return;
  }

  // ── Tier 3: Normal Driving Commands ───────────────────────────────────────

  if (cmd.startsWith("PING")) {
    Serial.print(ACK_PREFIX);
    Serial.println("PONG");

  } else if (cmd.startsWith("STOP")) {
    // Non-latched stop — immediate, goes to idle
    goIdle();
    Serial.print(ACK_PREFIX);
    Serial.println("STOP");

  } else if (cmd.startsWith("DRIVE")) {
    // Format: DRIVE <speed> <duration_ms>
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0) {
      int speed = cmd.substring(firstSpace + 1, secondSpace).toInt();
      unsigned long duration = cmd.substring(secondSpace + 1).toInt();
      speed = constrain(speed, 0, 255);
      setMotors(speed, speed);
      currentState = STATE_DRIVING;
      savedDriveSpeed = speed;
      stateStartTime = millis();
      stateDuration = duration;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
      Serial.print(TELEM_PREFIX);
      Serial.print("STATE=DRIVING SPEED=");
      Serial.println(speed);
    }

  } else if (cmd.startsWith("REV")) {
    // Format: REV <speed> <duration_ms>
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0) {
      int speed = cmd.substring(firstSpace + 1, secondSpace).toInt();
      unsigned long duration = cmd.substring(secondSpace + 1).toInt();
      speed = constrain(speed, 0, 255);
      setMotors(-speed, -speed);
      currentState = STATE_REVERSING;
      savedDriveSpeed = 0;
      stateStartTime = millis();
      stateDuration = duration;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
      Serial.print(TELEM_PREFIX);
      Serial.print("STATE=REVERSING SPEED=");
      Serial.println(speed);
    }

  } else if (cmd.startsWith("TURN")) {
    // Format: TURN <angle_deg> <speed> (skid-steer)
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0) {
      int angle = cmd.substring(firstSpace + 1, secondSpace).toInt();
      int speed = cmd.substring(secondSpace + 1).toInt();
      speed = constrain(speed, 0, 255);
      // Positive angle = turn right (left fwd, right rev)
      if (angle > 0) {
        setMotors(speed, -speed);
      } else {
        setMotors(-speed, speed);
      }
      currentState = STATE_TURNING;
      savedDriveSpeed = 0;
      stateStartTime = millis();
      // Calibrated duration placeholder: abs(angle) * 10 ms
      stateDuration = (unsigned long)abs(angle) * 10;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
      Serial.print(TELEM_PREFIX);
      Serial.println("STATE=TURNING");
    }
  }
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  // Motor pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Ultrasonic pins
  pinMode(PIN_US_LEFT_TRIG,  OUTPUT);
  pinMode(PIN_US_LEFT_ECHO,  INPUT);
  pinMode(PIN_US_RIGHT_TRIG, OUTPUT);
  pinMode(PIN_US_RIGHT_ECHO, INPUT);

  stopMotors();

  Serial.print(TELEM_PREFIX);
  Serial.println("READY Uno-ATmega328P");
}

// =============================================================================
// Main Loop — Priority-driven execution
// =============================================================================

void loop() {
  // ── 1. Read serial commands from Pi 5 (or Serial Monitor) ─────────────────

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

<<<<<<< HEAD
  // ── 2. Tier 1 check: If Pi latch is active, do nothing else ───────────────

  if (currentState == STATE_PI_STOPPED) {
    return;  // Frozen. Only PI_GO (via serial above) can release.
  }

  // ── 3. Tier 2: Ultrasonic polling & avoidance ─────────────────────────────

  // If currently in an avoidance maneuver, run the avoidance FSM
  if (currentState == STATE_AVOIDING) {
    updateAvoidance();
    return;  // Don't poll ultrasonics during an active avoidance maneuver
  }

  // Poll ultrasonics at regular intervals
  unsigned long now = millis();
  if (now - lastUsPollTime >= US_PING_INTERVAL_MS) {
    lastUsPollTime = now;

    usLeftDistance  = readUltrasonicCm(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
    usRightDistance = readUltrasonicCm(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);

    // Broadcast ultrasonic telemetry to Pi
    Serial.print(TELEM_PREFIX);
    Serial.print("US LEFT=");
    Serial.print(usLeftDistance, 1);
    Serial.print(" RIGHT=");
    Serial.println(usRightDistance, 1);

    // Check for obstacle — only trigger avoidance while actively moving
    if (currentState == STATE_DRIVING || currentState == STATE_REVERSING ||
        currentState == STATE_TURNING) {
      bool leftBlocked  = (usLeftDistance  < US_OBSTACLE_DISTANCE_CM);
      bool rightBlocked = (usRightDistance < US_OBSTACLE_DISTANCE_CM);

      if (leftBlocked && rightBlocked) {
        // Both sides blocked — reverse and turn right (arbitrary default)
        startAvoidance(1);
        return;
      } else if (leftBlocked) {
        // Obstacle on left — turn right
        startAvoidance(1);
        return;
      } else if (rightBlocked) {
        // Obstacle on right — turn left
        startAvoidance(-1);
        return;
      }
    }
  }

  // ── 4. Tier 3: Handle timed state transitions for normal commands ─────────

  if (currentState == STATE_DRIVING || currentState == STATE_TURNING ||
      currentState == STATE_REVERSING) {
=======
  // Poll ultrasonics at intervals
  if (millis() - lastSonarTime >= SONAR_INTERVAL_MS) {
    lastSonarTime = millis();
    pollSonar();
  }

  // Handle timed state transitions
  if (currentState != STATE_IDLE) {
>>>>>>> 65f0686ad95bd14e8d070b79ea17d2129212f935
    if (millis() - stateStartTime >= stateDuration) {
      Serial.print(TELEM_PREFIX);
      if (currentState == STATE_DRIVING) {
        Serial.println("DONE DRIVE");
      } else if (currentState == STATE_TURNING) {
        Serial.println("DONE TURN");
      } else {
        Serial.println("DONE REV");
      }
      goIdle();
    }
  }
}
