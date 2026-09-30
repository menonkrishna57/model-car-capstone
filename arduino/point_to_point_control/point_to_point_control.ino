// =============================================================================
// point_to_point_control.ino â€” Autonomous RC Car Arduino Firmware
// ================================================================
// Hardware:  Arduino Uno (ATmega328P) + L298N 4WD skid-steer + 2Ã— HC-SR04P
// Pi Link:   Serial 115200 baud (USB-Serial to Raspberry Pi 5)
// Protocol:  arduino/PROTOCOL.md  (source of truth)
// Pins:      arduino/lib/pins.h   (source of truth)
// Constants: arduino/lib/protocol.h (source of truth)
//
// Three-Tier Priority Stack (highest wins):
//   Tier 1 â€” PI_STOP / PI_GO / PI_AVOID  (Pi camera latch â€” overrides everything)
//   Tier 2 â€” Ultrasonic avoidance reflex  (autonomous, no Pi needed)
//   Tier 3 â€” DRIVE / REV / TURN / STOP / PING  (Pi commands, normal driving)
//
// State machine:
//   IDLE â†’ DRIVING / TURNING / REVERSING  (via Tier 3 commands)
//   DRIVING / TURNING / REVERSING â†’ AVOIDING  (Tier 2 trigger, < 20cm)
//   ANY â†’ PI_STOPPED  (PI_STOP from Pi)
//   PI_STOPPED â†’ IDLE  (PI_GO from Pi)
//   AVOIDING â†’ DRIVING  (Phase 3 resume forward)
//   DRIVING / TURNING / REVERSING â†’ IDLE  (timer expires)
// =============================================================================

#include "pins.h"
#include "protocol.h"

// =============================================================================
// State Machine Definitions
// =============================================================================

enum CarState {
  STATE_IDLE,
  STATE_DRIVING,
  STATE_TURNING,
  STATE_REVERSING,
  STATE_AVOIDING,       // Tier 2 ultrasonic avoidance in progress
  STATE_PI_STOPPED      // Tier 1 latch â€” frozen until PI_GO
};

// Avoidance sub-phases (Tier 2): Reverse â†’ Skid-turn â†’ Resume forward
enum AvoidPhase {
  AVOID_PHASE_REVERSE,
  AVOID_PHASE_TURN,
  AVOID_PHASE_RESUME
};

// =============================================================================
// Global State Variables
// =============================================================================

CarState     currentState    = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long stateDuration  = 0;

// Avoidance sub-state
AvoidPhase   avoidPhase      = AVOID_PHASE_REVERSE;
unsigned long avoidPhaseStart = 0;
int          avoidTurnDir    = 1;    // +1 = turn right, -1 = turn left

// Speed at the time avoidance was triggered (restored in Phase 3 if > 0)
int savedDriveSpeed = 0;

// PI_AVOID one-shot hint: +1 = right, -1 = left, 0 = no hint
int piAvoidHint = 0;

// Ultrasonic polling
unsigned long lastUsPollTime   = 0;
float         usLeftDistance   = 999.0f;
float         usRightDistance  = 999.0f;

// Avoidance cooldown (suppresses re-trigger during Phase 3 resume)
unsigned long avoidCooldownEnd = 0;

// Serial watchdog (Pi silence detection)
unsigned long lastPiRxTime     = 0;
bool          watchdogArmed    = false;  // Only arm once we've received at least one Pi command

// Serial input buffer
char serialBuf[SERIAL_BUFFER_SIZE];
int  serialBufLen = 0;

// =============================================================================
// Motor Control â€” Skid-Steer L298N
// =============================================================================

/**
 * Set individual left/right motor speeds.
 * Positive value = forward, negative = reverse, 0 = stop.
 * Range: -255 to +255.
 */
void setMotors(int leftSpeed, int rightSpeed) {
  // â”€â”€ Left channel (Channel A) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

  // â”€â”€ Right channel (Channel B) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

/** Immediately cuts all motor output. */
void stopMotors() {
  setMotors(0, 0);
}

/** Transition to IDLE: stop motors, reset speed, emit telemetry. */
void goIdle() {
  stopMotors();
  currentState    = STATE_IDLE;
  savedDriveSpeed = 0;
  Serial.print(TELEM_PREFIX);
  Serial.println("STATE=IDLE SPEED=0");
}

// =============================================================================
// Ultrasonic Distance Measurement
// =============================================================================

/**
 * Reads distance in cm from one HC-SR04P sensor.
 * Returns 999.0 on timeout (no echo = open space beyond max range).
 *
 * ATmega328P note: pulseIn() is blocking. At 115200 baud with 100ms poll
 * interval, each call blocks < 30ms (US_TIMEOUT_US = 30000Âµs). Two
 * sensors per cycle = < 60ms worst case, well within the 100ms budget.
 */
float readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (duration == 0) {
    return 999.0f;   // Timeout â†’ no obstacle in range
  }
  // Speed of sound â‰ˆ 0.0343 cm/Âµs; round-trip so divide by 2
  return (float)duration * 0.0343f / 2.0f;
}

// =============================================================================
// Tier 2 â€” Avoidance Maneuver (Spine Reflex)
// =============================================================================

/**
 * Initiates a 3-phase avoidance maneuver.
 * turnDirection: +1 = turn right (away from left obstacle),
 *                -1 = turn left  (away from right obstacle).
 *
 * If a PI_AVOID hint is stored, that direction overrides the local decision
 * (one-shot â€” consumed immediately).
 */
void startAvoidance(int turnDirection) {
  if (piAvoidHint != 0) {
    turnDirection = piAvoidHint;
    piAvoidHint   = 0;  // Consume hint
  }

  avoidTurnDir  = turnDirection;
  avoidPhase    = AVOID_PHASE_REVERSE;
  avoidPhaseStart = millis();
  currentState  = STATE_AVOIDING;

  // Phase 1: Reverse at AVOID_SPEED
  setMotors(-AVOID_SPEED, -AVOID_SPEED);

  Serial.print(TELEM_PREFIX);
  Serial.print("AVOID ");
  Serial.println(avoidTurnDir > 0 ? "RIGHT" : "LEFT");
}

/**
 * Advances the avoidance state machine each loop iteration.
 * Called only while currentState == STATE_AVOIDING.
 */
void updateAvoidance() {
  unsigned long elapsed = millis() - avoidPhaseStart;

  switch (avoidPhase) {

    case AVOID_PHASE_REVERSE:
      if (elapsed >= AVOID_REVERSE_MS) {
        // â†’ Phase 2: Skid-turn
        avoidPhase    = AVOID_PHASE_TURN;
        avoidPhaseStart = millis();
        if (avoidTurnDir > 0) {
          setMotors( AVOID_SPEED, -AVOID_SPEED);  // Right: left fwd, right rev
        } else {
          setMotors(-AVOID_SPEED,  AVOID_SPEED);  // Left: left rev, right fwd
        }
      }
      break;

    case AVOID_PHASE_TURN:
      if (elapsed >= AVOID_TURN_MS) {
        // â†’ Phase 3: Resume forward
        avoidPhase    = AVOID_PHASE_RESUME;
        avoidPhaseStart = millis();
        int resumeSpeed = (savedDriveSpeed > 0) ? savedDriveSpeed : AVOID_SPEED;
        setMotors(resumeSpeed, resumeSpeed);
        // Transition to DRIVING so timed-state machinery handles DONE DRIVE
        currentState  = STATE_DRIVING;
        stateStartTime = millis();
        stateDuration  = AVOID_RESUME_MS;
        // Arm cooldown so ultrasonics don't re-trigger immediately
        avoidCooldownEnd = millis() + AVOID_COOLDOWN_MS;
        Serial.print(TELEM_PREFIX);
        Serial.println("DONE AVOID");
        Serial.print(TELEM_PREFIX);
        Serial.print("STATE=DRIVING SPEED=");
        Serial.println(resumeSpeed);
      }
      break;

    case AVOID_PHASE_RESUME:
      // Phase 3 is managed by the normal timed-state machinery
      // (currentState = STATE_DRIVING with stateDuration = AVOID_RESUME_MS).
      // Nothing extra to do here.
      break;
  }
}

// =============================================================================
// Tier 3 â€” Command Handler (Pi 5 â†’ Arduino)
// =============================================================================

/**
 * Parses and dispatches a single ASCII command line from the Pi 5.
 * Commands are newline-terminated, whitespace-trimmed.
 */
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Reset watchdog timer on any incoming Pi message
  lastPiRxTime  = millis();
  watchdogArmed = true;

  // â”€â”€ Tier 1: Pi Camera Latch â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

  if (cmd.startsWith("PI_STOP")) {
    // Latched stop â€” overrides EVERYTHING including mid-avoidance
    stopMotors();
    currentState    = STATE_PI_STOPPED;
    savedDriveSpeed = 0;
    avoidCooldownEnd = 0;  // Cancel cooldown
    Serial.print(ACK_PREFIX);
    Serial.println("PI_STOP");
    Serial.print(TELEM_PREFIX);
    Serial.println("STATE=PI_STOPPED");
    return;
  }

  if (cmd.startsWith("PI_GO")) {
    if (currentState == STATE_PI_STOPPED) {
      goIdle();
      Serial.print(ACK_PREFIX);
      Serial.println("PI_GO");
    } else {
      // Not in PI_STOPPED â€” acknowledge but ignore (idempotent)
      Serial.print(ACK_PREFIX);
      Serial.println("PI_GO IGNORED (not PI_STOPPED)");
    }
    return;
  }

  if (cmd.startsWith("PI_AVOID")) {
    // Format: PI_AVOID LEFT  or  PI_AVOID RIGHT
    // One-shot hint; last hint wins if multiple are sent.
    if (cmd.indexOf("LEFT") >= 0) {
      piAvoidHint = -1;
    } else if (cmd.indexOf("RIGHT") >= 0) {
      piAvoidHint = 1;
    }
    Serial.print(ACK_PREFIX);
    Serial.println(cmd);
    return;
  }

  // â”€â”€ Block Tier 3 commands while Pi latch is active â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

  if (currentState == STATE_PI_STOPPED) {
    Serial.print(ACK_PREFIX);
    Serial.println("BLOCKED (PI_STOPPED)");
    return;
  }

  // â”€â”€ Tier 3: Normal Driving Commands â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

  if (cmd.startsWith("PING")) {
    Serial.print(ACK_PREFIX);
    Serial.println("PONG");

  } else if (cmd.startsWith("STOP")) {
    // Non-latched stop â€” immediate, returns to IDLE
    goIdle();
    Serial.print(ACK_PREFIX);
    Serial.println("STOP");

  } else if (cmd.startsWith("DRIVE")) {
    // Format: DRIVE <speed> <duration_ms>
    // A new DRIVE replaces any current timed action immediately (no queue).
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > 0) {
      int          speed    = constrain(cmd.substring(sp1 + 1, sp2).toInt(), 0, 255);
      unsigned long duration = (unsigned long)cmd.substring(sp2 + 1).toInt();
      setMotors(speed, speed);
      currentState    = STATE_DRIVING;
      savedDriveSpeed = speed;
      stateStartTime  = millis();
      stateDuration   = duration;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
      Serial.print(TELEM_PREFIX);
      Serial.print("STATE=DRIVING SPEED=");
      Serial.println(speed);
    }

  } else if (cmd.startsWith("REV")) {
    // Format: REV <speed> <duration_ms>
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > 0) {
      int          speed    = constrain(cmd.substring(sp1 + 1, sp2).toInt(), 0, 255);
      unsigned long duration = (unsigned long)cmd.substring(sp2 + 1).toInt();
      setMotors(-speed, -speed);
      currentState    = STATE_REVERSING;
      savedDriveSpeed = 0;  // REV doesn't set a forward resume speed
      stateStartTime  = millis();
      stateDuration   = duration;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
      Serial.print(TELEM_PREFIX);
      Serial.print("STATE=REVERSING SPEED=");
      Serial.println(speed);
    }

  } else if (cmd.startsWith("TURN")) {
    // Format: TURN <angle_deg> <speed>
    // Positive angle = turn right (left fwd / right rev).
    // Negative angle = turn left  (left rev / right fwd).
    // Duration is derived from TURN_MS_PER_DEGREE (calibrate on real chassis).
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > 0) {
      int angle = cmd.substring(sp1 + 1, sp2).toInt();
      int speed = constrain(cmd.substring(sp2 + 1).toInt(), 0, 255);
      if (angle > 0) {
        setMotors( speed, -speed);   // Turn right: left fwd, right rev
      } else {
        setMotors(-speed,  speed);   // Turn left: left rev, right fwd
      }
      currentState    = STATE_TURNING;
      savedDriveSpeed = 0;
      stateStartTime  = millis();
      stateDuration   = (unsigned long)abs(angle) * TURN_MS_PER_DEGREE;
      Serial.print(ACK_PREFIX);
      Serial.println(cmd);
      Serial.print(TELEM_PREFIX);
      Serial.println("STATE=TURNING");
    }
  }
  // Unknown command: silently ignore (no ACK noise for malformed lines)
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  // Motor output pins
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

  // IR proximity sensors (reserved; may be scrapped)
  pinMode(PIN_IR1, INPUT);
  pinMode(PIN_IR2, INPUT);

  // Ensure all trigger lines start LOW (HC-SR04P spec)
  digitalWrite(PIN_US_LEFT_TRIG,  LOW);
  digitalWrite(PIN_US_RIGHT_TRIG, LOW);

  stopMotors();

  lastPiRxTime = millis();  // Initialise watchdog clock

  // Boot announcement â€” Pi SensorManager listens for this line
  Serial.print(TELEM_PREFIX);
  Serial.println("READY Uno-ATmega328P");
}

// =============================================================================
// Main Loop â€” Priority-Driven Execution
// =============================================================================

void loop() {

  // â”€â”€ 1. Read serial command from Pi 5 (or Serial Monitor for bench tests) â”€â”€
  //
  // Use a manual byte-by-byte buffer instead of Serial.readStringUntil() so
  // that the loop is non-blocking. Serial.readStringUntil() has an internal
  // timeout that stalls the MCU and can delay ultrasonic polling.

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBufLen > 0) {
        serialBuf[serialBufLen] = '\0';
        handleCommand(String(serialBuf));
        serialBufLen = 0;
      }
    } else if (serialBufLen < SERIAL_BUFFER_SIZE - 1) {
      serialBuf[serialBufLen++] = c;
    }
    // Overflow: silently discard extra characters; buffer resets on next '\n'
  }

  // â”€â”€ 2. Tier 1 check: If Pi latch active, motors stay frozen â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

  if (currentState == STATE_PI_STOPPED) {
    // Serial watchdog is deliberately not active while PI_STOPPED â€”
    // the Pi intentionally latched us; we shouldn't self-release.
    return;
  }

  // â”€â”€ 3. Serial Watchdog: safe-stop if Pi goes silent while driving â”€â”€â”€â”€â”€â”€â”€â”€â”€

#if WATCHDOG_TIMEOUT_MS > 0
  if (watchdogArmed &&
      (currentState == STATE_DRIVING || currentState == STATE_TURNING || currentState == STATE_REVERSING)) {
    if (millis() - lastPiRxTime >= WATCHDOG_TIMEOUT_MS) {
      goIdle();
      Serial.print(TELEM_PREFIX);
      Serial.println("HEARTBEAT_TIMEOUT");
      watchdogArmed = false;  // Disarm until next Pi command
      return;
    }
  }
#endif

  // â”€â”€ 4. Tier 2: Ultrasonic polling & avoidance â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

  // If currently executing an avoidance maneuver, advance the FSM
  if (currentState == STATE_AVOIDING) {
    updateAvoidance();
    return;  // Don't poll sensors while avoidance FSM runs (avoid re-entry)
  }

  unsigned long now = millis();
  if (now - lastUsPollTime >= US_PING_INTERVAL_MS) {
    lastUsPollTime = now;

    usLeftDistance  = readUltrasonicCm(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
    usRightDistance = readUltrasonicCm(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);

    // Broadcast telemetry â€” format matches PROTOCOL.md Â§3
    Serial.print(TELEM_PREFIX);
    Serial.print("US LEFT=");
    Serial.print(usLeftDistance, 1);
    Serial.print(" RIGHT=");
    Serial.println(usRightDistance, 1);

    // Trigger avoidance only while actively moving and outside cooldown window
    bool coolingDown = (now < avoidCooldownEnd);
    if (!coolingDown &&
        (currentState == STATE_DRIVING || currentState == STATE_TURNING || currentState == STATE_REVERSING)) {

      bool leftBlocked  = (usLeftDistance  < US_OBSTACLE_DISTANCE_CM);
      bool rightBlocked = (usRightDistance < US_OBSTACLE_DISTANCE_CM);

      if (leftBlocked && rightBlocked) {
        // Both blocked: default turn right (arbitrary), Pi_AVOID hint overrides
        startAvoidance(1);
        return;
      } else if (leftBlocked) {
        // Obstacle on left â†’ turn right to avoid
        startAvoidance(1);
        return;
      } else if (rightBlocked) {
        // Obstacle on right â†’ turn left to avoid
        startAvoidance(-1);
        return;
      }
    }
  }

  // â”€â”€ 5. Tier 3: Timed-state expiry for normal driving commands â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

  if (currentState == STATE_DRIVING || currentState == STATE_TURNING || currentState == STATE_REVERSING) {
    if (millis() - stateStartTime >= stateDuration) {
      // Emit the appropriate DONE message before going idle
      Serial.print(TELEM_PREFIX);
      if      (currentState == STATE_DRIVING)   Serial.println("DONE DRIVE");
      else if (currentState == STATE_TURNING)   Serial.println("DONE TURN");
      else if (currentState == STATE_REVERSING) Serial.println("DONE REV");
      goIdle();
    }
  }
}
