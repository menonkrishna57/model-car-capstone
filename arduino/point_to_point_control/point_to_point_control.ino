// =============================================================================
// point_to_point_control.ino — Autonomous RC Car Arduino Firmware v2.0
// =============================================================================
// Hardware:  Arduino Uno (ATmega328P) + L298N 4WD skid-steer + 2× HC-SR04P
// Pi Link:   Serial 115200 baud (USB-Serial or UART to Raspberry Pi 5)
// Protocol:  arduino/PROTOCOL.md  (source of truth)
// Pins:      pins.h / arduino/lib/pins.h
// Constants: protocol.h / arduino/lib/protocol.h
//
// Multi-Tier Priority Stack (highest wins):
//   Tier 1   — PI_STOP / PI_GO / PI_AVOID (Pi camera latch — overrides everything)
//   Tier 1.5 — Cliff Detection Reflex     (angled HC-SR04P negative obstacle)
//              Action: Immediate reverse burst, then latch until PI_GO
//   Tier 2   — Ultrasonic Avoidance Reflex (autonomous obstacle avoidance)
//              Action: Reverse → Turn → Resume forward
//   Tier 3   — DRIVE / REV / TURN / STOP / PING (Normal driving commands)
//   Meta     — TUNE GET / TUNE SET / TUNE RECAL (Runtime parameter tuning)
//
// Features:
//   - Boot-time auto-calibration of ground baseline for tilted ultrasonic sensors
//   - Auto-adjustment of obstacle threshold if baseline < default threshold
//   - 2-reading cliff confirmation filter to suppress sensor noise
//   - Runtime parameter tuning without reflashing via TUNE commands
//   - Safe SRAM usage with F() macros for all serial string literals
// =============================================================================

#include <EEPROM.h>
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
  STATE_AVOIDING,          // Tier 2 ultrasonic obstacle avoidance
  STATE_CLIFF_REVERSING,   // Tier 1.5 backing away from detected drop-off
  STATE_CLIFF_STOPPED,     // Tier 1.5 latched stop (requires PI_GO to release)
  STATE_PI_STOPPED         // Tier 1 latched stop from Pi camera (requires PI_GO)
};

// Avoidance sub-phases (Tier 2): Reverse → Skid-turn → Resume forward
enum AvoidPhase {
  AVOID_PHASE_REVERSE,
  AVOID_PHASE_TURN,
  AVOID_PHASE_RESUME
};

// =============================================================================
// Runtime Tunable Parameters Struct & Persistence
// =============================================================================

struct TunableParams {
  float         usObstacleDist;     // Obstacle trigger distance (cm)
  unsigned long usPollMs;           // Ultrasonic poll interval (ms)
  float         cliffMargin;        // Cliff trigger margin above ground baseline (cm)
  float         cliffBaselineL;     // Left ground baseline (cm)
  float         cliffBaselineR;     // Right ground baseline (cm)
  unsigned long cliffRevMs;         // Cliff reverse burst duration (ms)
  int           cliffRevSpeed;      // Cliff reverse PWM speed (0–255)
  int           cliffConfirmCount;  // Consecutive readings needed to confirm cliff
  int           avoidSpeed;         // Avoidance PWM speed (0–255)
  unsigned long avoidRevMs;         // Avoidance phase 1 reverse duration (ms)
  unsigned long avoidTurnMs;        // Avoidance phase 2 turn duration (ms)
  unsigned long avoidResumeMs;      // Avoidance phase 3 forward duration (ms)
  unsigned long avoidCooldownMs;    // Avoidance re-trigger cooldown (ms)
  unsigned long watchdogMs;         // Serial watchdog timeout (ms, 0 = disabled)
  int           turnMsPerDeg;       // Skid-turn duration per degree (ms/deg)
  int           motorTrim;          // Motor bias trim (-50 to +50; >0 right faster, <0 left faster)
  int           telemStream;        // 1 = stream periodic US telemetry, 0 = silent (events and STATUS only)
  int           cliffEnable;        // 1 = cliff reflex enabled (table/drop-offs), 0 = disabled (floor driving)
};

TunableParams tune = {
  US_OBSTACLE_DISTANCE_CM,          // usObstacleDist
  US_PING_INTERVAL_MS,              // usPollMs
  CLIFF_MARGIN_CM,                  // cliffMargin
  CLIFF_BASELINE_DEFAULT_CM,        // cliffBaselineL (updated during setup)
  CLIFF_BASELINE_DEFAULT_CM,        // cliffBaselineR (updated during setup)
  CLIFF_REVERSE_MS,                 // cliffRevMs
  CLIFF_REVERSE_SPEED,              // cliffRevSpeed
  CLIFF_CONFIRM_COUNT,              // cliffConfirmCount
  AVOID_SPEED,                      // avoidSpeed
  AVOID_REVERSE_MS,                 // avoidRevMs
  AVOID_TURN_MS,                    // avoidTurnMs
  AVOID_RESUME_MS,                  // avoidResumeMs
  AVOID_COOLDOWN_MS,                // avoidCooldownMs
  WATCHDOG_TIMEOUT_MS,              // watchdogMs
  TURN_MS_PER_DEGREE,               // turnMsPerDeg
  MOTOR_TRIM_DEFAULT,               // motorTrim
  TELEM_STREAM_DEFAULT,             // telemStream
  CLIFF_ENABLE_DEFAULT              // cliffEnable
};

struct EEPROMHeader {
  uint16_t magic;
  uint16_t version;
};

// =============================================================================
// Global State Variables
// =============================================================================

CarState      currentState        = STATE_IDLE;
unsigned long stateStartTime      = 0;
unsigned long stateDuration       = 0;

// Avoidance sub-state (Tier 2)
AvoidPhase    avoidPhase          = AVOID_PHASE_REVERSE;
unsigned long avoidPhaseStart     = 0;
int           avoidTurnDir        = 1;    // +1 = turn right, -1 = turn left
int           savedDriveSpeed     = 0;

// PI_AVOID directional hint: +1 = right, -1 = left, 0 = none
int           piAvoidHint         = 0;

// Ultrasonic polling & readings
unsigned long lastUsPollTime      = 0;
float         usLeftDistance      = 999.0f;
float         usRightDistance     = 999.0f;
int           cliffCountL         = 0;     // Consecutive cliff readings on left sensor
int           cliffCountR         = 0;     // Consecutive cliff readings on right sensor

// Avoidance cooldown
unsigned long avoidCooldownEnd    = 0;

// Serial watchdog
unsigned long lastPiRxTime        = 0;
bool          watchdogArmed       = false;

// Serial input buffer
char          serialBuf[SERIAL_BUFFER_SIZE];
int           serialBufLen        = 0;

// =============================================================================
// Motor Control — Skid-Steer L298N
// =============================================================================

/**
 * Set individual left/right motor speeds.
 * Positive value = forward, negative = reverse, 0 = stop.
 * Range: -255 to +255.
 */
void setMotors(int leftSpeed, int rightSpeed) {
  // Invert polarity if chassis TT motors are wired in reverse (corrects backward drive)
  if (INVERT_MOTOR_POLARITY) {
    leftSpeed  = -leftSpeed;
    rightSpeed = -rightSpeed;
  }

  // Apply motor trim when driving forward (corrects chassis mechanical drift)
  if (leftSpeed > 0 && rightSpeed > 0 && tune.motorTrim != 0) {
    if (tune.motorTrim > 0) {
      rightSpeed = constrain(rightSpeed + tune.motorTrim, 0, 255);
    } else {
      leftSpeed = constrain(leftSpeed - tune.motorTrim, 0, 255);
    }
  }

  // Left channel (Channel B: ENB=6, IN3=10, IN4=11)
  if (leftSpeed > 0) {
    digitalWrite(PIN_LEFT_IN1, HIGH);
    digitalWrite(PIN_LEFT_IN2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, HIGH);
    leftSpeed = -leftSpeed;
  } else {
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, LOW);
    leftSpeed = 0;
  }
  analogWrite(PIN_LEFT_PWM, constrain(leftSpeed, 0, 255));

  // Right channel (Channel A: ENA=5, IN1=8, IN2=9)
  if (rightSpeed > 0) {
    digitalWrite(PIN_RIGHT_IN1, HIGH);
    digitalWrite(PIN_RIGHT_IN2, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(PIN_RIGHT_IN1, LOW);
    digitalWrite(PIN_RIGHT_IN2, HIGH);
    rightSpeed = -rightSpeed;
  } else {
    digitalWrite(PIN_RIGHT_IN1, LOW);
    digitalWrite(PIN_RIGHT_IN2, LOW);
    rightSpeed = 0;
  }
  analogWrite(PIN_RIGHT_PWM, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
  setMotors(0, 0);
}

// =============================================================================
// Ultrasonic Sensor Reading
// =============================================================================

/**
 * Trigger HC-SR04P and read echo duration. Returns distance in cm.
 * Returns 999.0f on timeout (no reflection / far away / drop-off).
 */
float readUltrasonicRaw(int trigPin, int echoPin) {
  // If the echo pin is latched/stuck HIGH, briefly pulse it LOW to unstick the sensor
  if (digitalRead(echoPin) == HIGH) {
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(10);
    pinMode(echoPin, INPUT);
    delayMicroseconds(5);
  }

  // Clean trigger pulse (10-12µs HIGH)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);

  unsigned long durationUs = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (durationUs == 0) {
    return 999.0f; // Timeout or out of range
  }
  return (float)durationUs * 0.0343f / 2.0f;
}

float readUltrasonic(int trigPin, int echoPin) {
  float dist = readUltrasonicRaw(trigPin, echoPin);
  // Anti-glitch: if a single ping missed the table surface, retry once after 3ms
  if (dist >= 990.0f) {
    delay(3);
    dist = readUltrasonicRaw(trigPin, echoPin);
  }
  return dist;
}

// =============================================================================
// Cliff Baseline Auto-Calibration
// =============================================================================

/**
 * Calibrates ground baseline at boot or on TUNE RECAL request.
 * Takes multiple samples, averages valid readings, and sanity checks.
 * Also auto-adjusts obstacle threshold if baseline < obstacle threshold.
 */
void calibrateCliffBaselines() {
  Serial.println(F("[TELEM] CAL_CLIFF STARTING"));

  float sumL = 0.0f;
  float sumR = 0.0f;
  int countL = 0;
  int countR = 0;

  for (int i = 0; i < CLIFF_CAL_SAMPLES; i++) {
    float dl = readUltrasonic(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
    delay(20);
    float dr = readUltrasonic(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);

    if (dl > 0.0f && dl < 100.0f) {
      sumL += dl;
      countL++;
    }
    if (dr > 0.0f && dr < 100.0f) {
      sumR += dr;
      countR++;
    }
    delay(CLIFF_CAL_DELAY_MS);
  }

  // Evaluate left sensor baseline
  if (countL >= 3) {
    float avgL = sumL / (float)countL;
    if (avgL >= CLIFF_CAL_MIN_CM && avgL <= CLIFF_CAL_MAX_CM) {
      tune.cliffBaselineL = avgL;
    } else {
      tune.cliffBaselineL = CLIFF_BASELINE_DEFAULT_CM;
    }
  } else {
    tune.cliffBaselineL = CLIFF_BASELINE_DEFAULT_CM;
  }

  // Evaluate right sensor baseline
  if (countR >= 3) {
    float avgR = sumR / (float)countR;
    if (avgR >= CLIFF_CAL_MIN_CM && avgR <= CLIFF_CAL_MAX_CM) {
      tune.cliffBaselineR = avgR;
    } else {
      tune.cliffBaselineR = CLIFF_BASELINE_DEFAULT_CM;
    }
  } else {
    tune.cliffBaselineR = CLIFF_BASELINE_DEFAULT_CM;
  }

  // Ground baseline check: When angled down, if default obstacle threshold
  // is greater than or equal to ground distance, adjust it to 60% of baseline
  // so the ground surface is never mistaken for an obstacle!
  float minBaseline = min(tune.cliffBaselineL, tune.cliffBaselineR);
  if (tune.usObstacleDist >= minBaseline) {
    tune.usObstacleDist = minBaseline * 0.60f;
  }

  Serial.print(F("[TELEM] CAL_CLIFF BASE_L="));
  Serial.print(tune.cliffBaselineL, 1);
  Serial.print(F(" BASE_R="));
  Serial.print(tune.cliffBaselineR, 1);
  Serial.print(F(" OBS_THRESH="));
  Serial.println(tune.usObstacleDist, 1);
}

// =============================================================================
// State Transitions & Helper Functions
// =============================================================================

void transitionTo(CarState newState, unsigned long durationMs = 0) {
  currentState   = newState;
  stateStartTime = millis();
  stateDuration  = durationMs;
}

const char* stateToString(CarState st) {
  switch (st) {
    case STATE_IDLE:            return "IDLE";
    case STATE_DRIVING:         return "DRIVING";
    case STATE_TURNING:         return "TURNING";
    case STATE_REVERSING:       return "REVERSING";
    case STATE_AVOIDING:        return "AVOIDING";
    case STATE_CLIFF_REVERSING: return "CLIFF_REVERSING";
    case STATE_CLIFF_STOPPED:   return "CLIFF_STOPPED";
    case STATE_PI_STOPPED:      return "PI_STOPPED";
    default:                    return "UNKNOWN";
  }
}

// =============================================================================
// Avoidance FSM (Tier 2 Obstacle Avoidance)
// =============================================================================

void startAvoidance() {
  stopMotors();

  // Direction: Pi hint takes precedence; otherwise steer away from closer obstacle
  if (piAvoidHint != 0) {
    avoidTurnDir = piAvoidHint;
    piAvoidHint  = 0;
  } else {
    avoidTurnDir = (usLeftDistance < usRightDistance) ? 1 : -1;
  }

  Serial.print(F("[TELEM] AVOID "));
  Serial.println((avoidTurnDir > 0) ? F("RIGHT") : F("LEFT"));

  avoidPhase      = AVOID_PHASE_REVERSE;
  avoidPhaseStart = millis();
  transitionTo(STATE_AVOIDING);

  // Phase 1: Reverse burst
  setMotors(-tune.avoidSpeed, -tune.avoidSpeed);
}

void tickAvoidance() {
  unsigned long elapsed = millis() - avoidPhaseStart;

  switch (avoidPhase) {
    case AVOID_PHASE_REVERSE:
      if (elapsed >= tune.avoidRevMs) {
        avoidPhase      = AVOID_PHASE_TURN;
        avoidPhaseStart = millis();
        // Phase 2: Skid-turn
        if (avoidTurnDir > 0) {
          setMotors(tune.avoidSpeed, -tune.avoidSpeed);  // Turn Right
        } else {
          setMotors(-tune.avoidSpeed, tune.avoidSpeed);  // Turn Left
        }
      }
      break;

    case AVOID_PHASE_TURN:
      if (elapsed >= tune.avoidTurnMs) {
        avoidPhase      = AVOID_PHASE_RESUME;
        avoidPhaseStart = millis();
        // Phase 3: Resume forward
        int resumeSpd = (savedDriveSpeed > 0) ? savedDriveSpeed : tune.avoidSpeed;
        setMotors(resumeSpd, resumeSpd);
      }
      break;

    case AVOID_PHASE_RESUME:
      if (elapsed >= tune.avoidResumeMs) {
        stopMotors();
        avoidCooldownEnd = millis() + tune.avoidCooldownMs;
        Serial.println(F("[TELEM] DONE AVOID"));
        transitionTo(STATE_IDLE);
        Serial.println(F("[TELEM] STATE=IDLE"));
      }
      break;
  }
}

// =============================================================================
// Cliff Detection (Tier 1.5)
// =============================================================================

void triggerCliffSafety() {
  stopMotors();
  cliffCountL = 0;
  cliffCountR = 0;

  Serial.print(F("[TELEM] CLIFF LEFT="));
  Serial.print(usLeftDistance, 1);
  Serial.print(F(" RIGHT="));
  Serial.print(usRightDistance, 1);
  Serial.print(F(" BASE_L="));
  Serial.print(tune.cliffBaselineL, 1);
  Serial.print(F(" BASE_R="));
  Serial.println(tune.cliffBaselineR, 1);

  // Start immediate reverse burst to back away from drop-off
  setMotors(-tune.cliffRevSpeed, -tune.cliffRevSpeed);
  transitionTo(STATE_CLIFF_REVERSING, tune.cliffRevMs);

  Serial.println(F("[TELEM] STATE=CLIFF_REVERSING"));
}

// =============================================================================
// Periodic Ultrasonic Polling
// =============================================================================

void pollUltrasonics() {
  unsigned long now = millis();
  unsigned long interval = tune.usPollMs / 2;
  if (interval < 25) interval = 25; // Guarantee at least 25ms settling between pings

  if (now - lastUsPollTime < interval) {
    return;
  }
  lastUsPollTime = now;

  static uint8_t activeSensor = 0; // 0 = Left, 1 = Right
  if (activeSensor == 0) {
    usLeftDistance = readUltrasonic(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
    if (usLeftDistance > (tune.cliffBaselineL + tune.cliffMargin)) {
      cliffCountL++;
    } else {
      cliffCountL = 0;
    }
    activeSensor = 1;
  } else {
    usRightDistance = readUltrasonic(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);
    if (usRightDistance > (tune.cliffBaselineR + tune.cliffMargin)) {
      cliffCountR++;
    } else {
      cliffCountR = 0;
    }
    activeSensor = 0;

    // Broadcast complete sensor pair readings if telemetry stream is enabled
    if (tune.telemStream) {
      Serial.print(F("[TELEM] US LEFT="));
      Serial.print(usLeftDistance, 1);
      Serial.print(F(" RIGHT="));
      Serial.println(usRightDistance, 1);
    }
  }

  // ---------------------------------------------------------------------------
  // Tier 1.5: Cliff Detection Check (Negative Obstacle)
  // Distance jumping significantly higher than ground baseline (or timeout 999)
  // Either sensor must independently confirm cliff across multiple pings of ITS OWN
  // ---------------------------------------------------------------------------
  bool cliffConfirmed = tune.cliffEnable &&
                        ((cliffCountL >= tune.cliffConfirmCount) ||
                         (cliffCountR >= tune.cliffConfirmCount));

  // Trigger cliff safety only if moving forward or turning
  if (cliffConfirmed) {
    if (currentState == STATE_DRIVING || currentState == STATE_TURNING ||
        (currentState == STATE_AVOIDING && avoidPhase == AVOID_PHASE_RESUME)) {
      triggerCliffSafety();
      return;
    }
  }

  // ---------------------------------------------------------------------------
  // Tier 2: Obstacle Detection Check (Positive Obstacle)
  // Distance dropping below obstacle threshold
  // ---------------------------------------------------------------------------
  if (currentState == STATE_DRIVING || currentState == STATE_TURNING) {
    if (now >= avoidCooldownEnd) {
      if (usLeftDistance < tune.usObstacleDist || usRightDistance < tune.usObstacleDist) {
        startAvoidance();
        return;
      }
    }
  }
}

// =============================================================================
// Command Handlers
// =============================================================================

void handleDrive(char* args) {
  if (currentState == STATE_PI_STOPPED) {
    Serial.println(F("[ACK] BLOCKED (PI_STOPPED)"));
    return;
  }
  if (currentState == STATE_CLIFF_STOPPED) {
    Serial.println(F("[ACK] BLOCKED (CLIFF_STOPPED)"));
    return;
  }
  if (!args) {
    Serial.println(F("[ACK] ERROR (BAD_DRIVE_ARGS)"));
    return;
  }

  char* speedStr = strtok(args, " ");
  char* durStr   = strtok(NULL, " ");
  if (!speedStr || !durStr) {
    Serial.println(F("[ACK] ERROR (BAD_DRIVE_ARGS)"));
    return;
  }

  int speed = constrain(atoi(speedStr), 0, 255);
  unsigned long duration = (unsigned long)atol(durStr);

  savedDriveSpeed = speed;
  setMotors(speed, speed);
  transitionTo(STATE_DRIVING, duration);

  Serial.print(F("[ACK] DRIVE "));
  Serial.print(speed);
  Serial.print(' ');
  Serial.println(duration);

  Serial.print(F("[TELEM] STATE=DRIVING SPEED="));
  Serial.println(speed);
}

void handleReverse(char* args) {
  if (currentState == STATE_PI_STOPPED) {
    Serial.println(F("[ACK] BLOCKED (PI_STOPPED)"));
    return;
  }
  if (currentState == STATE_CLIFF_STOPPED) {
    Serial.println(F("[ACK] BLOCKED (CLIFF_STOPPED)"));
    return;
  }
  if (!args) {
    Serial.println(F("[ACK] ERROR (BAD_REV_ARGS)"));
    return;
  }

  char* speedStr = strtok(args, " ");
  char* durStr   = strtok(NULL, " ");
  if (!speedStr || !durStr) {
    Serial.println(F("[ACK] ERROR (BAD_REV_ARGS)"));
    return;
  }

  int speed = constrain(atoi(speedStr), 0, 255);
  unsigned long duration = (unsigned long)atol(durStr);

  savedDriveSpeed = 0;
  setMotors(-speed, -speed);
  transitionTo(STATE_REVERSING, duration);

  Serial.print(F("[ACK] REV "));
  Serial.print(speed);
  Serial.print(' ');
  Serial.println(duration);

  Serial.print(F("[TELEM] STATE=REVERSING SPEED="));
  Serial.println(speed);
}

void handleTurn(char* args) {
  if (currentState == STATE_PI_STOPPED) {
    Serial.println(F("[ACK] BLOCKED (PI_STOPPED)"));
    return;
  }
  if (currentState == STATE_CLIFF_STOPPED) {
    Serial.println(F("[ACK] BLOCKED (CLIFF_STOPPED)"));
    return;
  }
  if (!args) {
    Serial.println(F("[ACK] ERROR (BAD_TURN_ARGS)"));
    return;
  }

  char* angleStr = strtok(args, " ");
  char* speedStr = strtok(NULL, " ");
  if (!angleStr || !speedStr) {
    Serial.println(F("[ACK] ERROR (BAD_TURN_ARGS)"));
    return;
  }

  int angle = atoi(angleStr);
  int speed = constrain(atoi(speedStr), 0, 255);
  unsigned long duration = (unsigned long)abs(angle) * (unsigned long)tune.turnMsPerDeg;

  savedDriveSpeed = 0;
  if (angle > 0) {
    setMotors(speed, -speed);  // Turn Right
  } else {
    setMotors(-speed, speed);  // Turn Left
  }
  transitionTo(STATE_TURNING, duration);

  Serial.print(F("[ACK] TURN "));
  Serial.print(angle);
  Serial.print(' ');
  Serial.println(speed);

  Serial.print(F("[TELEM] STATE=TURNING ANGLE="));
  Serial.print(angle);
  Serial.print(F(" SPEED="));
  Serial.println(speed);
}

void handleStop() {
  stopMotors();
  cliffCountL = 0;
  cliffCountR = 0;
  if (currentState != STATE_PI_STOPPED && currentState != STATE_CLIFF_STOPPED) {
    transitionTo(STATE_IDLE);
    Serial.println(F("[ACK] STOP"));
    Serial.println(F("[TELEM] STATE=IDLE"));
  } else {
    Serial.print(F("[ACK] BLOCKED ("));
    Serial.print(stateToString(currentState));
    Serial.println(F(")"));
  }
}

void handlePiStop() {
  stopMotors();
  cliffCountL = 0;
  cliffCountR = 0;
  transitionTo(STATE_PI_STOPPED);
  Serial.println(F("[ACK] PI_STOP"));
  Serial.println(F("[TELEM] STATE=PI_STOPPED"));
}

void handlePiGo() {
  if (currentState == STATE_PI_STOPPED || currentState == STATE_CLIFF_STOPPED) {
    stopMotors();
    cliffCountL = 0;
    cliffCountR = 0;
    transitionTo(STATE_IDLE);
    Serial.println(F("[ACK] PI_GO"));
    Serial.println(F("[TELEM] STATE=IDLE"));
  } else {
    Serial.println(F("[ACK] PI_GO IGNORED (NOT_STOPPED)"));
  }
}

void handlePiAvoid(char* args) {
  if (!args) {
    Serial.println(F("[ACK] ERROR (BAD_PI_AVOID_ARGS)"));
    return;
  }
  char* dir = strtok(args, " ");
  if (dir && strcmp(dir, "LEFT") == 0) {
    piAvoidHint = -1;
    Serial.println(F("[ACK] PI_AVOID LEFT"));
  } else if (dir && strcmp(dir, "RIGHT") == 0) {
    piAvoidHint = 1;
    Serial.println(F("[ACK] PI_AVOID RIGHT"));
  } else {
    Serial.println(F("[ACK] ERROR (BAD_PI_AVOID_ARGS)"));
  }
}

// =============================================================================
// =============================================================================
// EEPROM Non-Volatile Persistence
// =============================================================================

void saveTuneParamsToEEPROM() {
  EEPROMHeader hdr = { EEPROM_CONFIG_MAGIC, 1 };
  EEPROM.put(0, hdr);
  EEPROM.put(sizeof(EEPROMHeader), tune);
  Serial.println(F("[ACK] TUNE SAVE SUCCESS"));
}

bool loadTuneParamsFromEEPROM() {
  EEPROMHeader hdr;
  EEPROM.get(0, hdr);
  if (hdr.magic == EEPROM_CONFIG_MAGIC) {
    EEPROM.get(sizeof(EEPROMHeader), tune);
    Serial.println(F("[TELEM] TUNE LOADED_FROM_EEPROM"));
    return true;
  }
  return false;
}

void resetTuneParamsToDefaults() {
  TunableParams defaultTune = {
    US_OBSTACLE_DISTANCE_CM,
    US_PING_INTERVAL_MS,
    CLIFF_MARGIN_CM,
    CLIFF_BASELINE_DEFAULT_CM,
    CLIFF_BASELINE_DEFAULT_CM,
    CLIFF_REVERSE_MS,
    CLIFF_REVERSE_SPEED,
    CLIFF_CONFIRM_COUNT,
    AVOID_SPEED,
    AVOID_REVERSE_MS,
    AVOID_TURN_MS,
    AVOID_RESUME_MS,
    AVOID_COOLDOWN_MS,
    WATCHDOG_TIMEOUT_MS,
    TURN_MS_PER_DEGREE,
    MOTOR_TRIM_DEFAULT,
    TELEM_STREAM_DEFAULT,
    CLIFF_ENABLE_DEFAULT
  };
  tune = defaultTune;
  Serial.println(F("[ACK] TUNE RESET_TO_DEFAULTS"));
}

// =============================================================================
// Runtime Tuning Handler
// =============================================================================

void dumpTuneParams() {
  Serial.print(F("[TELEM] TUNE US_THRESH="));
  Serial.print(tune.usObstacleDist, 1);
  Serial.print(F(" US_POLL_MS="));
  Serial.print(tune.usPollMs);
  Serial.print(F(" CLIFF_MARGIN="));
  Serial.print(tune.cliffMargin, 1);
  Serial.print(F(" CLIFF_BASE_L="));
  Serial.print(tune.cliffBaselineL, 1);
  Serial.print(F(" CLIFF_BASE_R="));
  Serial.println(tune.cliffBaselineR, 1);

  Serial.print(F("[TELEM] TUNE CLIFF_REV_MS="));
  Serial.print(tune.cliffRevMs);
  Serial.print(F(" CLIFF_REV_SPD="));
  Serial.print(tune.cliffRevSpeed);
  Serial.print(F(" CLIFF_CONFIRM="));
  Serial.println(tune.cliffConfirmCount);

  Serial.print(F("[TELEM] TUNE AVOID_SPD="));
  Serial.print(tune.avoidSpeed);
  Serial.print(F(" AVOID_REV_MS="));
  Serial.print(tune.avoidRevMs);
  Serial.print(F(" AVOID_TURN_MS="));
  Serial.print(tune.avoidTurnMs);
  Serial.print(F(" AVOID_FWD_MS="));
  Serial.print(tune.avoidResumeMs);
  Serial.print(F(" AVOID_COOL_MS="));
  Serial.println(tune.avoidCooldownMs);

  Serial.print(F("[TELEM] TUNE WDG_MS="));
  Serial.print(tune.watchdogMs);
  Serial.print(F(" TURN_CAL="));
  Serial.print(tune.turnMsPerDeg);
  Serial.print(F(" MOTOR_TRIM="));
  Serial.print(tune.motorTrim);
  Serial.print(F(" TELEM_STREAM="));
  Serial.print(tune.telemStream);
  Serial.print(F(" CLIFF_ENABLE="));
  Serial.println(tune.cliffEnable);
}

void handleTune(char* args) {
  if (!args) {
    Serial.println(F("[ACK] ERROR (MISSING_TUNE_ARGS)"));
    return;
  }

  char* subcmd = strtok(args, " ");
  if (!subcmd) {
    Serial.println(F("[ACK] ERROR (BAD_TUNE_CMD)"));
    return;
  }

  if (strcmp(subcmd, "GET") == 0) {
    dumpTuneParams();
    Serial.println(F("[ACK] TUNE GET"));
    return;
  }

  if (strcmp(subcmd, "SAVE") == 0) {
    saveTuneParamsToEEPROM();
    return;
  }

  if (strcmp(subcmd, "LOAD") == 0) {
    if (loadTuneParamsFromEEPROM()) {
      Serial.println(F("[ACK] TUNE LOAD SUCCESS"));
    } else {
      Serial.println(F("[ACK] ERROR (NO_STORED_EEPROM_CONFIG)"));
    }
    return;
  }

  if (strcmp(subcmd, "RESET") == 0) {
    resetTuneParamsToDefaults();
    return;
  }

  if (strcmp(subcmd, "RECAL") == 0) {
    if (currentState == STATE_IDLE || currentState == STATE_CLIFF_STOPPED) {
      calibrateCliffBaselines();
      Serial.println(F("[ACK] TUNE RECAL DONE"));
    } else {
      Serial.println(F("[ACK] REJECTED (MUST_BE_IDLE)"));
    }
    return;
  }

  if (strcmp(subcmd, "SET") == 0) {
    char* param = strtok(NULL, " ");
    char* val   = strtok(NULL, " ");
    if (!param || !val) {
      Serial.println(F("[ACK] ERROR (BAD_TUNE_SET_ARGS)"));
      return;
    }

    if (strcmp(param, "US_THRESH") == 0) {
      tune.usObstacleDist = atof(val);
    } else if (strcmp(param, "US_POLL_MS") == 0) {
      tune.usPollMs = (unsigned long)atol(val);
    } else if (strcmp(param, "CLIFF_MARGIN") == 0) {
      tune.cliffMargin = atof(val);
    } else if (strcmp(param, "CLIFF_BASE_L") == 0) {
      tune.cliffBaselineL = atof(val);
    } else if (strcmp(param, "CLIFF_BASE_R") == 0) {
      tune.cliffBaselineR = atof(val);
    } else if (strcmp(param, "CLIFF_REV_MS") == 0) {
      tune.cliffRevMs = (unsigned long)atol(val);
    } else if (strcmp(param, "CLIFF_REV_SPD") == 0) {
      tune.cliffRevSpeed = constrain(atoi(val), 0, 255);
    } else if (strcmp(param, "CLIFF_CONFIRM") == 0) {
      tune.cliffConfirmCount = max(1, atoi(val));
    } else if (strcmp(param, "AVOID_SPD") == 0) {
      tune.avoidSpeed = constrain(atoi(val), 0, 255);
    } else if (strcmp(param, "AVOID_REV_MS") == 0) {
      tune.avoidRevMs = (unsigned long)atol(val);
    } else if (strcmp(param, "AVOID_TURN_MS") == 0) {
      tune.avoidTurnMs = (unsigned long)atol(val);
    } else if (strcmp(param, "AVOID_FWD_MS") == 0) {
      tune.avoidResumeMs = (unsigned long)atol(val);
    } else if (strcmp(param, "AVOID_COOL_MS") == 0) {
      tune.avoidCooldownMs = (unsigned long)atol(val);
    } else if (strcmp(param, "WDG_MS") == 0) {
      tune.watchdogMs = (unsigned long)atol(val);
    } else if (strcmp(param, "TURN_CAL") == 0) {
      tune.turnMsPerDeg = atoi(val);
    } else if (strcmp(param, "MOTOR_TRIM") == 0) {
      tune.motorTrim = constrain(atoi(val), -50, 50);
    } else if (strcmp(param, "TELEM_STREAM") == 0) {
      tune.telemStream = (atoi(val) > 0) ? 1 : 0;
    } else if (strcmp(param, "CLIFF_ENABLE") == 0) {
      tune.cliffEnable = (atoi(val) > 0) ? 1 : 0;
    } else {
      Serial.print(F("[ACK] ERROR (UNKNOWN_PARAM: "));
      Serial.print(param);
      Serial.println(F(")"));
      return;
    }

    Serial.print(F("[ACK] TUNE SET "));
    Serial.print(param);
    Serial.print('=');
    Serial.println(val);
    return;
  }

  Serial.println(F("[ACK] ERROR (UNKNOWN_TUNE_SUBCMD)"));
}

// =============================================================================
// Serial Protocol Parser
// =============================================================================

void processSerialLine(char* line) {
  // Strip leading whitespace
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  // Mark Pi activity for watchdog
  lastPiRxTime  = millis();
  watchdogArmed = true;

  // Split command and arguments
  char* cmd  = strtok(line, " \r\n");
  char* args = strtok(NULL, "\r\n");

  if (!cmd) return;

  // Tier 1 Commands
  if (strcmp(cmd, "PI_STOP") == 0) {
    handlePiStop();
  } else if (strcmp(cmd, "PI_GO") == 0) {
    handlePiGo();
  } else if (strcmp(cmd, "PI_AVOID") == 0) {
    handlePiAvoid(args);
  }
  // Meta Tuning Commands
  else if (strcmp(cmd, "TUNE") == 0) {
    handleTune(args);
  }
  // Tier 3 Commands
  else if (strcmp(cmd, "DRIVE") == 0) {
    handleDrive(args);
  } else if (strcmp(cmd, "REV") == 0) {
    handleReverse(args);
  } else if (strcmp(cmd, "TURN") == 0) {
    handleTurn(args);
  } else if (strcmp(cmd, "STOP") == 0) {
    handleStop();
  } else if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("[ACK] PONG"));
  } else if (strcmp(cmd, "STATUS") == 0) {
    Serial.print(F("[TELEM] STATE="));
    Serial.print(stateToString(currentState));
    Serial.print(F(" LEFT="));
    Serial.print(usLeftDistance, 1);
    Serial.print(F(" RIGHT="));
    Serial.println(usRightDistance, 1);
  } else {
    Serial.print(F("[ACK] ERROR (UNKNOWN_CMD: "));
    Serial.print(cmd);
    Serial.println(F(")"));
  }
}

void readSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBufLen > 0) {
        serialBuf[serialBufLen] = '\0';
        processSerialLine(serialBuf);
        serialBufLen = 0;
      }
    } else {
      if (serialBufLen < SERIAL_BUFFER_SIZE - 1) {
        serialBuf[serialBufLen++] = c;
      } else {
        // Buffer overflow: discard line to recover safely
        serialBufLen = 0;
        Serial.println(F("[ACK] ERROR (BUFFER_OVERFLOW)"));
      }
    }
  }
}

// =============================================================================
// Watchdog & Timed Actions Check
// =============================================================================

void checkTimersAndWatchdog() {
  unsigned long now = millis();

  // Watchdog timeout check
  if (watchdogArmed && tune.watchdogMs > 0) {
    if (currentState == STATE_DRIVING || currentState == STATE_TURNING ||
        currentState == STATE_REVERSING) {
      if (now - lastPiRxTime > tune.watchdogMs) {
        stopMotors();
        transitionTo(STATE_IDLE);
        Serial.println(F("[TELEM] HEARTBEAT_TIMEOUT"));
        Serial.println(F("[TELEM] STATE=IDLE"));
      }
    }
  }

  // Active state timer expiration
  switch (currentState) {
    case STATE_DRIVING:
      if (stateDuration > 0 && (now - stateStartTime >= stateDuration)) {
        stopMotors();
        transitionTo(STATE_IDLE);
        Serial.println(F("[TELEM] DONE DRIVE"));
        Serial.println(F("[TELEM] STATE=IDLE"));
      }
      break;

    case STATE_REVERSING:
      if (stateDuration > 0 && (now - stateStartTime >= stateDuration)) {
        stopMotors();
        transitionTo(STATE_IDLE);
        Serial.println(F("[TELEM] DONE REV"));
        Serial.println(F("[TELEM] STATE=IDLE"));
      }
      break;

    case STATE_TURNING:
      if (stateDuration > 0 && (now - stateStartTime >= stateDuration)) {
        stopMotors();
        transitionTo(STATE_IDLE);
        Serial.println(F("[TELEM] DONE TURN"));
        Serial.println(F("[TELEM] STATE=IDLE"));
      }
      break;

    case STATE_CLIFF_REVERSING:
      // When cliff reverse burst finishes, latch into CLIFF_STOPPED
      if (now - stateStartTime >= tune.cliffRevMs) {
        stopMotors();
        transitionTo(STATE_CLIFF_STOPPED);
        Serial.println(F("[TELEM] STATE=CLIFF_STOPPED"));
      }
      break;

    case STATE_AVOIDING:
      tickAvoidance();
      break;

    default:
      break;
  }
}

// =============================================================================
// setup() and loop()
// =============================================================================

void setup() {
  // Serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 1000); // Wait up to 1s for native USB boards if any

  // Configure Motor Pins
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_LEFT_PWM,  OUTPUT);
  pinMode(PIN_LEFT_IN1,  OUTPUT);
  pinMode(PIN_LEFT_IN2,  OUTPUT);
  stopMotors();

  // Configure Ultrasonic Pins
  pinMode(PIN_US_LEFT_TRIG,  OUTPUT);
  pinMode(PIN_US_LEFT_ECHO,  INPUT);
  pinMode(PIN_US_RIGHT_TRIG, OUTPUT);
  pinMode(PIN_US_RIGHT_ECHO, INPUT);

  // Announce ready
  Serial.println(F("[TELEM] READY Uno-ATmega328P v2.1"));

  // Check and load persistent calibration from EEPROM if present
  bool loadedEEPROM = loadTuneParamsFromEEPROM();

  // If no stored EEPROM calibration, perform boot-time ground auto-calibration
  if (!loadedEEPROM) {
    calibrateCliffBaselines();
  }

  Serial.println(F("[TELEM] STATE=IDLE"));
}

void loop() {
  readSerialInput();
  pollUltrasonics();
  checkTimersAndWatchdog();
}
