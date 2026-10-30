/**
 * protocol.h — Shared Serial Protocol Constants
 * ===============================================
 * Single source of truth for timing, thresholds, and prefixes used by
 * the Arduino Uno firmware. Source of truth for the human-readable spec:
 * arduino/PROTOCOL.md
 *
 * All constants below are compile-time DEFAULTS. At runtime, they are
 * loaded into a TunableParams struct and can be overridden via the
 * serial TUNE command (changes reset on power cycle).
 *
 * Never hardcode magic numbers in the .ino sketch — reference these
 * defines or the tune.xxx runtime values.
 */

#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H

// =============================================================================
// Serial Link
// =============================================================================
#define SERIAL_BAUD_RATE    115200   // 8-N-1, matches PROTOCOL.md and Pi-side config.py
#define SERIAL_BUFFER_SIZE  64       // Max command length in bytes (including '\n')

// =============================================================================
// Telemetry / Acknowledgment Prefixes (Pi parses these)
// =============================================================================
#define TELEM_PREFIX   "[TELEM] "
#define ACK_PREFIX     "[ACK] "

// =============================================================================
// Ultrasonic Obstacle Detection
// =============================================================================
#define US_OBSTACLE_DISTANCE_CM  20.0f   // Tier 2 avoidance triggers if either sensor < this
                                         // NOTE: Auto-adjusted after cliff calibration if this
                                         // value exceeds the ground baseline (tilted sensors).
#define US_PING_INTERVAL_MS      100     // Poll ultrasonics every 100ms (staggered 50ms per sensor)
#define US_TIMEOUT_US            15000   // pulseIn timeout in µs (~250cm range; balances detection and loop rate)

// =============================================================================
// Cliff Detection (Negative Obstacle) — Tier 1.5
// -----------------------------------------------------------------------------
// Sensors are tilted downward at an angle for dual-purpose sensing:
//   - Obstacle: reading DROPS below threshold (something closer than ground)
//   - Cliff:    reading JUMPS above baseline + margin (ground disappears)
//
// Auto-calibrated at boot: sensors read the ground surface in setup() to
// establish the baseline. Sanity-checked against plausible range; falls
// back to CLIFF_BASELINE_DEFAULT_CM if calibration looks wrong.
// =============================================================================
#define CLIFF_BASELINE_DEFAULT_CM  30.0f  // Fallback baseline if auto-cal fails (cm)
#define CLIFF_MARGIN_CM            15.0f  // Cliff triggers when reading > baseline + margin (cm)
#define CLIFF_CAL_SAMPLES          10     // Number of readings to average during auto-calibration
#define CLIFF_CAL_MIN_CM           3.0f   // Sanity check: minimum plausible ground baseline (cm)
#define CLIFF_CAL_MAX_CM           60.0f  // Sanity check: maximum plausible ground baseline (cm)
#define CLIFF_CAL_DELAY_MS         50     // Delay between calibration readings (ms)
#define CLIFF_REVERSE_MS           400    // Reverse burst before latching (ms)
#define CLIFF_REVERSE_SPEED        185    // PWM speed for cliff reverse burst (0–255, calibrated >= 180)
#define CLIFF_CONFIRM_COUNT        2      // Consecutive readings above threshold to confirm cliff
#define CLIFF_ENABLE_DEFAULT       1      // 1 = cliff reflex enabled (table/drop-offs), 0 = disabled (floor driving)

// =============================================================================
// Avoidance Maneuver Timing — Tier 2
// Calibrate all three phases on the physical chassis before demo.
// Phase sequence: Reverse → Skid-turn → Resume forward
// =============================================================================
#define AVOID_REVERSE_MS         200     // Phase 1: Back away from obstacle (ms)
#define AVOID_CORNER_REV_MAX_MS  200     // Max reverse duration during dual obstacle / corner backup (ms)
#define AVOID_TURN_MS            500     // Phase 2: Skid-steer turn away from obstacle (ms)
#define AVOID_RESUME_MS          2000    // Phase 3: Drive forward on new heading (ms)
#define AVOID_SPEED              200     // PWM speed used for phases 1 & 2 (0–255, calibrated >= 200)

// =============================================================================
// Avoidance Cooldown (prevents re-trigger during Phase 3 resume)
// While the car is resuming forward after an avoidance, ultrasonic readings
// near the original obstacle may still be < threshold. This cooldown
// suppresses re-triggering for AVOID_COOLDOWN_MS after avoidance completes.
// =============================================================================
#define AVOID_COOLDOWN_MS        500     // Suppress re-trigger for 500ms after avoidance done

// =============================================================================
// Serial Watchdog
// If the Pi goes silent for WATCHDOG_TIMEOUT_MS while the car is driving,
// the Arduino stops motors and sends a HEARTBEAT_TIMEOUT telemetry event.
// Set to 0 to disable.
// =============================================================================
#define WATCHDOG_TIMEOUT_MS      5000    // 5 seconds of Pi silence → safe stop

// =============================================================================
// TURN calibration placeholder
// TURN_MS_PER_DEGREE: how many ms of skid-turn equals 1 degree of heading change.
// Must be measured on the physical chassis. Placeholder = 10ms/deg (uncalibrated).
// =============================================================================
#define TURN_MS_PER_DEGREE       10      // ms per degree — UNCALIBRATED, measure on real chassis

// =============================================================================
// Motor Trim Bias & EEPROM Persistence
// =============================================================================
#define MOTOR_TRIM_DEFAULT       0       // Default motor bias trim (-50 to +50; >0 right faster, <0 left faster)
#define TELEM_STREAM_DEFAULT     1       // 1 = stream periodic US telemetry, 0 = silent (events and STATUS only)
#define EEPROM_CONFIG_MAGIC      0xCA57  // Magic signature to validate stored EEPROM calibration

#endif // CAR_PROTOCOL_H
