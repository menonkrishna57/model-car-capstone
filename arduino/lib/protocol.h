/**
 * protocol.h — Shared Serial Protocol Constants
 * ===============================================
 * Single source of truth for timing, thresholds, and prefixes used by
 * the Arduino Uno firmware. Source of truth for the human-readable spec:
 * arduino/PROTOCOL.md
 *
 * All avoidance timing constants are calibrate-able via this file only —
 * never hardcode magic numbers in the .ino sketch.
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
#define US_PING_INTERVAL_MS      100     // Poll both ultrasonics every 100ms
#define US_TIMEOUT_US            30000   // pulseIn timeout in µs (~5m max range for HC-SR04P)

// =============================================================================
// Avoidance Maneuver Timing
// Calibrate all three phases on the physical chassis before demo.
// Phase sequence: Reverse → Skid-turn → Resume forward
// =============================================================================
#define AVOID_REVERSE_MS         400     // Phase 1: Back away from obstacle (ms)
#define AVOID_TURN_MS            500     // Phase 2: Skid-steer turn away from obstacle (ms)
#define AVOID_RESUME_MS          2000    // Phase 3: Drive forward on new heading (ms)
#define AVOID_SPEED              150     // PWM speed used for phases 1 & 2 (0–255)

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

#endif // CAR_PROTOCOL_H
