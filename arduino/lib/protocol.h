#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H

// =============================================================================
// Shared Serial Protocol Constants (9600 Baud)
// Source of truth: arduino/PROTOCOL.md
// =============================================================================

#define SERIAL_BAUD_RATE 9600
#define SERIAL_BUFFER_SIZE 64

// Telemetry prefixes
#define TELEM_PREFIX "[TELEM] "
#define ACK_PREFIX   "[ACK] "

// Ultrasonic obstacle detection
#define US_OBSTACLE_DISTANCE_CM  20.0f  // Emergency stop if obstacle closer than this
#define US_PING_INTERVAL_MS      100    // Poll ultrasonics every 100ms
#define US_TIMEOUT_US            30000  // pulseIn timeout in microseconds (~5m max range)

// Avoidance maneuver timing (calibrate on real chassis)
#define AVOID_REVERSE_MS         400    // How long to reverse during avoidance
#define AVOID_TURN_MS            500    // How long to skid-turn during avoidance
#define AVOID_SPEED              150    // PWM speed for avoidance maneuvers

#endif // CAR_PROTOCOL_H
