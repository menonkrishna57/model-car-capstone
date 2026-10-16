/**
 * pins.h — Arduino Uno (ATmega328P) Pin Mapping
 * ===============================================
 * Single source of truth for all pin assignments.
 * Owned by firmware-agent. Update this file after any physical rewiring.
 *
 * WIRING STATUS: ✅ Architecture v2 Verified (2026-10-11)
 *   - ENA wired to D5 (Timer0 PWM)
 *   - ENB wired to D6 (Timer0 PWM)
 *   - Left US:  Trig=D7,  Echo=D13 (LED flicker cosmetic)
 *   - Right US: Trig=D12, Echo=D4
 *   - D2, D3: Physically empty / reserved for encoders/interrupts
 *   - INVERT_MOTOR_POLARITY = true (corrects TT motor chassis wiring)
 */

#ifndef CAR_PINS_H
#define CAR_PINS_H

// =============================================================================
// Left Motor Pair — Channel A (FL + RL motors)
// L298N: IN1 / IN2 control direction; ENA controls PWM speed (0–255)
// =============================================================================
const int PIN_ENA = 5;    // ENA: PWM speed control (Timer0/OC0B) — wired to D5 ✅
const int PIN_IN1 = 8;    // IN1: Direction bit 1
const int PIN_IN2 = 9;    // IN2: Direction bit 2

// =============================================================================
// Right Motor Pair — Channel B (FR + RR motors)
// L298N: IN3 / IN4 control direction; ENB controls PWM speed (0–255)
// =============================================================================
const int PIN_ENB = 6;    // ENB: PWM speed control (Timer0/OC0A) — wired to D6 ✅
const int PIN_IN3 = 10;   // IN3: Direction bit 1
const int PIN_IN4 = 11;   // IN4: Direction bit 2

// Motor Polarity Configuration
const bool INVERT_MOTOR_POLARITY = true;

// =============================================================================
// Ultrasonic Sensors — 2× HC-SR04P, both front-facing
// Verified Physical Pinout (Architecture v2):
//   Left Ultrasonic:   Trig=D7,  Echo=D13
//   Right Ultrasonic:  Trig=D12, Echo=D4
// =============================================================================

// Left Ultrasonic (HC-SR04P front-left)
const int PIN_US_LEFT_TRIG = 7;    // D7  — Trigger (10μs pulse OUTPUT)
const int PIN_US_LEFT_ECHO = 13;   // D13 — Echo (INPUT, pulse timing) — built-in LED flickers (cosmetic only)

// Right Ultrasonic (HC-SR04P front-right)
const int PIN_US_RIGHT_TRIG = 12;  // D12 — Trigger (10μs pulse OUTPUT)
const int PIN_US_RIGHT_ECHO = 4;   // D4  — Echo (INPUT, pulse timing)

// =============================================================================
// Reserved Pins — D2 / D3 (Physically empty on chassis)
// Reserved for future wheel encoders (INT0 / INT1)
// =============================================================================
const int PIN_RESERVED_INT0 = 2;   // D2 — External Interrupt 0 (INT0)
const int PIN_RESERVED_INT1 = 3;   // D3 — External Interrupt 1 (INT1)

#endif // CAR_PINS_H
