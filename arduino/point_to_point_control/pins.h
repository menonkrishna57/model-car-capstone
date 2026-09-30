/**
 * pins.h — Arduino Uno (ATmega328P) Pin Mapping
 * ===============================================
 * Single source of truth for all pin assignments.
 * Owned by firmware-agent. Update this file after any physical rewiring.
 *
 * WIRING STATUS: ✅ PWM migration COMPLETE (2026-09-30)
 *   - ENA wired to D5 (PWM speed control enabled)
 *   - ENB wired to D6 (PWM speed control enabled)
 *   - US1 Trigger moved from D5 → D12
 *   - US2 Echo moved from D6 → D13
 */

#ifndef CAR_PINS_H
#define CAR_PINS_H

// =============================================================================
// Left Motor Pair — Channel A (FL + RL motors)
// L298N: IN1 / IN2 control direction; ENA controls PWM speed (0–255)
// =============================================================================
const int PIN_ENA = 5;    // ENA: PWM speed control (Timer0/OC0B) — wired to D5 ✅
const int PIN_IN1 = 8;    // IN1: Direction bit 1 — HIGH = forward
const int PIN_IN2 = 9;    // IN2: Direction bit 2 — HIGH = reverse

// =============================================================================
// Right Motor Pair — Channel B (FR + RR motors)
// L298N: IN3 / IN4 control direction; ENB controls PWM speed (0–255)
// =============================================================================
const int PIN_ENB = 6;    // ENB: PWM speed control (Timer0/OC0A) — wired to D6 ✅
const int PIN_IN3 = 10;   // IN3: Direction bit 1 — HIGH = forward
const int PIN_IN4 = 11;   // IN4: Direction bit 2 — HIGH = reverse

// =============================================================================
// Ultrasonic Sensors — 2× HC-SR04P, both front-facing
//
// After PWM migration (2026-09-30):
//   US1 Front-Left:   Trig=D12, Echo=D4
//   US2 Front-Right:  Trig=D7,  Echo=D13
// =============================================================================

// Left Ultrasonic (HC-SR04P front-left)
const int PIN_US_LEFT_TRIG = 12;   // D12 — Trigger (10μs pulse OUTPUT)
const int PIN_US_LEFT_ECHO = 4;    // D4  — Echo (INPUT, pulse timing)

// Right Ultrasonic (HC-SR04P front-right)
const int PIN_US_RIGHT_TRIG = 7;   // D7  — Trigger (10μs pulse OUTPUT)
const int PIN_US_RIGHT_ECHO = 13;  // D13 — Echo (INPUT, pulse timing) — built-in LED flickers (cosmetic only)

// =============================================================================
// IR Proximity Sensors (×2) — wired on D2/D3
// D3 is interrupt-capable.
// =============================================================================
const int PIN_IR1 = 2;    // D2 — IR Sensor 1 (digital HIGH/LOW)
const int PIN_IR2 = 3;    // D3 — IR Sensor 2 (interrupt-capable)

#endif // CAR_PINS_H
