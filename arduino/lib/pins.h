/**
 * pins.h — Arduino Uno (ATmega328P) Pin Mapping
 * ===============================================
 * Single source of truth for all pin assignments.
 * Owned by firmware-agent. Update this file after any physical rewiring.
 *
 * CURRENT WIRING (as-built per arduino_rf_robot_pin_configuration_wiring_guide.md):
 *   - ENA/ENB are JUMPERED to 5V → motors run full speed only (no PWM speed control).
 *   - D5 = US1 Trigger, D6 = US2 Echo (PWM-capable pins wasted on digital signals).
 *
 * TARGET WIRING (after 4-wire PWM migration — see architecture.md §3c):
 *   - ENA → D5, ENB → D6 (PWM speed control enabled).
 *   - US1 Trigger → D12, US2 Echo → D13 (freed from D5/D6).
 *
 * *** THE HARDWARE HAS NOT BEEN CHANGED YET ***
 * This file reflects the CURRENT physical wiring.
 * Update PIN_ENA, PIN_ENB, PIN_US_LEFT_TRIG, PIN_US_RIGHT_ECHO after the swap.
 */

#ifndef CAR_PINS_H
#define CAR_PINS_H

// =============================================================================
// Left Motor Pair — Channel A (FL + RL motors)
// L298N: IN1 / IN2 control direction; ENA controls speed (or jumper = full speed)
// =============================================================================
const int PIN_ENA = 5;    // ENA: PWM speed control (Timer0/OC0B) — currently jumpered to 5V!
const int PIN_IN1 = 8;    // IN1: Direction bit 1 — HIGH = forward
const int PIN_IN2 = 9;    // IN2: Direction bit 2 — HIGH = reverse

// =============================================================================
// Right Motor Pair — Channel B (FR + RR motors)
// L298N: IN3 / IN4 control direction; ENB controls speed (or jumper = full speed)
// =============================================================================
const int PIN_ENB = 6;    // ENB: PWM speed control (Timer0/OC0A) — currently jumpered to 5V!
const int PIN_IN3 = 10;   // IN3: Direction bit 1 — HIGH = forward
const int PIN_IN4 = 11;   // IN4: Direction bit 2 — HIGH = reverse

// =============================================================================
// Ultrasonic Sensors — 2× HC-SR04P, both front-facing
//
// CURRENT WIRING (per RF wiring guide):
//   US1 Front-Left:   Trig=D5,  Echo=D4
//   US2 Front-Right:  Trig=D7,  Echo=D6
//
// ⚠️  D5 and D6 are PWM-capable pins being used for US signals.
//     After the PWM migration these will move to D12/D13.
// =============================================================================

// Left Ultrasonic (HC-SR04P front-left)
const int PIN_US_LEFT_TRIG = 5;    // D5 — Trigger (10µs pulse OUTPUT)
const int PIN_US_LEFT_ECHO = 4;    // D4 — Echo (INPUT, pulse timing)

// Right Ultrasonic (HC-SR04P front-right)
const int PIN_US_RIGHT_TRIG = 7;   // D7 — Trigger (10µs pulse OUTPUT)
const int PIN_US_RIGHT_ECHO = 6;   // D6 — Echo (INPUT, pulse timing)

// =============================================================================
// IR Proximity Sensors (×2) — Likely to be scrapped; pins reserved.
// D2 / D3 are the current wiring per RF guide. D3 is interrupt-capable.
// =============================================================================
const int PIN_IR1 = 2;    // D2 — IR Sensor 1 (digital HIGH/LOW)
const int PIN_IR2 = 3;    // D3 — IR Sensor 2 (interrupt-capable)

#endif // CAR_PINS_H
