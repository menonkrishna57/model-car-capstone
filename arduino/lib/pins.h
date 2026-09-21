#ifndef CAR_PINS_H
#define CAR_PINS_H

// =============================================================================
// Arduino Uno (ATmega328P) Pin Mapping for L298N Skid-Steer Motor Driver
// Left Pair: Motors FL + RL (Channel A)
// Right Pair: Motors FR + RR (Channel B)
// =============================================================================

// Left Motor Pair (Channel A)
const int PIN_ENA = 5;    // PWM Speed Control (Timer0 / Pin D5)
const int PIN_IN1 = 8;    // Direction 1
const int PIN_IN2 = 9;    // Direction 2

// Right Motor Pair (Channel B)
const int PIN_ENB = 6;    // PWM Speed Control (Timer0 / Pin D6)
const int PIN_IN3 = 10;   // Direction 1
const int PIN_IN4 = 11;   // Direction 2

// =============================================================================
// Ultrasonic Sensors (2× HC-SR04P) — Left and Right
// ⚠️ PINS NOT FINALIZED — placeholder defaults, teammate will confirm wiring
// =============================================================================

// Left Ultrasonic
const int PIN_US_LEFT_TRIG  = 2;   // Trigger
const int PIN_US_LEFT_ECHO  = 3;   // Echo (D3 is interrupt-capable)

// Right Ultrasonic
const int PIN_US_RIGHT_TRIG = 4;   // Trigger
const int PIN_US_RIGHT_ECHO = 7;   // Echo

#endif // CAR_PINS_H
