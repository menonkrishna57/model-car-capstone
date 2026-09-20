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

// Ultrasonic Sensors (HC-SR04P) — using Analog pins as Digital I/O
const int PIN_TRIG_LEFT   = A0;
const int PIN_ECHO_LEFT   = A1;
const int PIN_TRIG_CENTER = A2;
const int PIN_ECHO_CENTER = A3;
const int PIN_TRIG_RIGHT  = A4;
const int PIN_ECHO_RIGHT  = A5;

#endif // CAR_PINS_H
