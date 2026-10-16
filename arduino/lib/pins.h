#ifndef PINS_H
#define PINS_H

// --- Verified Working Hardware Pinout (October 2026) ---

// Left Ultrasonic Sensor
#define PIN_US_LEFT_TRIG    7
#define PIN_US_LEFT_ECHO    13

// Right Ultrasonic Sensor
#define PIN_US_RIGHT_TRIG   12
#define PIN_US_RIGHT_ECHO   4

// L298N Motor Driver
#define PIN_MOTOR_ENA       5
#define PIN_MOTOR_IN1       8
#define PIN_MOTOR_IN2       9

#define PIN_MOTOR_ENB       6
#define PIN_MOTOR_IN3       10
#define PIN_MOTOR_IN4       11

// Motor Polarity Configuration
#define INVERT_MOTOR_POLARITY true

#endif // PINS_H
