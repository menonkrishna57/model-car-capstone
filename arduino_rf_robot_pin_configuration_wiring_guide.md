# Arduino RF-Controlled Robot — Pin Configuration & Wiring Guide

A consolidated technical reference for the wireless (RF Control: Forward / Backward / Left / Right) Arduino robot incorporating dual ultrasonic sensors, dual IR obstacle/line sensors, an H-bridge motor driver, and a centralized power bus.

---

## 1. System Overview

* **Main Controller:** Arduino (Uno / Nano / Mega compatible)
* **Control Interface:** RF Wireless Control ($F / B / L / R$)
* **Actuation:** 2× DC Motors driven via H-bridge driver module (e.g., L298N)
* **Sensing:**
  * 2× HC-SR04 Ultrasonic Distance Sensors
  * 2× Infrared (IR) Proximity / Line Sensors
* **Power Distribution:** 16-pin Male Berg strip (8 pins $5\text{V}$, 8 pins $\text{GND}$)

---

## 2. Complete Arduino Digital Pinout

| Arduino Pin | Subsystem | Function | Direction / Mode | Description |
| :--- | :--- | :--- | :--- | :--- |
| **D2** | IR Sensor 1 | `IR1_OUT` | `INPUT` | Digital proximity / line detection 1 |
| **D3** | IR Sensor 2 | `IR2_OUT` | `INPUT` | Digital proximity / line detection 2 |
| **D4** | Ultrasonic 1 | `ECHO_1` | `INPUT` | Echo pulse measurement return 1 |
| **D5** | Ultrasonic 1 | `TRIG_1` | `OUTPUT` | Ultrasonic burst trigger 1 ($10\ \mu\text{s}$) |
| **D6** | Ultrasonic 2 | `ECHO_2` | `INPUT` | Echo pulse measurement return 2 |
| **D7** | Ultrasonic 2 | `TRIG_2` | `OUTPUT` | Ultrasonic burst trigger 2 ($10\ \mu\text{s}$) |
| **D8** | Motor Driver | `IN1` | `OUTPUT` | Motor A Direction Bit 1 |
| **D9** | Motor Driver | `IN2` | `OUTPUT` | Motor A Direction Bit 2 |
| **D10** | Motor Driver | `IN3` | `OUTPUT` | Motor B Direction Bit 1 |
| **D11** | Motor Driver | `IN4` | `OUTPUT` | Motor B Direction Bit 2 |

---

## 3. Bill of Wiring & Connector Inventory

| Wire / Connector Type | Quantity | Used For |
| :--- | :---: | :--- |
| **Female-to-Female (F to F)** | 10 pcs | Power rails ($5\text{V}$, $\text{GND}$) to sensors + driver enable jumpers |
| **Female-to-Male (F to M)** | 10 pcs | Sensor and driver signal pins to Arduino headers |
| **Male-to-Male (M to M)** | 6 pcs | Arduino $5\text{V}$ & $\text{GND}$ to Berg distribution bus + spares |
| **Multi-strand Wire** | As needed | Motor driver outputs (`OUT1`–`OUT4`) to DC motor terminals |
| **Male Berg Strip** | 16 pins total | Split into $8\times \text{GND}$ and $8\times 5\text{V}\ (\text{VCC})$ power rails |

---

## 4. Subsystem Interconnect Details

### 4.1 Ultrasonic Sensors ($2\times$ HC-SR04)
* **Total Jumpers:** 4× F to M (Signals) + 4× F to F (Power) = 8 wires

| Sensor Pin | Wire Type | Destination | Notes |
| :--- | :--- | :--- | :--- |
| **Sensor 1 Echo** | Female-to-Male (F-M) | Arduino **D4** | Echo pulse read |
| **Sensor 1 Trig** | Female-to-Male (F-M) | Arduino **D5** | Trigger pulse send |
| **Sensor 1 VCC** | Female-to-Female (F-F) | $5\text{V}$ Berg Rail | Requires regulated $+5\text{V}$ |
| **Sensor 1 GND** | Female-to-Female (F-F) | $\text{GND}$ Berg Rail | Common ground |
| **Sensor 2 Echo** | Female-to-Male (F-M) | Arduino **D6** | Echo pulse read |
| **Sensor 2 Trig** | Female-to-Male (F-M) | Arduino **D7** | Trigger pulse send |
| **Sensor 2 VCC** | Female-to-Female (F-F) | $5\text{V}$ Berg Rail | Requires regulated $+5\text{V}$ |
| **Sensor 2 GND** | Female-to-Female (F-F) | $\text{GND}$ Berg Rail | Common ground |

---

### 4.2 Infrared Sensors ($2\times$ Digital IR Modules)
* **Total Jumpers:** 2× F to M (Signals) + 4× F to F (Power) = 6 wires

| Sensor Pin | Wire Type | Destination | Notes |
| :--- | :--- | :--- | :--- |
| **IR 1 OUT / Sense** | Female-to-Male (F-M) | Arduino **D2** | Active LOW / HIGH signal |
| **IR 1 VCC** | Female-to-Female (F-F) | $5\text{V}$ Berg Rail | $+5\text{V}$ power input |
| **IR 1 GND** | Female-to-Female (F-F) | $\text{GND}$ Berg Rail | Common ground |
| **IR 2 OUT / Sense** | Female-to-Male (F-M) | Arduino **D3** | Active LOW / HIGH signal |
| **IR 2 VCC** | Female-to-Female (F-F) | $5\text{V}$ Berg Rail | $+5\text{V}$ power input |
| **IR 2 GND** | Female-to-Female (F-F) | $\text{GND}$ Berg Rail | Common ground |

---

### 4.3 Motor Driver Module (L298N / Dual H-Bridge)
* **Total Jumpers:** 4× F to M (Logic) + 2× F to F (Enables) + Multi-strand (Motors)

| Driver Pin / Terminal | Wire Type | Destination | Notes |
| :--- | :--- | :--- | :--- |
| **IN1** | Female-to-Male (F-M) | Arduino **D8** | Motor Left Direction |
| **IN2** | Female-to-Male (F-M) | Arduino **D9** | Motor Left Direction |
| **IN3** | Female-to-Male (F-M) | Arduino **D10** | Motor Right Direction |
| **IN4** | Female-to-Male (F-M) | Arduino **D11** | Motor Right Direction |
| **ENA** | Female-to-Female (F-F) | $+5\text{V}$ or Jumper Clip | Enable Motor A (or PWM) |
| **ENB** | Female-to-Female (F-F) | $+5\text{V}$ or Jumper Clip | Enable Motor B (or PWM) |
| **OUT1 & OUT2** | Multi-strand | Left DC Motor | Direct motor screw terminals |
| **OUT3 & OUT4** | Multi-strand | Right DC Motor | Direct motor screw terminals |

---

### 4.4 Central Power Distribution (Male Berg Strip)

```
        +5V Rail (8 Pins)                  GND Rail (8 Pins)
 [Pin 1] <-- Arduino 5V Out         [Pin 1] <-- Arduino GND
 [Pin 2] --> Ultrasonic 1 VCC       [Pin 2] --> Ultrasonic 1 GND
 [Pin 3] --> Ultrasonic 2 VCC       [Pin 3] --> Ultrasonic 2 GND
 [Pin 4] --> IR Sensor 1 VCC        [Pin 4] --> IR Sensor 1 GND
 [Pin 5] --> IR Sensor 2 VCC        [Pin 5] --> IR Sensor 2 GND
 [Pin 6] --> Driver Logic VCC (5V)  [Pin 6] --> Driver GND (Common)
 [Pin 7] --> [Spare 5V]             [Pin 7] --> [Spare GND]
 [Pin 8] --> [Spare 5V]             [Pin 8] --> [Spare GND]
```

---

## 5. Arduino Starter Code Template

```cpp
/**
 * Wireless RF Robot Controller - Pin Definitions & Basic Setup
 */

// IR Sensor Pins
const uint8_t PIN_IR1 = 2;
const uint8_t PIN_IR2 = 3;

// Ultrasonic Sensor 1 Pins
const uint8_t PIN_US1_ECHO = 4;
const uint8_t PIN_US1_TRIG = 5;

// Ultrasonic Sensor 2 Pins
const uint8_t PIN_US2_ECHO = 6;
const uint8_t PIN_US2_TRIG = 7;

// Motor Driver Pins (L298N)
const uint8_t PIN_MOTOR_IN1 = 8;
const uint8_t PIN_MOTOR_IN2 = 9;
const uint8_t PIN_MOTOR_IN3 = 10;
const uint8_t PIN_MOTOR_IN4 = 11;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Configure IR Sensors
  pinMode(PIN_IR1, INPUT);
  pinMode(PIN_IR2, INPUT);

  // Configure Ultrasonic 1
  pinMode(PIN_US1_TRIG, OUTPUT);
  pinMode(PIN_US1_ECHO, INPUT);
  digitalWrite(PIN_US1_TRIG, LOW);

  // Configure Ultrasonic 2
  pinMode(PIN_US2_TRIG, OUTPUT);
  pinMode(PIN_US2_ECHO, INPUT);
  digitalWrite(PIN_US2_TRIG, LOW);

  // Configure Motor Outputs
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);

  // Stop Motors Initially
  stopMotors();
}

void loop() {
  // Add RF receiver parsing & obstacle detection logic here
}

void stopMotors() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}
```