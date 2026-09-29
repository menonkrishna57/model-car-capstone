# ADR 0001: Serial Transport — USB-Serial vs GPIO-UART

- **Status**: Accepted
- **Deciders**: Krishna (Lead / Hardware / Integration), firmware-agent, integration-agent
- **Date**: 2026-09-29 (Updated from 2026-09-08 evaluation)

## Context
The Raspberry Pi 5 communicates with the Arduino Uno (and later Nano) over full-duplex serial at 115200 baud. Two physical transport mechanisms exist on the chassis:

1. **USB-Serial**:
   - Standard USB Type-A (Pi 5) to USB Type-B (Uno) or mini-USB (Nano) cable.
   - Appears on Pi as `/dev/ttyUSB0` or `/dev/ttyACM0`.
2. **GPIO-UART**:
   - Hardware UART pins on Pi 5 GPIO header (GPIO 14 TXD, GPIO 15 RXD) wired directly to Arduino RX (Pin 0) and TX (Pin 1).
   - Requires logic level shifting (Pi is 3.3V logic; Uno/Nano is 5V logic).
   - Appears on Pi as `/dev/ttyAMA0`.

## Trade-offs

| Factor | USB-Serial | GPIO-UART |
|---|---|---|
| **Level Shifting** | Not needed (handled internally by USB-UART IC on Arduino) | Mandatory bidirectional 3.3V ↔ 5V level shifter to protect Pi GPIO |
| **Wiring Simplicity** | Plug-and-play single USB cable | 4 jumper wires + level shifter board on breadboard/perfboard |
| **Reset Behavior** | DTR line toggles auto-reset on connect (requires 2s bootloader sleep) | Reset pin not connected; Arduino does not reset on port open |
| **Physical Space** | Bulkier USB connectors | Low-profile Dupont wiring |

## Decision
**USB-Serial (`/dev/ttyUSB0` at 115200 baud)** is accepted as the active transport for the vehicle. It eliminates level shifting complexity and provides stable isolation. Code references remain parameterized in `car/perception/config.py` and `car/control/motor_controller.py`.
