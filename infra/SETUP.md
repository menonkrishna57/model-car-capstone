# Raspberry Pi 5 Headless Bring-Up & Setup Guide

**Owned by**: `network-agent`  
**Target Device**: Raspberry Pi 5 (8GB)  
**Hostnames**:
- Krishna: `pi5-krishna`
- Aditya: `pi5-aditya`
- Parth: `pi5-parth`

---

## 1. Cloud-Init One-Shot Flashing

> [!WARNING]
> Cloud-init files (`network-config` and `user-data`) are evaluated **only once** on first boot. If Wi-Fi fails to connect, edit the files on the boot partition of the SD card/NVMe and reflash or recreate the configuration.

1. Flash Raspberry Pi OS (64-bit Lite or Desktop) using Raspberry Pi Imager.
2. In the `bootfs` partition, copy:
   - `infra/pi-image/network-config` → `/boot/firmware/network-config` (or root of boot partition)
   - `infra/pi-image/user-data` → `/boot/firmware/user-data`
3. Eject safely and insert into the Raspberry Pi 5.
4. Power up with the dedicated boAt EnergyShroom PB400 power bank.

---

## 2. Interface Enablement via `raspi-config`

Once connected via SSH (`ssh pi@pi5-krishna.local`):

Run:
```bash
sudo raspi-config
```

Enable the following interfaces:
1. **Camera (CSI)**: `Interface Options` → `Camera` → `Enable`
2. **I2C**: `Interface Options` → `I2C` → `Enable` (required for ICM20948 IMU)
3. **Serial Port**:
   - `Interface Options` → `Serial Port`
   - *Would you like a login shell to be accessible over serial?* → **NO** (CRITICAL: frees UART for Arduino Uno link)
   - *Would you like the serial port hardware to be enabled?* → **YES**

Reboot to apply changes:
```bash
sudo reboot
```

---

## 3. SSH and Raspberry Pi Connect
- **SSH**: Enabled by default via `user-data`.
- **Pi Connect**: Optional browser-based remote access:
  ```bash
  sudo apt install rpi-connect
  rpi-connect on
  rpi-connect signin
  ```
