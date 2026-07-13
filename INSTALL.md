# Installation Guide — premaiduino2

Custom STM32duino firmware for the PremaidAI (PMA) humanoid robot, forked from [devemin/Pre-maiduino](https://github.com/devemin/Pre-maiduino).

> ⚠️ **This is a beta build.** Do **not** flash this to your robot. It contains known bugs and can physically damage your PremaidAI. The code is provided for reference and development purposes only.

---

## Table of Contents

- [What This Is](#what-this-is)
- [Hardware Requirements](#hardware-requirements)
- [Prerequisites](#prerequisites)
- [Installation Steps](#installation-steps)
  - [1. Install Arduino IDE](#1-install-arduino-ide)
  - [2. Install Arduino Core for STM32](#2-install-arduino-core-for-stm32)
  - [3. Install the Kondo ICS Library](#3-install-the-kondo-ics-library)
  - [4. Clone This Repository](#4-clone-this-repository)
  - [5. Configure the Sketch Folder](#5-configure-the-sketch-folder)
  - [6. Select Board Settings](#6-select-board-settings)
  - [7. Compile](#7-compile)
- [Repository Structure](#repository-structure)
- [Companion Web Tools](#companion-web-tools)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)

---

## What This Is

This project replaces the factory firmware on a PremaidAI (Kondo Kagaku) 25-servo desktop humanoid robot with a custom STM32duino-based firmware, adding:

- Wireless control over Bluetooth (RN42), compatible with the original app/Unity protocol
- `.pma` motion file playback (both live streaming and on-board Flash storage)
- IMU-based orientation sensing (MPU-6050)
- A table-based walking gait
- ASCII serial command interface for debugging and manual control

## Hardware Requirements

- PremaidAI robot (STM32F102CBT6-based main board, 25× Kondo ICS3.5 servos)
- RN42 Bluetooth module (onboard)
- MPU-6050 IMU (onboard)
- USB-to-serial programmer capable of flashing STM32F102 (e.g. ST-Link, or USB-serial adapter depending on your board's bootloader setup)


## Prerequisites

Install these three components **in order** before opening the sketch:

| Component | Version | Link |
|---|---|---|
| Arduino IDE | 2.3.7 | [arduino.cc/en/software](https://www.arduino.cc/en/software) |
| Arduino Core for STM32 | 2.12.0 | [stm32duino/Arduino_Core_STM32 releases](https://github.com/stm32duino/Arduino_Core_STM32/releases) |
| ICS Library for Arduino | v3 | [Kondo Robot FAQ — ICS Library](https://kondo-robot.com/faq/ics-library-a3) |

---

## Installation Steps

### 1. Install Arduino IDE

Download and install **Arduino IDE 2.3.7** (or a compatible 2.x release) from the [official site](https://www.arduino.cc/en/software).

### 2. Install Arduino Core for STM32

1. In Arduino IDE, go to **File → Preferences**.
2. Add the following URL to **Additional Boards Manager URLs**:
   ```
   https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
   ```
3. Go to **Tools → Board → Boards Manager**, search for **STM32**, and install **STM32 MCU based boards** version **2.12.0**.

Reference: [stm32duino/Arduino_Core_STM32 releases](https://github.com/stm32duino/Arduino_Core_STM32/releases)

### 3. Install the Kondo ICS Library

Follow Kondo's official instructions to install **ICS Library for Arduino ver.3**:

[https://kondo-robot.com/faq/ics-library-a3](https://kondo-robot.com/faq/ics-library-a3)

This provides `IcsHardSerialClass.h` / `IcsBaseClass.h`, which the firmware depends on for half-duplex communication with the HV/MV servo buses.

### 4. Clone This Repository

```bash
git clone https://github.com/arthurson/premaiduino2.git
```

### 5. Configure the Sketch Folder

Arduino IDE requires the `.ino` file and **all** of its header (`.h`) files to sit in a folder with the **same name as the `.ino` file**. Rename the cloned folder (or its contents) so the structure looks like this:

```
Pre-maiduino2/
├── Pre-maiduino2.ino
├── ServoConfig.h
├── PmaProtocol.h
├── LEDControl.h
├── VoltageMonitor.h
├── MPU6050IMU.h
├── IcsBaseClass.h
├── hal_conf_extra.h
├── table_walk.h
└── table_walk.c
```

> `hal_conf_extra.h` **must** sit in this folder alongside the `.ino` file — this is a hard requirement of the STM32duino core's build system (it is picked up by `HardwareSerial.cpp`'s compilation unit, not by the `.ino` file's own defines), and increases the serial RX/TX buffer size needed for reliable `.pma` packet reception.

Open `Pre-maiduino2.ino` in Arduino IDE — the other `.h`/`.c` files will appear as additional tabs automatically.

### 6. Select Board Settings

Under **Tools**, set:

| Setting | Value |
|---|---|
| Board | Generic STM32F1 series |
| Board part number | Generic F102CBTx (128KB Flash) — select the closest match for STM32F102CBT6; if this exact part isn't listed in your STM32 core version, "BluePill F103CB" shares the same 128KB Flash layout and can work as a fallback, but confirm pin mapping matches your board first |
| Upload method | (per your programmer — e.g. STM32CubeProgrammer / Serial) |
| U(S)ART support | Enabled (generic 'Serial') |
| USB support | (per your board) |

Confirm your exact chip marking (`STM32F102CBT6`) against your PremaidAI's main board silkscreen before flashing anything — do not assume the board variant from this guide matches your unit without checking.

### 7. Compile

**Sketch → Verify/Compile** (or `Ctrl+R`). The Arduino IDE console will report flash usage at the end, e.g.:

```
Sketch uses XXXXX bytes (XX%) of program storage space. Maximum is 131072 bytes.
```

Firmware size should stay under **~69,632 bytes** (68KB), since the top 60KB of the STM32F102CBT6's 128KB flash is reserved for on-device `.pma` motion storage (see `PmaProtocol.h` for the exact memory map).

---

## Repository Structure

| File | Purpose |
|---|---|
| `Pre-maiduino2.ino` | Main sketch: servo control, table_walk dispatch, command processing, `setup()`/`loop()` |
| `ServoConfig.h` | `ServoInfo` struct and the 25-servo data table (angle limits, home positions, bus assignment) |
| `PmaProtocol.h` | `.pma`/ICS binary packet protocol, on-chip Flash motion storage, and the non-blocking UART receive state machine |
| `LEDControl.h` | RGB status LED driver (active-low, non-blocking breathing effects) |
| `VoltageMonitor.h` | Battery voltage monitoring and shutdown/warning thresholds |
| `MPU6050IMU.h` | MPU-6050 IMU driver (I2C, pitch/roll computation, gyro calibration) |
| `IcsBaseClass.h` | Kondo ICS servo communication base class |
| `hal_conf_extra.h` | STM32duino HAL override (increases serial buffer size) |
| `table_walk.h` / `table_walk.c` | Table-based walking gait generator (ported from kazzlog's PreMaidMathWalkSample) |
| `controller.html` | Web Serial API browser tool for manual servo control |
| `pma_bt_sender.html` | Web Serial API tool for sending `.pma` motion files over Bluetooth |
| `plen2.conversion.html` | Motion format conversion utility |

## Companion Web Tools

The `.html` files in this repo use the **Web Serial API** and run directly in a Chromium-based browser (Chrome/Edge) — no installation needed. Open the file locally and connect to your robot's Bluetooth serial port from the page.

## Troubleshooting

- **Compile error mentioning a type/struct "does not name a type" or "has not been declared":** Arduino IDE auto-generates forward declarations for every function and inserts them near the top of the file. If you've modified `.ino` internals, ensure any `struct`/`typedef` used by those forward declarations is still defined before this auto-generated block (see the comments at the top of `Pre-maiduino2.ino`).
- **Servo writes intermittently fail / garbled ICS responses:** Confirm you're using single-pin (half-duplex) `HardwareSerial` constructors for `Serial2`/`Serial3` — dual-pin constructors listen on an unconnected RX pin and will not read ICS responses correctly.
- **Robot freezes mid-motion during Bluetooth streaming:** Confirm `hal_conf_extra.h` is present in the sketch folder and `SERIAL_RX_BUFFER_SIZE` is taking effect (check the compiled binary isn't silently falling back to the default 64-byte buffer).

## Credits

- Original firmware base: [devemin/Pre-maiduino](https://github.com/devemin/Pre-maiduino)
- Walking gait algorithm: ported from [kazzlog/pm_sample](https://github.com/kazzlog/pm_sample/tree/master/pm_sample)
- Protocol reference: community analysis of the official PremaidAI app and Unity client (`PreMaidServo.cs`)
- ICS servo communication: [Kondo Kagaku ICS Library for Arduino](https://kondo-robot.com/faq/ics-library-a3)

---

**Disclaimer:** This firmware is experimental and provided as-is with no warranty. Flashing it to real hardware may cause unexpected servo behavior, including motion outside safe mechanical limits. Proceed at your own risk.
