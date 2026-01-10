# RocketChip Standards and Protocols

## Coding Standards

**Coding standards are mandatory.** All code must adhere to these standards. Deviations require explicit approval and must be documented in the changelog with rationale. Approved exceptions are tracked in the Exceptions Table below.

### Foundation

This project follows the **JSF AV C++ Coding Standards** (Lockheed Martin, 2005) as the primary reference. JSF is the foundation for NASA/JPL's institutional C++ standard.

**References:**
- [JSF AV C++ Standards (PDF)](http://www.stroustrup.com/JSF-AV-rules.pdf) - Primary reference
- [JPL C Coding Standard (PDF)](https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf) - Additional reference for C-specific guidance
- Power of 10 Rules (Holzmann, JPL) - Distilled safety-critical rules

### JPL Additions (Pro Tier Goals)

These additional rules from JPL's standard are stretch goals for Pro tier:
- Mandatory `static_assert` for compile-time validation
- No uninitialized local variables
- Stricter `const`/`constexpr` usage
- Triple-voting on safety-critical variables

### Exceptions Table

Approved deviations from coding standards. Each exception requires documented rationale.

| ID | Rule | Exception | Rationale | Approved By | Date |
|----|------|-----------|-----------|-------------|------|
| *No exceptions yet* | | | | | |

*When exceptions are approved, add them here with full context. If this table grows large, it may be broken out into a separate EXCEPTIONS.md file.*

---

## Communication Protocols

### Telemetry

| Protocol | Use Case | Notes |
|----------|----------|-------|
| **MAVLink v2** | Primary telemetry | Compatible with QGroundControl, Mission Planner |
| **RFM69HCW** | Physical layer for long-range | 915MHz ISM band (US), current testing board |
| **WiFi** | Short-range / ground testing | ESP32 variants only |
| **Bluetooth/BLE** | Config and data download | ESP32 variants, close-range |
| **NRF24L01** | Budget short-range option | 2.4GHz, mentioned as alternative |

### Sensor Buses

| Bus | Speed | Use Case | Notes |
|-----|-------|----------|-------|
| **I2C** | 400kHz (Fast Mode) | Most sensors | STEMMA QT connectors, easy daisy-chain |
| **SPI** | Up to 10MHz+ | High-speed sensors, SD card | Lower latency than I2C |
| **UART** | 9600-115200 baud | GPS, debug serial | GPS typically 9600 default |

### GPS Protocols

| Protocol | Notes |
|----------|-------|
| **NMEA** | Standard ASCII sentences, human-readable |
| **UBX** | u-blox binary protocol, higher precision/rate |

---

## Data Formats

### Logging

| Format | Use Case | Notes |
|--------|----------|-------|
| **CSV** | Human-readable logs | Easy post-flight analysis, larger file size |
| **Binary** | High-rate logging | Compact, requires parser, preserves precision |
| **MAVLink** | Structured telemetry logs | Compatible with existing analysis tools |

### Configuration

| Format | Use Case | Notes |
|--------|----------|-------|
| **YAML** | Changelogs, mission configs | Human-readable, good for structured data |
| **Header files** | Compile-time config | `config.h` style for build-time settings |

---

## Hardware Interface Standards

| Standard | Description |
|----------|-------------|
| **Feather** | Adafruit board form factor (50.8mm x 22.8mm) |
| **FeatherWing** | Expansion board standard for Feather |
| **STEMMA QT / Qwiic** | 4-pin JST SH connector for I2C (3.3V) |
| **SWD** | Serial Wire Debug interface for programming/debugging |

---

## Safety and Regulatory

### RF Regulations (US)

| Band | Regulation | Limits |
|------|------------|--------|
| **915MHz ISM** | FCC Part 15 | Power and duty cycle limited, license-free |
| **2.4GHz ISM** | FCC Part 15 | WiFi, Bluetooth, NRF24L01 |

*Check local regulations for non-US deployments.*

### Rocketry Safety Codes

| Organization | Applies To |
|--------------|------------|
| **NAR** (National Association of Rocketry) | Low/mid power |
| **TRA** (Tripoli Rocketry Association) | High power |

### Pyro Channel Safety

Firmware handling pyro channels must implement:
- Software arm state (cannot fire unless armed)
- Physical arm switch recommended (Pro tier)
- Continuity checking before arm
- Timeout/auto-disarm after landing
- All pyro code requires council review before merge

### Watchdog Requirements

- Watchdog timer must be enabled in flight-critical code
- Watchdog timeout appropriate for loop rate
- Watchdog kick only in main loop (not in interrupts)

---

## Software Architecture

### RTOS

| Tier | RTOS | Notes |
|------|------|-------|
| Core | Evaluating | May not need RTOS for simple logging |
| Main | Evaluating | Depends on telemetry complexity |
| Pro | **FreeRTOS** | Required for TVC, pyro timing, multi-task |

### State Machines

Flight phases managed via explicit state machine:
- States: IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ERROR
- All transitions logged
- Timeout fallbacks for stuck states

### Mission Engine

Event-action-state architecture (separate module/library):
- **Mission** defines sensors, states, events, actions, logging
- Same hardware runs different Missions for different applications
- See Mission Engine documentation for details

### ArduPilot Library Integration

Using ArduPilot libraries via compatibility shim (not full ArduPilot firmware):
- **AP_Math** - Vectors, matrices, quaternions
- **AP_AHRS** - Attitude estimation (DCM)
- **Filter** - LowPass, Notch filters
- **AP_AccelCal / AP_Compass** - Calibration routines

Full ArduPilot port (ArduRocket) is a stretch goal requiring ChibiOS HAL.
