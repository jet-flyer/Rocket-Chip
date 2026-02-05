# RocketChip Directory Structure Scaffolding

This document defines the target directory structure for the RocketChip firmware.
Created: 2026-01-09
Updated: 2026-02-02

**Status:** Fresh start post-branch reorganization. Previous ArduPilot integration archived in `AP_FreeRTOS` and `AP_ChibiOS` branches.

## Build System

Pure CMake + Pico SDK (bare-metal) (PlatformIO abandoned - requires Arduino framework for RP2350)

## Directory Tree

Structure below shows planned production architecture. See `docs/SAD.md` Section 3.1 for authoritative version.

```
rocketchip/
├── CMakeLists.txt                 # Primary build system (Pico SDK)
├── pico_sdk_import.cmake          # Pico SDK integration
├── README.md                      # Agent instructions
├── CHANGELOG.md                   # Development history
│
├── docs/
│   ├── SAD.md                     # Software Architecture Document
│   ├── SCAFFOLDING.md             # This file
│   ├── PROJECT_OVERVIEW.md        # Vision and product tiers
│   ├── PROJECT_STATUS.md          # Current phase and blockers
│   ├── TOOLCHAIN_VALIDATION.md    # Build/debug setup guide
│   ├── ROCKETCHIP_OS.md           # CLI design
│   ├── PIO_ALLOCATION.md          # PIO state machine allocation tracker (future)
│   ├── ESKF/                      # Sensor fusion architecture
│   │   ├── FUSION_ARCHITECTURE.md
│   │   └── FUSION_ARCHITECTURE_DECISION.md
│   ├── hardware/                  # Hardware design documents
│   │   ├── HARDWARE.md            # Hardware specs, pin assignments
│   │   ├── GEMINI_CARRIER_BOARD.md    # Gemini redundant flight computer (future)
│   │   ├── STATUS_INDICATORS.md   # LED patterns and status feedback
│   │   └── TELSTAR_BOOSTER_PACK.md   # ELRS RC link + FPV video Booster Pack
│   └── icd/                       # Interface Control Documents
│       ├── EXPANSION_CONNECTOR_ICD.md
│       └── GEMINI_PROTOCOL_ICD.md
│
├── standards/
│   ├── CODING_STANDARDS.md        # Code style and safety rules
│   ├── DEBUG_OUTPUT.md            # USB CDC output conventions
│   ├── GIT_WORKFLOW.md            # Git conventions
│   ├── STANDARDS_DEVIATIONS.md    # Deviation tracking
│   └── protocols/
│       └── SPACEWIRE_LITE.md      # SpaceWire-Lite (aspirational)
│
├── include/
│   └── rocketchip/
│       ├── config.h               # Build configuration, feature flags
│       ├── pins.h                 # GPIO assignments (from HARDWARE.md)
│       └── features.h             # Tier feature detection
│
├── src/
│   ├── main.cpp                   # Production entry point
│   │
│   ├── core/                      # Mission Engine (Phase 5+)
│   │   ├── MissionEngine.*        # Top-level orchestrator
│   │   ├── StateMachine.*         # State management
│   │   ├── EventEngine.*          # Event detection & dispatch
│   │   ├── ActionExecutor.*       # Action handling
│   │   └── ControlLoop.*          # PID control (Titan/TVC)
│   │
│   ├── hal/                       # Hardware Drivers (Phase 1-2)
│   │   ├── Bus.*                  # I2C/SPI bus abstraction
│   │   ├── IMU_ICM20948.*         # ICM-20948 9-DoF driver
│   │   ├── Baro_DPS310.*          # DPS310 barometer driver
│   │   ├── GPS_PA1010D.*          # PA1010D GPS driver (NMEA)
│   │   ├── Radio_RFM95W.*         # RFM95W LoRa driver
│   │   ├── Storage.*              # Flash storage
│   │   └── LED.*                  # NeoPixel/status LED driver
│   │
│   ├── services/                  # Application Modules
│   │   ├── SensorTask.*           # High-rate sensor sampling
│   │   ├── FusionTask.*           # ESKF/AHRS processing
│   │   ├── MissionTask.*          # Event/state processing
│   │   ├── LoggerTask.*           # Data logging to storage
│   │   ├── TelemetryTask.*        # MAVLink transmission
│   │   └── UITask.*               # Display, LED, CLI
│   │
│   ├── fusion/                    # Sensor Fusion (Phase 4)
│   │   ├── ESKF.*                 # Error-State Kalman Filter
│   │   ├── MMAE.*                 # Multi-Model Adaptive Estimator (Titan)
│   │   ├── AHRS.*                 # Mahony AHRS cross-check
│   │   └── ConfidenceGate.*       # Estimate validation (Titan)
│   │
│   ├── math/                      # Math Utilities
│   │   ├── Vector3.h              # 3D vector operations
│   │   ├── Quaternion.h           # Quaternion math
│   │   └── Matrix.h               # Matrix operations (may wrap CMSIS-DSP)
│   │
│   ├── cli/                       # CLI/RC_OS Interface
│   │   └── RC_OS.*                # Serial menu and command handling
│   │
│   ├── protocol/                  # Communication Protocols (Phase 7)
│   │   └── MAVLink.*              # MAVLink encoding/decoding
│   │
│   └── missions/                  # Built-in Mission Definitions (Phase 5+)
│       ├── Mission_Rocket.cpp     # Basic model rocket
│       ├── Mission_HPR.cpp        # High-power dual deploy
│       └── Mission_Freeform.cpp   # Just log everything
│
├── lib/                           # External Libraries
│   ├── pico-sdk/                  # Pico SDK (git submodule or system)
│   └── mavlink/                   # MAVLink v2 headers (generated)
│
├── tests/
│   ├── smoke/                     # Hardware smoke tests
│   │   ├── i2c_scan.cpp           # I2C device scanner
│   │   ├── imu_test.cpp           # IMU validation
│   │   ├── baro_test.cpp          # Barometer validation
│   │   └── radio_test.cpp         # Radio TX/RX validation
│   └── unit/                      # Unit tests (host-side, future)
│
└── .claude/                       # Agent context
    ├── CLAUDE.md                  # Main includes
    ├── LESSONS_LEARNED.md         # Debugging journal
    └── DEBUG_PROBE_NOTES.md       # OpenOCD/GDB setup
```

## Module Responsibilities

See `docs/SAD.md` Section 3.2 for authoritative version.

| Module | Responsibility |
|--------|----------------|
| **MissionEngine** | Load missions, coordinate subsystems, manage lifecycle |
| **StateMachine** | Track current state, validate transitions, enforce timeouts |
| **EventEngine** | Evaluate conditions against sensor data, fire events |
| **ActionExecutor** | Execute actions (log, beep, LED, pyro, etc.) |
| **SensorTask** | Sample IMU/Baro/GPS at configured rates |
| **FusionTask** | ESKF navigation, MMAE bank management (Titan), AHRS cross-check |
| **LoggerTask** | Buffer data, write to flash, manage pre-launch buffer |
| **TelemetryTask** | Encode MAVLink, transmit via radio |
| **UITask** | Update display, handle buttons, drive LED/CLI |

## Execution Architecture

Bare-metal Pico SDK with a polling main loop. Modules are called at their target rates using Pico SDK timer/alarm infrastructure. No RTOS task priorities, stacks, or core pinning apply. See `docs/SAD.md` for authoritative architecture.

## CMake Build Targets

**Current status:** Fresh start. Targets will be defined as implementation proceeds.

| Target | Type | Description |
|--------|------|-------------|
| `rocketchip` | Prod | Main application (production firmware) |
| `i2c_scan` | Dev | I2C device scanner utility |
| `smoke_imu` | Dev | IMU validation (ICM-20948) |
| `smoke_baro` | Dev | Barometer validation (DPS310) |
| `smoke_radio` | Dev | Radio TX/RX test (RFM95W) |

## Implementation Status

**📋 For the complete development roadmap, see `docs/SAD.md` Section 10.**
**📋 For current focus and blockers, see `docs/PROJECT_STATUS.md`.**

> **Note:** Starting fresh after archiving ArduPilot integration attempts. All implementation status reset.

**Phase 1: Foundation** - 🔧 **CURRENT**
- [ ] CMake build system with Pico SDK
- [ ] Minimal `main.cpp` with polling main loop
- [ ] USB CDC serial output (debug)
- [ ] LED status indicator (NeoPixel)
- [ ] I2C bus initialization
- [ ] I2C scanner smoke test

**Phases 2-9** - 📋 **PLANNED**
- See SAD.md Section 10 for full roadmap

**Archived Work:**
Previous ArduPilot integration (AP_HAL_RP2350, sensor drivers, calibration) is preserved in:
- `AP_FreeRTOS` branch - FreeRTOS + ArduPilot HAL (working but complex)
- `AP_ChibiOS` branch - ChibiOS exploration (XIP flash issues)

## Related Documents

- **docs/SAD.md** - Software Architecture Document (authoritative for architecture)
- **docs/PROJECT_STATUS.md** - Current phase and blockers
- **docs/PROJECT_OVERVIEW.md** - Vision and product tiers
- **docs/hardware/HARDWARE.md** - Hardware specifications, pin assignments, I2C addresses
- **docs/TOOLCHAIN_VALIDATION.md** - Build and debug setup guide
- **docs/ROCKETCHIP_OS.md** - CLI design
- **docs/ESKF/** - Sensor fusion architecture
- **docs/hardware/GEMINI_CARRIER_BOARD.md** - Gemini redundant flight computer (future)
- **docs/hardware/TELSTAR_BOOSTER_PACK.md** - Telstar ELRS RC link + FPV video Booster Pack
- **standards/CODING_STANDARDS.md** - Code style and safety rules
