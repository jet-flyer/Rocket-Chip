# RocketChip Directory Structure Scaffolding

This document defines the target directory structure for the RocketChip firmware.
Created: 2026-01-09
Updated: 2026-01-31

## Build System

Pure CMake + Pico SDK + FreeRTOS SMP (PlatformIO abandoned - requires Arduino framework for RP2350)

## Directory Tree

```
rocketchip/
├── CMakeLists.txt                 # CMake build configuration
├── FreeRTOSConfig.h               # FreeRTOS SMP configuration
├── pico_sdk_import.cmake          # Pico SDK integration
├── FreeRTOS_Kernel_import.cmake   # FreeRTOS integration
├── build.sh                       # Build script
├── openocd_cmsis_dap.cfg          # Debug probe config
├── README.md                      # Project overview
├── CHANGELOG.md                   # Development history
├── PROJECT_STATUS.md              # Current phase and active work
├── PROJECT_OVERVIEW.md            # High-level project description
├── GETTING_STARTED.md             # Setup guide for new contributors
├── AGENT_WHITEBOARD.md            # Multi-agent review flags
├── COUNCIL_PROCESS.md             # Council review protocol
├── CROSS_AGENT_REVIEW.md          # Cross-agent collaboration
│
├── docs/
│   ├── SAD.md                     # Software Architecture Document
│   ├── SCAFFOLDING.md             # This file
│   ├── HARDWARE.md                # Hardware specs, pin assignments, I2C addresses
│   ├── TOOLCHAIN_VALIDATION.md    # Build/debug setup guide
│   ├── GEMINI_CARRIER_BOARD.md    # Gemini redundant flight computer design
│   ├── Missions.md                # Mission format specification (TBD)
│   │
│   └── icd/                       # Interface Control Documents
│       ├── EXPANSION_CONNECTOR_ICD.md  # Feather-based expansion connector
│       └── GEMINI_PROTOCOL_ICD.md      # Gemini inter-MCU protocol
│
├── standards/
│   ├── CODING_STANDARDS.md        # Code style and safety rules
│   ├── DEBUG_OUTPUT.md            # USB CDC output format
│   ├── GIT_WORKFLOW.md            # Git conventions
│   │
│   └── protocols/                 # Communication protocol standards
│       └── SPACEWIRE_LITE.md      # SpaceWire-Lite (aspirational standard)
│
├── tools/
│   └── state_to_dot.py            # State machine visualization
│
├── include/
│   └── rocketchip/
│       ├── config.h               # Build configuration, feature flags
│       ├── version.h              # Version info
│       ├── pins.h                 # Pin definitions per board variant
│       └── features.h             # Feature detection macros
│
├── src/
│   ├── main.cpp                   # Entry point, FreeRTOS task creation
│   ├── hooks.cpp                  # FreeRTOS hooks (idle, malloc, stack overflow)
│   │
│   ├── core/                      # Mission Engine
│   │   ├── MissionEngine.h/.cpp   # Top-level orchestrator
│   │   ├── StateMachine.h/.cpp    # State management
│   │   ├── EventEngine.h/.cpp     # Event detection & dispatch
│   │   ├── ActionExecutor.h/.cpp  # Action handling
│   │   ├── Condition.h/.cpp       # Condition parsing & evaluation
│   │   ├── ControlLoop.h/.cpp     # PID control (Titan/TVC)
│   │   └── MissionLoader.h/.cpp   # Mission loading & validation
│   │
│   ├── hal/                       # Hardware Abstraction Layer
│   │   ├── HAL.h/.cpp             # Top-level HAL init
│   │   ├── Bus.h/.cpp             # I2C/SPI bus abstraction
│   │   ├── GPIO.h/.cpp            # Digital I/O
│   │   ├── ADC.h/.cpp             # Analog input
│   │   ├── PWM.h/.cpp             # PWM output
│   │   ├── PIO.h/.cpp             # PIO (NeoPixel driver)
│   │   ├── Timing.h/.cpp          # Delays, timestamps
│   │   ├── UART.h/.cpp            # Serial communication
│   │   ├── BoardDetect.h/.cpp     # Runtime board/pack detection
│   │   ├── IMU.h                  # IMU interface
│   │   ├── IMU_ICM20948.cpp       # ICM-20948 9-DOF driver (primary)
│   │   ├── Mag_LIS3MDL.cpp        # LIS3MDL magnetometer driver
│   │   ├── Baro.h                 # Barometer interface
│   │   ├── Baro_DPS310.cpp        # DPS310 driver
│   │   ├── GPS.h/.cpp             # GPS interface + driver
│   │   ├── Radio.h                # Radio interface
│   │   ├── Radio_RFM95.cpp        # LoRa driver
│   │   ├── Storage.h/.cpp         # Flash storage abstraction
│   │   ├── Display.h/.cpp         # OLED driver (optional)
│   │   ├── LED.h/.cpp             # NeoPixel driver
│   │   ├── Buttons.h/.cpp         # Button handling
│   │   ├── Pyro.h/.cpp            # Pyro channels (Titan)
│   │   └── Servo.h/.cpp           # Servo PWM (Titan)
│   │
│   ├── services/                  # FreeRTOS Tasks
│   │   ├── SensorTask.h/.cpp      # High-rate sensor sampling
│   │   ├── FusionTask.h/.cpp      # ESKF navigation, MMAE bank (Titan)
│   │   ├── MissionTask.h/.cpp     # Event/state processing
│   │   ├── LoggerTask.h/.cpp      # Data logging to storage
│   │   ├── TelemetryTask.h/.cpp   # MAVLink transmission
│   │   ├── UITask.h/.cpp          # Display, LED, buttons
│   │   └── ControlTask.h/.cpp     # TVC control loop (Titan)
│   │
│   ├── protocol/                  # Communication protocols
│   │   ├── MAVLink.h/.cpp         # MAVLink encoding/decoding
│   │   └── CommandHandler.h/.cpp  # USB/Serial command interface
│   │
│   ├── missions/                  # Built-in mission definitions
│   │   ├── Missions.h             # Mission registry
│   │   ├── Mission_Rocket.cpp     # Basic model rocket
│   │   ├── Mission_HPR.cpp        # High-power dual deploy
│   │   ├── Mission_Glider.cpp     # Glider/bungee test
│   │   ├── Mission_HAB.cpp        # High-altitude balloon
│   │   └── Mission_Freeform.cpp   # Just log everything
│   │
│   └── utils/                     # Utilities
│       ├── RingBuffer.h           # Lock-free ring buffer
│       ├── MovingAverage.h        # Signal smoothing
│       ├── PID.h                  # PID controller
│       └── CRC.h                  # CRC calculations
│
├── lib/                           # External libraries
│   ├── ap_compat/                 # ArduPilot compatibility layer (36+ directories)
│   │   ├── AP_HAL_Compat.h        # Config, feature flags, utility macros
│   │   ├── AP_HAL/                # Stub headers for AP includes [SEE NOTE 1]
│   │   ├── AP_HAL_RP2350/         # Platform HAL implementation
│   │   │   ├── AP_HAL_RP2350.h    # Main include
│   │   │   ├── HAL_RP2350_Class.* # HAL singleton
│   │   │   ├── Scheduler.*        # FreeRTOS task/timer mapping
│   │   │   ├── Semaphores.*       # Mutex/BinarySemaphore
│   │   │   ├── Util.*             # Memory, system ID, arming
│   │   │   ├── Storage.*          # Flash storage [IMPLEMENTED]
│   │   │   ├── I2CDevice.*        # I2C bus manager [IMPLEMENTED]
│   │   │   ├── SPIDevice.*        # SPI bus manager [IMPLEMENTED]
│   │   │   ├── DeviceBus.*        # Device polling thread [IMPLEMENTED]
│   │   │   └── hwdef.h            # Board definitions
│   │   ├── AP_InertialSensor/     # IMU abstraction with std::atomic fix (PD12)
│   │   ├── AP_InternalError/      # Error reporting
│   │   ├── GCS_MAVLink/           # MAVLink GCS integration [IMPLEMENTED]
│   │   ├── RocketChip/            # RocketChip-specific hwdef
│   │   └── stubs/                 # Additional ArduPilot stubs
│   ├── ardupilot/                 # ArduPilot libraries (sparse checkout)
│   │   ├── AP_Math/               # Vector, matrix, quaternion math
│   │   ├── Filter/                # Signal processing filters
│   │   ├── AP_AccelCal/           # Accelerometer calibration
│   │   ├── AP_InertialSensor/     # Base sensor interface
│   │   ├── AP_FlashStorage/       # Wear-leveled flash [IMPLEMENTED]
│   │   └── StorageManager/        # Storage regions [IMPLEMENTED]
│   ├── mavlink/                   # MAVLink v2 headers [IMPLEMENTED]
│   └── st_drivers/                # ST platform-independent drivers (legacy)
│
├── tests/
│   └── smoke_tests/               # Hardware validation tests
│       ├── hal_validation.cpp     # HAL hardware smoke test
│       ├── st_sensors_test.cpp    # ST driver sensors (IMU, Mag, Baro)
│       ├── ap_hal_test.cpp        # AP_HAL_RP2350 smoke test [VALIDATED]
│       ├── calibration_test.cpp   # Accelerometer calibration
│       ├── storage_test.cpp       # Flash storage (PLANNED)
│       ├── gps_test.cpp           # GPS module test
│       ├── radio_tx_test.cpp      # Radio transmit test
│       ├── i2c_scan.c             # I2C device scanner
│       ├── simple_test.c          # Basic LED blink test
│       └── imu_qwiic_test.c       # IMU QWIIC connectivity test
│
├── ground_station/                # Ground station receiver
│   └── radio_rx.cpp               # RX bridge (RFM95W breakout on Feather M0)
│
├── test/                          # Unit tests (future)
│   ├── test_state_machine.cpp
│   ├── test_condition.cpp
│   └── test_event_engine.cpp
│
├── pico-sdk/                      # Git submodule
└── FreeRTOS-Kernel/               # Git submodule
```

## Module Responsibilities

| Module | Responsibility |
|--------|----------------|
| **MissionEngine** | Load missions, coordinate subsystems, manage lifecycle |
| **StateMachine** | Track current state, validate transitions, enforce timeouts |
| **EventEngine** | Evaluate conditions against sensor data, fire events |
| **ActionExecutor** | Execute actions (log, beep, LED, pyro, etc.) |
| **Condition** | Parse and evaluate condition expressions |
| **SensorTask** | Sample IMU/Baro/GPS at configured rates |
| **FusionTask** | ESKF navigation, MMAE bank management (Titan), AHRS cross-check |
| **LoggerTask** | Buffer data, write to flash, manage pre-launch buffer |
| **TelemetryTask** | Encode MAVLink, transmit via radio |
| **UITask** | Update display, handle buttons, drive LED patterns |

## Task Architecture

| Task | Priority | Rate | Stack | Core | Notes |
|------|----------|------|-------|------|-------|
| SensorTask | 5 (highest) | 1kHz | 1KB | 0 | Hard real-time, DMA preferred |
| ControlTask | 5 | 500Hz | 1KB | 0 | Only active during BOOST (Titan) |
| FusionTask | 4 | 200-400Hz | 2KB | 1 | ESKF navigation (see docs/ESKF/) |
| MissionTask | 4 | 100Hz | 2KB | 1 | State machine, events |
| LoggerTask | 3 | 50Hz | 2KB | 1 | Buffered writes |
| TelemetryTask | 2 | 10Hz | 1KB | 1 | MAVLink over LoRa |
| UITask | 1 (lowest) | 30Hz | 1KB | 1 | Display, LEDs, buttons |

## CMake Build Targets

| Target | Type | Description |
|--------|------|-------------|
| `freertos_validation` | Dev | Main FreeRTOS + HAL validation firmware |
| `smoke_hal_validation` | Dev | HAL hardware smoke test |
| `smoke_st_sensors` | Dev | ST driver sensor validation (IMU, Mag, Baro) |
| `smoke_gps` | Dev | GPS module validation |
| `smoke_radio_tx` | Dev | Radio transmit test |
| `smoke_imu_qwiic` | Dev | IMU QWIIC connectivity test |
| `i2c_scan` | Dev | I2C device scanner utility |
| `simple_test` | Dev | Basic LED blink test |
| `radio_rx` | GCS | Ground station receiver bridge (for Feather M0) |
| `rocketchip_core` | Core | Minimal - local logging only (future) |
| `rocketchip_main` | Main | GPS, telemetry capable (future) |
| `rocketchip_titan` | Titan | Full features - pyro, TVC, high-G (future) |

## Testing Workflow

CMake places all build artifacts in `build/`. After verifying a smoke test works on hardware:

1. Copy the verified UF2 to `tests/smoke_tests/` for archival:
   ```
   cp build/smoke_*.uf2 tests/smoke_tests/
   ```

2. Add to `.gitignore` if UF2s shouldn't be tracked, or commit them for quick re-flashing without rebuilding.

This keeps verified binaries organized with their source files while keeping the build directory disposable.

## Implementation Status

**📋 For the complete development roadmap and detailed phase tracking, see `docs/SAD.md` Section 10.**

This section provides a high-level snapshot:

**Phase 1: Foundation** - ✅ **COMPLETE**
- CMake build system, FreeRTOS SMP, core HAL (Bus, GPIO, ADC, PWM, PIO, UART, Timing)
- Smoke tests, documentation, build scripts
- See SAD.md Section 10 for full checklist

**Phase 2: Sensors** - ⚙️ **IN PROGRESS**
- ✅ Hardware drivers complete: ICM-20948 (ArduPilot Invensensev2), DPS310, PA1010D GPS, RFM95W radio
- ✅ Smoke tests validated on hardware
- ✅ FreeRTOS SensorTask (high-rate sampling) - **COMPLETE**
- ✅ AP_HAL_RP2350 - GPIO, AnalogIn, UART, I2C, SPI all validated
- ✅ std::atomic fix for dual-core memory visibility (PD12) - **COMPLETE**
- ✅ CLI/RC_OS menu system with MAVLink calibration integration
- ❌ Data logging to flash - pending
- See SAD.md Section 10 for full checklist

**Phases 3-9: Mission Engine, Fusion, Storage, Telemetry, UI** - 📋 **PLANNED**
- State machine, ESKF + MMAE sensor fusion (see `docs/ESKF/`), MAVLink telemetry, flash logging
- See SAD.md Section 10 for full roadmap

**File-Level Details:**

Implemented files:
- Build: CMakeLists.txt, build.sh, FreeRTOSConfig.h
- HAL: HAL, Bus, GPIO, ADC, PWM, PIO, UART, Timing
- Sensors: IMU_ICM20948 (primary), Baro_DPS310, GPS_PA1010D, Radio_RFM95W
- Services: SensorTask (production entry point with calibration support)
- CLI: RC_OS.h (MAVLink command routing)
- AP_HAL_RP2350: Scheduler, Semaphores, Util, Storage, I2CDevice, SPIDevice, DeviceBus
- Tests: Multiple smoke tests (see `tests/smoke_tests/`)
- Ground station: radio_rx.cpp (deprecated - using Fruit Jam for GCS)
- Docs: SAD, SCAFFOLDING, HARDWARE, standards

## Related Documents

- **docs/SAD.md** - Software Architecture Document (full architecture details)
- **docs/HARDWARE.md** - Hardware specifications, pin assignments, I2C addresses
- **docs/TOOLCHAIN_VALIDATION.md** - Build and debug setup guide
- **docs/GEMINI_CARRIER_BOARD.md** - Gemini redundant flight computer design
- **docs/icd/EXPANSION_CONNECTOR_ICD.md** - Feather-based expansion connector ICD
- **docs/icd/GEMINI_PROTOCOL_ICD.md** - Gemini inter-MCU protocol ICD
- **standards/CODING_STANDARDS.md** - Code style and safety rules
- **standards/protocols/SPACEWIRE_LITE.md** - SpaceWire-Lite communication protocol
- **docs/RP2350_FULL_AP_PORT.md** - Platform differences for full ArduPilot port

---

## Notes

### Note 1: AP_HAL Stub Headers

The `lib/ap_compat/AP_HAL/` directory contains minimal stub headers that satisfy ArduPilot's `#include <AP_HAL/AP_HAL.h>` chain without implementing functionality. Real implementations live in `lib/ap_compat/AP_HAL_RP2350/`.

**RE-EVALUATION REQUIRED:** When Phase 2 implements real UART/GPIO/SPI drivers, evaluate whether stubs should:
1. Forward to real implementations
2. Be deleted and include paths restructured
3. Remain as fallbacks

See `docs/AP_HAL_RP2350_PLAN.md` Decision D5 for full context.
