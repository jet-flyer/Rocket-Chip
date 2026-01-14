# RocketChip Directory Structure Scaffolding

This document defines the target directory structure for the RocketChip firmware.
Created: 2026-01-09
Updated: 2026-01-14

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
│   └── Missions.md                # Mission format specification (TBD)
│
├── standards/
│   ├── CODING_STANDARDS.md        # Code style and safety rules
│   ├── DEBUG_OUTPUT.md            # USB CDC output format
│   └── GIT_WORKFLOW.md            # Git conventions
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
│   ├── main.c                     # Entry point, FreeRTOS task creation
│   ├── hooks.c                    # FreeRTOS hooks (idle, malloc, stack overflow)
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
│   │   ├── IMU_ISM330DHCX.cpp     # ISM330DHCX 6-DOF driver
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
│   │   ├── FusionTask.h/.cpp      # AHRS, altitude, velocity
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
│   ├── ap_compat/                 # ArduPilot compatibility shim
│   │   ├── AP_HAL_Compat.h        # HAL function stubs
│   │   └── AP_HAL_Compat.cpp
│   ├── AP_Math/                   # ArduPilot math (submodule)
│   ├── Filter/                    # ArduPilot filters (submodule)
│   └── mavlink/                   # MAVLink headers
│
├── tests/
│   └── smoke_tests/
│       ├── hal_validation.cpp     # HAL hardware smoke test
│       ├── simple_test.c          # Basic LED blink test
│       └── imu_qwiic_test.c       # IMU QWIIC test
│
├── test/                          # Unit tests
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
| **FusionTask** | AHRS, altitude estimation, velocity integration |
| **LoggerTask** | Buffer data, write to flash, manage pre-launch buffer |
| **TelemetryTask** | Encode MAVLink, transmit via radio |
| **UITask** | Update display, handle buttons, drive LED patterns |

## Task Architecture

| Task | Priority | Rate | Stack | Core | Notes |
|------|----------|------|-------|------|-------|
| SensorTask | 5 (highest) | 1kHz | 1KB | 0 | Hard real-time, DMA preferred |
| ControlTask | 5 | 500Hz | 1KB | 0 | Only active during BOOST (Titan) |
| FusionTask | 4 | 200Hz | 2KB | 1 | AHRS, altitude calc |
| MissionTask | 4 | 100Hz | 2KB | 1 | State machine, events |
| LoggerTask | 3 | 50Hz | 2KB | 1 | Buffered writes |
| TelemetryTask | 2 | 10Hz | 1KB | 1 | MAVLink over LoRa |
| UITask | 1 (lowest) | 30Hz | 1KB | 1 | Display, LEDs, buttons |

## CMake Build Targets

| Target | Tier | Description |
|--------|------|-------------|
| `freertos_smp_validation` | Dev | FreeRTOS dual-core validation firmware |
| `smoke_hal_validation` | Dev | HAL hardware smoke test |
| `simple_test` | Dev | Basic LED blink test |
| `rocketchip_core` | Core | Minimal - local logging only (planned) |
| `rocketchip_main` | Main | GPS, telemetry capable (planned) |
| `rocketchip_titan` | Titan | Full features - pyro, TVC, high-G (planned) |

## Implementation Status

**Implemented:**
- [x] CMakeLists.txt, build.sh, pico_sdk_import.cmake, FreeRTOS_Kernel_import.cmake
- [x] FreeRTOSConfig.h (SMP dual-core config)
- [x] main.c, hooks.c (FreeRTOS entry point)
- [x] hal/HAL.h/.cpp (top-level init)
- [x] hal/Bus.h/.cpp (I2C/SPI)
- [x] hal/GPIO.h/.cpp, ADC.h/.cpp, PWM.h/.cpp
- [x] hal/PIO.h/.cpp (NeoPixel), Timing.h/.cpp, UART.h/.cpp
- [x] tests/smoke_tests/hal_validation.cpp, simple_test.c, imu_qwiic_test.c
- [x] docs/ (SAD, SCAFFOLDING, HARDWARE, TOOLCHAIN_VALIDATION)
- [x] standards/ (CODING_STANDARDS, DEBUG_OUTPUT, GIT_WORKFLOW)
- [x] tools/state_to_dot.py

**Phase 2 - In Progress:**
- [ ] hal/IMU_ISM330DHCX.cpp (sensor driver)
- [ ] hal/Mag_LIS3MDL.cpp (magnetometer driver)
- [ ] hal/Baro_DPS310.cpp (barometer driver)

**Phase 3+ - Planned:**
- [ ] include/rocketchip/ headers (config.h, pins.h, etc.)
- [ ] core/ modules (MissionEngine, StateMachine, EventEngine, etc.)
- [ ] services/ tasks (SensorTask, FusionTask, LoggerTask, etc.)
- [ ] hal/ peripherals (GPS, Radio, Storage, Display, LED, Buttons)
- [ ] hal/ Titan features (Pyro, Servo)
- [ ] protocol/ modules (MAVLink, CommandHandler)
- [ ] missions/ definitions
- [ ] utils/ helpers
- [ ] lib/ external libraries (ap_compat, mavlink)
- [ ] test/ unit tests

## Related Documents

- **docs/SAD.md** - Software Architecture Document (full architecture details)
- **docs/HARDWARE.md** - Hardware specifications, pin assignments, I2C addresses
- **docs/TOOLCHAIN_VALIDATION.md** - Build and debug setup guide
- **standards/CODING_STANDARDS.md** - Code style and safety rules
