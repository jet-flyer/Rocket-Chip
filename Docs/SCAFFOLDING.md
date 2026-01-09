# RocketChip Directory Structure Scaffolding

This document defines the target directory structure for the RocketChip firmware.
Created: 2026-01-09

## Directory Tree

```
rocketchip/
├── platformio.ini                 # PlatformIO build configuration
├── README.md                      # Project overview
│
├── docs/
│   ├── SAD.md                     # Software Architecture Document
│   ├── SCAFFOLDING.md             # This file
│   └── Missions.md                # Mission format specification (TBD)
│
├── include/
│   └── rocketchip/
│       ├── config.h               # Build configuration, feature flags
│       ├── version.h              # Version info
│       ├── pins.h                 # Pin definitions per board variant
│       └── features.h             # Feature detection macros
│
├── src/
│   ├── main.cpp                   # Entry point, task creation
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
│   │   ├── HAL.h                  # Abstract interfaces
│   │   ├── BoardDetect.h/.cpp     # Runtime board/pack detection
│   │   ├── IMU.h                  # IMU interface
│   │   ├── IMU_ICM20948.cpp       # ICM20948 driver
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
└── test/                          # Unit tests
    ├── test_state_machine.cpp
    ├── test_condition.cpp
    └── test_event_engine.cpp
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

## PlatformIO Environments

| Environment | Tier | Features |
|-------------|------|----------|
| `core` | Core | Minimal - local logging only |
| `main` | Main | GPS, telemetry capable |
| `titan` | Titan | Full features - pyro, TVC, high-G |
| `dev` | Main+Debug | Debug symbols, verbose logging |
| `test` | Native | Unit tests on host |

## Implementation Status

Header stubs created for:
- [x] config.h, pins.h
- [x] HAL.h, BoardDetect.h, IMU.h, Baro.h
- [x] StateMachine.h, EventEngine.h, ActionExecutor.h, MissionEngine.h
- [x] SensorTask.h, FusionTask.h, MissionTask.h, LoggerTask.h, UITask.h
- [x] TelemetryTask.h, ControlTask.h
- [x] AP_HAL_Compat.h
- [x] main.cpp (entry point)
- [x] platformio.ini

Implementation needed for:
- [ ] All .cpp files
- [ ] GPS.h, Radio.h, Storage.h, Display.h, LED.h, Buttons.h
- [ ] Pyro.h, Servo.h (Titan)
- [ ] protocol/ modules
- [ ] missions/ definitions
- [ ] utils/ helpers
- [ ] test/ unit tests

## Related Documents

- **SAD.md** - Software Architecture Document (full architecture details)
- **HARDWARE.md** - Hardware specifications, pin assignments, I2C addresses
- **STANDARDS.md** - Coding standards, safety requirements
