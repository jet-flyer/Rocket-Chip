# RocketChip Software Architecture Document
## Version 0.1 - Architectural Scaffolding

**Document Status:** Draft  
**Last Updated:** 2026-01-02  
**Target Platform:** RP2350 (Adafruit Feather HSTX w/ 8MB PSRAM)  
**Development Environment:** PlatformIO + Pico SDK + FreeRTOS  
**Hardware Reference:** `Agent_Instructions/HARDWARE.md` (authoritative source)

---

## 1. Executive Summary

This document defines the software architecture for RocketChip, a modular motion tracking platform. The architecture supports three product tiers (Core, Main, Titan) from a unified codebase, with features enabled via runtime detection or compile-time configuration.

### 1.1 MVP Target (Crowdfunding Demo)
- Main tier: Core board + GPS/Telemetry expansion
- Working flight state detection
- Live telemetry to ground station
- QGroundControl/Mission Planner compatibility
- Tested via bungee-launched glider for repeatability

### 1.2 Key Architectural Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| RTOS | FreeRTOS (all tiers) | Deterministic timing, task isolation, foundation for Titan features |
| Codebase | Unified | Single source, feature flags, easier maintenance |
| ArduPilot | Library shims only | Get proven math/calibration without ChibiOS port |
| Sensor Bus | I2C with Qwiic | Stress test high utilization; Qwiic on Core for easy sensor additions |
| Telemetry | MAVLink over LoRa | GCS compatibility, proven protocol |
| Logging | User-configurable | MAVLink binary default; CSV and MATLAB export options |
| Power | Always-on | ~30min runtime on 400mAh; ULP mode deferred |

---

## 2. System Architecture

### 2.1 Hardware Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                           CORE BOARD                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌───────────┐  │
│  │   RP2350    │  │  ICM20948   │  │   DPS310    │  │  Flash    │  │
│  │  Dual M33   │◄─┤  9-DoF IMU  │  │  Barometer  │  │  Storage  │  │
│  │  + 8MB PSRAM│  │  (I2C/SPI)  │  │   (I2C)     │  │  (QSPI)   │  │
│  └──────┬──────┘  └─────────────┘  └─────────────┘  └───────────┘  │
│         │                                                           │
│    ┌────┴────┐   ┌─────────────┐   ┌─────────────┐                 │
│    │  USB-C  │   │  NeoPixel   │   │  Buttons    │                 │
│    │  (CDC)  │   │   Status    │   │  (2x min)   │                 │
│    └─────────┘   └─────────────┘   └─────────────┘                 │
│         │                                                           │
│    ┌────┴──────────────────────────────────────┐                   │
│    │         EXPANSION CONNECTOR               │◄── Castellated    │
│    │   I2C (Qwiic) | SPI | UART | GPIO | PWR   │    or header      │
│    └───────────────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│ MERCURY PACK  │    │  JUNO PACK    │    │ VULCAN PACK   │
│  (Telemetry)  │    │    (GPS)      │    │  (Pyro/TVC)   │
├───────────────┤    ├───────────────┤    ├───────────────┤
│  RFM95/96     │    │  PA1010D or   │    │  Pyro Ch x2   │
│  LoRa Radio   │    │  similar GPS  │    │  Servo x2     │
│  915MHz/868MHz│    │  + Backup Baro│    │  Arm Switch   │
└───────────────┘    └───────────────┘    └───────────────┘
        │                                         │
        └────── MVP REQUIREMENT ──────────────────┘
```

### 2.2 Hardware Specifications

#### Current Prototype Components

| Function | Part | Adafruit P/N | Specs | Interface |
|----------|------|--------------|-------|-----------|
| MCU | Feather RP2350 HSTX | #6130 | Dual M33 @ 150MHz, 520KB SRAM, 8MB PSRAM | - |
| IMU | ICM-20948 | #4554 | 9-DoF, ±16g accel, ±2000°/s gyro, mag | I2C (0x68/0x69) |
| Barometer | DPS310 | #4494 | ±1Pa precision, temperature | I2C (0x77/0x76) |
| Battery | Li-Ion 400mAh | #3898 | 3.7V nominal | JST-PH |

#### Booster Pack Components (MVP)

| Pack | Part | Adafruit P/N | Specs | Interface |
|------|------|--------------|-------|-----------|
| Juno (GPS) | PA1010D Mini GPS | #4415 | 10Hz, -165dBm sensitivity | I2C (0x10) |
| Mercury (Telemetry) | LoRa FeatherWing 915MHz | #3179 | 915MHz ISM, ~2km range | SPI |

#### Titan Tier Upgrades (Future)

| Part | Adafruit P/N | Purpose |
|------|--------------|---------|
| ADXL375 High-G | #5374 | ±200g for high-power rockets |
| BMP580 | #6407 | 2cm noise @ 85Hz, baro upgrade |

#### I2C Address Map

| Address | Device | Notes |
|---------|--------|-------|
| 0x68/0x69 | ICM-20948 | Primary IMU |
| 0x77/0x76 | DPS310 | Barometer |
| 0x10 | PA1010D | GPS (Juno pack) |
| 0x6A/0x6B | LSM6DSOX | Auxiliary IMU (if used) |
| 0x1C/0x1E | LIS3MDL | Auxiliary mag (if used) |

**Known Conflicts:** DPS310 and BMP280/BMP580 share 0x77/0x76 - cannot use simultaneously without multiplexer.

### 2.3 Software Layer Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌───────────┐  │
│  │   Mission   │  │   Mission   │  │   Mission   │  │  Custom   │  │
│  │   Rocket    │  │   Glider    │  │    HAB      │  │  Mission  │  │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬─────┘  │
│         └────────────────┴────────────────┴───────────────┘        │
│                                 │                                   │
│                                 ▼                                   │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                      MISSION ENGINE                          │   │
│  │  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌─────────────┐  │   │
│  │  │   State   │ │   Event   │ │  Action   │ │  Condition  │  │   │
│  │  │  Machine  │ │  Engine   │ │ Executor  │ │  Evaluator  │  │   │
│  │  └───────────┘ └───────────┘ └───────────┘ └─────────────┘  │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                                 │
┌─────────────────────────────────────────────────────────────────────┐
│                        SERVICES LAYER                               │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐           │
│  │  Sensor   │ │   Data    │ │ Telemetry │ │    UI     │           │
│  │  Fusion   │ │  Logger   │ │  Service  │ │  Service  │           │
│  └───────────┘ └───────────┘ └───────────┘ └───────────┘           │
│                                                                     │
│  ┌───────────┐ ┌───────────────────────────────────────┐           │
│  │  Control  │ │  ArduPilot Compat (AP_Math, Filters)  │           │
│  │   Loop    │ │         via shim layer                │           │
│  └───────────┘ └───────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────────┘
                                 │
┌─────────────────────────────────────────────────────────────────────┐
│                    HARDWARE ABSTRACTION LAYER                       │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐           │
│  │    IMU    │ │   Baro    │ │    GPS    │ │   Radio   │           │
│  │  Driver   │ │  Driver   │ │  Driver   │ │  Driver   │           │
│  └───────────┘ └───────────┘ └───────────┘ └───────────┘           │
│                                                                     │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐           │
│  │  Storage  │ │  Display  │ │   Pyro    │ │   Servo   │           │
│  │  Driver   │ │  Driver   │ │  Driver   │ │  Driver   │           │
│  └───────────┘ └───────────┘ └───────────┘ └───────────┘           │
└─────────────────────────────────────────────────────────────────────┘
                                 │
┌─────────────────────────────────────────────────────────────────────┐
│                         PLATFORM LAYER                              │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                       FreeRTOS                               │   │
│  │   Tasks | Queues | Semaphores | Timers | Memory Management  │   │
│  └─────────────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    Pico SDK / RP2350                         │   │
│  │   GPIO | I2C | SPI | UART | DMA | PIO | Flash | USB         │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. Module Decomposition

### 3.1 Directory Structure

```
rocketchip/
├── CMakeLists.txt                 # Primary build system
├── platformio.ini                 # PlatformIO configuration
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
│   ├── hal/                       # Hardware Abstraction
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
├── test/                          # Unit tests
│   ├── test_state_machine.cpp
│   ├── test_condition.cpp
│   └── test_event_engine.cpp
│
└── docs/                          # Documentation
    ├── SAD.md                     # This document
    ├── API.md                     # API reference
    └── Missions.md                # Mission format specification
```

### 3.2 Module Responsibilities

| Module | Responsibility | Dependencies |
|--------|----------------|--------------|
| **MissionEngine** | Load missions, coordinate subsystems, manage lifecycle | StateMachine, EventEngine, ActionExecutor |
| **StateMachine** | Track current state, validate transitions, enforce timeouts | None (standalone) |
| **EventEngine** | Evaluate conditions against sensor data, fire events | Condition, SensorData |
| **ActionExecutor** | Execute actions (log, beep, LED, pyro, etc.) | HAL drivers |
| **Condition** | Parse and evaluate condition expressions | AP_Math |
| **SensorTask** | Sample IMU/Baro/GPS at configured rates | HAL drivers |
| **FusionTask** | AHRS, altitude estimation, velocity integration | AP_Math, Filter |
| **LoggerTask** | Buffer data, write to flash, manage pre-launch buffer | Storage HAL |
| **TelemetryTask** | Encode MAVLink, transmit via radio | Radio HAL, MAVLink |
| **UITask** | Update display, handle buttons, drive LED patterns | Display, LED, Buttons HAL |

---

## 4. Interface Definitions

### 4.1 Core Data Structures

```cpp
// Shared sensor data (protected by mutex)
struct SensorData {
    // IMU (updated at IMU_RATE, e.g., 1kHz)
    Vector3f accel;           // m/s² in body frame
    Vector3f gyro;            // rad/s in body frame
    Vector3f mag;             // µT (if available)
    uint32_t imu_timestamp_us;
    
    // Barometer (updated at BARO_RATE, e.g., 50Hz)
    float pressure_pa;
    float temperature_c;
    uint32_t baro_timestamp_us;
    
    // GPS (updated at GPS_RATE, e.g., 10Hz)
    bool gps_valid;
    double latitude_deg;
    double longitude_deg;
    float altitude_msl_m;
    float ground_speed_mps;
    float course_deg;
    uint8_t satellites;
    uint32_t gps_timestamp_us;
};

// Fused state estimates
struct FusedState {
    // Attitude
    Quaternion attitude;
    Vector3f euler_deg;       // roll, pitch, yaw
    
    // Position/velocity
    float altitude_agl_m;     // Above ground level (launch point)
    float altitude_msl_m;     // Above mean sea level
    float velocity_z_mps;     // Vertical velocity (+ up)
    float velocity_xy_mps;    // Horizontal velocity magnitude
    
    // Derived/tracked
    float max_altitude_m;
    float max_velocity_mps;
    float max_accel_g;
    
    uint32_t timestamp_us;
};

// Mission state
struct MissionState {
    uint8_t current_state;
    uint8_t previous_state;
    uint32_t state_entry_time_ms;
    uint32_t mission_start_time_ms;
    
    bool is_armed;
    bool is_logging;
    bool is_transmitting;
    
    // Flight markers
    uint32_t launch_time_ms;
    uint32_t apogee_time_ms;
    uint32_t landing_time_ms;
    float apogee_altitude_m;
};
```

### 4.2 HAL Interfaces

```cpp
// IMU interface - implementations: IMU_ICM20948, IMU_LSM6DSO, etc.
class IMU {
public:
    virtual bool begin() = 0;
    virtual bool read(Vector3f& accel, Vector3f& gyro, Vector3f& mag) = 0;
    virtual void setAccelRange(uint8_t g) = 0;
    virtual void setGyroRange(uint16_t dps) = 0;
    virtual void setSampleRate(uint16_t hz) = 0;
    virtual void calibrate() = 0;
    virtual ~IMU() = default;
};

// Barometer interface
class Baro {
public:
    virtual bool begin() = 0;
    virtual bool read(float& pressure_pa, float& temperature_c) = 0;
    virtual void setOversample(uint8_t rate) = 0;
    virtual ~Baro() = default;
};

// GPS interface
class GPS {
public:
    virtual bool begin() = 0;
    virtual bool read(GPSData& data) = 0;
    virtual bool hasValidFix() = 0;
    virtual ~GPS() = default;
};

// Radio interface
class Radio {
public:
    virtual bool begin() = 0;
    virtual bool send(const uint8_t* data, size_t len) = 0;
    virtual int receive(uint8_t* buffer, size_t max_len) = 0;
    virtual int getRSSI() = 0;
    virtual void setFrequency(float mhz) = 0;
    virtual void setPower(int8_t dbm) = 0;
    virtual ~Radio() = default;
};

// Storage interface
class Storage {
public:
    virtual bool begin() = 0;
    virtual bool write(const void* data, size_t len) = 0;
    virtual bool read(void* buffer, size_t len, size_t offset) = 0;
    virtual bool flush() = 0;
    virtual size_t getFreeSpace() = 0;
    virtual bool format() = 0;
    virtual ~Storage() = default;
};
```

### 4.3 Inter-Task Communication

```cpp
// FreeRTOS primitives
extern SemaphoreHandle_t g_sensorDataMutex;
extern SemaphoreHandle_t g_fusedStateMutex;
extern SemaphoreHandle_t g_missionStateMutex;

// Queues
extern QueueHandle_t g_actionQueue;      // MissionTask -> ActionExecutor
extern QueueHandle_t g_logQueue;         // All tasks -> LoggerTask
extern QueueHandle_t g_telemetryQueue;   // Selected data -> TelemetryTask
extern QueueHandle_t g_eventQueue;       // External events -> MissionTask

// Action message format
struct ActionMessage {
    uint8_t action_type;      // ACTION_BEEP, ACTION_LED, ACTION_LOG, etc.
    int32_t param1;
    int32_t param2;
    uint32_t timestamp_ms;
};

// Log message format
struct LogMessage {
    uint8_t msg_type;         // LOG_SENSOR, LOG_STATE, LOG_EVENT, etc.
    uint32_t timestamp_us;
    uint8_t data[64];         // Payload
    uint8_t data_len;
};
```

---

## 5. Task Architecture

### 5.1 Task Priorities and Rates

| Task | Priority | Rate | Stack | Core | Notes |
|------|----------|------|-------|------|-------|
| SensorTask | 5 (highest) | 1kHz | 1KB | 0 | Hard real-time, DMA preferred |
| ControlTask | 5 | 500Hz | 1KB | 0 | Only active during BOOST (Titan) |
| FusionTask | 4 | 200Hz | 2KB | 1 | AHRS, altitude calc |
| MissionTask | 4 | 100Hz | 2KB | 1 | State machine, events |
| LoggerTask | 3 | 50Hz | 2KB | 1 | Buffered writes |
| TelemetryTask | 2 | 10Hz | 1KB | 1 | MAVLink over LoRa |
| UITask | 1 (lowest) | 30Hz | 1KB | 1 | Display, LEDs, buttons |

### 5.2 Dual-Core Strategy

The RP2350's dual Cortex-M33 cores enable clean separation:

**Core 0 (Real-Time):**
- SensorTask: Deterministic sensor sampling
- ControlTask: PID loops with guaranteed timing (Titan only)
- Minimal ISRs for timing-critical operations

**Core 1 (Application):**
- FusionTask, MissionTask, LoggerTask, TelemetryTask, UITask
- All non-timing-critical processing
- Can tolerate jitter

### 5.3 Task Flow Diagram

```
                    ┌─────────────────────────────────────────┐
                    │              CORE 0                     │
                    │  ┌─────────────┐   ┌─────────────┐     │
                    │  │ SensorTask  │   │ ControlTask │     │
                    │  │   1kHz      │   │   500Hz     │     │
                    │  └──────┬──────┘   └──────┬──────┘     │
                    │         │                  │            │
                    └─────────┼──────────────────┼────────────┘
                              │                  │
                              ▼                  ▼
                    ┌─────────────────────────────────────────┐
                    │         SHARED DATA (mutex protected)   │
                    │   SensorData | FusedState | MissionState│
                    └─────────────────────────────────────────┘
                              │
                    ┌─────────┴─────────────────────────────────┐
                    │                    CORE 1                  │
                    │                                            │
                    │  ┌─────────────┐   ┌─────────────┐        │
                    │  │ FusionTask  │──▶│ MissionTask │        │
                    │  │   200Hz     │   │   100Hz     │        │
                    │  └─────────────┘   └──────┬──────┘        │
                    │                           │               │
                    │         ┌─────────────────┼───────────┐   │
                    │         ▼                 ▼           ▼   │
                    │  ┌───────────┐   ┌───────────┐ ┌────────┐│
                    │  │LoggerTask │   │TelemTask  │ │ UITask ││
                    │  │   50Hz    │   │   10Hz    │ │  30Hz  ││
                    │  └───────────┘   └───────────┘ └────────┘│
                    │                                           │
                    └───────────────────────────────────────────┘
```

---

## 6. State Machine

### 6.1 Default States (Rocket Mission)

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│    ┌──────┐  arm_cmd   ┌───────┐  launch   ┌───────┐            │
│    │ IDLE │───────────▶│ ARMED │──────────▶│ BOOST │            │
│    └──┬───┘            └───┬───┘           └───┬───┘            │
│       │                    │                   │                 │
│       │◀── disarm_cmd ─────┘                   │ burnout        │
│       │◀── timeout (180s) ─┘                   ▼                │
│       │                                   ┌───────┐              │
│       │                                   │ COAST │              │
│       │                                   └───┬───┘              │
│       │                                       │ apogee           │
│       │                                       ▼                  │
│       │                                  ┌─────────┐             │
│       │                                  │ DESCENT │             │
│       │                                  └────┬────┘             │
│       │                                       │ landing          │
│       │                                       ▼                  │
│       │◀───────── reset_cmd ───────────┌────────┐               │
│       │                                │ LANDED │               │
│       │                                └────────┘               │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### 6.2 State Definitions

| State | Entry Condition | Exit Conditions | Actions on Entry |
|-------|-----------------|-----------------|------------------|
| IDLE | Power on, reset | arm_cmd | LED: breathing blue |
| ARMED | Button hold 2s | disarm, timeout 3min, launch | LED: solid amber, start pre-buffer |
| BOOST | accel_z > 2.5g sustained 50ms | burnout (accel_z < 1.2g) | Start logging, mark LAUNCH |
| COAST | burnout detected | apogee (vel_z < 0 and baro falling) | Mark BURNOUT |
| DESCENT | apogee detected | landing (accel ~1g sustained 5s) | Mark APOGEE |
| LANDED | landing detected | reset_cmd | Stop logging, LED: green, beep x5 |

### 6.3 Event-Condition-Action Examples

```ini
# Event definitions
launch = "state:ARMED AND accel_z > 2.5 AND sustained:50ms"
         -> set_state:BOOST, start_log, mark:LAUNCH, led:red_flash

burnout = "state:BOOST AND accel_z < 1.2"
          -> set_state:COAST, mark:BURNOUT

apogee = "state:COAST AND velocity_z < 0 AND baro_falling:3"
         -> set_state:DESCENT, mark:APOGEE, beep:2

landing = "state:DESCENT AND accel_mag < 1.1 AND sustained:5000ms"
          -> set_state:LANDED, stop_log, led:green, beep:5
```

---

## 7. Build Configuration

### 7.1 Feature Flags

```cpp
// include/rocketchip/features.h

// Product tier (set at compile time or detected)
#define ROCKETCHIP_CORE    1
#define ROCKETCHIP_MAIN    2
#define ROCKETCHIP_TITAN   3

#ifndef ROCKETCHIP_TIER
#define ROCKETCHIP_TIER    ROCKETCHIP_MAIN  // Default
#endif

// Feature detection (runtime where possible)
#ifndef HAS_DISPLAY
#define HAS_DISPLAY        (ROCKETCHIP_TIER >= ROCKETCHIP_MAIN)
#endif

#ifndef HAS_GPS
#define HAS_GPS            0  // Detected at runtime via I2C scan
#endif

#ifndef HAS_RADIO
#define HAS_RADIO          0  // Detected at runtime
#endif

#ifndef HAS_PYRO
#define HAS_PYRO           (ROCKETCHIP_TIER == ROCKETCHIP_TITAN)
#endif

#ifndef HAS_TVC
#define HAS_TVC            (ROCKETCHIP_TIER == ROCKETCHIP_TITAN)
#endif

// Compile-time feature exclusion for memory savings
#ifndef ENABLE_TELEMETRY
#define ENABLE_TELEMETRY   1
#endif

#ifndef ENABLE_MAVLINK
#define ENABLE_MAVLINK     1
#endif

#ifndef ENABLE_CONTROL_LOOP
#define ENABLE_CONTROL_LOOP (ROCKETCHIP_TIER == ROCKETCHIP_TITAN)
#endif
```

### 7.2 PlatformIO Environments

```ini
; platformio.ini

[env]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
framework = 
board = pico2
board_build.core = earlephilhower

build_flags = 
    -std=c++17
    -DPICO_BOARD=pico2
    -DUSE_FREERTOS=1

lib_deps = 
    freertos/FreeRTOS-Kernel

; === CORE (minimal) ===
[env:core]
build_flags = 
    ${env.build_flags}
    -DROCKETCHIP_TIER=1
    -DENABLE_TELEMETRY=0
    -DENABLE_DISPLAY=0

; === MAIN (standard) ===
[env:main]
build_flags = 
    ${env.build_flags}
    -DROCKETCHIP_TIER=2

; === TITAN (full features) ===
[env:titan]
build_flags = 
    ${env.build_flags}
    -DROCKETCHIP_TIER=3
    -DHAS_PYRO=1
    -DHAS_TVC=1
    -DENABLE_CONTROL_LOOP=1

; === Development/Debug ===
[env:dev]
extends = env:main
build_type = debug
build_flags = 
    ${env:main.build_flags}
    -DDEBUG=1
    -DLOG_LEVEL=4
```

---

## 8. Data Flow

### 8.1 Sensor to Telemetry Pipeline

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│    IMU      │     │    Baro     │     │    GPS      │
│  ICM20948   │     │   DPS310    │     │  PA1010D    │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │ 1kHz              │ 50Hz              │ 10Hz
       └───────────────────┴───────────────────┘
                           │
                           ▼
                  ┌─────────────────┐
                  │   SensorTask    │
                  │  (read, buffer) │
                  └────────┬────────┘
                           │ SensorData
                           ▼
                  ┌─────────────────┐
                  │   FusionTask    │
                  │ (AHRS, altitude)│
                  └────────┬────────┘
                           │ FusedState
              ┌────────────┼────────────┐
              ▼            ▼            ▼
       ┌───────────┐ ┌───────────┐ ┌───────────┐
       │MissionTask│ │LoggerTask │ │TelemetryTask
       │ (events)  │ │ (storage) │ │ (MAVLink) │
       └───────────┘ └───────────┘ └───────────┘
                           │            │
                           ▼            ▼
                     ┌──────────┐  ┌──────────┐
                     │  Flash   │  │   LoRa   │
                     │ Storage  │  │  Radio   │
                     └──────────┘  └──────────┘
```

### 8.2 Pre-Launch Buffer

To capture data before launch detection:

```
PSRAM Ring Buffer (e.g., 512KB = ~5 seconds at full rate)
┌────────────────────────────────────────────────────────────┐
│ [...older data...]│ current write ──▶ │ [wrap around]     │
└────────────────────────────────────────────────────────────┘

On launch detection:
1. Mark current position
2. Continue writing forward
3. On landing, flush entire buffer (pre + post) to flash
4. Pre-launch data preserved for analysis
```

### 8.3 Logging Format Configuration

Logging format is user-configurable via mission settings or runtime command.

#### Supported Formats

| Format | Extension | Use Case | GCS Compatible |
|--------|-----------|----------|----------------|
| **MAVLink Binary** | `.bin` | Default. Direct import to Mission Planner/QGC | ✅ Yes |
| **CSV** | `.csv` | Excel, Google Sheets, simple analysis | ❌ No |
| **MATLAB** | `.mat` | University/research, MATLAB import | ❌ No |

#### Format Selection

```cpp
enum class LogFormat : uint8_t {
    MAVLINK_BIN = 0,  // Default - GCS compatible
    CSV         = 1,  // Human-readable, spreadsheet import
    MATLAB_MAT  = 2,  // MATLAB .mat file (v5 format)
};
```

#### Implementation Notes

- **MAVLink Binary**: Native format, streams same packets as telemetry. Use MAVLink log tools for analysis.
- **CSV**: Header row with field names, one sample per line. Larger files but universal compatibility.
- **MATLAB**: MAT-file v5 format (widely compatible). Each field stored as named array. Requires MAT file writer library or post-flight conversion.

#### Recommended Workflow

1. **Flight logging**: MAVLink binary (smallest, fastest)
2. **Post-flight export**: Convert to CSV or MATLAB via USB tool
3. **Direct logging**: CSV for simple projects, MATLAB for research

#### Export Tool (Future)

USB command interface will support:
```
export csv <flight_id>      # Export as CSV
export matlab <flight_id>   # Export as MATLAB .mat
export mavlink <flight_id>  # Re-export as MAVLink (default)
```

---

## 9. Memory Budget

### 9.1 RAM Allocation (520KB SRAM + 8MB PSRAM)

| Component | SRAM | PSRAM | Notes |
|-----------|------|-------|-------|
| FreeRTOS kernel | 16KB | - | Heap, TCBs |
| Task stacks | 12KB | - | 7 tasks × 1-2KB |
| Shared data structures | 4KB | - | Sensor, Fused, Mission state |
| DMA buffers | 8KB | - | I2C/SPI transfers |
| MAVLink buffers | 4KB | - | TX/RX buffers |
| USB buffers | 4KB | - | CDC serial |
| Pre-launch buffer | - | 512KB | Ring buffer |
| Flight log buffer | - | 4MB | Before flush to flash |
| **Available** | ~470KB | ~3.5MB | Headroom |

### 9.2 Flash Allocation (4MB typical)

| Region | Size | Contents |
|--------|------|----------|
| Bootloader | 16KB | UF2 bootloader |
| Firmware | 512KB | Application code |
| Calibration | 16KB | Sensor offsets, scales |
| Built-in missions | 64KB | Compiled mission data |
| User missions | 128KB | User-defined missions |
| Configuration | 16KB | Settings, preferences |
| Flight logs | ~3.2MB | LittleFS filesystem |

---

## 10. Development Phases

### Phase 1: Foundation (Weeks 1-2)
- [ ] PlatformIO project setup with FreeRTOS
- [ ] Basic HAL: GPIO, I2C, SPI initialization
- [ ] LED driver (NeoPixel)
- [ ] Button handling with debounce
- [ ] Serial debug output

### Phase 2: Sensors (Weeks 3-4)
- [ ] IMU driver (ICM20948)
- [ ] Barometer driver (DPS310)
- [ ] SensorTask implementation
- [ ] Basic sensor data logging to serial

### Phase 3: Core Logic (Weeks 5-6)
- [ ] StateMachine implementation
- [ ] Condition parser/evaluator
- [ ] EventEngine
- [ ] ActionExecutor (LED, beep)
- [ ] Basic rocket mission

### Phase 4: Storage (Weeks 7-8)
- [ ] Flash filesystem (LittleFS)
- [ ] LoggerTask implementation
- [ ] Pre-launch ring buffer
- [ ] USB data download

### Phase 5: Fusion (Weeks 9-10)
- [ ] AP_Math integration
- [ ] Altitude estimation
- [ ] Velocity integration
- [ ] FusionTask implementation

**=== MVP CHECKPOINT: Internal logging + state detection ===**

### Phase 6: GPS (Week 11)
- [ ] GPS driver
- [ ] Position logging
- [ ] Juno pack detection

### Phase 7: Telemetry (Weeks 12-13)
- [ ] MAVLink encoder
- [ ] LoRa driver (RFM95)
- [ ] TelemetryTask
- [ ] QGroundControl testing

### Phase 8: Polish (Week 14)
- [ ] Display driver (optional OLED)
- [ ] Menu system
- [ ] Calibration workflow
- [ ] Documentation

**=== MVP+ CHECKPOINT: GPS + Telemetry demo-ready ===**

### Future (Titan):
- [ ] ControlTask (PID)
- [ ] Servo driver
- [ ] Pyro driver + safety interlocks
- [ ] AP_AHRS_DCM integration

---

## 11. Open Questions

### Resolved

| Question | Decision |
|----------|----------|
| Log format | User-configurable: MAVLink binary (default), CSV, MATLAB export |
| Power management | Always-on for MVP (~30min on 400mAh). ULP deferred. |
| Glider test mission | Not needed - glide phase = descent/recovery phase |
| Product naming | Core, Main+Packs, Titan confirmed. Nova reserved. |
| Sensor bus | I2C with Qwiic connector on Core for expandability |

### Still Open

1. **Booster Pack detection**: EEPROM ID byte on each pack, GPIO sense pins, or I2C address scanning for known devices?

2. **Configuration storage**: EEPROM emulation in flash, or dedicated config partition in LittleFS?

3. **OTA updates**: Required for MVP? USB-only acceptable initially?

4. **USB command protocol**: Text-based CLI, or binary protocol, or both?

5. **Calibration persistence**: Store in same partition as config, or separate calibration region?

---

## 12. References

- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation)
- [ArduPilot Source](https://github.com/ArduPilot/ardupilot)
- [MAVLink Protocol](https://mavlink.io/en/)
- RocketChip Architecture v3 (internal)
- RocketChip Mission Engine Architecture v2 (internal)

---

*Document maintained in: `docs/SAD.md`*
