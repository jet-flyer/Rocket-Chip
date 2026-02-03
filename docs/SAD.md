# RocketChip Software Architecture Document (SAD) v2.0

**Status:** Fresh Start - Bespoke FreeRTOS Implementation
**Last Updated:** 2026-02-02
**Target Platform:** RP2350 (Adafruit Feather HSTX w/ 8MB PSRAM)
**Development Environment:** CMake + Pico SDK + FreeRTOS SMP
**Hardware Reference:** `docs/HARDWARE.md` (authoritative source)

> **Note:** This document describes the target architecture. Implementation is starting fresh after archiving previous ArduPilot integration attempts. See `docs/PROJECT_STATUS.md` for current progress.

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
| Sensor Fusion | Custom ESKF + MMAE | Faster than EKF3, no dependencies, community-extensible hypothesis libraries |
| Math/Utilities | CMSIS-DSP + custom | ARM-optimized matrix ops; may reference ArduPilot algorithms but not import code directly |
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
│  │   RP2350    │  │  ICM-20948  │  │   DPS310    │  │  Flash    │  │
│  │  Dual M33   │◄─┤  9-DoF IMU  │  │  Barometer  │  │  Storage  │  │
│  │  + 8MB PSRAM│  │   (I2C)     │  │   (I2C)     │  │  (QSPI)   │  │
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
│ TELEMETRY     │    │  GPS PACK     │    │  PYRO/TVC     │
│     PACK      │    │               │    │     PACK      │
├───────────────┤    ├───────────────┤    ├───────────────┤
│  RFM95W       │    │  PA1010D or   │    │  Pyro Ch x2   │
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
| IMU | ICM-20948 9-DoF | #4554 | Accel/gyro/mag (AK09916) | I2C (0x69 default) |
| Barometer | DPS310 | #4494 | ±1Pa precision, temperature | I2C (0x77/0x76) |
| Battery | Li-Ion 400mAh | #3898 | 3.7V nominal | JST-PH |

#### Booster Pack Components (MVP)

| Pack | Part | Adafruit P/N | Specs | Interface |
|------|------|--------------|-------|-----------|
| GPS | PA1010D Mini GPS | #4415 | 10Hz, -165dBm sensitivity | I2C (0x10) |
| Telemetry | RFM95W LoRa FeatherWing 915MHz | #3231 | 915MHz ISM, ~2km range | SPI |

#### Titan Tier Upgrades (Future)

| Part | Adafruit P/N | Purpose |
|------|--------------|---------|
| ADXL375 High-G | #5374 | ±200g for high-power rockets |
| BMP580 | #6407 | 2cm noise @ 85Hz, baro upgrade |

#### I2C Address Map

| Address | Device | Notes |
|---------|--------|-------|
| 0x69 | ICM-20948 | Primary 9-DoF IMU (Adafruit default, AD0=HIGH) |
| 0x77/0x76 | DPS310 | Barometer |
| 0x10 | PA1010D | GPS (GPS pack) |
| 0x6A/0x6B | ISM330DHCX | Auxiliary IMU (if used, FeatherWing #4569) |
| 0x1C/0x1E | LIS3MDL | Auxiliary magnetometer (if used) |

**Known Conflicts:** DPS310 and BMP280/BMP580 share 0x77/0x76 - cannot use simultaneously without multiplexer.

### 2.3 Software Layer Diagram

> **Note:** Math/utility library selection (CMSIS-DSP vs custom vs partial ArduPilot reference) is pending final decision during implementation.

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
│  │  Control  │ │  Math Utilities (CMSIS-DSP, custom)   │           │
│  │   Loop    │ │    Vectors, Quaternions, Matrices     │           │
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

### 2.4 Fault Handling
- Watchdog: RP2350 HW WDT enabled with 5s timeout via watchdog_enable(5000, 1) in production main.
  Pseudocode (to be implemented in production):
  ```
  // In production main
  #include <hardware/watchdog.h>
  watchdog_enable(5000, 1);  // 5s timeout, pause on debug

  // In each task loop
  watchdog_update();  // Kick the dog

  // On boot: Log reset cause
  if (watchdog_caused_reboot()) {
      log_error("WDT reset occurred");
  }
  ```
- Per-task hooks via FreeRTOS (configCHECK_FOR_STACK_OVERFLOW=2).
- Recovery: Pre-reset flash logging.

---

## 3. Module Decomposition

### 3.1 Directory Structure

**Current status:** Fresh start post-branch reorganization. Repository is clean; structure below shows the planned production architecture. See `docs/SCAFFOLDING.md` for implementation status tracking.

```
rocketchip/
├── CMakeLists.txt                 # Primary build system (Pico SDK + FreeRTOS)
├── FreeRTOSConfig.h               # FreeRTOS SMP configuration
│
├── include/
│   └── rocketchip/                # Public headers
│       ├── config.h               # Build configuration, feature flags
│       ├── pins.h                 # GPIO assignments (from HARDWARE.md)
│       └── features.h             # Tier feature detection
│
├── src/
│   ├── main.cpp                   # Production entry point
│   ├── hooks.cpp                  # FreeRTOS hooks (stack overflow, malloc fail)
│   │
│   ├── core/                      # Mission Engine (Phase 5+)
│   │   ├── MissionEngine.*        # Top-level orchestrator
│   │   ├── StateMachine.*         # State management
│   │   ├── EventEngine.*          # Event detection & dispatch
│   │   ├── ActionExecutor.*       # Action handling
│   │   └── ControlLoop.*          # PID control (Titan/TVC)
│   │
│   ├── hal/                       # Hardware Abstraction (Phase 1-2)
│   │   ├── Bus.*                  # I2C/SPI bus abstraction
│   │   ├── IMU_ICM20948.*         # ICM-20948 9-DoF driver
│   │   ├── Baro_DPS310.*          # DPS310 barometer driver
│   │   ├── GPS_PA1010D.*          # PA1010D GPS driver (NMEA)
│   │   ├── Radio_RFM95W.*         # RFM95W LoRa driver
│   │   ├── Storage.*              # Flash storage
│   │   └── LED.*                  # NeoPixel/status LED driver
│   │
│   ├── services/                  # FreeRTOS Tasks
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
│   ├── FreeRTOS-Kernel/           # FreeRTOS (git submodule)
│   ├── pico-sdk/                  # Pico SDK (git submodule or system)
│   └── mavlink/                   # MAVLink v2 headers (generated)
│
├── tests/                         # Validation Tests
│   ├── smoke/                     # Hardware smoke tests
│   │   ├── i2c_scan.cpp           # I2C device scanner
│   │   ├── imu_test.cpp           # IMU validation
│   │   ├── baro_test.cpp          # Barometer validation
│   │   └── radio_test.cpp         # Radio TX/RX validation
│   └── unit/                      # Unit tests (host-side, future)
│
└── docs/                          # Documentation
    ├── SAD.md                     # This document
    ├── SCAFFOLDING.md             # Implementation status
    ├── HARDWARE.md                # Hardware reference
    ├── PROJECT_OVERVIEW.md        # Vision and product tiers
    ├── PROJECT_STATUS.md          # Current phase and blockers
    ├── ESKF/                      # Sensor fusion architecture
    └── icd/                       # Interface Control Documents
```

### 3.2 Module Responsibilities

| Module | Responsibility | Dependencies |
|--------|----------------|--------------|
| **MissionEngine** | Load missions, coordinate subsystems, manage lifecycle | StateMachine, EventEngine, ActionExecutor |
| **StateMachine** | Track current state, validate transitions, enforce timeouts | None (standalone) |
| **EventEngine** | Evaluate conditions against sensor data, fire events | Condition, SensorData |
| **ActionExecutor** | Execute actions (log, beep, LED, pyro, etc.) | HAL drivers |
| **Condition** | Parse and evaluate condition expressions | Math utilities |
| **SensorTask** | Sample IMU/Baro/GPS at configured rates | HAL drivers |
| **FusionTask** | ESKF/AHRS, altitude estimation, velocity integration | Math utilities, CMSIS-DSP |
| **LoggerTask** | Buffer data, write to flash, manage pre-launch buffer | Storage HAL |
| **TelemetryTask** | Encode MAVLink, transmit via radio | Radio HAL, MAVLink |
| **UITask** | Update display, handle buttons, drive LED/CLI | Display, LED HAL |

> **Note:** Math utilities (Vector3, Quaternion, Matrix) are custom implementations. May reference ArduPilot/Adafruit algorithms for correctness but not import code directly. CMSIS-DSP provides ARM-optimized primitives for matrix operations in ESKF.

### 3.3 MissionEngine

#### 3.3.1 States
- IDLE, ARMED, LAUNCH_DETECTED, ASCENT, APOGEE, DESCENT, LANDED.
- Visualization: Export to Graphviz DOT via tools/state_to_dot.py.

#### 3.3.2 Condition Evaluator
- Deferred: See PROJECT_STATUS.md Back Burner for compliant parser eval.

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

### 4.2 Driver Interfaces

> **Note:** These are RocketChip's own driver interfaces, not ArduPilot's AP_HAL. Implementations use Pico SDK directly.

```cpp
// IMU interface - implementations: IMU_ICM20948 (primary)
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

### 5.4 Sensor Fusion

> **Architecture Decision:** See `docs/ESKF/FUSION_ARCHITECTURE_DECISION.md` for full rationale.
> **Note:** Specific numerical parameters (state counts, filter counts, latency budgets) are pending systematic review.

RocketChip uses a custom **Error-State Kalman Filter (ESKF)** with **Multiple Model Adaptive Estimation (MMAE)** for anomaly resilience. This replaces the previously planned EKF3 extraction approach.

#### ESKF State Vector (15 elements — *pending review*)

| State Group     | Count | Description                           |
|-----------------|-------|---------------------------------------|
| Attitude error  | 3     | Rotation error vector (NOT quaternion) |
| Velocity error  | 3     | NED frame (m/s)                       |
| Position error  | 3     | NED frame (m)                         |
| Gyro bias       | 3     | rad/s                                 |
| Accel bias      | 3     | m/s²                                  |

The nominal state (full quaternion + position + velocity) is propagated separately through the nonlinear model. The ESKF tracks deviations from this nominal. After each measurement update, the error correction is folded back into the nominal state and the error state resets to zero.

#### Architecture (Three-Layer Separation)

```
Mission Config → selects → Hypothesis Library (validated process models)
Hypothesis Library → initializes → MMAE Filter Bank (parallel ESKFs)
MMAE Filter Bank → feeds → Confidence Gate (platform-level, NOT configurable)
Confidence Gate → signals → Mission Engine (state estimate + confidence flag)

Independent AHRS (Mahony) runs alongside as cross-check → feeds Confidence Gate
```

#### Tier Differences (*filter counts and latencies pending review*)

| Tier | Filter Configuration | AHRS Cross-Check | Confidence Gate |
|------|---------------------|------------------|-----------------|
| Core | Single ESKF | Yes (Mahony) | No |
| Titan | MMAE bank of parallel ESKFs | Yes (Mahony) | Yes |
| Gemini | Dual MCU, each with MMAE bank | Yes | Yes + cross-MCU validation |

#### Implementation Foundation

- **Matrix math:** CMSIS-DSP (ARM's optimized library for Cortex-M)
- **Custom code:** ESKF class, MMAE bank manager, confidence gate, hypothesis interface
- **NOT using:** ArduPilot EKF3, PX4 ECL, TinyEKF, Eigen, or any external filter framework

#### References
- Solà, J. "Quaternion kinematics for the error-state Kalman filter" (2017)
- Groves, P. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"

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

### 7.2 CMake Build System

The project uses CMake with the Pico SDK and FreeRTOS-Kernel as submodules.

**Build commands:**
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

**Current status:** Fresh start. Build system and targets will be defined as implementation proceeds.

**Planned targets:**
- `rocketchip` - Main application (production firmware)
- `i2c_scan` - I2C device scanner utility
- `smoke_*` - Hardware validation tests (per-sensor)

**Feature flags** (to be implemented):
```cmake
add_compile_definitions(
    ROCKETCHIP_TIER=2        # 1=Core, 2=Main, 3=Titan
    ENABLE_TELEMETRY=1
    ENABLE_LOGGING=1
)
```

---

## 8. Data Flow

### 8.1 Sensor to Telemetry Pipeline

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│    IMU      │     │    Baro     │     │    GPS      │
│  ICM-20948  │     │   DPS310    │     │  PA1010D    │
│  (9-DoF)    │     │             │     │             │
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

### 9.2 Storage Architecture

RocketChip uses a multi-tier storage model:

| Tier | Technology | Use Case | Characteristics |
|------|------------|----------|-----------------|
| **Tier 1** | Dual-sector wear-leveled storage | Calibration, config | Small, critical, survives firmware updates |
| **Tier 2A** | LittleFS (on-chip flash) | Flight logs, session data | Filesystem, power-safe, USB export |
| **Tier 2B** | FatFs (SD card) | Bulk export, extended logging | PC-readable, removable media (future) |

> **Key Technique (from ArduPilot):** Tier 1 uses a dual-sector log-structured approach where data is never overwritten in place. Instead, new values are appended until a sector fills, then the valid data is compacted to the alternate sector. This ensures calibration data survives both power loss and firmware updates.

### 9.3 Flash Memory Layout (8MB flash)

```
0x10000000  +-----------------+
            |   Bootloader    |  16KB (Pico SDK boot2)
0x10004000  +-----------------+
            |                 |
            |    Firmware     |  ~1MB (depends on build)
            |                 |
            +-----------------+
            | Persistent A    |  4KB  --+-- Tier 1: Calibration/Config
            +-----------------+         |   Dual-sector wear leveling
            | Persistent B    |  4KB  --+   (survives firmware updates)
            +-----------------+
            |                 |
            |    LittleFS     |  Remainder (~6.5MB)
            |  (Flight Logs)  |
            |                 |
0x10800000  +-----------------+  (End of 8MB)
```

**Note:** Exact addresses TBD during implementation. Firmware size and LittleFS partition will be finalized when build is functional.

### 9.4 Tier 1 Storage Layout

| Region | Offset | Size | Contents |
|--------|--------|------|----------|
| Calibration | 0 | 512B | Sensor offsets, scales, temperature coefficients |
| Config | 512 | 256B | Active mission ID, preferences |
| Reserved | 768 | ~3KB | Future expansion |

**Tier 1 characteristics:**
- Log-structured writes (append-only during operation)
- Automatic sector switching for wear leveling
- Power-loss safety (always recoverable state)
- Survives firmware updates (located at end of flash, outside firmware region)

---

## 10. Development Phases

This section tracks the implementation roadmap. See `docs/SCAFFOLDING.md` for detailed file-level status and `docs/PROJECT_STATUS.md` for current focus.

> **Note:** Starting fresh after archiving previous ArduPilot integration attempts. All items reset to planned state.

### Phase 1: Foundation 🔧 **CURRENT**
Build system, FreeRTOS, and minimal validation.

- [ ] CMake build system with Pico SDK + FreeRTOS submodules
- [ ] FreeRTOS SMP configuration (dual-core)
- [ ] Minimal `main.cpp` with FreeRTOS task structure
- [ ] USB CDC serial output (debug)
- [ ] LED status indicator (NeoPixel)
- [ ] I2C bus initialization
- [ ] I2C scanner smoke test
- [ ] Documentation updated for fresh start

### Phase 2: Sensors 📡 **PLANNED**
Direct sensor interfaces using Pico SDK (no HAL abstraction).

- [ ] ICM-20948 9-DoF IMU driver (I2C)
- [ ] DPS310 barometer driver (I2C)
- [ ] SensorTask (high-rate sampling)
- [ ] Sensor data structures
- [ ] Dual-sector persistent storage (Tier 1 - calibration/config) — *required for calibration persistence*
- [ ] Basic calibration workflow (accel, mag level)
- [ ] RC_OS CLI menu structure (see `docs/ROCKETCHIP_OS.md`)

### Phase 3: GPS Navigation 📍 **PLANNED**
GPS integration.

- [ ] PA1010D GPS driver (NMEA parsing)
- [ ] Position logging
- [ ] GPS pack runtime detection

### Phase 4: Sensor Fusion 🧭 **PLANNED**
ESKF navigation and state estimation. See `docs/ESKF/FUSION_ARCHITECTURE_DECISION.md`.

- [ ] Math utilities (Vector3, Quaternion, Matrix)
- [ ] ESKF core (15-state error-state filter) — *state count pending review*
- [ ] Independent AHRS cross-check (Mahony)
- [ ] FusionTask (200-400Hz) — *frequency pending review*
- [ ] FusedState data structure
- [ ] MMAE bank manager (Titan tier) — *filter count pending review*
- [ ] Confidence gate (Titan tier)

### Phase 5: Mission Engine 🚀 **PLANNED**
Core flight logic and state machine.

- [ ] StateMachine implementation
- [ ] EventEngine
- [ ] ActionExecutor (LED, beep, logging triggers)
- [ ] MissionTask (100Hz event processing)
- [ ] Basic rocket mission definition
- [ ] State transition validation

### Phase 6: Data Logging 💾 **PLANNED**
Flight log storage and pre-launch buffering. (Tier 1 storage implemented in Phase 2)

- [ ] LittleFS for flight logs (Tier 2A)
- [ ] LoggerTask (50Hz buffered writes)
- [ ] Pre-launch ring buffer (PSRAM)
- [ ] USB data download interface
- [ ] Log format configuration (MAVLink binary, CSV)

### Phase 7: Telemetry 📶 **PLANNED**
MAVLink telemetry and ground station integration.

- [ ] RFM95W LoRa radio driver
- [ ] MAVLink message encoder/decoder
- [ ] TelemetryTask (10Hz downlink)
- [ ] QGroundControl/Mission Planner compatibility

### Phase 8: User Interface 🖥️ **PLANNED**
Display, menus, and calibration.

- [ ] UITask (display/LED/button update)
- [ ] Menu system (mission select, calibration, settings)
- [ ] LED status patterns
- [ ] Button handling

### Phase 9: Polish & Testing 📚 **PLANNED**
Final integration and documentation.

- [ ] Full system integration testing
- [ ] Bench and flight test procedures
- [ ] User manual

### Future: Titan Tier Features 🔥 **DEFERRED**
Advanced features for high-power rocketry.

- [ ] ControlTask (500Hz PID for TVC)
- [ ] Servo driver (PIO-based PWM)
- [ ] Pyro driver with safety interlocks
- [ ] High-G accelerometer (ADXL375)
- [ ] Dual IMU cross-checking

---

## 11. Tools and Validation

### 11.1 Validation Tools
- Graphviz: Script tools/state_to_dot.py parses MissionEngine states, generates DOT, outputs SVG via `dot -Tsvg states.dot -o states.svg`.
  Integrate into Makefile: `make docs` target auto-gens.
  Pseudocode for state_to_dot():
  ```
  def state_to_dot(states):
      dot = "digraph MissionStates {\n"
      for state, transitions in states.items():
          for trans in transitions:
              dot += f'"{state}" -> "{trans.target}" [label="{trans.condition}"];\n'
      dot += "}\n"
      return dot
  ```

### 11.2 Debug Output

Development builds include serial debug output via USB-CDC for diagnosing timing and state issues.

**Full specification:** `standards/DEBUG_OUTPUT.md`

**Quick reference:**
- Debug macros compile out in release builds
- SensorTask/ControlTask: minimal prints (timing-critical)
- MissionTask: primary debugging target (state transitions, events)

---

## 12. Power and Performance

> **[PLACEHOLDER — To be completed during Phase 4 (Fusion) when power profiling is possible]**

- Budget: 400mAh for 30min — detailed power table TBD after hardware validation
- WCET: Analyze Sensor/Control tasks during Phase 2 implementation
- Queue overflow policy: Define per-queue behavior (block/drop oldest/drop newest) — evaluate during RTOS framework implementation

---

## 13. Extensibility

> **[PLACEHOLDER — To be expanded post-MVP when plugin architecture is designed]**

- Mission plugins: User-defined missions loadable at runtime (format TBD)
- Tier differentiation: Compile-time `#ifdef` for CORE/MAIN/TITAN feature sets
- Booster Pack API: Hardware abstraction for expansion packs (see Open Questions)

---

## 14. Gemini Redundant Architecture

Gemini is a carrier board configuration that pairs two Core modules for fault-tolerant operation. This section summarizes the software architecture; see `docs/GEMINI_CARRIER_BOARD.md` for full design documentation.

### 14.1 Overview

```
┌─────────────────┐         SpaceWire-Lite         ┌─────────────────┐
│    CORE A       │◄────────────────────────────▶│    CORE B       │
│   (Primary)     │         (10 Mbps LVDS)        │   (Secondary)   │
│                 │                                │                 │
│  Full Mission   │  ◄── State Sync ──────────▶  │  Hot Standby    │
│  Control        │       Heartbeat (10 Hz)       │  Monitoring     │
└────────┬────────┘                                └────────┬────────┘
         │                                                  │
         │ ARM_A                                    ARM_B   │
         │ FIRE_A                                   FIRE_B  │
         ▼                                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    HARDWARE VOTING LOGIC                            │
│   ARM_OUT = ARM_A AND ARM_B     FIRE_OUT = (FIRE_A OR FIRE_B)      │
│                                  AND ARM_OUT                        │
└─────────────────────────────────────────────────────────────────────┘
```

### 14.2 Software Components

| Component | Location | Function |
|-----------|----------|----------|
| GeminiTask | `src/services/GeminiTask.*` | Inter-MCU communication (future) |
| SpaceWireDriver | `src/hal/SpaceWire.*` | PIO-based DS encoding (future) |
| FailoverManager | `src/core/FailoverManager.*` | Role arbitration (future) |

### 14.3 Communication Protocol

Gemini uses SpaceWire-Lite, a subset of ECSS-E-ST-50-12C:

| Layer | Implementation | Notes |
|-------|----------------|-------|
| Physical | LVDS via SN65LVDS049 | GPIO for prototype |
| Data-Link | Data-Strobe encoding via PIO | Clock recovery = XOR |
| Network | **Not implemented** | Point-to-point only |

Protocol specification: `standards/protocols/SPACEWIRE_LITE.md`

### 14.4 Failover Behavior

| Event | Detection | Response | Timing |
|-------|-----------|----------|--------|
| Primary failure | 5 missed heartbeats | Secondary takes over | 1.5 s |
| Communication loss | No messages | Both continue independently | 500 ms |
| Sensor failure | Health flags | Cross-validate with partner | Immediate |

### 14.5 Pyro Safety

Hardware voting ensures safety independent of firmware state:

- **ARM requires consensus**: Both MCUs must assert ARM
- **FIRE allows either**: Once armed, either MCU can fire
- **Fail-safe**: Discrete logic operates even if both MCUs hang

### 14.6 Related Documents

- `docs/GEMINI_CARRIER_BOARD.md` - Full design documentation
- `docs/icd/EXPANSION_CONNECTOR_ICD.md` - Connector interface
- `docs/icd/GEMINI_PROTOCOL_ICD.md` - Message protocol
- `standards/protocols/SPACEWIRE_LITE.md` - Communication standard (aspirational)

---

## Appendix A: Phase-to-Section Cross-Reference

Quick reference for which SAD sections are critical for each development phase.

| Phase | Critical Sections | Reference Sections |
|-------|-------------------|-------------------|
| **1: Foundation** | Build Config (7), Task Architecture (5) | Directory Structure (3.1), Fault Handling (2.4) |
| **2: Sensors** | Data Structures (4.1), Driver Interfaces (4.2), Storage (9.2-9.4) | Hardware Specs (2.2), Data Flow (8) |
| **3: GPS** | GPS Interface (4.2) | Data Flow (8) |
| **4: Fusion** | Sensor Fusion (5.4), FusedState (4.1), ESKF docs | RAM Allocation (9.1) |
| **5: Mission Engine** | State Machine (6), Module Responsibilities (3.2), Inter-Task Comm (4.3) | MissionEngine (3.3) |
| **6: Logging** | Pre-Launch Buffer (8.2), Logging Format (8.3), Flash Layout (9.3) | Storage Interface (4.2) |
| **7: Telemetry** | Radio Interface (4.2), MAVLink (8.1) | Hardware Specs (2.2) |
| **8: UI** | All UI-related | ROCKETCHIP_OS.md |
| **9: Polish** | All sections | All docs |

---

## 14. Open Questions

### Resolved

| Question | Decision |
|----------|----------|
| Log format | User-configurable: MAVLink binary (default), CSV, MATLAB export |
| Power management | Always-on for MVP (~30min on 400mAh). ULP deferred. |
| Glider test mission | Not needed - glide phase = descent/recovery phase |
| Product naming | Core, Main+Packs, Titan confirmed. Nova reserved. |
| Sensor bus | I2C with Qwiic connector on Core for expandability |
| Configuration storage | AP_FlashStorage + StorageManager (Tier 1). See Section 9.2. |
| Calibration persistence | AP_FlashStorage CalibrationStore region (512B at offset 0). See Section 9.4. |
| Sensor fusion algorithm | Custom ESKF + MMAE bank (not AP EKF3 extraction). Council reviewed 2026-02-02. See `docs/ESKF/`. |

### Still Open

1. **Booster Pack detection**: EEPROM ID byte on each pack, GPIO sense pins, or I2C address scanning for known devices?

2. **OTA updates**: Required for MVP? USB-only acceptable initially?

3. **USB command protocol**: Text-based CLI, or binary protocol, or both?

4. **State machine formalism**: Is MissionEngine a Mealy machine (actions on transitions) or Moore machine (actions on state entry)? Section 6.2 suggests Moore ("Actions on Entry"), but 6.3 event-condition-action syntax implies Mealy. Clarify before Phase 3 implementation.

---

## 15. References

- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation)
- [ArduPilot Source](https://github.com/ArduPilot/ardupilot)
- [MAVLink Protocol](https://mavlink.io/en/)
- RocketChip Architecture v3 (internal)
- RocketChip Mission Engine Architecture v2 (internal)
- Solà, J. "Quaternion kinematics for the error-state Kalman filter" (2017) — ESKF reference
- Groves, P. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"

---

*Document maintained in: `docs/SAD.md`*
