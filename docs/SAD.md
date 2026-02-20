# RocketChip Software Architecture Document (SAD) v2.0

**Status:** Active — High-Level Architecture Reference
**Last Updated:** 2026-02-20
**Target Platform:** RP2350 (Adafruit Feather HSTX w/ 8MB PSRAM)
**Development Environment:** CMake + Pico SDK
**Hardware Reference:** `docs/hardware/HARDWARE.md` (authoritative source)

> **Note:** This document describes the high-level software architecture. For implementation status, see `docs/PROJECT_STATUS.md`. For the step-by-step integration plan, see `docs/IVP.md`. For file-level structure, see `docs/SCAFFOLDING.md`.

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
| Architecture | Bare-metal Pico SDK | Simpler architecture, direct hardware control, no scheduler overhead |
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
| GPS | PA1010D Mini GPS | #4415 | 10Hz, -165dBm sensitivity | UART (preferred, 57600 baud) or I2C (0x10) |
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

> **Note:** Math utilities are custom implementations (`src/math/`): `mat.h` (header-only NxM matrix template), `vec3.cpp/.h`, `quat.cpp/.h`. CMSIS-DSP is not used — the custom template library was sufficient for ESKF 15x15 matrix operations.

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
│  │                    Pico SDK / RP2350                         │   │
│  │   GPIO | I2C | SPI | UART | DMA | PIO | Flash | USB         │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.4 Fault Handling
- **Watchdog:** RP2350 HW WDT enabled with 5s timeout via `watchdog_enable(5000, 1)`. Implemented in `main.cpp` (IVP-30).
- **Watchdog reboot detection:** Custom sentinel in scratch register 0 (`kWatchdogSentinel = 0x52435754`). SDK functions `watchdog_caused_reboot()` and `watchdog_enable_caused_reboot()` are both unreliable for this use case — see `.claude/LESSONS_LEARNED.md` and AGENT_WHITEBOARD.md (IVP-51) for details.
- **Hard fault handler:** `memmanage_fault_handler()` with LED blink pattern and intentional halt.
- **MPU stack guard:** Custom ARMv8-M MPU region on Core 0 stack bottom (IVP-29).
- **Recovery policy:** IVP-51 (Stage 6) will define mid-flight recovery: scratch register persistence, reboot counting with safe-mode lockout, ESKF state backoff.

---

## 3. Module Decomposition

### 3.1 Directory Structure

**Note:** The tree below shows the planned production architecture with modules not yet implemented. For the actual current filesystem, see `docs/SCAFFOLDING.md`.

```
rocketchip/
├── CMakeLists.txt                 # Primary build system (Pico SDK)
│
├── include/
│   └── rocketchip/                # Public headers
│       ├── config.h               # Build configuration, feature flags
│       ├── pins.h                 # GPIO assignments (from HARDWARE.md)
│       └── features.h             # Tier feature detection
│
├── src/
│   ├── main.cpp                   # Production entry point
│   │
│   ├── core/                      # Flight Director (Phase 5+)
│   │   ├── FlightDirector.*      # Top-level orchestrator
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
| **FlightDirector** | Load Mission Profile, coordinate subsystems, manage lifecycle | StateMachine, EventEngine, ActionExecutor |
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

### 3.3 FlightDirector

#### 3.3.1 States
- IDLE, ARMED, LAUNCH_DETECTED, ASCENT, APOGEE, DESCENT, LANDED.
- Visualization: Export to Graphviz DOT via tools/state_to_dot.py.

#### 3.3.2 Condition Evaluator
- Deferred: See PROJECT_STATUS.md Back Burner for compliant parser eval.

---

## 4. Interface Definitions

### 4.1 Core Data Structures

```cpp
// Shared sensor data (cross-core via seqlock double-buffer — see Section 4.3)
struct SensorData {
    // IMU (updated at IMU_RATE — target 1kHz, validate during IVP-25)
    Vector3f accel;           // m/s² in body frame
    Vector3f gyro;            // rad/s in body frame
    Vector3f mag;             // µT (if available)
    bool accel_valid;         // false if read failed or data stale
    bool gyro_valid;
    bool mag_valid;
    uint32_t imu_timestamp_us;

    // Barometer (updated at BARO_RATE — target 50Hz, validate during IVP-26)
    float pressure_pa;
    float temperature_c;
    bool baro_valid;          // false if read failed or data stale
    uint32_t baro_timestamp_us;

    // GPS (updated at GPS_RATE — target 10Hz, validate during IVP-33)
    bool gps_valid;           // false if no fix or read failed
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

### 4.3 Inter-Module Communication

In the bare-metal dual-core architecture, inter-module communication uses two patterns:

**Same-core (Core 0):** Direct function calls and global structs. Modules in the Core 0 superloop (fusion, mission, logger, telemetry, UI) share data through global data structures accessed synchronously — no locking needed since they execute cooperatively in a single loop.

**Cross-core (Core 1 → Core 0):** Seqlock double-buffer for sensor data. See `docs/decisions/SEQLOCK_DESIGN.md` for the full design rationale. See `docs/MULTICORE_RULES.md` for RP2350-specific inter-core rules.

#### Seqlock Pattern

Core 1 (writer) publishes sensor data lock-free. Core 0 (reader) retries if it detects a torn read. Uses `<stdatomic.h>` with explicit memory ordering — plain `volatile` is insufficient on ARM Cortex-M33 (LL Entry 8).

```c
// Shared between cores
typedef struct {
    _Atomic uint32_t sequence;    // Odd = write in progress, even = consistent
    SensorData buffers[2];        // Double buffer
} shared_sensor_t;

// Writer (Core 1) — called after each sensor read cycle
void publish_sensors(shared_sensor_t* shared, const SensorData* data) {
    uint32_t seq = atomic_load_explicit(&shared->sequence, memory_order_relaxed);
    atomic_store_explicit(&shared->sequence, seq + 1, memory_order_release);  // Odd → writing
    shared->buffers[((seq + 1) >> 1) & 1] = *data;                           // Write to inactive buffer
    atomic_store_explicit(&shared->sequence, seq + 2, memory_order_release);  // Even → done
}

// Reader (Core 0) — called from fusion/mission/UI modules
bool read_sensors(shared_sensor_t* shared, SensorData* out) {
    for (int retry = 0; retry < 4; retry++) {
        uint32_t seq1 = atomic_load_explicit(&shared->sequence, memory_order_acquire);
        if (seq1 & 1) continue;                    // Write in progress, retry
        *out = shared->buffers[(seq1 >> 1) & 1];   // Copy from active buffer
        uint32_t seq2 = atomic_load_explicit(&shared->sequence, memory_order_acquire);
        if (seq1 == seq2) return true;              // Consistent read
    }
    return false;  // Failed after retries (should be rare — <1%)
}
```

#### RP2350 Inter-Core Hardware Primitives

The RP2350 provides several hardware mechanisms for inter-core coordination, all accessed through the SIO (Single-cycle I/O) block:

| Primitive | Use in RocketChip | Notes |
|-----------|------------------|-------|
| **Hardware spinlocks** (32 via SIO) | Short critical sections if needed | IDs 24-31 available for app use; RP2350 errata E17 may force SW spinlocks |
| **Multicore FIFO** (4-entry, 32-bit) | Command passing between cores | Used internally by `multicore_lockout` — cannot overlap |
| **Doorbell interrupts** (RP2350-only) | Lightweight inter-core signaling | Alternative to polling seqlock sequence counter |
| **Multicore lockout** | Flash operation safety | Pauses Core 1 in RAM while Core 0 erases/programs flash |
| **Atomics** (`<stdatomic.h>`) | Seqlock sequence counter, flags | Required — `volatile` alone lacks HW memory barriers |
| **MPU (PMSAv8)** | Stack guard regions per core | 8 regions per core, 32-byte minimum alignment |

See `docs/IVP.md` Stage 3 for detailed verification steps exercising each primitive.

#### PIO State Machine Allocation

The RP2350 has 3 PIO blocks (PIO0, PIO1, PIO2) with 4 state machines each (12 total). PIO is a shared hardware resource that must be tracked:

| PIO Block | SM | Current Use | Notes |
|-----------|-----|------------|-------|
| PIO0 | SM0 | WS2812 NeoPixel | Allocated in `ws2812_status_init()` |
| PIO0 | SM1-3 | Available | |
| PIO1 | SM0-3 | Available | |
| PIO2 | SM0-3 | Available | |

**Future candidates for PIO:** SPI DMA for high-rate sensor mode, precision PWM for servos (Titan), custom protocols. Track allocations here as they are committed.

#### Action and Log Message Formats

```cpp
// Action message format (Core 0 internal, same-core only)
struct ActionMessage {
    uint8_t action_type;      // ACTION_BEEP, ACTION_LED, ACTION_LOG, etc.
    int32_t param1;
    int32_t param2;
    uint32_t timestamp_ms;
};

// Log message format (Core 0 internal, same-core only)
struct LogMessage {
    uint8_t msg_type;         // LOG_SENSOR, LOG_STATE, LOG_EVENT, etc.
    uint32_t timestamp_us;
    uint8_t data[64];         // Payload
    uint8_t data_len;
};
```

---

## 5. Execution Architecture

### 5.1 Bare-Metal Polling Architecture

RocketChip uses a bare-metal dual-core AMP (Asymmetric Multiprocessing) architecture. This replaces the previous FreeRTOS task model with a simpler, more deterministic approach. See `docs/decisions/SEQLOCK_DESIGN.md` for the council decision rationale.

> **Naming convention:** Module names like "SensorTask" and "FusionTask" are logical module names, not RTOS tasks. In the bare-metal architecture, Core 0 modules are functions called from a cooperative superloop. Core 1 runs a dedicated polling loop. The "Task" suffix is retained for continuity with the module decomposition in Section 3.2.

> **Numerical values below are preliminary targets** — each must be validated empirically during implementation. See `docs/IVP.md` for verification gates. Actual achievable rates depend on I2C transaction times, computation costs, and bus contention measured during bringup.

| Module | Target Rate | Core | Trigger | Notes |
|--------|-------------|------|---------|-------|
| Sensor sampling (IMU) | 1kHz | Core 1 | Tight polling loop (`time_us_64()`) | Rate limited by I2C transaction time — validate in IVP-25 |
| Sensor sampling (Baro) | 50Hz | Core 1 | Rate divider on IMU loop | Validate in IVP-26 |
| Sensor sampling (GPS) | 10Hz | Core 1 or Core 0 | Rate divider | GPS I2C reads are long — validate fit in IVP-33 |
| Control loop | 500Hz | Core 1 | Derived from sensor tick | Only active during BOOST (Titan) |
| Sensor fusion | 200Hz | Core 0 | Main loop polling | ESKF propagation + measurement updates |
| Flight Director | 100Hz | Core 0 | Main loop polling | State machine, events |
| Data logger | 50Hz | Core 0 | Main loop polling | Buffered writes |
| Telemetry | 10Hz | Core 0 | Main loop polling | MAVLink over LoRa |
| UI/CLI | 30Hz | Core 0 | Main loop polling | Display, LEDs, buttons |

### 5.2 Dual-Core Strategy

The RP2350's dual Cortex-M33 cores enable clean separation via AMP (Asymmetric Multiprocessing):

**Core 0 (Main Loop + USB):**
- Cooperative time-sliced superloop: fusion, mission, logging, telemetry, UI
- USB CDC serial (SDK-managed IRQ handlers registered on Core 0 by `stdio_init_all()`)
- All `printf`/USB I/O guarded by `stdio_usb_connected()`

**Core 1 (Sensor Sampling):**
- Tight polling loop using `time_us_64()` for deterministic timing (not timer ISR — avoids ISR jitter)
- IMU + baro reads on I2C1, published via seqlock to Core 0
- Control loop (Titan only)
- Calls `multicore_lockout_victim_init()` on entry (required for flash safety)
- Minimal processing to maintain deterministic timing

**Cross-core data flow:** Core 1 → seqlock double-buffer → Core 0. See Section 4.3.

### 5.3 Execution Flow Diagram

```
                    ┌─────────────────────────────────────────┐
                    │              CORE 1                     │
                    │  ┌──────────────────────────────────┐   │
                    │  │  Timer-Driven Sensor Sampling    │   │
                    │  │  1kHz repeating_timer callback   │   │
                    │  └──────────────┬───────────────────┘   │
                    │                 │                        │
                    └─────────────────┼────────────────────────┘
                                      │
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │      SHARED DATA (global structs)       │
                    │   SensorData | FusedState | MissionState│
                    └─────────────────────────────────────────┘
                                      │
                    ┌─────────────────┴────────────────────────┐
                    │              CORE 0                       │
                    │                                           │
                    │  ┌────────────────────────────────────┐   │
                    │  │        Polling Main Loop           │   │
                    │  │                                    │   │
                    │  │  if (fusion_due)   run_fusion();   │   │
                    │  │  if (mission_due)  run_mission();  │   │
                    │  │  if (logger_due)   run_logger();   │   │
                    │  │  if (telem_due)    run_telemetry();│   │
                    │  │  if (ui_due)       run_ui();       │   │
                    │  └────────────────────────────────────┘   │
                    │                                           │
                    └───────────────────────────────────────────┘
```

### 5.4 Sensor Fusion

> **Architecture Decision:** See `docs/decisions/ESKF/FUSION_ARCHITECTURE_DECISION.md` for full rationale.
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
Confidence Gate → signals → Flight Director (state estimate + confidence flag)

Independent AHRS (Mahony) runs alongside as cross-check → feeds Confidence Gate
```

#### Tier Differences (*filter counts and latencies pending review*)

| Tier | Filter Configuration | AHRS Cross-Check | Confidence Gate |
|------|---------------------|------------------|-----------------|
| Core | Single ESKF | Yes (Mahony) | No |
| Titan | MMAE bank of parallel ESKFs | Yes (Mahony) | Yes |
| Gemini | Dual MCU, each with MMAE bank | Yes | Yes + cross-MCU validation |

#### Implementation Foundation

- **Matrix math:** Custom `mat.h` header-only NxM template (not CMSIS-DSP)
- **Implemented:** ESKF class (`eskf.cpp`), Mahony AHRS (`mahony_ahrs.cpp`), WMM declination (`wmm_declination.cpp`)
- **Planned (Titan):** MMAE bank manager, confidence gate
- **NOT using:** ArduPilot EKF3, PX4 ECL, TinyEKF, Eigen, CMSIS-DSP, or any external filter/math framework

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

The project uses CMake with the Pico SDK.

**Build commands:**
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

**Current targets:**
- `rocketchip` - Main firmware (RP2350 target)
- `rocketchip_tests` - Host-side Google Test suite (187+ tests)
- `mat_benchmark` - Matrix math performance benchmark

**Feature flags:**
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
| Shared data structures | 4KB | - | Sensor, Fused, Mission state |
| DMA buffers | 8KB | - | I2C/SPI transfers |
| MAVLink buffers | 4KB | - | TX/RX buffers |
| USB buffers | 4KB | - | CDC serial |
| Pre-launch buffer | - | 512KB | Ring buffer |
| Flight log buffer | - | 4MB | Before flush to flash |
| **Available** | ~498KB | ~3.5MB | Headroom |

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

This section describes the high-level implementation roadmap. For the detailed 71-step integration plan with verification gates, see **`docs/IVP.md`**. For current status, see **`docs/PROJECT_STATUS.md`**.

| Phase | SAD Sections | IVP Stages | Status |
|-------|-------------|------------|--------|
| 1: Foundation | Build (7), Execution (5) | 1 (IVP-01–08) | **Complete** |
| 2: Sensors | Data Structures (4.1), Drivers (4.2), Storage (9) | 2 (IVP-09–18) | **Complete** |
| 3: GPS | GPS Interface (4.2) | 4 (IVP-31–33) | **Complete** |
| 4: Sensor Fusion | Fusion (5.4), FusedState (4.1) | 5 (IVP-39–50) | **In Progress** — Core ESKF + all measurements done. Optimization and Titan features pending |
| 5: Flight Director | State Machine (6), Modules (3.2) | 6 (IVP-51–56) | Planned |
| 6: Data Logging | Pre-Launch Buffer (8.2), Logging (8.3), Flash (9.3) | 7 (IVP-57–61) | Planned |
| 7: Telemetry | Radio Interface (4.2), Data Flow (8.1) | 8 (IVP-62–66) | Planned |
| 8: UI | All UI-related | — | Planned |
| 9: Polish & Testing | All sections | 9 (IVP-67–71) | Planned |
| Titan Features | Control Loop, Pyro, High-G | Titan-tier IVPs | Deferred |

Additional completed milestones not in the original phase plan:
- **Stage 3: Dual-Core** (IVP-19–30) — Core 1 sensor loop, seqlock, MPU stack guard, watchdog
- **Phase M: Mag Cal** (IVP-34–38) — Magnetometer calibration wizard, LM ellipsoid fit
- **I2C Bypass Mode** — ICM-20948 AK09916 direct reads (eliminates bank-switching race)
- **Standards Audit** — Manual (249 rules) + automated (clang-tidy, 1,251 findings remediated)
- **ESKF Divergence Fix** — Silent ICM-20948 zero-output detection + velocity sentinel

---

## 11. Tools and Validation

### 11.1 Validation Tools
- Graphviz: Script tools/state_to_dot.py parses FlightDirector states, generates DOT, outputs SVG via `dot -Tsvg states.dot -o states.svg`.
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
- Buffer overflow policy: Define per-buffer behavior (drop oldest/drop newest) — evaluate during implementation

---

## 13. Extensibility

### 13.1 Tier Differentiation

Feature sets are controlled at compile time via `ROCKETCHIP_TIER_*` defines:

- **Core/Middle tiers**: Qwiic I2C expansion, USB configuration, optional WiFi/BT (Pico 2 W)
- **Titan tier**: Full sensor suite, SPI high-rate mode, advanced fusion, dedicated expansion bus
- Mission plugins: User-defined missions loadable at runtime (format TBD post-MVP)

### 13.2 I2C Peripheral Detection and Driver Management

RocketChip automatically detects I2C devices connected via Qwiic/STEMMA QT — at boot and optionally at runtime. The goal is a Flipper Zero-style experience: plug in a sensor or peripheral, RocketChip identifies it, and either activates a built-in driver or prompts the user to download support for it.

This applies to **any I2C device** — standalone sensors (TOF, altimeter, extra IMU), displays, Booster Packs, or third-party Qwiic breakouts.

#### Boot-Time Detection (Implemented)

At startup, `init_sensors()` probes known I2C addresses before attempting driver initialization (implemented 2026-02-10). Absent devices are skipped — no wasted init, no bus side-effects.

```
Boot sequence:
  1. i2c_bus_init() — bus recovery + peripheral init
  2. Probe each known address (1-byte read, check ACK)
  3. Initialize only detected devices
  4. Report detected/missing in boot banner
```

The probe-first pattern prevents issues like LL Entry 28 (recovery triggered by NACK from absent device corrupting the bus).

#### Runtime Hot-Plug Detection (Planned)

**Status:** Crowdfunding stretch goal for Core/Middle tiers

Periodic I2C address scan detects newly connected or disconnected peripherals and notifies the user via CLI, status LED, or connected app.

```
Runtime detection loop (Core 0 idle time):
  1. Scan one I2C address per main loop tick (time-sliced)
  2. Compare results against last-known device set
  3. New device found:
     a. Known address → activate built-in driver, notify user
     b. Unknown address → identify if possible, prompt for driver
  4. Device removed → deactivate driver, update status
```

**Constraints:**
- Scan must NOT run while Core 1 owns the I2C bus for sensor sampling (see LL Entry 23)
- PA1010D GPS (0x10) requires special handling — excluded from generic scan per LL Entry 20
- Scan interval configurable (default 5s full sweep across valid address range)

#### Device Registry

A table maps I2C addresses to device identifiers and driver support:

| Address | Device | Driver | Category |
|---------|--------|--------|----------|
| 0x69 | ICM-20948 IMU | `icm20948.cpp` | Built-in |
| 0x77 | DPS310 Barometer | `baro_dps310.cpp` | Built-in |
| 0x10 | PA1010D GPS | `gps_pa1010d.cpp` | Built-in |
| 0x0C | AK09916 Magnetometer | via ICM-20948 | Built-in |
| 0x29 | VL53L0X TOF | — | Downloadable |
| 0x68 | ICM-20948 (AD0=LOW), BME280, MPU-6050, etc. | — | Downloadable |
| *others* | *Community peripherals* | — | *Catalog lookup* |

For addresses shared by multiple devices (e.g., 0x68), a WHO_AM_I register read or device-specific identification sequence disambiguates.

#### Firmware Updates for New Peripherals

When an unrecognized device is detected, RocketChip can acquire driver support through two paths:

**USB (all models):**
1. CLI displays: `New I2C device at 0x29 — no driver installed`
2. User downloads driver package from RocketChip catalog via PC
3. Flash update via USB includes the new driver

**OTA (WiFi/BT models — Pico 2 W and similar):**
1. Unknown address detected → user prompted: "New device at 0x29 (VL53L0X TOF). Download driver?"
2. Driver package downloaded over WiFi/BT → stored in flash filesystem
3. Device activated on next boot (or immediately if hot-plug is supported)

This mirrors the Flipper Zero model where plugging in new hardware prompts a firmware update that adds support for it.

**Implementation considerations:**
- Driver packages: pre-compiled modules or interpreted config blocks (TBD)
- Peripheral catalog: static JSON index + binary blobs, hosted with minimal infrastructure
- Security: signed packages with version pinning
- Titan tier ships with comprehensive built-in driver set; Core/Middle tiers rely on catalog for expanded peripheral support

### 13.3 Booster Pack API

Hardware abstraction for expansion packs. Booster Packs are a special case of Section 13.2 — multi-device assemblies identified by their I2C address signature (the set of detected addresses matching a known pack profile). A Booster Pack containing an IMU + barometer + flash chip would be identified by the combination of addresses, not any single one.

---

## 14. Gemini Redundant Architecture

Gemini is a carrier board configuration that pairs two Core modules for fault-tolerant operation. This section summarizes the software architecture; see `docs/hardware/GEMINI_CARRIER_BOARD.md` for full design documentation.

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

### 14.5 Watchdog Integration

On Gemini, the partner board provides a recovery path that doesn't exist on single-board configurations. The Gemini heartbeat failover window (1.5s) is shorter than the watchdog timeout (5s), so partner takeover completes before the WDT fires in all single-board failure cases. Per-board status LEDs (separate from NeoPixel) allow ground crew to visually identify which board has faulted. See `docs/hardware/GEMINI_CARRIER_BOARD.md` Sections 8.3-8.4 for cross-board watchdog handoff protocol, rejoin procedure, and LED patterns.

### 14.6 Pyro Safety

Hardware voting ensures safety independent of firmware state:

- **ARM requires consensus**: Both MCUs must assert ARM
- **FIRE allows either**: Once armed, either MCU can fire
- **Fail-safe**: Discrete logic operates even if both MCUs hang

### 14.7 Related Documents

- `docs/hardware/GEMINI_CARRIER_BOARD.md` - Full design documentation
- `docs/icd/EXPANSION_CONNECTOR_ICD.md` - Connector interface
- `docs/icd/GEMINI_PROTOCOL_ICD.md` - Message protocol
- `standards/protocols/SPACEWIRE_LITE.md` - Communication standard (aspirational)

---

## Appendix A: Phase-to-Section Cross-Reference

Quick reference for which SAD sections are critical for each development phase. For detailed integration steps, see `docs/IVP.md`.

| Phase | Critical SAD Sections | IVP Stage | Reference Sections |
|-------|----------------------|-----------|-------------------|
| **1: Foundation** | Build Config (7), Execution Architecture (5) | Stage 1 | Directory Structure (3.1), Fault Handling (2.4) |
| **2: Sensors** | Data Structures (4.1), Driver Interfaces (4.2), Storage (9.2-9.4) | Stages 2-3 | Hardware Specs (2.2), Data Flow (8) |
| **3: GPS** | GPS Interface (4.2) | Stage 4 | Data Flow (8) |
| **4: Fusion** | Sensor Fusion (5.4), FusedState (4.1), ESKF docs | Stage 5 | RAM Allocation (9.1) |
| **5: Flight Director** | State Machine (6), Module Responsibilities (3.2), Inter-Module Comm (4.3) | Stage 6 | FlightDirector (3.3) |
| **6: Logging** | Pre-Launch Buffer (8.2), Logging Format (8.3), Flash Layout (9.3) | Stage 7 | Storage Interface (4.2) |
| **7: Telemetry** | Radio Interface (4.2), Data Flow (8.1) | Stage 8 | Hardware Specs (2.2) |
| **8: UI** | All UI-related | — | ROCKETCHIP_OS.md |
| **9: Polish** | All sections | Stage 9 | All docs |

---

## 15. Open Questions

### Resolved

| Question | Decision |
|----------|----------|
| Log format | User-configurable: MAVLink binary (default), CSV, MATLAB export |
| Power management | Always-on for MVP (~30min on 400mAh). ULP deferred. |
| Glider test mission | Not needed - glide phase = descent/recovery phase |
| Product naming | Core, Main+Packs, Titan confirmed. Nova reserved. |
| Sensor bus | I2C (Qwiic) for Core expandability; UART preferred for GPS (see `docs/SENSOR_ARCHITECTURE.md`) |
| Configuration storage | AP_FlashStorage + StorageManager (Tier 1). See Section 9.2. |
| Calibration persistence | AP_FlashStorage CalibrationStore region (512B at offset 0). See Section 9.4. |
| Sensor fusion algorithm | Custom ESKF + MMAE bank (not AP EKF3 extraction). Council reviewed 2026-02-02. See `docs/decisions/ESKF/`. |

### Resolved

1. **Peripheral / Booster Pack detection**: I2C address scanning with device registry and WHO_AM_I disambiguation (Section 13.2). Booster Packs identified by address signature (Section 13.3). No EEPROM required.

2. **OTA updates**: USB-only for MVP. OTA driver downloads via WiFi/BT are a crowdfunding stretch goal for Core/Middle tiers with wireless connectivity (Section 13.2).

### Still Open

3. **USB command protocol**: Text-based CLI, or binary protocol, or both?

4. **State machine formalism**: Is FlightDirector a Mealy machine (actions on transitions) or Moore machine (actions on state entry)? Section 6.2 suggests Moore ("Actions on Entry"), but 6.3 event-condition-action syntax implies Mealy. Clarify before Phase 3 implementation.

---

## 16. References

- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Pico SDK Documentation](https://www.raspberrypi.com/documentation/pico-sdk/)
- [ArduPilot Source](https://github.com/ArduPilot/ardupilot)
- [MAVLink Protocol](https://mavlink.io/en/)
- RocketChip Architecture v3 (internal)
- RocketChip Flight Director Architecture v2 (internal)
- Solà, J. "Quaternion kinematics for the error-state Kalman filter" (2017) — ESKF reference
- Groves, P. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"

---

*Document maintained in: `docs/SAD.md`*
