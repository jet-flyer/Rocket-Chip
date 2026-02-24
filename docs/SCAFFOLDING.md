# RocketChip Directory Structure

**Created:** 2026-01-09
**Updated:** 2026-02-24

**Status:** Reflects actual filesystem as of Stage 5 (Sensor Fusion). Previous ArduPilot integration archived in `AP_FreeRTOS` and `AP_ChibiOS` branches.

## Build System

Pure CMake + Pico SDK (bare-metal, C++20). All source files are `.cpp`. Third-party libs (`ruuvi`, `lwGPS`) remain C.

## Directory Tree

```
rocketchip/
├── CMakeLists.txt                 # Primary build system (Pico SDK + Ninja)
├── pico_sdk_import.cmake          # Pico SDK integration
├── README.md                      # Agent instructions
├── CHANGELOG.md                   # Development history
├── AGENT_WHITEBOARD.md            # Cross-session communication
├── COUNCIL_PROCESS.md             # Panel review protocol
├── CROSS_AGENT_REVIEW.md          # Inter-agent review protocol
├── LICENSE
│
├── docs/
│   ├── SAD.md                     # Software Architecture Document
│   ├── SCAFFOLDING.md             # This file
│   ├── IVP.md                     # Integration and Verification Plan (71 steps)
│   ├── PROJECT_STATUS.md          # Current phase and blockers
│   ├── PROJECT_OVERVIEW.md        # Vision and product tiers
│   ├── ROCKETCHIP_OS.md           # CLI design
│   ├── SENSOR_ARCHITECTURE.md     # Transport-neutral sensor backend design
│   ├── SEQLOCK_STRUCT_LAYOUT.md   # Cross-core data sharing layout
│   ├── MULTICORE_RULES.md         # RP2350 dual-core programming rules
│   ├── PHASE5_ESKF_PLAN.md        # ESKF implementation plan
│   ├── DYNAMIC_VALIDATION.md      # Physical ESKF verification test methods
│   ├── ESKF_TESTING_GUIDE.md      # ESKF test and replay harness guide
│   ├── HARDWARE_BUDGETS.md        # Power and weight budgets
│   ├── TOOLCHAIN_VALIDATION.md    # Build/debug setup guide
│   ├── AP_CAL.md                  # ArduPilot calibration algorithm reference
│   ├── CC_HANDOFF.md              # Context handoff notes
│   ├── PIO/
│   │   └── PIO_WATCHDOG.md        # PIO-based watchdog design
│   ├── audits/
│   │   └── CLANG_TIDY_AUDIT_2026-02-09.md  # Automated audit (1,251 findings)
│   ├── decisions/
│   │   ├── README.md
│   │   ├── FPRIME_HARDWARE_EVALUATION.md
│   │   ├── SENSOR_FUSION_TIERS.md
│   │   ├── SEQLOCK_DESIGN.md
│   │   ├── TITAN_BOARD_ANALYSIS.md
│   │   └── ESKF/
│   │       ├── FUSION_ARCHITECTURE.md
│   │       ├── FUSION_ARCHITECTURE_DECISION.md
│   │       └── ESKF_RESEARCH_SUMMARY.md  # MMAE pivot research (2026-02-24)
│   ├── benchmarks/
│   │   └── UD_BENCHMARK_RESULTS.md      # UD factorization benchmark (2026-02-24)
│   ├── flight_director/
│   │   ├── FLIGHT_DIRECTOR_DESIGN.md    # Flight Director design (Stage 6)
│   │   └── RESEARCH.md
│   ├── hardware/
│   │   ├── HARDWARE.md            # Hardware specs, pin assignments
│   │   ├── STATUS_INDICATORS.md   # LED patterns and status feedback
│   │   ├── GEMINI_CARRIER_BOARD.md    # Gemini redundant flight computer (future)
│   │   ├── TELSTAR_BOOSTER_PACK.md    # ELRS RC link + FPV video Booster Pack
│   │   └── ESP32_S3_PIVOT_ANALYSIS.md
│   ├── icd/
│   │   ├── EXPANSION_CONNECTOR_ICD.md
│   │   └── GEMINI_PROTOCOL_ICD.md
│   └── mission_profiles/
│       └── MISSION_PROFILES.md
│
├── standards/
│   ├── CODING_STANDARDS.md        # Code style and safety rules
│   ├── DEBUG_OUTPUT.md            # USB CDC output conventions
│   ├── GIT_WORKFLOW.md            # Git conventions
│   ├── VENDOR_GUIDELINES.md       # Hardware vendor constraints and datasheets
│   ├── STANDARDS_AUDIT.md         # Audit template
│   ├── STANDARDS_AUDIT_2026-02-07.md  # Manual audit (249 rules, 90% compliant)
│   ├── AUDIT_REMEDIATION.md       # Line-level fix tracking
│   ├── STANDARDS_DEVIATIONS.md    # Accepted deviation log
│   └── protocols/
│       └── SPACEWIRE_LITE.md      # SpaceWire-Lite (aspirational)
│
├── include/
│   └── rocketchip/
│       └── config.h               # DBG_* macros, build-type config, feature flags
│
├── src/
│   ├── main.cpp                   # Entry point: Core 0 superloop + Core 1 sensor loop
│   │
│   ├── drivers/                   # Hardware drivers (Flight-Critical)
│   │   ├── i2c_bus.cpp/.h         # I2C bus init, read/write, probe, recovery
│   │   ├── icm20948.cpp/.h        # ICM-20948 9-DoF IMU (I2C bypass mode)
│   │   ├── baro_dps310.cpp/.h     # DPS310 barometer
│   │   ├── gps_pa1010d.cpp/.h     # PA1010D GPS (I2C backend)
│   │   ├── gps_uart.cpp/.h        # GPS UART backend (preferred, 57600 baud / 10Hz)
│   │   ├── gps.h                  # Transport-neutral GPS interface
│   │   ├── ws2812_status.cpp/.h   # WS2812 NeoPixel status LED
│   │   └── lwgps_opts.h           # lwGPS config overrides
│   │
│   ├── fusion/                    # Sensor Fusion (Stage 5)
│   │   ├── eskf.cpp/.h            # 24-state Error-State Kalman Filter
│   │   ├── eskf_codegen.cpp/.h    # SymPy-generated FPFT (SRAM, .time_critical)
│   │   ├── eskf_state.h           # ESKF state vector definitions
│   │   ├── mahony_ahrs.cpp/.h     # Independent Mahony AHRS cross-check
│   │   ├── baro_kf.cpp/.h         # 1D baro Kalman filter (host tests only, not in firmware)
│   │   ├── ud_factor.cpp/.h       # UD factorization (benchmark only, not in firmware)
│   │   ├── wmm_declination.cpp/.h # World Magnetic Model declination lookup
│   │   └── (future: confidence_gate.* — Stage 7)
│   │
│   ├── math/                      # Math utilities (header-heavy)
│   │   ├── mat.h                  # NxM matrix template (header-only)
│   │   ├── vec3.cpp/.h            # 3D vector operations
│   │   └── quat.cpp/.h            # Quaternion math
│   │
│   ├── calibration/               # Calibration (Ground classification)
│   │   ├── calibration_data.cpp/.h      # Calibration data structures
│   │   ├── calibration_manager.cpp/.h   # 6-pos accel cal, mag cal algorithms
│   │   └── calibration_storage.cpp/.h   # Flash persistence (dual-sector)
│   │
│   ├── cli/                       # CLI / Local GCS (Ground classification)
│   │   └── rc_os.cpp/.h           # Serial menu, command dispatch, CLI hooks
│   │
│   └── tools/
│       └── mat_benchmark.cpp      # Matrix math benchmark (standalone target)
│
├── test/                          # Host-side tests (Google Test)
│   ├── CMakeLists.txt
│   ├── test_vec3.cpp
│   ├── test_quat.cpp
│   ├── test_mat.cpp
│   ├── test_baro_kf.cpp
│   ├── test_eskf_propagation.cpp
│   ├── test_eskf_update.cpp
│   ├── test_eskf_gps_update.cpp
│   ├── test_eskf_mag_update.cpp
│   ├── test_eskf_zupt.cpp
│   ├── test_mahony.cpp
│   ├── test_wmm_declination.cpp
│   ├── test_replay_regression.cpp
│   ├── csv_loader.h               # CSV data loader for test fixtures
│   ├── data/                      # Synthetic test trajectories
│   │   ├── *.csv                  # Input data (static, const_accel, etc.)
│   │   └── reference/             # Expected output for regression tests
│   ├── replay/                    # Replay harness for ESKF validation
│   │   ├── replay_harness.cpp
│   │   ├── sensor_log.h
│   │   └── output_log.h
│   └── scripts/
│       └── generate_synthetic.py  # Test trajectory generator
│
├── scripts/                       # Python test and utility scripts
│   ├── cli_test.py                # Automated CLI testing via serial
│   ├── accel_cal_6pos.py          # Interactive 6-position calibration
│   ├── i2c_soak_test.py           # Long-duration I2C reliability test
│   ├── eskf_gps_soak.py           # ESKF GPS soak test
│   ├── codegen_soak_test.py       # Binary change soak comparison
│   ├── generate_fpft.py           # SymPy codegen for ESKF FPFT (CSE optimization)
│   └── run_clang_tidy.sh          # clang-tidy runner with project config
│
├── lib/                           # External libraries (git submodules)
│   ├── icm20948/                  # Vendor reference library
│   ├── lwgps/                     # Lightweight GPS NMEA parser
│   ├── ruuvi.dps310.c/            # Ruuvi DPS310 C driver
│   └── ws2812b-animation/         # WS2812 animation library
│
├── ground_station/                # Ground station prototypes
│   ├── radio_rx.cpp
│   ├── lora_rx_simple/            # LoRa receiver (Arduino)
│   └── rfm69_rx_simple/           # RFM69 receiver (Arduino)
│
├── tools/
│   └── state_to_dot.py            # State machine DOT graph generator
│
├── pico-sdk/                      # Pico SDK (git submodule)
│
├── .claude/                       # Agent context
│   ├── CLAUDE.md                  # Main includes
│   ├── AK_GUIDELINES.md           # Behavioral guidelines
│   ├── PROTECTED_FILES.md         # Files requiring explicit edit permission
│   ├── SESSION_CHECKLIST.md       # Session handoff procedures
│   ├── LESSONS_LEARNED.md         # Debugging journal (29 entries)
│   └── DEBUG_PROBE_NOTES.md       # OpenOCD/GDB setup
│
├── build/                         # CMake build output (gitignored)
├── build_host/                    # Host test build output (gitignored)
└── logs/                          # Serial capture logs (gitignored)
```

## Module Responsibilities

See `docs/SAD.md` Section 3.2 for the planned production architecture. Below reflects the current implemented modules.

| Module | Responsibility |
|--------|----------------|
| **main.cpp** | Core 0 superloop (fusion, CLI, USB), Core 1 sensor loop (IMU/baro/GPS/mag reads), boot init |
| **i2c_bus** | I2C peripheral init, bus read/write/probe, 9-clock bit-bang recovery |
| **icm20948** | ICM-20948 IMU driver — accel/gyro/temp reads, AK09916 mag via I2C bypass mode |
| **baro_dps310** | DPS310 barometer driver — pressure/temperature reads |
| **gps_pa1010d** | PA1010D GPS driver — I2C backend with 32-byte chunked reads |
| **gps_uart** | GPS UART backend — interrupt-driven ring buffer, baud negotiation, 10Hz rate |
| **ws2812_status** | NeoPixel status LED — animation engine with mode-based patterns |
| **eskf** | 24-state Error-State Kalman Filter — propagation + baro/mag/GPS/ZUPT updates |
| **eskf_codegen** | SymPy-generated FPFT covariance prediction — SRAM execution (.time_critical) |
| **mahony_ahrs** | Independent Mahony AHRS — 200Hz attitude cross-check for ESKF |
| **wmm_declination** | World Magnetic Model — declination lookup by lat/lon |
| **calibration_manager** | Gyro bias, level cal, 6-position accel cal, magnetometer ellipsoid fit |
| **calibration_storage** | Dual-sector flash persistence for calibration data |
| **rc_os** | CLI menu system — "local GCS" translating keystrokes to commands |

## Execution Architecture

Bare-metal dual-core AMP (Asymmetric Multiprocessing) on RP2350:

- **Core 0:** Cooperative superloop — ESKF fusion (200Hz), Mahony AHRS (200Hz), CLI/USB I/O, calibration commands
- **Core 1:** Tight sensor polling loop — IMU (~1kHz), baro (~8Hz), GPS (10Hz), mag (100Hz via ICM-20948 bypass)
- **Cross-core:** Seqlock double-buffer (Core 1 writes, Core 0 reads). See `docs/decisions/SEQLOCK_DESIGN.md`.

## CMake Build Targets

| Target | Type | Description |
|--------|------|-------------|
| `rocketchip` | Prod | Main firmware (RP2350 target) |
| `rocketchip_tests` | Dev | Host-side Google Test suite (194+ tests) |
| `mat_benchmark` | Dev | Matrix math performance benchmark |

Build commands:
```bash
# Firmware (RP2350)
cmake -B build -G Ninja && cmake --build build

# Host tests
cmake -B build_host -G Ninja -DBUILD_HOST_TESTS=ON && cmake --build build_host
```

## Implementation Status

For the complete 72-step development roadmap, see **`docs/IVP.md`**.
For current focus and blockers, see **`docs/PROJECT_STATUS.md`**.

| Stage | IVPs | Status | Summary |
|-------|------|--------|---------|
| 1: Foundation | 01–08 | Complete | Build, LED, NeoPixel, USB CDC, debug macros, I2C bus, heartbeat |
| 2: Single-Core Sensors | 09–18 | Complete | IMU, baro, calibration (gyro/level/6-pos), CLI |
| 3: Dual-Core | 19–30 | Complete | Core 1, atomics, spinlock, FIFO, doorbell, seqlock, sensor migration, MPU, watchdog |
| 4: GPS Integration | 31–33 | Complete | PA1010D on Core 1, outdoor fix validated |
| Phase M: Mag Cal | 34–38 | Complete | Data structures, LM ellipsoid solver, CLI wizard, live apply |
| 5: Sensor Fusion | 39–48 | Complete | 24-state ESKF, codegen FPFT, health tuning, Mahony AHRS |
| 6: Flight Director | 49–53 | Planned | Watchdog recovery, state machine, event engine, action executor, mission config |
| 7: Adaptive Estimation | 54–57 | Planned | Phase-scheduled Q/R, confidence gate, confidence-gated actions, vehicle profiles |
| 8: Data Logging | 58–62 | Planned | LittleFS, logger service, pre-launch buffer |
| 9: Telemetry | 63–67 | Planned | RFM95W, MAVLink, GCS compatibility |
| 10: System Integration | 68–72 | Planned | Bench test, flight test, environmental |

**Archived Work:**
Previous ArduPilot integration preserved in `AP_FreeRTOS` and `AP_ChibiOS` branches.

## Related Documents

- **docs/SAD.md** — Software Architecture Document (high-level design)
- **docs/IVP.md** — Integration and Verification Plan (71 steps)
- **docs/PROJECT_STATUS.md** — Current phase and blockers
- **docs/SENSOR_ARCHITECTURE.md** — Transport-neutral sensor backend design
- **docs/decisions/SEQLOCK_DESIGN.md** — Cross-core data sharing rationale
- **docs/MULTICORE_RULES.md** — RP2350 dual-core programming rules
- **standards/CODING_STANDARDS.md** — Code style and safety rules
- **docs/hardware/HARDWARE.md** — Hardware specifications
