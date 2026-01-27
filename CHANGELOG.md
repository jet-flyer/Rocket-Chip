# Changelog

## Format
`### YYYY-MM-DD-NNN | Author | tags`

Files affected in parentheses if relevant.

**Frequency:** Typically one entry per session, not per individual change. Log when a task is completed or work transitions to a new focus. However, if multiple *significant* changes occur in one session (e.g., refactoring logging system AND redesigning state engine), create separate entries for each.

**Conciseness is the default.** Most entries should be 1-3 sentences. The entry itself should state *what* changed. If additional context is needed, a brief parenthetical or second sentence suffices.

**Rationale sections are rare.** Only add an italicized rationale block when:
- An unconventional approach was chosen (e.g., experimental driver, workaround for a known issue)
- A decision would appear wrong without context (e.g., why we avoided the "obvious" solution)
- Architectural trade-offs need to be preserved for future contributors

Routine work—even if complex—does not warrant rationale. Bugfixes, documentation updates, configuration changes, and hardware corrections rarely need explanation. When in doubt, omit the rationale.

**Tags:** bugfix, feature, architecture, tooling, hardware, council, documentation, refactor

*Guidelines updated 2026-01-14. Earlier entries may be more verbose.*

---

### 2026-01-27-001 | Claude Code CLI | feature

**ChibiOS Phase 0: UART Debug Output Working**

Added UART debug test for ChibiOS on RP2350. Output via Raspberry Pi Debug Probe UART pass-through at 115200 baud. Uses ChibiOS SIO driver with chprintf stream wrapper.

Also includes incomplete test_usb_cdc project (TinyUSB integration attempt - blocked on Pico SDK header dependencies, documented for future bare-metal approach).

(chibios/phase0_validation/test_uart/, chibios/phase0_validation/test_usb_cdc/, chibios/PROGRESS.md)

---

### 2026-01-26-001 | Claude Code CLI | refactor, architecture

**ArduPilot Dependency Bypass Code Removed - Reevaluation Required**

Removed custom wrapper code that was created without explicit approval, bypassing ArduPilot's proven implementations:

**Files Removed:**
- `lib/ap_compat/AP_Compass/AP_Compass.cpp` - Custom compass wrapper with offset sign bug (subtracted instead of added offsets)
- `lib/ap_compat/AP_Compass/AP_Compass.h` - Custom compass wrapper header
- `lib/ap_compat/AP_Compass/CompassCalibrator.cpp` - Duplicate of ArduPilot's CompassCalibrator
- `lib/ap_compat/AP_Compass/CompassCalibrator.h` - Duplicate header
- `lib/ap_compat/AP_Param/AP_Param.cpp` - Simplified AP_Param implementation
- `lib/ap_compat/AP_Param/AP_Param.h` - Simplified AP_Param header

**Files Retained:**
- `lib/ap_compat/AP_Compass/AP_Compass_config.h` - Legitimate config flags
- `lib/ap_compat/AP_Param/ParamStorageLayout.h` - Custom storage layout (RocketChip-specific)

**Standards Updated:**
- Added "Dependency Bypassing Policy" to CODING_STANDARDS.md requiring explicit approval before bypassing any ArduPilot dependencies
- Added "Sparse Checkout Review" requirement - missing ArduPilot libraries must be flagged for review before creating alternatives

**Sparse Checkout Updated:**
- Added `libraries/AP_Param` to ArduPilot sparse checkout

Magnetometer calibration work paused pending comprehensive dependency reevaluation. The custom AP_Compass wrapper introduced a sign convention bug that ArduPilot's proven code already handles correctly.

*Rationale: Custom wrappers bypass battle-tested ArduPilot code and introduce subtle bugs. The offset sign convention error (subtracting instead of adding) was found because ArduPilot's AP_Compass_Backend.cpp:85 does `mag += offsets`. Future work must use ArduPilot's implementations directly or receive explicit approval for alternatives.*

(lib/ap_compat/AP_Compass/, lib/ap_compat/AP_Param/, standards/CODING_STANDARDS.md)

---

### 2026-01-24-004 | Claude Code CLI | bugfix, feature

**Accelerometer Calibration Persistence Working**

Fixed flash storage persistence on RP2350 and completed accelerometer calibration integration:

- **Flash page alignment fix**: RP2350's `flash_range_program()` requires 256-byte aligned writes. Added `do_flash_program_unaligned()` helper that reads existing page, modifies bytes, writes full page. This was causing silent write failures corrupting AP_FlashStorage sector headers.
- **Unit conversion fix**: AccelCalibrator expects m/s², but ISM330DHCX driver returns g. Added GRAVITY_MSS (9.80665) conversion in calibration sample collection.
- **HAL init order**: Moved `hal.init()` before `stdio_init_all()` to avoid BASEPRI conflicts with USB interrupts during flash operations.

Calibration now persists correctly across power cycles. 6-position calibration produces reasonable offsets (~0.03-0.1g) and scale factors (~1.000-1.004).

(lib/ap_compat/AP_HAL_RP2350/Storage.cpp, tests/smoke_tests/calibration_test.cpp)

---

### 2026-01-24-003 | Claude Code CLI | feature, architecture

**AP_HAL_RP2350 Phase 2 Complete - All Core Peripherals Implemented**

Completed Phase 2 AP_HAL_RP2350 implementation with all core peripherals hardware-validated (23/23 tests passing):

- **GPIO**: Pin I/O with DigitalSource wrapper, E9 erratum documented
- **AnalogIn**: ADC channels, MCU temperature, board voltage monitoring
- **UARTDriver**: USB CDC + 2x hardware UART via BetterStream interface
- **I2CDevice**: Bus manager with device pooling, extended timeouts per PD7
- **SPIDevice**: Polling-only transfers per PD8, device-name-based lookup

Hardware validated: ISM330DHCX IMU, LIS3MDL magnetometer (I2C), RFM95W LoRa radio (SPI). Fixed SPI pin mapping (SCK=22, MOSI=23, MISO=20) to match Pico SDK defaults. Added Adafruit Arduino pin naming convention documentation to HARDWARE.md. RCOutput deferred to Titan tier (TVC/servo work).

(lib/ap_compat/AP_HAL_RP2350/*, tests/smoke_tests/smoke_phase2.cpp, docs/HARDWARE.md)

---

### 2026-01-24-002 | Claude Code CLI | bugfix, architecture

**AP_HAL_RP2350 Phase 1 Complete - Flash/FreeRTOS SMP Fix**

Fixed critical crash in Storage where `flash_safe_execute()` conflicts with FreeRTOS SMP. Root cause: `multicore_lockout` used by `flash_safe_execute()` cannot coexist with FreeRTOS SMP's dual-core scheduler. Solution: RAM-resident free functions with `__not_in_flash_func()`, direct `flash_range_erase()`/`flash_range_program()` calls with interrupt disable, and `xip_cache_invalidate_all()` after flash ops. Updated PD1 in RP2350_FULL_AP_PORT.md with correct solution. Created REBUILD_CONTEXT.md documenting the debugging process. Phase 1 now fully hardware-validated: Scheduler, Semaphores, Util, Storage, CalibrationStore all working.

*Rationale: The Pico SDK's flash_safe_execute() is designed for single-core or bare-metal multicore operation, not RTOS-managed SMP. With FreeRTOS controlling both cores, the lockout signaling deadlocks. Direct flash ops work because disabling interrupts on the current core is sufficient when the other core isn't actively accessing flash (FreeRTOS tasks yield regularly).*

(lib/ap_compat/AP_HAL_RP2350/Storage.cpp, HAL_RP2350_Class.cpp, REBUILD_CONTEXT.md, docs/RP2350_FULL_AP_PORT.md, docs/AP_HAL_RP2350_PLAN.md)

---

### 2026-01-24-001 | Claude Code CLI | documentation, architecture

**RP2040/RP2350 Port Research and Platform Differences Documentation**

Researched existing RP2040/RP2350 ArduPilot port attempts (ChibiOS, Betaflight, MadFlight, ESP32 HAL) to identify known issues for current and future phases. Added 10 platform differences to `RP2350_FULL_AP_PORT.md`:

- **Phase 1:** PD2 (SIO FIFO IRQ shared), PD3 (TinyUSB Core 1), PD5 (stage2 bootloader), PD6 (no ChibiOS)
- **Phase 2:** PD4 (PWM timer sharing), PD7 (I2C clock stretching), PD8 (SPI+DMA stops at ~253 cycles), PD9 (RP2350-E9 GPIO/ADC erratum)
- **Phase 4+:** PD10 (gyro rate requirements), PD11 (malloc must zero)

Updated entry format to include "Source URLs" field. Key risks: PD8 (SPI+DMA) is critical - recommend polling initially until DMA proven stable. Our SPI-based sensors avoid PD7 (I2C) issues.

(docs/RP2350_FULL_AP_PORT.md)

---

### 2026-01-23-004 | Claude Code CLI | architecture, refactor

**Tier 1 Storage Implementation and AP_HAL Shim Refactor**

Implemented Tier 1 persistent storage using ArduPilot's AP_FlashStorage for wear-leveled calibration/config storage. Created comprehensive AP_HAL stub headers to isolate ArduPilot library dependencies from full HAL requirements. Refactored `lib/ap_compat/` structure: `AP_HAL/` now provides minimal stubs for ArduPilot libraries, `AP_HAL_RP2350/` provides the full FreeRTOS-based HAL implementation. Added `AP_HAL::Storage` base class, `CalibrationStore` wrapper with CRC validation, flash memory layout headers (`flash_map.h`, `storage_layout.h`). Key files created: Storage.h/cpp, CalibrationStore.h/cpp, 15+ stub headers (UARTDriver.h, GPIO.h, etc.). Build validated with smoke_storage test target.

(lib/ap_compat/, include/rocketchip/, src/calibration/, tests/smoke_tests/storage_test.cpp, CMakeLists.txt)

---

### 2026-01-23-003 | Claude Code CLI | documentation, refactor

**Documentation Rectification: Errant Prompt Correction**

Discovered that AP_HAL_RP2350_PLAN.md was created from an outdated prompt (pre-council review) that lacked the storage architecture decisions and council context. The correct prompt (`flash_storage_integration_prompt.md`) and conversation summary (`conversation_summary.md`) from the council review were not used. Rectified by rewriting documentation to reflect council decisions: two-tier storage model, genuine HAL implementation rationale, and directory structure (`lib/ap_compat/AP_HAL_RP2350/`). Also fixed stale references in SAD.md Section 1.2 (changed "Library shims only" to "AP_HAL_RP2350 implementation") and updated architecture diagram. Phase 1 implementation code was unaffected - only documentation needed correction.

(docs/AP_HAL_RP2350_PLAN.md, docs/SAD.md)

---

### 2026-01-23-002 | Claude Code CLI | architecture, documentation, council

**Storage Architecture and AP_HAL Documentation Update**

Integrated council-approved storage architecture into project documentation. Two-tier model: Tier 1 (AP_FlashStorage for calibration/config/missions), Tier 2A (LittleFS for flight logs), Tier 2B (FatFs for SD card, future). Moved `lib/AP_HAL_RP2350/` into `lib/ap_compat/AP_HAL_RP2350/` per council recommendation. Rewrote `AP_HAL_RP2350_PLAN.md` with council decision context, storage tier architecture, and flash memory layout. Updated SAD.md Section 9.2-9.4 with storage architecture, resolved open questions #2 (configuration storage) and #5 (calibration persistence).

(docs/AP_HAL_RP2350_PLAN.md, docs/SAD.md, docs/SCAFFOLDING.md, CMakeLists.txt, lib/ap_compat/AP_HAL_RP2350/)

---

### 2026-01-23-001 | Claude Code CLI | tooling, documentation

**Claude Code memory structure and naming cleanup**

Added `.claude/CLAUDE.md` (references README.md) and `.claude/settings.json` (build tool permissions, secrets deny patterns) for Claude Code project memory. Updated `.gitignore` to track `.claude/` while ignoring local files. Corrected HARDWARE.md product naming: Advanced tier confirmed as **Titan**, Nova reserved for future space-rated product (aligns with existing decisions in SAD.md and PROJECT_OVERVIEW.md).

(.claude/CLAUDE.md, .claude/settings.json, .gitignore, README.md, docs/HARDWARE.md)

---

### 2026-01-22-002 | Claude Code CLI | validation

**AP_HAL_RP2350 Phase 1 Hardware Validated**

All 7 smoke tests passing on Feather RP2350 hardware: Timing (millis/micros accuracy), Scheduler delays, Semaphores (recursive mutex), Binary semaphores, Util functions (memory reporting, system ID, arming state), Timer callbacks (1kHz timer, 100Hz I/O), and System state. Fixed `available_memory()` to handle static-allocation-only configuration (returns total heap when FreeRTOS reports 0 free). Refactored test to follow DEBUG_OUTPUT.md pattern: tests run immediately, results stored, LED indicates status, serial output waits for USB connection.

(lib/AP_HAL_RP2350/Util.cpp, tests/smoke_tests/ap_hal_test.cpp)

---

### 2026-01-22-001 | Claude Code CLI | feature, architecture

**AP_HAL_RP2350 Phase 1: Scheduler, Semaphores, Util**

Implemented core ArduPilot HAL components for RocketChip RP2350, enabling direct ArduPilot library usage without the full ChibiOS port. This work was prompted by calibration integration attempt (2026-01-21-004) which revealed that the existing ap_compat shim layer was insufficient for ArduPilot's calibration libraries.

**Scheduler (Scheduler.h/.cpp):**
- FreeRTOS task/timer mapping for ArduPilot's callback system
- millis/micros timing via hardware timer
- 1kHz timer callbacks, 100Hz I/O callbacks
- delay() with RTOS yield, delay_microseconds() busy-wait
- Thread creation and priority management

**Semaphores (Semaphores.h/.cpp):**
- HAL_Semaphore as recursive mutex with FreeRTOS primitives
- BinarySemaphore for thread signaling
- ISR-safe signal_ISR() method
- WithSemaphore RAII helper for automatic lock management

**Util (Util.h/.cpp):**
- Memory info via FreeRTOS heap APIs
- System ID from RP2350 unique flash ID
- Soft arming state tracking

**HAL Singleton (HAL_RP2350_Class.h/.cpp):**
- Global `hal` instance matching ArduPilot's AP_HAL pattern
- Subsystem initialization and accessor methods

**hwdef.h:**
- Board definitions (GPIO pins, I2C addresses, SPI config)
- Feature flags for ArduPilot compatibility
- FreeRTOS task priorities and stack sizes

**Validation:**
- Smoke test (ap_hal_test.cpp) exercises all components
- Build verified: smoke_ap_hal.uf2 compiles (58KB code, 88KB BSS)

(lib/AP_HAL_RP2350/*, tests/smoke_tests/ap_hal_test.cpp, CMakeLists.txt, docs/AP_HAL_RP2350_PLAN.md)

---

### 2026-01-21-004 | Claude Code CLI | architecture

**Calibration System Integration (DEFERRED)**

Attempted calibration infrastructure integration with ArduPilot's AccelCalibrator and CompassCalibrator libraries. Work included CalibrationManager in src/calibration/ with startup calibration routines, compass calibration smoke test with sphere fit and WMM field validation, and SensorTask integration for runtime calibration application.

*Revealed that existing ap_compat shim layer was insufficient—ArduPilot calibration libraries require proper HAL_Semaphore, Scheduler, and timing primitives. Deferred pending AP_HAL_RP2350 implementation (see 2026-01-22-001). Local work stashed; will revisit after HAL validation.*

---

### 2026-01-21-003 | Claude Code CLI | [STANDARDS]

**Resolve Grok-identified deviations**

- Introduced compile-time guarded DBG_PRINT/DBG_ERROR macros in new include/debug.h; replaced all diagnostic printf in src/main.cpp and services/ with macros to enable clean release builds (resolves raw printf deviation).
- Standardized constant naming with 'k' prefix across src/ (e.g., kLedPin, kUiTaskPriority) for consistency with test code conventions (resolves naming drift deviation).

Additional council-approved resolutions included in this commit:
- Full ArduPilot AP_InternalError integration via shim for production error reporting.
- Cleanup of STANDARDS_DEVIATIONS.md: closed low-severity test items and error handling; retained only external ArduPilot lib deviations.

(include/debug.h, src/main.cpp, src/services/SensorTask.cpp, lib/ap_compat/AP_InternalError/AP_InternalError.h, lib/ap_compat/AP_HAL_Compat.cpp, standards/STANDARDS_DEVIATIONS.md, CMakeLists.txt)

---

### 2026-01-21-002 | Claude Code CLI | documentation, refactor

**Standards Compliance and Verification Process**

Fixed JSF AV C++ standards compliance in calibration smoke test: added named constants for all magic numbers (`kI2cSpeedHz`, `kBlinkSlowPeriodMs`, etc.), applied `g_` prefix to globals, `k` prefix to constants. Created `standards/STANDARDS_DEVIATIONS.md` to track deviations with severity, difficulty, and remediation status. Added "Code Verification Process" section to `CODING_STANDARDS.md` with pre-commit checklist for standards compliance.

(tests/smoke_tests/calibration_test.cpp, standards/STANDARDS_DEVIATIONS.md, standards/CODING_STANDARDS.md)

**Next Steps**: Hardware validate calibration on sensors, resolve error handling strategy before EKF3 integration (Medium severity deviation pending council decision).

---

### 2026-01-21-001 | Claude Code CLI | feature, architecture

**ArduPilot Calibration Library Integration**

Integrated ArduPilot's AP_Math and AccelCalibrator libraries via compatibility shim layer, enabling sensor calibration infrastructure before EKF3 fusion implementation.

**ArduPilot Submodule (lib/ardupilot/)**
- Added ArduPilot repository as sparse checkout containing only needed libraries: AP_Math, Filter, AP_AccelCal, AP_Compass, AP_HAL, AP_Common, AP_InternalError
- Sparse checkout minimizes repo size while providing battle-tested math and calibration code

**Compatibility Shim Layer (lib/ap_compat/)**
- `AP_HAL_Compat.h`: Maps ArduPilot HAL timing functions to RocketChip HAL (AP_HAL::millis() → Timing::millis32(), etc.), provides HAL_Semaphore stub for single-threaded operation, defines utility macros (ARRAY_SIZE, PACKED, MIN/MAX, etc.)
- `AP_HAL/AP_HAL.h`, `AP_HAL/AP_HAL_Namespace.h`, `AP_HAL/AP_HAL_Boards.h`, `AP_HAL/Semaphores.h`: Stub headers to satisfy ArduPilot include paths
- `AP_InternalError/AP_InternalError.h`: Error reporting stub with full error_t bitmask enum matching ArduPilot's structure; includes TODO for error handling strategy decision before EKF3 integration
- `AP_Vehicle/AP_Vehicle_Type.h`: Vehicle type definitions stub
- `AP_CustomRotations/AP_CustomRotations.h`: Custom rotations disabled stub
- `AP_Logger/AP_Logger_config.h`: Logging disabled stub

**CMakeLists.txt Updates**
- Added `ap_compat` library (shim layer)
- Added `ap_math` library: AP_Math.cpp, vector2/3.cpp, matrix3.cpp, matrixN.cpp, matrix_alg.cpp, quaternion.cpp, location.cpp, polygon.cpp, crc.cpp
- Added `ap_filter` library: LowPassFilter.cpp, LowPassFilter2p.cpp, NotchFilter.cpp, DerivativeFilter.cpp, ModeFilter.cpp, SlewLimiter.cpp
- Added `ap_calibrators` library: AccelCalibrator.cpp
- Added `smoke_calibration` executable target

**Calibration Smoke Test (tests/smoke_tests/calibration_test.cpp)**
- Interactive 6-position accelerometer calibration using ArduPilot's Gauss-Newton solver
- NeoPixel status LED colors per deprecated codebase patterns:
  - Magenta blink: Waiting for USB connection
  - Yellow pulsing: Initializing
  - Blue slow blink (500ms): Waiting for user orientation input
  - Blue solid: Collecting samples
  - Green fast blink (100ms): Calibration successful
  - Red solid: Error
- USB CDC connection handling per DEBUG_OUTPUT.md: visual feedback during wait, 500ms settle time
- Outputs calibration constants (offsets, scale factors, cross-axis) ready to copy into configuration
- Converts between RocketChip Vector3f and ArduPilot Vector3f types

**HARDWARE.md Updates**
- Added complete Feather RP2350 HSTX built-in hardware GPIO table: UART0 (GPIO 0/1), I2C1/STEMMA QT (GPIO 2/3), Red LED (GPIO 7), PSRAM CS (GPIO 8), HSTX (GPIO 12-19), SPI0 (GPIO 20/22/23), NeoPixel (GPIO 21)
- Added RocketChip pin assignments table with current and TBD assignments
- Added HSTX connector documentation (22-pin back connector with GPIO 12-19)
- Added important notes: PSRAM GPIO 8 restriction, E9 erratum workaround for pull-downs

**Standards Compliance Review**
- Reviewed against JSF AV C++ standards, DEBUG_OUTPUT.md, CODING_STANDARDS.md, SAD.md, HARDWARE.md
- Identified JSF violations in smoke test (dynamic allocation, primitive types, naming conventions) - acceptable for test code, documented for future cleanup
- Confirmed architecture alignment: ArduPilot via shim layer per CODING_STANDARDS.md section "ArduPilot Library Integration"

(lib/ardupilot/, lib/ap_compat/*, tests/smoke_tests/calibration_test.cpp, CMakeLists.txt, docs/HARDWARE.md)

*Rationale: EKF3 expects calibrated sensor inputs. ArduPilot's AccelCalibrator provides proven 6-position calibration with ellipsoid fitting. Shim layer approach avoids full ChibiOS HAL port while leveraging ArduPilot's battle-tested math. Error handling strategy (Option A: full AP_InternalError vs Option B: RocketChip-specific) deferred to pre-fusion decision point.*

---

### 2026-01-19-001 | Claude Code CLI | documentation, architecture

Added Gemini carrier board documentation for dual-Core redundant flight computer configuration. Created Interface Control Documents (ICDs) following NASA-style templates and SpaceWire-Lite protocol specification as aspirational standard (ECSS-E-ST-50-12C subset). Gemini features dual RP2350 MCUs with hardware voting logic for pyro safety (AND to ARM, OR to FIRE), SpaceWire-derived inter-MCU communication via LVDS transceivers, and independent power regulation with isolation.

**New files:**
- `docs/GEMINI_CARRIER_BOARD.md` - Master design document
- `docs/icd/EXPANSION_CONNECTOR_ICD.md` - Feather-based expansion connector ICD
- `docs/icd/GEMINI_PROTOCOL_ICD.md` - Inter-MCU protocol (heartbeat, failover, pyro coordination)
- `standards/protocols/SPACEWIRE_LITE.md` - SpaceWire-Lite physical/data-link layer specification

**Updated files:**
- `PROJECT_OVERVIEW.md` - Added Gemini configuration and related documentation
- `docs/HARDWARE.md` - Added Gemini hardware section (voting logic, LVDS, components)
- `docs/SAD.md` - Added Section 14 for Gemini redundant architecture
- `docs/SCAFFOLDING.md` - Updated directory structure with icd/ and protocols/ directories

---

### 2026-01-16-004 | Claude Code CLI | architecture, refactor

Migrated from validation scaffolding to production architecture. Moved FreeRTOS validation code (main.c, hooks.c) to `tests/validation/freertos_validation/` as reference. Created production entry point `src/main.cpp` with HAL initialization and FreeRTOS task creation. Implemented `SensorTask` in `src/services/` using real HAL drivers (ISM330DHCX, LIS3MDL, DPS310) at 1kHz/100Hz/50Hz rates respectively, with thread-safe shared `SensorData` struct. Added `services` library and `rocketchip` target to CMakeLists.txt. Enabled mutex support in FreeRTOSConfig.h. Both targets build: rocketchip (70KB) and freertos_validation (52KB).

(src/main.cpp, src/services/SensorTask.*, tests/validation/freertos_validation/, CMakeLists.txt, FreeRTOSConfig.h)

*Note: Re-implemented after previous Claude session error caused work loss. Review for any discrepancies from original intent.*

---

### 2026-01-16-003 | Claude Code CLI | documentation

Synced SAD and SCAFFOLDING with actual implementation. Updated hardware references (ICM-20948 → ISM330DHCX + LIS3MDL, LoRa part #3179 → #3231), replaced PlatformIO references with CMake build system, updated I2C address map, removed Roman god naming from booster packs (Mercury/Juno/Vulcan → generic Telemetry/GPS/Pyro packs), added ground_station/ directory documentation, clarified radio driver status (debug bridge now, full MAVLink in Phase 7). Consolidated development checklists: SAD.md Section 10 is now single source of truth with Phase 1 marked complete and Phase 2 in progress. SCAFFOLDING references SAD for detailed roadmap.

(docs/SAD.md, docs/SCAFFOLDING.md)

---

### 2026-01-16-002 | Claude Code CLI | feature

GPS driver (PA1010D) and LoRa radio driver (RFM95W) implemented. GPS validated on hardware - I2C connectivity and NMEA parsing working. Radio driver is a minimal wireless serial bridge for debugging only (not full MAVLink telemetry - that comes later). Includes TX smoke test and ground station RX firmware in `ground_station/`. Added raw I2C read/write to Bus for stream devices. Radio hardware test pending.

(src/hal/GPS_PA1010D.*, src/hal/Radio_RFM95W.*, src/hal/Bus.*, tests/smoke_tests/gps_test.cpp, tests/smoke_tests/radio_tx_test.cpp, ground_station/radio_rx.cpp, CMakeLists.txt)

---

### 2026-01-16-001 | Claude Code CLI | council

EKF3-derived architecture approved for all tiers. 22-state filter (EKF3 minus wind states) plus GSF yaw estimator. Single IMU for Core/Main, dual IMU option for Titan. Implementation after GPS driver complete.

---

### 2026-01-15-001 | Claude Code CLI | feature, architecture

Integrated ST platform-independent sensor drivers per council approval. Added git submodules for ISM330DHCX (IMU), LIS3MDL (magnetometer), and ruuvi DPS310 (barometer). Created C++ wrappers in `src/hal/` using callback bridge pattern to connect vendor drivers to RocketChip SensorBus abstraction. Updated CMakeLists.txt with `hal_sensors` library linking ST/ruuvi drivers. Added `smoke_st_sensors` test target for hardware validation.

(src/hal/IMU_ISM330DHCX.*, src/hal/Mag_LIS3MDL.*, src/hal/Baro_DPS310.*, tests/smoke_tests/st_sensors_test.cpp, CMakeLists.txt, .gitmodules)

*Rationale: Council approved leveraging vendor-tested, MISRA-compliant drivers rather than writing register access from scratch. Wrappers call driver functions (e.g., `ism330dhcx_device_id_get()`, `dps310_init()`) through platform callbacks that bridge to SensorBus.*

---

### 2026-01-14-002 | Claude Code CLI | refactor, documentation

Repo cleanup: moved HARDWARE.md to docs/, simple_test.c to tests/smoke_tests/. Updated SCAFFOLDING.md to reflect actual structure. Slimmed PROJECT_STATUS.md to rotating checklist format.

---

### 2026-01-14-001 | Claude Code CLI | feature, architecture, hardware, documentation

Implemented complete Hardware Abstraction Layer (HAL) for RP2350/Pico SDK with FreeRTOS integration. Modules cover Bus (I2C/SPI), GPIO, ADC, PWM, Timing, PIO (NeoPixel), and UART. See `src/hal/README.md` for module details.

**Documentation Updates (HARDWARE.md):**
- Added comprehensive Peripheral Interfaces section covering SPI, I2C, UART, PIO, GPIO, ADC, CAN, HSTX with tier availability matrix
- Bus performance comparison table (SPI vs I2C overhead at 1kHz)
- Updated GPIO Assignments table with SPI, PIO PWM, pyro, battery ADC pins
- Corrected IMU reference from ICM-20948 to ISM330DHCX+LIS3MDL FeatherWing (#4569)
- Updated I2C Address Map with correct sensor addresses

**Build System (CMakeLists.txt):**
- Added `hal` static library target linking Pico SDK hardware libraries and FreeRTOS
- Added `smoke_hal_validation` test target

**Validation (tests/smoke_tests/hal_validation.cpp):**
- Comprehensive smoke test covering: HAL init, timing accuracy, GPIO LED/button, ADC temperature, I2C sensor probing (ISM330DHCX, LIS3MDL, DPS310 WHO_AM_I verification), NeoPixel color cycle
- Visual pass/fail indication via NeoPixel (green=pass, red=fail) and LED blink rate
- USB serial output with detailed test results

(src/hal/*, CMakeLists.txt, HARDWARE.md, tests/smoke_tests/hal_validation.cpp)

### 2026-01-12-002 | Claude Code CLI | documentation

Clarified TinyUSB is initialized at runtime via stdio_init_all(). Submodule init is optional (eliminates build warning only).

(PROJECT_STATUS.md, src/main.c, CHANGELOG.md)

### 2026-01-12-001 | Claude Code CLI | bugfix, hardware, tooling

FreeRTOS SMP validation complete on Adafruit Feather RP2350 using Raspberry Pi Debug Probe with hardware debugging via OpenOCD + GDB. Identified and fixed two critical FreeRTOS configuration bugs: (1) configMAX_PRIORITIES set to 5 but code used priority 5 (valid range is 0 to MAX-1), increased to 8; (2) configMAX_SYSCALL_INTERRUPT_PRIORITY set to 191 but RP2350 Cortex-M33 requires multiple of 16 per FreeRTOS port spec, changed to 16. Debugged wrong Debug Probe firmware issue - was flashing debugprobe_on_pico.uf2 (for DIY Pico-based probe with different GPIO mapping) instead of debugprobe.uf2 (for official Debug Probe hardware), causing LED to stay off and OpenOCD connection failures. With correct firmware v2.2.3, Debug Probe operates at full performance. VS Code debugging integration validated. FreeRTOS now running: dual-core SMP scheduler operational, sensor task (1kHz Core 1), logger task (queue-based Core 1), UI task (LED + USB serial Core 0), all tasks executing with correct priority and core affinity. USB serial output confirmed with 5-second status reports showing sensor samples, logger processed count, queue depth, and free heap.

(FreeRTOSConfig.h, .vscode/launch.json, PROJECT_STATUS.md)

*Rationale: Hardware debugging with direct connection via Debug Probe enabled precise identification of configuration bugs through GDB backtraces showing exact assertion failures. The debugprobe.uf2 vs debugprobe_on_pico.uf2 distinction is critical - wrong firmware has incompatible GPIO pin assignments for official Debug Probe hardware. Both configASSERT() catches prevented silent failures and led directly to root causes. This validates the entire FreeRTOS SMP toolchain and establishes hardware debugging workflow for Phase 1 sensor integration.*

### 2026-01-11-003 | Claude | bugfix, hardware, tooling

Fixed board configuration for Adafruit Feather RP2350 (was incorrectly set to standard Pico2), corrected `taskDISABLE_INTERRUPTS()` to `portDISABLE_INTERRUPTS()` in FreeRTOSConfig.h and hooks.c, added VS Code Extension configuration to CMakeLists.txt, created simple_test.c for hardware validation. Simple LED test passes on hardware (LED pin 7, COM7 USB serial), but FreeRTOS SMP firmware crashes during task creation after USB initialization. Next step: use Adafruit Debug Probe with OpenOCD + GDB to identify crash location.

(CMakeLists.txt, FreeRTOSConfig.h, src/hooks.c, src/simple_test.c, openocd_cmsis_dap.cfg, PROJECT_STATUS.md)

*Rationale: Wrong board configuration (pico2 vs adafruit_feather_rp2350) caused LED and platform issues. The taskDISABLE_INTERRUPTS vs portDISABLE_INTERRUPTS bug would have caused silent crashes in error handlers. Simple test confirms hardware works, isolating the issue to FreeRTOS initialization. Debug probe setup prepared for next session.*

### 2026-01-11-002 | Claude | bugfix

Fixed FreeRTOS build errors: removed problematic pico/config.h include from FreeRTOSConfig.h that caused include order issues, added missing configSUPPORT_STATIC_ALLOCATION required for static task memory hooks in hooks.c, corrected include order in hooks.c (FreeRTOS.h must precede pico headers).

(FreeRTOSConfig.h, src/hooks.c)

*Rationale: Build was failing due to cascading header issues from pico/config.h being included at wrong point in preprocessing, and missing static allocation config that the SMP port requires for idle task memory.*

### 2026-01-11-001 | Claude | documentation

Added GETTING_STARTED.md: toolchain setup, cloning with submodules, building, and flashing instructions for new contributors.

### 2026-01-10-001 | Claude | tooling, feature, documentation

Implemented FreeRTOS SMP validation for RP2350: Added pico-sdk and FreeRTOS-Kernel as git submodules, created CMakeLists.txt with RP2350 Armv8-M configuration, added FreeRTOSConfig.h with critical SMP settings (dual-core, FPU, TrustZone disabled), implemented validation test firmware with sensor task (Core 1, 1kHz), logger task (Core 1, queue-based), and UI task (Core 0, LED + USB serial status reports). Created build.sh script with ARM toolchain detection and comprehensive toolchain validation documentation with expected behavior and failure mode debugging guide. Updated .gitignore for CMake build artifacts (build/, CMakeCache.txt, etc.). Abandoned PlatformIO approach (requires Arduino framework for RP2350) in favor of pure CMake + Pico SDK as decided by council.

(pico-sdk/, FreeRTOS-Kernel/, pico_sdk_import.cmake, FreeRTOS_Kernel_import.cmake, CMakeLists.txt, FreeRTOSConfig.h, src/main.c, src/hooks.c, build.sh, docs/TOOLCHAIN_VALIDATION.md, .gitignore, PROJECT_STATUS.md, CHANGELOG.md)

*Rationale: First-time validation of FreeRTOS SMP on RP2350 hardware. Pure CMake approach provides direct hardware access without Arduino abstraction layers, aligning with real-time requirements. Validation firmware exercises core affinity, inter-task queues, and priority scheduling. Branch cleanup ensures build artifacts don't clutter the repository.*

### 2026-01-09-003 | Claude | documentation, architecture

SAD v1.0 cleanup: fixed section numbering (2.5→2.4, 3.3.4→3.3.2), reordered sections so Open Questions and References are last, marked placeholder sections (Power, Extensibility), added state machine formalism to open questions, created standards/ directory with DEBUG_OUTPUT.md, added phase-to-section cross-reference appendix, marked document approved for Phase 1.

(docs/SAD.md, standards/CODING_STANDARDS.md, standards/DEBUG_OUTPUT.md, README.md, PROJECT_OVERVIEW.md)

*Rationale: Council review identified numbering issues and missing debug output specification. Reorganization improves document navigability for phase-based development.*

### 2026-01-09-002 | Grok | documentation, tooling
Finalized SAD v1.0 with watchdog and Graphviz details; added back-burner evaluator note; created state_to_dot.py script.

(Docs/SAD.md, PROJECT_STATUS.md, tools/state_to_dot.py, Makefile)

*Rationale: Integrates council feedback for Phase 1 readiness.*

### 2026-01-09-001 | Claude | documentation, refactor
Streamlined PROJECT_STATUS.md (removed redundant Recently Completed section), made PROJECT_OVERVIEW.md more static/descriptive, added I2C vs SPI performance note to HARDWARE.md, removed legacy lib/ folder and .gitmodules for clean Phase 0 start.

(PROJECT_STATUS.md, PROJECT_OVERVIEW.md, HARDWARE.md, lib/, .gitmodules)

### 2026-01-08-001 | Grok | documentation, refactor
Updated root .gitignore to remove Arduino IDE-specific ignores (e.g., *.hex, *.elf, *.ino.cpp, build/, sketch_*/) and the erroneous *.cpp ignore entry. Retained PlatformIO/VSCode essentials (e.g., .pio/, .vscode/ temps) for repo hygiene.

*Transitions from Arduino compat to native VSCode+PlatformIO setup, allowing proper commits of C/C++ source files (e.g., firmware/2350/src/main.cpp for FreeRTOS SMP milestone). No impact on existing commits; improves future firmware scaffolding.*

(.gitignore)

### 2026-01-06-001 | Claude | hardware, architecture, documentation
Comprehensive on-hand hardware inventory update from Adafruit order history. Added sensors (MPU-6050, BMP280, VL53L4CD ToF), dev boards (ESP32-S3 Reverse TFT, Feather M0 Adalogger), accessories (OLED FeatherWing, Proto PiCowBell, 150mAh battery). Corrected IMU from LSM6DSOX to ISM330DHCX (#4569). Marked GPS units as on-hand. Reorganized project structure: archived Dev/ and docs/ folders to DEPRECATED/, promoted agent instruction files to root level for better visibility.

*Hardware inventory now reflects actual parts available for prototyping. Agent instruction files now live at project root. Legacy development code and outdated documentation preserved in DEPRECATED/ for reference.*

(HARDWARE.md, Dev/→DEPRECATED/Dev/, docs/→DEPRECATED/docs/, Agent_Instructions/*→root)

### 2026-01-05-002 | Claude | hardware
Corrected telemetry hardware: LoRa FeatherWing RFM95W (#3231) is current testing board, not RFM69HCW. Added GCS receiver (#3072) and Feather M0 RFM69HCW (#3176) to on-hand inventory.

*Previous confusion between RFM69 and LoRa variants resolved. M0-based #3176 retained but noted as limited utility due to dated MCU.*

(HARDWARE.md)

### 2026-01-05-001 | Claude | hardware
Added on-hand development boards: Pico 2W (#6087), KB2040 (#5302), Tiny 2350 (#6248/Pimoroni PIM721). Tiny 2350 noted as potential Core board candidate due to compact footprint and castellations. Added Pimoroni as secondary vendor in Sourcing Policy.

*Pimoroni products sourced directly; some items previously carried by Adafruit are being delisted.*

(HARDWARE.md)

### 2026-01-04-001 | Claude | documentation, hardware
Consolidated hardware documentation and deprecated outdated files. Corrected RFM69HCW vs LoRa radio confusion across all agent instruction files. Updated project status to reflect Phase 0 scaffolding progress.

*RFM69HCW (#3229) confirmed as current testing board; LoRa (#3179) available as alternative. hardware_info.yaml deprecated in favor of HARDWARE.md. Legacy performance analysis files from Arduino codebase moved to DEPRECATED/.*

(HARDWARE.md, PROJECT_OVERVIEW.md, STANDARDS.md, PROJECT_STATUS.md, hardware_info.yaml→DEPRECATED/, DEPENDENCY_AUDIT.md→DEPRECATED/, PERFORMANCE_ANALYSIS.md→DEPRECATED/, PERFORMANCE_ISSUES_SUMMARY.md→DEPRECATED/)

### 2025-12-29-001 | Claude | documentation
Created initial agent instruction file set for multi-agent development workflow. Establishes documentation structure, coding standards references, hardware tracking, council review process, and cross-agent collaboration protocols.
(README.md, STANDARDS.md, HARDWARE.md, PROJECT_STATUS.md, COUNCIL_PROCESS.md, CROSS_AGENT_REVIEW.md, AGENT_WHITEBOARD.md)
