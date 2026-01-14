# Changelog

## Format
`### YYYY-MM-DD-NNN | Author | tags`

Optional rationale in italics below. Files affected in parentheses if relevant.

**Frequency:** Typically one entry per session, not per individual change. Log when a task is completed or work transitions to a new focus. However, if multiple *significant* changes occur in one session (e.g., refactoring logging system AND redesigning state engine), create separate entries for each.

**Conciseness:** Keep entries brief. Minor changes (doc clarifications, small fixes, typos) should be 1-2 sentences. Save detail for significant technical work. Rationale section is only needed when the "why" is non-obvious or involves important architectural decisions. Straightforward changes don't need rationale.

**Tags:** bugfix, feature, architecture, tooling, hardware, council, documentation, refactor

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
