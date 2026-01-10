# Changelog

## Format
`### YYYY-MM-DD-NNN | Author | tags`

Optional rationale in italics below. Files affected in parentheses if relevant.

**Frequency:** Typically one entry per session, not per individual change. Log when a task is completed or work transitions to a new focus. However, if multiple *significant* changes occur in one session (e.g., refactoring logging system AND redesigning state engine), create separate entries for each.

**Tags:** bugfix, feature, architecture, tooling, hardware, council, documentation, refactor

### 2026-01-10-001 | Claude | tooling, feature, documentation

Implemented FreeRTOS SMP validation for RP2350: Added pico-sdk and FreeRTOS-Kernel as git submodules, created CMakeLists.txt with RP2350 Armv8-M configuration, added FreeRTOSConfig.h with critical SMP settings (dual-core, FPU, TrustZone disabled), implemented validation test firmware with sensor task (Core 1, 1kHz), logger task (Core 1, queue-based), and UI task (Core 0, LED + USB serial status reports). Created comprehensive toolchain validation documentation with expected behavior and failure mode debugging guide. Abandoned PlatformIO approach (requires Arduino framework for RP2350) in favor of pure CMake + Pico SDK as decided by council.

(pico-sdk/, FreeRTOS-Kernel/, pico_sdk_import.cmake, FreeRTOS_Kernel_import.cmake, CMakeLists.txt, FreeRTOSConfig.h, src/main.c, src/hooks.c, docs/TOOLCHAIN_VALIDATION.md, PROJECT_STATUS.md)

*Rationale: First-time validation of FreeRTOS SMP on RP2350 hardware. Pure CMake approach provides direct hardware access without Arduino abstraction layers, aligning with real-time requirements. Validation firmware exercises core affinity, inter-task queues, and priority scheduling.*

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
