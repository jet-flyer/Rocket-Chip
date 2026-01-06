# Changelog

## Format
`### YYYY-MM-DD-NNN | Author | tags`

Optional rationale in italics below. Files affected in parentheses if relevant.

**Frequency:** Typically one entry per session, not per individual change. Log when a task is completed or work transitions to a new focus. However, if multiple *significant* changes occur in one session (e.g., refactoring logging system AND redesigning state engine), create separate entries for each.

**Tags:** bugfix, feature, architecture, tooling, hardware, council, documentation, refactor

### 2026-01-06-001 | Claude | hardware
Comprehensive on-hand hardware inventory update from Adafruit order history. Added sensors (MPU-6050, BMP280, VL53L4CD ToF), dev boards (ESP32-S3 Reverse TFT, Feather M0 Adalogger), accessories (OLED FeatherWing, Proto PiCowBell, 150mAh battery). Corrected IMU from LSM6DSOX to ISM330DHCX (#4569). Marked GPS units as on-hand.

*Hardware inventory now reflects actual parts available for prototyping. Excluded ancient MCUs (32u4 Adalogger) and unrelated items (stepper motor).*

(HARDWARE.md)

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
