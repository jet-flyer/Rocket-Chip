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

---

### 2026-02-08-002 | Claude Code CLI | refactor, documentation

**Standards audit remediation — Tier 4: RC_ASSERT, goto elimination, deviation docs**

Completed Phase D of the audit remediation plan. D1: Added `RC_ASSERT()` macro to `config.h` (debug = printf + watchdog spin, release = no-op). D2: Eliminated all 6 `goto` statements in `cmd_accel_6pos_cal()` by extracting body into `cmd_accel_6pos_cal_inner()` helper — wrapper guarantees I2C master pre/post hook execution. D4: Documented bare-metal loop deviations (BM-1 through BM-6), stdio usage (IO-1/IO-2), and preprocessor defines (PP-1) in `STANDARDS_DEVIATIONS.md`. Updated audit dashboard: 220 PASS / 25 PARTIAL/FAIL (90% compliance, up from 82%).

(`include/rocketchip/config.h`, `src/cli/rc_os.c`, `standards/STANDARDS_DEVIATIONS.md`, `standards/AUDIT_REMEDIATION.md`, `standards/STANDARDS_AUDIT_2026-02-07.md`)

---

### 2026-02-08-001 | Claude Code CLI | architecture, council

**F' (F Prime) comprehensive evaluation for Titan**

Expanded the F' addendum in `TITAN_BOARD_ANALYSIS.md` from a brief note into a full 14-section evaluation. Covers: F' architecture and component model, coding standards alignment with JSF AV (near-complete overlap), platform support matrix (RP2350 HSTX is officially supported via fprime-arduino), the critical multicore limitation (Zephyr has no Cortex-M SMP — F' multicore only works on Linux via pthreads), Pi Zero 2 W vs STM32H7 hardware comparison, a hybrid architecture proposal (Pi Zero 2 W running F'/Linux as mission CPU + RP2350 as real-time safety CPU), MAVLink incompatibility (F' uses its own protocol), F' flight heritage (Ingenuity, ASTERIA, RapidScat), and a "cherry-pick" alternative for adopting F' patterns without the framework.

(`docs/decisions/TITAN_BOARD_ANALYSIS.md`)

---

### 2026-02-07-001 | Claude Code CLI | documentation

**New standard: Vendor & OEM Guidelines**

Created `standards/VENDOR_GUIDELINES.md` — centralized reference for vendor-specific constraints, datasheet-sourced values, and OEM recommendations. Consolidates knowledge previously scattered across LL entries, driver comments, IVP notes, and whiteboard flags. Covers ICM-20948 (bank-switching, I2C master race), DPS310 (config, noise spec gaps), PA1010D (255-byte full-buffer reads per vendor app note, bus interference behavior, PMTK commands), RP2350 (errata E2, USB/flash ordering, memory constraints), and Feather board pin assignments. Includes datasheet inventory with gap analysis — DPS310 and PA1010D datasheets are missing locally.

(`standards/VENDOR_GUIDELINES.md`)

---

### 2026-02-06-003 | Claude Code CLI | architecture, documentation

**Stage 4 GPS IVP revision — restructured for dual-core architecture**

Rewrote IVP-31 through IVP-34 (now IVP-31 through IVP-33) after Stage 3 established that Core 1 owns the I2C bus exclusively. Original IVPs assumed GPS could run on Core 0 — this causes bus collisions (LL Entry 20). Key changes: GPS init before Core 1 launch, GPS reads on Core 1 sensor loop (full 255-byte reads at 10Hz per vendor recommendation), seqlock for Core 0 access, old IVP-33 (Core 1 migration) merged into IVP-31 (it's now a prerequisite, not a follow-on). Updated `gps_pa1010d.c` with full-buffer reads and PMTK314 sentence filter. Renumbered IVP-35+ down by 1 to close the gap (now IVP-34 through IVP-63). Reverted partial IVP-31 implementation from main.cpp.

(`docs/IVP.md`, `src/drivers/gps_pa1010d.c`, `src/main.cpp`)

*Rationale: The original Stage 4 IVPs were written before Stage 3's dual-core work revealed that I2C bus access is single-core only (no mutual exclusion in Pico SDK). Attempting Core 0 GPS reads with Core 1 running IMU/baro caused ICM-20948 init failures. Council review confirmed GPS on Core 1 is the only correct approach. The 32-byte read size was an Arduino Wire.h software limitation — vendor app notes and Pico SDK examples both use full 255-byte reads.*

---

### 2026-02-06-002 | Claude Code CLI | architecture, council, documentation

**Stage 3 prep: Seqlock cross-core design — research, council review, IVP corrections**

Created `docs/decisions/SEQLOCK_DESIGN.md` — council-reviewed decision document for cross-core data sharing via seqlock. Four parallel research agents investigated struct layout, prior art (ArduPilot/Betaflight/PX4), RP2350 memory model, and notification mechanisms. Council (3 personas) unanimously approved with 7 required modifications (all incorporated): bounded retry loop, `core1_loop_count`, `mag_read_count`, `_Static_assert` guards, DMB rationale comments, `_Atomic bool cal_reload_pending` signaling. Corrected RP2350 errata reference from E17 to E2 across IVP and whiteboard. Audited SAD Section 4.3 seqlock code — found 5 issues (missing DMB barriers, unnecessary double-buffer, wrong errata ID, nonexistent types, missing timestamps). Updated `AGENT_WHITEBOARD.md` with Stage 3 session plan and research findings.

(`docs/decisions/SEQLOCK_DESIGN.md`, `docs/IVP.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-06-001 | Claude Code CLI | feature, bugfix

**IVP-15/16/17/18: Calibration suite + CLI integration — Minimum Viable Demo milestone**

Fixed ICM-20948 returning all zeros after ~150 rapid accel reads during 6-position calibration. Root cause: internal I2C master (for AK09916 mag) races with external reads on shared bank-select register (0x7F). Added `icm20948_set_i2c_master_enable()` API and pre/post calibration hooks to disable I2C master during sampling. Removed motion check and orientation pre-check (ArduPilot doesn't use either — Gauss-Newton solver handles bad data). Reordered positions to QGroundControl standard. Added `i2c_bus_reset()` after flash saves. Version bumped to 0.1.1. HW-verified: 3/3 consecutive runs pass across USB replug cycles.

(`src/drivers/icm20948.c/h`, `src/calibration/calibration_manager.c`, `src/cli/rc_os.c/h`, `src/main.cpp`, `scripts/accel_cal_6pos.py`, `include/rocketchip/config.h`)

---

### 2026-02-05-005 | Claude Code CLI | feature

**IVP-14: Calibration storage (flash persistence)**

Re-enabled calibration system (`calibration_data`, `calibration_manager`, `calibration_storage`) in CMakeLists.txt with `hardware_flash` and `pico_flash` libs. Added storage init before USB (per LL Entry 4/12), manager init after sensors. IVP-14 self-test verifies all 4 gates: load/defaults, save/readback match, power cycle persistence, 10 consecutive saves (wear leveling). Added `kSkipVerifiedGates` flag to skip IVP-10/12/13 at boot — set false to re-run. All gates pass across power cycles.

(`CMakeLists.txt`, `src/main.cpp`)

---

### 2026-02-05-004 | Claude Code CLI | feature

**IVP-13a: I2C bus recovery under fault**

Implemented I2C bus recovery for sensor disconnect/reconnect scenarios. Recovery fires after 50 consecutive errors (~333ms), performs bus reset (deinit + 9-clock bit-bang + STOP + reinit). IMU self-recovers from reads; baro gets lazy reinit after 100 consecutive failures when IMU confirms bus is healthy. Hardware-verified: Qwiic cable disconnect during 100Hz/50Hz polling — no hang, both sensors resume at full rate after reconnect, error counter visible in status output. All 4 IVP-13a gates pass.

(`src/main.cpp`)

---

### 2026-02-05-003 | Claude Code CLI | feature

**IVP-09 through IVP-13: IMU, barometer, and multi-sensor polling**

Re-enabled ICM-20948 and DPS310 drivers incrementally with verification at each gate. IMU: fixed 6 magnetometer init issues, added 3-attempt retry, set ±4g/±500dps. DPS310: ruuvi library with 8Hz/8x oversampling continuous mode. IVP-10 validation (50 IMU samples, magnitude-based gate checks), IVP-12 validation (100 baro samples, pressure/noise/stuck-value gates), IVP-13 multi-sensor polling (IMU 100Hz, baro 50Hz, 60s, zero I2C errors). Measured I2C timing: IMU avg 774us, baro avg 251us. All gates pass across reboot cycles.

(`CMakeLists.txt`, `src/main.cpp`, `src/drivers/icm20948.c`, `docs/PROJECT_STATUS.md`)

---

### 2026-02-05-002 | Claude Code CLI | bugfix, refactor

**Reset to clean Stage 1 baseline, fix I2C bus recovery**

Stripped all Stage 2 code from build (drivers, calibration, CLI) to resolve I2C bus reliability issues. Root cause: two compounding issues — (1) no bus recovery on boot after picotool `--force` reboots left sensors mid-transaction with SDA held low, (2) Stage 2 init code (IMU bank switching, mag I2C master, flash ops) was corrupting bus state. Added `i2c_bus_recover()` call before `i2c_init()` in `i2c_bus_init()`. Restored 400kHz (100kHz was a red herring). All 3 devices (0x69, 0x77, 0x10) now detected reliably. Stage 2 source files remain on disk for incremental re-enablement.

(`CMakeLists.txt`, `src/main.cpp`, `src/drivers/i2c_bus.c`, `src/drivers/i2c_bus.h`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-05-001 | Claude Code CLI | documentation, architecture

**Telstar Booster Pack, docs/hardware/ reorganization, Gemini ELRS section**

Created `docs/hardware/` subdirectory and moved hardware design documents into it (`HARDWARE.md`, `GEMINI_CARRIER_BOARD.md`, `STATUS_INDICATORS.md`) using `git mv` to preserve history. Created `docs/hardware/TELSTAR_BOOSTER_PACK.md` — Telstar Booster Pack design document covering ELRS RC link, CRSF protocol, FPV video transmitter, standalone product potential, FAA Remote ID module support, and updated Booster Pack lineup. Added Section 8.4 (Dedicated ELRS Communications Core) to Gemini doc. Updated all cross-references across repo (README, SAD, SCAFFOLDING, PROJECT_OVERVIEW, HARDWARE, ICDs, SpaceWire-Lite).

---

### 2026-02-04-003 | Claude Code CLI | bugfix

**I2C debug: drop to 100kHz, add verbose scan output**

Cross-referenced current I2C code against working `AP_FreeRTOS` branch. Code is functionally identical (same pins, same instance, same pull-ups) except the working `i2c_scan.c` test used 100kHz, not 400kHz. Changed `I2C_BUS_FREQ_HZ` to 100kHz. Added verbose scan output showing I2C instance number, GPIO pin states, and configured frequency. Build ready, not yet flashed.

(`src/drivers/i2c_bus.c`, `src/drivers/i2c_bus.h`)

---

### 2026-02-04-002 | Claude Code CLI | feature

**IVP Stage 2: RC_OS CLI + Calibration Integration**

Implemented bare-metal RC_OS CLI with calibration integration (IVP-15, IVP-16, IVP-18). CLI provides single-key command interface with calibration menu, sensor status, and I2C rescan capability. Sensor availability checks prevent calibration commands when sensors not initialized. Non-blocking calibration progress monitoring with dots and OK/FAIL output.

**Status:** Calibration logic complete but blocked on I2C issue — see AGENT_WHITEBOARD.md for details.

---

### 2026-02-04-001 | Claude Code CLI | feature, hardware

**IVP Stage 1 Complete: Foundation (IVP-01 through IVP-08)**

Created `src/main.cpp` implementing bare-metal firmware foundation: red LED heartbeat, NeoPixel rainbow via PIO, USB CDC serial with terminal reconnect handling, I2C bus init and scan, structured HW validation output. All gates hardware-verified (ICM-20948, DPS310, PA1010D detected).

Deleted stale files with broken dependencies (`accel_calibrator.c/h`, `debug.h`). Updated CMakeLists.txt to build only Stage 1 sources; other drivers commented with IVP stage markers for incremental re-enablement.

---

### 2026-02-03-007 | Claude Code CLI | tooling

Added plan mode council review instructions to `.claude/CLAUDE.md`. Before ExitPlanMode, agent now asks which council personas to use, spawns a Task agent to run the review, and attaches the verdict to the plan.

---

### 2026-02-03-006 | Claude Code CLI | documentation, architecture

Added Section 8.2 (Dual-IMU Fusion and EKF Lane Switching) to `docs/GEMINI_CARRIER_BOARD.md`. Covers EKF3-style lane switching concept using Gemini's dual independent sensor suites, bandwidth analysis for high-rate bidirectional sensor exchange over SpaceWire-Lite, and protocol implications including new SENSOR_RAW and EKF_HEALTH message types.

---

### 2026-02-03-005 | Claude Code CLI | documentation, architecture, council

**Integration and Verification Plan (IVP) + SAD Updates**

Created `docs/IVP.md` — 64-step development roadmap across 9 stages with pass/fail verification gates. Council review (4 personas) produced 12 findings, all implemented: `⚠️ VALIDATE` convention for numerical values, RP2350 inter-core primitives section (spinlocks, FIFO, doorbells as IVP-21 through IVP-23), GPS as Stage 4 before fusion, I2C bus recovery, MPU details, milestone markers.

SAD.md updates: per-sensor validity flags, seqlock implementation replacing TODO, inter-core primitives table, PIO allocation table, AMP architecture corrections, Section 14 numbering fix. SCAFFOLDING.md and PROJECT_STATUS.md updated with PIO_ALLOCATION.md placeholders.

---

### 2026-02-03-004 | Claude Code CLI | documentation, architecture

Added PIO hardware watchdog design concepts document (`docs/PIO/PIO_WATCHDOG.md`). Covers heartbeat watchdog, dual-core cross-check, and pyro channel lockout using PIO state machines as CPU-independent safety monitors. Concept stage — not committed to IVP.

---

### 2026-02-03-003 | Claude Code CLI | documentation, refactor

**Bare-Metal Pivot — Documentation Cleanup (continued)**

Systematic cleanup of FreeRTOS/ArduPilot references across all documentation.

- LESSONS_LEARNED.md — archived 8 FreeRTOS-specific entries (7-10, 14, 17-19), minor rewording on entries 3, 4, 12, 15
- DEBUG_OUTPUT.md — replaced deferred logging with direct printf macros, renamed per-task to per-module, updated build config table
- CODING_STANDARDS.md — removed RTOS tier table, ArduPilot Library Integration, Dependency Bypassing Policy, HAL Adaptation Policy sections; demoted ArduPilot from "check first" to "useful reference" in Prior Art Research; removed archived LL entry references
- PROJECT_OVERVIEW.md — replaced "RTOS" with "deterministic control loops" in Titan tier, "FreeRTOS" with "Bare-metal Pico SDK" in Technical Foundation
- PROJECT_STATUS.md — added ChibiOS upstream note, added full ArduPilot integration as back burner goal
- ROCKETCHIP_OS.md — replaced FreeRTOS Task Model with bare-metal Execution Model, updated source files table, replaced platform-specific FreeRTOS issues with USB CDC concerns, removed priority-based references
- Deleted GETTING_STARTED.md (thoroughly outdated, not needed until development matures)

---

### 2026-02-03-002 | Claude Code CLI | architecture, refactor

**Pivot from FreeRTOS to Bare-Metal Pico SDK**

Removed all FreeRTOS dependencies, pivoting to bare-metal Pico SDK with polling main loop.

**Deleted:**
- FreeRTOS-Kernel submodule, FreeRTOSConfig.h, FreeRTOS_Kernel_import.cmake
- docs/FreeRTOS/TASK_PRIORITIES.md
- src/main.cpp, src/tasks/sensor_task.c/h, src/debug/debug_stream.c/h (RTOS glue — will be rewritten)

**Edited:**
- CMakeLists.txt — removed FreeRTOS imports, heap link, commented out deleted sources
- config.h — replaced task priorities/stacks with polling timing constants, simplified debug macros to direct printf
- CODING_STANDARDS.md — replaced FreeRTOS platform constraints with bare-metal rules
- SAD.md, PROJECT_STATUS.md, SCAFFOLDING.md, MULTICORE_RULES.md — updated for bare-metal architecture
- DEBUG_PROBE_NOTES.md, SESSION_CHECKLIST.md — minor RTOS reference removal
- ws2812_status.h — comment update

**Deferred for later evaluation (>40% rewrite needed):**
- DEBUG_OUTPUT.md — deferred logging architecture references deleted code
- LESSONS_LEARNED.md — ~50% of entries are FreeRTOS-specific

**Preserved unchanged:** src/calibration/*, src/drivers/* (sensor drivers, calibration, LED — no RTOS dependency)

---

### 2026-02-03-001 | Claude Code CLI | documentation

**Platform Constraints, Multi-Core Rules, Session Checklist, Decisions Folder**

Added critical documentation capturing RP2350 + FreeRTOS SMP platform constraints and session management procedures.

- Added `standards/CODING_STANDARDS.md` "RP2350 + FreeRTOS SMP Platform Constraints" section (~98 lines) - non-negotiable rules derived from LESSONS_LEARNED entries
- Created `docs/MULTICORE_RULES.md` - core assignment rules, cross-core communication, memory barriers
- Created `.claude/SESSION_CHECKLIST.md` - session start/end procedures, handoff protocol
- Created `docs/decisions/` folder structure for council review outputs
- Moved `docs/ESKF/` to `docs/decisions/ESKF/`
- Updated `docs/PROJECT_STATUS.md` to "Reboot: Validate & Rebuild" phase with validation checklist
- Updated `.claude/PROTECTED_FILES.md` with new protected entries
- Updated `.claude/CLAUDE.md` with new @ references

**Files:** CODING_STANDARDS.md, MULTICORE_RULES.md, SESSION_CHECKLIST.md, docs/decisions/README.md, PROJECT_STATUS.md, PROTECTED_FILES.md, CLAUDE.md

---

### 2026-02-02-004 | Claude Code CLI | bugfix, architecture

**FreeRTOS SMP + USB CDC Fix + Calibration Stack Fix**

Fixed USB CDC enumeration failure on RP2350 with FreeRTOS SMP, plus stack overflow during 6-position accel calibration.

**USB CDC Root cause:** Using main FreeRTOS repo instead of Raspberry Pi's fork, plus incorrect FreeRTOSConfig.h settings.

**Calibration Root cause:** Sensor task stack (256 words) too small for ellipsoid fit local matrices. Increased to 512 words.

**Changes:**
- Replaced FreeRTOS-Kernel submodule with raspberrypi/FreeRTOS-Kernel (has RP2350_ARM_NTZ port)
- Rewrote FreeRTOSConfig.h to match pico-examples/freertos/hello_freertos
- Removed custom USB task - SDK's IRQ-based handling is correct approach
- Pinned sensor sampling to Core 1 (Core 0 hosts USB IRQ handlers)
- Simplified CMakeLists.txt USB configuration to use SDK defaults
- Increased SENSOR_TASK_STACK from 256 to 512 words for calibration

**Files:** FreeRTOS-Kernel (submodule), FreeRTOSConfig.h, FreeRTOS_Kernel_import.cmake, src/main.cpp, CMakeLists.txt, src/tasks/sensor_task.c

**Reference:** LESSONS_LEARNED.md Entry 18 (USB), Entry 19 (stack overflow)

**Status:** Calibration runs to completion but ellipsoid fit diverges - needs algorithm tuning.

*Rationale: pico-examples/freertos/hello_freertos is the canonical reference for FreeRTOS on RP2350. Previous attempts used custom configurations that conflicted with SDK's USB stack. The Raspberry Pi FreeRTOS fork contains tested, working SMP support that the main FreeRTOS repo lacks.*

---

### 2026-02-02-003 | Claude Code CLI | documentation

**Documentation Cleanup for Fresh Start**

Reviewed all docs folder files to ensure consistency with bespoke FreeRTOS approach after branch reorganization.

- Updated SAD.md: Rewrote directory structure, module dependencies, driver interfaces, storage architecture, and reset all development phases
- Updated SCAFFOLDING.md: Aligned with SAD.md, reset implementation status
- Updated TOOLCHAIN_VALIDATION.md: Removed stale sensor references, added Phase 2 note
- Updated HARDWARE.md: Removed ArduPilot driver references from IMU and GPS sections
- Updated .claude/CLAUDE.md: Removed reference to deleted RP2350_FULL_AP_PORT.md
- Deleted: docs/RP2350_FULL_AP_PORT.md, docs/AP_HAL_RP2350_PLAN.md (archived in AP_FreeRTOS branch)
- Moved: PROJECT_STATUS.md, PROJECT_OVERVIEW.md to docs/ folder

ESKF docs and ROCKETCHIP_OS.md left as-is (historical reference for design decisions).

---

### 2026-02-02-002 | Nathan (via Claude Opus council review) | architecture

**Sensor Fusion Architecture: ESKF + MMAE**

Changed sensor fusion approach from ArduPilot EKF3 extraction to custom Error-State Kalman Filter (ESKF) with Multiple Model Adaptive Estimation (MMAE) bank for anomaly resilience.

- Added `docs/ESKF/FUSION_ARCHITECTURE_DECISION.md` - full rationale and design
- Added `docs/ESKF/FUSION_ARCHITECTURE.md` - reference document
- Updated SAD.md §5.4 (fusion), §10 (Phase 4), key decisions table, open questions
- Updated SCAFFOLDING.md FusionTask descriptions
- AP_HAL_RP2350 still used for calibration/math/storage - NOT for fusion
- Added `.claude/PROTECTED_FILES.md` - files requiring explicit permission to edit

**Note:** Specific numerical parameters (state counts, filter counts, latencies) pending systematic review before implementation.

---

### 2026-02-02-001 | Claude Code CLI | architecture

**Branch Reorganization - Fresh Start with Bespoke FreeRTOS**

Major pivot: Archived ArduPilot integration attempts, starting fresh with bespoke FreeRTOS approach.

- Created `AP_ChibiOS` branch (archived ChibiOS exploration - blocked by XIP flash issues)
- Created `AP_FreeRTOS` branch (archived FreeRTOS + ArduPilot HAL work - mature but complex)
- Fresh `main` branch for new bespoke implementation

Cherry-picked universal reference documentation (LESSONS_LEARNED, DEBUG_PROBE_NOTES, HARDWARE, SAD, SCAFFOLDING, ROCKETCHIP_OS, coding standards).

**Reference:** Full reorganization plan at `C:\Users\pow-w\.claude\plans\spicy-toasting-breeze.md`

---
