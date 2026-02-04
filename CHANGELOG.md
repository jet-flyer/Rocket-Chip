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
