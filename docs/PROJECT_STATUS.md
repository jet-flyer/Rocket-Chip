# RocketChip Project Status

**Last Updated:** 2026-02-03

## Current Phase

**Reboot: Validate & Rebuild** — Bespoke FreeRTOS implementation on corrected foundation

## Context

Archived ArduPilot integration attempts after encountering fundamental blockers:
- `AP_ChibiOS` branch: XIP flash execution issues prevented viable ChibiOS port
- `AP_FreeRTOS` branch: Working but complex; fundamental architectural incompatibilities

Starting fresh with bespoke approach. Previous development revealed critical platform constraints (wrong FreeRTOS fork, incorrect core assignments, USB init ordering) that are now documented in `standards/CODING_STANDARDS.md` and `.claude/LESSONS_LEARNED.md`.

## What Exists (from previous work — needs validation against corrected platform rules)

Build system and RTOS:
- [x] CMakeLists.txt (Pico SDK + FreeRTOS SMP)
- [x] FreeRTOSConfig.h (based on pico-examples)
- [x] FreeRTOS-Kernel submodule (raspberrypi fork — CORRECT)
- [x] main.cpp with FreeRTOS task structure

Sensor drivers:
- [x] ICM-20948 9-DoF IMU (custom driver with magnetometer)
- [x] DPS310 barometer (ruuvi library wrapper)
- [x] PA1010D GPS (lwGPS library wrapper)

Infrastructure:
- [x] Calibration system with flash persistence (gyro, level, 6-position accel, baro)
- [x] RC_OS CLI menu (terminal-connected pattern)
- [x] Debug stream (deferred USB CDC logging)
- [x] WS2812 NeoPixel status LED driver

## Validation Needed (before building on top)

- [ ] Clean build from fresh clone
- [ ] USB CDC enumeration
- [ ] Core assignment correctness (SensorTask on Core 1, USB on Core 0)
- [ ] Sensor readings (IMU, baro)
- [ ] Calibration load/save across power cycles
- [ ] printf from unpinned tasks (validate cross-core USB safety)
- [ ] RC_OS CLI commands

## Caution

Previous code was developed partly under the WRONG FreeRTOS fork and with INCORRECT core assignments documented in SAD.md and TASK_PRIORITIES.md. Some existing code may contain assumptions or patterns from those errors. Validate before trusting.

## Next Steps (After Validation)

- [ ] Fix any issues found during validation
- [ ] GPS integration into task architecture
- [ ] Replace any remaining ArduPilot math dependencies
- [ ] Begin ESKF sensor fusion (Phase 4) — see `docs/decisions/ESKF/`

## Blockers

None — clean foundation with corrected platform rules.

## Reference Material

**Platform rules and constraints:**
- `standards/CODING_STANDARDS.md` — RP2350 + FreeRTOS SMP constraints section
- `docs/MULTICORE_RULES.md` — Core assignment and cross-core rules
- `.claude/LESSONS_LEARNED.md` — 19 debugging entries with root causes
- `.claude/SESSION_CHECKLIST.md` — Session handoff procedures
- `.claude/DEBUG_PROBE_NOTES.md` — OpenOCD/GDB setup

**Architecture:**
- `docs/SAD.md` — Software Architecture Document (⚠ core assignments need correction)
- `docs/FreeRTOS/TASK_PRIORITIES.md` — Task hierarchy (⚠ core assignments need correction)
- `docs/decisions/` — Council review outputs (ESKF architecture, etc.)

**Available in archive branches:**
- `AP_FreeRTOS` — Full FreeRTOS + ArduPilot implementation (for reference only)
- `AP_ChibiOS` — ChibiOS exploration (for reference only)

## Back Burner

- Middle tier product name decision
- Ground station software evaluation
- Telemetry protocol selection
- SAD.md / TASK_PRIORITIES.md core assignment correction
