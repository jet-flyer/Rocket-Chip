# RocketChip Project Status

**Last Updated:** 2026-02-03

## Current Phase

**Reboot: Bare-Metal Pico SDK Pivot** — Rewriting from FreeRTOS to bare-metal Pico SDK

## Context

Archived ArduPilot integration attempts after encountering fundamental blockers:
- `AP_ChibiOS` branch: XIP flash execution issues prevented viable ChibiOS port (official RP2350 ChibiOS support is actively being developed upstream)
- `AP_FreeRTOS` branch: Working but complex; fundamental architectural incompatibilities

Pivoting away from FreeRTOS to bare-metal Pico SDK. FreeRTOS SMP added complexity (cross-core memory visibility, priority inversion, USB init ordering) without proportional benefit at this stage. Bare-metal simplifies the architecture while retaining dual-core capability through Pico SDK primitives. Previous platform constraints are documented in `standards/CODING_STANDARDS.md` and `.claude/LESSONS_LEARNED.md`.

## What Exists (from previous work — needs adaptation for bare-metal)

Sensor drivers:
- [x] ICM-20948 9-DoF IMU (custom driver with magnetometer)
- [x] DPS310 barometer (ruuvi library wrapper)
- [x] PA1010D GPS (lwGPS library wrapper)

Infrastructure:
- [x] Calibration system with flash persistence (gyro, level, 6-position accel, baro)
- [x] RC_OS CLI menu (terminal-connected pattern)
- [x] WS2812 NeoPixel status LED driver
- [x] CMakeLists.txt (needs update to remove FreeRTOS dependencies)

Deleted (need rewrite for bare-metal):
- main.cpp (was FreeRTOS task-based)
- sensor_task (was FreeRTOS task)
- debug_stream (was FreeRTOS stream buffer-based)

## Validation Needed (after bare-metal rewrite)

- [ ] Clean build from fresh clone
- [ ] USB CDC enumeration
- [ ] Sensor readings (IMU, baro)
- [ ] Calibration load/save across power cycles
- [ ] RC_OS CLI commands

## Caution

Code is being rewritten for bare-metal Pico SDK. Existing source files may still contain FreeRTOS includes, task primitives, or assumptions from the previous architecture. These need to be identified and replaced during the rewrite.

## Next Steps

- [ ] Write new bare-metal main.cpp (super-loop or timer-interrupt architecture)
- [ ] Implement sensor polling loop without FreeRTOS tasks
- [ ] Rewrite debug output for bare-metal (direct printf or simple ring buffer)
- [ ] Update CMakeLists.txt to remove FreeRTOS dependencies
- [ ] GPS integration
- [ ] Replace any remaining ArduPilot math dependencies
- [ ] Begin ESKF sensor fusion (Phase 4) — see `docs/decisions/ESKF/`

## Blockers

None — clean foundation, pivoting to simpler bare-metal approach.

## Reference Material

**Platform rules and constraints:**
- `standards/CODING_STANDARDS.md` — RP2350 platform constraints
- `docs/MULTICORE_RULES.md` — Core assignment and cross-core rules
- `.claude/LESSONS_LEARNED.md` — 19 debugging entries with root causes
- `.claude/SESSION_CHECKLIST.md` — Session handoff procedures
- `.claude/DEBUG_PROBE_NOTES.md` — OpenOCD/GDB setup

**Architecture:**
- `docs/SAD.md` — Software Architecture Document
- `docs/decisions/` — Council review outputs (ESKF architecture, etc.)

**Available in archive branches:**
- `AP_FreeRTOS` — Full FreeRTOS + ArduPilot implementation (for reference only)
- `AP_ChibiOS` — ChibiOS exploration (for reference only)

## Back Burner

- Middle tier product name decision
- Ground station software evaluation
- Telemetry protocol selection
- Full ArduPilot integration via ChibiOS HAL — official ChibiOS RP2350 support is actively being developed and imminent. Once available, a native AP_HAL_ChibiOS port becomes viable, enabling full ArduPilot firmware (ArduRocket) on RocketChip hardware
