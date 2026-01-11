# RocketChip Project Status

**Last Updated:** 2026-01-11 by Claude

## Current Phase
Phase 1 (Foundation) - Architecture defined, scaffolding complete, ready for implementation.

## Completed
- [x] Software Architecture Document (SAD) created - `docs/SAD.md`
- [x] Directory structure scaffolding defined - `docs/SCAFFOLDING.md`
- [x] Header stubs for core modules (HAL, services, Mission Engine)
- [x] PlatformIO configuration (Core/Main/Titan environments)
- [x] ArduPilot compatibility shim structure
- [x] Product naming finalized: Core, Main+Packs, Titan (Nova reserved)
- [x] Logging format decision: MAVLink binary default, CSV/MATLAB export
- [x] Toolchain validation: Pure CMake + Pico SDK + FreeRTOS SMP (PlatformIO requires Arduino framework for RP2350)
- [x] GETTING_STARTED.md created
- [x] Board configuration corrected for Adafruit Feather RP2350 (was configured for standard Pico2)
- [x] Simple LED blink test validated on hardware

## Active Work
- [ ] Debug FreeRTOS SMP firmware with Adafruit Debug Probe (CMSIS-DAP) - program reaches main(), USB initializes, but crashes during FreeRTOS task creation. Use OpenOCD + GDB to get stack trace and identify root cause.
- [ ] Fix FreeRTOS initialization issue and complete hardware validation
- [ ] Initialize TinyUSB submodule in pico-sdk (eliminates build warning)
- [ ] Hardware verification tests (sensors functional on dev board)
- [ ] Implement HAL drivers (IMU, Baro)

## Blocked On
Nothing currently.

## Next Milestone
Phase 1 complete: Project compiles, uploads, runs FreeRTOS blink task. Basic sensor reads working. CI passes.

## Architecture Documents
- `docs/SAD.md` - Software Architecture Document (modules, interfaces, task model)
- `docs/SCAFFOLDING.md` - Directory structure and implementation status

## Back Burner
- Middle tier product name decision
- Ground station software evaluation
- Telemetry protocol final selection (CRSF/CCSDS vs raw MAVLink)
- Evaluate compliant stand-in for MissionEngine condition evaluator (custom non-recursive parser per MISRA/JSF; skip if core requirements inherently non-compliant)

## Side Projects
- Pegasus flight controller (full FC for drones, glide-back boosters - informed by madflight FC3)
- Fruit Jam as potential ground station platform
- OpenMCT integration exploration

## Known Issues
- Legacy Arduino codebase archived in DEPRECATED/ (not carried forward)

## Future Documentation Planned
- `docs/ICD.md` - Interface Control Document (Booster Pack connector)
- `docs/TEST_PLAN.md` - Test procedures and criteria
