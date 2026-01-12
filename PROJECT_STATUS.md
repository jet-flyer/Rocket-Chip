# RocketChip Project Status

**Last Updated:** 2026-01-12 by Claude Code CLI

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
- [x] Raspberry Pi Debug Probe configured with correct firmware (debugprobe.uf2) and OpenOCD
- [x] FreeRTOS SMP validation complete: dual-core scheduler running, tasks executing on both cores with proper priority and affinity, inter-task queues operational, USB serial output working
- [x] Hardware debugging workflow established: OpenOCD + GDB + VS Code integration functional

## Active Work
- [ ] Initialize TinyUSB submodule in pico-sdk (eliminates build warning)
- [ ] Hardware verification tests (sensors functional on dev board)
- [ ] Implement HAL drivers (IMU, Baro)

## Blocked On
Nothing currently.

## Next Milestone
Phase 1 FreeRTOS validation: ✅ COMPLETE
- Project compiles, uploads, runs FreeRTOS SMP scheduler with multiple tasks
- Dual-core operation validated (Core 0: UI task, Core 1: Sensor + Logger tasks)
- Hardware debugging functional (Debug Probe + OpenOCD + GDB)

Next: Sensor integration (IMU, Barometer) and HAL driver implementation

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
