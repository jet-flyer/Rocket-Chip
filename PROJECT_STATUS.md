# RocketChip Project Status

**Last Updated:** 2026-01-09 by Claude

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

## Active Work
- [ ] Validate PlatformIO + RP2350 + FreeRTOS toolchain
- [ ] Hardware verification tests (sensors functional on dev board)
- [ ] Implement HAL drivers (IMU, Baro)
- [ ] GETTING_STARTED.md for fresh-clone verification

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
