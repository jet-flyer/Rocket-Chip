# RocketChip Project Status

**Last Updated:** 2026-01-04 by Claude

## Current Phase
Beginning clean-sheet refactor. Phase 0 (scaffolding) in progress.

## Active Work
- [ ] Archive legacy code to branch
- [ ] Validate PlatformIO + RP2350 + FreeRTOS toolchain
- [ ] Hardware verification tests (sensors functional on dev board)
- [ ] Finalize platformio.ini configurations
- [ ] GETTING_STARTED.md for fresh-clone verification

## Recently Completed
- [x] Agent instruction file set (all 9 files)
- [x] Software Architecture Document (SAD) created
- [x] Directory scaffolding (22 files)
- [x] Development plan and timeline (10-week MVP, 4-week MVP+)
- [x] Council review on refactor approach
- [x] ArduPilot/ChibiOS port feasibility research (confirmed stretch goal)
- [x] Council review on telemetry hardware (LoRa vs ELRS vs CRSF)
- [x] Hardware documentation consolidated (HARDWARE.md, deprecated hardware_info.yaml)
- [x] RFM69HCW vs LoRa clarified across all agent instructions
- [x] Council review process defined
- [x] RP2350 confirmed as primary platform (council reviewed)
- [x] Decision to abandon Arduino IDE (ToS concerns)
- [x] Cross-agent review protocol established

## Blocked On
Nothing currently.

## Next Milestone
Phase 0 complete: Project compiles, uploads, runs FreeRTOS blink task. CI passes. Hardware verified.

## Back Burner
- Middle tier product name decision
- Pro tier name decision (Nova vs Titan)
- Ground station software evaluation
- Telemetry protocol final selection (CRSF/CCSDS vs raw MAVLink)

## Side Projects
- Pegasus flight controller (full FC for drones, glide-back boosters - informed by madflight FC3)
- Fruit Jam as potential ground station platform
- OpenMCT integration exploration

## Known Issues
- Legacy Arduino codebase to be archived (not carried forward to new architecture)
