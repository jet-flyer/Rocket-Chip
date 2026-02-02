# RocketChip Project Status

**Last Updated:** 2026-02-02 by Claude Code CLI

## Current Phase

**Fresh Start** - Bespoke FreeRTOS implementation (post-branch reorganization)

## Context

Archived ArduPilot integration attempts after encountering fundamental blockers:
- `AP_ChibiOS` branch: XIP flash execution issues prevented viable ChibiOS port
- `AP_FreeRTOS` branch: Working but complex; FreeRTOS SMP Core 1 issues (PD15)

Starting fresh with simpler, bespoke approach that doesn't require ArduPilot HAL compatibility layer.

## Next Steps

- [ ] Set up fresh build system (CMake + Pico SDK + FreeRTOS)
- [ ] Create minimal `main.cpp` with FreeRTOS task structure
- [ ] Implement direct sensor interfaces (no HAL abstraction)
- [ ] Re-integrate RC_OS CLI menu structure (see `docs/ROCKETCHIP_OS.md`)
- [ ] Basic sensor reading (ICM-20948, DPS310)
- [ ] USB CDC serial output

## Blockers

None - clean slate.

## Reference Material

**Preserved from previous work:**
- `.claude/LESSONS_LEARNED.md` - 17 debugging entries
- `.claude/DEBUG_PROBE_NOTES.md` - OpenOCD/GDB setup
- `docs/HARDWARE.md` - Pin assignments, sensors
- `docs/ROCKETCHIP_OS.md` - CLI menu structure
- `docs/SAD.md` - Architecture concepts

**Available in archive branches:**
- `AP_FreeRTOS` - Full FreeRTOS + ArduPilot implementation (for reference)
- `AP_ChibiOS` - ChibiOS exploration (for reference)

## Architecture Documents

- `docs/SAD.md` - Software Architecture Document
- `docs/SCAFFOLDING.md` - Directory structure
- `docs/ROCKETCHIP_OS.md` - CLI design

## Back Burner

- Middle tier product name decision
- Ground station software evaluation
- Telemetry protocol selection
