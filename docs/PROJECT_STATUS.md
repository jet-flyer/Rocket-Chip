# RocketChip Project Status

**Last Updated:** 2026-02-10

## Current Phase

**Stage 4 COMPLETE** — GPS Integration

All Stage 4 IVPs (31/32/33) hardware-verified. GPS fix confirmed outdoors, CLI displays all GPS data from seqlock. Next: complete foundational features (mag cal wizard, non-blocking USB, unified calibration) before Stage 5 ESKF.

## Completed

*Per-IVP detail in [IVP.md](IVP.md).*

| Stage | IVPs | Verified | Summary |
|-------|------|----------|---------|
| 1: Foundation | 01-08 | 2026-02-04 | Build, LED, NeoPixel, USB CDC, debug macros, I2C bus, heartbeat |
| 2: Single-Core Sensors | 09-18 | 2026-02-06 | IMU, baro, multi-sensor polling, calibration (gyro/level/6-pos), CLI |
| 3: Dual-Core | 19-30 | 2026-02-07 | Core 1 launch, atomics, spinlock, FIFO, doorbell, seqlock, sensor migration, USB/flash stress, MPU stack guard, watchdog |
| 4: GPS Integration | 31-33 | 2026-02-08 | PA1010D on Core 1, outdoor fix validated, CLI integration |
| Standards Audit | — | 2026-02-08 | 249 rules, 90% compliant, 25 accepted deviations. See `standards/STANDARDS_AUDIT_2026-02-07.md` |
| D3 Refactoring | — | 2026-02-09 | main() 992→65 lines, core1_entry() 367→15 lines. Tick-function dispatcher pattern. Binary unchanged |
| Automated Audit | — | 2026-02-09 | clang-tidy 127 checks, 1,251 findings across 10 files. See `docs/audits/CLANG_TIDY_AUDIT_2026-02-09.md` |
| IVP Code Strip | — | 2026-02-09 | main.cpp 3,623→1,073 lines (70% reduction). Binary 198,144→155,648 bytes (-21.4%). 2 deviations resolved |
| I2C Recovery Fix | — | 2026-02-10 | Fixed peripheral corruption in bus recovery (LL Entry 28). Probe-first detection, SCL/SDA improvements. 386K reads, 0 errors |
| Non-blocking USB | — | 2026-02-10 | Firmware runs without terminal. Boot banner deferred to first connect. Soak verified: 536K reads, 0 errors |
| Phase M: Mag Cal | IVP-34–38 | 2026-02-10 | Full magnetometer calibration: data structures, LM ellipsoid solver, CLI wizard, Core 1 live apply + heading. 4 commits, all HW verified |
| Phase M.5: Wizard | — | 2026-02-10 | Unified 5-step calibration wizard with NeoPixel feedback. Core 1 cal feeds (no I2C contention). Raw mag data in seqlock for recalibration. All 5 steps HW verified including full mag cal (300 samples, 81% coverage, RMS 2.499 uT) |
| I2C Bypass Mode | — | 2026-02-10 | ICM-20948 mag access migrated from I2C master to bypass mode (ArduPilot approach). Eliminates bank-switching race, master stall, disable/enable corruption. HW verified with GPS on bus: mag cal 300 samples, RMS 0.878 uT |

## In Progress

**All foundational features COMPLETE.** Ready for Stage 5 ESKF (IVP-39+).

1. ~~Soak baseline~~ — **DONE** (536K IMU reads, 0 errors, 6 min with all 3 sensors)
2. ~~Non-blocking USB init~~ — **DONE** (ec87703). Prior failures were picotool artifacts (LL Entry 25/27)
3. ~~Phase M: Magnetometer calibration~~ — **DONE** (4 commits: 94dffad, d261b66, c84c38c). CLI wizard + Core 1 live apply + heading display
4. ~~Unified calibration wizard~~ — **DONE** (wizard-12). 5-step wizard with NeoPixel feedback, ENTER/x navigation, raw mag data fix. Full mag cal HW verified (300 samples, 81% coverage, RMS 2.499 uT)
5. ~~I2C bypass mode~~ — **DONE** (bypass-5). Council-approved migration from I2C master to bypass mode. GPS pause during mag cal. Two-level device recovery

## Blockers

None currently.

## Reference

- `docs/IVP.md` — Full 68-step integration plan with verification gates (includes Phase M mag cal)
- `docs/SAD.md` — Software Architecture Document
- `docs/SCAFFOLDING.md` — Directory structure and file listing
- `standards/CODING_STANDARDS.md` — Platform constraints
- `.claude/LESSONS_LEARNED.md` — Debugging journal
