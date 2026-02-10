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

## In Progress

**Foundational features — must complete before Stage 5 ESKF (IVP-39+):**

1. **Soak baseline (next session)** — 6-min soak via debug probe on current build to establish post-fix baseline
2. **Phase M: Magnetometer calibration wizard** — Commits 1+2 done (94dffad: data structures, apply functions, sample collection, LM solver). Commit 3 (CLI command) in stash@{0}. Commit 4 (Core 1 live apply) blocked by Commit 3
3. **Non-blocking USB init** — Retest via debug probe (prior failures were picotool artifacts per LL Entry 25/27)
4. **Unified calibration wizard** — Gyro, level, 6-pos accel, mag cal through single CLI flow

## Blockers

- **Non-blocking USB init** — Attempted and reverted (6de6245). All 4 build variants (prod-13 through prod-16) failed soak at 40-90s, but all testing used picotool rapid flash cycles — now proven to corrupt I2C bus (LL Entry 25/27). "Codegen sensitivity" hypothesis **disproved** (2026-02-09): three soak tests via debug probe with i2c_bus.cpp modifications all passed with 0 errors (~370K reads each). Should be re-attempted with probe-only testing.

## Reference

- `docs/IVP.md` — Full 68-step integration plan with verification gates (includes Phase M mag cal)
- `docs/SAD.md` — Software Architecture Document
- `docs/SCAFFOLDING.md` — Directory structure and file listing
- `standards/CODING_STANDARDS.md` — Platform constraints
- `.claude/LESSONS_LEARNED.md` — Debugging journal
