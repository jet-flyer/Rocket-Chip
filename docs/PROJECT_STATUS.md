# RocketChip Project Status

**Last Updated:** 2026-02-18

## Current Phase

**Stage 5 IN PROGRESS** — Sensor Fusion (ESKF)

IVP-39 through IVP-46 complete. All measurement updates wired with real sensor feeds (baro, mag, GPS). ESKF outdoor-validated with GPS feeding position/velocity. Next: IVP-47 (attitude init refinement) and IVP-48 (health + diagnostics).

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
| 5: ESKF (partial) | IVP-39–44b | 2026-02-13 | Vec3/Quat/Mat math libs, ESKF propagation, replay harness, baro/mag/ZUPT measurement updates, WMM declination table (wired into mag update). 155/155 host tests pass. HW verified: stationary |
| 5: ESKF GPS | IVP-46 | 2026-02-18 | GPS position/velocity measurement updates. UART-first transport detection, P covariance reset fix, interrupt-driven ring buffer, ESKF init NeoPixel. Outdoor validated: Fix=3 Sats=12 HDOP=0.90. 60s soak: bNIS 0.00–2.13. 173/173 host tests |

## In Progress

**Stage 5: Sensor Fusion (ESKF)** — IVP-46 complete, IVP-47/48 next.

- IVP-39: Vec3/Quat/Mat math — DONE
- IVP-40: Matrix ops + state indices — DONE
- IVP-41: 1D baro KF — DONE (standalone filter removed from firmware; ESKF baro update supersedes it. Host tests retained.)
- IVP-42a-d: ESKF propagation + replay harness — DONE
- IVP-43: Baro measurement update — DONE (b59b341)
- IVP-44: Mag heading update — DONE (261ab98). WMM declination wired in (uses GPS position when available).
- IVP-44b: ZUPT (zero-velocity) — DONE (261ab98, merged with IVP-44)
- IVP-45: ZUPT + stationarity — subsumed by IVP-44b
- IVP-46: GPS position/velocity update — DONE (6489266). 9-step incremental plan. Step 9 outdoor validated: Fix=3 Sats=12, GPS feeding ESKF. Interrupt-driven UART ring buffer fixed FIFO overflow.
- IVP-47: Attitude initialization refinement — pending
- IVP-48: ESKF health + diagnostics — pending

## Blockers

None currently.

## Future Features (Tracked)

- **MATLAB .mat v5 export for flight logs** — Target research/educational users who standardize on MATLAB. Compatible with GNU Octave. Post-flight analysis workflow: `export matlab <flight_id>` CLI command. Architecture already in SAD.md (Section 10, logging format enum). Implementation deferred until flight logging is active.
- **GPS-free 3D flight path reconstruction (RC GCS)** — Forward-backward RTS smoother using IMU+baro+mag logs with known boundary conditions (zero velocity at launch/landing, launch site position). Enables full 3D trajectory for Core tier without GPS. Deferred to GCS development.

## Reference

- `docs/IVP.md` — Full 68-step integration plan with verification gates (includes Phase M mag cal)
- `docs/SAD.md` — Software Architecture Document
- `docs/SCAFFOLDING.md` — Directory structure and file listing
- `standards/CODING_STANDARDS.md` — Platform constraints
- `.claude/LESSONS_LEARNED.md` — Debugging journal
