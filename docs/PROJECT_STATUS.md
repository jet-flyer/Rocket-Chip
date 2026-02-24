# RocketChip Project Status

**Last Updated:** 2026-02-24 (Bierman measurement update adoption)

## Current Phase

**Stage 5 IN PROGRESS** — Sensor Fusion (ESKF)

IVP-39 through IVP-48 complete (all measurement updates, health tuning, Mahony AHRS). IVP-47 codegen FPFT complete — 9.1× speedup (538µs → 59µs avg). SymPy CSE generates flat scalar C++ running from SRAM (.time_critical section). 194/194 host tests pass. HW benchmark verified. Stage 5 ESKF core complete.

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
| 5: Mahony AHRS | IVP-45 | 2026-02-19 | Independent attitude estimator integrated in main.cpp. Running at 200Hz alongside ESKF. Mdiv cross-check in CLI `s`/`e` output. 187/187 host tests pass. HW verified |
| ESKF divergence fix | — | 2026-02-19 | Silent ICM-20948 zero-output detection: accel magnitude guard in core1_read_imu() + velocity sentinel in healthy(). 60s soak: max bNIS=3.81, max vel=0.077 m/s, 0 errors |
| GPS 10Hz / 57600 baud | — | 2026-02-20 | GPS negotiated to 57600 baud + 10Hz in gps_uart_init(). Cold-start safe (always inits at 9600 then negotiates). HW verified: ~127 GPS reads/10s, rxOvf=0 |
| 5: Health Tuning | IVP-48 | 2026-02-20 | Fixed mNIS=124.99 death spiral: tilt R inflation (30-60°), hard reject >60°, 300σ gate (ArduPilot match), public reset_mag_heading() API. Per-sensor diagnostic counters + CLI health dashboard. 192/192 host tests. HW verified: mNIS 0.00–0.52 |
| 5: FPFT experiment | IVP-47 | 2026-02-21 | Block-sparse tried first (31% slower, reverted). Codegen FPFT via SymPy CSE: 199 intermediates, SRAM execution. 9.1× speedup: 538µs → 59µs avg. 194/194 host tests. Binary +21KB text, +10KB .data |
| Dense+SRAM benchmark | — | 2026-02-23 | Dense O(N³) at 24 states NOT VIABLE from SRAM: 1,747µs avg vs codegen 111µs (15.7×). Codegen mandatory. All other hot-path functions <640B, no further SRAM placements needed |
| UD factorization benchmark | — | 2026-02-24 | Phase 1 gate PASS: P stable at 100K steps, UD not needed. DCP f64 7.8× slower than f32. Thornton f32 29.6× slower than codegen. Bierman 2× faster than Joseph. Fixed D-array corruption + NaN bug in Thornton |
| Bierman measurement update | — | 2026-02-24 | Replaced Joseph with Bierman behind `ESKF_USE_BIERMAN=1`. PRepr state machine (lazy factorize/reconstruct). 43% faster per epoch (486µs vs 851µs). Alpha canary: relErr=1.37e-08, DCP Phase 2 deferred. 207/207 host tests. HW soak: 88K reads, 0 errors, ESKF HEALTHY, predict 561µs avg |

## In Progress

**Stage 5: Sensor Fusion (ESKF)** — COMPLETE (IVP-39 through IVP-48).

**Next: Stage 6 (Flight Director)** — IVP-49 through IVP-53. Watchdog recovery policy, state machine core, event engine, action executor, mission configuration. Prerequisite: SAD Open Question #4 (Mealy vs Moore state machine) must be resolved before IVP-50.

**Then: Stage 7 (Adaptive Estimation & Safety)** — IVP-54 through IVP-57. Phase-scheduled Q/R (replaces MMAE — see `docs/decisions/ESKF/ESKF_RESEARCH_SUMMARY.md`), confidence gate, confidence-gated actions, vehicle parameter profiles.

## Blockers

None currently.

## Future Features (Tracked)

- **MATLAB .mat v5 export for flight logs** — Target research/educational users who standardize on MATLAB. Compatible with GNU Octave. Post-flight analysis workflow: `export matlab <flight_id>` CLI command. Architecture already in SAD.md (Section 10, logging format enum). Implementation deferred until flight logging is active.
- **GPS-free 3D flight path reconstruction (RC GCS)** — Forward-backward RTS smoother using IMU+baro+mag logs with known boundary conditions (zero velocity at launch/landing, launch site position). Enables full 3D trajectory for Core tier without GPS. Deferred to GCS development.

## Reference

- `docs/IVP.md` — Full 72-step integration plan with verification gates (includes Phase M mag cal)
- `docs/SAD.md` — Software Architecture Document
- `docs/SCAFFOLDING.md` — Directory structure and file listing
- `standards/CODING_STANDARDS.md` — Platform constraints
- `.claude/LESSONS_LEARNED.md` — Debugging journal
