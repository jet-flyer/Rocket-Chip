# RocketChip Project Status

**Last Updated:** 2026-04-12 (Stage 16A pre-bench complete)

## Current Phase

**Stages 1-14 COMPLETE** — Notification Engine + Display Decoupling

8-phase migration: main.cpp 3384→706 lines (79% reduction). All subsystems extracted into own modules/AOs with explicit interfaces. Council-reviewed (5 personas). All phases HW verified on Feather RP2350.

**RC_OS consolidation complete:** Callback indirection eliminated (15→5), CLI files renamed to rc_os_* convention, calibration wizards converted to non-blocking async state machine in AO_RCOS (20Hz tick-driven). Idle bridge reduced to watchdog + ESKF + WFI.

**Done this stage:**
- sensor_seqlock.h, led_patterns.h (shared headers)
- sensor_core1.cpp (Core 1 sensor loop module)
- eskf_runner.cpp (ESKF fusion module, stays in idle bridge)
- AO_FlightDirector completed (owns FD HSM, guard eval, Go/No-Go)
- AO_Logger completed (owns ring buffer, flight table, FusedState)
- AO_LedEngine priority compositor (6 layers, Core 1 vitality check)
- health_monitor.cpp (centralized health state, called from FD at 10Hz)
- cli_commands.cpp (display/command handlers extracted from main.cpp)
- cal_hooks.cpp (calibration cross-core protocol)
- docs/AO_ARCHITECTURE.md (inventory, event flow, signal catalog)

**Post-Stage 13 side items (2026-04-06):**
- AO signal audit: wired SIG_PHASE_CHANGE, SIG_RADIO_STATUS, SIG_PYRO_FIRED. Removed 3 dead signals.
- Flash layout portability (flash_layout.h, PICO_FLASH_SIZE_BYTES)
- Flight log metadata header (64B FlightLogHeader)
- Passive ejection mission profile (profiles/passive.cfg)
- Code comments audit: 76 files, 213 lines net reduction, 6 buried action items surfaced
- PIO timer values wired from MissionProfile, Mahony ARM termination, radio ownership to AO_Radio

**WMM2025 update (2026-04-07):**
- Replaced expired IGRF13 table with WMM2025 (valid 2025-2030)
- Three-component tables: declination + inclination + intensity
- Default location in Mission Profile for no-GPS mag heading
- Stage 3D (3-axis mag model) defined in IVP.md

**Stage 3D: 3-Axis Magnetometer (2026-04-07):**
- IVP-99 through IVP-102 complete. IVP-103 (station GPS push) deferred.
- update_mag_3axis(): 6 sequential scalar updates, magnitude gating
- Auto-enable on mag cal + WMM field (GPS/stored/default position)
- WMM position persisted in cal storage v4
- 610/610 host tests. HW verified.

**Stage 7 Take 2 (2026-04-07):**
- IVP-62a–d, 64, 65 complete. Station→vehicle ARM over LoRa verified.
- QGC direct USB: connected, HSI responsive, recovers from stutters.
  Remaining instability needs USB protocol analysis (Stage 13 polish).
- Root cause found: QGC MAVLink bytes triggered CLI commands (flash erase crash).
  Fixed with 0xFD lockout + CDC buffer 1024B + CRLF disable + 10ms timeout.

**Stage 14 (2026-04-10):**
- IVP-113–118 complete. AO_Notify intent layer with 5 per-category typed enums, priority resolver, direct-call output backends (no vtable per JSF AV Rule 170).
- All 5 direct LED callers (FD, CLI, Radio, Health, LedEngine seqlock) rewired through the intent API. LedEngine slimmed to 3-layer pure display driver.
- Core 1 vitality check moved to AO_HealthMonitor primary + LedEngine fallback (Council A1 defense-in-depth). Fixes Stage 13 gap.
- Latent QP use-after-free bug fixed (LL Entry 35) — `AO_LedEngine_post_pattern()` had stack-local events passed to `QACTIVE_POST` since Stage 7, worked by luck until IVP-117 stack usage exposed it.
- 669/669 host tests. Council-reviewed. HW verified via GDB memory inspection. SPIN 11/11 unchanged. NOTIFY_CONTRACT.md with IVP-118 verification amendment.

**Next:** Stage 15 (Pre-Flight Polish: AO audit, audio output, user guide, RBM refresh), Stage 16 (Field Tuning)

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
| 6: Data Logging | IVP-49–54b | 2026-03-04 | FusedState/TelemetryState data model, PCM frames, CRC-16/CRC-32, PSRAM ring buffer (8MB, 50Hz), box-car decimation (200→50Hz), flash flight table (dual-sector), flash flush engine, CLI flight list/download/erase, Python decoder script. End-to-end: 3733 frames, 0 corrupt, CRC-32 OK |
| 7: Radio Driver | IVP-57 | 2026-02-25 | RFM95W LoRa driver (SPI bus, GPIO-controlled CS, TX/RX polling). Ground station RX bridge on Fruit Jam (#6200) + RFM95W breakout (#3072) via SPI1. All 8 verification gates passed: init, absent HW, TX, RX, RSSI, loopback (100%, 0 gaps), range (through walls at 5 dBm), integration soak (120K IMU reads, 0 errors at 10Hz TX) |
| J: Fruit Jam HAL | J.1-J.3 | 2026-03-07 | Compile-time board abstraction (BSP). board.h selector, board_feather_rp2350.h, board_fruit_jam.h. All drivers use board:: constants. 15/15 parity gate items passed on Fruit Jam HW. Core 1 absent-sensor guards added |
| 7: Telemetry Pipeline | IVP-57–65 | 2026-04-08 | CCSDS encoder, telemetry service, station RX+decode, Mission Profile infra, MAVLink v2 encoder (c_library_v2). QGC connected via FJ bridge + direct USB. IVP-62a–d complete (MAVLink RX, station→vehicle ARM, QGC USB fix). IVP-64/65 complete. IVP-63 deferred (Titan) |

| 8: Flight Director | IVP-66–75 | 2026-03-26 | QEP HSM (9 states, descent superstate), guard functions + combinators + three-layer safety, Go/No-Go pre-arm, action executor (NeoPixel + pyro intent), bench flight sim (9/9 PASS), mission profile .cfg + generator, QF+QV compile gate. Council-reviewed (IVP-71, IVP-73, IVP-74). 552 host tests |
| Standards Audit | — | 2026-03-26 | Tiered audit (clang-tidy + lizard + RP2350 guards + Prior Art). ~60 magic numbers remediated, 8 Prior Art blocks added, ring_buffer init fixed. See `standards/STANDARDS_AUDIT_2026-03-26.md` |
| 9: Active Objects | IVP-76–82b | 2026-03-27 | QF+QV BSP, 6 AOs, superloop removal, SPIN formal verification. Council-reviewed (3 reviews, 18 amendments total). Queue depth 32 for LoRa blocking (LL Entry 32). 8 SPIN properties verified (107K states, 0 errors). 552/552 host tests, bench sim 9/9, 10-min soak clean. Verification overview doc added |
| 10: Adaptive Estimation | IVP-83–85 | 2026-03-29 | Phase-scheduled Q/R (Mission Profile `.cfg`), sliding-window NIS innovation monitor, confidence gate (500ms loss / 2s recovery hysteresis), confidence-gated pyro lockout. SPIN model updated. IVP-86 retired. Council-reviewed (7 amendments). 598/598 host tests, 65s HW soak clean |
| 11: PIO Safety Architecture | IVP-87–91 | 2026-03-29 | Baseline benchmark, PIO2 heartbeat watchdog (IRQ-based), PIO backup deployment timers (drogue + main, HW verified), SDK watchdog removed, post-change benchmark (no regression). SPIN updated (6/6 safety). Persistent pyro-fired flags. 2 council reviews. LL 33 + LL 34 |
| 12A: Radio Module + FJ GCS | IVP-92–98 | 2026-03-31 | Non-blocking TX, AO_Radio + AO_Telemetry split, RadioScheduler, 3-Job system (Vehicle/Station/Relay), RadioConfig in .cfg, RSSI bar (5 NeoPixels), relay forwarding. RP2350B I2C pad isolation fix. 3 council reviews. HW verified: TX→RX 296+ pkts 0 CRC, relay confirmed, GPS on FJ |

| 13: Health Monitor | IVP-104–112 | 2026-04-09 | AO_HealthMonitor (standalone), 2-bit encoding (absent/fault/degraded/healthy), sliding window degraded, fault latch (pre-launch + mid-flight), auto-DISARM on critical fault, LED fault patterns (6), preflight Go/No-Go CLI, debug sub-menu. SPIN 11/11. 647 host tests. v0.3.0 |
| 14: Notification Engine | IVP-113–118 | 2026-04-10 | AO_Notify intent layer (per-category typed enums, priority resolver, direct-call backends — no vtable). Rewires 5 direct LED callers through intent API. LedEngine → pure 3-layer display driver. Core 1 vitality moved to HealthMonitor primary + LedEngine fallback (A1). Latent QP stack-local event bug fixed (LL Entry 35). Council-reviewed. 669 host tests. SPIN 11/11 unchanged. Audio backend stubbed for future I2S DAC. NOTIFY_CONTRACT.md with IVP-118 amendment |
| P7: MAIN_DESCENT Liveness | IVP-119–121 | 2026-04-12 | Multi-channel landing detection (ESKF + raw baro + conjunction + backstop with beacon). DT-Promela bounded counters for all flight phases. All 8 SPIN properties pass (0 errors). 699 host tests. Bench sim 2/2 PASS. LL Entry 36 (bench sim rot). Pre-commit hook gates ctest + needs-based HW bench sim |
| 15: Radio + Station | IVP-122–124 | 2026-04-12 | Half-duplex ACK (CCSDS APID 0x003), ARM confirm UX, X-DISARM, distance-to-rocket (haversine+bearing), station help refresh. Command delivery partial — RadioScheduler sync IVP needed. 709 host tests |
| 16A: Docs (pre-bench) | IVP-127–130 | 2026-04-12 | AO audit (removed dead Notify Core1 fields, 2-layer model). TBD purge (memory/power from build + datasheets). RBM complete rewrite for QV. Graphviz diagrams regenerated. User Guide quick-reference card. IVP-125/126 deferred to post-bench. 709 host tests |

## In Progress

*None currently.*

**Next:** Stage 16B (Bench Testing: 30-min soak, PIO shakedown) → Stage 16A' (IVP-125 + IVP-126: SAD/SCAFFOLDING superloop purge) → Stage 16C (Field Testing) → Stage 17 (Field Tuning)

## Blockers

None currently.

## Future Features (Tracked)

- **MATLAB .mat v5 export for flight logs** — Target research/educational users who standardize on MATLAB. Compatible with GNU Octave. Post-flight analysis workflow: convert downloaded PCM log to MATLAB via Python script.
- **GPS-free 3D flight path reconstruction (RC GCS)** — Forward-backward RTS smoother using IMU+baro+mag logs with known boundary conditions (zero velocity at launch/landing, launch site position). Enables full 3D trajectory for Core tier without GPS.
- **Mission Profile OTA/runtime loading** — GCS-pushed profile updates with proper lockouts. Compile-time `.cfg` → Python generator → C++ header is the intentional safety path. Runtime loading requires binary serializer, CRC32, fallback profile, and validated GCS upload infrastructure.
- **F' Evaluation (Titan tier)** — Three paths: STM32H7+F'/Zephyr, Pi Zero 2W+F'/Linux, Hybrid. Research in `docs/decisions/TITAN_BOARD_ANALYSIS.md`. Deferred until Titan development begins.
- **u-blox GPS (Matek M8Q-5883)** — UART + QMC5883L compass. UBX binary protocol. Production/flight hardware upgrade.
- **Dynamic Peripheral Detection + OTA Drivers** — Runtime hot-plug, driver registry, OTA firmware downloads. Crowdfunding stretch goal. Boot-time probe-first already implemented.
- **FSK Continuous Bitstream (IVP-63)** — SX1276 FSK mode for IRIG-heritage PCM telemetry. Titan tier only. Requires separate driver from LoRa.

## Reference

- `docs/IVP.md` — Full integration plan with verification gates (includes Phase M mag cal)
- `docs/SAD.md` — Software Architecture Document
- `docs/SCAFFOLDING.md` — Directory structure and file listing
- `standards/CODING_STANDARDS.md` — Platform constraints
- `.claude/LESSONS_LEARNED.md` — Debugging journal
