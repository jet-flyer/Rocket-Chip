# RocketChip Project Status

**Last Updated:** 2026-04-22 (Stage T complete — RF link-manager AO + pre-arm link-health + aggressive retry)

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

**Stage 16B bench validation COMPLETE (2026-04-17):** Vehicle + station both passed 30-min flight-binary integration soaks on hardware with council-required T=0 preconditions (RegVersion readback, identity strings, SPI error counter, per-AO ring-index activity proxy). All bench IVPs closed except IVP-132a.5 which produced a characterization result (not gate) confirming the RadioScheduler-sync issue, deferred to Stage 16C.

**Stage 16C COMPLETE (2026-04-18):** Station runtime decoupling (IVP-140/141 station_idle_tick for GPS + MCU temp), MCU die-temp pipeline (142a driver, 142b-1 hysteresis FSM, 142b-2 HealthCritical byte, 142b-3 persistence + phase-gated critical_fault), **station HealthMonitor parity** (142c — capability-masking lets station run the same health pipeline as vehicle; `[FAIL]` vs `[N/A]` distinction for uninstalled sensors; evaluate_gps startup-race fix), Tiny 2350+/Pico 2 board scaffolding (143 with `#error` bring-up guards), `cmd_station_*` decoupling audit (144, clean), exit gate (145). 724/724 host tests, bench_sim 2/2 PASS, vehicle + station 5-min soaks PASS.

**Station verification symmetry follow-ups (IVP-146/147, landed same session 2026-04-18):** `scripts/station_bench_sim.py` ships a 3-test regression detector parallel to vehicle `bench_sim.py` (boot/main-menu, Hardware Status [N/A] presentation, Health pipeline populated) plus QP assertion scan; self-test 3/3 PASS. `tools/spin/rocketchip_station.pml` covers the station tracked-command RX/ACK/retry protocol with 2 properties (P_TERMINATION liveness + P_NO_DOUBLE_CLEAR safety); both PASS. Combined SPIN count is now 10/10 (8 vehicle + 2 station). Vehicle SPIN re-verified unchanged.

**Stage L COMPLETE (2026-04-18):** LED/notification polish — AP-parity color swaps (ARMED red, cal gyro/level yellow-blink, pre-arm fail yellow double-flash, boot rainbow), beacon-overlay composition layer with two orthogonal flags (beacon_auto preserves state color +white, beacon_manual forces pure white). Vehicle CLI `b` key + GCS `MAV_CMD_USER_1` both publish `SIG_BEACON_MANUAL`. Pre-arm-fail pure-helper auto-clear (`include/rocketchip/prearm_fail_ticks.h`, host-testable). Boot-init rainbow with min-visibility gate (essential on warm resets where ESKF+IMU are ready before Notify ticks). Two new driver modes (MODE_ALTERNATE for beacon overlays, MODE_DOUBLE_FLASH for pre-arm fail). Full-tree JSF-AV rule-1 sweep decomposed 5 latent over-threshold functions that had accumulated silently since they were written (pre-commit hook only gates staged files; `src/dev/**` whitelisted alongside `src/cli/**`). SESSION_CHECKLIST item 17 added to catch future accumulation at milestone close. 755/755 host tests (up from 724), 4 builds clean, SPIN 7/7, bench_sim 2/2+3/3. Station→vehicle end-to-end roundtrip for manual beacon not verified: vehicle-side resolver chain proven via GDB `set var beacon_manual=true`, but live radio test showed vehicle `rx_count=0` due to the pre-existing RadioScheduler TX-window sync issue.

**Stage T COMPLETE (2026-04-22):** RF link manager architecture, renamed from Stage M. Three batches landed:

- **Batch A** (`acd399d`): IVP-T11 SX1276 register hygiene (RegLna, RegModemConfig3) + IVP-T12 manual sweep.
- **Batch B** (2026-04-21/22 chain ending `5adb878`): IVP-T14 `AO_RfManager` — ACQ→Tentative→Track→TrackDegraded state machine, anchored station TX to vehicle RxDone, α-filtered anchor estimate, forced-ACQ on prolonged silence. Pure state-machine helpers in `src/safety/rf_link_health.h` (32 host tests). SPIN model with 5 LTL properties (all pass). IVP-T14a MAV_CMD newest-wins dedupe (ARM safety-class excluded). IVP-T14b retry instrumentation with per-command-class counters. IVP-T14d wrap: aggressive retry 8×250 ms, ARM/DISARM from dashboard (no key overlap), vehicle-lost notify wiring, FD pre-arm aggregator reads AO_RfManager state.
- **Batch C** (`9f0e04a`): IVP-T14c live retry indicator on dashboard CMD row (Try N/9, ACK Nms, FAILED). IVP-T13 (LQ-adaptive retry) deferred to post-CCSDS-rework at `8fdf951` — parametric tuning on top of a command-layer we've already marked as STOP-GAP.

Stage T surfaced and fixed RP2350-E2 silicon erratum boot deadlock via `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` (commits `e5fd105`, `45ab8a7`). Also marked LL Entry 25 SUPERSEDED and corrected 791 host-test rot from long-stale flash-layout constants.

All Stage T code done. Three scope-bench rigor items (ACK-path timing + α PDF, station LDO rail under retry-storm, station RX-window baseline vs Batch A) dropped from plan — council-approved beyond what's practical for our bench setup. N=100 Wilson 95% CI was an instrumentation-bound non-result; formally deferred to re-measure under the eventual CCSDS command-layer rework.

**Next:** Stage 17 (Field Testing — DEFERRED, awaits airframe + launch window), Stage 18 (Field Tuning — awaits flight data). Pending whiteboard high-priority work: (1) watchdog-reboot → safe-mode refactor; (2) deep RP2350 errata sweep + codified watchlist; (3) Stage T 95% first-try re-baseline after CCSDS layer lands. Remaining station/vehicle disparity items (pre-commit classification-awareness, pre-commit role-awareness, hook-portability into tracked scripts/hooks/) tracked in AGENT_WHITEBOARD.md.

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
| 16B: Bench Validation | IVP-124a, 125, 126, 127a, 127b, 129–132, 132a, 134 | 2026-04-17 | SAD/SCAFFOLDING AO-first rewrites. Dev code audit + build-tier split (bench vs flight). Version string single source of truth. GDB fault injection harness (7 hooks, 5 scripts). PIO backup timer shakedown (5/5 scenarios). Sensor replay harness (5/5 F15-6 profiles → kLanded). **Vehicle 30-min flight-binary battery soak GATE PASS** (2.25M IMU reads, 0 errors). Station bench tests: fault hooks, diag_stats, replay harness, **idle + integration soaks GATE PASS** on flight binary (9023 packets / 0 CRC errors / MSP 0 drift). Pre-flight checklist document. Council-required T=0 preconditions block (RegVersion readback, identity strings, SPI error counter). IVP-132a.5 ACK stress characterized RadioScheduler-sync gap (6.7% first-try ACK) — fix deferred to Stage 16C |
| 16C: Station Decoupling + MCU Temp + Board Scaffolding | IVP-139, 140, 141, 142a, 142b-1, 142b-2, 142b-3, 142c, 143, 144, 145 | 2026-04-18 | Station runtime decoupling: station_idle_tick drives GPS poll + MCU temp capture on Core 0 idle bridge (was vehicle-Core-1-only). MCU die-temp driver with package-aware AINSEL (4 for RP2350A, 8 for RP2350B). MCU HealthLevel hysteresis FSM + HealthCritical byte (visibility-only, no dead safe-mode path per council) + persistence + phase-gated critical_fault (5-tick × 100 ms hysteresis to suppress transient-noise false auto-DISARM). **Station HealthMonitor parity** (142c) — capability-masking (kRoleSamplesCore1, kRoleRunsLogger) + `[FAIL]` vs `[N/A]` for uninstalled sensors + evaluate_gps startup-race fix. Tiny 2350+/Pico 2 scaffolding with #error bring-up guards. CMakePresets.json. 724/724 host tests, bench_sim 2/2, vehicle 5-min soak (IMU 1153/s, 0 errors, health READY) + station 5-min soak PASS |

## In Progress

None. Stage 16C complete.

**Next:** Stage 17 (Field Testing) awaits airframe + launch window. Station verification IVPs (station_bench_sim.py + station SPIN model) queued in `AGENT_WHITEBOARD.md`. Per-session/vehicle-disparity tracker also in whiteboard.

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
