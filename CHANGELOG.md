# Changelog

## Format
`### YYYY-MM-DD-NNN | Author | tags`

Files affected in parentheses if relevant.

**Frequency:** Typically one entry per session, not per individual change. Log when a task is completed or work transitions to a new focus. However, if multiple *significant* changes occur in one session (e.g., refactoring logging system AND redesigning state engine), create separate entries for each.

**Conciseness is the default.** Most entries should be 1-3 sentences. The entry itself should state *what* changed. If additional context is needed, a brief parenthetical or second sentence suffices.

**Rationale sections are rare.** Only add an italicized rationale block when:
- An unconventional approach was chosen (e.g., experimental driver, workaround for a known issue)
- A decision would appear wrong without context (e.g., why we avoided the "obvious" solution)
- Architectural trade-offs need to be preserved for future contributors

Routine work—even if complex—does not warrant rationale. Bugfixes, documentation updates, configuration changes, and hardware corrections rarely need explanation. When in doubt, omit the rationale.

**Tags:** bugfix, feature, architecture, tooling, hardware, council, documentation, refactor

---

### 2026-02-25-001 | Claude Code CLI | feature, hardware

**Ground station LoRa RX bridge — Fruit Jam + RFM95W breakout, end-to-end link verified.** Standalone Pico SDK build for Adafruit Fruit Jam (#6200, RP2350B) with RFM95W breakout (#3072) wired via jumper cables on SPI1. Receives LoRa packets and prints to USB serial with RSSI/SNR per packet and 30-second link quality summaries (min/avg/max RSSI+SNR, packet count, CRC errors). 5 onboard NeoPixels show RSSI bar graph (red→green gradient, 0-5 bars based on signal strength, 2-second timeout to off). Button 3 (GPIO5) toggles NeoPixels on/off. `gs_spi.cpp` provides the same `spi_bus_*()` API as the flight `spi_bus.cpp` but targets SPI1 (GPIO 28/30/31), allowing `rfm95w.cpp` to compile unchanged for both builds. Temporary 1 Hz test TX heartbeat added to flight firmware for link verification. TX power reduced to +5 dBm for bench testing. HW verified: RSSI -54 to -58 dBm, SNR 8-10 dB, <1% CRC errors at desk range. RP2350B PIO gpiobase=16 required for GPIO32 NeoPixels (SDK issue #2030).

(`ground_station/radio_rx.cpp`, `ground_station/gs_spi.cpp`, `ground_station/CMakeLists.txt`, `src/main.cpp`, `src/drivers/rfm95w.cpp`, `ground_station/lora_rx_simple/lora_rx_simple.ino`)

---

### 2026-02-24-004 | Claude Code CLI | hardware, documentation

**Hardware inventory update from Adafruit order.** Added new components to HARDWARE.md: ADXL375 high-G accelerometer and LoRa FeatherWings (×2) moved to on-hand inventory, Ground Station section expanded with Fruit Jam (#6200), HyperPixel 4.0" display, Cyberdeck HAT/Bonnet, LoRa Radio Bonnet with OLED, and Pi Zero 2W. Teensy 3.x Feather Adapter and Pico-to-Pi HAT X Converter added to accessories. ADXL375 datasheet (Analog Devices Rev. B) downloaded to `docs/hardware/datasheets/` and added to VENDOR_GUIDELINES.md inventory. SRAM audit closure, SCAFFOLDING.md fix, and 10 datasheets from prior session folded in per whiteboard note.

---

### 2026-02-24-003 | Claude Code CLI | feature, architecture

**Bierman measurement update adoption + SRAM DCP benchmark.** Replaced Joseph scalar measurement updates with Bierman on UD-factored covariance (43% faster per epoch: 486µs vs 851µs). Compile-time switch via `ESKF_USE_BIERMAN=1` (target only; host tests keep Joseph path unchanged). P representation state machine (`PRepr` enum + `ensure_dense()`/`ensure_ud()`) handles lazy factorize/reconstruct around codegen predict. SRAM DCP micro-benchmark confirmed DCP overhead is intrinsic register shuffling (~58 cyc/op from SRAM vs ~63 from flash), not XIP cache. Alpha precision canary: f32 relErr=1.37e-08 vs f64 reference — DCP Phase 2 deferred (not blocking). 207/207 host tests (199 original + 8 new Bierman). Target build clean.

**Net change summary:**

| Metric | Before | After | Delta |
|--------|--------|-------|-------|
| Measurement epoch | 851 µs | 486 µs | **-43%** (365 µs saved) |
| Scalar update (per) | 81 µs (Joseph) | 43 µs (Bierman) | **-47%** |
| CPU headroom @ 200Hz | 3,149 µs free | 3,514 µs free | **+365 µs WFI/cycle** |
| text (flash) | 137,732 B | 139,100 B | +1,368 B (+1.0%) |
| BSS (SRAM) | 88,268 B | 90,960 B | +2,692 B (+3.0%) |
| Host tests | 199 | 207 | +8 (Bierman suite) |
| f32 alpha precision | — | relErr 1.37e-08 | No f64 DCP needed |
| P stability | Symmetric, positive | Symmetric, positive | No regression (1000-cycle stress test) |
| Joseph form | Active | Retained behind `#ifdef` | Zero-risk A/B switch |

(`CMakeLists.txt`, `src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/fusion/ud_factor.cpp`, `src/benchmark/ud_benchmark.cpp`, `test/test_eskf_bierman.cpp`, `test/CMakeLists.txt`)

---

### 2026-02-24-002 | Claude Code CLI | documentation

**MMAE pivot documentation + IVP restructuring.** Updated project documentation to reflect ESKF research findings: MMAE/IMM replaced by phase-scheduled Q/R (IVP-54, Stage 7). Stage 5 marked complete (IVP-39–48). New Stage 6: Flight Director (IVP-49–53). New Stage 7: Adaptive Estimation & Safety (IVP-54–57). Downstream stages renumbered 8–10 (IVP-58–72, total 72). Audited and fixed stale IVP references across 8 docs + 3 source files (comment-only: IVP-52→IVP-50 in eskf.h, main.cpp, mahony_ahrs.h). Added superseded banner to PHASE5_ESKF_PLAN.md.

(`docs/IVP.md`, `docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md`, `docs/SAD.md`, `docs/SCAFFOLDING.md`, `docs/PHASE5_ESKF_PLAN.md`, `docs/ESKF_TESTING_GUIDE.md`, `docs/flight_director/FLIGHT_DIRECTOR_DESIGN.md`, `docs/DYNAMIC_VALIDATION.md`, `src/main.cpp`, `src/fusion/eskf.h`, `src/fusion/mahony_ahrs.h`)

---

### 2026-02-24-001 | Claude Code CLI | architecture

**UD factorization + DCP float64 benchmark.** Implemented and ran 5-test benchmark suite comparing UD (Thornton WMGS + Bierman) vs current codegen FPFT + Joseph architecture. Phase 1 gate **PASS**: P is rock-stable at 100K steps with codegen — no negative diagonals, zero asymmetry, condition number bounded. UD not needed for numerical stability. DCP float64 is 7.8× slower than f32 FPU per MAC. Thornton f32 predict is 29.6× slower than codegen (1,420µs vs 48µs). Bierman scalar update is 2× faster than Joseph (43µs vs 81µs). Fixed Thornton D-array in-place corruption bug (algorithm requires old D values during WMGS sweep — added snapshot). Fixed NaN detection in `ud_all_positive()`.

(`src/fusion/ud_factor.h`, `src/fusion/ud_factor.cpp`, `src/benchmark/ud_benchmark.cpp`, `docs/benchmarks/UD_BENCHMARK_RESULTS.md`, `CMakeLists.txt`, `src/fusion/eskf.h`)

---

### 2026-02-23-002 | Claude Code CLI | documentation

**Dynamic validation methods document.** Created `docs/DYNAMIC_VALIDATION.md` — six repeatable physical test methods for verifying ESKF accuracy beyond host-side unit tests and stationary soaks: Allan variance (Q tuning validation), turntable rotation test, pendulum test, elevator test (baro fusion), data logging + replay infrastructure, and vehicle GPS-vs-INS comparison. Includes truth references, pass/fail criteria, and Python Allan variance implementation.

(`docs/DYNAMIC_VALIDATION.md`)

---

### 2026-02-23-001 | Claude Code CLI | architecture

**Dense FPFT + SRAM feasibility benchmark.** Tested dense O(N³) F*P*F^T at 24 states with SRAM placement (`.time_critical` section) to evaluate eliminating codegen maintenance. Result: **NOT VIABLE** — 1,747µs avg vs codegen 111µs (15.7× slower, 34.9% CPU at 200Hz). SRAM eliminated XIP cache thrashing (device runs, tight min/max) but O(24³)=13,824 MACs is the fundamental bottleneck. Codegen stays. SRAM audit of remaining hot-path functions found all <640 bytes — no further `.time_critical` placements needed. No code changes on main.

---

### 2026-02-22-002 | Claude Code CLI | documentation, tooling

**Prior art audit & license compliance.** Rewrote `LICENSE` (removed stale Adafruit/Arduino/LittleFS refs from FreeRTOS era). Created `THIRD_PARTY_LICENSES.md` with full attribution for all actual dependencies (Pico SDK BSD-3, ruuvi MIT, lwGPS MIT, ICM-20948 MIT, GoogleTest BSD-3, WMM table from ArduPilot GPL-3). Added SPDX headers (`GPL-3.0-or-later`) to all 54 project-owned source files. Removed unused `ws2812b-animation` submodule. Added explicit GPL-3.0 attribution to WMM declination table.

(`LICENSE`, `THIRD_PARTY_LICENSES.md`, `.gitmodules`, 54 source/test files)

---

### 2026-02-22-001 | Claude Code CLI | refactor

**Post-Stage 5 code audit cleanup.** Updated stale build tag (`ivp45-4` → `stage5-complete`), fixed outdated "not yet implemented" comment for 6-pos accel cal in `calibration_manager.h`, clarified baro_kf rejection comment. Deleted 6 untracked `.uf2` files (old debug probe firmware + test binary) from repo root. Full codebase audit found no debugging artifacts, commented-out code, or dead code — codebase is clean.

---

### 2026-02-21-003 | Claude Code CLI | feature, architecture

**24-State ESKF Expansion with Runtime Inhibit Flags**

Expanded error-state vector from 15 to 24 states: earth_mag NED (3), body_mag_bias (3), wind_NE (2), baro_bias (1). ArduPilot EKF3-pattern runtime inhibit flags (`inhibit_mag_states_`, `inhibit_wind_states_`, `inhibit_baro_bias_`) — all inhibited by default, P zeroed for inhibited blocks. When inhibited, codegen propagates all 24 states (identity F for new states, near-zero cost) but `clamp_covariance()` zeros inhibited P blocks. `set_inhibit_*()` methods handle enable/disable with proper P initialization and cross-covariance zeroing.

Codegen regenerated for 24 states via `generate_fpft.py` (N=24). Benchmark: **111µs avg, 101µs min, 156µs max** (was 59µs at 15-state). Critical runtime fix: O(N²) rank-1 Joseph form replaced O(N³) dense triple product in `scalar_kalman_update()` — device was unresponsive with dense 24×24 matrix multiplies at measurement update rates. Sparse `reset()` exploits G=I structure except 3×3 attitude block (~450 MACs vs ~27,648 dense). `healthy()` made inhibit-aware (skips P diagonal check for inhibited indices).

Baro update subtracts `baro_bias_` when enabled. Mag states (15-20) are **unobservable** with current yaw-only H — require 3-axis mag model (Titan tier). 5 new inhibit tests, Mat15→Mat24 across all test files. All 5 replay reference CSVs regenerated. CLI shows `inhib: mag=Y wind=Y bbias=Y` + conditional extended state display when states enabled. 199/199 host tests pass. HW verified: 0 sensor errors.

(`src/fusion/eskf.cpp`, `src/fusion/eskf.h`, `src/fusion/eskf_state.h`, `src/fusion/eskf_codegen.cpp`, `src/fusion/eskf_codegen.h`, `scripts/generate_fpft.py`, `src/math/mat.h`, `src/main.cpp`, `test/test_eskf_propagation.cpp`, `test/test_eskf_update.cpp`, `test/test_eskf_mag_update.cpp`, `test/test_eskf_zupt.cpp`, `test/test_mat.cpp`, `test/data/reference/*.csv`)

---

### 2026-02-21-002 | Claude Code CLI | feature, architecture

**IVP-47: Codegen FPFT — 9.1× speedup, predict() at 59µs (was 538µs)**

SymPy codegen script (`scripts/generate_fpft.py`) generates flat scalar C++ for F*P*F^T + Q_d covariance propagation. 199 CSE intermediates, 120 upper-triangle outputs, Q_d constants baked in. Read-after-write hazard fixed by snapshotting all 120 P inputs to locals before computation. Running from SRAM (`.time_critical` section) — XIP cache (2KB) thrashes on 10KB function from flash (398µs), SRAM eliminates cache entirely (59µs). Three-layer verification: SymPy self-test (2.6e-18 vs NumPy), Test 8 CodegenVsDenseFPFT (100 steps, 1e-4), Test 15 CodegenSingleStep (1e-6). `static_assert` guards sync codegen constants with eskf.h. 194/194 host tests pass. Binary: 239,616 bytes (+21KB codegen, +10KB .data from SRAM placement). Benchmark: 59µs avg, 50µs min, 113µs max (17,883 calls), 0 sensor errors. Standards deviation CG-1 logged for auto-generated function exceeding 200 L-SLOC.

(`scripts/generate_fpft.py`, `src/fusion/eskf_codegen.cpp`, `src/fusion/eskf_codegen.h`, `src/fusion/eskf.cpp`, `CMakeLists.txt`, `test/test_eskf_propagation.cpp`, `test/data/reference/*.csv`, `standards/STANDARDS_DEVIATIONS.md`)

---

### 2026-02-21-001 | Claude Code CLI | feature, architecture

**IVP-47: Block-sparse FPFT experiment — reverted to dense after benchmark**

Implemented and verified block-sparse FPFT covariance propagation exploiting F_x block structure (15 of 25 blocks are zero). Algebraically correct: 193/193 host tests pass, 1e-6 single-step tolerance vs dense reference. However, on-target benchmark showed block-sparse was **31% slower** than dense (712µs avg vs 542µs avg). Root cause: ~30+ `block3()` Mat3 copies per step, poor cache locality jumping around 15×15 P matrix, and dense inner loops autovectorize better on Cortex-M33. Theoretical 10× FMA reduction doesn't offset memory access overhead.

**Outcome:** Reverted predict() to dense. Retained `block3()`/`set_block3()`/`add_block3()` helpers in mat.h (useful for measurement updates). Retained PSymmetry test. predict() at 538µs avg confirmed on target (matches pre-experiment baseline). **Codegen (SymPy/SymForce element-wise scalar expansion) is the correct optimization path** — scheduled for 24-state wind estimation expansion, where dense becomes O(24³×3) ≈ 41K FMA vs codegen O(N²).

*Block-sparse and codegen are fundamentally different approaches. ArduPilot/PX4 use codegen (SymPy/SymForce), not block-sparse. Codegen emits flat scalar C++ with CSE — zero function calls, zero copies, zero temporaries at runtime.*

(`src/fusion/eskf.cpp`, `src/math/mat.h`, `test/test_eskf_propagation.cpp`, `test/data/reference/*.csv`)

---

### 2026-02-20-005 | Claude Code CLI | feature

**IVP-47: ESKF health tuning — mag heading fix + diagnostics**

Fixed mNIS=124.99 death spiral (mag heading completely non-functional). Three-part fix: (1) tilt-conditional R inflation at 30-60° with hard reject above 60° (ArduPilot `fuseEulerYaw` pattern), (2) public `reset_mag_heading()` API for state machine use (IVP-52), (3) mag gate widened to 300σ matching ArduPilot EKF3 (interference detection handles bad data, not the gate). Added per-sensor accept/reject counters for baro, mag, GPS pos/vel, ZUPT. Q constants reviewed — all correct per ICM-20948 datasheet. CLI `s` command shows gate counters + P velocity/bias diagonals. CLI `e` command shows mag accept ratio + ZUPT NIS. 192/192 host tests pass. HW verified: 65s soak, mNIS 0.00–0.52 (was 124.99), bNIS 0–4, zero sensor errors.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/main.cpp`, `test/test_eskf_mag_update.cpp`)

---

### 2026-02-20-004 | Claude Code CLI | tooling

**Pre-commit hook: function size + cognitive complexity gate**

Added `.git/hooks/pre-commit` that runs `readability-function-size` and `readability-function-cognitive-complexity` checks on staged `.cpp` files in `src/`. Blocks commit if any function exceeds thresholds (60 lines, 200 statements, nesting 6, CC 25). Uses same clang-tidy path and ARM cross-compilation args as `scripts/run_clang_tidy.sh`. Skippable with `--no-verify`.

---

### 2026-02-20-003 | Claude Code CLI | refactor

**P5c: decompose 24 function-size/CC warnings to zero**

Per plan: `.claude/plans/quirky-squishing-clarke.md`. Four batches executed in council-mandated order (lowest risk first): Batch 3 `rc_os.cpp` (wizard + mag cal extraction), Batch 4 `calibration_manager.cpp` (LM solver dedup via function pointers), Batch 2 `main.cpp` (eskf_tick + sensor print decomposition), Batch 1 `eskf.cpp` (Joseph-form deduplication into `scalar_kalman_update()`). Additional borderline findings caught in final audit: `core1_read_imu`, `core1_read_gps`, `core1_sensor_loop`, `eskf_tick_gps`, `lm_solve` — all decomposed. Binary: text=114,144 (-2,248 from P5b baseline), BSS=96,524 (-11,408, from consolidating 5 sets of static Mat15 into 1). Full clang-tidy audit: 0 function-size/CC findings in production code. HW verified: 60s soak, 86K IMU reads, 0 errors, ESKF stable (qnorm=1.000000, vel<0.03 m/s).

(`src/fusion/eskf.cpp`, `src/fusion/eskf.h`, `src/main.cpp`, `src/calibration/calibration_manager.cpp`, `src/cli/rc_os.cpp`)

---

### 2026-02-20-002 | Claude Code CLI | documentation

**Pre-audit documentation cleanup: 10 files updated for consistency**

Cross-document review and reconciliation ahead of full clang-tidy audit. Major updates: SCAFFOLDING.md complete rewrite to match actual filesystem (was 18 days stale with fictional directories); SAD.md converted from living-document style to high-level architecture reference with current implementation status; VENDOR_GUIDELINES.md updated for ICM-20948 bypass mode (implemented 2026-02-10) and UART GPS transport (preferred since IVP-46); GPS UART driver added to CODING_STANDARDS.md file classification and STANDARDS_DEVIATIONS.md IO-2 deviation. Removed empty `src/debug/` and `src/tasks/` directories and 6 stale temp files.

(`docs/SCAFFOLDING.md`, `docs/SAD.md`, `docs/PROJECT_STATUS.md`, `docs/IVP.md`, `standards/VENDOR_GUIDELINES.md`, `standards/CODING_STANDARDS.md`, `standards/AUDIT_REMEDIATION.md`, `standards/STANDARDS_AUDIT.md`, `standards/STANDARDS_DEVIATIONS.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-20-001 | Claude Code CLI | feature

**GPS upgraded to 57600 baud + 10Hz update rate**

`gps_uart_init()` now negotiates the MT3339 from factory-default 9600 baud to 57600 baud via PMTK251, then sets 10Hz update rate via PMTK220. Cold-start safe: always detects presence at 9600 then negotiates. 9600 baud saturates at ~4.8 NMEA bursts/sec and cannot sustain 10Hz; 57600 gives 2.8× headroom. ESKF GPS measurement updates (`update_gps_position()` / `update_gps_velocity()`) automatically benefit — they fire on each new `gps_read_count` increment with no further code changes needed.

HW verified: ~127 GPS reads/10s, rxOvf=0, IMUerr=0. (`src/drivers/gps_uart.cpp`, `src/drivers/gps_uart.h`)

---

### 2026-02-19-001 | Claude Code CLI | bugfix, architecture

**ESKF velocity divergence: root-cause fix for silent ICM-20948 zero-output fault**

Fixed catastrophic ESKF divergence (`vel=1688 m/s`, `bNIS=23,000,000`) caused by ICM-20948 entering sleep/reset state mid-session. The device ACKs I2C reads in sleep mode and returns all-zero output registers — invisible to the existing consecutive-fail counter which only triggers on NACKs. With raw accel ≈ (0,0,0), the ESKF propagated a constant +9.8 m/s² specific force in NED-Z, accumulating 1688 m/s after ~3 min. ZUPT and ESKF init guards both failed because `|A|=0.285 m/s²` (calibration offset residual) is nowhere near 9.8 m/s².

Three-part fix: (1) `core1_read_imu()` now validates raw accel magnitude against `kAccelMinHealthyMag = 3.0f` m/s² (gravity floor at 72° tilt: 9.8×cos(72°)) after every successful I2C read, routing zero-output reads to the consecutive-fail path so bus-recover and device-reinit fire. (2) `healthy()` in `eskf.cpp` now checks `v.norm() >= kMaxHealthyVelocity (500 m/s)` as a velocity-divergence sentinel — above max hobby rocket burnout, below any slow divergence trajectory. (3) `eskf_tick()` reset-on-unhealthy path already correct (verified, no change needed).

HW verified: 60s indoor soak, max bNIS=3.81, max vel=0.077 m/s, 0 IMU errors. 187/187 host tests pass.

*Root cause established by backward math: stored cal `off=[0.0235, 0.1632, -0.2364]` implies raw_x≈0, raw_y≈0, raw_z≈0 — physically impossible from any real sensor orientation. The calibration was correct; the sensor was in sleep state.*

(`src/main.cpp`, `src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/drivers/gps_uart.cpp`)

---

### 2026-02-18-003 | Claude Code CLI | bugfix, feature

**IVP-46 Step 9 fix: interrupt-driven UART GPS + ESKF init NeoPixel**

Fixed UART FIFO overflow that caused Step 9 outdoor validation failure — GPS GGA/RMC sentences lost due to 32-byte hardware FIFO filling between 10Hz polls. Added interrupt-driven 512-byte SPSC ring buffer (ISR on Core 0, consumer on Core 1). Also added fast red NeoPixel blink during ESKF stationary init as "hold still" cue.

Outdoor verified: Fix=3 Sats=12 HDOP=0.90, GPS feeding ESKF (G=Y). 60s soak: bNIS 0.00–2.13, zero UNHEALTHY, rxOvf=0.

(`src/drivers/gps_uart.cpp`, `src/drivers/gps_uart.h`, `src/main.cpp`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-18-002 | Claude Code CLI | feature, architecture

**Stage 4 GPS: IVP-46 complete (8-step incremental plan)**

ESKF GPS position + velocity measurement updates (IVP-46). 8 individually HW-verified commits:
1. Transport-neutral `gps_data_t` types extracted to `gps.h`
2. UART GPS driver (`gps_uart.cpp`) for FeatherWing on GPIO0/1
3. ESKF GPS methods + **P covariance reset fix** in `set_origin()` (council-identified root cause of previous bNIS explosion) + 18 host tests including PostOriginBaroStability
4. HDOP/VDOP in shared sensor data + CLI display
5. Transport detection (UART-first, I2C fallback) + function pointer dispatch + hold-on-valid pattern
6. Best-fix diagnostic capture with atomic cross-core visibility
7. ESKF GPS wiring in `eskf_tick()` — origin establishment, geodetic-to-NED, position/velocity updates, 10km origin re-centering
8. SENSOR_ARCHITECTURE doc + soak script + CHANGELOG

**Critical fix:** `set_origin()` now resets P[3..8] diagonal to GPS-derived uncertainty and zeros all cross-covariance terms. Without this, stale correlations from pre-origin predict cycles corrupted baro Kalman gain → bNIS explosion.

Host tests: 173/173 pass. 5-minute indoor soak: bNIS max=4.09, 0 UNHEALTHY, ZUPT 100% active.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/fusion/eskf_state.h`, `src/drivers/gps.h`, `src/drivers/gps_uart.h`, `src/drivers/gps_uart.cpp`, `src/drivers/gps_pa1010d.h`, `src/drivers/gps_pa1010d.cpp`, `src/main.cpp`, `test/test_eskf_gps_update.cpp`, `docs/SENSOR_ARCHITECTURE.md`, `scripts/eskf_gps_soak.py`)

---

### 2026-02-18-001 | Claude Code CLI | documentation

**Session handoff: Stage 4 GPS reboot deferred — bNIS regression unresolved**

Attempted Stage 4 GPS reboot (protocol-agnostic/UART-first approach per plan `purrfect-pondering-hopcroft.md`). Commits A/B/C built clean and passed 172/172 host tests, but HW validation revealed ESKF bNIS explosion (~125K→589K after ~12 baro updates, stationary). Root cause not diagnosed — may predate Stage 4. All Stage 4 commits reverted; repo returned to `17c4111`. Next session must diagnose bNIS regression before resuming Stage 4. See AGENT_WHITEBOARD.md for full handoff notes.

(`AGENT_WHITEBOARD.md`)

---

### 2026-02-14-001 | Claude Code CLI | documentation, architecture

**Flight Director & Mission Profile: naming, docs, cross-reference sync**

Established council-decided naming convention: **Flight Director** (runtime engine) and **Mission Profile** (configuration data), replacing the umbrella term "Mission Engine." Created `docs/flight_director/` with comprehensive design spec (`FLIGHT_DIRECTOR_DESIGN.md`, all details PRELIMINARY) and moved research doc there (`RESEARCH.md`, historical — unchanged). Created `docs/mission_profiles/` with stub (`MISSION_PROFILES.md`, to be fleshed out in future session). Updated 32 cross-references across 11 files to use correct terminology. Historical research doc preserved with original naming.

(`docs/flight_director/FLIGHT_DIRECTOR_DESIGN.md`, `docs/flight_director/RESEARCH.md`, `docs/mission_profiles/MISSION_PROFILES.md`, `docs/SAD.md`, `docs/IVP.md`, `docs/SCAFFOLDING.md`, `standards/CODING_STANDARDS.md`, `docs/decisions/SENSOR_FUSION_TIERS.md`, `docs/decisions/TITAN_BOARD_ANALYSIS.md`, `docs/decisions/ESKF/FUSION_ARCHITECTURE_DECISION.md`, `docs/ESKF_TESTING_GUIDE.md`, `docs/PHASE5_ESKF_PLAN.md`, `docs/PROJECT_OVERVIEW.md`, `tools/state_to_dot.py`)

---

### 2026-02-13-004 | Claude Code CLI | architecture, feature

**Modular GPS refactor: transport-neutral types + UART backend**

Refactored GPS subsystem into transport-neutral data types + transport-specific backends, establishing the pattern for migrating all base sensors off I2C (IMU→SPI, baro→SPI in future). Created `gps.h` with shared `gps_data_t`, `gps_fix_t`, `gps_transport_t` (zero transport dependencies). Updated `gps_pa1010d.h/cpp` to use shared types. Created UART GPS backend (`gps_uart.h/cpp`) for Adafruit Ultimate GPS FeatherWing (#3133) on GPIO0/1 (9600 baud NMEA, lwGPS parser, 2-second presence detection timeout). Added function pointer dispatch in main.cpp — auto-detects UART first (production), falls back to I2C (Qwiic dev). Boot banner and CLI status display show transport type. Created `docs/SENSOR_ARCHITECTURE.md` documenting the modular sensor pattern.

Host tests: 172/172 pass. Target build: 0 errors, 0 warnings. Binary: 216,576 bytes UF2. HW verification deferred to UART GPS wiring.

(`src/drivers/gps.h`, `src/drivers/gps_uart.h`, `src/drivers/gps_uart.cpp`, `src/drivers/gps_pa1010d.h`, `src/drivers/gps_pa1010d.cpp`, `src/main.cpp`, `CMakeLists.txt`, `docs/SENSOR_ARCHITECTURE.md`)

---

### 2026-02-13-003 | Claude Code CLI | feature

**IVP-46: GPS position & velocity measurement update for ESKF**

Added GPS position (3 sequential scalar updates) and velocity (2 sequential scalar updates, N/E only) measurement updates to the ESKF. Flat-earth NED frame with double-precision geodetic origin, moving-origin mechanism for HAB-range flights (>10km), HDOP-scaled position noise (σ=3.5m base, MT3333 CEP50), innovation gating (5σ), per-axis NIS tracking.

New files: `test/test_eskf_gps_update.cpp` (17 tests: origin management, geodetic conversion, position/velocity updates, gating, convergence, full pipeline), `scripts/eskf_gps_soak.py` (indoor/outdoor/walk modes). Added HDOP/VDOP to seqlock struct, named state index constants (`kIdxPosN` etc.) to `eskf_state.h`, GPS constants + NED origin state + 5 methods to ESKF. Wired into `eskf_tick()` between mag update and ZUPT. Updated `s` and `e` CLI displays with position, GPS NIS, and origin flag.

Council-approved (3 panelists, unanimous). 8 conditions integrated: C-1 MT3333 noise, C-2 velocity NE-only, C-3 named indices, C-4 HDOP origin gate, C-5 double-precision subtraction, C-6 ZUPT stays IMU-only, C-7 origin reset continuity, C-8 course-to-velocity formula.

Host tests: 172/172 pass (155 existing + 17 new). Target build: 0 errors, 0 warnings. Binary: 213,504 bytes UF2 (+6,144 from GPS methods). Outdoor gate tests deferred to LoRa serial bridge setup.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/fusion/eskf_state.h`, `src/main.cpp`, `test/test_eskf_gps_update.cpp`, `test/CMakeLists.txt`, `scripts/eskf_gps_soak.py`)

---

### 2026-02-13-002 | Claude Code CLI | refactor, documentation

**Pre-IVP-46 cleanup: remove standalone BaroKF from firmware, wire WMM declination, doc sweep**

Removed standalone BaroKF from main firmware — superseded by ESKF `update_baro()` (IVP-43). Removed `fusion_tick()`, BaroKF globals, KF display line from `print_seqlock_sensors`, CMakeLists target build entry. Host tests + mat_benchmark retain baro_kf for unit testing and benchmarking.

Wired WMM declination into mag heading update: uses `wmm_get_declination(lat, lon)` when GPS has valid fix (fix_type >= 2), falls back to 0 (magnetic heading) without GPS. Removed unused `g_loopCounter`. Updated stale GPS comment in `i2c_bus.cpp`.

Living document cleanup: updated PROJECT_STATUS.md (cleared stale baro NIS blocker, fixed IVP-46 as next step, noted standalone BaroKF removal), HARDWARE.md (baro cross-reference now points to eskf.h), ESKF_TESTING_GUIDE.md (removed false "gitignored" claim), AGENT_WHITEBOARD.md (DPS310 datasheet priority adjusted). CODING_STANDARDS.md and SEQLOCK_DESIGN.md cosmetic edits from earlier in session.

Host tests: 155/155 pass. Target build: 0 errors, 0 warnings. Binary: 207,360 bytes UF2.

(`src/main.cpp`, `CMakeLists.txt`, `src/drivers/i2c_bus.cpp`, `docs/PROJECT_STATUS.md`, `docs/hardware/HARDWARE.md`, `docs/ESKF_TESTING_GUIDE.md`, `AGENT_WHITEBOARD.md`, `standards/CODING_STANDARDS.md`, `docs/decisions/SEQLOCK_DESIGN.md`)

---

### 2026-02-13-001 | Claude Code CLI | feature

**IVP-44 + IVP-44b: ESKF magnetometer heading update + zero-velocity (ZUPT)**

Implemented mag heading measurement update (IVP-44): tilt-compensated heading via zero-yaw rotation (ArduPilot fuseEulerYaw approach), wrap_pi innovation, two-tier interference detection (25% inflates R 10x, 50% hard rejects, council), declination_rad parameter for WMM true heading. 16 new host tests. Mag-based initial yaw at ESKF init prevents gate rejection when heading far from 0°.

Added ZUPT pseudo-measurement (IVP-44b): prevents horizontal velocity divergence in GPS-denied operation (previously ~48 m/s in 30s, now <0.03 m/s for 60s+). Stationarity detection from accel/gyro, three sequential scalar velocity updates with Joseph form, kSigmaZupt=0.5 matching ArduPilot EKF3. 13 new host tests.

Target integration: sensor_to_ned INTERIM Z-negate helpers (ICM-20948 Z-up → NED Z-down), mag update at ~10Hz via seqlock, ZUPT at 200Hz, live display shows yaw/mNIS/ZUPT flag. Watchdog sentinel refinement (scratch[0] custom sentinel replaces broken SDK functions). Synthetic data + replay harness updated with mag columns.

Host tests: 135/135 (29 new). Binary: 202,752 bytes UF2. HW verified: 60s stationary, vh<0.03, Z=Y, mNIS<1, yaw drift <1°, 0 IMU errors.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/main.cpp`, `test/test_eskf_mag_update.cpp`, `test/test_eskf_zupt.cpp`, `test/CMakeLists.txt`, `test/scripts/generate_synthetic.py`, `test/replay/replay_harness.cpp`, `test/test_replay_regression.cpp`, `test/data/*.csv`)

---

### 2026-02-12-003 | Claude Code CLI | feature, bugfix

**IVP-43: ESKF barometric altitude measurement update + 6-param accel cal fix**

Implemented ESKF baro measurement update (IVP-43): scalar update with Joseph form P update (inline statics for stack safety), 3σ innovation gating, NIS diagnostic, `isfinite`/S guards per council conditions. Added live ESKF mode ('e' key for 1Hz compact status: alt, velocity, Patt, bNIS). 11 new host tests (106/106 total pass). Fixed 6-pos accel cal: reduced Gauss-Newton solver from 9 to 6 parameters — 6 axis-aligned positions provide only 6 constraints, offdiag sits in Jacobian null space. Board rotation matrix added to status output. HW verified: |A|=9.770, bNIS stable, 0 I2C errors.

(`src/fusion/eskf.cpp`, `src/fusion/eskf.h`, `src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `src/calibration/calibration_manager.cpp`, `test/test_eskf_update.cpp`, `test/CMakeLists.txt`)

---

### 2026-02-12-002 | Claude Code CLI | bugfix

**Fix calibration data flow: 6-pos accel zeros, mag cal zeros, wizard persistence**

Three bugs fixed during HW calibration testing: (1) 6-pos accel cal returned zeros after ~200 reads — `read_accel_for_cal()` used 6-byte read that doesn't clear ICM-20948 data-ready flag, switched to 14-byte `icm20948_read()`. (2) Mag cal fed zero samples — `icm20948_read()` mag divider skipped 9/10 calls without setting `mag_valid=false`, uninitialized stack value bypassed staleness gate, added else clause. (3) Wizard gyro/level results lost after Core 1 resume — wizard stored in RAM but never called `calibration_save()`, Core 1 reload from flash overwrote them, now saves to flash after each step. HW verified: mag cal 300 samples, 90% coverage, RMS 0.927 uT.

(`src/main.cpp`, `src/drivers/icm20948.cpp`, `src/cli/rc_os.cpp`)

---

### 2026-02-12-001 | Claude Code CLI | bugfix, architecture

**Watchdog FMEA analysis, IVP-51 Watchdog Recovery Policy, reboot detection bugfix**

Fixed `watchdog_enable_caused_reboot()` → `watchdog_caused_reboot()` in `init_hardware()`. The `_enable_` variant checks scratch[4] magic that persists across picotool flashes, causing false "PREVIOUS REBOOT WAS CAUSED BY WATCHDOG" warnings on clean boots. The base function uses RP2350-specific `rom_get_last_boot_type()` for correct POR discrimination.

Added IVP-51 (Watchdog Recovery Policy) as first step of Stage 6, prerequisite to state machine. Covers: scratch register persistence for reboot diagnostics, reboot counting with safe-mode lockout, ESKF failure backoff, recovery boot path, ground-side launch abort on any WDT reset. All downstream IVP numbers shifted +1 (Stage 7: IVP-57-61, Stage 8: IVP-62-66, Stage 9: IVP-67-71). Updated Gemini carrier board doc with cross-board watchdog handoff and per-board status LEDs. Updated SAD Section 14.

(`src/main.cpp`, `docs/IVP.md`, `AGENT_WHITEBOARD.md`, `docs/hardware/GEMINI_CARRIER_BOARD.md`, `docs/SAD.md`)

---

### 2026-02-10-008 | Claude Code CLI | council, documentation

**Council decision: Sensor fusion tier architecture (MMAE + sensor affinity)**

Added council-approved architectural decision document for hybrid MMAE + sensor affinity across hardware tiers (Core: single ESKF, Titan: MMAE regime switching, Gemini: full MMAE + affinity with dual-MCU). Unanimous approval with staged implementation. Added MATLAB .mat v5 export to PROJECT_STATUS.md future features roadmap.

(`docs/decisions/SENSOR_FUSION_TIERS.md`, `docs/PROJECT_STATUS.md`)

---

### 2026-02-10-007 | Claude Code CLI | feature, bugfix

**ICM-20948: Migrate from I2C master mode to bypass mode**

Replaced ICM-20948's internal I2C master with bypass mode (`INT_PIN_CFG.BYPASS_EN=1`) for AK09916 magnetometer access. AK09916 now reads directly at 0x0C on the external I2C bus, eliminating the bank-switching race (LL Entry 21), progressive master stall, and unrecoverable disable/enable cycle that caused mag cal failures. Removed ~120 lines of I2C master plumbing (Bank 3 registers, SLV0 config, master clock setup). Added mag read divider (100Hz vs 1kHz), two-level device recovery (bus recover at 10 fails, full device reset at 50), lazy mag re-init after device reset, and GPS pause during mag cal (`rc_os_mag_cal_active` flag). Reordered `init_sensors()` so GPS probe/init happens after IMU bypass mode is established. Council-approved (unanimous). HW verified: mag cal 300 samples/72% coverage/RMS 0.878 uT with GPS on bus, system self-recovers from lockups.

*ArduPilot uses the same approach (`AP_InertialSensor_Invensensev2.cpp`). The I2C master creates an entire class of bugs that bypass mode eliminates entirely.*

(`src/drivers/icm20948.cpp`, `src/drivers/icm20948.h`, `src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`)

---

### 2026-02-10-006 | Claude Code CLI | feature, bugfix

**Phase M.5 complete: Full calibration wizard + mag cal HW verified**

NeoPixel now shows mode color during ENTER wait (before user presses ENTER, not after). All standalone cal commands match wizard UX with ENTER prompt + NeoPixel feedback. Added `cmd_accel_6pos_cal()` initial ENTER prompt (was missing). Added mag cal diagnostics: cancel prints accepted/read/readFail/close/range breakdown, sensor status shows `M=` (mag_read_count), periodic `mag_valid=false` diagnostic prints. Full mag cal HW verified: 300 samples, 81% coverage, ellipsoid fit RMS 2.499 uT, save to flash OK. Build tag: wizard-12.

(`src/main.cpp`, `src/cli/rc_os.cpp`)

---

### 2026-02-10-005 | Claude Code CLI | feature

**Phase M.5: Unified calibration wizard with NeoPixel feedback**

Implemented full 5-step calibration wizard (gyro, level, baro, 6-pos accel, compass) accessible from CLI `c` → `w`. Each step waits for ENTER to start, 'x' to skip/cancel. NeoPixel shows calibration state via cross-core atomic override (blue breathe = IMU sampling, cyan = baro, yellow = accel positioning, rainbow = mag rotation, green = success, red = failure). Moved async calibration sensor feeds from Core 0 to Core 1 to eliminate I2C bus contention — Core 0 no longer touches I2C during gyro/level/baro calibrations. Extracted `mag_cal_inner()` for reuse between standalone command and wizard. Fixed watchdog crash in blocking prompts (polling loop with `watchdog_update()` instead of 30s `getchar_timeout_us()`). Fixed auto-progress bug from stale USB input buffer bytes (double-drain with 100ms gap). Fixed mag cal collecting only 1 sample: seqlock now carries raw mag data (`mag_raw_x/y/z`) alongside calibrated — ellipsoid solver needs uncorrected data. Struct grew 128→140 bytes. HW verified: wizard-6, gyro/level/baro/6-pos accel all pass.

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`)

---

### 2026-02-10-004 | Claude Code CLI | feature

**Phase M complete: Core 1 live mag apply + heading display (IVP-38)**

Applied mag calibration on Core 1 via `calibration_apply_mag_with()`, matching existing accel/gyro pattern. Sensor status (`s`) now shows calibrated magnitude `|M|` and tilt-uncorrected heading (0-360 deg). Both seqlock display and pre-sensor-phase direct-read paths updated. HW verified: 0 IMU errors, `|M|` ~60 µT stable, heading tracks smoothly. Phase M (IVP-34 through IVP-38) is now complete — all 4 commits merged.

(`src/main.cpp`)

---

### 2026-02-10-003 | Claude Code CLI | feature

**Non-blocking USB init — firmware runs without terminal**

Replaced blocking `wait_for_usb_connection()` with non-blocking `stdio_init_all()`. Boot banner and HW status deferred to first terminal connection via `rc_os_print_boot_status` callback. Core 1 sensor phase and watchdog start immediately with no USB dependency. Qwiic chain order documented: GPS must be first (closest to board) — at end of chain, probe detection is intermittent. Soak verified: 536K IMU reads, 0 errors (6 min, all 3 sensors).

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `docs/hardware/HARDWARE.md`)

---

### 2026-02-10-002 | Claude Code CLI | architecture, documentation

**SAD Section 13: I2C peripheral detection and driver management architecture**

Expanded SAD Section 13 (Extensibility) from placeholder to full architecture for I2C peripheral detection. Boot-time probe-first detection (already implemented). Runtime hot-plug detection, device registry with WHO_AM_I disambiguation, and OTA driver downloads for WiFi/BT models (crowdfunding stretch goal for Core/Middle tiers). Resolved open questions #1 (Booster Pack detection) and #2 (OTA updates). Established ESKF gate: foundational features (mag cal wizard, non-blocking USB, unified calibration) must complete before Stage 5.

(`docs/SAD.md`, `AGENT_WHITEBOARD.md`, `docs/PROJECT_STATUS.md`)

---

### 2026-02-10-001 | Claude Code CLI | bugfix, feature

**Fix i2c_bus_recover() peripheral corruption + probe-first detection**

Fixed critical bug: `i2c_bus_recover()` corrupted the DW_apb_i2c peripheral by switching GPIO functions (SIO↔I2C) while the peripheral was enabled. Now properly deinits before GPIO switch and reinits after (LL Entry 28). Added probe-first peripheral detection in `init_sensors()` — only inits drivers for devices physically present on the bus. GPS drain no longer triggers recovery for absent devices. Recovery improved with SCL-stuck-low check and per-pulse SDA early exit (Linux kernel pattern). New soak test script with early-fail detection and SWD reset support. Soak verified: 386K IMU reads, 0 errors.

(`src/drivers/i2c_bus.cpp`, `src/main.cpp`, `scripts/i2c_soak_test.py`)

---

### 2026-02-09-008 | Claude Code CLI | bugfix

**Revert non-blocking USB init (6de6245) — soak failures across 4 build variants**

Reverted premature commit 6de6245 which removed `wait_for_usb_connection()`. Deep research confirmed Pico SDK has no I2C bus recovery function (custom recovery is correct per I2C spec NXP UM10204 Section 3.1.16). Attempted 4 build variants (prod-13 through prod-16) with robust bus recovery (retry loop, 100µs GPIO settling delay, `i2c_deinit()` before bit-bang) and non-blocking USB. All failed soak testing at 40-90 seconds with cascading I2C errors (IMU first, then baro/GPS). Phase M stash@{0} (CLI mag cal) preserved. **[CORRECTION 2026-02-09]:** The "codegen sensitivity" attribution was wrong — all testing used picotool rapid flash cycles, which corrupt the I2C bus (LL Entry 25). Disproved by three soak tests via debug probe with i2c_bus.cpp modifications, all 0 errors (LL Entry 27). Non-blocking USB should be retested via probe.

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-09-007 | Claude Code CLI | documentation

**IVP plan expansion: Phase M magnetometer calibration + Stage 5 sensor fusion flesh-out**

Added Phase M (IVP-34 through IVP-38) — 5 magnetometer calibration IVPs with full ArduPilot CompassCalibrator parity (sphere-coverage, two-step Levenberg-Marquardt, ellipsoid fit). Renumbered all downstream IVPs +5 (Stage 5: IVP-39-50, Stage 6: IVP-51-56, Stage 7: IVP-57-61, Stage 8: IVP-62-66, Stage 9: IVP-67-71). Fleshed out all 10 Stage 5 IVPs with full implementation specs: Vec3/Quaternion library, matrix operations, 1D baro KF, ESKF propagation (Sola 2017), baro/mag/GPS measurement updates, Mahony AHRS, MMAE bank manager, confidence gate. Updated cross-references in AGENT_WHITEBOARD.md, PROJECT_STATUS.md, VENDOR_GUIDELINES.md.

(`docs/IVP.md`, `AGENT_WHITEBOARD.md`, `docs/PROJECT_STATUS.md`, `standards/VENDOR_GUIDELINES.md`)

---

### 2026-02-09-006 | Claude Code CLI | refactor

**Strip IVP test code from production codebase**

Removed all Stage 1-4 IVP test code from `main.cpp` after hardware verification: 3,623→1,073 lines (**2,550 lines removed, 70% reduction**). 33 functions, ~80 state variables, ~70 constants deleted including 4KB jitter timestamp array. Binary **198,144→155,648 bytes (42,496 bytes saved, 21.4% smaller)**. Replaced `g_ivp25Active` with production `g_sensorPhaseActive` (plain bool). Unconditional watchdog enable (council critical fix). Renamed `hw_validate_stage1()` to `print_hw_status()`. Resolved 2 standards deviations: RC-1 (recursion), BM-6 (unbounded test loop). Updated IO-1 printf count (428→212). Changed main.cpp classification from "IVP Test (mixed)" to "Ground (mixed)". Deleted 4 IVP files (2 test scripts, 2 log files). All IVP code preserved in git history.

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `standards/STANDARDS_DEVIATIONS.md`, `standards/CODING_STANDARDS.md`, `standards/AUDIT_REMEDIATION.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-09-005 | Claude Code CLI | refactor

**Clang-tidy P5b-f: identifier naming, bool conversions, init vars, function decomposition**

Completed remaining clang-tidy remediation for all production code. P5b: 162 identifier renames to camelCase. P5c: 41 implicit bool conversions made explicit. P5d: 51 uninitialized variables initialized at declaration. P5e: 9 production functions decomposed under 60-line JSF AV limit. P5f: JSF AV Rule 213 math parentheses check disabled (LL Entry 26). Binary 198,656 bytes (+512 from function call overhead). HW verified: 0 errors across all sensors. All production code now fully remediated — only IVP test code remains deferred.

(18 files across src/, standards/, docs/, .claude/)

---

### 2026-02-09-004 | Claude Code CLI | refactor

**Clang-tidy audit remediation — 5 phases, 911 findings resolved**

P1: Safety-critical fixes (widening multiplication overflow, narrowing conversions, missing default clause) — HW verified 0 errors/117K reads. P2: 354 auto-fixable cosmetic warnings (uppercase suffixes, void args, nullptr, bool literals, using). P3: 275 magic numbers extracted to named constexpr (production code); IVP test code NOLINT'd. P4: 170 missing braces added per JSF AV Rule 59. P5: ~80 C-style casts converted to static_cast/reinterpret_cast per JSF AV Rule 185. Binary size unchanged throughout (198,144 bytes). Remaining ~370 cosmetic findings (naming, parens, implicit bool, uninit vars) documented as deferred in `standards/AUDIT_REMEDIATION.md`.

(all 10 source files, `standards/AUDIT_REMEDIATION.md`)

---

### 2026-02-09-003 | Claude Code CLI | tooling

**Comprehensive clang-tidy standards audit — 127 checks mapped to JSF AV / P10 / JPL C**

Rewrote `.clang-tidy` from 78-line function-size-only config to 376-line comprehensive standards audit config covering bugprone (34), cert (14), cppcoreguidelines (11), google (6), hicpp (3), misc (11), modernize (11), performance (11), readability (26), and clang-analyzer checks. Ran full audit across all 10 source files: 1,251 warnings in our code. Key finding: `misc-no-recursion` caught a recursive call chain that the manual audit missed (P10-1 was marked PASS). 5 safety-critical findings documented. Results in `docs/audits/CLANG_TIDY_AUDIT_2026-02-09.md`.

(`.clang-tidy`, `docs/audits/CLANG_TIDY_AUDIT_2026-02-09.md`)

---

### 2026-02-09-002 | Claude Code CLI | refactor

**Decompose main() and core1_entry() — P10-4 function length compliance**

Extracted `main()` (992→65 lines) into `init_hardware()`, `print_boot_status()`, `init_application()`, and 15 tick-function dispatchers per council-approved plan. Extracted `core1_entry()` (367→15 lines) into `core1_test_dispatcher()`, `core1_spinlock_soak()`, `core1_sensor_loop()`. Added `g_lastTickFunction` watchdog tracking (council recommendation). Binary size unchanged (198,144 bytes). Hardware-verified: all behavior identical.

(`src/main.cpp`)

---

### 2026-02-09-001 | Claude Code CLI | documentation

**Trim tracking documents — collapse completed sections**

Cleaned up PROJECT_STATUS.md (completed stages → summary table), AGENT_WHITEBOARD.md (erased resolved flags, compacted deferred items into bullet list), AUDIT_REMEDIATION.md (line-by-line fix tables → summary). Net -365 lines. No information lost — per-IVP detail remains in IVP.md, audit detail in STANDARDS_AUDIT_2026-02-07.md.

(`docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md`, `standards/AUDIT_REMEDIATION.md`)

---

### 2026-02-08-006 | Claude Code CLI | feature

**IVP-32/33: GPS outdoor validation + CLI integration (Phase 4 complete)**

GPS fix confirmed outdoors (PPS lock, lat/lon accuracy verified). CLI `s` command format polished to match IVP-33 gate wording (`GPS: no fix (N sats)`). BSS layout regression investigated — `alignas(64)` + flag grouping caused new errors. Minimal-change build (`ivp32-33-1`) passes with 68K+ reads, 0 errors. **[CORRECTION 2026-02-09]:** "BSS layout regression" was actually picotool bus corruption from rapid flash cycles (LL Entry 25/27). Disproved — see LL Entry 27.

(`src/main.cpp`)

---

### 2026-02-08-005 | Claude Code CLI | feature

**IVP-31: PA1010D GPS integration on Core 1 with I2C contention fix**

GPS reads at 10Hz on Core 1 via seqlock. Full NMEA parsing (lwGPS), GGA+RMC+GSA sentence filter, Adafruit-style 0x0A padding filter, cold-boot bus recovery. I2C contention (8.4% IMU error rate at gps-10) resolved with 500us post-read settling delay — controlled isolation tests (gps-12a/b/c) proved delay alone is sufficient at 10Hz. Auto-detection: delay only active when I2C GPS at 0x10 is detected; UART GPS skips it. `kCore1ConsecFailMax` lowered from 50 to 10 for GPS-induced burst detection.

*Rationale: 500us delay vs 5Hz rate reduction — delay preserves 10Hz GPS data with only 0.5% CPU overhead. Isolation testing documented in LL Entry 24. ArduPilot never shares GPS with high-rate sensors on I2C; production migrates to UART FeatherWing (Adafruit 3133).*

(`src/main.cpp`, `src/drivers/gps_pa1010d.cpp`, `src/drivers/gps_pa1010d.h`, `.claude/LESSONS_LEARNED.md`)

---

### 2026-02-08-004 | Claude Code CLI | documentation

**Titan doc: H7 board candidates section**

Added Section 13 to `TITAN_BOARD_ANALYSIS.md` — STM32H7 board landscape research. No maker H7 boards exist in Feather/Thing Plus form factor. Recommended Matek H743-Wing V3 flight controller for Titan Path A prototyping (ArduPilot-validated, dual IMU, DPS310, CAN, 13 PWM, ~$55-65).

(`docs/decisions/TITAN_BOARD_ANALYSIS.md`)

---

### 2026-02-08-003 | Claude Code CLI | refactor

**C++20 conversion: .c→.cpp rename + #define→constexpr (PP-1 resolved)**

Two-commit migration of all 9 C source files to C++20. Commit 1: renamed files via `git mv`, fixed C99 compound literals, added `extern "C"` wrap for ruuvi DPS310 library, removed internal `extern "C"` guards from headers, fixed `const` cast in GPS driver. Commit 2: converted 90+ `#define` macros to `constexpr` across all drivers/calibration/CLI — ICM-20948 registers organized into bank-scoped namespaces, calibration/storage constants use `k` prefix. Resolved PP-1 deviation (JSF 29/30/31). Binary size delta: -1404 bytes (97988 vs 99392 baseline). Third-party libs (ruuvi, lwGPS) remain C.

(`src/**/*.cpp`, `src/**/*.h`, `include/rocketchip/config.h`, `CMakeLists.txt`, `standards/STANDARDS_DEVIATIONS.md`)

---

### 2026-02-08-002 | Claude Code CLI | refactor, documentation

**Standards audit remediation — Tier 4: RC_ASSERT, goto elimination, deviation docs**

Completed Phase D of the audit remediation plan. D1: Added `RC_ASSERT()` macro to `config.h` (debug = printf + watchdog spin, release = no-op). D2: Eliminated all 6 `goto` statements in `cmd_accel_6pos_cal()` by extracting body into `cmd_accel_6pos_cal_inner()` helper — wrapper guarantees I2C master pre/post hook execution. D4: Documented bare-metal loop deviations (BM-1 through BM-6), stdio usage (IO-1/IO-2), and preprocessor defines (PP-1) in `STANDARDS_DEVIATIONS.md`. Updated audit dashboard: 220 PASS / 25 PARTIAL/FAIL (90% compliance, up from 82%).

(`include/rocketchip/config.h`, `src/cli/rc_os.c`, `standards/STANDARDS_DEVIATIONS.md`, `standards/AUDIT_REMEDIATION.md`, `standards/STANDARDS_AUDIT_2026-02-07.md`)

---

### 2026-02-08-001 | Claude Code CLI | architecture, council

**F' (F Prime) comprehensive evaluation for Titan**

Expanded the F' addendum in `TITAN_BOARD_ANALYSIS.md` from a brief note into a full 14-section evaluation. Covers: F' architecture and component model, coding standards alignment with JSF AV (near-complete overlap), platform support matrix (RP2350 HSTX is officially supported via fprime-arduino), the critical multicore limitation (Zephyr has no Cortex-M SMP — F' multicore only works on Linux via pthreads), Pi Zero 2 W vs STM32H7 hardware comparison, a hybrid architecture proposal (Pi Zero 2 W running F'/Linux as mission CPU + RP2350 as real-time safety CPU), MAVLink incompatibility (F' uses its own protocol), F' flight heritage (Ingenuity, ASTERIA, RapidScat), and a "cherry-pick" alternative for adopting F' patterns without the framework.

(`docs/decisions/TITAN_BOARD_ANALYSIS.md`)

---

### 2026-02-07-001 | Claude Code CLI | documentation

**New standard: Vendor & OEM Guidelines**

Created `standards/VENDOR_GUIDELINES.md` — centralized reference for vendor-specific constraints, datasheet-sourced values, and OEM recommendations. Consolidates knowledge previously scattered across LL entries, driver comments, IVP notes, and whiteboard flags. Covers ICM-20948 (bank-switching, I2C master race), DPS310 (config, noise spec gaps), PA1010D (255-byte full-buffer reads per vendor app note, bus interference behavior, PMTK commands), RP2350 (errata E2, USB/flash ordering, memory constraints), and Feather board pin assignments. Includes datasheet inventory with gap analysis — DPS310 and PA1010D datasheets are missing locally.

(`standards/VENDOR_GUIDELINES.md`)

---

### 2026-02-06-003 | Claude Code CLI | architecture, documentation

**Stage 4 GPS IVP revision — restructured for dual-core architecture**

Rewrote IVP-31 through IVP-34 (now IVP-31 through IVP-33) after Stage 3 established that Core 1 owns the I2C bus exclusively. Original IVPs assumed GPS could run on Core 0 — this causes bus collisions (LL Entry 20). Key changes: GPS init before Core 1 launch, GPS reads on Core 1 sensor loop (full 255-byte reads at 10Hz per vendor recommendation), seqlock for Core 0 access, old IVP-33 (Core 1 migration) merged into IVP-31 (it's now a prerequisite, not a follow-on). Updated `gps_pa1010d.c` with full-buffer reads and PMTK314 sentence filter. Renumbered IVP-35+ down by 1 to close the gap (now IVP-34 through IVP-71). Reverted partial IVP-31 implementation from main.cpp.

(`docs/IVP.md`, `src/drivers/gps_pa1010d.c`, `src/main.cpp`)

*Rationale: The original Stage 4 IVPs were written before Stage 3's dual-core work revealed that I2C bus access is single-core only (no mutual exclusion in Pico SDK). Attempting Core 0 GPS reads with Core 1 running IMU/baro caused ICM-20948 init failures. Council review confirmed GPS on Core 1 is the only correct approach. The 32-byte read size was an Arduino Wire.h software limitation — vendor app notes and Pico SDK examples both use full 255-byte reads.*

---

### 2026-02-06-002 | Claude Code CLI | architecture, council, documentation

**Stage 3 prep: Seqlock cross-core design — research, council review, IVP corrections**

Created `docs/decisions/SEQLOCK_DESIGN.md` — council-reviewed decision document for cross-core data sharing via seqlock. Four parallel research agents investigated struct layout, prior art (ArduPilot/Betaflight/PX4), RP2350 memory model, and notification mechanisms. Council (3 personas) unanimously approved with 7 required modifications (all incorporated): bounded retry loop, `core1_loop_count`, `mag_read_count`, `_Static_assert` guards, DMB rationale comments, `_Atomic bool cal_reload_pending` signaling. Corrected RP2350 errata reference from E17 to E2 across IVP and whiteboard. Audited SAD Section 4.3 seqlock code — found 5 issues (missing DMB barriers, unnecessary double-buffer, wrong errata ID, nonexistent types, missing timestamps). Updated `AGENT_WHITEBOARD.md` with Stage 3 session plan and research findings.

(`docs/decisions/SEQLOCK_DESIGN.md`, `docs/IVP.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-06-001 | Claude Code CLI | feature, bugfix

**IVP-15/16/17/18: Calibration suite + CLI integration — Minimum Viable Demo milestone**

Fixed ICM-20948 returning all zeros after ~150 rapid accel reads during 6-position calibration. Root cause: internal I2C master (for AK09916 mag) races with external reads on shared bank-select register (0x7F). Added `icm20948_set_i2c_master_enable()` API and pre/post calibration hooks to disable I2C master during sampling. Removed motion check and orientation pre-check (ArduPilot doesn't use either — Gauss-Newton solver handles bad data). Reordered positions to QGroundControl standard. Added `i2c_bus_reset()` after flash saves. Version bumped to 0.1.1. HW-verified: 3/3 consecutive runs pass across USB replug cycles.

(`src/drivers/icm20948.c/h`, `src/calibration/calibration_manager.c`, `src/cli/rc_os.c/h`, `src/main.cpp`, `scripts/accel_cal_6pos.py`, `include/rocketchip/config.h`)

---

### 2026-02-05-005 | Claude Code CLI | feature

**IVP-14: Calibration storage (flash persistence)**

Re-enabled calibration system (`calibration_data`, `calibration_manager`, `calibration_storage`) in CMakeLists.txt with `hardware_flash` and `pico_flash` libs. Added storage init before USB (per LL Entry 4/12), manager init after sensors. IVP-14 self-test verifies all 4 gates: load/defaults, save/readback match, power cycle persistence, 10 consecutive saves (wear leveling). Added `kSkipVerifiedGates` flag to skip IVP-10/12/13 at boot — set false to re-run. All gates pass across power cycles.

(`CMakeLists.txt`, `src/main.cpp`)

---

### 2026-02-05-004 | Claude Code CLI | feature

**IVP-13a: I2C bus recovery under fault**

Implemented I2C bus recovery for sensor disconnect/reconnect scenarios. Recovery fires after 50 consecutive errors (~333ms), performs bus reset (deinit + 9-clock bit-bang + STOP + reinit). IMU self-recovers from reads; baro gets lazy reinit after 100 consecutive failures when IMU confirms bus is healthy. Hardware-verified: Qwiic cable disconnect during 100Hz/50Hz polling — no hang, both sensors resume at full rate after reconnect, error counter visible in status output. All 4 IVP-13a gates pass.

(`src/main.cpp`)

---

### 2026-02-05-003 | Claude Code CLI | feature

**IVP-09 through IVP-13: IMU, barometer, and multi-sensor polling**

Re-enabled ICM-20948 and DPS310 drivers incrementally with verification at each gate. IMU: fixed 6 magnetometer init issues, added 3-attempt retry, set ±4g/±500dps. DPS310: ruuvi library with 8Hz/8x oversampling continuous mode. IVP-10 validation (50 IMU samples, magnitude-based gate checks), IVP-12 validation (100 baro samples, pressure/noise/stuck-value gates), IVP-13 multi-sensor polling (IMU 100Hz, baro 50Hz, 60s, zero I2C errors). Measured I2C timing: IMU avg 774us, baro avg 251us. All gates pass across reboot cycles.

(`CMakeLists.txt`, `src/main.cpp`, `src/drivers/icm20948.c`, `docs/PROJECT_STATUS.md`)

---

### 2026-02-05-002 | Claude Code CLI | bugfix, refactor

**Reset to clean Stage 1 baseline, fix I2C bus recovery**

Stripped all Stage 2 code from build (drivers, calibration, CLI) to resolve I2C bus reliability issues. Root cause: two compounding issues — (1) no bus recovery on boot after picotool `--force` reboots left sensors mid-transaction with SDA held low, (2) Stage 2 init code (IMU bank switching, mag I2C master, flash ops) was corrupting bus state. Added `i2c_bus_recover()` call before `i2c_init()` in `i2c_bus_init()`. Restored 400kHz (100kHz was a red herring). All 3 devices (0x69, 0x77, 0x10) now detected reliably. Stage 2 source files remain on disk for incremental re-enablement.

(`CMakeLists.txt`, `src/main.cpp`, `src/drivers/i2c_bus.c`, `src/drivers/i2c_bus.h`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-05-001 | Claude Code CLI | documentation, architecture

**Telstar Booster Pack, docs/hardware/ reorganization, Gemini ELRS section**

Created `docs/hardware/` subdirectory and moved hardware design documents into it (`HARDWARE.md`, `GEMINI_CARRIER_BOARD.md`, `STATUS_INDICATORS.md`) using `git mv` to preserve history. Created `docs/hardware/TELSTAR_BOOSTER_PACK.md` — Telstar Booster Pack design document covering ELRS RC link, CRSF protocol, FPV video transmitter, standalone product potential, FAA Remote ID module support, and updated Booster Pack lineup. Added Section 8.4 (Dedicated ELRS Communications Core) to Gemini doc. Updated all cross-references across repo (README, SAD, SCAFFOLDING, PROJECT_OVERVIEW, HARDWARE, ICDs, SpaceWire-Lite).

---

### 2026-02-04-003 | Claude Code CLI | bugfix

**I2C debug: drop to 100kHz, add verbose scan output**

Cross-referenced current I2C code against working `AP_FreeRTOS` branch. Code is functionally identical (same pins, same instance, same pull-ups) except the working `i2c_scan.c` test used 100kHz, not 400kHz. Changed `I2C_BUS_FREQ_HZ` to 100kHz. Added verbose scan output showing I2C instance number, GPIO pin states, and configured frequency. Build ready, not yet flashed.

(`src/drivers/i2c_bus.c`, `src/drivers/i2c_bus.h`)

---

### 2026-02-04-002 | Claude Code CLI | feature

**IVP Stage 2: RC_OS CLI + Calibration Integration**

Implemented bare-metal RC_OS CLI with calibration integration (IVP-15, IVP-16, IVP-18). CLI provides single-key command interface with calibration menu, sensor status, and I2C rescan capability. Sensor availability checks prevent calibration commands when sensors not initialized. Non-blocking calibration progress monitoring with dots and OK/FAIL output.

**Status:** Calibration logic complete but blocked on I2C issue — see AGENT_WHITEBOARD.md for details.

---

### 2026-02-04-001 | Claude Code CLI | feature, hardware

**IVP Stage 1 Complete: Foundation (IVP-01 through IVP-08)**

Created `src/main.cpp` implementing bare-metal firmware foundation: red LED heartbeat, NeoPixel rainbow via PIO, USB CDC serial with terminal reconnect handling, I2C bus init and scan, structured HW validation output. All gates hardware-verified (ICM-20948, DPS310, PA1010D detected).

Deleted stale files with broken dependencies (`accel_calibrator.c/h`, `debug.h`). Updated CMakeLists.txt to build only Stage 1 sources; other drivers commented with IVP stage markers for incremental re-enablement.

---

### 2026-02-03-007 | Claude Code CLI | tooling

Added plan mode council review instructions to `.claude/CLAUDE.md`. Before ExitPlanMode, agent now asks which council personas to use, spawns a Task agent to run the review, and attaches the verdict to the plan.

---

### 2026-02-03-006 | Claude Code CLI | documentation, architecture

Added Section 8.2 (Dual-IMU Fusion and EKF Lane Switching) to `docs/GEMINI_CARRIER_BOARD.md`. Covers EKF3-style lane switching concept using Gemini's dual independent sensor suites, bandwidth analysis for high-rate bidirectional sensor exchange over SpaceWire-Lite, and protocol implications including new SENSOR_RAW and EKF_HEALTH message types.

---

### 2026-02-03-005 | Claude Code CLI | documentation, architecture, council

**Integration and Verification Plan (IVP) + SAD Updates**

Created `docs/IVP.md` — 64-step development roadmap across 9 stages with pass/fail verification gates. Council review (4 personas) produced 12 findings, all implemented: `⚠️ VALIDATE` convention for numerical values, RP2350 inter-core primitives section (spinlocks, FIFO, doorbells as IVP-21 through IVP-23), GPS as Stage 4 before fusion, I2C bus recovery, MPU details, milestone markers.

SAD.md updates: per-sensor validity flags, seqlock implementation replacing TODO, inter-core primitives table, PIO allocation table, AMP architecture corrections, Section 14 numbering fix. SCAFFOLDING.md and PROJECT_STATUS.md updated with PIO_ALLOCATION.md placeholders.

---

### 2026-02-03-004 | Claude Code CLI | documentation, architecture

Added PIO hardware watchdog design concepts document (`docs/PIO/PIO_WATCHDOG.md`). Covers heartbeat watchdog, dual-core cross-check, and pyro channel lockout using PIO state machines as CPU-independent safety monitors. Concept stage — not committed to IVP.

---

### 2026-02-03-003 | Claude Code CLI | documentation, refactor

**Bare-Metal Pivot — Documentation Cleanup (continued)**

Systematic cleanup of FreeRTOS/ArduPilot references across all documentation.

- LESSONS_LEARNED.md — archived 8 FreeRTOS-specific entries (7-10, 14, 17-19), minor rewording on entries 3, 4, 12, 15
- DEBUG_OUTPUT.md — replaced deferred logging with direct printf macros, renamed per-task to per-module, updated build config table
- CODING_STANDARDS.md — removed RTOS tier table, ArduPilot Library Integration, Dependency Bypassing Policy, HAL Adaptation Policy sections; demoted ArduPilot from "check first" to "useful reference" in Prior Art Research; removed archived LL entry references
- PROJECT_OVERVIEW.md — replaced "RTOS" with "deterministic control loops" in Titan tier, "FreeRTOS" with "Bare-metal Pico SDK" in Technical Foundation
- PROJECT_STATUS.md — added ChibiOS upstream note, added full ArduPilot integration as back burner goal
- ROCKETCHIP_OS.md — replaced FreeRTOS Task Model with bare-metal Execution Model, updated source files table, replaced platform-specific FreeRTOS issues with USB CDC concerns, removed priority-based references
- Deleted GETTING_STARTED.md (thoroughly outdated, not needed until development matures)

---

### 2026-02-03-002 | Claude Code CLI | architecture, refactor

**Pivot from FreeRTOS to Bare-Metal Pico SDK**

Removed all FreeRTOS dependencies, pivoting to bare-metal Pico SDK with polling main loop.

**Deleted:**
- FreeRTOS-Kernel submodule, FreeRTOSConfig.h, FreeRTOS_Kernel_import.cmake
- docs/FreeRTOS/TASK_PRIORITIES.md
- src/main.cpp, src/tasks/sensor_task.c/h, src/debug/debug_stream.c/h (RTOS glue — will be rewritten)

**Edited:**
- CMakeLists.txt — removed FreeRTOS imports, heap link, commented out deleted sources
- config.h — replaced task priorities/stacks with polling timing constants, simplified debug macros to direct printf
- CODING_STANDARDS.md — replaced FreeRTOS platform constraints with bare-metal rules
- SAD.md, PROJECT_STATUS.md, SCAFFOLDING.md, MULTICORE_RULES.md — updated for bare-metal architecture
- DEBUG_PROBE_NOTES.md, SESSION_CHECKLIST.md — minor RTOS reference removal
- ws2812_status.h — comment update

**Deferred for later evaluation (>40% rewrite needed):**
- DEBUG_OUTPUT.md — deferred logging architecture references deleted code
- LESSONS_LEARNED.md — ~50% of entries are FreeRTOS-specific

**Preserved unchanged:** src/calibration/*, src/drivers/* (sensor drivers, calibration, LED — no RTOS dependency)

---

### 2026-02-03-001 | Claude Code CLI | documentation

**Platform Constraints, Multi-Core Rules, Session Checklist, Decisions Folder**

Added critical documentation capturing RP2350 + FreeRTOS SMP platform constraints and session management procedures.

- Added `standards/CODING_STANDARDS.md` "RP2350 + FreeRTOS SMP Platform Constraints" section (~98 lines) - non-negotiable rules derived from LESSONS_LEARNED entries
- Created `docs/MULTICORE_RULES.md` - core assignment rules, cross-core communication, memory barriers
- Created `.claude/SESSION_CHECKLIST.md` - session start/end procedures, handoff protocol
- Created `docs/decisions/` folder structure for council review outputs
- Moved `docs/ESKF/` to `docs/decisions/ESKF/`
- Updated `docs/PROJECT_STATUS.md` to "Reboot: Validate & Rebuild" phase with validation checklist
- Updated `.claude/PROTECTED_FILES.md` with new protected entries
- Updated `.claude/CLAUDE.md` with new @ references

**Files:** CODING_STANDARDS.md, MULTICORE_RULES.md, SESSION_CHECKLIST.md, docs/decisions/README.md, PROJECT_STATUS.md, PROTECTED_FILES.md, CLAUDE.md

---

### 2026-02-02-004 | Claude Code CLI | bugfix, architecture

**FreeRTOS SMP + USB CDC Fix + Calibration Stack Fix**

Fixed USB CDC enumeration failure on RP2350 with FreeRTOS SMP, plus stack overflow during 6-position accel calibration.

**USB CDC Root cause:** Using main FreeRTOS repo instead of Raspberry Pi's fork, plus incorrect FreeRTOSConfig.h settings.

**Calibration Root cause:** Sensor task stack (256 words) too small for ellipsoid fit local matrices. Increased to 512 words.

**Changes:**
- Replaced FreeRTOS-Kernel submodule with raspberrypi/FreeRTOS-Kernel (has RP2350_ARM_NTZ port)
- Rewrote FreeRTOSConfig.h to match pico-examples/freertos/hello_freertos
- Removed custom USB task - SDK's IRQ-based handling is correct approach
- Pinned sensor sampling to Core 1 (Core 0 hosts USB IRQ handlers)
- Simplified CMakeLists.txt USB configuration to use SDK defaults
- Increased SENSOR_TASK_STACK from 256 to 512 words for calibration

**Files:** FreeRTOS-Kernel (submodule), FreeRTOSConfig.h, FreeRTOS_Kernel_import.cmake, src/main.cpp, CMakeLists.txt, src/tasks/sensor_task.c

**Reference:** LESSONS_LEARNED.md Entry 18 (USB), Entry 19 (stack overflow)

**Status:** Calibration runs to completion but ellipsoid fit diverges - needs algorithm tuning.

*Rationale: pico-examples/freertos/hello_freertos is the canonical reference for FreeRTOS on RP2350. Previous attempts used custom configurations that conflicted with SDK's USB stack. The Raspberry Pi FreeRTOS fork contains tested, working SMP support that the main FreeRTOS repo lacks.*

---

### 2026-02-02-003 | Claude Code CLI | documentation

**Documentation Cleanup for Fresh Start**

Reviewed all docs folder files to ensure consistency with bespoke FreeRTOS approach after branch reorganization.

- Updated SAD.md: Rewrote directory structure, module dependencies, driver interfaces, storage architecture, and reset all development phases
- Updated SCAFFOLDING.md: Aligned with SAD.md, reset implementation status
- Updated TOOLCHAIN_VALIDATION.md: Removed stale sensor references, added Phase 2 note
- Updated HARDWARE.md: Removed ArduPilot driver references from IMU and GPS sections
- Updated .claude/CLAUDE.md: Removed reference to deleted RP2350_FULL_AP_PORT.md
- Deleted: docs/RP2350_FULL_AP_PORT.md, docs/AP_HAL_RP2350_PLAN.md (archived in AP_FreeRTOS branch)
- Moved: PROJECT_STATUS.md, PROJECT_OVERVIEW.md to docs/ folder

ESKF docs and ROCKETCHIP_OS.md left as-is (historical reference for design decisions).

---

### 2026-02-02-002 | Nathan (via Claude Opus council review) | architecture

**Sensor Fusion Architecture: ESKF + MMAE**

Changed sensor fusion approach from ArduPilot EKF3 extraction to custom Error-State Kalman Filter (ESKF) with Multiple Model Adaptive Estimation (MMAE) bank for anomaly resilience.

- Added `docs/ESKF/FUSION_ARCHITECTURE_DECISION.md` - full rationale and design
- Added `docs/ESKF/FUSION_ARCHITECTURE.md` - reference document
- Updated SAD.md §5.4 (fusion), §10 (Phase 4), key decisions table, open questions
- Updated SCAFFOLDING.md FusionTask descriptions
- AP_HAL_RP2350 still used for calibration/math/storage - NOT for fusion
- Added `.claude/PROTECTED_FILES.md` - files requiring explicit permission to edit

**Note:** Specific numerical parameters (state counts, filter counts, latencies) pending systematic review before implementation.

---

### 2026-02-02-001 | Claude Code CLI | architecture

**Branch Reorganization - Fresh Start with Bespoke FreeRTOS**

Major pivot: Archived ArduPilot integration attempts, starting fresh with bespoke FreeRTOS approach.

- Created `AP_ChibiOS` branch (archived ChibiOS exploration - blocked by XIP flash issues)
- Created `AP_FreeRTOS` branch (archived FreeRTOS + ArduPilot HAL work - mature but complex)
- Fresh `main` branch for new bespoke implementation

Cherry-picked universal reference documentation (LESSONS_LEARNED, DEBUG_PROBE_NOTES, HARDWARE, SAD, SCAFFOLDING, ROCKETCHIP_OS, coding standards).

**Reference:** Full reorganization plan at `C:\Users\pow-w\.claude\plans\spicy-toasting-breeze.md`

---
