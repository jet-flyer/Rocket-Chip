# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes

---

## Open Flags

---

### SRAM Execution Audit — Check for Other XIP Cache Bottlenecks

**Added 2026-02-21.** IVP-47 codegen FPFT revealed that RP2350's 2KB XIP cache causes catastrophic performance loss for large hot functions (10KB codegen: 398µs from flash → 59µs from SRAM, 6.7× difference). The `.time_critical` section is the SDK's intended solution. **Updated 2026-02-21:** 24-state codegen is ~1100 lines (~566 changes), function is larger than 15-state version. Still in `.time_critical` SRAM section. Benchmark: 111µs avg (was 59µs at 15-state). **Audit remaining high-frequency code paths** for functions that may exceed XIP cache: `scalar_kalman_update()` (now O(N²) rank-1 Joseph form, runs at baro/mag/GPS rates), `propagate_nominal()` (200Hz), sensor read functions on Core 1. Low-rate functions (<10Hz) are unlikely to benefit.

---

### Watchdog Recovery Policy — IVP-51 (New, Stage 6 Prerequisite)

**Added 2026-02-12.** IVP-30 watchdog mechanism is correct for ground/IDLE. But a full reboot mid-flight loses all ESKF state, pyro timers, and nav knowledge. IVP-51 added as first step of Stage 6 (before state machine IVP-52) to define the recovery policy: scratch register persistence, reboot counting with safe-mode lockout, ESKF failure backoff, and recovery boot path.

**False warning bug — FIXED (2026-02-12).** Root cause: both SDK functions are broken for our use case. `watchdog_caused_reboot()` gives false positives on SWD `monitor reset run` (reason register persists, `rom_get_last_boot_type()` returns `BOOT_TYPE_NORMAL`). `watchdog_enable_caused_reboot()` gives false negatives on real timeouts (bootrom overwrites scratch[4]). Fix: custom sentinel in scratch[0] (`kWatchdogSentinel = 0x52435754`), written before `watchdog_enable()`, checked and cleared at boot. Scratch[0] survives watchdog resets but is cleared by POR/SWD reset. HW verified: no warning on cold plug, boot button, or first SWD flash. Warning correctly appears on SWD reflash while watchdog is running (genuine timeout during bootrom — bootrom takes >5s, watchdog fires). This remaining case will be addressed with the broader IVP-51 watchdog recovery policy.

**IVP renumber (2026-02-18):** All Stage 6-9 IVP numbers shifted +2 (IVP-49→51 through IVP-69→71). New IVP-47 (Sparse FPFT) and IVP-48 (ESKF Health Tuning) inserted.

---

### Protected File Updates Pending Approval

*None currently.*

---

### Missing Vendor Datasheets

| Document | Priority | Needed By |
|----------|----------|-----------|
| DPS310 datasheet (Infineon) | MEDIUM | ESKF baro tuning, IVP-48 health diagnostics |
| RFM95W / SX1276 datasheet (Semtech) | MEDIUM | Stage 8 (telemetry) |

Source URLs in `standards/VENDOR_GUIDELINES.md` Datasheet Inventory section.

---

## Deferred Notes

*Items noted for future stages — not blocking, no action needed now.*

- **3-Axis Magnetometer Model (Titan tier):** Current `update_mag_heading()` uses yaw-only scalar H (heading + WMM declination). Mag states (earth_mag indices 15-17, body_mag_bias 18-20) are **unobservable** with this model — no F coupling (identity propagation), no H entries at 15-20. States MUST remain inhibited until a proper 3-axis measurement model is implemented: `z_predicted = R(q) * earth_mag_NED + body_mag_bias`, with H entries at attitude (0-2), earth_mag (15-17), body_mag_bias (18-20). WMM table already has field data — needs inclination + magnitude extraction in addition to current declination-only use. ArduPilot reference: `fuseMagnetometer()` in `NavEKF3_MagFusion.cpp`. Once implemented, flip `inhibit_mag_states_` to enable.
- **F' Evaluation:** Three Titan paths identified (A: STM32H7+F'/Zephyr, B: Pi Zero 2 W+F'/Linux, C: Hybrid). Research complete in `docs/decisions/TITAN_BOARD_ANALYSIS.md`. Decision deferred until Titan development begins.
- ~~**FeatherWing UART GPS:**~~ **DONE** (2026-02-18). `gps_uart.cpp` driver complete with interrupt-driven ring buffer. Outdoor validated.
- **u-blox GPS (Matek M8Q-5883):** UART + QMC5883L compass. UBX binary protocol. For production/flight builds, not current IVP.
- **LED Engine Refactor (back burner):** Current ws2812 driver is a solid animation engine but callers must manage transition-only `set_mode()` calls manually (fixed 2026-02-19 with `neo_set_if_changed()`). Refactor to a proper declarative state machine where flight/status states map to LED patterns — the engine handles transitions internally. Natural integration point: IVP-54 (action executor) when mapping to ArduPilot AP_Notify standard codes. Outdoor LED verification pending with GPS fix test.
- **clang-tidy Integration:** LLVM installed, 127-check config active, full audit clean (2026-02-20, P5c complete). **All code fully remediated** across 6 phases (P1-P5c, 1,501 total findings + 24 function decompositions). Pre-commit hook active for function-size + cognitive complexity gate. Full 127-check pre-commit enforcement deferred (too slow for every commit). **Post-dev caveat:** clang-tidy has no native cyclomatic complexity check — the cognitive complexity gate is a proxy, not equivalent. JSF AV Rule 3 requires cyclomatic CC <= 20. A post-dev audit with a true cyclomatic tool is needed for final JSF compliance sign-off. Options: (1) `lizard` — Python, `pip install lizard`, run `lizard -C 20 src/` for instant per-function CC report; (2) `pmccabe` — C-native, lightweight; (3) `metrix++` — Python, multi-metric. Manual review of functions scoring cognitive 20+ is also viable given the codebase size.
- **Dynamic Peripheral Detection + OTA Drivers (Crowdfunding Goal):** Boot-time probe-first detection implemented (2026-02-10). Runtime hot-plug, driver registry, and OTA firmware downloads for unrecognized devices are stretch goals. Full architecture documented in SAD Section 13.2. Flipper Zero-style: plug in a sensor, RC identifies it, prompts for driver. WiFi/BT OTA for Core/Middle tiers.
- **State-Aware ZUPT for State Machine (IVP-52):** Current ZUPT (IVP-44b) uses IMU-based stationarity detection with kSigmaZupt=0.5 m/s. When the state machine knows we're IDLE or ARMED (on pad), we can: (1) tighten R to ~0.1 m/s since stationarity is guaranteed, (2) skip the accel/gyro stationarity check entirely, (3) keep loose ZUPT as fallback for uncertain states (LANDED before confirmed). ArduPilot EKF3 `onGround` flag and PX4 ECL `vehicle_at_rest` both use vehicle state to override IMU-based detection. Natural integration point: `eskf_tick()` checks flight state and calls `update_zupt()` with tighter R when on pad.
- **Direct NOAA/IGRF WMM Table Generation:** Current WMM declination table is converted from ArduPilot's `AP_Declination/tables.cpp` (IGRF13 epoch). Long-term goal: generate table directly from NOAA WMM/IGRF coefficients (spherical harmonic expansion) to remove AP dependency. NOAA publishes WMM coefficients as `WMM.COF` file every 5 years (next: WMM2030). Python script to evaluate the model at grid points and output C++ table. Not blocking — current table is valid through ~2028-2029.
- **RC GCS: GPS-free 3D Flight Path Reconstruction (user request, 2026-02-13):** Post-processing feature for RC GCS. Uses raw IMU+baro+mag flight logs with forward-backward RTS smoother and known boundary conditions (launch point = origin, v=0 at ignition and landing, landing point from recovery GPS/manual entry). Reconstructs full 3D flight path without real-time GPS. Core tier ships without GPS — this makes 3D visualization viable for all tiers. Natural companion to IVP-44b ZUPT (stationary constraint) and IVP-46 (GPS when available). Reference: ArduPilot `tools/replay/`, PX4 `ecl/EKF/ekf_helper.cpp` smoother.

---

## Resolved

### 24-State ESKF Expansion — COMPLETE (2026-02-21)

15→24 error states: earth_mag NED (3), body_mag_bias (3), wind_NE (2), baro_bias (1). Runtime inhibit flags (ArduPilot EKF3 pattern) — all new states inhibited by default. Codegen regenerated for 24 states (111µs avg, was 59µs at 15-state). O(N²) rank-1 Joseph form replaced O(N³) dense triple product in measurement updates. Sparse reset exploiting G=I except 3×3 attitude block (~450 MACs vs ~27,648 dense). 199/199 host tests, 0 sensor errors on target. CLI shows `inhib: mag=Y wind=Y bbias=Y` + conditional extended state display. 18 files changed.

### IVP-47 Codegen FPFT — COMPLETE (2026-02-21)

SymPy codegen (`scripts/generate_fpft.py`) generates flat scalar C++ for F*P*F^T + Q_d. 199 CSE intermediates, SRAM execution (`.time_critical` section). Block-sparse tried first (31% slower, reverted). Codegen: **9.1× speedup, 538µs → 59µs avg** (50µs min, 113µs max). 194/194 host tests, 0 sensor errors on target. Binary +21KB text, +10KB .data. Standards deviation CG-1 logged. Joseph-form measurement updates (`scalar_kalman_update()`) remain dense — lower rate (8-10Hz), profile when expanding to 24 states.

### IVP-48 ESKF Health Tuning — COMPLETE (2026-02-20)

Fixed mNIS=124.99 death spiral: tilt R inflation (30-60°, ArduPilot `fuseEulerYaw`), hard reject >60°, 300σ gate (ArduPilot EKF3 match), public `reset_mag_heading()` for IVP-52 state machine. Per-sensor accept/reject counters, CLI health dashboard (`s` gate counters + P diags, `e` mA + zNIS). Q review: all values correct. 192/192 host tests. HW verified: mNIS 0.00–0.52, zero errors.

### P5c clang-tidy Function Decomposition — COMPLETE (2026-02-20)

24 function-size/CC warnings decomposed to zero across 4 batches. Pre-commit hook added for function-size + CC gate. Binary: text=114,144, BSS=96,524. HW verified: 60s soak, 0 errors, ESKF stable.

### GPS 57600 Baud + 10Hz — COMPLETE (2026-02-20)

`gps_uart_init()` negotiates 57600 baud and 10Hz. HW verified: ~127 GPS reads/10s, rxOvf=0, IMUerr=0.

### IVP-45 Mahony AHRS — COMPLETE (2026-02-19)

`mahony_ahrs.h/.cpp` + 14 host tests (187/187). Wired into `eskf_tick()` at 200Hz. `Mdiv` in both `e` and `s` CLI output. HW verified: 60s soak, movement tracking, zero sensor errors. Committed `a120e6f`. Standards audit complete (P5c, 2026-02-20).

### Foundational Features Gate — COMPLETE (2026-02-10)

All 5 prerequisites for ESKF completed: soak baseline (536K reads/0 errors), mag cal wizard (300 samples, 81% coverage), non-blocking USB (boot banner deferred), boot-time peripheral detection, unified 5-step calibration wizard. Qwiic chain order finding: GPS must be first in chain.

### Non-Blocking USB — COMPLETE (2026-02-10)

Replaced blocking `wait_for_usb_connection()` with non-blocking `stdio_init_all()`. Boot banner deferred to first terminal connect. Soak verified: 536K reads, 0 errors.

### Stage 4 GPS (IVP-46) — COMPLETE (2026-02-18)

9-step incremental plan fully executed and outdoor-validated. bNIS explosion from previous session was caused by missing P covariance reset in `set_origin()` (fixed in Step 3). UART FIFO overflow fixed with interrupt-driven ring buffer (Step 9). ESKF init NeoPixel (fast red blink) prevents user-error divergence from resetting while moving. 60s soak: bNIS 0.00–2.13, zero UNHEALTHY, G=Y. 173/173 host tests pass.

### ICM-20948 I2C Bypass Mode Migration — COMPLETE (2026-02-10)

Migrated from I2C master mode to bypass mode (`BYPASS_EN=1`). AK09916 reads directly at 0x0C. Removed ~120 lines of I2C master code (Bank 3, SLV0, master clock). Added mag read divider (100Hz), two-level device recovery, lazy mag re-init, GPS pause during mag cal. Council-approved (unanimous). HW verified: mag cal 300 samples, 72% coverage, RMS 0.878 uT with GPS on bus. Build tag: bypass-5.

### D3 main.cpp Refactoring + IVP Strip — COMPLETE (2026-02-09)

`main()` 992→65 lines, `core1_entry()` 367→15 lines. Tick-function dispatcher pattern. Clang-tidy audit (127 checks, 1,251 findings). IVP test code stripped: 3,623→1,073 lines (33 functions, ~2,550 lines removed). Binary 198,144→155,648 bytes (-21.4%). RC-1 and BM-6 deviations resolved. IVP scripts deleted.

### BSS Layout / Codegen Sensitivity — DISPROVED (2026-02-09)

**The "codegen sensitivity" hypothesis was false.** All prior evidence was contaminated by picotool `--force` bus corruption during rapid flash cycles (LL Entry 25).

**Definitive test (2026-02-09):** Three 6-minute soak tests via debug probe (SWD), all zero I2C errors:
1. Baseline (`prod-1`): 398,665 IMU reads, 0 errors
2. `constexpr` added to i2c_bus.cpp: 369,604 reads, 0 errors
3. `static volatile` added to i2c_bus.cpp (BSS layout change): 369,549 reads, 0 errors

**Root cause of all prior "codegen sensitivity" observations:** picotool `--force` reboot interrupts in-progress I2C transactions. Rapid flash cycles accumulate bus corruption that bus recovery can't clear. Debug probe halts both cores cleanly via SWD before flashing — no mid-transaction interruption.

**Impact:** No special BSS isolation, `__attribute__((section))`, separate `.cpp` files, or BUSCTRL investigation needed. Changes to any source file are safe when flashed via probe. See LL Entry 27.

*Previous deferred notes about `.map` diffs, BUSCTRL perf counters, and BSS isolation are obsoleted.*
