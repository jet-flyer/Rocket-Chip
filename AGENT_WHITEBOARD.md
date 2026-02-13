# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes

**Last reviewed by Nathan**: *[DATE]*

---

## Open Flags

### PRIORITY: Foundational Features Before ESKF

**Do NOT move to Stage 5 ESKF (IVP-39+) until these are complete:**

1. ~~**Soak baseline**~~ — **DONE** (2026-02-10). 536K IMU reads, 0 errors, 6 min with all 3 sensors (`nonblock-usb-3`).
2. ~~**Phase M: Magnetometer calibration**~~ — **DONE** (2026-02-10). All 4 commits merged (94dffad, d261b66, c84c38c). CLI wizard + Core 1 live apply + heading display.
3. ~~**Non-blocking USB init**~~ — **DONE** (2026-02-10, ec87703). Boot banner deferred to first terminal connect. Prior failures were picotool artifacts.
4. ~~**Boot-time peripheral detection**~~ — **DONE** (2026-02-10). Runtime hot-plug detection is a stretch goal, not a blocker.
5. ~~**Full calibration wizard**~~ — **DONE** (2026-02-10, wizard-12). 5-step wizard (gyro, level, baro, 6-pos accel, compass) with NeoPixel feedback. ENTER/x navigation, Core 1 sensor feeds. Fixed mag cal raw data (seqlock now carries raw + calibrated mag). All 5 steps HW verified including full mag cal (300 samples, 81% coverage, RMS 2.499 uT).

**Rationale (user directive):** Features like peripheral detection, calibration wizards, and non-blocking USB are not "future nice-to-haves" — they are core functionality that ESKF depends on. Mag cal provides the hard-iron/soft-iron corrections that the ESKF magnetometer measurement model needs. Non-blocking USB is required for any untethered operation. Get the foundation solid before adding fusion.

---

### Non-Blocking USB — RESOLVED (2026-02-10)

Replaced blocking `wait_for_usb_connection()` with non-blocking `stdio_init_all()`. Boot banner deferred to first terminal connect via `rc_os_print_boot_status` callback. Core 1 + watchdog start immediately. Soak verified: 536K reads, 0 errors. Prior prod-13 through prod-16 failures were picotool artifacts (LL Entry 25/27).

**Qwiic chain order finding:** PA1010D GPS must be first in chain (closest to board/power). At end of chain, probe detection is intermittent. Documented in `docs/hardware/HARDWARE.md`.

**Plan files:** `~/.claude/plans/phase-m-mag-cal.md` (Phase M plan, completed).

---

### Watchdog Recovery Policy — IVP-49 (New, Stage 6 Prerequisite)

**Added 2026-02-12.** IVP-30 watchdog mechanism is correct for ground/IDLE. But a full reboot mid-flight loses all ESKF state, pyro timers, and nav knowledge. IVP-49 added as first step of Stage 6 (before state machine IVP-50) to define the recovery policy: scratch register persistence, reboot counting with safe-mode lockout, ESKF failure backoff, and recovery boot path.

**False warning bug — FIXED (2026-02-12).** Root cause: both SDK functions are broken for our use case. `watchdog_caused_reboot()` gives false positives on SWD `monitor reset run` (reason register persists, `rom_get_last_boot_type()` returns `BOOT_TYPE_NORMAL`). `watchdog_enable_caused_reboot()` gives false negatives on real timeouts (bootrom overwrites scratch[4]). Fix: custom sentinel in scratch[0] (`kWatchdogSentinel = 0x52435754`), written before `watchdog_enable()`, checked and cleared at boot. Scratch[0] survives watchdog resets but is cleared by POR/SWD reset. HW verified: no warning on cold plug, boot button, or first SWD flash. Warning correctly appears on SWD reflash while watchdog is running (genuine timeout during bootrom — bootrom takes >5s, watchdog fires). This remaining case will be addressed with the broader IVP-49 watchdog recovery policy.

**IVP renumber:** All Stage 7-9 IVP numbers shifted +1 (old IVP-54→55 through IVP-68→69). Regression matrix updated. ArduPilot LED Patterns deferred note references IVP-51 → now IVP-52.

---

### Sparse FPFT Optimization — Deferred to Post-IVP-46

**Added 2026-02-12.** `predict()` currently uses dense FPFT (three 15×15 matrix multiplies). Benchmarked at ~496µs on target (IVP-42d, `6c84cd3`), vs <100µs gate target. Within cycle budget at 200Hz (~10%) but 5x over target. The sparse path exploits F_x block structure (many zero/identity blocks) to avoid the full O(N³) triple product — should bring it under 100µs.

**When:** After all measurement updates are wired with real sensor feeds (IVP-43 baro done, IVP-44 mag, IVP-46 GPS pending). Current benchmarks use injected data; real performance picture needs all feeds live. Natural slot: alongside or after IVP-46, before IVP-48 health pass.

**Also applies to:** `update_baro()` Joseph form has two dense 15×15 multiplies (~8Hz, less critical than 200Hz predict). Mag and GPS updates will add more. Profile full pipeline with all feeds before optimizing.

---

### Protected File Updates Pending Approval

- `CODING_STANDARDS.md` — needs cross-reference to `standards/VENDOR_GUIDELINES.md` in Prior Art Research section
- `SEQLOCK_DESIGN.md` line 69 — comment says "reserved for IVP-33", now IVP-31 (cosmetic)

---

### Missing Vendor Datasheets

| Document | Priority | Needed By |
|----------|----------|-----------|
| DPS310 datasheet (Infineon) | HIGH | IVP-41 (ESKF 1D baro KF) |
| RFM95W / SX1276 datasheet (Semtech) | MEDIUM | Stage 8 (telemetry) |

Source URLs in `standards/VENDOR_GUIDELINES.md` Datasheet Inventory section.

---

## Deferred Notes

*Items noted for future stages — not blocking, no action needed now.*

- **F' Evaluation:** Three Titan paths identified (A: STM32H7+F'/Zephyr, B: Pi Zero 2 W+F'/Linux, C: Hybrid). Research complete in `docs/decisions/TITAN_BOARD_ANALYSIS.md`. Decision deferred until Titan development begins.
- **FeatherWing UART GPS:** Adafruit 3133 (PA1616D) on hand. Eliminates I2C contention. New `gps_uart.cpp` driver needed. `g_gpsOnI2C` flag already in place. Blocked on user soldering.
- **u-blox GPS (Matek M8Q-5883):** UART + QMC5883L compass. UBX binary protocol. For production/flight builds, not current IVP.
- **ArduPilot LED Patterns:** Map NeoPixel to AP standard codes at IVP-52 (action executor). Known NeoPixel green-transition bug deferred to same IVP.
- **clang-tidy Integration:** LLVM installed, 127-check config active, first full audit complete (2026-02-09). **All code fully remediated** across 6 phases (P1-P5f, 1,251 total findings). IVP test code stripped — zero deferred findings. Pre-commit enforcement deferred to next cycle.
- **Dynamic Peripheral Detection + OTA Drivers (Crowdfunding Goal):** Boot-time probe-first detection implemented (2026-02-10). Runtime hot-plug, driver registry, and OTA firmware downloads for unrecognized devices are stretch goals. Full architecture documented in SAD Section 13.2. Flipper Zero-style: plug in a sensor, RC identifies it, prompts for driver. WiFi/BT OTA for Core/Middle tiers.
- **State-Aware ZUPT for State Machine (IVP-50):** Current ZUPT (IVP-44b) uses IMU-based stationarity detection with kSigmaZupt=0.5 m/s. When the state machine knows we're IDLE or ARMED (on pad), we can: (1) tighten R to ~0.1 m/s since stationarity is guaranteed, (2) skip the accel/gyro stationarity check entirely, (3) keep loose ZUPT as fallback for uncertain states (LANDED before confirmed). ArduPilot EKF3 `onGround` flag and PX4 ECL `vehicle_at_rest` both use vehicle state to override IMU-based detection. Natural integration point: `eskf_tick()` checks flight state and calls `update_zupt()` with tighter R when on pad.
- **Direct NOAA/IGRF WMM Table Generation:** Current WMM declination table is converted from ArduPilot's `AP_Declination/tables.cpp` (IGRF13 epoch). Long-term goal: generate table directly from NOAA WMM/IGRF coefficients (spherical harmonic expansion) to remove AP dependency. NOAA publishes WMM coefficients as `WMM.COF` file every 5 years (next: WMM2030). Python script to evaluate the model at grid points and output C++ table. Not blocking — current table is valid through ~2028-2029.
- **RC GCS: GPS-free 3D Flight Path Reconstruction (user request, 2026-02-13):** Post-processing feature for RC GCS. Uses raw IMU+baro+mag flight logs with forward-backward RTS smoother and known boundary conditions (launch point = origin, v=0 at ignition and landing, landing point from recovery GPS/manual entry). Reconstructs full 3D flight path without real-time GPS. Core tier ships without GPS — this makes 3D visualization viable for all tiers. Natural companion to IVP-44b ZUPT (stationary constraint) and IVP-46 (GPS when available). Reference: ArduPilot `tools/replay/`, PX4 `ecl/EKF/ekf_helper.cpp` smoother.

---

## Resolved

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
