# Advanced & Experimental Settings Tracking

**Purpose:** Track all items destined for advanced user configuration, experimental/research mode features, and hardware configuration options. Organized by category, not by when they were mentioned.

**Status:** Tracking document. No advanced settings menu implemented yet.

---

## Wizard UX Principle (Opt-In, Not Suggested)

The Mission Profile wizard's job is to get a new user — a Space Camp counselor, a 6th-grade student, a first-time hobbyist — to a flyable profile without showing them a single option they don't need. Anything in this document is **opt-in only**.

**Default wizard flow:** walks the user through the safe, essential profile settings. At the end, a single gate:

> **"Enable advanced settings? [y/N]"**

Answer `N` and the wizard exits with a complete, flyable profile. Answer `Y` and the advanced section opens.

**Inside the advanced section** — tiered warnings per setting:

- **Informational** — setting you probably haven't seen before; neutral explanation, safe to try.
- **"Here be dragons"** — settings that are genuinely dangerous, legally constrained (e.g., RF power limits, duty cycle), or easy to break the vehicle with. Explicit warning before the prompt; user must type through it, not just press Enter.

**Menu shape:**

- **Near-term:** a single flat checklist covering all advanced settings, as long as the count stays manageable. The User Guide is the reference for what each one does.
- **Later:** if the count grows past what fits in one screen, split into sub-menus by category.

**Not in scope for the wizard:**

- Discoverability lives in the User Guide, not the wizard prompts.
- Direct `.cfg` editing remains supported for users who already know the key names.

Any future feature added to this document inherits the opt-in principle by default. If a feature is load-bearing enough to belong in the default wizard flow, it does not belong in this document.

---

## 1. Research Mode (Council-Reviewed Concept)

Gated experimental features — each individually toggleable, all behind a Research Mode acknowledgment. Prevents accidental activation of unstable or risky features.

| Feature | Description | Source | Status |
|---------|-------------|--------|--------|
| Raw sensor logging (IVP-55) | SensorSnapshot per-sensor bitmask at native rate (up to 1kHz IMU) | council_data_logging.md | Deferred |
| FSK continuous bitstream (IVP-63) | SX1276 FSK mode for IRIG-heritage PCM telemetry | revised_ivp_stages.md | Placeholder |
| MCU overclocking | RP2350 clock boost beyond 150MHz | council_data_logging.md | Concept only |
| Disabled safety interlocks | Bypass confidence gate, lockouts for testing | council_data_logging.md | Concept only |
| Unlocked ESKF tuning | Runtime Q/R/gate threshold adjustment | council_data_logging.md | Concept only |
| Verbose logging mode | Innovation alphas, Q/R params, P-diagonals in log frames | AGENT_WHITEBOARD.md | Deferred |
| MP-configurable logged events | Per-event-type flags (pyro, abort, confidence, phase changes) | AGENT_WHITEBOARD.md | Deferred |
| Diagnostic log tier (`LOG_DIAGNOSTICS=1`) | Tier 2 payload: Mahony quaternion, gyro bias, ZUPT state, confidence gate state + ESKF↔Mahony delta, raw+calibrated accel, per-frame RSSI+SNR. Default OFF (Rocket profile). Default ON in new "Validation" profile used for Stage 17 static/dynamic tests. | Stage 17 IVP-135a | Planned |
| Runtime log rate (`LOG_RATE_HZ`) | Per-profile log rate selector: 10 / 50 / 100 / 400 Hz. Enables single-test-duration budget tuning (statics = 10 Hz long dwell; drop/pendulum = 400 Hz short burst). | Stage 17 IVP-135a | Planned |

## 2. Hardware Configuration

Options related to which hardware is connected and how it's configured.

| Feature | Description | Source | Status |
|---------|-------------|--------|--------|
| Dynamic peripheral detection | Boot-time probe-first (done). Runtime hot-plug, driver registry | SAD Section 13.2 | Probe done, hot-plug deferred |
| OTA driver download | Flipper Zero-style: plug in unknown sensor, download driver via WiFi/BT | AGENT_WHITEBOARD.md | Crowdfunding goal |
| Board type auto-detect | Feather vs Tiny2350 vs Fruit Jam (currently compile-time via board.h) | board.h / BSP | Compile-time only |
| Sensor bus selection | I2C vs SPI for IMU (SPI gives 1.2% vs 30% overhead at 1kHz) | CODING_STANDARDS.md | I2C only currently |
| GPS transport | UART (preferred) vs I2C (auto-detected at boot) | gps_uart.cpp / gps_pa1010d.cpp | Auto-detect done |
| u-blox GPS support | Matek M8Q-5883 with UBX binary protocol + QMC5883L compass | AGENT_WHITEBOARD.md | Future HW upgrade |

## 3. Mission Profile Runtime Settings

Settings that belong in the Mission Profile `.cfg` but aren't yet wired through the generator/struct.

| Setting | .cfg Field | In Struct? | Description |
|---------|-----------|------------|-------------|
| PIO drogue action | `DROGUE_TIMER_ACTION` | No | 0=disabled, 1=fire drogue, 2=fire main |
| PIO main action | `MAIN_TIMER_ACTION` | No | Same as above for main channel |
| Safe mode behavior | `SAFE_MODE_ACTION` | No | 0=degrade, 1=safe+radio recovery, 2=immediate deploy |
| Station output default | — | No | ANSI/CSV/MAVLink boot default (station_output_mode.h) |
| Radio mode profiles (IVP-64) | — | Partial | Full radio parameter customization without reflash |
| Runtime profile switching | — | No | Select profile from flash/SD at boot (Stage 12 prerequisite) |

## 4. Developer / Tuning Constants

Firmware constants that advanced users or developers might want to adjust. Currently compile-time only.

| Setting | Location | Value | Description |
|---------|----------|-------|-------------|
| `ESKF_USE_BIERMAN` | `eskf.h` (forced on) + CMake | 1 (always) | **Sole** ESKF **measurement** path: Bierman UD scalar updates (host + flight). Joseph dual path removed 2026-07 after host parity. Rationale: NASA/TP-2018-219822 + UD_BENCHMARK (Bierman ~43 µs vs Joseph ~81 µs). **Predict** is separate: codegen FPFT (not dense Thornton). Thornton is **shelved on RP2350 for cycle budget** (~30× denser O(N³) cost), not physically impossible — advantages + revisit triggers in `docs/benchmarks/UD_BENCHMARK_RESULTS.md`. CODE_TRIMMING 2026-07-03 §1 closed for Joseph. |
| `kMagInnovationGate` | eskf.h | 300σ | Mag heading innovation gate |
| `kAccelMinHealthyMag` | sensor_core1.cpp | 3.0 m/s² | IMU zero-output fault threshold |
| `kMaxHealthyVelocity` | eskf.h | 500 m/s | Velocity divergence sentinel |
| `kLinkLostMs` | ao_radio.cpp | 5000 ms | Radio link lost threshold |
| `kLinkGapMs` | ao_radio.cpp | 2000 ms | Radio link gap threshold |
| `kRMag3dPerAxis` | eskf.h (planned) | 0.36 µT² | 3-axis mag noise (AK09916) |
| Confidence gate thresholds | confidence_gate.h | various | VALIDATE — need flight data |
| Phase Q/R matrices | phase_qr.h / .cfg | various | VALIDATE — per-phase ESKF tuning |
| Innovation monitor params | innovation_monitor.h | window=20 | VALIDATE — sliding window config |

## 5. Future Advanced Menu Items (RCOS)

Runtime-accessible settings via a future advanced menu key.

| Item | Description | Dependency |
|------|-------------|------------|
| WMM table OTA update | Download WMM.COF via WiFi, regenerate tables (every 5 years) | ESP32-C6 (Stage 12B) |
| WMM distance-triggered refresh | Re-lookup WMM field when GPS moves >50km from last lookup. Off by default — GPS errors could corrupt field. Opt-in via profile flag. HAB/drone only. | HAB/drone profiles |
| Station GPS position push | FJ station sends GPS position to GPS-less vehicle (advanced command). Vehicle does WMM lookup + stores + sets ESKF origin. | Stage 3D (IVP-103) |
| CCSDS SDLS command authentication | Full 355.0-B-2 telecommand auth. Shared key pairing. ON for Rocket profile, OFF for Passive. | Stage 13 |
| Mag fusion mode toggle | Force heading-only vs 3D auto vs disabled | Stage 3D |
| ESKF state inspector | Full 24-state vector + P diagonals display | Developer tool |
| Debug log verbosity | Runtime DBG_PRINT level control | Currently compile-time DEBUG flag |
| Flight log rate toggle | 25Hz vs 50Hz without reflash | Currently in profile |
| Radio TX power adjust | Adjust dBm without reflash | Currently in profile |
| Boot banner verbosity | Compact (default) vs full HW detail | Currently 'b' key |
| Profile selection | Choose from multiple stored profiles at boot | Stage 12 (profile boot-load) |

---

## Implementation Path

1. **Stage 12:** Mission Profile boot-load from flash/SD enables runtime profile switching — DONE
2. **Stage 17 IVP-135a:** Diagnostic log tier + runtime log rate — first concrete opt-in flags land (`LOG_DIAGNOSTICS`, `LOG_RATE_HZ`). Establishes the `.cfg`-opt-in pattern for the rest of the document.
3. **Future (post-Stage-17, IVP number TBD):** **Flesh out Advanced Settings.** Placeholder for a dedicated IVP, well down the roadmap. Drives every row in this document to Implemented, explicitly cut, or re-scoped. Implements the wizard's `Enable advanced settings? [y/N]` gate and the flat-checklist UX described above. Sub-menu reorg happens here if the count has grown past one screen. User Guide gets a pass for discoverability of every flag that ships. Not scheduled — recorded so it doesn't get forgotten.
4. **Stage 18:** VALIDATE parameter tuning from flight data (was Stage 14 in old numbering)
5. **Post-launch:** OTA drivers, dynamic peripheral detection, WiFi config

---

*This document consolidates items from: council_data_logging.md, AGENT_WHITEBOARD.md, SAD Section 13.2, station_output_mode.h, UNIQUE_COMMENT_ITEMS.md, and session discussions.*
