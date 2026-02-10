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
5. **Full calibration wizard** — Gyro, level, 6-pos accel, and mag cal should all be accessible through a unified CLI flow. Currently separate commands.

**Rationale (user directive):** Features like peripheral detection, calibration wizards, and non-blocking USB are not "future nice-to-haves" — they are core functionality that ESKF depends on. Mag cal provides the hard-iron/soft-iron corrections that the ESKF magnetometer measurement model needs. Non-blocking USB is required for any untethered operation. Get the foundation solid before adding fusion.

---

### Non-Blocking USB — RESOLVED (2026-02-10)

Replaced blocking `wait_for_usb_connection()` with non-blocking `stdio_init_all()`. Boot banner deferred to first terminal connect via `rc_os_print_boot_status` callback. Core 1 + watchdog start immediately. Soak verified: 536K reads, 0 errors. Prior prod-13 through prod-16 failures were picotool artifacts (LL Entry 25/27).

**Qwiic chain order finding:** PA1010D GPS must be first in chain (closest to board/power). At end of chain, probe detection is intermittent. Documented in `docs/hardware/HARDWARE.md`.

**Plan files:** `~/.claude/plans/phase-m-mag-cal.md` (Phase M plan, completed).

---

### Protected File Updates Pending Approval

- `CODING_STANDARDS.md` — needs cross-reference to `standards/VENDOR_GUIDELINES.md` in Prior Art Research section
- `SEQLOCK_DESIGN.md` line 69 — comment says "reserved for IVP-33", now IVP-31 (cosmetic)

---

### ICM-20948 I2C Bypass Mode Migration

Council recommended migrating from I2C master mode to bypass mode (ArduPilot's approach) before IVP-42 (ESKF Propagation). Eliminates bank-switching race (LL Entry 21). Currently mitigated by disabling I2C master during calibration.

See: ArduPilot `AP_InertialSensor_Invensensev2.cpp`, ICM-20948 `INT_PIN_CFG` register `I2C_BYPASS_EN` bit.

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
- **ArduPilot LED Patterns:** Map NeoPixel to AP standard codes at IVP-51 (state machine). Known NeoPixel green-transition bug deferred to same IVP.
- **clang-tidy Integration:** LLVM installed, 127-check config active, first full audit complete (2026-02-09). **All code fully remediated** across 6 phases (P1-P5f, 1,251 total findings). IVP test code stripped — zero deferred findings. Pre-commit enforcement deferred to next cycle.
- **Dynamic Peripheral Detection + OTA Drivers (Crowdfunding Goal):** Boot-time probe-first detection implemented (2026-02-10). Runtime hot-plug, driver registry, and OTA firmware downloads for unrecognized devices are stretch goals. Full architecture documented in SAD Section 13.2. Flipper Zero-style: plug in a sensor, RC identifies it, prompts for driver. WiFi/BT OTA for Core/Middle tiers.

---

## Resolved

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
