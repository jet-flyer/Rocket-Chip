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

### I2C Bus Recovery + Non-Blocking USB — UNSOLVED, DEFERRED

**Goal:** Remove blocking `wait_for_usb_connection()` so firmware runs without terminal; improve I2C bus recovery robustness.

**Deep research findings (2026-02-09):**
- Pico SDK has NO I2C bus recovery function. `i2c_init()` resets peripheral state machine via RESETS register but does nothing for external bus (slave holding SDA low). Custom recovery is the correct approach (I2C spec NXP UM10204 Section 3.1.16). Linux kernel, ArduPilot, Zephyr all implement custom recovery.
- The original "BSS sensitivity" hypothesis was partially confirmed as **I2C bus recovery timing sensitivity** — `gpio_set_function(GPIO_FUNC_I2C)` needs settling time before peripheral starts transactions.

**What was tried (prod-13 through prod-16):**
- prod-13: Robust recovery (retry loop, settling delay, `i2c_deinit()` before bit-bang) + non-blocking USB — soak failed at ~60s
- prod-14: Changed Core 1 error path from `i2c_bus_reset()` to lightweight `i2c_init()` — soak failed at ~90s, same cascade pattern
- prod-15: Always clock all 9 pulses (match old behavior) — soak failed at ~42s
- prod-16: Reverted i2c_bus.cpp entirely, kept only main.cpp USB changes — not tested (user requested full revert)

**Key observation:** ANY change to i2c_bus.cpp (even adding constants that don't affect runtime code paths) triggers the codegen sensitivity. The ~60-90s soak failure is a new manifestation — different from BSS sensitivity (immediate failure) and from picotool corruption (immediate on cold boot). Errors cascade: IMU first, then baro and GPS follow.

**Reverted to:** 94dffad (pre-USB-changes). 6de6245 reverted. stash@{0} has Phase M Commit 3 CLI changes.

**Next steps (same as before + new):** (1) Separate `.cpp` file for new globals, (2) `__attribute__((section))` BSS isolation, (3) `.map` + `objdump -d` diff between working and failing binaries, (4) BUSCTRL perf counters, (5) Try USB changes in isolation without ANY i2c_bus.cpp modifications. Does not block Phase M Commits 3+4 (stash@{0}).

**Plan files preserved:** `~/.claude/plans/silly-dreaming-wilkinson.md` (USB+recovery plan), `~/.claude/plans/phase-m-mag-cal.md` (Phase M plan).

---

### BSS Layout / Codegen Sensitivity — UNSOLVED, DEFERRED

Adding new `std::atomic` static globals to `main.cpp` causes ~30-80% IMU I2C errors. `alignas(64)` and flag grouping both failed. Baseline `ivp32-33-1` is rock solid (68K+ reads, 0 errors). Extremely sensitive to code generation changes — even changes to i2c_bus.cpp that don't affect runtime code paths can trigger runtime I2C degradation after 40-90 seconds.

**Next steps (when Stage 5 ESKF needs new cross-core globals):** (1) Separate `.cpp` file for new globals, (2) `__attribute__((section))` BSS isolation, (3) `.map` + `objdump -d` diff, (4) BUSCTRL perf counters. Does not block current work.

---

### Protected File Updates Pending Approval

- `CODING_STANDARDS.md` — needs cross-reference to `standards/VENDOR_GUIDELINES.md` in Prior Art Research section
- `SEQLOCK_DESIGN.md` line 69 — comment says "reserved for IVP-33", now IVP-31 (cosmetic)

---

### ICM-20948 I2C Bypass Mode Migration

Council recommended migrating from I2C master mode to bypass mode (ArduPilot's approach) before IVP-42 (ESKF Propagation). Eliminates bank-switching race (LL Entry 21). Currently mitigated by disabling I2C master during calibration.

See: ArduPilot `AP_InertialSensor_Invensensev2.cpp`, ICM-20948 `INT_PIN_CFG` register `I2C_BYPASS_EN` bit.

---

### D3 main.cpp Refactoring + IVP Strip — COMPLETE

`main()` 992→65 lines, `core1_entry()` 367→15 lines. Tick-function dispatcher pattern. Clang-tidy audit (127 checks, 1,251 findings). IVP test code stripped: 3,623→1,073 lines (33 functions, ~2,550 lines removed). Binary 198,144→155,648 bytes (-21.4%). RC-1 and BM-6 deviations resolved. IVP scripts deleted.

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

---

## Resolved

*Resolved items are erased. See LESSONS_LEARNED.md for historical debugging context.*
