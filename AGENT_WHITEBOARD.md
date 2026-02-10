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

### Non-Blocking USB — DEFERRED

**Goal:** Remove blocking `wait_for_usb_connection()` so firmware runs without terminal.

**What was tried (prod-13 through prod-16):** All 4 variants failed soak tests at 42-90s. However, all testing was done via **picotool rapid flash cycles** — now known to corrupt I2C bus (LL Entry 25). These results are contaminated and should be retested via debug probe.

**Next step:** Re-attempt non-blocking USB changes, testing exclusively via debug probe. The prod-13 through prod-16 failures may have been picotool artifacts, not code regressions.

**Plan files preserved:** `~/.claude/plans/silly-dreaming-wilkinson.md` (USB+recovery plan), `~/.claude/plans/phase-m-mag-cal.md` (Phase M plan).

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

### BSS Layout / Codegen Sensitivity — DISPROVED (2026-02-09)

**The "codegen sensitivity" hypothesis was false.** All prior evidence was contaminated by picotool `--force` bus corruption during rapid flash cycles (LL Entry 25).

**Definitive test (2026-02-09):** Three 6-minute soak tests via debug probe (SWD), all zero I2C errors:
1. Baseline (`prod-1`): 398,665 IMU reads, 0 errors
2. `constexpr` added to i2c_bus.cpp: 369,604 reads, 0 errors
3. `static volatile` added to i2c_bus.cpp (BSS layout change): 369,549 reads, 0 errors

**Root cause of all prior "codegen sensitivity" observations:** picotool `--force` reboot interrupts in-progress I2C transactions. Rapid flash cycles accumulate bus corruption that bus recovery can't clear. Debug probe halts both cores cleanly via SWD before flashing — no mid-transaction interruption.

**Impact:** No special BSS isolation, `__attribute__((section))`, separate `.cpp` files, or BUSCTRL investigation needed. Changes to any source file are safe when flashed via probe. See LL Entry 27.

*Previous deferred notes about `.map` diffs, BUSCTRL perf counters, and BSS isolation are obsoleted.*
