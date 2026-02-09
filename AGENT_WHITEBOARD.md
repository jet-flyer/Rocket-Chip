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

### BSS Layout Regression — UNSOLVED, DEFERRED

Adding new `std::atomic` static globals to `main.cpp` causes ~30-80% IMU I2C errors. `alignas(64)` and flag grouping both failed. Baseline `ivp32-33-1` is rock solid (68K+ reads, 0 errors). Extremely sensitive to code generation changes.

**Next steps (when Stage 5 ESKF needs new cross-core globals):** (1) Separate `.cpp` file for new globals, (2) `__attribute__((section))` BSS isolation, (3) `.map` + `objdump -d` diff, (4) BUSCTRL perf counters. Does not block current work.

---

### Protected File Updates Pending Approval

- `CODING_STANDARDS.md` — needs cross-reference to `standards/VENDOR_GUIDELINES.md` in Prior Art Research section
- `SEQLOCK_DESIGN.md` line 69 — comment says "reserved for IVP-33", now IVP-31 (cosmetic)

---

### ICM-20948 I2C Bypass Mode Migration

Council recommended migrating from I2C master mode to bypass mode (ArduPilot's approach) before IVP-36 (ESKF). Eliminates bank-switching race (LL Entry 21). Currently mitigated by disabling I2C master during calibration.

See: ArduPilot `AP_InertialSensor_Invensensev2.cpp`, ICM-20948 `INT_PIN_CFG` register `I2C_BYPASS_EN` bit.

---

### D3 main.cpp Refactoring + IVP Strip — COMPLETE

`main()` 992→65 lines, `core1_entry()` 367→15 lines. Tick-function dispatcher pattern. Clang-tidy audit (127 checks, 1,251 findings). IVP test code stripped: 3,623→1,073 lines (33 functions, ~2,550 lines removed). Binary 198,144→155,648 bytes (-21.4%). RC-1 and BM-6 deviations resolved. IVP scripts deleted.

---

### Missing Vendor Datasheets

| Document | Priority | Needed By |
|----------|----------|-----------|
| DPS310 datasheet (Infineon) | HIGH | IVP-36 (ESKF noise tuning) |
| RFM95W / SX1276 datasheet (Semtech) | MEDIUM | Stage 8 (telemetry) |

Source URLs in `standards/VENDOR_GUIDELINES.md` Datasheet Inventory section.

---

## Deferred Notes

*Items noted for future stages — not blocking, no action needed now.*

- **F' Evaluation:** Three Titan paths identified (A: STM32H7+F'/Zephyr, B: Pi Zero 2 W+F'/Linux, C: Hybrid). Research complete in `docs/decisions/TITAN_BOARD_ANALYSIS.md`. Decision deferred until Titan development begins.
- **FeatherWing UART GPS:** Adafruit 3133 (PA1616D) on hand. Eliminates I2C contention. New `gps_uart.cpp` driver needed. `g_gpsOnI2C` flag already in place. Blocked on user soldering.
- **u-blox GPS (Matek M8Q-5883):** UART + QMC5883L compass. UBX binary protocol. For production/flight builds, not current IVP.
- **ArduPilot LED Patterns:** Map NeoPixel to AP standard codes at IVP-46 (state machine). Known NeoPixel green-transition bug deferred to same IVP.
- **clang-tidy Integration:** LLVM installed, 127-check config active, first full audit complete (2026-02-09). **All code fully remediated** across 6 phases (P1-P5f, 1,251 total findings). IVP test code stripped — zero deferred findings. Pre-commit enforcement deferred to next cycle.

---

## Resolved

*Resolved items are erased. See LESSONS_LEARNED.md for historical debugging context.*
