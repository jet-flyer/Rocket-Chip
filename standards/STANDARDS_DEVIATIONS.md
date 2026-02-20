# Standards Deviations Log

**Purpose**: Track deviations from project coding standards (JSF AV C++, DEBUG_OUTPUT.md, CODING_STANDARDS.md) with severity, remediation difficulty, and rationale.

**Last Updated**: 2026-02-09 (RC-1, BM-6 resolved: IVP test code stripped)

---

## Severity Levels

| Level | Description |
|-------|-------------|
| **Critical** | Must fix before flight code; safety or reliability impact |
| **High** | Should fix before production; may cause subtle issues |
| **Medium** | Fix when touching affected code; technical debt |
| **Low** | Acceptable for test/debug code; cosmetic or stylistic |
| **Accepted** | Intentional deviation with documented rationale |

## Difficulty Levels

| Level | Description |
|-------|-------------|
| **Trivial** | Simple search-and-replace or single-line change |
| **Easy** | Localized changes, no API impact |
| **Moderate** | Multiple files or requires testing |
| **Hard** | Architectural changes or external dependencies |

---

## Active Deviations

### ArduPilot Algorithm Deviations (IVP-17: 6-Position Accel Calibration)

These are implementation deviations from ArduPilot's `AccelCalibrator` class. The core math (Jacobian formulas, residual computation, parameter update) is identical.

| ID | Location | ArduPilot Approach | Our Approach | Severity | Difficulty | Rationale |
|----|----------|-------------------|--------------|----------|-----------|-----------|
| AP-1 | `calibration_manager.cpp` | Heap-allocated `std::vector` for samples | Static array `g_6pos_samples[300][3]` (3.6KB) | Accepted | N/A | LL Entry 1: no heap after init; RP2350 has 264KB RAM |
| AP-2 | `calibration_manager.cpp` | `mat_inverseN()` with LU decomposition | Gaussian elimination with partial pivoting | Accepted | N/A | Simpler, no separate L/U/P arrays; adequate for 9x9 |
| AP-3 | `rc_os.cpp` | Progress callback + async state machine | Blocking function with sensor read callback | Accepted | N/A | Bare-metal CLI doesn't need async; matches existing `cmd_reset_cal()` pattern |
| AP-4 | `rc_os.cpp` | Trusts user orientation (implicit) | Explicit orientation confirmation per position | Accepted | N/A | Catches positioning errors immediately vs failing at fit time |

### Bare-Metal Loop Deviations (P10-2 / LOC-2.2)

P10 Rule 2 requires all loops to have a fixed upper bound. Bare-metal main loops are inherently unbounded — this is universal across embedded systems including JPL's own flight software.

| ID | Location | Rule | Severity | Rationale |
|----|----------|------|----------|-----------|
| BM-1 | `main.cpp` main() | P10-2 | Accepted | Core 0 application loop. Watchdog timer (IVP-30, 5s timeout) provides the safety net. Loop runs indefinitely by design — bare-metal has no OS to return to |
| BM-2 | `main.cpp` core1_entry() wait loop | P10-2 | Accepted | Core 1 waits for sensor phase signal from Core 0. Bounded by boot sequence timing |
| BM-3 | `main.cpp` core1_sensor_loop() | P10-2 | Accepted | Core 1 sensor polling loop. Watchdog-protected. Runs for device lifetime |
| BM-4 | `main.cpp` memmanage_fault_handler() | P10-2 | Accepted | Unrecoverable fault handler — intentional halt-forever with LED blink pattern. Interrupts disabled, no recovery possible |
| BM-5 | `main.cpp` mpu_setup_stack_guard() | P10-2 | Accepted | MPU config failure — intentional halt. Same rationale as BM-4 |
**Mitigation:** Hardware watchdog (5s timeout) ensures no main loop can hang silently. Fault handlers (BM-4, BM-5) are unrecoverable by design — the LED blink pattern signals the failure mode. All bare-metal embedded systems (FreeRTOS idle task, ChibiOS main thread, Zephyr main loop) use the same pattern.

### stdio.h Usage Deviation (JSF 22/24)

JSF AV Rule 22 prohibits `<stdio.h>`. Our usage is mitigated by code classification and runtime lockout.

| ID | Location | Category | Severity | Rationale |
|----|----------|----------|----------|-----------|
| IO-1 | main.cpp, rc_os.cpp, i2c_bus.cpp | 212 printf + 6 getchar_timeout_us | Accepted | All Ground classification. Runtime lockout when state != IDLE (same binary principle). Zero stdio in flight-critical sensor drivers |
| IO-2 | gps_pa1010d.cpp, gps_uart.cpp | 6 snprintf (3 per file) | Accepted | Flight-Critical — bounded by `sizeof()`, constant format strings, MISRA-C 2012 accepted safe subset. Migration path: custom formatters or u-blox UBX binary protocol. See AUDIT_REMEDIATION.md Fix C17 |

---

## Resolved

### RC-1: Intentional Recursion in IVP-29 Test (P10-1)

**Fixed 2026-02-09.** `ivp_test_key_handler()` removed with all IVP test code. No recursion remaining in codebase.

### BM-6: IVP-21 Spinlock Soak Unbounded Loop (P10-2)

**Fixed 2026-02-09.** `core1_spinlock_soak()` removed with all IVP test code. No unbounded test loops remaining.

### PP-1: 60+ `#define` for Constants/Macros (JSF 29/30/31)

**Fixed 2026-02-08.** All 9 `.c` source files renamed to `.cpp` (C++20). All constant `#define` macros converted to `constexpr` with `k` prefix naming. ICM-20948 register defines (64) restructured into bank-scoped namespaces (`bank0::`, `bank2::`, `bank3::`, `bit::`, `ak09916::`). Remaining `#define` are: include guards, feature flags (`ROCKETCHIP_TIER_*`), `I2C_BUS_INSTANCE` (SDK macro), lwGPS config, and `DBG_*` compatibility macros wrapping C++20 template functions. Binary size unchanged (constexpr produces identical codegen to `#define`).

### GT-1: goto-for-Cleanup in cmd_accel_6pos_cal() (JSF 188/189)

**Fixed 2026-02-07.** Extracted calibration body into `cmd_accel_6pos_cal_inner()` helper function. All 6 `goto cal_cleanup` replaced with `return`. Wrapper function guarantees pre/post hook execution (I2C master disable/re-enable). Zero goto, zero labels remaining in codebase.

*Previous deviations archived with `AP_FreeRTOS` branch.*

---

## Review Schedule

- **Before Flight Code**: Review all Low severity items in production paths
- **Quarterly**: Audit new code for standards compliance

---

## Notes

1. Test code (`tests/`) has relaxed standards enforcement to enable rapid validation
2. Production code (`src/`) must strictly follow all standards
3. New deviations should be logged here before merging to main
