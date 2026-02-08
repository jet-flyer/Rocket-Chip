# Audit Remediation Log

**Purpose:** Track exactly which lines were changed and why during standards audit remediation. Keeps CHANGELOG.md concise — reference this document for line-level detail.

**Source:** `standards/STANDARDS_AUDIT_2026-02-07.md` (44 PARTIAL/FAIL findings)

---

## Tier 1: Quick Fixes (Applied 2026-02-07)

### Fix #1 — JSF AV Rule 41: Line length >120 chars

| File | Line | Before | After |
|------|------|--------|-------|
| `src/main.cpp` | 2717 | Single 135-char printf format string | Split into 3 adjacent string literals (auto-concatenated) |

### Fix #2 — JSF AV Rule 59: Braces required for if/else/while/for

| File | Line | Before | After |
|------|------|--------|-------|
| `src/main.cpp` | 401 | `if (seq1 & 1u) continue;` | Added `{ }` around continue |
| `src/main.cpp` | 406 | `if (seq1 == seq2) return true;` | Added `{ }` around return |
| `src/main.cpp` | 482 | `if (!stdio_usb_connected()) return;` | Added `{ }` around return |
| `src/main.cpp` | 498 | `if (!stdio_usb_connected()) return;` | Added `{ }` around return |

### Fix #3 — JSF AV Rule 150 / Env 14: Hex constant suffix uppercase

| File | Line | Before | After |
|------|------|--------|-------|
| `src/main.cpp` | 457 | `~0x1Fu` | `~0x1FU` |
| `src/main.cpp` | 463 | `~0x1Fu` | `~0x1FU` |

### Fix #4 — JSF AV Rule 194: Switch must have default clause

| File | Line | Before | After |
|------|------|--------|-------|
| `src/cli/rc_os.c` | 623 (end of switch) | No default clause | Added `default: break;` |

### Fix #5 — JSF AV Rule 213: Explicit operator precedence

| File | Line | Before | After |
|------|------|--------|-------|
| `src/calibration/calibration_data.c` | 17 | `crc ^= (uint16_t)data[i] << 8;` | `crc ^= ((uint16_t)data[i] << 8);` |

### Fix #6 — JSF AV Rule 115 / P10-7 / LOC-3.7: Check return values

| File | Line | Before | After |
|------|------|--------|-------|
| `src/cli/rc_os.c` | 190 | `i2c_bus_reset();` (return ignored) | `if (!i2c_bus_reset()) { printf("[WARN]..."); }` |
| `src/cli/rc_os.c` | 443 | `i2c_bus_reset();` (return ignored) | `if (!i2c_bus_reset()) { printf("[WARN]..."); }` |

### Fix #7 — Debug D.3: Build iteration tag (LL Entry 2)

| File | Line | Before | After |
|------|------|--------|-------|
| `src/main.cpp` | 2373 | `printf("  Build: %s %s\n", __DATE__, __TIME__);` | Added `kBuildTag` constant; `printf("  Build: %s (%s %s)\n", kBuildTag, __DATE__, __TIME__);` |

### Fix #8 — Session D.7: PROJECT_STATUS.md

**Finding corrected:** `docs/PROJECT_STATUS.md` already exists (111 lines, current as of 2026-02-07). Audit finding was incorrect — the file is present but wasn't found by the audit's search path. Audit document to be updated.

---

## Tier 2: Moderate Fixes (Applied 2026-02-07)

### Fix B1 — JSF 42 / LOC-4.2: One expression per line

| File | Line(s) | Before | After |
|------|---------|--------|-------|
| `src/main.cpp` | 2872-2873 | `r = ...; g = ...; b = ...;` on same line | Split to one assignment per line |
| `src/drivers/ws2812_status.c` | 307-317 | HSV branches: `r1 = c; g1 = x; b1 = 0;` | Split to one assignment per line across all 6 branches |

### Fix B2 — JSF 152: One variable per declaration

| File | Location | Before | After |
|------|----------|--------|-------|
| `src/main.cpp` | struct `shared_sensor_data_t` | `float accel_x, accel_y, accel_z;` (11 multi-var lines) | Each field on its own line with type repeated |
| `src/main.cpp` | Core 1 IMU read | `float ax, ay, az, gx, gy, gz;` | 6 separate `float` declarations |
| `src/main.cpp` | Core 0 sensor status | `float gx, gy, gz, ax, ay, az;` | 6 separate `float` declarations |
| `src/main.cpp` | IVP-15 gyro gate | `float cx, cy, cz;` | 3 separate declarations |
| `src/main.cpp` | IVP-16 accel gate 2 | `float axSum = 0.0f, aySum = 0.0f, azSum = 0.0f;` | 3 separate initialized declarations |
| `src/main.cpp` | IVP-16 accel read | `float cx, cy, cz;` | 3 separate declarations |
| `src/main.cpp` | IVP-25 jitter gate | `uint32_t minDt = UINT32_MAX, maxDt = 0;` | 2 separate initialized declarations |

### Fix B3 — JSF 190: Remove `continue` (partial)

| File | Line | Before | After |
|------|------|--------|-------|
| `src/drivers/i2c_bus.c` | 91 | `if (addr == I2C_ADDR_PA1010D) continue;` | `if ((addr != I2C_ADDR_PA1010D) && i2c_bus_probe(addr)) {` — inverted condition |
| `src/main.cpp` | 419 | `continue; // Write in progress` | **Kept** — seqlock retry is the natural pattern; else-wrapping harms readability of lock-free code |
| `src/main.cpp` | 728 | `continue; // Restart loop timing` | **Kept** — wrapping 100+ line sensor section in else adds nesting without safety benefit |

### Fix B4 — JSF 209: Fixed-width loop counters

| File | Line | Before | After |
|------|------|--------|-------|
| `src/main.cpp` | seqlock read | `for (int i = 0; i < 3; ...)` | `for (uint8_t i = 0; i < 3; ...)` |
| `src/main.cpp` | IVP-14 gate 4 | `for (int i = 0; i < 10; ...)` | `for (uint8_t i = 0; i < 10; ...)` |
| `src/main.cpp` | IVP-15 gate 1 | `for (int i = 0; i < 10; ...)` | `for (uint8_t i = 0; i < 10; ...)` |
| `src/main.cpp` | IVP-16 gate 2 | `for (int i = 0; i < 10; ...)` | `for (uint8_t i = 0; i < 10; ...)` |
| `src/calibration/calibration_data.c` | CRC16 inner | `for (int j = 0; j < 8; ...)` | `for (uint8_t j = 0; j < 8; ...)` |
| `src/drivers/i2c_bus.c` | bus recovery | `for (int i = 0; i < 9; ...)` | `for (uint8_t i = 0; i < 9; ...)` |
| `src/drivers/icm20948.c` | WHO_AM_I retry | `for (int tries = 0; tries < 10; ...)` | `for (uint8_t tries = 0; tries < 10; ...)` |
| `src/drivers/icm20948.c` | mag init retry | `for (int mag_attempt = 0; mag_attempt < 3; ...)` | `for (uint8_t mag_attempt = 0; mag_attempt < 3; ...)` |
| `src/drivers/gps_pa1010d.c` | NMEA read | `int ret`, `int len`, `for (int i ...)` | `int32_t ret`, `int32_t len`, `for (int32_t i ...)` |

### Fix B5 — JSF 62: Pointer `*` with type

| File | Line | Before | After |
|------|------|--------|-------|
| `src/main.cpp` | read_accel_for_cal | `float *ax, float *ay, ...` | `float* ax, float* ay, ...` |
| `src/main.cpp` | kBuildTag | `const char *kBuildTag` | `const char* kBuildTag` |
| `src/cli/rc_os.h` | typedef | `float *ax, float *ay, ...` | `float* ax, float* ay, ...` |
| `src/calibration/calibration_manager.h` | accel_read_fn | `float *ax, float *ay, ...` | `float* ax, float* ay, ...` |
| `src/calibration/calibration_manager.h` | _with functions | `const calibration_store_t *cal` | `const calibration_store_t* cal` |
| `src/calibration/calibration_manager.c` | _with functions | `const calibration_store_t *cal` | `const calibration_store_t* cal` |

### Fix B6 — P10-10 / LOC-1.2: `-Werror` and `-Wpedantic`

| File | Change |
|------|--------|
| `CMakeLists.txt` | Added `-Werror` globally; `-Wpedantic` via `set_source_files_properties` for project sources only (SDK sources excluded — SDK has `void*` to fn-ptr casts that violate strict pedantic) |
| `src/drivers/ws2812_status.h` | Color macros: added `#ifdef __cplusplus` / `#else` to use C++ aggregate init `Type{...}` vs C99 compound literals `(Type){...}` |

---

## Tier 3: Deviations (Pending in-depth review)

Each item to be reviewed individually with user before disposition.

| # | Rule | Finding | Disposition |
|---|------|---------|-------------|
| 15 | JSF 29/30/31 | 60+ `#define` for constants/macros | **Deferred** — pending .c → .cpp conversion |
| 16 | JSF 26/28 | `#ifdef DEBUG`, `#ifdef __cplusplus` | **Partially fixed** — C++20 `constexpr bool` + `if constexpr` eliminates `#ifdef DEBUG` in C++ path. `#ifdef __cplusplus` deferred to .c→.cpp conversion |
| 17 | JSF 22/24 (SDK) | stdio.h, sleep_ms/sleep_us | **Mitigated** — see Fix C17 below |
| 18 | JSF 162 | 6 signed/unsigned casts | **Accepted** — all intentional at SDK/hardware type boundaries. IMU casts match ArduPilot pattern. Add inline comments per JSF 162 |
| 19 | P10-8 / LOC-4.3 | DBG_PRINT variadic macros | **Partially fixed** — C++ path uses variadic templates. C fallback retains macros pending conversion |
| 20 | JSF 8 / Env | C17+C++20 with 3 extensions | **Fixed** — upgraded from C++17 to C++20. Extensions remain justified |
| 21 | LOC-2.5 | FIFO pop without validation | **Accepted** — IVP-22 exercise-only test code (main.cpp:565). FIFO reserved by multicore_lockout. Stripped during D3 refactoring |
| 22 | Prior art | SDK refs, doc gaps | **Pass** — all drivers reference datasheets/libraries. i2c_bus.h is original Pico SDK wrapper, no external ref to cite |

### Fix C16 — JSF 26/28: C++20 debug macro refactoring

| File | Change |
|------|--------|
| `CMakeLists.txt` | `CMAKE_CXX_STANDARD 17` → `20` |
| `include/rocketchip/config.h` | C++ path: `#ifdef DEBUG` now sets `inline constexpr bool kDebugEnabled`. Template functions `dbg_print()`, `dbg_error()`, `dbg_state()` use `if constexpr` for zero-overhead toggling. Compatibility macros map `DBG_PRINT` → `dbg_print()` etc. C fallback retained for .c files |
| `src/main.cpp` | Fault handler delay loops: `volatile int32_t d++` → `int32_t d` with `__asm volatile("")` barrier (C++20 deprecates `++` on volatile) |

### Fix C17 — JSF 22/24: stdio.h usage and SDK timing functions

**Rule 22:** `<stdio.h>` shall not be used — format string UB, unbounded timing, hidden heap.
**Rule 24:** `abort`, `exit`, `getenv`, `system` shall not be used — uncontrolled termination.

**Analysis (428 printf, 3 snprintf, 40 sleep_ms/us, 4 getchar calls):**

| Category | Count | Files | Flight Path? | Disposition |
|----------|-------|-------|-------------|-------------|
| `printf` | 428 | main.cpp (270), rc_os.c (140), i2c_bus.c (15), gps_pa1010d.c (3) | **No** — all CLI, IVP test, or diagnostics | Runtime lockout: disabled when state != IDLE |
| `snprintf` | 3 | gps_pa1010d.c (NMEA command formatting) | **Yes** — unavoidable for NMEA protocol | Mitigated: bounded by `sizeof()`, constant format strings, MISRA-C 2012 accepted safe subset |
| `getchar_timeout_us` | 4 | main.cpp (CLI input) | **No** — CLI only | Runtime lockout with printf |
| `sleep_ms`/`sleep_us` | 40 | 5 files | N/A | **Reclassified** — these are `pico/time.h` hardware timer wrappers, NOT `<stdlib.h>` POSIX functions. Rule 24 does not apply |

**Mitigation plan (runtime lockout — same binary, not compile-time flag):**
- Flight state machine (IDLE → ARMED → BOOST → ...) controls stdio access
- When state != IDLE: no printf, no getchar, no USB reads
- Extends existing `rc_os_i2c_scan_allowed` pattern to all ground-only code
- GPS `snprintf` exempted — bounded, constant-format, MISRA-compliant safe subset
- Implementation deferred to ARM state machine integration (post-Stage 3)

**GPS snprintf detail (3 calls in `gps_pa1010d.c`):**
```c
int len = snprintf(sentence, sizeof(sentence) - 5, "$%s*", cmd);       // line 191
snprintf(sentence + len, 5, "%02X\r\n", cs);                           // line 198
snprintf(cmd, sizeof(cmd), "PMTK220,%u", interval_ms);                 // line 219
```
NMEA protocol requires formatted ASCII with hex checksums (`$PMTK220,1000*1F\r\n`).

**Mitigation options (ranked by effort):**
1. **Current:** Bounded `snprintf` with `sizeof()` + constant format strings — documented MISRA deviation (FreeRTOS pattern: "utility code, not core")
2. **Near-term:** Replace with custom `uint32_to_hex()`/`uint32_to_dec()` formatters (~50 LOC, zero stdio dependency) — standard DO-178C pattern
3. **Hardware upgrade:** Switch to u-blox GPS with UBX binary protocol (ArduPilot's approach) — eliminates NMEA string formatting entirely
4. **Alternative:** Standalone printf replacement (mpaland/printf or nanoprintf — no stdio.h, no malloc, ~600 LOC)

### Fix C20 — JSF 8: C++20 upgrade

| File | Change |
|------|--------|
| `CMakeLists.txt` | `set(CMAKE_CXX_STANDARD 20)` — GCC 14.2.1 has substantial C++20 support. SDK requires C++11 minimum, no conflicts |

---

## Tier 4: Architectural (Pending task breakdown)

| Task | Rule | Status | Notes |
|------|------|--------|-------|
| D1: RC_ASSERT() macro | P10-5 / LOC-3.1 | Pending | Infrastructure prerequisite |
| D2: goto/label cleanup | JSF 188/189 / P10-1 | Pending | Evaluate during RC_OS redesign |
| D3: main.cpp refactoring | JSF 1 / P10-4 / LOC-4.1 | Pending | ~950-line main() → modules |
| D4: while(true) documentation | P10-2 / LOC-2.2 | Pending | Document bare-metal deviation |
| D5: IVP test globals | P10-6 / LOC-3.6 | Pending | Depends on D3 |

---

## Build Verification Log

| Date | After Fix | Build Result |
|------|-----------|-------------|
| 2026-02-07 | Tier 1 (#1-7) complete | PASS — 4/4 files rebuilt, 0 warnings |
| 2026-02-07 | Tier 2 (B1-B6) complete | PASS — 103/103 clean build, -Werror -Wpedantic enforced, 0 warnings |
| 2026-02-07 | Tier 3 (C16, C20): C++20 upgrade + debug refactor | PASS — `-std=gnu++20`, 0 warnings, volatile loop fix applied |
