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

## Tier 2: Moderate Fixes (Pending)

| Task | Rule | Status | Notes |
|------|------|--------|-------|
| B1: Multi-expression lines | JSF 42 / LOC-4.2 | Pending | 8 lines in main.cpp + ws2812_status.c |
| B2: Multi-variable declarations | JSF 152 | Pending | 12 declarations in main.cpp |
| B3: Remove `continue` | JSF 190 | Pending | 3 instances in i2c_bus.c + main.cpp |
| B4: Fix `int` loop counters | JSF 209 | Pending | 14 instances across multiple files |
| B5: Pointer style | JSF 62 | Pending | Mixed `float*` vs `Type *` in headers |
| B6: `-Werror`/`-Wpedantic` | P10-10 / LOC-1.2 | Pending | CMakeLists.txt + fix new warnings |

---

## Tier 3: Deviations (Pending in-depth review)

Each item to be reviewed individually with user before disposition.

| # | Rule | Finding | Disposition |
|---|------|---------|-------------|
| 15 | JSF 29/30/31 | 60+ `#define` for constants/macros | Pending review |
| 16 | JSF 26/28 | `#ifdef DEBUG`, `#ifdef __cplusplus` | Pending review |
| 17 | JSF 22/24 (SDK) | stdio.h, sleep_ms/sleep_us | Pending review |
| 18 | JSF 162 | 6 signed/unsigned casts | Pending review |
| 19 | P10-8 / LOC-4.3 | DBG_PRINT variadic macros | Pending review |
| 20 | JSF 8 / Env | C17+C++17 with 3 extensions | Pending review |
| 21 | LOC-2.5 | FIFO pop without validation | Pending review |
| 22 | Prior art | SDK refs, doc gaps | Pending review |

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
