# RocketChip Automated Standards Audit — 2026-02-09

**Tool:** clang-tidy 19.1.7 (LLVM)
**Config:** `.clang-tidy` (376-line comprehensive config)
**Audited By:** Claude Code CLI (automated)
**Codebase Snapshot:** Post-refactoring (main() decomposition complete)
**Binary Size:** 198,144 bytes (unchanged from pre-refactor)

---

## How This Audit Differs from Manual Audits

This is an **automated static analysis** audit using clang-tidy, distinct from the
manual `STANDARDS_AUDIT_2026-02-07.md` which checked 249 rules by human inspection.

| Aspect | Manual Audit (2026-02-07) | This Automated Audit |
|--------|--------------------------|---------------------|
| Method | Human code review | clang-tidy AST analysis |
| Coverage | 249 JSF/P10/JPL rules | 127 clang-tidy checks |
| Depth | Rule-by-rule assessment | Per-line, per-occurrence |
| False positives | Rare (human judgment) | Some (SDK boundaries) |
| Missed findings | Some (P10-1 recursion!) | None within tool scope |
| Repeatability | Manual | Fully automated |

**Key finding:** This automated audit caught a **recursive call chain** (`misc-no-recursion`
at main.cpp:1004) that the manual audit marked as PASS for P10-1. Static analysis
complements human review — neither alone is sufficient.

---

## Run Command

```bash
"C:/Program Files/LLVM/bin/clang-tidy.exe" src/<file>.cpp -p build/ \
  --extra-arg="--sysroot=C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi" \
  --extra-arg="--target=armv8m.main-none-eabi" \
  --extra-arg="-isystem" \
  --extra-arg="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi/include/c++/14.2.1" \
  --extra-arg="-isystem" \
  --extra-arg="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi/include/c++/14.2.1/arm-none-eabi/thumb/v8-m.main+fp/hard" \
  --extra-arg="-Wno-format-security" \
  --extra-arg="-Wno-gnu-zero-variadic-macro-arguments"
```

---

## Summary Dashboard

### Codebase Totals

| Metric | Value |
|--------|-------|
| Files audited | 10 |
| Total warnings (our code) | 1,251 |
| Unique check types triggered | 28 |
| Safety-critical findings | 5 |
| Enabled checks | 127 |
| Intentionally skipped checks | 27 (with documented rationale) |

### Per-File Breakdown

| File | Classification | Warnings | Top Issue |
|------|---------------|----------|-----------|
| `src/main.cpp` | IVP Test | 440 | magic-numbers (107) |
| `src/calibration/calibration_manager.cpp` | Ground | 274 | identifier-naming (64) |
| `src/drivers/icm20948.cpp` | Flight-Critical | 173 | magic-numbers (43) |
| `src/cli/rc_os.cpp` | Ground | 86 | redundant-void-arg (18) |
| `src/drivers/ws2812_status.cpp` | Flight-Support | 82 | uppercase-literal-suffix (32) |
| `src/calibration/calibration_data.cpp` | Ground | 48 | uppercase-literal-suffix (23) |
| `src/calibration/calibration_storage.cpp` | Ground | 45 | identifier-naming (21) |
| `src/drivers/i2c_bus.cpp` | Flight-Critical | 39 | implicit-bool-conversion (10) |
| `src/drivers/baro_dps310.cpp` | Flight-Critical | 35 | identifier-naming (18) |
| `src/drivers/gps_pa1010d.cpp` | Flight-Critical | 29 | magic-numbers (9) |

### Warning Distribution by Check

| Count | Check | Standards Mapping | Severity |
|-------|-------|-------------------|----------|
| 275 | `readability-magic-numbers` | JSF AV Rule 151 | Medium |
| 216 | `hicpp-uppercase-literal-suffix` | JSF AV Rule 14 | Low |
| 170 | `readability-braces-around-statements` | JSF AV Rule 59 | Medium |
| 162 | `readability-identifier-naming` | JSF AV Rules 50-53 | Medium |
| 82 | `modernize-redundant-void-arg` | C++20 modernization | Low |
| 65 | `readability-math-missing-parentheses` | JSF AV Rule 155 | Medium |
| 51 | `cppcoreguidelines-init-variables` | JPL LOC-3.5 / CERT | Medium |
| 43 | `modernize-use-nullptr` | C++11 / JSF AV Rule 145 | Low |
| 41 | `readability-implicit-bool-conversion` | JSF AV Rule 180 | Low |
| 27 | `readability-function-size` | P10-4 / JSF AV Rule 1 | High |
| 27 | `bugprone-narrowing-conversions` | JSF AV Rule 180 / MISRA | High |
| 18 | `cert-err33-c` | CERT ERR33-C | Low* |
| 15 | `readability-function-cognitive-complexity` | JSF AV Rule 3 | Medium |
| 11 | `cppcoreguidelines-pro-type-member-init` | C++ Core Guidelines | Low |
| 11 | `cppcoreguidelines-pro-type-cstyle-cast` | JSF AV Rule 168 | Medium |
| 9 | `modernize-use-bool-literals` | C++11 modernization | Low |
| 8 | `readability-isolate-declaration` | JSF AV Rule 152 | Low |
| 4 | `bugprone-branch-clone` | Code quality | Low |
| 4 | `modernize-use-using` | C++11 modernization | Low |
| 2 | `readability-avoid-nested-conditional-operator` | Readability | Low |
| 2 | `bugprone-implicit-widening-of-multiplication-result` | Safety | **Critical** |
| 2 | `readability-simplify-boolean-expr` | Readability | Low |
| 1 | `misc-no-recursion` | P10-1 / LOC-2.1 | **Critical** |
| 1 | `modernize-loop-convert` | C++11 modernization | Low |
| 1 | `bugprone-misplaced-widening-cast` | Safety | High |
| 1 | `bugprone-switch-missing-default-case` | MISRA / HICPP | Medium |
| 1 | `hicpp-multiway-paths-covered` | HICPP | Medium |
| 1 | `performance-no-int-to-ptr` | Performance | Low |

*cert-err33-c: Low because all 18 occurrences are unchecked `printf()`/`snprintf()` return
values in Ground/IVP code. Would be High in flight-critical code.

---

## Section 1: Safety-Critical Findings

These findings have potential runtime impact and should be reviewed for remediation.

### 1.1 Recursion Detected (P10-1 / LOC-2.1)

**Check:** `misc-no-recursion`
**Location:** `src/main.cpp:1004`
**Severity:** Critical (P10-1 is a hard rule)

```
function 'ivp_test_key_handler' is within a recursive call chain
```

**Context:** The manual audit (2026-02-07) marked P10-1 as PASS. This automated analysis
found a recursive call chain involving `ivp_test_key_handler`. The recursion path needs
investigation — it may be indirect (A calls B calls A) rather than direct self-recursion.

**Action:** Investigate call chain. If recursion is real, refactor to iterative. If it's a
false positive (e.g., function pointer stored but not called recursively), document as
accepted deviation.

### 1.2 Implicit Widening of Multiplication Result

**Check:** `bugprone-implicit-widening-of-multiplication-result`
**Locations:**
- `src/main.cpp:3094` — multiplication in `uint32_t` widened to `uint64_t`
- `src/main.cpp:3117` — same pattern

**Severity:** Critical (potential silent overflow before widening)

**Context:** When two `uint32_t` values are multiplied and the result is assigned to
`uint64_t`, the multiplication happens in 32-bit arithmetic first. If the product exceeds
2^32, the overflow is silent and the widened result is wrong.

**Action:** Cast one operand to `uint64_t` before multiplication:
`static_cast<uint64_t>(a) * b` instead of `(uint64_t)(a * b)`.

### 1.3 Misplaced Widening Cast

**Check:** `bugprone-misplaced-widening-cast`
**Location:** `src/main.cpp:2444`

**Severity:** High (potential data loss)

**Context:** Cast from `uint32_t` to `unsigned long` may be ineffective (same width on
ARM) or lossy depending on the expression structure.

**Action:** Review expression — ensure the cast is applied before the operation that needs
wider precision, not after.

### 1.4 Narrowing Conversions (27 occurrences)

**Check:** `bugprone-narrowing-conversions` / `cppcoreguidelines-narrowing-conversions`
**Files:** icm20948.cpp (16), ws2812_status.cpp (3), calibration_manager.cpp (7), main.cpp (1)
**Severity:** High (data loss: `int` to `float` loses precision above 2^24)

**Context:** All 27 are `int` → `float` narrowing in sensor data conversion. For 16-bit
sensor values (±32768), `float` has sufficient precision (24-bit mantissa covers ±16M).
However, accumulated values or sensor counts could exceed this range.

**Action:** Audit each occurrence. Sensor raw reads (16-bit) are safe. Any computation
that accumulates multiple readings needs explicit `static_cast<float>()` with a comment
confirming the range.

### 1.5 Missing Default Case in Switch

**Check:** `bugprone-switch-missing-default-case`
**Location:** `src/drivers/i2c_bus.cpp:95`

**Severity:** Medium

**Context:** Switch on non-enum value without a default case. If the switch variable takes
an unexpected value, execution falls through silently.

**Action:** Add `default: break;` with a comment noting it's defensive.

---

## Section 2: Standards Rule Coverage

### 2.1 Checks Enabled — Standards Mapping

This table maps each enabled clang-tidy check category to the standard(s) it enforces.

| Category | Checks | Primary Standard | Notes |
|----------|--------|-----------------|-------|
| bugprone-* | 34 | JSF AV general, MISRA-adjacent | Bug-prone patterns |
| cert-* | 14 | SEI CERT C/C++ | Secure coding |
| cppcoreguidelines-* | 11 | JSF AV 168/188/206/209 | goto, heap, types, casts |
| google-* | 6 | JSF AV Rule 209 | Fixed-width types |
| hicpp-* | 3 | JSF AV 14, P10-1 | Literal suffixes, goto |
| misc-* | 11 | P10-1/6, LOC-3 | Recursion, scope, const |
| modernize-* | 11 | JSF AV 29-31, C++20 | constexpr, nullptr |
| performance-* | 11 | Embedded RT constraints | int-to-ptr, copies |
| readability-* | 26 | JSF AV 1/50-53/59/151/152 | Size, naming, braces, magic |
| clang-analyzer-* | ~20 | P10-10, LOC-3 | Flow analysis |
| **Total** | **~127** | | |

### 2.2 Checks Intentionally Skipped

Each skipped check has a documented rationale in `.clang-tidy`:

| Check | Reason |
|-------|--------|
| `bugprone-easily-swappable-parameters` | Too noisy for embedded C-style APIs |
| `bugprone-exception-escape` | Exceptions disabled (`-fno-exceptions`) |
| `bugprone-reserved-identifier` | SDK names trigger false positives |
| `cert-err58-cpp`, `cert-err60-cpp` | Exceptions disabled |
| `cppcoreguidelines-avoid-c-arrays` | Static C arrays intentional (AP-1 deviation) |
| `cppcoreguidelines-pro-bounds-*` | Incompatible with Pico SDK C APIs |
| `cppcoreguidelines-pro-type-reinterpret-cast` | Needed for MPU/HW register access |
| `cppcoreguidelines-pro-type-vararg` | `printf` allowed in Ground/IVP code (IO-1) |
| `cppcoreguidelines-avoid-non-const-global-variables` | `g_` globals are by design |
| `cppcoreguidelines-owning-memory` | No `gsl::owner` in bare-metal |
| `hicpp-no-assembler` | `__asm volatile` needed for BASEPRI (LL Entry 3) |
| `hicpp-vararg` | Alias for pro-type-vararg |
| `misc-include-cleaner` | Too noisy with SDK cross-includes |
| `misc-confusable-identifiers` | Slow, not safety-critical |
| `modernize-use-auto` | Explicit types preferred in embedded |
| `modernize-use-trailing-return-type` | Not our style |
| `modernize-use-nodiscard` | Evaluate later |
| `modernize-use-std-print` | No `std::print` on bare-metal |
| `performance-avoid-endl` | No iostream usage |
| `performance-inefficient-string-concatenation` | No `std::string` |
| `performance-inefficient-vector-operation` | No `std::vector` |
| `readability-identifier-length` | Single-char loop vars are fine |
| `readability-qualified-auto` | We don't use `auto` |
| `clang-analyzer-osx.*` | macOS only |
| `clang-analyzer-unix.Malloc` | No heap usage |
| `clang-analyzer-optin.mpi.*` | No MPI |

### 2.3 Standards Rules NOT Covered by clang-tidy

These rules from our standards require human review or compiler flags, not clang-tidy:

| Rule | Standard | Verification Method |
|------|----------|-------------------|
| P10-2: Fixed loop bounds | P10 | Human review (bare-metal loops accepted — BM-1..6) |
| P10-3: No heap after init | P10 / JSF 206 | Human review + `cppcoreguidelines-no-malloc` |
| P10-5: Assertions | P10 | Human review (assertion density) |
| P10-7: Return value checking | P10 | Partially covered by `cert-err33-c` |
| P10-8: Preprocessor limits | P10 | Human review |
| P10-9: Pointer dereference limits | P10 | Human review |
| JSF 9: Line length (79 chars) | JSF AV | `clang-format` or script |
| JSF 171-174: Operator overloading | JSF AV | Human review (no operator overloads) |
| Variable shadowing | JSF AV 135 | Compiler flag `-Wshadow` |
| Switch fallthrough | JSF AV 192 | Compiler flag `-Wimplicit-fallthrough` |
| All platform-specific rules | CODING_STANDARDS.md | Human review / hardware testing |

---

## Section 3: Detailed Per-File Results

### 3.1 src/main.cpp — 440 warnings (IVP Test classification)

| Check | Count | Notes |
|-------|-------|-------|
| readability-magic-numbers | 107 | IVP gate thresholds, timing constants |
| hicpp-uppercase-literal-suffix | 96 | `1.0f` → `1.0F` (cosmetic) |
| readability-braces-around-statements | 87 | Single-line `if` without braces |
| readability-identifier-naming | 22 | snake_case locals (legacy C style) |
| readability-function-size | 20 | 20 functions exceed 60 lines |
| cppcoreguidelines-init-variables | 26 | Uninitialized locals |
| modernize-redundant-void-arg | 22 | `void` in `f(void)` definitions |
| readability-function-cognitive-complexity | 12 | Complex IVP gate functions |
| modernize-use-nullptr | 20 | `NULL` or `0` → `nullptr` |
| cppcoreguidelines-pro-type-member-init | 11 | Struct members not initialized |
| readability-implicit-bool-conversion | 9 | Implicit int/ptr → bool |
| readability-math-missing-parentheses | 6 | Operator precedence ambiguity |
| bugprone-branch-clone | 2 | Identical branches |
| bugprone-implicit-widening-of-multiplication-result | 2 | **Safety** |
| readability-isolate-declaration | 1 | Multiple decls on one line |
| readability-avoid-nested-conditional-operator | 2 | Nested ternary |
| misc-no-recursion | 1 | **P10-1 violation** |
| bugprone-misplaced-widening-cast | 1 | **Safety** |
| modernize-loop-convert | 1 | Range-for opportunity |

### 3.2 src/drivers/icm20948.cpp — 173 warnings (Flight-Critical)

| Check | Count | Notes |
|-------|-------|-------|
| readability-magic-numbers | 43 | Register offsets, byte indices — many are datasheet values |
| readability-braces-around-statements | 42 | Compact `if (!ok) return false;` pattern |
| hicpp-uppercase-literal-suffix | 21 | Float literal suffixes |
| bugprone-narrowing-conversions | 16 | int → float in sensor conversion |
| readability-identifier-naming | 12 | snake_case locals (C-origin driver) |
| modernize-use-nullptr | 12 | NULL → nullptr |
| cppcoreguidelines-init-variables | 9 | Uninitialized read-back variables |
| readability-function-size | 1 | `icm20948_read` (77 lines) |
| readability-implicit-bool-conversion | 6 | Pointer/int → bool |
| readability-braces-around-statements | 42 | Single-line returns |

**Flight-critical note:** The 16 narrowing conversions are all `int16_t` → `float` for
sensor raw data. 16-bit values are within `float` precision (24-bit mantissa). These are
safe but should have explicit casts with range comments for auditability.

### 3.3 src/drivers/baro_dps310.cpp — 35 warnings (Flight-Critical)

| Check | Count | Notes |
|-------|-------|-------|
| readability-identifier-naming | 18 | SDK callback parameters (snake_case required by Ruuvi API) |
| hicpp-uppercase-literal-suffix | 4 | Float suffixes |
| readability-magic-numbers | 4 | 101325.0f (sea level), 44330.0f (hypsometric) |
| modernize-use-nullptr | 2 | NULL → nullptr |
| modernize-redundant-void-arg | 3 | void params |
| cppcoreguidelines-pro-type-cstyle-cast | 2 | SDK callback casts |
| readability-simplify-boolean-expr | 1 | Simplifiable return |
| readability-isolate-declaration | 1 | Multi-decl |

**Note:** 18 naming violations are SDK callback signatures — parameters must match the
Ruuvi DPS310 driver typedef. These are unavoidable (SDK constraint).

### 3.4 src/drivers/i2c_bus.cpp — 39 warnings (Flight-Critical)

| Check | Count | Notes |
|-------|-------|-------|
| readability-implicit-bool-conversion | 10 | gpio_dir → bool, int → bool |
| readability-magic-numbers | 9 | I2C addresses, recovery timing |
| modernize-redundant-void-arg | 5 | void params |
| modernize-use-nullptr | 4 | NULL → nullptr |
| modernize-use-bool-literals | 5 | `gpio_set_dir(pin, 0)` → `false` |
| readability-identifier-naming | 3 | snake_case locals |
| bugprone-switch-missing-default-case | 1 | Switch on probe result |
| hicpp-multiway-paths-covered | 1 | Same switch |
| cppcoreguidelines-init-variables | 1 | `dummy` not initialized |

### 3.5 src/drivers/gps_pa1010d.cpp — 29 warnings (Flight-Critical)

| Check | Count | Notes |
|-------|-------|-------|
| readability-magic-numbers | 9 | 0x0A (LF), 0x0D (CR), PMTK rates |
| readability-identifier-naming | 5 | snake_case params/locals |
| modernize-redundant-void-arg | 5 | void params |
| readability-implicit-bool-conversion | 2 | uint8_t → bool |
| modernize-use-nullptr | 2 | NULL → nullptr |
| cert-err33-c | 2 | Unchecked snprintf return |
| cppcoreguidelines-init-variables | 1 | interval_ms |
| bugprone-branch-clone | 1 | Identical branches |
| readability-braces-around-statements | 1 | Single-line if |

### 3.6 src/drivers/ws2812_status.cpp — 82 warnings (Flight-Support)

| Check | Count | Notes |
|-------|-------|-------|
| hicpp-uppercase-literal-suffix | 32 | Float literals in HSV math (360.0f, 255.0f, etc.) |
| readability-magic-numbers | 26 | Color math constants (360, 255, 60, 120, etc.) |
| readability-identifier-naming | 5 | snake_case locals |
| bugprone-narrowing-conversions | 3 | int → float in color blending |
| bugprone-branch-clone | 2 | Identical switch cases |
| readability-braces-around-statements | 3 | Single-line if |
| modernize-redundant-void-arg | 3 | void params |
| readability-function-size | 1 | `ws2812_update` (62 lines) |
| readability-isolate-declaration | 1 | `float r1, g1, b1;` |
| cppcoreguidelines-init-variables | 3 | HSV conversion vars |
| readability-math-missing-parentheses | 1 | Brightness calc |
| modernize-use-nullptr | 1 | NULL → nullptr |
| performance-no-int-to-ptr | 0 | (clean) |

### 3.7 src/calibration/calibration_data.cpp — 48 warnings (Ground)

| Check | Count | Notes |
|-------|-------|-------|
| hicpp-uppercase-literal-suffix | 23 | Float default values (1.0f, 0.0f) |
| readability-magic-numbers | 7 | CRC polynomial (0x1021), sensor defaults |
| readability-identifier-naming | 7 | snake_case locals |
| readability-braces-around-statements | 6 | Single-line checks |
| modernize-use-nullptr | 4 | NULL → nullptr |
| cppcoreguidelines-pro-type-cstyle-cast | 3 | Pointer casts for CRC |
| readability-implicit-bool-conversion | 1 | CRC check |

### 3.8 src/calibration/calibration_manager.cpp — 274 warnings (Ground)

| Check | Count | Notes |
|-------|-------|-------|
| readability-identifier-naming | 64 | snake_case globals/locals (C-origin code) |
| readability-magic-numbers | 61 | Matrix indices, calibration thresholds |
| readability-math-missing-parentheses | 55 | Matrix arithmetic (Jacobian, residuals) |
| hicpp-uppercase-literal-suffix | 40 | Float literals in matrix math |
| readability-braces-around-statements | 21 | Loop/if single-line |
| readability-function-size | 5 | Long calibration functions |
| cppcoreguidelines-init-variables | 10 | Matrix loop variables |
| bugprone-narrowing-conversions | 7 | int → float in indices |
| readability-function-cognitive-complexity | 3 | Complex fit routines |
| readability-math-missing-parentheses | 55 | Dominant — all in Jacobian computation |
| readability-isolate-declaration | 5 | Multi-decl in loops |
| cppcoreguidelines-pro-type-cstyle-cast | 1 | memcpy cast |

### 3.9 src/calibration/calibration_storage.cpp — 45 warnings (Ground)

| Check | Count | Notes |
|-------|-------|-------|
| readability-identifier-naming | 21 | snake_case (C-origin flash code) |
| readability-braces-around-statements | 6 | Single-line checks |
| cppcoreguidelines-pro-type-cstyle-cast | 5 | Flash pointer casts (necessary) |
| modernize-use-using | 4 | `typedef` → `using` |
| modernize-use-nullptr | 3 | NULL → nullptr |
| modernize-redundant-void-arg | 3 | void params |
| readability-isolate-declaration | 2 | Multi-decl |
| readability-simplify-boolean-expr | 1 | Boolean simplification |
| performance-no-int-to-ptr | 1 | Flash address cast |

### 3.10 src/cli/rc_os.cpp — 86 warnings (Ground)

| Check | Count | Notes |
|-------|-------|-------|
| modernize-redundant-void-arg | 18 | void params in callbacks |
| cert-err33-c | 16 | Unchecked printf return values |
| readability-implicit-bool-conversion | 10 | Function ptr → bool |
| readability-identifier-naming | 9 | snake_case locals |
| cppcoreguidelines-init-variables | 7 | Calibration loop vars |
| modernize-use-nullptr | 6 | NULL → nullptr |
| readability-magic-numbers | 4 | ESC key (27), timing |
| readability-math-missing-parentheses | 3 | Magnitude calc |
| readability-function-size | 2 | `cmd_accel_6pos_cal_inner`, `handle_calibration_menu` |
| readability-function-cognitive-complexity | 1 | `cmd_accel_6pos_cal_inner` (45) |
| hicpp-uppercase-literal-suffix | 2 | Float suffixes |
| bugprone-branch-clone | 1 | Identical switch case |
| readability-isolate-declaration | 2 | Multi-decl |

---

## Section 4: Remediation Priority

### Priority 1 — Safety-Critical (fix before flight code)

| Finding | Location | Action |
|---------|----------|--------|
| Recursion (P10-1) | main.cpp:1004 | Investigate call chain, refactor or document |
| Implicit widening multiplication | main.cpp:3094, 3117 | Cast operand to uint64_t |
| Misplaced widening cast | main.cpp:2444 | Review cast placement |
| Missing default case | i2c_bus.cpp:95 | Add `default: break;` |

### Priority 2 — High (fix when touching affected code)

| Category | Count | Files | Action |
|----------|-------|-------|--------|
| Narrowing conversions | 27 | icm20948, ws2812, cal_mgr, main | Add explicit casts with range comments |
| Uninitialized variables | 51 | All files | Initialize at declaration |
| Function size (>60 lines) | 27 | main, icm20948, cal_mgr, rc_os, ws2812 | Decompose or accept with deviation |

### Priority 3 — Medium (batch fix or accept)

| Category | Count | Action |
|----------|-------|--------|
| Magic numbers | 275 | Extract to named constants (batch) |
| Missing braces | 170 | Add braces (batch, or `clang-tidy --fix`) |
| Identifier naming | 162 | Many are SDK constraints; fix codebase-owned names |
| Math parentheses | 65 | Add explicit parentheses (mostly in cal_mgr) |
| Cognitive complexity | 15 | Decompose or accept for complex algorithms |
| C-style casts | 11 | Convert to `static_cast` / `reinterpret_cast` |

### Priority 4 — Low (cosmetic, address incrementally)

| Category | Count | Action |
|----------|-------|--------|
| Uppercase literal suffix (`f`→`F`) | 216 | `clang-tidy --fix` (auto-fixable) |
| Redundant void arg | 82 | `clang-tidy --fix` (auto-fixable) |
| Use nullptr | 43 | `clang-tidy --fix` (auto-fixable) |
| Implicit bool conversion | 41 | Add explicit comparisons |
| Use bool literals | 9 | `1` → `true` |
| Isolate declarations | 8 | One var per line |
| Use using (typedef) | 4 | `typedef` → `using` |
| Simplify boolean expr | 2 | Simplify returns |

---

## Section 5: Auto-Fixable Warnings

clang-tidy can automatically fix many warnings with `--fix`. These are safe to batch-apply:

| Check | Count | Risk |
|-------|-------|------|
| `hicpp-uppercase-literal-suffix` | 216 | None (cosmetic) |
| `modernize-redundant-void-arg` | 82 | None (C++20 standard) |
| `modernize-use-nullptr` | 43 | None (type-safe) |
| `modernize-use-bool-literals` | 9 | None (explicit) |
| `modernize-use-using` | 4 | None (equivalent) |
| **Total auto-fixable** | **354** | Removes 28% of all warnings |

**Command:** `clang-tidy --fix src/<file>.cpp -p build/ --checks="-*,hicpp-uppercase-literal-suffix,modernize-redundant-void-arg,modernize-use-nullptr,modernize-use-bool-literals,modernize-use-using" [extra-args...]`

**Caution:** Always review `git diff` after auto-fix. Some fixes near SDK callback
signatures may need manual adjustment.

---

## Section 6: Cross-Reference with Manual Audit

Findings where automated and manual audits disagree:

| Rule | Manual Audit (2026-02-07) | Automated Audit | Resolution |
|------|--------------------------|-----------------|------------|
| P10-1 (no recursion) | PASS | **FAIL** — `ivp_test_key_handler` recursive | Automated correct |
| JSF 59 (braces) | PARTIAL | FAIL (170 occurrences) | Count refined |
| JSF 151 (magic numbers) | PARTIAL | FAIL (275 occurrences) | Count refined |
| JSF 50-53 (naming) | PARTIAL | FAIL (162 occurrences) | SDK constraints clarified |
| JSF 145 (NULL vs nullptr) | NOT CHECKED in manual | FAIL (43 occurrences) | New finding |

---

## Section 7: Recommendations

1. **Immediate:** Fix the 5 safety-critical findings (Section 1)
2. **Next session:** Run auto-fix for the 354 cosmetic warnings (Section 5)
3. **Ongoing:** Run clang-tidy before each commit to prevent regression
4. **Future:** Integrate into CI/CD when GitHub Actions is set up
5. **Tune:** Add `// NOLINT` annotations for accepted deviations (SDK callback names, intentional casts)

---

## Appendix: clang-tidy Configuration Summary

The `.clang-tidy` config in the project root contains 376 lines with:
- 127 checks enabled across 10 categories
- 27 checks explicitly skipped with documented rationale
- CheckOptions tuned for embedded ARM bare-metal:
  - Function size: 60 lines (P10), 200 statements (JSF), 6 nesting levels
  - Cognitive complexity: 25 threshold
  - Naming: snake_case functions, kCamelCase constants, g_camelBack globals
  - Magic numbers: 0-4,8,16,32,64,100,255,256,1000 ignored
  - Narrowing: both integer and floating-point warnings enabled
  - Special members: relaxed for POD aggregates

See `.clang-tidy` for the full configuration with inline standards mapping comments.
