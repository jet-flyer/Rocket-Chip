# RocketChip Automated Standards Audit — 2026-02-20

**Tool:** clang-tidy 21.1.8 (LLVM)
**Config:** `.clang-tidy` (379-line comprehensive config, 127 checks)
**Audited By:** Claude Code CLI (automated)
**Codebase Snapshot:** `74d9adc` (post-doc cleanup, pre-IVP-47)
**Lines Audited:** 10,478 (18 .cpp, 20 .h, 1 config.h)
**Binary Size:** 221,184 bytes
**Run Script:** `scripts/run_clang_tidy.sh`

---

## Changes Since Previous Audit (2026-02-09)

| Item | Previous | Current |
|------|----------|---------|
| LLVM version | 19.1.7 | 21.1.8 |
| Files audited | 10 | 17 |
| Lines audited | ~5,149 | 10,478 |
| C++ include path | `v8-m.main/nofp` (incorrect) | `v8-m.main+fp/softfp` (matches `-mfloat-abi=softfp`) |
| Safety-critical findings | 5 | **0** |

**New files since 2026-02-09 audit (never previously audited):**
- `src/fusion/eskf.cpp` (1,305 lines) — ESKF propagation + measurement updates
- `src/fusion/baro_kf.cpp` (145 lines) — Barometric Kalman filter
- `src/fusion/wmm_declination.cpp` (131 lines) — WMM declination lookup table
- `src/fusion/mahony_ahrs.cpp` (128 lines) — Mahony AHRS cross-check
- `src/drivers/gps_uart.cpp` (429 lines) — UART GPS driver
- `src/math/vec3.cpp` (52 lines) — 3D vector math
- `src/math/quat.cpp` (180 lines) — Quaternion math

**Significant changes to previously-audited files:**
- `src/main.cpp` — ESKF integration, GPS wiring, Mahony wiring, zero-output detection
- `src/cli/rc_os.cpp` — Mag cal wizard, ESKF CLI output, calibration wizard

---

## Summary Dashboard

### Totals

| Metric | Value |
|--------|-------|
| Files audited | 17 (all production `.cpp` files) |
| Total warnings | 1,501 |
| Unique check types triggered | 20 |
| Safety-critical findings | **0** |
| Enabled checks | 127 |
| Intentionally skipped checks | 27 (unchanged — documented in `.clang-tidy`) |

### Comparison with Previous Audit

| Metric | 2026-02-09 | 2026-02-20 | Delta |
|--------|------------|------------|-------|
| Total warnings | 1,251 | 1,501 | +250 |
| Files | 10 | 17 | +7 new files |
| Lines | ~5,149 | 10,478 | +5,329 (2x) |
| **Warnings per 100 LOC** | **24.3** | **14.3** | **-41%** |
| Safety-critical | 5 | 0 | -5 (all resolved) |
| IVP test code warnings | ~440 | 0 | Stripped |

Warning density improved by 41% despite doubling the codebase — the new fusion/math code is cleaner than the original code was before remediation.

### Per-File Breakdown

| File | Classification | Warnings | Top Issue |
|------|---------------|----------|-----------|
| `src/fusion/wmm_declination.cpp` | Flight-Support | 719 | uppercase-suffix (710) — data table |
| `src/fusion/eskf.cpp` | Flight-Critical | 264 | identifier-naming (141) |
| `src/main.cpp` | Ground (mixed) | 148 | identifier-naming (48) |
| `src/math/quat.cpp` | Flight-Critical | 84 | uppercase-suffix (64) |
| `src/calibration/calibration_manager.cpp` | Ground | 80 | identifier-naming (68) |
| `src/fusion/baro_kf.cpp` | Flight-Critical | 48 | uppercase-suffix (23) |
| `src/fusion/mahony_ahrs.cpp` | Flight-Support | 46 | identifier-naming (24) |
| `src/cli/rc_os.cpp` | Ground | 43 | cert-err33-c (22) |
| `src/drivers/gps_uart.cpp` | Flight-Critical | 11 | cert-err33-c (3) |
| `src/drivers/gps_pa1010d.cpp` | Flight-Critical | 9 | identifier-naming (3) |
| `src/calibration/calibration_storage.cpp` | Ground | 9 | identifier-naming (5) |
| `src/drivers/i2c_bus.cpp` | Flight-Critical | 9 | implicit-bool-conversion (3) |
| `src/drivers/ws2812_status.cpp` | Flight-Support | 8 | branch-clone (2) |
| `src/drivers/icm20948.cpp` | Flight-Critical | 6 | identifier-naming (3) |
| `src/drivers/baro_dps310.cpp` | Flight-Critical | 6 | identifier-naming (4) |
| `src/math/vec3.cpp` | Flight-Critical | 6 | uppercase-suffix (4) |
| `src/calibration/calibration_data.cpp` | Ground | 5 | identifier-naming (3) |

### Warning Distribution by Check

| Count | Check | Standards Mapping | Severity |
|-------|-------|-------------------|----------|
| 893 | `hicpp-uppercase-literal-suffix` | JSF AV Rule 14 | Low |
| 339 | `readability-identifier-naming` | JSF AV Rules 50-53 | Medium |
| 113 | `readability-magic-numbers` | JSF AV Rule 151 | Medium |
| 29 | `cert-err33-c` | CERT ERR33-C | Low* |
| 25 | `readability-function-size` | P10-4 / JSF AV Rule 1 | Medium |
| 23 | `readability-implicit-bool-conversion` | JSF AV Rule 180 | Low |
| 20 | `cppcoreguidelines-init-variables` | JPL LOC-3.5 / CERT | Medium |
| 11 | `google-build-using-namespace` | Google C++ / JSF AV 99 | Medium |
| 9 | `readability-function-cognitive-complexity` | JSF AV Rule 3 | Medium |
| 6 | `readability-isolate-declaration` | JSF AV Rule 152 | Low |
| 6 | `bugprone-branch-clone` | Code quality | Low |
| 5 | `cppcoreguidelines-pro-type-member-init` | C++ Core Guidelines | Low |
| 4 | `readability-simplify-boolean-expr` | Readability | Low |
| 2 | `readability-inconsistent-declaration-parameter-name` | Readability | Low |
| 2 | `readability-avoid-nested-conditional-operator` | Readability | Low |
| 2 | `modernize-loop-convert` | C++11 modernization | Low |
| 1 | `performance-no-int-to-ptr` | Performance | Low |
| 1 | `misc-use-internal-linkage` | P10-6 / LOC-3.6 | Low |
| 1 | `clang-analyzer-deadcode.DeadStores` | Clang analyzer | Low |

*cert-err33-c: All 29 are unchecked `printf()`/`snprintf()` returns in Ground code.

---

## Section 1: Safety Assessment

**Zero safety-critical findings.** All 5 safety-critical findings from the 2026-02-09 audit have been resolved:

| Previous Finding | Resolution |
|-----------------|------------|
| Recursion (`misc-no-recursion`) in IVP test code | IVP test code stripped (2026-02-09) |
| Implicit widening multiplication (2 instances) | IVP test code stripped |
| Misplaced widening cast | IVP test code stripped |
| Missing default case in i2c_bus.cpp switch | Fixed during remediation |
| 27 narrowing conversions | Fixed with explicit casts |

**No new safety-critical findings in the 7 new files.** The fusion/math code has zero bugprone-narrowing-conversions, zero widening issues, and zero recursion.

---

## Section 2: Remediation Plan

### Phase 1: Auto-Fix — Uppercase Literal Suffix (893 warnings)

All `f` → `F` suffix changes. Zero risk, cosmetic only.

```bash
"C:/Program Files/LLVM/bin/clang-tidy.exe" <file> -p build/ --fix \
  --checks="-*,hicpp-uppercase-literal-suffix" [extra-args...]
```

| File | Count | Notes |
|------|-------|-------|
| `wmm_declination.cpp` | 710 | Float data table |
| `quat.cpp` | 64 | Math constants |
| `eskf.cpp` | 54 | Filter constants |
| `baro_kf.cpp` | 23 | Filter constants |
| `mahony_ahrs.cpp` | 20 | Constants |
| `main.cpp` | 17 | Various |
| Other (5 files) | 5 | Scattered |

### Phase 2: Identifier Naming (339 warnings)

**Decision needed:** The 7 new fusion/math files use `snake_case` for locals and parameters. The `.clang-tidy` config expects `camelBack` (configured during 2026-02-09 remediation). Options:

**A) Convert new files to camelBack** — Matches existing remediated code. ~339 renames across 7 files.
**B) Change config to allow snake_case** — Accepts the current style. The `snake_case` reads more naturally for math-heavy code (`accel_body`, `gyro_meas`, `ned_down`) vs `accelBody`, `gyroMeas`, `nedDown`.

| File | Count | Style |
|------|-------|-------|
| `eskf.cpp` | 141 | snake_case locals/params |
| `calibration_manager.cpp` | 68 | snake_case (pre-existing from C origin) |
| `main.cpp` | 48 | Mixed |
| `mahony_ahrs.cpp` | 24 | snake_case |
| `baro_kf.cpp` | 20 | snake_case |
| `wmm_declination.cpp` | 9 | snake_case |
| `quat.cpp` | 8 | snake_case |
| Other (7 files) | 21 | Scattered |

### Phase 3: Magic Numbers (113 warnings)

Extract to named `constexpr` constants. Concentrated in:

| File | Count | Type |
|------|-------|------|
| `eskf.cpp` | 46 | Filter thresholds, NIS gates, noise parameters |
| `main.cpp` | 34 | Sensor config, timing, thresholds |
| `quat.cpp` | 11 | Trig constants, epsilon values |
| Other | 22 | Scattered |

### Phase 4: Function Size + Complexity (25 + 9 warnings)

Functions exceeding the 60-line / 25-complexity thresholds:

| File | Function | Lines/CC | Assessment |
|------|----------|----------|------------|
| `eskf.cpp` | `update_gps_position` | CC=39 | Each measurement update follows the same Kalman pattern; decomposing would fragment the algorithm |
| `eskf.cpp` | `update_zupt` | CC=35 | Stationarity detection + update in one function |
| `eskf.cpp` | `update_gps_velocity` | CC=33 | Same Kalman pattern |
| `eskf.cpp` | `predict`, `update_baro`, `update_mag_heading` | >60 lines | Matrix math — line count is inherent |
| `main.cpp` | `eskf_tick` | CC=70 | Central dispatcher — orchestrates all ESKF feeds |
| `main.cpp` | `print_seqlock_sensors` | CC=43 | Status output formatting |
| `main.cpp` | `core1_sensor_loop`, `core1_read_imu`, `core1_read_gps` | >60 lines | Sensor read + error handling |
| `rc_os.cpp` | `cmd_wizard` | CC=61 | 5-step calibration wizard |
| `rc_os.cpp` | `mag_cal_inner` | CC=44 | Mag calibration workflow |
| `rc_os.cpp` | `handle_main_menu`, `handle_calibration_menu`, `rc_os_update` | >60 lines | CLI dispatch |
| `calibration_manager.cpp` | `mag_sphere_fit` | CC=34 | LM optimizer — algorithmic complexity |
| `calibration_manager.cpp` | `mag_ellipsoid_fit` | CC=31 | LM optimizer |
| Others | Various | >60 lines | `i2c_bus_recover`, `icm20948_read`, `gps_uart_init`, etc. |

**Recommendation:** Accept ESKF measurement updates and calibration algorithms at current size — decomposing Kalman update steps or LM iterations would reduce locality without improving readability. `eskf_tick` (CC=70) is the primary decomposition candidate.

### Phase 5: Remaining Quality Findings (97 warnings)

| Check | Count | Action |
|-------|-------|--------|
| `cert-err33-c` | 29 | All unchecked `printf()`/`snprintf()` in Ground code. Accept (IO-1/IO-2 deviation) |
| `implicit-bool-conversion` | 23 | Add explicit `!= 0`/`!= nullptr` |
| `init-variables` | 20 | Initialize at declaration |
| `using-namespace` | 11 | Replace `using namespace rc;` with `using rc::Vec3;` etc. in eskf.cpp |
| `cognitive-complexity` | 9 | See Phase 4 |
| `isolate-declaration` | 6 | One var per line |
| `branch-clone` | 6 | Review for consolidation |
| `member-init` | 5 | Add member initializers |
| Other | 7 | Case-by-case |

---

## Section 3: Checks Not Triggered (Positive)

These enabled checks found **zero warnings** — confirming codebase compliance:

| Check Category | Count | What It Means |
|---------------|-------|---------------|
| `bugprone-narrowing-conversions` | 0 | No implicit narrowing (was 27 in 2026-02-09) |
| `bugprone-implicit-widening-of-multiplication-result` | 0 | No silent overflow risk |
| `misc-no-recursion` | 0 | P10-1 compliant — no recursion |
| `cppcoreguidelines-no-malloc` | 0 | P10-3 compliant — no heap |
| `cppcoreguidelines-avoid-goto` | 0 | No goto statements |
| `bugprone-switch-missing-default-case` | 0 | All switches have default |
| `modernize-use-nullptr` | 0 | No NULL/0 as pointer (was 43 in 2026-02-09) |
| `modernize-redundant-void-arg` | 0 | No `f(void)` style (was 82 in 2026-02-09) |
| `readability-braces-around-statements` | 0 | All if/for/while have braces (was 170 in 2026-02-09) |
| `cppcoreguidelines-pro-type-cstyle-cast` | 0 | No C-style casts (was 11 in 2026-02-09) |
| All `clang-analyzer-core.*` | 0 | No null deref, div-by-zero, or use-after-free |
| All `clang-analyzer-cplusplus.*` | 0 | No C++ lifetime issues |

---

## Section 4: Raw Output

Per-file clang-tidy output stored in `logs/clang-tidy-2026-02-20/`:

```
logs/clang-tidy-2026-02-20/
├── summary.txt           # One-line-per-file warning counts
├── main.txt              # 148 warnings
├── eskf.txt              # 264 warnings
├── wmm_declination.txt   # 719 warnings
├── quat.txt              # 84 warnings
├── calibration_manager.txt # 80 warnings
├── baro_kf.txt           # 48 warnings
├── mahony_ahrs.txt       # 46 warnings
├── rc_os.txt             # 43 warnings
├── gps_uart.txt          # 11 warnings
├── i2c_bus.txt           # 9 warnings
├── gps_pa1010d.txt       # 9 warnings
├── calibration_storage.txt # 9 warnings
├── ws2812_status.txt     # 8 warnings
├── icm20948.txt          # 6 warnings
├── baro_dps310.txt       # 6 warnings
├── vec3.txt              # 6 warnings
└── calibration_data.txt  # 5 warnings
```
