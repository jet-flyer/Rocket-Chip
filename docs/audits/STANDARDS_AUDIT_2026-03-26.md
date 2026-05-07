# RocketChip Standards Audit — 2026-03-26

**Audit Date:** 2026-03-26
**Audited By:** Claude Code CLI + Grok (tiered audit script design)
**Codebase Snapshot:** `5d6ee8a` (post-Stage 8, IVP-66–75 complete)
**Lines Audited:** ~40 production source files, 552 host tests passing
**Tools:** clang-tidy 21.1.8 (127 checks), lizard 1.21.2 (cyclomatic), custom grep tiers

---

## Executive Summary

| Tier | Tool | Total | Actionable | Critical |
|------|------|-------|-----------|----------|
| 1 | clang-tidy | 2,315 | ~497 (excl. codegen+bench) | 1 (uninit var) |
| 2 | lizard | 26 | 2 CCN>20 | 0 (Ground code) |
| 3 | RP2350 guards | 25 | ~2 real | 0 |
| 4 | Prior Art | 8 files | 6 files | 0 |

**Overall assessment:** No flight-safety violations found. One potential bug
(uninitialized variable in ring_buffer.cpp). Remaining findings are
documentation gaps, naming style, and cosmetic issues.

---

## Tier 1: clang-tidy (2,315 warnings)

### Excluded from remediation

| File | Warnings | Reason |
|------|----------|--------|
| `eskf_codegen.cpp` | 1,529 | Auto-generated (deviation CG-1). Cannot remediate. |
| `ud_benchmark.cpp` | 254 | Benchmark tool, non-production. |
| `mat_benchmark.cpp` | 35 | Benchmark tool, non-production. |

**Remaining:** ~497 warnings across 33 production files.

### Findings by priority

#### HIGH — Potential bug

| File | Line | Warning | Action |
|------|------|---------|--------|
| `ring_buffer.cpp` | ? | `variable 'pos' is not initialized` | Investigate and fix. Potential read of garbage value. |

#### MEDIUM — Magic numbers (JSF 151)

~60 instances across production code. Largest concentrations:

| File | Count | Examples | Action |
|------|-------|---------|--------|
| `telemetry_encoder.cpp` | ~15 | `32767.0F`, `40` (header sizes) | Replace with named constants |
| `data_convert.cpp` | ~8 | `2147483647`, `15` (bit widths) | Replace with named constants |
| `main.cpp` | ~11 | `7` (GPIO), `5`/`6` (cal flags) | Some already have constants elsewhere |
| `rfm95w.cpp` | ~7 | `10`, `20` (register values) | Move to register constant namespace |
| `eskf.cpp` | ~6 | `6` (state indices), `1e-6F` (epsilon) | Some are state index references |

#### LOW — Naming style

~40 instances of `snake_case` parameter names flagged. Our API convention uses
`snake_case` for C-style function parameters (consistent with Pico SDK).
This is intentional and matches CODING_STANDARDS.md conventions.

**Action:** Document as accepted deviation or adjust clang-tidy naming config.

#### LOW — QEP C-style casts

31 instances in `flight_director.cpp` — all from QEP framework macros
(`Q_STATE_CAST`, `Q_TRAN`). These are QP/C library patterns, not our code.

**Action:** None. Suppress via NOLINT or accept as framework deviation.

#### LOW — Implicit bool conversions

~15 instances of pointer/int → bool in null checks and GPIO operations.
Idiomatic C/C++ patterns (`if (ptr)` instead of `if (ptr != nullptr)`).

**Action:** Low priority. Can fix incrementally with `!= nullptr` pattern.

#### LOW — Unused return values

~9 instances of ignored SDK function returns (`sleep_ms`, etc.).

**Action:** Cast to `(void)` where appropriate.

---

## Tier 2: Lizard Cyclomatic Complexity

### CCN > 20 violations (JSF AV Rule 3)

| Function | File | CCN | NLOC | Classification | Action |
|----------|------|-----|------|---------------|--------|
| `handle_calibration_menu` | rc_os.cpp:1218 | 24 | 54 | Ground | Accept — CLI switch inherent complexity |
| `handle_flight_menu` | rc_os.cpp:1304 | 28 | 30 | Ground | Accept — CLI switch, Stage 8 addition |

Both are Ground-classified CLI menu handlers. High CCN is driven by switch
case count (one case per menu key), not nested logic. Accepted per code
classification: Ground code has relaxed standards enforcement.

### PARAM > 5 (informational, 14 functions)

Largest: `mag_cal_print_progress` (10 params), `save_flight_entry` (8),
calibration apply functions (7 each). These could benefit from struct
parameter grouping in a future refactor but are not blocking.

### NLOC > 60 (informational, ~10 functions)

`codegen_fpft` (1084, deviation CG-1), `ESKF::init` (61), `guard_evaluator_tick` (65),
`mag_cal_collect_samples` (62), `handle_main_menu` (68). Most are near the
threshold. Pre-commit hook already gates at 60 lines.

---

## Tier 3: RP2350 Platform Guards

### False positives (22 of 25 hits)

| Source | Hits | Why false positive |
|--------|------|-------------------|
| `i2c_bus.cpp` `i2c_bus_scan()` | 14 | Function is Ground-only per CODING_STANDARDS.md file classification |
| `flight_director.cpp` log lines | 7 | Diagnostic logging on Core 0 CLI thread, not sensor loop |
| `go_nogo_checks.cpp` | 2 | Go/No-Go output only from CLI ARM command |

### Real findings (3 of 25)

| Source | Hits | Concern |
|--------|------|---------|
| `guard_combinator.cpp:148` | 1 | Backup timer `printf` — runs in flight context (100Hz tick). Low risk (Core 0, output buffered), but should use `DBG_PRINT` |

**Action:** Consider wrapping flight_director diagnostic `printf` calls in
`DBG_PRINT` so they compile out in Release builds. Not urgent — these
are on Core 0 and harmless when USB is disconnected in flight.

---

## Tier 4: Prior Art Documentation

### Missing `Prior Art:` blocks (8 driver files)

| File | Has references? | What's needed |
|------|----------------|---------------|
| `baro_dps310.cpp` | No | Infineon DPS310 datasheet, Adafruit DPS310 library |
| `gps_pa1010d.cpp` | No | CDTop PA1010D datasheet, Adafruit GPS library |
| `gps_uart.cpp` | Partial (mentions Adafruit) | Formalize with `Prior Art:` block |
| `i2c_bus.cpp` | No | NXP UM10204 I2C spec, Linux kernel i2c-algo-bit |
| `icm20948.cpp` | Yes (has refs, wrong tag) | Add `Prior Art:` tag to existing references |
| `rfm95w.cpp` | Yes (has refs, wrong tag) | Add `Prior Art:` tag to existing references |
| `spi_bus.cpp` | No | Pico SDK SPI examples |
| `ws2812_status.cpp` | No | Pico SDK PIO WS2812 example |

---

## Remediation Plan

### Batch 1: Critical (do now)
- [ ] `ring_buffer.cpp` — investigate uninitialized `pos` variable

### Batch 2: Medium priority (this session or next)
- [ ] Add `Prior Art:` blocks to 8 driver files (documentation)
- [ ] `guard_combinator.cpp:148` — wrap backup timer printf in `DBG_PRINT`

### Batch 3: Low priority (incremental, as files are touched)
- [ ] Magic number cleanup in telemetry_encoder, data_convert, main.cpp
- [ ] Document `snake_case` parameter naming as accepted convention
- [ ] Add `(void)` casts for ignored SDK return values

### Batch 4: No action needed
- QEP C-style casts (framework code)
- CLI menu CCN > 20 (Ground code, accepted)
- `eskf_codegen.cpp` warnings (auto-generated, deviation CG-1)
- Benchmark file warnings (non-production)

---

## Comparison with Previous Audit (2026-02-07)

| Metric | Feb 2026 | Mar 2026 | Change |
|--------|----------|----------|--------|
| Production files | 17 | 40 | +23 (Stage 5-8 additions) |
| Host tests | ~190 | 552 | +362 |
| clang-tidy warnings | 1,251 | 2,315 | +1,064 (mostly codegen+bench+new files) |
| CCN > 20 violations | 0 | 2 | +2 (CLI menus, Ground code) |
| Standards deviations | 8 | 8 | No new deviations |
