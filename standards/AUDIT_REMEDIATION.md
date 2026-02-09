# Audit Remediation Log

**Purpose:** Track remediation from standards audit findings.
**Source:** `standards/STANDARDS_AUDIT_2026-02-07.md` (44 PARTIAL/FAIL findings)
**Status:** All tiers complete and build-verified (2026-02-07).

---

## Summary

| Tier | Scope | Fixes | Build |
|------|-------|-------|-------|
| 1: Quick Fixes | Braces (JSF 59), hex case (150), default clause (194), precedence (213), return checks (115), build tag (LL Entry 2) | 7 fixes across main.cpp, rc_os.c, calibration_data.c | PASS — 0 warnings |
| 2: Moderate | One-expr-per-line (42), one-var-per-decl (152), continue removal (190), fixed-width loop counters (209), pointer style (62), `-Werror`/`-Wpedantic` (P10-10) | 6 fix groups across all source files + CMakeLists.txt | PASS — 0 warnings |
| 3: Deviations | C++20 upgrade (JSF 8), debug macro refactor (26/28), stdio analysis (22/24), signed/unsigned casts (162), variadic macros (P10-8) | See disposition table below | PASS — 0 warnings |
| 4: Architectural | RC_ASSERT macro (P10-5), goto elimination (188/189), bare-metal loop docs (P10-2), IVP globals docs (P10-6) | D1-D2 done, D3/D5 deferred to production refactoring | PASS — 0 warnings |

---

## Tier 3 Disposition (Accepted Deviations)

| # | Rule | Finding | Disposition |
|---|------|---------|-------------|
| 15 | JSF 29/30/31 | `#define` for constants/macros | **Resolved** — .c→.cpp rename + constexpr conversion (2026-02-08) |
| 16 | JSF 26/28 | `#ifdef DEBUG`, `#ifdef __cplusplus` | C++ path uses `constexpr bool` + `if constexpr`. `#ifdef __cplusplus` resolved by .c→.cpp conversion |
| 17 | JSF 22/24 | stdio.h / sleep_ms | **Accepted** — runtime lockout when state != IDLE. GPS `snprintf` bounded by `sizeof()`. `sleep_ms` is SDK timer, not stdlib |
| 18 | JSF 162 | 6 signed/unsigned casts | **Accepted** — intentional at SDK/hardware type boundaries |
| 19 | P10-8 | DBG_PRINT variadic macros | **Resolved** — C++20 variadic templates. Compatibility macros are simple name mappings |
| 20 | JSF 8 | C17+C++20 with 3 extensions | **Accepted** — `__asm volatile`, `__attribute__((aligned))`, `<atomic>` justified |
| 21 | LOC-2.5 | FIFO pop without validation | **Accepted** — IVP-22 exercise-only test code. Stripped during D3 refactoring |
| 22 | Prior art | SDK refs, doc gaps | **Pass** — all drivers reference datasheets/libraries |

---

## GPS snprintf Mitigation (Fix C17 detail)

3 bounded `snprintf` calls in `gps_pa1010d.cpp` for NMEA command formatting. Documented MISRA deviation. Migration path: custom formatters or u-blox UBX binary protocol.
