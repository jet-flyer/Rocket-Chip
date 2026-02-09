# Audit Remediation Log

**Purpose:** Track remediation from standards audit findings.
**Source:** `standards/STANDARDS_AUDIT_2026-02-07.md` (44 PARTIAL/FAIL findings)
**Status:** All tiers complete and build-verified (2026-02-07). Clang-tidy remediation: all production code fixed (2026-02-09).

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

## Clang-Tidy Automated Audit Remediation (2026-02-09)

**Source:** `docs/audits/CLANG_TIDY_AUDIT_2026-02-09.md` (1,251 warnings, 127 checks)

| Phase | Scope | Count | Commit |
|-------|-------|-------|--------|
| P1 | Safety-critical: widening mult, misplaced cast, missing default, narrowing conversions | 32 | `4407d35` — HW verified (0 errors/117K reads) |
| P2 | Auto-fix: uppercase suffixes, void args, nullptr, bool literals, using | 354 | `692da81` |
| P3 | Magic numbers: named constexpr for production code, NOLINT for IVP test | 275 | `5ee49b9` |
| P4 | Missing braces: JSF AV Rule 59, all if/for/while bodies | 170 | `28dd3dc` |
| P5 | C-style casts: static_cast/reinterpret_cast per JSF AV Rule 185 | ~80 | `66eb7cb` |
| P5b | Identifier naming: camelCase locals/params across all production files | 162 | pending |
| P5c | Implicit bool conversion: explicit `!= 0`/`!= nullptr` in production code | 41 | pending |
| P5d | Uninitialized variables: init at declaration in production code | 51 | pending |
| P5e | Function decomposition: 9 production functions under 60-line limit | 9 | pending |
| P5f | Math parentheses: check disabled per JSF AV Rule 213 exemption (LL Entry 26) | 65 | N/A |

### Remaining (IVP Test Code Only)

All production code (flight-critical, flight-support, ground) is now fully remediated. The only remaining findings are in IVP test functions in `main.cpp`:

| Category | Count | Rationale |
|----------|-------|-----------|
| Identifier naming (IVP) | ~100 | IVP test functions — will be stripped for production |
| Function size (IVP) | ~20 | IVP test functions — will be stripped |
| Cognitive complexity (IVP) | ~12 | IVP test functions — inherently complex test harness |
| Magic numbers (IVP) | varies | NOLINT with `// IVP test — will be stripped` |

### IVP Test Code Strategy

IVP test functions in `main.cpp` received NOLINT annotations (magic numbers only) rather than full remediation. This code is temporary test harness — it will be stripped or refactored when the flight state machine replaces the IVP dispatcher. Braces (P4) and casts (P5) were applied everywhere including IVP code since those are mechanical and improve safety regardless of code lifetime.

---

## GPS snprintf Mitigation (Fix C17 detail)

3 bounded `snprintf` calls in `gps_pa1010d.cpp` for NMEA command formatting. Documented MISRA deviation. Migration path: custom formatters or u-blox UBX binary protocol.
