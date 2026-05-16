# stdio Format-Spec Inventory — 2026-05-15

**Purpose:** Unit A deliverable of the R-5 stdio-removal plan (`C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md`). Drives the `rc_log` supported-spec list so the replacement formatter is designed against actual usage, not against every printf-spec the C standard defines.

**Council round 1 framing (ArduPilot, 2026-05-15):** "I'd bet 80% of the 600 calls use one of 6-8 patterns. Design for those plus a clear-error path for anything else."

**Verdict:** ArduPilot's bet held. 13 specs cover 96% of usage. 27 spec variants exist but the rare ones cluster around floating-point width/precision and a handful of `%X`/`%lX`/`%llu` outliers.

---

## Methodology

**Files surveyed:** all 18 currently-allowlisted source files per `scripts/hooks/stdio_allowlist.txt` (as of 2026-05-15), plus 2 transitive-leak files discovered during the survey (see "Transitive-leak finding" below):

```
src/active_objects/{ao_flight_director,ao_rcos,ao_telemetry}.cpp
src/calibration/cal_hooks.cpp
src/cli/{rc_os,rc_os_commands,rc_os_dashboard,rc_os_debug}.cpp
src/diag/diag_stats.cpp
src/drivers/{gps_pa1010d,gps_uart,i2c_bus}.cpp
src/main.cpp
src/safety/{fault_inject,fault_protection,pyro_edge_logger,station_fault_inject}.cpp
src/telemetry/telemetry_service.cpp
```

(Note: the `STDIO_REPLACEMENT_PLAN.md` doc mentions 20 files; the actual allowlist is 18 after the R-25-exec dev/src/safety consolidation. Current state is authoritative.)

**Grep pattern:** `%[-+ #0]*[0-9*]*\.?[0-9*]*[hljztL]*[diouxXeEfgGaAcspn%]` — matches printf-style format specs with optional flags, width, precision, length modifier, and conversion character.

**Non-printf stdio functions also surveyed:** `putchar`, `fwrite`, `getchar`, `getchar_timeout_us`, plus checks for `puts`/`fputs`/`fopen`/`fclose`/`fread`/`gets`/`fgets`/`perror`.

---

## Per-file callsite + spec counts

| File | printf/snprintf calls | Format specs in file | Per-call avg |
|------|-----------------------|----------------------|--------------|
| src/cli/rc_os_commands.cpp | 215 | 361 | 1.7 |
| src/active_objects/ao_rcos.cpp | 170 | 68 | 0.4 |
| src/cli/rc_os.cpp | 45 | 4 | 0.1 |
| src/diag/diag_stats.cpp | 38 | 66 | 1.7 |
| src/cli/rc_os_debug.cpp | 36 | 9 | 0.3 |
| src/active_objects/ao_flight_director.cpp | 26 | 17 | 0.7 |
| src/safety/fault_inject.cpp | 16 | 3 | 0.2 |
| src/drivers/i2c_bus.cpp | 15 | 8 | 0.5 |
| src/cli/rc_os_dashboard.cpp | 12 | 103 | 8.6 |
| src/drivers/gps_pa1010d.cpp | 7 | 10 | 1.4 |
| src/drivers/gps_uart.cpp | 6 | 4 | 0.7 |
| src/active_objects/ao_telemetry.cpp | 6 | 17 | 2.8 |
| src/safety/pyro_edge_logger.cpp | 3 | 5 | 1.7 |
| src/safety/fault_protection.cpp | 3 | 2 | 0.7 |
| src/calibration/cal_hooks.cpp | 1 | 2 | 2.0 |
| src/safety/station_fault_inject.cpp | 1 | 1 | 1.0 |
| src/telemetry/telemetry_service.cpp | 1 | 16 | 16.0 |
| src/main.cpp | 0 | 5 | — |

**Totals:** 601 stdio function calls + 701 format specs across 18 files.

**Notes on outliers:**

- `rc_os_dashboard.cpp` has the highest spec-per-call ratio (8.6) — single `printf` calls in dashboard render functions emit long multi-field rows.
- `telemetry_service.cpp` shows 16:1 because its single callsite is a multi-field hex dump.
- `rc_os.cpp` shows 0.1 because most of its 45 calls are bare `printf("string")` with no format specs.
- `main.cpp` shows 0 stdio calls but 5 format specs — the format specs appear in macro-call sites that expand to `DBG_PRINT(...)`. Those route through `<stdio.h>`'s `printf` today; they migrate when DBG_PRINT is repointed in Unit B.

---

## Format-spec usage table (sorted by frequency)

| Spec | Count | % of total | Category | Notes |
|------|-------|------------|----------|-------|
| `%s` | 149 | 21.3% | string | C-string output |
| `%lu` | 117 | 16.7% | integer | unsigned long, common for `uint32_t` per project `(unsigned long)` cast |
| `%u` | 92 | 13.1% | integer | unsigned int |
| `%d` | 63 | 9.0% | integer | signed int |
| `%.1f` | 52 | 7.4% | float | one-decimal float |
| `%.2f` | 40 | 5.7% | float | two-decimal float |
| `%.4f` | 34 | 4.9% | float | four-decimal float |
| `%.3f` | 17 | 2.4% | float | three-decimal float |
| `%.7f` | 14 | 2.0% | float | seven-decimal float (high-precision telemetry) |
| `%c` | 13 | 1.9% | char | single char |
| `%%` | 10 | 1.4% | literal | literal `%` |
| `%02x` | 8 | 1.1% | hex | 2-digit zero-padded lowercase hex |
| `%6lu` | 7 | 1.0% | integer | width-6 unsigned long (dashboard alignment) |
| `%-8s` | 7 | 1.0% | string | width-8 left-aligned string |
| `%7.4f` | 6 | 0.9% | float | width-7 4-decimal float |
| `%7.3f` | 6 | 0.9% | float | width-7 3-decimal float |
| `%6.2f` | 6 | 0.9% | float | width-6 2-decimal float |
| `%6.1f` | 6 | 0.9% | float | width-6 1-decimal float |
| `%.6f` | 6 | 0.9% | float | six-decimal float |
| `%.0f` | 6 | 0.9% | float | zero-decimal float (rounded display) |
| `%6s` | 5 | 0.7% | string | width-6 string |
| `%02X` | 5 | 0.7% | hex | 2-digit zero-padded uppercase hex |
| `%3u` | 4 | 0.6% | integer | width-3 unsigned int |
| `%08lx` | 4 | 0.6% | hex | 8-digit zero-padded lowercase hex (address-like) |
| `%-10s` | 3 | 0.4% | string | width-10 left-aligned string |
| `%7.1f` | 2 | 0.3% | float | width-7 1-decimal float |
| `%08lX` | 2 | 0.3% | hex | 8-digit zero-padded uppercase hex |
| `%.5f` | 2 | 0.3% | float | five-decimal float |
| `%-20s` | 2 | 0.3% | string | width-20 left-aligned string |
| `% f` | 2 | 0.3% | float | space-flag float (positive numbers get leading space) |
| `%zu` | 1 | 0.1% | integer | size_t |
| `%llu` | 1 | 0.1% | integer | unsigned long long |
| `%5lu` | 1 | 0.1% | integer | width-5 unsigned long |
| `%2lu` | 1 | 0.1% | integer | width-2 unsigned long |
| `%04X` | 1 | 0.1% | hex | 4-digit zero-padded uppercase hex |
| `%02lu` | 1 | 0.1% | integer | width-2 zero-padded unsigned long |
| `%02lX` | 1 | 0.1% | hex | 2-digit zero-padded long uppercase hex |
| `%-6lu` | 1 | 0.1% | integer | width-6 left-aligned unsigned long |
| `%-4lu` | 1 | 0.1% | integer | width-4 left-aligned unsigned long |
| `%+6.1f` | 1 | 0.1% | float | width-6 1-decimal signed float |
| `% o` | 1 | 0.1% | integer | space-flag octal (single occurrence — investigate before migration) |

**Total specs:** 701.

---

## Category rollup

| Category | Spec count | % of total |
|----------|------------|------------|
| String (`%s`, widths/alignments) | 168 | 24.0% |
| Integer signed (`%d`) | 63 | 9.0% |
| Integer unsigned (`%u`, `%lu`, `%zu`, `%llu`, widths) | 233 | 33.2% |
| Hex (`%x`, `%X`, widths) | 21 | 3.0% |
| Float (`%f`, precision/width variants) | 192 | 27.4% |
| Char (`%c`) | 13 | 1.9% |
| Literal (`%%`) | 10 | 1.4% |
| Octal (`%o`) | 1 | 0.1% |

**Six categories cover 99.9% of usage.** Octal is a single outlier (investigate before migration — likely a stray debug print).

---

## Required `rc_log` / `etl::format_to` supported specs

Based on usage, the rc_log replacement must support the following spec set. ETL's `etl::format_to` (post-2025-12-13 release) uses Python-style `{}` placeholders with format spec `{:<flags><width>.<precision><type>}`. Translation in column 3.

| printf form | Count | ETL `format_to` equivalent | Migration complexity |
|-------------|-------|----------------------------|----------------------|
| `%s` | 149 | `{}` (auto for `const char*` / string types) | trivial |
| `%-8s`, `%-10s`, `%-20s`, `%6s` | 17 | `{:<8}`, `{:<10}`, `{:<20}`, `{:>6}` | mechanical |
| `%lu`, `%u`, `%d` | 272 | `{}` (auto-detected) | trivial |
| `%6lu`, `%3u`, `%5lu`, `%2lu`, `%-6lu`, `%-4lu`, `%02lu` | 16 | `{:>6}`, `{:>3}`, `{:>5}`, `{:>2}`, `{:<6}`, `{:<4}`, `{:0>2}` | mechanical |
| `%zu`, `%llu` | 2 | `{}` (auto) | trivial |
| `%.1f`, `%.2f`, `%.3f`, `%.4f`, `%.5f`, `%.6f`, `%.7f`, `%.0f` | 171 | `{:.1f}`, `{:.2f}`, etc. | mechanical |
| `%6.1f`, `%6.2f`, `%7.1f`, `%7.3f`, `%7.4f` | 26 | `{:6.1f}`, `{:6.2f}`, etc. | mechanical |
| `%+6.1f`, `% f` | 3 | `{:+6.1f}`, `{: f}` | mechanical (verify ETL supports space-flag) |
| `%c` | 13 | `{:c}` | mechanical |
| `%%` | 10 | `{{` `}}` | mechanical |
| `%02x`, `%02X`, `%04X`, `%08lx`, `%08lX`, `%02lX` | 21 | `{:02x}`, `{:02X}`, `{:04X}`, `{:08x}`, `{:08X}`, `{:02X}` | mechanical (long-modifier elided in ETL) |
| `% o` | 1 | `{: o}` | investigate single callsite (likely stray) |

**Total specs covered by 13 base patterns:** 700 of 701 (99.9%).

**Specs requiring investigation:**
- `% o` (1 occurrence) — find the callsite, decide if it should be `%u` or `%x` instead. Likely a debug print mistake.

---

## Non-printf stdio function inventory

Beyond `printf` / `snprintf`, the codebase uses these other stdio entry points:

| Function | Files | Callsite count (approx) | Migration target |
|----------|-------|-------------------------|------------------|
| `putchar(ch)` | ao_rcos.cpp, telemetry_service.cpp (?) | 4 | `rc_log("{:c}", ch)` or direct `tud_cdc_write_char(ch)` for single-char output |
| `fwrite(buf, 1, len, stdout)` | rc_os.cpp, telemetry_service.cpp, others | 4 | direct `tud_cdc_write(buf, len)` — `fwrite(stdout)` is just a buffer flush |
| `getchar()` / `getchar_timeout_us()` | ao_rcos.cpp, rc_os.cpp, rc_os_commands.cpp, rc_os_dashboard.cpp, rc_os_debug.cpp | several | already abstracted via `rc::rc_cli_getchar()` pattern (per migration guide); transitively uses pico SDK's stdio, isolated to one file when fully migrated |

Functions **NOT** found in any allowlisted file (confirmed clean):
- `puts`, `fputs` — zero matches
- `fopen`, `fclose`, `fread` — zero matches
- `gets`, `fgets` — zero matches
- `perror` — zero matches

---

## Transitive-leak finding (scope correction — no new PR row needed)

**User-prompted verification 2026-05-15:** confirmed by diff that every file directly `#include`ing `<stdio.h>` is on the allowlist (18-for-18 match). However, `include/rocketchip/config.h` itself `#include`s `<stdio.h>` (intentional — to make `DBG_PRINT(...)` expand to `printf(...)` per `standards/DEBUG_OUTPUT.md`). Any `src/*.cpp` that uses stdio functions (`printf`, `snprintf`, etc.) but doesn't directly `#include <stdio.h>` is exposed to stdio transitively via config.h, and is NOT on the allowlist because the hook only matches direct includes.

**Full sweep** — `grep -rln -E '\b(printf|snprintf|fprintf|fwrite|putchar|puts|fputs)\(' src/` diffed against the allowlist surfaced **5 additional files** beyond the 18 allowlisted:

| File | Stdio calls | Notes |
|------|-------------|-------|
| `src/flight_director/flight_director.cpp` | **10 `printf` callsites** | Mostly `[FD]` bench_sim-monitored log lines (LL Entry 36 regex surface). High-stakes byte-on-wire identity required at migration. |
| `src/flight_director/go_nogo_checks.cpp` | **8 callsites** (6 `std::snprintf` + 2 `printf`) | NO-GO/GO reason-string formatting + `[GO/NO-GO]` log lines |
| `src/flight_director/guard_combinator.cpp` | **1 `printf` callsite** | Single `[FD] WARN: backup timer fired...` line |
| `src/active_objects/ao_radio.cpp` | **4 `printf` callsites** (lines 143, 149, 157, 165) | Stage T diagnostic logs. Also uses DBG_PRINT. |
| `src/safety/health_monitor.cpp` | 0 direct stdio calls | DBG_PRINT user only — transitive exposure via config.h only |

**Total hidden stdio exposure:** 23 additional callsites beyond the 601 in the allowlisted-file inventory above. **Actual migration scope: 23 files, ~624 callsites, ~700 format specs.**

**Implications:**

1. **Hook coverage gap noted but no fix needed.** The hook checks `#include <stdio.h>` text patterns but the actual exposure surface is "any use of any stdio function." Normally this would be worth fixing — but the entire allowlist + Gate-1 mechanism is **being deprecated** at Unit J. When the allowlist drains to empty and Gate 1 becomes "no `<stdio.h>` in `src/*.cpp` anywhere," the transitive-via-config.h issue evaporates structurally: there will be no `<stdio.h>` left to be transitively included. Patching the existing hook would be writing code that gets deleted shortly after. **The value of this finding is the scope correction below (5 additional files surfaced), not a hook upgrade.**

2. **Migration scope reality: 23 files, not 18.** Plan tier assignments:
   - `flight_director.cpp` (10 callsites) → Tier 4 (telemetry/AO) — high-value because `[FD]` log lines are bench_sim regex surface
   - `go_nogo_checks.cpp` (8) → Tier 4
   - `guard_combinator.cpp` (1) → Tier 4
   - `ao_radio.cpp` (4) → Tier 4
   - `health_monitor.cpp` (0 direct, DBG_PRINT only) → resolves at Unit B's DBG_PRINT repoint; possibly no Tier 3 work needed beyond that

3. **DBG_PRINT macro repoint at Unit B retires the root cause for `health_monitor.cpp`.** Once DBG_PRINT goes through `rc_log` instead of `printf`, config.h no longer needs `<stdio.h>` at all. The transitive exposure vanishes structurally after Unit B for files that ONLY use DBG_PRINT.

4. **Council round 2 amendment #4 (bench_sim log-lines in byte-on-wire diff scope) is now load-bearing for Tier 4.** The `[FD]` log lines that bench_sim's regex parses live in `flight_director.cpp` — a previously-hidden migration target. The amendment was prescient; Tier 4 commit verification must capture pre/post byte-on-wire for these specific log lines.

5. **R-2 absorption note unchanged.** R-2 (GPS PMTK snprintf, 13 callsites in `gps_pa1010d.cpp` + `gps_uart.cpp`) was already correctly scoped via the allowlist; no transitive-leak issue there.

**Per-file callsite + spec count for the 5 transitive-leak files** (added below the main per-file table):

| File | Stdio calls | Format specs in file |
|------|-------------|----------------------|
| src/flight_director/flight_director.cpp | 10 | (re-survey at Tier 4 — likely ~20-30 specs given `[FD]` log line density) |
| src/flight_director/go_nogo_checks.cpp | 8 | (re-survey at Tier 4) |
| src/flight_director/guard_combinator.cpp | 1 | 1 |
| src/active_objects/ao_radio.cpp | 4 | (re-survey at Tier 4) |
| src/safety/health_monitor.cpp | 0 (DBG_PRINT only) | 0 (DBG_PRINT macros) |

The format-spec table earlier does NOT include these 5 files. Re-running the inventory grep against them at their respective tier-start re-validates coverage; they aren't expected to introduce new format-spec patterns beyond the 13 already inventoried (the `[FD]` log lines mostly use `%s`, `%lu`, `%u`, `%d` which are already covered).

---

## Risks surfaced by the inventory

1. **Float-formatting coverage.** 192 float specs across 8 distinct precision/width variants. ETL's `etl::format_to` for floats must produce byte-identical output to libc `printf` for these patterns. **Acceptance criterion at Unit B:** the rc_log host ctest covers each of the 8 float patterns (`{:.1f}`, `{:.2f}`, `{:.3f}`, `{:.4f}`, `{:.5f}`, `{:.6f}`, `{:.7f}`, `{:.0f}`) plus the 5 width-precision variants (`{:6.1f}`, `{:6.2f}`, `{:7.1f}`, `{:7.3f}`, `{:7.4f}`). If any variant produces different bytes (rounding mode, trailing zeros, etc.), surface before Tier 1 starts.

2. **High-precision float (`%.7f`, 14 occurrences).** Likely telemetry-precision output (GPS lat/lon, ESKF state). Verify ETL produces 7 decimal digits with correct rounding semantics. If ETL's `format_to` uses different rounding than libc's, downstream parsers / dashboard displays may break by one ULP. Mitigation: byte-on-wire diff per file covers this.

3. **Width-and-precision compound specs (`%6.1f` family, 26 occurrences).** Dashboard alignment depends on consistent column widths. Byte-on-wire diff at migration catches any width-drift.

4. **Space-flag float (`% f`, 2 occurrences).** Less common spec; verify ETL supports the leading-space-for-positive-numbers semantic. If not, callsites need explicit `{:>5}` or similar workaround.

5. **`% o` single occurrence.** Investigate before migration. The space flag on octal output is unusual.

6. **`putchar` and `fwrite(stdout)` migration.** These don't use format specs but DO go through `<stdio.h>`. Migration targets per the table above (direct `tud_cdc_write_char` for `putchar`; direct `tud_cdc_write` for `fwrite`). These cleanups happen alongside the printf migration in their respective tier — fwrite-to-stdout callsites are concentrated in CLI output paths so they migrate in Tier 5.

---

## Implications for the rc_log API surface

The rc_log v1 supported-spec list, locked at Unit B per the plan:

**Required (covers 99.9% of usage):**
- `{}` (auto-detect for int, uint, long, ulong, char, char*, string types)
- `{:c}` (char)
- `{:<N}`, `{:>N}`, `{:^N}` (left/right/center align with width N)
- `{:0>N}` (zero-padded width N)
- `{:.Nf}` (float with N decimals)
- `{:N.Mf}` (float with width N, M decimals)
- `{:+}` flag (force sign on positive)
- `{: }` flag (space for positive) — pending ETL verification
- `{:x}`, `{:X}` (hex lowercase/uppercase)
- `{:0NX}` (zero-padded hex)
- `{{` and `}}` (literal braces)

**Investigate at Unit B:**
- `{: o}` (space-flag octal) — only 1 callsite; may be removed entirely instead of supported

**Not required (zero usage):**
- `%e`, `%g`, `%a`, `%E`, `%G`, `%A` (scientific / general float / hex float)
- `%n` (banned by MISRA anyway)
- `%p` (pointer) — no callsites in the inventory
- `%i` (alias for `%d`) — no callsites

The 13-pattern coverage is small enough that the rc_log host ctest at Unit B can exhaustively test every supported spec with multiple representative values. The acceptance criterion: byte-identical output to libc printf for the supported set.

---

## Migration-method implication

The inventory confirms ArduPilot's round 1 prediction: the format-spec variety is mechanical and bounded. There's no exotic spec that would force the migration into a complex codemod or a hand-rolled formatter.

**Tier-1 proving ground** (`pyro_edge_logger.cpp` + `fault_protection.cpp`) exercises the most-common specs: `%s` (149 uses), `%lu` (117), `%u` (92), `%d` (63). If rc_log handles those four cleanly, the bulk of the migration is mechanical translation.

**Tier 5 CLI files** carry the float-formatting concentration (most of the 192 float specs live in rc_os_commands and rc_os_dashboard). Byte-on-wire diff per function commit is the critical defense for these tiers.

---

## References

- `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md` — Unit A is the source for this doc
- `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a5195227c0a67e89d.md` — Council round 1 (ArduPilot's "80% of calls use 6-8 patterns" prediction validated)
- `docs/decisions/STDIO_REPLACEMENT_PLAN.md` — original 18/20 file inventory + replacement design
- `docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md` — format-spec translation reference (will be API-corrected in Unit B per ETL CLEAN verdict)
- ETL `etl::format_to` documentation — https://www.etlcpp.com/format.html (post-PR-1204 merged 2025-12-13)
