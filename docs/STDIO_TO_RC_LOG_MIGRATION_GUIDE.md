# `<stdio.h>` → `rc_log` Migration Guide

**Audience:** Anyone familiar with `<stdio.h>` who needs to write new code or migrate existing code in the RocketChip project.

**Why this exists:** The project is migrating off `<stdio.h>` per `docs/decisions/STDIO_REPLACEMENT_PLAN.md`. This doc is the cheat-sheet: for each common stdio pattern, here's what to use instead, with side-by-side examples.

**Status of the replacement infrastructure as of 2026-05-16:**
- `rc::rc_log(fmt, ...)` — **available** as of R-5 Unit B (commit 5f2a805). printf-shape, 128-byte fixed buffer, drop-on-overflow, non-blocking USB CDC sink via ring buffer. Use this for any new `printf`-shape call. See `include/rocketchip/rc_log.h` for the contract.
- ETL 20.47.1 vendored at `EXTERNAL/etl-20.47.1/etl/`. **Most callers should use rc_log, not ETL directly.** ETL is the formatting backend rc_log dispatches to; project-side code calling ETL directly is reserved for callers that need typed buffer building (e.g., the GPS PMTK const-array migration in R-2/Tier 2 — see below).
- Pre-commit gate (`scripts/hooks/pre-commit` + `stdio_allowlist.txt`) — **active**. New `<stdio.h>` includes outside the allowlist are blocked. The allowlist deprecates at Unit J when it drains to empty.

If you're writing **new code** today: use `rc::rc_log(fmt, ...)` for printf-shape logging. Format-spec surface supported documented in `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md` — covers the 13 patterns the project actually uses (the 99.9%-coverage set).

If you're migrating **existing code**: wait for the dedicated R-5 tier sessions (Units C-J of `parsed-soaring-popcorn.md`) unless the user explicitly directs otherwise. The migration order is risk-tier ordered (proving ground → drivers → safety/diag → telemetry/AO → CLI/dashboard → cleanup).

---

## Quick reference table

| stdio function | Replacement | Status | Notes |
|---|---|---|---|
| `printf("fmt", args)` | `rc::rc_log("fmt", args)` | **available** | Same format-string surface; routes through bounded log channel. 13 supported specs per Unit A inventory. |
| `printf(...)` in flight loop or hot path | (don't — log via telemetry instead) | guidance | Hot-path logging isn't `rc_log`'s job either; consider whether you really need this. |
| `printf(...)` (debug-only) | `DBG_PRINT(...)` (existing macro) | **available** | DBG_PRINT backend will route through `rc_log` after Unit B's macro repoint (planned next commit). Surface stays identical for callers. |
| `snprintf(buf, sizeof buf, "fmt", args)` | (case-by-case — see below) | available now | If output is for logging, use `rc::rc_log` instead. If you genuinely need a string in a buffer, use `etl::to_string(value, str)` for single-value or `etl::string_stream` + `<<` operators for multi-arg. The proposed `etl::format(buf, "x={}", x, y)` API does NOT exist as written — ETL's actual API is `etl::format_to(out_iter, "x={}", x, y)` (post-2025-12-13 release; we vendored 20.47.1 which has it). For most callsites, `rc_log` removes the need to touch ETL directly. |
| `snprintf(...)` for fixed PMTK / protocol commands | precomputed `static constexpr char[]` arrays | available now | R-2 absorbed into R-5b (Tier 2 drivers). PMTK strings with `constexpr` checksum verification via `static_assert`. |
| `getchar() / getchar_timeout_us(...)` | `rc::rc_cli_getchar()` (existing — uses `pico/stdio.h` indirectly but not `<stdio.h>`) | available now | The CLI path already abstracts this. |
| `puts("string")` | `rc::rc_log("string\n")` | **available** | The `\n` is explicit (puts adds it; rc_log doesn't). |
| `putchar('c')` | `rc::rc_log("%c", 'c')` or direct `tud_cdc_write_char()` | mixed | For single-character output (LED feedback, prompts) the direct path is fine. |
| `fprintf(stderr, ...)` | `rc::rc_log(...)` (project doesn't distinguish severities yet) | available | No separate stderr in firmware. Severity-tagged variants (`rc_log_err`, `rc_log_dbg`) are post-Unit-J enhancement work. |
| `fopen / fread / fwrite / fclose` | `flash_log_*` API or `pico/flash` | already in place | We don't use stdio file I/O; flash is `flash_safe_execute` + custom format. |
| `scanf` / `sscanf` family | `etl::string_view` + `etl::to_value` / hand-written parser | available now | Parsing untrusted input is a separate safety concern; prefer explicit per-field parsing. |
| `perror(...)` | `rc::rc_log("%s: %s\n", ctx, error_name)` | available | We don't use `errno` anyway (banned by AV Rule 17), so `perror` doesn't translate directly. |
| `FILE*` | n/a | — | We have no `FILE*` users. Don't introduce. |

---

## Worked examples

### printf → rc_log

**Before:**
```cpp
#include <stdio.h>
// ...
printf("[FD] PYRO FIRED: DROGUE\n");
printf("Sensor count = %d, ESKF status = %s\n", count, status_name);
```

**After:**
```cpp
#include "rocketchip/rc_log.h"
// ...
rc::rc_log("[FD] PYRO FIRED: DROGUE\n");
rc::rc_log("Sensor count = %d, ESKF status = %s\n", count, status_name);
```

Format-string surface is identical — argument list, conversion specifiers, escape sequences all work the same. The difference is what's underneath: `rc_log` formats into a fixed-size stack buffer via ETL, then dispatches the result to the configured sink (USB CDC by default). No libc, no implementation-defined behavior, no allocation.

**Truncation behavior:** if your format produces more than the per-call buffer size (default 128 bytes), the message is truncated with a `...\n` marker. Don't rely on `rc_log` for "I need 500 chars of output" — break the message into multiple calls.

---

### snprintf → etl::to_string / etl::format

**Before:**
```cpp
#include <stdio.h>
char buf[32];
int len = snprintf(buf, sizeof buf, "x=%d y=%d", x, y);
// use buf...
```

**After (most cases — if output is for logging):**
```cpp
#include "rocketchip/rc_log.h"
rc::rc_log("x=%d y=%d", x, y);
// no buffer needed — output goes directly to USB CDC ring
```

**After (when you genuinely need a string in a buffer, single value):**
```cpp
#include "etl/to_string.h"
etl::string<32> buf;
etl::to_string(x, buf);   // single integer/float/etc.
// use buf.c_str() / buf.data()...
```

**After (when you genuinely need a string in a buffer, multi-arg):**
```cpp
#include "etl/string.h"
#include "etl/format.h"
etl::string<32> buf;
auto out_iter = etl::back_inserter(buf);
etl::format_to(out_iter, "x={} y={}", x, y);   // {} placeholders
// use buf.c_str() / buf.data()...
```

**API correction note (2026-05-16):** ETL's actual multi-arg API is `etl::format_to(out_iterator, "fmt", args...)`, not `etl::format(buf, "fmt", args...)`. The prior version of this guide showed the wrong signature; corrected by R-5 Unit B step 7 commit after the ETL CLEAN verification + vendoring at `EXTERNAL/etl-20.47.1/` confirmed the actual API surface. The `etl::format_to` function landed via PR #1204 (merged 2025-12-13); our pinned 20.47.1 release (2026-04-05) includes it.

**ETL format-string differences from printf:**
- ETL uses Python-style `{}` placeholders, not `%d` / `%s`.
- Field width / precision spec is `{:5}` / `{:.2f}` style.
- Hex is `{:x}`, padded hex is `{:08x}`.
- ETL has no `%n` (which is banned by MISRA anyway, so no loss).

**Float-formatting note:** rc_log uses a hand-rolled float formatter (NOT ETL's) that matches libc printf byte-for-byte for IEEE 754 round-half-to-even semantics. ETL's float formatter uses round-half-away-from-zero — that diverges from libc and from ground-side parsers' expectations. If you call `etl::to_string` directly on a float, you get ETL's rounding; if you call `rc_log("%f", v)`, you get libc-matching rounding. Prefer `rc_log` for any float output that downstream parsers will read.

---

### snprintf for fixed protocol strings → const arrays (R-2 pattern)

**Before:**
```cpp
char cmd[32];
snprintf(cmd, sizeof cmd, "$PMTK220,1000*1F\r\n");
i2c_write(GPS_ADDR, cmd, strlen(cmd));
```

**After:**
```cpp
static constexpr char kPmtkSet10Hz[] = "$PMTK220,1000*1F\r\n";
i2c_write(GPS_ADDR, kPmtkSet10Hz, sizeof kPmtkSet10Hz - 1);  // -1 to skip \0
```

If the command has a single runtime field (e.g., a checksum), keep the const-prefix + write the field separately, rather than reaching for `snprintf`:

```cpp
static constexpr char kPmtkPrefix[] = "$PMTK220,";
i2c_write(addr, kPmtkPrefix, sizeof kPmtkPrefix - 1);
etl::string<8> rate_str;
etl::to_string(rate_ms, rate_str);
i2c_write(addr, rate_str.c_str(), rate_str.size());
i2c_write(addr, "\r\n", 2);
```

This is the R-2 pattern for GPS PMTK commands; same approach applies wherever you have a fixed protocol template + a small variable field.

---

### DBG_PRINT (debug-only logging) — backend swap, surface stays the same

**Existing pattern (still the right pattern):**
```cpp
DBG_PRINT("Boot: phase %d\n", phase);
DBG_ERROR("ESKF divergence detected: |v|=%.2f\n", v_norm);
```

These compile to `((void)0)` under `NDEBUG` per `standards/DEBUG_OUTPUT.md`. **Keep using `DBG_PRINT` / `DBG_ERROR` as-is for new code.** When the R-5 refactor lands, the macro's backend swaps from `printf` to `rc_log_dbg` invisibly; your code doesn't change.

---

### Handler-context printing (fault handler, ISR)

The fault handler (`memmanage_fault_handler`) **does not print anything** — it writes the crash record to preserved SRAM and triggers reset. The post-boot `health_monitor_init` does the printing for the prior fault, and that runs in normal Thread mode where `rc_log` is fine.

If you find yourself wanting to print from an ISR or no-stack-safe context: **stop and reconsider.** ISR printing is almost never the right answer. Set a flag or push to a queue; the printing happens in normal context later.

For the rare case where direct serial output really is needed and no other path will do: use `tud_cdc_write` directly with a precomputed `const char[]` literal (no formatting, no allocation, no `rc_log`). The few callsites that need this are explicitly documented per-case.

---

### getchar / CLI input

The CLI path already abstracts this:

```cpp
// Existing pattern — keep using
int c = rc::rc_cli_getchar();
if (c != PICO_ERROR_TIMEOUT) {
    handle_command(static_cast<char>(c));
}
```

`rc_cli_getchar` uses the Pico SDK's `getchar_timeout_us` internally. That SDK function transitively pulls in `<stdio.h>` headers right now, but isolating its use to a single project file (`rc_os.cpp` or similar) means migration is one place, not 50.

---

## Format-specifier translation reference

When migrating from printf-style to ETL `{}`-style format strings:

| printf | ETL format | Notes |
|---|---|---|
| `%d`, `%i` | `{}` (auto-detected for int) | |
| `%u`, `%lu` | `{}` (auto for uint, ulong) | |
| `%x`, `%08x` | `{:x}`, `{:08x}` | Hex; uppercase is `{:X}`. |
| `%f`, `%.2f` | `{}`, `{:.2f}` | Float precision via `{:.<n>f}`. |
| `%e`, `%.3e` | `{:e}`, `{:.3e}` | Scientific. |
| `%s` | `{}` (for char*, string types) | |
| `%c` | `{}` (for char) | |
| `%p` | `{:p}` | Pointer hex. |
| `%%` | `{{` `}}` | Literal `{` is `{{`; literal `}` is `}}`. |
| `%n` | (unavailable, MISRA-banned) | Don't migrate; refactor caller. |
| `*` width via arg | `{:{}}` | E.g., `"{:{}d}"` for width-from-argument. |

---

## What to do RIGHT NOW (before the dedicated R-5 sessions land)

1. **For new code**: don't add new `<stdio.h>` includes. The pre-commit hook blocks it. Use:
   - `DBG_PRINT` / `DBG_ERROR` for debug-only messages (compiles out for release).
   - `etl::to_string` / `etl::format` for any string-building (allocation-free).
   - `const char[]` arrays for fixed-string output.
   - Hold the line on actual `rc_log` calls until the infrastructure lands; if you absolutely need a `printf`-like call before then, ask the user.
2. **For existing code that already has `<stdio.h>`**: leave it alone unless the user explicitly directs migration. The dedicated R-5 sessions will work through the files in the documented order.
3. **For surfacing issues**: if you find a callsite where the planned migration won't work cleanly, note it in `docs/PROBLEM_REPORTS.md` so it surfaces in the R-5 dedicated session.

---

## Open design questions

Most pre-Unit-B questions closed by 2026-05-16:

- ~~**Exact ETL version + header subset**~~ — RESOLVED. Vendored ETL 20.47.1 full upstream tree at `EXTERNAL/etl-20.47.1/etl/`. Acceptance criterion (no `<stdio.h>` / `vsnprintf` in format+to_string subtree) verified clean.
- ~~**Compile-time format-string validation**~~ — RESOLVED. Uses GCC `__attribute__((format(printf, 1, 2)))` on `rc_log` declaration. Mismatched format-spec vs argument types is a `-Wformat -Werror=format` build error.

Deferred to post-Unit-J (after migration completes):

- **`rc_log` severity model** — three levels (info/warn/err)? Or more? Currently single severity. DBG_PRINT macro distinction is compile-out vs always-on, not runtime severity.
- **Telemetry routing** — does `rc_log` also publish to MAVLink when station is connected, or stay USB CDC only? Currently USB CDC only.
- **Throughput characterization** — measure `rc_log` cost on bench; document the cycle budget so callers can budget appropriately. Binary-size measurement captured at Unit B + Unit J close.

These don't block migration — they're enhancements that come after the allowlist drains.

---

## See also

- `docs/decisions/STDIO_REPLACEMENT_PLAN.md` — the design + scope + work breakdown.
- `scripts/hooks/pre-commit` + `scripts/hooks/stdio_allowlist.txt` — the no-new-`<stdio.h>` gate.
- `standards/CODING_STANDARDS.md` — JSF / MISRA / JPL references.
- `standards/DEBUG_OUTPUT.md` — the `DBG_PRINT` macro (backend swap pending).
