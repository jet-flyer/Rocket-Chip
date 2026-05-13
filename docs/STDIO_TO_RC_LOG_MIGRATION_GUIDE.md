# `<stdio.h>` → `rc_log` Migration Guide

**Audience:** Anyone familiar with `<stdio.h>` who needs to write new code or migrate existing code in the RocketChip project.

**Why this exists:** The project is migrating off `<stdio.h>` per `docs/decisions/STDIO_REPLACEMENT_PLAN.md`. This doc is the cheat-sheet: for each common stdio pattern, here's what to use instead, with side-by-side examples.

**Status of the replacement infrastructure as of 2026-05-13:**
- `rc_log` family — **planned, not yet implemented** (lands in the dedicated R-5 session).
- `etl::to_string` / `etl::format` — **available via the existing ETL dependency** (header-only); usable today.
- Pre-commit gate (`scripts/hooks/pre-commit` + `stdio_allowlist.txt`) — **active** as of 2026-05-13. New `<stdio.h>` includes outside the allowlist are blocked.

If you're writing **new code** today: use the patterns marked "available now" below. The `rc_log` patterns will be filled in as the infrastructure lands; for new code that would have used `printf`, prefer `etl::to_string` + direct USB CDC write, or hold the line on the function until `rc_log` is available.

If you're migrating **existing code**: wait for the dedicated R-5 sessions unless the user explicitly directs otherwise. The migration order is in `STDIO_REPLACEMENT_PLAN.md`.

---

## Quick reference table

| stdio function | Replacement | Status | Notes |
|---|---|---|---|
| `printf("fmt", args)` | `rc::rc_log("fmt", args)` | planned | Same format-string surface; routes through bounded log channel. |
| `printf(...)` in flight loop or hot path | (don't — log via telemetry instead) | guidance | Hot-path logging isn't `rc_log`'s job either; consider whether you really need this. |
| `printf(...)` (debug-only) | `DBG_PRINT(...)` (existing macro, backend swap pending) | available now | Existing macro stays, just rebuilt on `rc_log_dbg` under the hood. |
| `snprintf(buf, sizeof buf, "fmt", args)` | `etl::to_string(value, buf)` for single value; `etl::format` for multi-arg | available now | ETL is header-only, allocation-free, deterministic. |
| `snprintf(...)` for fixed PMTK / protocol commands | precomputed `static const char[]` arrays | available now | See R-2 — `gps_pa1010d.cpp` / `gps_uart.cpp` already do this. |
| `getchar() / getchar_timeout_us(...)` | `rc::rc_cli_getchar()` (existing — uses `pico/stdio.h` indirectly but not `<stdio.h>`) | available now | The CLI path already abstracts this. |
| `puts("string")` | `rc::rc_log("string\n")` | planned | The `\n` is explicit (puts adds it; rc_log doesn't). |
| `putchar('c')` | direct `tud_cdc_write_char()` or `rc::rc_log("%c", 'c')` | mixed | For single-character output (LED feedback, prompts) the direct path is fine. |
| `fprintf(stderr, ...)` | `rc::rc_log_err(...)` | planned | We don't have separate stderr; `rc_log_err` tags as error severity. |
| `fopen / fread / fwrite / fclose` | `flash_log_*` API or `pico/flash` | already in place | We don't use stdio file I/O; flash is `flash_safe_execute` + custom format. |
| `scanf` / `sscanf` family | `etl::string_view` + `etl::to_value` / hand-written parser | available now | Parsing untrusted input is a separate safety concern; prefer explicit per-field parsing. |
| `perror(...)` | `rc::rc_log_err("%s: %s\n", ctx, error_name)` | planned | We don't use `errno` anyway (banned by AV Rule 17), so `perror` doesn't translate directly. |
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

**After (multi-arg format):**
```cpp
#include "etl/format_spec.h"
#include "etl/string.h"
etl::string<32> buf;
etl::format(buf, "x={} y={}", x, y);   // ETL format spec is `{}`-style, not `%d`
// use buf.c_str() / buf.data()...
```

**After (single value, simpler):**
```cpp
#include "etl/to_string.h"
etl::string<32> buf;
etl::to_string(x, buf);   // for a single integer/float/etc.
```

**ETL format-string differences from printf:**
- ETL uses Python-style `{}` placeholders, not `%d` / `%s`.
- Field width / precision spec is `{:5d}` / `{:.2f}` style.
- Hex is `{:x}`, padded hex is `{:08x}`.
- ETL has no `%n` (which is banned by MISRA anyway, so no loss).

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

## Open design questions (resolved in the dedicated R-5 sessions)

These are the questions the dedicated sessions will close — listed here so a curious reader knows they're tracked, not forgotten:

- **Exact ETL version + header subset** — which ETL headers we adopt and how they're vendored.
- **`rc_log` severity model** — three levels (info/warn/err)? Or more?
- **Compile-time format-string validation** — does ETL's pattern give us this, or do we need a separate check?
- **Telemetry routing** — does `rc_log` also publish to MAVLink when station is connected, or stay USB CDC only?
- **Throughput characterization** — measure `rc_log` cost on bench; document the cycle budget so callers can budget appropriately.

---

## See also

- `docs/decisions/STDIO_REPLACEMENT_PLAN.md` — the design + scope + work breakdown.
- `scripts/hooks/pre-commit` + `scripts/hooks/stdio_allowlist.txt` — the no-new-`<stdio.h>` gate.
- `standards/CODING_STANDARDS.md` — JSF / MISRA / JPL references.
- `standards/DEBUG_OUTPUT.md` — the `DBG_PRINT` macro (backend swap pending).
