# `<stdio.h>` Replacement Plan — Project-Wide

**Status:** Decision recorded. Refactor work deferred to a dedicated session.
**Established:** 2026-05-13 (Phase 8 Category 3 of the 2026-05-07 master standards audit).
**Supersedes:** R-5 / IO-1 in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07_REMEDIATION_PRELIMINARY.md` (which carried a "Ground classification + runtime lockout" rationale that we no longer accept; see §"Why" below).

---

## Decision

The project will **eliminate every `#include <stdio.h>` from the entire `src/` tree** — flight-critical, flight-support, ground, and dev code alike. No carve-outs by classification. The work is large enough to warrant its own dedicated session(s) outside this audit cycle, but two defenses land NOW in this audit:

1. A **pre-commit hook gate** blocking any *new* `<stdio.h>` include in `src/*.cpp` files outside the explicitly-allowlisted set of 18 files that already use it (`scripts/hooks/stdio_allowlist.txt`). This prevents the deviation set from growing.
2. A **migration guide** (`docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md`) that documents the project-owned replacements for each stdio function someone might reach for. New code uses the replacements from day one; existing code migrates incrementally.

The actual refactor (replacing the ~400-500 callsites across 18 files, building the `rc_log` channel + ETL adapter, dropping `<stdio.h>` from each file, shrinking the allowlist to empty) lands in dedicated R-5-redesign sessions after this audit cycle.

---

## Why (the safety rationale chain)

The previous R-5 / IO-1 framing accepted `<stdio.h>` in Ground- and dev-classified code on the rationale that those paths are runtime-locked-out when state != IDLE or compile-excluded from flight builds (`NOT_CERTIFIED_FOR_FLIGHT=ON`). That framing is being retired because:

1. **Unintentional proliferation risk.** A dev-only file's `#include <stdio.h>` can transit through header chains into flight-critical code if include hygiene slips. The runtime lockout protects the runtime, not the compile-time include graph.
2. **The original safety rationale is broader than "in flight."** JSF AV Rule 22 / MISRA-C:2012 Rule 21.6 / JPL Rule 1 (the chain at the root) don't ban stdio because it's slow or non-deterministic in flight — they ban it because **its behavior is implementation-defined in ~30 places per ISO/IEC 9899 Appendix J** (line buffering, float-format precision, locale handling, FILE pointer semantics, errno error reporting). Implementation-defined behavior is implementation-defined whether the code runs in IDLE or in BOOST.
3. **Lived experience.** The project has had stability / implementation issues with stdio functions in the past. The cost of "keeping it working" via the lockout discipline has been higher than the cost of just not using it.
4. **No deviations even in ground/dev** (user direction 2026-05-13). The deviation file should contain the rare exception, not normalized accommodation across half the codebase.

---

## What the standards actually say

**JSF AV C++ Rule 22** (Lockheed Martin, December 2005), `§4.5.1`:

> *"The input/output library `<stdio.h>` **shall not** be used."* — verbatim, page 19 of the JSF AV PDF. Sits in the band of rules 17–25 that all prohibit standard-library features with implementation-defined behavior: `errno`, `offsetof`, `<locale.h>`, `<setjmp>`, `<signal.h>`, `<stdio.h>`, `atof/atoi/atol`, `abort/exit/getenv/system`, `<time.h>`. No exception in the rule text.

**MISRA-C:2012 Rule 21.6** (descendant of MISRA-C:2004 Rule 124 which JSF Rule 22 cites):

> *"The Standard Library input/output functions shall not be used."* — **Required severity**, no built-in exception.

**MISRA-C++:2023 Rule 30.0.1** — same wording, still Required.

**JPL Institutional C Coding Standard (2009)** — does not name `<stdio.h>` explicitly in rules 1–31 (LOC-1 through LOC-4). Inherits the explicit ban through LOC-5 (MISRA-C:2004 *shall* compliance). The umbrella principle is **Rule 1**:

> *"All C code shall conform to the ISO/IEC 9899-1999(E) standard for the C programming language, **with no reliance on undefined or unspecified behavior.**"*

The ISO/IEC 9899 Appendix J list of implementation-defined behaviors includes most of `<stdio.h>`. So **JPL Rule 1 + `<stdio.h>` use are structurally incompatible** even without a specific named ban.

**Power of 10 (Holzmann/JPL, 2006)** — does NOT explicitly prohibit `<stdio.h>`. Rule 7 (return-value checking) actually uses `printf` as the example case where casting to `(void)` is acceptable. P10 simply doesn't address libraries at that specificity level.

**Project's standards-precedence rule** (per `standards/CODING_STANDARDS.md`): newer overrides older absent explicit override; on silent conflict, default to more recent OR more restrictive. JSF Rule 22 (2005) is older but more restrictive than P10's silence on the topic, and MISRA-C:2012 Rule 21.6 (newer) reaffirms it. **The ban is in force.**

---

## Replacement design — the `rc_log` channel + ETL formatting

This composite pattern draws from existing aerospace practice (NASA F', NASA cFS, ArduPilot HAL, ETL embedded library). See the next section for source-by-source attribution. Concrete design:

### `rc_log` — bounded log channel

A single project-owned function family:

```cpp
namespace rc {
    // Bounded write to the active log sink. Drops on overflow; never blocks.
    // Format spec is the same as printf, but routed through a controlled
    // implementation that does NOT use <stdio.h>.
    void rc_log(const char* fmt, ...);

    // Variant with severity tagging (later — see future expansion below).
    void rc_log_err(const char* fmt, ...);
    void rc_log_dbg(const char* fmt, ...);
}
```

Internally:

- Fixed-size stack buffer per call (e.g., 128 bytes — never heap).
- Formatting via **ETL** (`etl::format` family) which provides type-safe, allocation-free, deterministic conversion. No libc `vsnprintf` involvement.
- Output goes through a single dispatch function `rc_log_emit(const char* buf, size_t len)` that writes directly via `tud_cdc_write` (USB CDC) or to telemetry / `/dev/null` based on build configuration.
- Drop-on-overflow: if the format produces more than the buffer size, the message is truncated with a marker (e.g., `"...\n"`); never grows the buffer.
- Compile-time format-string validation where ETL supports it (C++20 `consteval`-style checks).

### ETL as the formatting backend

The project already uses ETL (Embedded Template Library) for several containers. Extending its use to `etl::format` / `etl::to_string` for formatting is a natural fit:

- Allocation-free (uses caller's buffer)
- Deterministic conversion (no locale, no float-format ambiguity beyond what the project explicitly configures)
- Header-only — no library-side ABI dependency
- Already vetted for safety-critical use (ETL specifically targets MISRA / AUTOSAR / safety-critical embedded)

### Compile-out for release builds

`DBG_PRINT` becomes `rc_log_dbg`, which compiles to `((void)0)` under `NDEBUG` (or whatever the project's release-build flag is). Same surface as today, different backend.

### Handler-context usage

For contexts where even `rc_log` is unsafe (fault handler, ISR, no-stack-push), direct `tud_cdc_write` plus a tiny inline integer-to-hex helper handles the few legitimately-needed-in-handler messages. The fault handler (`memmanage_fault_handler`) doesn't emit logs anyway — it writes the crash record and resets; the post-boot `health_monitor_init` logs the prior fault via `rc_log_dbg` like any other code.

---

## Existing-practice survey (what others do)

| Project | Pattern | Notes |
|---|---|---|
| **NASA F' (Flight Software Framework)** | `Fw::Logger::log()` channel + `Fw::String` (fixed-size, no allocation). Format conversion via type-safe operator overloads, not `%d` strings. | No `printf`, no `<stdio.h>`. Routes to whatever sink the deployment configured. |
| **NASA cFS (Core Flight System)** | `OS_printf()` — controlled abstraction over the underlying OS logger. Internally uses `vsnprintf` on a fixed buffer, but the specific libc impl is certified per mission. | Takes the deviation but documents/controls it strictly. |
| **ArduPilot** | `AP_HAL::UARTDriver::printf()` — routes through HAL with documented bounded behavior. | Allows `printf`-style call surface but the actual formatter is HAL-controlled. |
| **ETL (Embedded Template Library)** | `etl::format`, `etl::to_string`, `etl::string` — allocation-free, deterministic. | Many embedded projects use ETL specifically as the `<stdio.h>` alternative. Header-only. |
| **fmt / std::format** (C++20) | `fmt::format_to(buffer, ...)`, `std::format` — compile-time-validated format strings, no allocation when output goes to fixed buffer. | C++20+ projects use these. Possible future direction once we move past C++20 minima. |
| **Tagged log channel** (Memfault, NASA mission ops) | Numeric log-ID + binary payload at the device; the ground station formats. | Strongest for telemetry-constrained environments. Not our current need but worth noting. |

The `rc_log + ETL` composite chosen here matches the F' + cFS + ETL pattern: a project-owned log channel with allocation-free type-safe formatting.

---

## Migration scope estimate

**18 files** in `src/` currently include `<stdio.h>` (per `scripts/hooks/stdio_allowlist.txt`):

- `src/active_objects/ao_flight_director.cpp` (26 calls)
- `src/active_objects/ao_rcos.cpp` (175 calls — calibration wizards)
- `src/active_objects/ao_telemetry.cpp` (6 calls)
- `src/calibration/cal_hooks.cpp` (1 call)
- `src/cli/rc_os.cpp` (48 calls)
- `src/cli/rc_os_commands.cpp` (213 calls — biggest single file)
- `src/cli/rc_os_dashboard.cpp` (12 calls)
- `src/dev/dev_cli.cpp` (38 calls)
- `src/dev/diag_stats.cpp` (38 calls)
- `src/dev/fault_inject.cpp` (15 calls)
- `src/dev/replay_inject.cpp` (2 calls)
- `src/drivers/gps_pa1010d.cpp` (7 calls — partially R-2)
- `src/drivers/gps_uart.cpp` (6 calls — partially R-2)
- `src/drivers/i2c_bus.cpp` (15 calls)
- `src/main.cpp`
- `src/safety/fault_protection.cpp` (2 calls — `Q_onError`)
- `src/safety/pyro_edge_logger.cpp` (3 calls)
- `src/telemetry/telemetry_service.cpp` (1 call)

**Total: ~600+ callsites**, mostly mechanical replacement. The largest concentrations (`rc_os_commands`, `ao_rcos`) are CLI / calibration wizard code where format strings dominate.

---

## Dedicated session work breakdown

When the dedicated session(s) open, the order is:

1. **Build `rc_log` infrastructure** — ~200 lines: `src/util/rc_log.{h,cpp}`, ETL formatter adapter, dispatch function, USB CDC sink. Verify with a small smoke test before any callsite work.
2. **Migrate the smallest files first** — `cal_hooks.cpp` (1 call), `telemetry_service.cpp` (1 call), `replay_inject.cpp` (2 calls), `pyro_edge_logger.cpp` (3 calls), `fault_protection.cpp` (2 calls). Quick wins that build confidence in the replacement pattern.
3. **Mechanical-replace mid-size files** — `i2c_bus.cpp` (15), `fault_inject.cpp` (15), GPS drivers (7+6 — R-2 absorbed into this session — see below), `ao_telemetry.cpp` (6), `ao_flight_director.cpp` (26). One commit per file.

   **R-2 absorption note (2026-05-13):** the audit's R-2 (GPS PMTK `snprintf` → const arrays) was initially planned as a stand-alone Phase 8 Cat 3 PR, but Phase 8 scope-investigation surfaced that taking the GPS drivers fully off the allowlist requires migrating one residual `snprintf` (`gps_pa1010d_get_debug_status()`, writes to a caller-supplied buffer — the MISRA-C 2012 accepted safe subset) that needs `etl::format` to migrate cleanly. Rather than (a) vendor ETL just for that callsite as out-of-R-2-scope infrastructure, or (b) hand-roll a one-off int-formatter that R-5's ETL already covers, R-2 is bumped onto the R-5 dedicated session.

   Council Approach-B + 3 amendments (2026-05-13, JPL + ArduPilot + Professor consensus) carry forward to that session:
   - **Approach B (surgical cleanup):** Delete dead `gps_pa1010d_set_rate()` (zero callers), dead-by-actual-use `gps_uart_set_rate()` (always called with `kGpsRateHz10`), and `negotiate_baud(uint32_t)` (always called with `kGpsBaud57600`). Remove `gps_pa1010d_send_command()` / `gps_uart_send_command()` from public headers (no out-of-driver callers). Replace 8 of the 9 `snprintf` callsites with `constexpr` wire-byte arrays for the 4 unique PMTK sentences. The 9th (debug status) migrates to `etl::format` using the R-5 infrastructure.
   - **Amendment 1: Compile-time checksum verification.** `nmea_checksum()` becomes `constexpr` and a `static_assert` verifies each `kPmtk*Wire[]` array's embedded checksum matches the computed value over the bracketed bytes. Eliminates LL-37-style synchronization-by-comment risk.
   - **Amendment 2: Verification per HW_GATE_DISCIPLINE Rule 1.** Capture wire bytes pre- and post-refactor (logic analyzer or hex-dump); byte-for-byte identity is the positive-control signal. Single cold-start GPS lock cycle confirms semantic outcome. Cite captured bytes in commit message per Rule 3.
   - **Amendment 3: Header-API removal documented in CHANGELOG.** Removing `gps_pa1010d_send_command`, `gps_uart_send_command`, `gps_pa1010d_set_rate`, `gps_uart_set_rate` from public headers is a contract change; confirm via `grep` that no caller exists outside the two `.cpp` files before the cut.

   The 4 unique PMTK sentences (with checksums computed 2026-05-13):
   - `$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n` — output sentence config
   - `$PMTK220,1000*1F\r\n` — 1 Hz update rate (pa1010d init)
   - `$PMTK220,100*2F\r\n` — 10 Hz update rate (gps_uart)
   - `$PMTK251,57600*2C\r\n` — baud-rate set
4. **Tackle the big files** — `rc_os.cpp` (48), `rc_os_dashboard.cpp` (12), `dev_cli.cpp` (38), `diag_stats.cpp` (38), `ao_rcos.cpp` (175), `rc_os_commands.cpp` (213). Probably 1-2 commits per big file due to volume.
5. **After each migration**, remove the file from `scripts/hooks/stdio_allowlist.txt`. The allowlist shrinks toward empty.
6. **When allowlist is empty**, update the pre-commit hook to remove the allowlist mechanism and reject `<stdio.h>` in any `src/*.cpp` unconditionally.
7. **Update `standards/CODING_STANDARDS.md`** to remove any "Ground stdio relaxation" language and reference this decision doc as the canonical replacement.
8. **Update `standards/DEBUG_OUTPUT.md`** — the existing `DBG_PRINT` macro section gets a new "Implementation note: routes through `rc_log_dbg`, not `<stdio.h>`."

---

## Allowlist policy

`scripts/hooks/stdio_allowlist.txt` lists files **allowed** to still include `<stdio.h>`. Initially populated with the 18 files above. As migration progresses, files come off the list. **A file is never added to the list** without an explicit commit-message justification cosigned by the user.

If during migration a new file legitimately needs to use `<stdio.h>` temporarily (e.g., a dependency on a third-party header that pulls it in), the allowlist edit is permitted but the commit message must:

- Name the specific reason
- Cite which subsequent commit will remove the file from the allowlist
- Acknowledge the deviation is logged in `docs/PROBLEM_REPORTS.md`

---

## References

- JSF AV C++ Standards (Lockheed Martin, December 2005) — `https://www.stroustrup.com/JSF-AV-rules.pdf` — Rule 22 verbatim on page 19.
- MISRA-C:2012 Rule 21.6 — Required severity, "Standard Library input/output functions shall not be used."
- JPL Institutional Coding Standard for C (NASA/JPL, 2009) — Rule 1 "no reliance on undefined or unspecified behavior" + LOC-5 MISRA inheritance.
- ISO/IEC 9899-1999 Appendix J — list of implementation-defined / unspecified / undefined C behaviors.
- ETL (Embedded Template Library) — `https://www.etlcpp.com/` — header-only, MISRA-aligned, allocation-free.
- NASA F' framework — Logger / Fw::String design.
- NASA cFS — `OS_printf` controlled abstraction.
- ArduPilot HAL `UARTDriver::printf` pattern.
- Memfault firmware logging best practices — tagged log channel + compact binary payload.

---

## Related docs

- `docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md` — developer-facing cheat-sheet ("if you used `printf`, use `rc_log`; if you used `snprintf`, use ETL's `etl::to_string`; ...").
- `scripts/hooks/pre-commit` + `scripts/hooks/stdio_allowlist.txt` — the no-new-`<stdio.h>` gate.
- `docs/PROBLEM_REPORTS.md` — R-5 status (DEFER → dedicated session, with safety-impact rationale).
- `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` — the IO-1 historical entry will reference this doc as the supersession.
- `standards/CODING_STANDARDS.md` — eventually updated when the migration completes.
