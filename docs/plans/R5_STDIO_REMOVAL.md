# Plan: R-5 stdio removal — dedicated migration method

The "why" (eliminate every `<stdio.h>` from `src/`) is settled in `docs/decisions/STDIO_REPLACEMENT_PLAN.md`. This plan covers the "how": the migration method, sequencing, verification discipline, and infrastructure shape.

## Provenance

- **Original four-cycle plan** (2026-05-15, `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md` for Cycles 3+4 stash). R-5 broken out for dedicated focus per user direction.
- **Council round 1** (2026-05-15, problem-statement-only, 4 personas: NASA/JPL + ArduPilot + Professor + Cubesat). Transcript at `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a5195227c0a67e89d.md`. Unanimous on method shape (6-tier risk-ordered + commit-zero infrastructure + macro repoint + function-granularity + byte-on-wire diff + locked surface).
- **Council round 2** (2026-05-15, assembled plan review, same 4 personas). Transcript at `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a70c2cab071f145e9.md`. **APPROVE-WITH-AMENDMENTS unanimous, no dissents.** Round 1 consensus preserved without drift. 5 Session B contract-tightener amendments folded into this plan (non-blocking `etl_putchar`, truncation policy explicit, binary-size measurement at B and J, bench_sim log-line scope in byte-on-wire diff, API-surface-locked-but-internals-iterate clarification).
- **IRL practice research** (2026-05-15): pattern catalog (Strangler Fig, Branch by Abstraction, In-place Sweep, Codemod, Parallel Run), real precedents (Linux printk→pr_*, fmt adoption, ArduPilot HAL, Boost→std), anti-patterns (premature abstraction, two-channel-forever, codemod semantic bugs), single-dev freedom analysis. Inline citations in execution-plan sections below.
- **ETL contamination check** (2026-05-15): VERDICT CLEAN. `etl::format_to` (PR #1204 merged 2025-12-13) and `etl::to_string` use manual digit-conversion, no `vsnprintf`/`snprintf` internally. Adoption eliminates the deviation rather than relocating it. One side-effect: `etl::print` routes through a user-provided `extern "C" void etl_putchar(int c)` callback — project supplies the USB CDC sink wiring.
- **User dispositions** (2026-05-15):
  1. Layout: `include/rocketchip/rc_log.h` + `src/log/rc_log.cpp`.
  2. Library: ETL, pin to ≥2025-12-13 release for `etl::format_to` availability.
  3. Format-spec inventory survey: own dedicated session before infrastructure work.
  4. Pre-tier-1 Level-2 baseline: capture before any migration commit.

## Context

**Scope (refined at Unit A 2026-05-15):** 23 files in `src/` use `<stdio.h>` — 18 via direct include (on the allowlist), 5 via transitive include through `include/rocketchip/config.h`'s `DBG_PRINT` macro (NOT on the allowlist, surfaced by Unit A's full stdio-function sweep). Total ~624 callsites, ~700 format specs across 13 patterns covering 99.9% of usage.

Biggest concentrations:

- `src/cli/rc_os_commands.cpp` — ~215
- `src/active_objects/ao_rcos.cpp` — ~170
- `src/cli/rc_os.cpp` — ~45
- `src/diag/diag_stats.cpp` — ~38
- `src/cli/rc_os_debug.cpp` — ~36

Mid-sized: `ao_flight_director.cpp` (~26), `fault_inject.cpp` (~16), `i2c_bus.cpp` (~15), `cli/rc_os_dashboard.cpp` (~12), `flight_director.cpp` (10, transitive-leak), `go_nogo_checks.cpp` (8, transitive-leak).

Small: GPS drivers (7+6), `ao_telemetry.cpp` (6), `ao_radio.cpp` (4 direct, transitive-leak), `pyro_edge_logger.cpp` (3), `fault_protection.cpp` (3), and several singleton-callsite files.

`health_monitor.cpp` uses DBG_PRINT only (no direct stdio); it resolves at Unit B's DBG_PRINT repoint without per-file callsite migration.

Pre-commit hook already blocks new direct `<stdio.h>` includes outside the allowlist (`scripts/hooks/stdio_allowlist.txt`). Direct-include proliferation is prevented. The transitive-via-DBG_PRINT path is the gap Unit A surfaced; it's mooted at Unit J when the allowlist deprecates. Inventory details: `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md`.

**Single-dev freedom:** main can be broken across many commits for weeks. No CI/CD pressure, no other contributors blocked. This shifts the right method away from strangler-fig (designed for team + production) toward something closer to direct sweep with commit-zero infrastructure + risk-ordered tier sequencing. Branch-by-abstraction's main benefit (trunk stays green) doesn't apply.

## Method shape (council unanimous)

### Replacement call surface

- **Named distinctly from printf**, not a `#define printf rc_log` macro substitution. Cubesat Engineer round 1: "every `<stdio.h>` removal is also a name-collision risk and your bisect history gets noisy" with macro substitution. Explicit rename-and-replace gives readable commits.
- **Architectural option (A) — direct replacement function** (Professor's framing): `rc::rc_log(fmt, ...)` with printf-style format string. Rejected: (B) type-safe streams (doubles per-callsite touch cost), (C) tagged binary log (massive ergonomic regression for serial CLI/dashboard).
- **Minimum viable v1**: single sink (USB CDC), no severity levels, no ring buffer, no runtime sink selection. Cubesat Engineer round 1: prior shop built multi-sink BEFORE migration was done, doubled binary size, took 3 weeks to tune + 2 weeks to rip out post-migration. AK_GUIDELINES Rule 2 (simplicity first).
- **API surface locked at commit zero; internal implementation iterates as bugs surface** (council round 2 amendment #5 — sharpened from round 1's "surface locked"). The `rc::rc_log` signature, supported format-specs, truncation behavior, and sink contract are frozen at Session B. The implementation of `etl::format_to` adaptation, `etl_putchar` ring-buffer, etc. is free to evolve. If a `%hhx` formatting bug surfaces mid-migration at Session E, the fix lands in rc_log internals (files already migrated get re-tested at next Tier-end Level-2 regression; no re-migration needed). Round 1 Cubesat: "halfway through tier 3 someone adds a severity arg — now files migrated before use a different API than files migrated after" — that's the API-surface change banned. ArduPilot endorses: same as ArduPilot's "freeze the HAL API mid-port" discipline, where internal HAL implementations iterate freely.
- **ETL `etl::format_to` as the formatting backend** (post-2025-12-13 release pin). Allocation-free, deterministic, manual digit-conversion (no `vsnprintf`).

### Commit-zero artifacts (before any callsite migration)

1. **Format-spec inventory survey** — its own session per user direction. Run `grep -oE '%[-+ #0]*[0-9]*\.?[0-9]*[hljztL]*[diouxXeEfgGaAcspn%]' src/...allowlisted files` over all 20 files. Categorize into ≤10 patterns. Output: `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-XX.md`. Drives the rc_log supported-spec list.
2. **Pre-tier-1 Level-2 baseline** — full audit-suite regression once, captured as "this is what working looks like" reference: bench_sim 2/2 + warm-reboot stable + `hardware: 14/14 ok` + ctest 800/800 + SPIN 31 PASS. Output: dated capture in `docs/baselines/`.
3. **ETL vendor decision pinned** — `EXTERNAL/etl-vN.N.N/` with N.N.N ≥ 2025-12-13 release. Acceptance criterion documented: `grep -r "vsnprintf\|<stdio.h>" EXTERNAL/etl/include/etl/format.h EXTERNAL/etl/include/etl/to_string.h EXTERNAL/etl/include/etl/private/to_string_helper.h` returns zero matches. CHANGELOG entry for the vendor.
4. **rc_log infrastructure** — `include/rocketchip/rc_log.h` + `src/log/rc_log.cpp`:
   - `rc::rc_log(fmt, ...)` API (printf-shape call surface, internally routes through `etl::format_to`)
   - **`extern "C" void etl_putchar(int c)` is non-blocking** (council round 2 amendment #1, Cubesat Engineer): writes to a small lock-free ring buffer (e.g., 256 bytes) drained by `tud_task` on Core 0's main loop. Never blocks the caller. Defends against LL Entry 32-style queue overflow when heavy CLI output meets USB CDC backpressure. Same pattern as AP_HAL UART putchar (tx ring + IRQ drain).
   - **Truncation policy explicit** (council round 2 amendment #2, NASA/JPL): 128-byte fixed stack buffer per `rc_log` call. Over-budget output truncates with a literal `...\n` marker. No error-return path — callsites are diagnostic, not bound to detect their own truncation. Named in the `rc_log.h` doc-comment so callers don't expect printf-style return-value semantics.
   - Host ctest coverage (`test/test_rc_log.cpp`) covering every format-spec the survey enumerated, plus a truncation-marker test.
   - **Binary-size measurement** (council round 2 amendment #3, NASA/JPL): capture `.text` + `.data` sizes for both target tiers at Session B (after rc_log + ETL lands, before any callsite migration) AND at Session J (after migration completes). Two data points, not a gate — just visibility for the user 6 months later who asks "what did this cost." Output captured in the Session B + Session J commit messages.
   - Build clean both target tiers (vehicle + station).
5. **Migration guide API correction** — `docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md` updated for ETL's actual API (`etl::format_to(out_iter, "x={}", x)` not `etl::format(buf, "x={}", x)`).
6. **DBG_PRINT macro repoint** — `standards/DEBUG_OUTPUT.md`'s `DBG_PRINT` / `DBG_ERROR` macros switch backend from `printf` to `rc_log`. Header-only change, recompiles the project, validates rc_log under realistic compile load before manual callsite migration starts. Bench_sim run + 3-boot HW gate per HW_GATE Rule 2. ArduPilot round 1: "the macro re-point is commit zero of the whole migration. Cheap, high-leverage, runs the replacement under realistic load."

### Migration sequence — 6-tier risk-ordered

NASA/JPL round 1 framing. Risk dimensions: callsite count + format-spec complexity + flight-critical classification + bench HW verification coverage + hot-path latency sensitivity. Order is lowest-risk first.

**Tier 1 — proving ground (1-2 files):**
- `src/safety/pyro_edge_logger.cpp` (~3 callsites)
- `src/safety/fault_protection.cpp` (~2 callsites)

One file at a time. End-to-end migration including allowlist drop. Validates rc_log infrastructure under realistic flight-critical use. ~1 session.

**Tier 2 — drivers (3-4 files):**
- `src/drivers/gps_pa1010d.cpp` (~7) — R-2 absorbed; PMTK const-array migration per 4 amendments from R-2's prior council
- `src/drivers/gps_uart.cpp` (~6) — R-2 absorbed
- `src/drivers/i2c_bus.cpp` (~15)
- `src/telemetry/telemetry_service.cpp` (~1)

Flight-critical adjacent, well-covered by bench_sim. Each file = own session or sub-session. Function-granularity within file. ~2-3 sessions.

**Tier 3 — safety/diag (3 files):**
- `src/safety/fault_inject.cpp` (~15)
- `src/safety/station_fault_inject.cpp`
- `src/diag/diag_stats.cpp` (~38)

Diagnostic, gated behind test-mode-gate or read-only. ~1-2 sessions.

**Tier 4 — telemetry/AO/flight-director (7 files, expanded by Unit A finding):**
- `src/active_objects/ao_flight_director.cpp` (~26)
- `src/active_objects/ao_telemetry.cpp` (~6)
- `src/active_objects/ao_radio.cpp` (4 direct + DBG_PRINT) — Unit A transitive-leak surface
- `src/flight_director/flight_director.cpp` (10) — Unit A transitive-leak surface; **load-bearing for bench_sim regex (LL Entry 36)** — `[FD] PYRO FIRED`, `[FD] ABORT`, `[FD]` state transitions all live here
- `src/flight_director/go_nogo_checks.cpp` (8) — Unit A transitive-leak surface
- `src/flight_director/guard_combinator.cpp` (1) — Unit A transitive-leak surface
- `src/calibration/cal_hooks.cpp` (~1)

Cross-cutting AOs + flight-director module functions. Council round 2 amendment #4 (bench_sim log-line byte-on-wire diff scope) is **load-bearing for this tier** since `flight_director.cpp` is the primary source of `[FD]` log lines that bench_sim's regex parses.

**Tier 5 — CLI/dashboard (5 files, the big ones):**
- `src/cli/rc_os.cpp` (~44)
- `src/cli/rc_os_commands.cpp` (~215)
- `src/cli/rc_os_debug.cpp` (~38)
- `src/cli/rc_os_dashboard.cpp` (~12)
- `src/active_objects/ao_rcos.cpp` (~167)

Function-granularity strictly enforced (ArduPilot amendment: per-pattern within file is theoretically right but conflicts on rebase; per-function gives 20-40 coherent commits per file). Threshold: ~30-50 callsites per commit max; split bigger functions per-pattern. ~3-4 sessions.

**Tier 6 — cleanup (2 files):**
- `src/main.cpp`
- Any remainders

Final allowlist drop to empty. **Pre-commit Gate 1 final upgrade** — current logic (lines 32-73 of `scripts/hooks/pre-commit`) is gated on `[[ -f "${STDIO_ALLOWLIST}" ]]` AND uses an allowlist-membership exemption. End-state changes (all in same commit):

1. **Delete `scripts/hooks/stdio_allowlist.txt`** entirely.
2. **Strip the allowlist-membership exemption** from Gate 1 — remove the `grep -qxF "${src}" "${STDIO_ALLOWLIST}"` check and the surrounding `if [[ -f ... ]]` guard. The check becomes unconditional: any staged `src/*.cpp` containing `#include <stdio.h>` (either in the diff or in the committed file content) fails the gate.
3. **Update Gate 1's comment block** (lines 18-26) — replace "Existing files that currently include it are listed in scripts/hooks/stdio_allowlist.txt; as the R-5 dedicated sessions migrate them, they come off the list. When the list is empty, the gate becomes..." with "No `<stdio.h>` in `src/*.cpp` anywhere. Allowlist mechanism retired YYYY-MM-DD per R-5 closure (commit SHA)."
4. **Verify by running the hook** against a synthetic test commit that adds `#include <stdio.h>` to any `src/*.cpp` — the hook must reject it. Cite the test in the Unit J commit message.

Update `standards/CODING_STANDARDS.md` to remove Ground-stdio-relaxation language. Update `standards/DEBUG_OUTPUT.md` to reflect new backend. `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` cross-reference if IO-1 still has a row (likely none, but verify).

### Per-commit granularity

ArduPilot round 1, NASA/JPL endorsed, Professor conceded:

- **Default:** one function migrated per commit. For files with 20-40 handlers (`rc_os_commands.cpp`), one handler at a time gives 5-15 callsites per commit, coherent diffs, independently revertable.
- **Threshold for splitting further:** ~30-50 callsites per commit. Larger functions split per-pattern within the function.
- **Allowlist drop rides with the file's last callsite commit.** ArduPilot round 1: "callsite changes + `<stdio.h>` removal + allowlist line removal, all in one commit. Verification ties to allowlist position."

### Verification discipline (council unanimous, maps to HW_GATE_DISCIPLINE Rule 6)

**Level-1 (local-commit, per HW_GATE Rule 3):** every per-function commit observes its own positive-control signal:
- Bench_sim PASS if the function is reached by the smoke test
- Manual serial-output byte-on-wire diff against pre-migration capture for CLI / dashboard / output functions not covered by bench_sim
- Pre-commit hook ctest baseline (unconditional)
- For driver migrations (Tier 2): probe trace byte-on-wire identity (already required by R-2's prior council amendment)
- **bench_sim-monitored log lines explicitly in byte-on-wire diff scope** (council round 2 amendment #4, NASA/JPL). Per LL Entry 36, bench_sim's regex can rot silently when a firmware log line changes by even one byte. Mitigation: the per-file byte-on-wire diff explicitly compares the firmware log lines that bench_sim's regex parses (e.g., `[FD] PYRO FIRED: ...`, `[FD] ABORT*`, `link lost`, the boot banner). Any single-byte drift surfaces before Tier-end Level-2 regression, not after.

**Level-2 (audit-suite regression, per tier-end):** after each of the 6 tiers closes, re-run:
- Full bench_sim 2/2
- Warm-reboot stable across 3 reseat boots
- `hardware: 14/14 ok` on vehicle, `11/11 ok` on station
- Full host ctest (currently 800)
- SPIN regression (currently 31 PASS)
- Manual dashboard read on station

Don't wait until allowlist-empty for any regression check — catch tier-level interactions early.

**Level-3 (independent verification, at allowlist-empty):** an audit-suite regression by a future session closes the migration. Same Level-3 pattern as FP-1 / R-6c per HW_GATE Rule 6.

### Defenses against named failure modes (council round 1, 6 explicit)

1. **Format-spec semantic drift.** Mitigation: format-spec inventory survey at commit zero; byte-on-wire diff per file migration (NASA/JPL: "exact-byte match for ASCII output, or it's a finding").
2. **Transitive include leakage.** `<stdio.h>` provides `NULL`/`size_t`/`FILE*` to legacy code. Mitigation: add `<cstddef>`/`<stdint.h>` first, remove `<stdio.h>` second, build-clean per file before commit.
3. **Implementation-defined behavior re-entering via libc.** Mitigation: explicit acceptance criterion — `grep -r vsnprintf src/ scripts/ EXTERNAL/etl/...` shows nothing project-code reaches. Already verified for ETL (CLEAN verdict 2026-05-15).
4. **"While we're here" trap.** Refactor diffs touching 600 callsites tempt scope creep (rename vars, fix comments, tighten formatting). Mitigation: aggressive surgical discipline per AK_GUIDELINES Rule 3. Every changed line traces to a stdio→rc_log substitution.
5. **Mid-stream API expansion.** Replacement gets a severity arg / hex helper / etc. halfway through, splits the codebase. Mitigation: API locked at commit zero (the rc_log header is the contract). Expansion happens post-allowlist-empty as separate enhancement work.
6. **Concurrent CLI refactor temptation.** Halfway through `rc_os_commands.cpp` someone notices the CLI dispatch is messy and wants to rewrite it. Don't. Sequence them, don't merge.

### Anti-patterns explicitly avoided (IRL research + council)

- **Big-bang rewrite** — universally discouraged; the council's 6-tier sequencing is the structural defense.
- **Two-channel-forever (strangler that never finishes)** — Linux's printk migration is partly this. Mitigation: the allowlist forces monotonic progress. When the allowlist is empty, the channel is one.
- **Premature abstraction** — Joel Spolsky / Netscape rewrite. Mitigation: v1 is minimum viable (single sink, no severity, no ring buffer), expansion is post-migration.
- **Codemod semantic bugs** — fmt's own migration notes document `printf("%d", x)` → `fmt::print("{}", x)` changing behavior when `x` is `char` (printf promotes to int, fmt prints character). Mitigation: byte-on-wire diff catches this; function-granularity commits make any divergence easy to bisect.
- **Two-channel-forever risk from IRL research:** GitHub Scientist library exists because parallel-run cutover often stalls at ~90%. The allowlist-drop discipline is our structural defense.

## Work-unit breakdown

Each entry = one focused work unit. Units are stoppable on any commit boundary (Drew DeVault's solo-refactor advice: commit per file, not per session, to survive long breaks). Natural pause points are commit boundaries — a "session" in user framing can span multiple units or pause for the night mid-unit.

**Unit A — format-spec inventory survey (own unit per user direction).** Run grep, categorize, produce `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-XX.md`. Pure-doc commit. No HW gate.

**Unit B — commit-zero infrastructure.** ETL vendoring + acceptance-criterion verification + `include/rocketchip/rc_log.h` + `src/log/rc_log.cpp` + host ctest + migration guide API correction + DBG_PRINT macro repoint. Bench_sim 2/2 + 3-boot reseat verification at unit end.

**Unit B closes with a single-callsite smoke validation** (user direction 2026-05-15): pick ONE low-stakes callsite (e.g., a startup banner `printf` in `main.cpp` or a `DBG_PRINT` in an init path) and migrate it standalone — through `rc_log` end-to-end. Flash + read serial banner + confirm byte-on-wire matches pre-migration capture. This is a smoke test *inside* Unit B, not part of any tier. Validates that rc_log → etl::format_to → etl_putchar ring → tud_cdc_write → wire actually produces the expected ASCII before any tier work begins. The smoke-test callsite then either stays migrated (if the file isn't in a later tier) or gets re-migrated naturally when its tier comes up — either way, Unit B exits with proof-of-life that rc_log works on real hardware, not just host ctest.

**Unit C — Tier 1 proving ground.** `pyro_edge_logger.cpp` + `fault_protection.cpp`. Validates rc_log under flight-critical use.

**Unit D — Tier 2 drivers (split if needed).** GPS + i2c_bus + telemetry_service. R-2 absorbed (council amendments from prior R-2 review carry forward: const-array PMTK + compile-time checksum static_assert + byte-on-wire identity + header-API removal).

**Unit E — Tier 3 safety/diag.** fault_inject + station_fault_inject + diag_stats.

**Unit F — Tier 4 telemetry/AO.** ao_flight_director + ao_telemetry + cal_hooks.

**Unit G — Tier 5 CLI part 1.** `rc_os.cpp` + `rc_os_debug.cpp` + `rc_os_dashboard.cpp`.

**Unit H — Tier 5 CLI part 2.** `rc_os_commands.cpp` (~215 callsites, function-granularity).

**Unit I — Tier 5 CLI part 3.** `ao_rcos.cpp` (~167 callsites).

**Unit J — Tier 6 cleanup + allowlist deletion.** `main.cpp` + remainders + allowlist drop + pre-commit Gate 1 upgrade + CODING_STANDARDS / DEBUG_OUTPUT updates.

**Unit K — Level-3 independent regression (deferred to a future session per HW_GATE Rule 6).** Re-runs full audit suite end-to-end against allowlist-empty state. Closes R-5 + R-2 at Level-3.

**Total: 10 work units A-J + 1 follow-on K (Level-3 regression).** Sizing is per-unit, not per-clock-session — natural pause points are commit boundaries, not session boundaries. A "session" in user framing is a working stretch that can span pauses for the night. Per user direction 2026-05-15 the multi-unit scope is approved as effectively-one-session work.

## Files touched (across all sessions)

**New files:**
- `EXTERNAL/etl-vN.N.N/` (vendored ETL, ~30k LOC header-only)
- `include/rocketchip/rc_log.h`
- `src/log/rc_log.cpp`
- `test/test_rc_log.cpp`
- `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-XX.md` (Session A output)
- `docs/baselines/PRE_R5_BASELINE_2026-05-XX.md` (Session A output)

**Modified files:**
- `CMakeLists.txt` (ETL + rc_log + etl_putchar wiring)
- `include/rocketchip/config.h` (DBG_PRINT backend repoint)
- All 20 currently-allowlisted files (one session/cluster each)
- `scripts/hooks/stdio_allowlist.txt` (drains to empty)
- `docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md` (API correction)
- `docs/decisions/STDIO_REPLACEMENT_PLAN.md` (cross-reference this plan)
- `scripts/hooks/pre-commit` (Gate 1 final upgrade at Session J)
- `standards/CODING_STANDARDS.md` (Session J, remove Ground stdio relaxation)
- `standards/DEBUG_OUTPUT.md` (Session J, backend swap note)

**Deleted files:**
- `scripts/hooks/stdio_allowlist.txt` (Session J, when empty)

**PROBLEM_REPORTS rows closed:**
- R-5 (full stdio.h removal) → closed at Session J
- R-2 (GPS PMTK snprintf, absorbed) → closed at Session D

## Open questions / future enhancements (out of this plan's scope)

Post-allowlist-empty, the rc_log surface can be extended:

- Severity tagging (rc_log_err / rc_log_dbg / rc_log_info) — currently DBG_PRINT already gives compile-out for debug; full severity model is post-migration.
- Telemetry routing (rc_log → MAVLink dual-sink) — out of scope.
- Throughput characterization (per-call cycle cost on bench) — post-migration measurement, not gating.
- Compile-time format-string validation via ETL's consteval (if supported) — investigate post-migration.

All deferred until after R-5 closes at Level-3.

## Triggers to start

Plan approval. Then:
- **Unit A first** (format-spec inventory survey). Pure-doc, no HW dependencies. Can start any time.
- **Unit B** requires probe + bench access (HW gate at unit end).
- **Units C-J** all require probe + bench (sometimes vehicle, sometimes station).
- **Unit K (Level-3)** waits for Units A-J to all close.

## Verification of the plan itself

Per HW_GATE_DISCIPLINE Rule 1: this plan names positive-control signals for each tier. Per Rule 2: 3-boot reseat at each tier-end Level-2 regression. Per Rule 3: every commit cites its observed signal. Per Rule 4: nothing is structurally a soft gate (allowlist position is mechanical, bench_sim is mechanical, byte-on-wire diff is mechanical). Per Rule 6: Level-1 per commit + Level-2 per tier + Level-3 at allowlist-empty.

## Critical files referenced

- `docs/decisions/STDIO_REPLACEMENT_PLAN.md` — authoritative "why" + 18-file inventory (pre-R-25-exec — current inventory is 20 files per `scripts/hooks/stdio_allowlist.txt`)
- `docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md` — patterns + format-spec translation reference
- `scripts/hooks/stdio_allowlist.txt` — current 20-file list
- `scripts/hooks/pre-commit` — Gate 1 stdio check
- `standards/HW_GATE_DISCIPLINE.md` Rules 1-6 — verification credit framework
- `standards/CODING_STANDARDS.md` — JSF AV / MISRA / JPL citations
- `.claude/SESSION_CHECKLIST.md` — commit-level discipline
- `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a5195227c0a67e89d.md` — council round 1 transcript
- `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md` — Cycles 3+4 stash (resumed after R-5 closes)
