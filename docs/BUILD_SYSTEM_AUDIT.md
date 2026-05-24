# Build System Audit

**Purpose:** Structured periodic check that `CMakeLists.txt` and related
build-system configuration haven't accreted dead code, silently-drifted
coverage gates, or stale narrative. A single broken auxiliary target
(`ud_benchmark`) went undetected for 2+ weeks in April 2026; a pedantic
warning gate drifted out of coverage for 14 of our source files over
~6 months. This document is the mechanism to catch that class of rot
before it compounds.

**Complements** the protected-file drift check in `docs/agents/SESSION_CHECKLIST.md`
item 15 (which covers architecture docs) and the station/vehicle build
parity check in item 8 (which covers cross-role compilation). This one
is content-level for the build system itself.

---

## When to run this audit

- **Every stage close / milestone.** Pair with `docs/agents/SESSION_CHECKLIST.md`
  item 15.
- **Any CMake edit >10 lines.** Small edits rarely introduce rot; large
  edits frequently add scaffolding that should be cleaned up later.
- **Any new auxiliary target or `option()` flag added.** These are the
  most common rot sources — they get added, serve their purpose, and
  nobody revisits them.
- **After deprecating a subsystem.** The machinery supporting a deleted
  subsystem rots silently until the audit surfaces it (ref: watchdog
  recovery module, `src/watchdog/` directory deleted 2026-04-22 but
  CMake glue for `rc_watchdog` lingered).
- **After any `-W*` compiler-flag change.** Warnings can silently
  stop covering files they used to cover (the `ROCKETCHIP_SOURCES`
  drift).
- **Every ~6 months unconditionally** as a calendar-driven backstop.

---

## Audit checks

Run in order. Stop and fix at the first failing check — later checks
may depend on earlier ones being clean.

### P1-A: Dev-tool gating discipline

**Every `option()` that introduces a diagnostic, testing, or A-B flag
must be gated behind `ROCKETCHIP_BUILD_DEV_TOOLS`.** Production builds
(all four standard presets) must never see bare `option()` declarations
for diagnostic flags.

**The hierarchy** (established 2026-04-23):
- `ROCKETCHIP_BUILD_DEV_TOOLS` (parent, default OFF) — gate for dev-tool
  targets and the stage archive. No standing dev-tool targets at present;
  re-add new bench utilities under this gate.
- `ROCKETCHIP_STAGE_ARCHIVE` (child, default OFF even when parent is ON)
  — unlocks stage-bounded momentary diagnostics like `STAGE_T_LOGGING`,
  `STAGE_T2_CHEAT`, `STAGE_T3_MAVLINK`.
- Individual stage flags (default OFF even when both parents are ON)
  — per-flag opt-in.

**Check:**
```bash
grep -n '^\s*option(' CMakeLists.txt
```

**Clean result:** every hit is either (a) a production configuration
flag (`BUILD_TESTS`, `ROCKETCHIP_JOB_STATION`, `ROCKETCHIP_JOB_RELAY`,
`ROCKETCHIP_BUILD_DEV_TOOLS`) or (b) nested inside an
`if(ROCKETCHIP_BUILD_DEV_TOOLS)` / `if(ROCKETCHIP_STAGE_ARCHIVE)` block.
(Single flight binary per role with runtime test-mode gating via
`rc::test_mode_active()`; see
`docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.)

**Dirty result** — any bare `option(ROCKETCHIP_*)` at production scope
for something diagnostic-flavored. Move it under the gate hierarchy.

### P1-B: Self-flagged dead code

**Look for self-declared temporary code that was never cleaned up.**

```bash
grep -nE "THROWAWAY|temporary|revert.after|delete.after|remove.after" \
    CMakeLists.txt test/CMakeLists.txt src/**/*.cpp src/**/*.h
```

**Clean:** zero hits. Or all hits are clearly current (dated within
the active stage, not stale).

**Dirty:** any hit with a date >1 stage old, or with language like
"revert after data collection" from a stage that's COMPLETE per
`docs/PROJECT_STATUS.md`. Delete the code or move it under
`ROCKETCHIP_STAGE_ARCHIVE` per P1-A.

### P2: ROCKETCHIP_SOURCES coverage

**Every `.cpp` file under `src/` that compiles into the firmware target
must also appear in `ROCKETCHIP_SOURCES`** (the pedantic-check list),
unless explicitly excluded with a documented reason.

**Why:** `ROCKETCHIP_SOURCES` carries `-Wpedantic` per-file. Without
explicit list maintenance, new files drift outside the warning gate.
Original drift 2026-04 missed 14 files for months.

**Check:**
```bash
# Files compiled into rocketchip
comm -23 \
    <(grep -oE 'src/[a-z_/]+\.cpp' CMakeLists.txt | \
      awk '/^    add_executable.rocketchip/,/^    \)/' | sort -u) \
    <(awk '/^    set.ROCKETCHIP_SOURCES/,/^    \)/' CMakeLists.txt | \
      grep -oE 'src/[a-z_/]+\.cpp' | sort -u)
```

**Clean:** empty output — every source in the firmware target is also
in the pedantic list.

**Dirty:** any file name — add to `ROCKETCHIP_SOURCES` (in the
appropriate module block to preserve organization). Be prepared for
pedantic warnings to surface; either fix them or, if they come from
a vendor header, check P4.

### P3: Host/target CMake split

`rc_flight_director` library (host build) and `add_executable(rocketchip ...)`
(target build) both list the same flight_director source files. **This
is intentional** — the two listings live in different `if(BUILD_TESTS)`
branches and compile with different preprocessor context. Do NOT
collapse into a shared variable without preserving the split.

**Check:** the comment block at `rc_flight_director` (CMakeLists.txt
around line 129) explains this. Verify the comment is still there and
current.

### P4: Vendor includes on -isystem

**Vendored third-party libraries must be classified as SYSTEM includes**
so their headers don't false-positive on our `-Wpedantic` gate.

**Vendored libs** (in `lib/`):
- `lib/mavlink` — IVP-61, third-party telemetry protocol
- `lib/qep` — IVP-67, vendored QP/C 8.1.3
- `lib/ruuvi.dps310.c` — IVP-11, DPS310 driver
- `lib/lwgps` — IVP-31, NMEA parser

**SDK-vendored** (TinyUSB etc., transitively via Pico SDK):
- `tinyusb_common_base`, `tinyusb_common`, `tinyusb_device_unmarked`,
  `tinyusb_device`, `tinyusb_device_base`

**Check:**
```bash
# Vendored libs should appear in a SYSTEM target_include_directories block
grep -A20 "target_include_directories.rocketchip SYSTEM" CMakeLists.txt

# SDK libs should have SYSTEM property set
grep -A5 "set_target_properties.*SYSTEM TRUE" CMakeLists.txt
```

**Clean:** both are present, both cover all currently-vendored libs.

**Dirty:** a vendored lib is `-I`-classified → pedantic-checked files
that `#include` it will fail. Move to SYSTEM.

### P5: Stage-era scaffolding comments

**Comments referencing IVPs or stages in transient "being added / being
implemented" language should age out after the stage closes.**

```bash
grep -nE "are implemented|will be added|will be implemented" \
    CMakeLists.txt test/CMakeLists.txt
```

**Clean:** zero hits.

**Dirty:** "Source files added as IVP-X/Y are implemented" type
comments. Remove — they describe a state that's no longer transitional.

**Preserve:** IVP numbers that explain *why a file exists* (e.g.,
"IVP-67: QEP hierarchical state machine dispatch"). Those are
provenance, not scaffolding.

### P6: Toolchain version check

Newly added 2026-04-23 — same rot class as CMake drift, different
dimension. Verify toolchain components aren't on outdated versions
with known issues.

**Components:**
- **Pico SDK** — check `pico-sdk/pico_sdk_version.cmake` vs
  `github.com/raspberrypi/pico-sdk/releases`. Currently pinned via
  the installed `pico-sdk` symlink.
- **Pico Probe firmware** — check installed version against
  `github.com/raspberrypi/debugprobe/releases`. See
  `AGENT_WHITEBOARD.md` errata entry for known firmware regressions
  (e.g., v2.3.0 has a known "fails to start" regression per upstream
  #201).
- **GCC ARM toolchain** — we use `14_2_Rel1`. Newer releases
  available at `developer.arm.com/Tools and Software/GNU Toolchain`.
- **OpenOCD** — Pi-fork at `0.12.0+dev`. Check
  `github.com/raspberrypi/openocd` for updates (flight-critical: the
  rp2350.cfg support lives here).
- **picotool** — `2.2.0-a4` as of 2026-04. Check
  `github.com/raspberrypi/picotool`.
- **CMake** — our minimum is 3.25 (set 2026-04-23); local machines
  often run 4.x. If bumping our minimum, audit that the workflow
  recommendations (VS Code Pico extension, etc.) ship a CMake that
  meets it.

**Clean:** all components within one minor version of current upstream,
or known-good-pinned with rationale in `AGENT_WHITEBOARD.md`.

**Dirty:** any component >3 minor versions behind without rationale.
Document the lag or upgrade.

---

## Guardrails — what NOT to delete

These look like scaffolding but are load-bearing. If the audit flags
them, leave them alone and add a comment-explanation if missing.

1. **`PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1`** block around
   CMakeLists.txt:230-280 — RP2350-E2 erratum workaround. See
   `standards/RP2350_ERRATA.md`.

2. **`src/diag/diag_stats.cpp` unconditional inclusion** — always-on
   read-only diagnostic snapshot (AO queues, MSP high-water, sensor
   reads, health latches, T=0 soak preconditions). Migrated from
   `src/dev/` to `src/diag/` per R-25-exec step 4 (2026-05-13);
   compiles into both flight binaries unconditionally. No
   `test_mode_active()` gate needed — reads only, no state mutation.

3. **`target_link_options(rocketchip --undefined=...)`** block —
   `--gc-sections` would otherwise strip symbols needed by
   runtime-bound fault-injection / diag infrastructure.

4. **Per-file `-O2` on `src/fusion/eskf.cpp` and
   `src/fusion/eskf_codegen.cpp`** — LL Entry 30 load-bearing
   (XIP cache, codegen performance). Do not remove the per-file
   override, do not replace with target-level `-O2`.

5. **The flight_director source-file duplication** between
   `rc_flight_director` (host) and `add_executable(rocketchip)`
   (target). Per P3 above — intentional host/target split.

6. **Stage T6 comment** inside the `ROCKETCHIP_STAGE_ARCHIVE` block
   explaining why there's no T6 compile flag. Earns its keep by
   clarifying an intentional gap in the archive surface.

---

## Incident log

Append-only record of what this audit has surfaced and fixed. New
entries at top.

### 2026-04-30 — One-time scaffolding cleanup

- **Removed:** `mat_benchmark` target + `src/tools/mat_benchmark.cpp` —
  IVP-40 gate (24-state ESKF state-count decision) closed, results in
  `docs/benchmarks/`. No documentation called for periodic re-runs.
  `ROCKETCHIP_BUILD_DEV_TOOLS` parent gate retained for the stage
  archive children and future dev tools; comment updated.
- **Removed:** `docs/TOOLCHAIN_VALIDATION.md` — Jan-2026 FreeRTOS SMP +
  Pico SDK 2.1.0 + "Phase 2" scaffolding, last touched 2026-02-02 and
  wrong on every axis since the QP/C bare-metal pivot. Build/debug
  guidance lives in `DEBUG_PROBE_NOTES.md`, `FLASHING.md`,
  `BENCH_TEST_PROCEDURE.md`, `BOARD_FIRMWARE_VERIFICATION.md`.

### 2026-04-23 — Initial audit + cleanup

- **Removed:** `ud_benchmark` target — Phase-1 gate decision delivered
  2026-02-24, results preserved in `docs/benchmarks/UD_BENCHMARK_RESULTS.md`,
  target broken for 2+ weeks without anyone noticing.
- **Gated behind `BUILD_DEV_TOOLS`:** `mat_benchmark` executable.
- **Gated behind `STAGE_ARCHIVE` (nested under `BUILD_DEV_TOOLS`):**
  `ROCKETCHIP_STAGE_T_LOGGING`, `ROCKETCHIP_STAGE_T2_CHEAT`,
  `ROCKETCHIP_STAGE_T3_MAVLINK` — all stage-bounded diagnostics from
  the now-complete Stage T investigation.
- **Fixed P2 coverage:** 14 of our own source files were drifting
  outside the `ROCKETCHIP_SOURCES` pedantic list. Now covered:
  active_objects (6), dev/diag_stats, fusion/{confidence_gate,
  innovation_monitor}, notify backends (2), safety (3).
- **P4 SYSTEM classification:** moved vendored `lib/` includes
  (mavlink, qep, ruuvi, lwgps) from `-I` to `-isystem`. Added
  `set_target_properties(SYSTEM TRUE)` for Pico SDK libs +
  tinyusb INTERFACE chain.
- **Side effect (verified benign):** SDK `.obj` files ~740 bytes
  smaller in aggregate due to `-isystem` semantics unlocking
  slightly more aggressive SDK-code optimization. Our code `.obj`
  byte-identical. Runtime verified via bench_sim 2/2 + preflight
  VERDICT GO.
- **P5 comments:** removed "Source files added as IVP-X/Y are
  implemented" scaffolding; trimmed eskf_brake migration note.
- **CMake minimum:** 3.13 → 3.25 (was already fiction — presets
  required 3.21 to parse).

Commits: `5bc463b` (Phase 0+1), `69a07f3` (Phase 2), `af45bbc`
(Phase 3), `45d3c53` (Phase 5). Plan file:
`.claude/plans/i-just-put-you-glimmering-elephant.md`.

---

## See also

- `docs/agents/SESSION_CHECKLIST.md` item 8 — station/vehicle build parity
  check (structural build level).
- `docs/agents/SESSION_CHECKLIST.md` item 15 — protected-doc drift check
  (architecture docs level).
- `standards/RP2350_ERRATA.md` — silicon errata matrix; some compile
  flags are erratum workarounds and must not be removed without
  cross-checking there first.
- `docs/agents/LESSONS_LEARNED.md` Entry 36 — "infrastructure vs artifact"
  discipline. This audit is explicitly structured as infrastructure:
  it runs automatically (pre-commit gates where applicable) rather
  than relying on human memory to trigger it.
