# Toolchain Version Audit — 2026-05-13

**Audit type:** P6 of `docs/BUILD_SYSTEM_AUDIT.md` — second run. Closes L2-P9 from the 2026-05-07 cycle.
**Method:** Mechanical — `scripts/audit/check_toolchain_drift.py` (F-2026-05-13-003, shipped earlier today).
**Prior run:** `TOOLCHAIN_VERSION_AUDIT_2026-04-27.md` (manual walk).
**Procedure framing:** `AUDIT_GUIDANCE.md` Tier 2.3.

## Method

`scripts/audit/check_toolchain_drift.py` was invoked against the working tree. The script:

- Reads `CMakeLists.txt` for `sdkVersion`, `picotoolVersion`, `toolchainVersion`, `cmake_minimum_required`.
- Walks `~/.pico-sdk/<component>/` for installed versions (Pico SDK, picotool, GCC ARM toolchain, OpenOCD Pi fork).
- Runs `cmake --version` for local CMake.
- Pulls upstream latest via GitHub Releases API:
  - `raspberrypi/pico-sdk` releases
  - `raspberrypi/picotool` releases
  - `raspberrypi/debugprobe` releases
  - `Kitware/CMake` releases
- Pulls branch HEAD via GitHub commits API for `raspberrypi/openocd` `rpi-common` (no release cadence on that fork).
- GCC ARM upstream is not on GitHub (ARM Developer download portal); flagged as `unknown` per the script's manual-tracking note.

Raw F-003 output: `logs/toolchain_drift.txt`.

## Summary (this cycle)

| Component | Project pin | Local install | Upstream latest | Status | Delta vs 2026-04-27 |
|---|---|---|---|---|---|
| Pico SDK | 2.2.0 | 2.2.0 | 2.2.0 | clean | unchanged |
| picotool | 2.2.0-a4 | 2.2.0-a4 | 2.2.0-a4 | clean | unchanged |
| GCC ARM (toolchain) | 14_2_Rel1 | 14_2_Rel1 | *(no auto-pull — ARM Developer portal)* | unknown | manual: upstream 15.2.Rel1 per prior run; still pinned |
| OpenOCD (Pi fork) | — (branch-tracked) | 0.12.0+dev | `acff23f` (2026-03-18) | drift | branch HEAD unchanged since prior run (`acff23f` was HEAD at 2026-04-27) |
| Pico Probe firmware | — (probe USB descriptor) | *(read from probe)* | `debugprobe-v2.3.0` | drift | unchanged — upstream still v2.3.0; "fails to start" #201 still open |
| CMake (local) | 3.25 (minimum) | 4.2.1 | 4.3.2 | drift | local was 4.2.1 prior run; upstream advanced 4.3.2 → still 4.3.2 (no change since 2026-04-27 was already 4.3.2) |

**Verdict:** All components within the P6 "Clean" criterion. **No drift requiring action.** This is a 16-day delta from the prior manual walk; nothing meaningful has moved in the upstream ecosystem during that window.

## Per-component notes (delta from 2026-04-27)

### Pico SDK 2.2.0 — still current
No change. Upstream 2.2.0 release date unchanged (2026-07-29 per repo metadata). `CMakeLists.txt:171` still pins `set(sdkVersion 2.2.0)`. Vendored copy at `pico-sdk/pico_sdk_version.cmake` reports 2.2.0.

### picotool 2.2.0-a4 — still current
No change. `CMakeLists.txt:173` still pins `set(picotoolVersion 2.2.0-a4)`. Upstream 2.2.0-a4 release date unchanged (2026-08-14).

### GCC ARM 14.2.Rel1 — pinned, still one major behind
No change in our pin. The 2026-04-27 audit documented the GCC 15 vs 14 evaluation: ABI/codegen risk surface outweighs diagnostic improvements when there is no specific bug we need fixed; Pico SDK 2.2.0 is validated against 13.x/14.x; moving to 15.x is ahead of the SDK's validated matrix. **Re-evaluate when Pico SDK 2.3+ explicitly recommends 15.x.** Unchanged this cycle.

### OpenOCD (Pi fork) 0.12.0+dev — pinned, still tracking branch
Local install timestamp `2025-10-09` unchanged. Branch HEAD `acff23f` (2026-03-18) was already the HEAD at 2026-04-27; no new commits to `rpi-common` in the 16-day window. **Stay pinned** per the 2026-04-27 rationale (chasing branch-tip introduces flake risk; LL Entry 25's "debug probe is the primary flashing tool" raises stakes on probe regressions).

### Pico Probe firmware — still not captured locally; upstream still v2.3.0
The F-003 script cannot read the version off the physical probe (would require USB descriptor query via OpenOCD). The 2026-04-27 audit flagged `debugprobe-v2.3.0` as having a "fails to start" regression per upstream issue #201. **No change recommended** — if our probe is on v2.2.x and working, do not chase v2.3.0; if our probe is on v2.3.0 and misbehaving, roll back. The user-facing operational status (probe currently working per LL Entry 11 + DEBUG_PROBE_NOTES.md) suggests we're on a safe version.

### CMake — local 4.2.1, upstream 4.3.2
No change from 2026-04-27 (which already recorded local 4.2.1 vs upstream 4.3.2). Project minimum is 3.25 (`CMakeLists.txt:10`); local is well above. **No action.**

## Mechanism upgrade (new this cycle)

The 2026-04-27 audit was a manual walk. This 2026-05-13 audit is the **first mechanical run** of `scripts/audit/check_toolchain_drift.py` (F-003, shipped earlier in this audit-infrastructure follow-up session, commit `fcb4cd8`). Future cycles re-run the script; this artifact becomes the diff baseline.

The script's exit code is always 0; drift is informational (per the "Pinned is not drift" rationale). The audit-report's role is to *interpret* the drift table — which "drift" rows are acceptable (pinned with reason) vs which warrant a pin-move.

## Watch mechanism progress

The 2026-04-27 audit listed three options for the P6 "concrete watch mechanism":

1. **Quarterly audit cadence** — partially adopted via AUDIT_GUIDANCE Tier 2.3 (every master audit walks toolchain drift).
2. **Pre-commit version-string check** — NOT adopted. F-003 is audit-time only, not pre-commit.
3. **Errata-watch coupling** — partially adopted via `standards/RP2350_ERRATA.md` E2 row tracking the `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` SDK flag.

The mechanical F-003 script subsumes option 1's manual quarterly cadence and is the better fit for the project's audit-driven cycle structure. **Option 2 (pre-commit) intentionally not adopted** — toolchain drift is not commit-blocking; it's milestone-relevant.

## References

- `docs/BUILD_SYSTEM_AUDIT.md` §P6 — what this audit is checking
- `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-04-27.md` — prior manual walk
- `scripts/audit/check_toolchain_drift.py` — mechanical implementation (F-2026-05-13-003)
- `logs/toolchain_drift.txt` — raw F-003 output for this cycle
- `standards/AUDIT_GUIDANCE.md` Tier 2.3 — procedure framing
- `standards/RP2350_ERRATA.md` — workaround status that depends on SDK version
- LL Entry 25 — debug probe is the primary flashing tool
- LL Entry 30 — codegen `.time_critical` SRAM placement
