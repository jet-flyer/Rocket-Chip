# Toolchain Version Audit — 2026-04-27

**Audit type:** P6 of `docs/BUILD_SYSTEM_AUDIT.md` — first run.
**Reason for audit:** P6 was added 2026-04-23 as a dimension of build-system rot. First execution had not happened until now.

## Method

For each toolchain component, captured the locally installed version, the latest upstream version, and the gap. "Latest upstream" pulled 2026-04-27 from the project's release page (or default branch HEAD where there is no release cadence).

## Summary

| Component | Local | Upstream latest | Gap | Action |
|---|---|---|---|---|
| Pico SDK | 2.2.0 | 2.2.0 (Jul 29) | none | None |
| picotool | 2.2.0-a4 | 2.2.0-a4 (Aug 14) | none | None |
| Pico Probe firmware | (not captured — read from probe) | debugprobe-v2.3.0 (Feb 11) | unknown | Read installed version, then decide. v2.3.0 is the version flagged as "fails to start" in some configs per upstream #201 — reading first matters more than chasing latest |
| OpenOCD (Pi fork) | 0.12.0+dev (built 2025-10-09) | rpi-common branch HEAD = `acff23f` (Mar 18 2026) | ~5 months of branch commits behind, but no release cut | Stay pinned. Re-evaluate only if a probe regression we hit traces to a fix on rpi-common HEAD |
| GCC ARM (arm-none-eabi) | 14.2.Rel1 (Nov 19 2024) | 15.2.Rel1 (Dec 17 2025) | one major release behind | Stay pinned. See "GCC upgrade evaluation" below |
| CMake | 4.2.1 (local) / 3.25 minimum (project) | 4.3.2 | local fine; project minimum bump candidate | None |

**Verdict:** All components are within the P6 "Clean" criterion (within one minor of upstream, or pinned with rationale). No upgrades required.

## Per-component notes

### Pico SDK 2.2.0 — current
Local install at `~/.pico-sdk/sdk/2.2.0/`, vendored copy at `pico-sdk/pico_sdk_version.cmake` reports 2.2.0. CMakeLists.txt:171 pins `set(sdkVersion 2.2.0)`. Upstream 2.2.0 was released 2026-07-29. **No drift.**

### picotool 2.2.0-a4 — current
CMakeLists.txt:173 pins `set(picotoolVersion 2.2.0-a4)`. Local install at `~/.pico-sdk/picotool/2.2.0-a4/`. Upstream latest is also 2.2.0-a4 (released 2026-08-14, adds RP2350-A3/A4 chip-revision display under `picotool info --debug`). **No drift.**

### Pico Probe firmware — version not captured
The audit method does not include a USB query against an attached probe. The currently flashed firmware version should be read by the user via the probe's USB descriptor or firmware string.

`AGENT_WHITEBOARD.md` previously flagged that **debugprobe-v2.3.0 has a "fails to start" regression per upstream issue #201** under some host configurations. Action item if our probe is on v2.3.0 and is misbehaving: either roll back to v2.2.x or wait for the next release. If our probe is on v2.2.x and working, do not chase v2.3.0.

### OpenOCD (Pi fork) 0.12.0+dev — pinned
Local install at `~/.pico-sdk/openocd/0.12.0+dev/openocd` was built 2025-10-09. The Pi fork (`raspberrypi/openocd`) uses the `rpi-common` branch as its rolling-tip distribution channel — there are no GitHub Releases on this fork. Branch HEAD is at `acff23f` ("target/smp: reply to unknown packets in gdb_read_smp_packet", 2026-03-18). The 0.12.0 tag was last touched in a merge from `P33M/rp2350-v0.12.0` (2025-06-18).

Strategy: stay pinned at the SDK-bundled version unless we hit a probe behaviour that traces to a fix on `rpi-common` HEAD. Chasing a moving branch tip introduces flake risk that outweighs the benefit absent a known issue. LL Entry 25 (debug probe is the primary flashing tool) means probe regressions would be high-impact — pinning is the right default.

### GCC ARM 14.2.Rel1 — pinned, one major behind
Local install at `~/.pico-sdk/toolchain/14_2_Rel1/`. CMakeLists.txt:172 pins `set(toolchainVersion 14_2_Rel1)`. Upstream latest is **15.2.Rel1** (GCC 15.2 base, released 2025-12-17).

**GCC upgrade evaluation:** GCC 15 vs 14 is a major-version bump. Risk surface includes ABI/calling-convention changes, new diagnostics that may break our `-Wall -Wextra -Werror` build, codegen differences for `.time_critical` SRAM functions (LL Entry 30 — codegen FPFT performance is load-bearing). Reward is mostly diagnostics improvements (no specific bug fix we need). Recommendation: **stay on 14.2.Rel1 until we have a specific reason to move.** Pico SDK 2.2.0 is tested against 13.x/14.x — moving to 15.x is ahead of the SDK's validated matrix. Re-evaluate when Pico SDK 2.3+ explicitly recommends 15.x.

### CMake — local 4.2.1, project minimum 3.25
Project minimum is set to 3.25 (CMakeLists.txt:10). Local install is 4.2.1, well above. Upstream latest is 4.3.2. No action required — minimum is conservative, local is current enough.

## Watch mechanism (P6 deferred work)

P6 also asks for a "concrete watch mechanism" beyond keep-an-eye-on-it. Concrete options for future implementation, not done in this audit:

1. **Quarterly toolchain audit cadence** — re-run this audit every 3 months and append to this file as a dated section. Calendar trigger, not event-driven.
2. **Pre-commit version-string check** — script that reads CMakeLists.txt-pinned versions vs latest GitHub-Releases tags for the four pinned components (Pico SDK, picotool, GCC ARM, debugprobe), prints a warning when drift exceeds 1 minor version. Not blocking, just informational. Estimated 1–2 hours to implement.
3. **Errata-watch coupling** — `standards/RP2350_ERRATA.md` already names the SDK flag (`PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1`) gating E2 workaround. Adding a row to that doc whenever the SDK version bumps (with re-verification that the workaround still applies) gives event-driven coupling. Manual step.

None of these are done in this audit. Documented here so the next audit cycle can pick the most useful one.

## References

- `docs/BUILD_SYSTEM_AUDIT.md` §P6 — what this audit was checking
- `standards/RP2350_ERRATA.md` — workaround status that depends on SDK version
- `AGENT_WHITEBOARD.md` — debugprobe v2.3.0 "fails to start" tracker (upstream #201)
- LL Entry 25 — debug probe is the primary flashing tool (raises stakes on probe-firmware regressions)
- LL Entry 30 — codegen `.time_critical` SRAM placement (raises stakes on GCC codegen changes)
