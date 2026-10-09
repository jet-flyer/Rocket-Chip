# Hardware Gate Discipline

**Status:** Normative. Applies to every IVP gate or session-end gate that involves external hardware (sensors, radios, GPS, USB, debug probe, soak runs).
**Authored:** 2026-04-27 (this session).
**Origin:** User ask 2026-04-17 after the IVP-140 false-positive + Fruit Jam GPS cable episode (`docs/plans/STAGE16C_STATION_DECOUPLING_STATUS.md`). Same meta-pattern as `LESSONS_LEARNED.md` Entry 36 (bench_sim silent rot).
**Related rules:** `standards/CODING_STANDARDS.md` (Code Verification Process), `docs/agents/SESSION_CHECKLIST.md` (build-parity check), `docs/CONFIG_TEST_MATRIX.md` (Tier 6b CI matrix).

---

## The problem this rule fixes

A hardware gate that says "build clean + MSP stable for 5 min" can pass while the hardware is actually broken. Two real incidents on this project:

- **IVP-140 (2026-04-15..17):** Station scaffolding gated on "no-op tick runs without crashing under `kRadioModeRx`." It passed. The hardware was delivering nothing useful - the I2C bus had a marginal SDA line, the onboard DAC at 0x18 was silently held in reset, and the GPS module wasn't ACKing PMTK writes. Several hours of firmware bisection later the root cause turned out to be a bad STEMMA QT cable. The firmware code-path-didn't-crash gate was satisfied; the *system actually working* gate was not.
- **bench_flight_sim.py rot (LL Entry 36):** A test script's regex went stale when a firmware log line was renamed. Five days of commits claimed "bench sim 9/9 PASS" without anyone running the script. The script's own self-consistency check was missing.

The unifying defect is **gates that say "X did not fail"** (negative evidence) rather than **gates that say "X delivered an observable signal that proves the hardware is working"** (positive evidence). Negative-evidence gates pass on broken hardware whenever the firmware happens to exit early, fall through to a default, or silently tolerate the failure.

---

## Rule 1. Every HW gate must name a positive-control signal

A gate definition is incomplete unless it specifies an observable signal that:

1. **Proves the bus, peripheral, or radio link is healthy** - independent of the device under test.
2. **Cannot be produced by firmware error-handling fallback** - must come from the hardware itself, observable in serial output or GDB.
3. **Is named explicitly in the gate text** - not "the soak ran fine" but "Hardware Status shows DAC at 0x18 ACK = OK *and* GPS PMTK round-trip = `[51,18,51]`."

Examples of valid positive-control signals already in the firmware:

| Signal | What it proves | Where it appears |
|---|---|---|
| `[FD] PYRO FIRED: DROGUE (primary)` | Action callback dispatched, log path live | `bench_sim.py` happy-path |
| `RegVersion=0x12` | RFM95W SPI bus + chip select working | Hardware Status `b` |
| `DAC 0x18 ACK = OK` | I2C bus electrically healthy | Hardware Status `b` |
| `GPS PMTK round-trip [51,18,51]` | UART/I2C GPS path healthy *and* MT3333 in I2C mode | Hardware Status `b` |
| `window_hit:1 init:1` | GPS init loop hit its timing window on first try | `[DBG ] GPS early-init:` |
| Sensor-rate counters incrementing under MSP soak | Core 1 actually polling, not stalled | `diag_stats_dump()` |

**Anti-examples** (do NOT use as the only gate evidence):
- "5-minute MSP soak stable" - *silent firmware = stable MSP*
- "no QP assertion" - *firmware can run for hours after a sensor fault before triggering an assertion*
- "build compiles clean" - *says nothing about the hardware*
- "ctest passes" - *host tests don't touch hardware*
- "bench_sim.py PASS" without a regex audit - *bench_sim's own log-line tokens can rot* (LL Entry 36)

A gate that names *only* anti-examples is a **soft gate dressed as a hard gate** and should be downgraded.

---

## Rule 2. Gates that depend on external hardware require a 3-boot repeatability protocol

Single-boot evidence is insufficient: a transient init quirk, an SDK timing window, or a one-off USB enumeration race can produce a passing gate that doesn't reproduce. Observing the same positive-control signal across multiple cold boots is the actual question.

**Protocol:**
1. **Boot 1:** Cold-restart the board, observe gate signals.
2. **Cold-restart again.**
3. **Boot 2:** Observe gate signals.
4. **Cold-restart again.**
5. **Boot 3:** Observe gate signals.

A "cold restart" means a full chip reset that exercises the boot path from scratch. USB replug is a slave POR (STEMMA 3V3 drops). CLI `k` (ABORT-then-STOP then watchdog) is an MCU restart on an idle bus. Do **not** use probe `monitor reset halt` with STEMMA QT powered: `rp2350.cfg` has no SRST, so that is MCU-only SYSRESETREQ while 3V3 stays up (Analog AN-686 / ICM `I2C_IF_DIS`). USB unplug is recovery when stuck, not a counted-boot step.

**Post-flash:** iterative SWD flash is `scripts/flash_elf_halt_write.py` (park, halt, `write_image`, vector-resume). That vector-resume **is** Boot 1 once CDC and LED are up. Do not follow it with `reset halt` - that extra restart latched Hardware 10/13. OpenOCD `program` is the leftover-image failure mode (it `reset init`s the old image first); this path does not do that. Do not start the 3-boot count while CDC is missing or the LED is off - attach stops the cores, so a missing LED during the flash session is not by itself a failed boot. See `docs/FLASHING.md`.

Gate **passes** only if all 3 boots produce the same positive-control signals.

**Why 3 and not 2:** Two boots tell you "twice in a row" (a single coin-flip can do that). Three identical results across two restart events tells you "consistent behavior across cold-init" - that's the actual question.

**Cabling is not in scope.** Earlier versions of this rule prescribed physical cable reseating as part of the protocol; that requirement is removed (user direction 2026-05-16). If a cable is bad on this bench it stays bad - operator inspection is the right answer, not a per-gate workflow. Reseating Qwiic / STEMMA QT cables to "stir" intermittent connections is anti-discipline: it hides a hardware fault we should be fixing, and it adds a manual step that has nothing to do with what the gate is observing.

**Exemption:** Pure-software changes (host tests, doc-only, comment-only, build-system reformatting that doesn't change a compiled artifact) skip the 3-boot protocol. The `pre_commit_matrix.py` triggers (Tier 6b) classify which changes need HW verification at all; this rule applies only to changes that already need HW verification.

---

## Rule 3. Commit messages must cite the observed control signal, not just the gate name

A commit message that says **"build clean, MSP stable, gate PASS"** records that the agent ran the gate but does not let a future agent (or the user) verify that the gate was satisfied for the right reason.

**Required form:**
> Verified: bench_sim 2/2 PASS, vehicle Hardware Status reports DAC=OK, RegVersion=0x12, GPS window_hit:1 across 3 cold-restart boots.

Or, if a probe-attached ctest run was the verification:
> Verified: 788/788 ctest, vehicle bench_sim 2/2 PASS (banner: vehicle v0.16.0 (kmenu)), 3-boot repeatable.

Or, if the 3-boot protocol was waived:
> Verified: pure-software change, host ctest 788/788 PASS, no HW restart cycle required.

Or, for a firmware commit (the bench runs at push, Rule 5):
> Verified: host ctest 788/788, flight target cross-compile both roles; bench at push.

Or, for the push (the line the pre-push gate prints; put it in the last commit or the CHANGELOG entry):
> Verified at push: vehicle 2/2 PASS (flight-<sha>), station 3/3 PASS (flight-<sha>); firmware tree <hash12>; range <first>..<last>.

This requirement extends `standards/CODING_STANDARDS.md` Pre-Commit Checklist Item 4 (Documentation). The CHANGELOG entry inherits the same signal, and the WB row that depends on the gate inherits it transitively.

---

## Rule 4. Gates without a positive-control signal are soft gates

If a gate's definition only includes anti-evidence ("did not crash," "compiles clean," "no assertion fired"), it is **structurally soft** in the LL Entry 36 sense - it can pass while the system is broken. Two responses:

1. **Strengthen the gate:** add a positive-control signal that the gate must observe. Once added, the gate is hard.
2. **Accept the gate as soft and label it explicitly:** the plan doc / IVP entry / commit message says **"soft gate (no positive-control signal available - requires human attention)"**. This is fine for early-bring-up scaffolding where a positive-control signal genuinely doesn't exist yet, but it's never fine to *present* a soft gate as a hard one.

Plan documents (`docs/plans/*.md`, `docs/IVP.md`) should classify each gate as **hard** or **soft** explicitly. Hard gates are mechanically enforced (SPIN, host tests via pre-commit, clang-tidy, bench_sim.py with audited regex). Soft gates require the human to satisfy themselves the gate was met for the right reason.

---

## Rule 5. Gates do not get silently skipped - the pre-push hook is the hardware line of defense

This rule addresses a separate failure mode from Rules 1-4: **gates that are claimed but never run.** The history is in `docs/plans/STAGE_P7_15_SHELVED_2026-04-11.md`: plans list "HW verify" as a gate, commits cite it, but running the gate is honor-system. Same root cause as LL Entry 36 (bench_sim regex rot).

### Hook vs canary vs recovery (same script, different question)

`scripts/bench_sim.py` is reused in three roles. Do not conflate them:

| Role | When | Tree / board | Question |
|------|------|--------------|----------|
| **Pre-push gate** (`scripts/hooks/pre-push`) | Once per push, on the commit being pushed (not HEAD) | Builds that commit in the clean bench worktree. Needs OpenOCD `:3333` (started from the main checkout). Does **not** flash. `bench_sim --commit <sha>` **refuses leftover**: banner `flight-<sha>` must match the ELF's `kGitHash`, the ELF's `kFirmwareTreeId` must equal the pushed commit's firmware-tree hash (paths: `scripts/ci/firmware_paths.txt` `[elf]`), and `rocketchip.elf.flashed.json` must be this ELF's sha256 (written by `flash_elf_halt_write.py` after `verify_image`, or by `--record-only`). A PASS is stored as a git note on the bench key (firmware tree, gate files, compiler, picotool and pioasm versions, Pico SDK, CMake presets). The note records the USB serial of each benched board. A push with the same bench key skips the flash only when those boards are attached and fresh mode is off | Stop "claimed PASS, nobody ran it" (prevention / LL 36) **and** "PASS on last week's image" (desk 2026-09-07) |
| **Flight-path canary** | *Before* editing flight-critical paths, only if that work is in play | Prefer unchanged HEAD if probe is up; see During Session in `docs/agents/SESSION_CHECKLIST.md` | Has the tool/gate itself rotted since last real run? |
| **Recovery** | After a skip, an ambiguous hook result, or on a commit that no gate benched | Flash last-known / `HEAD~` or known-good image, then re-run | Separate "gate broken" from "this sitting's code broke it" |

A full **session-start** canary on unchanged HEAD is often **not feasible** here: the probe is rarely up at session open, and the first `Read` of `src/` is the wrong moment. Prefer the checklist Flight-path canary when flight work is actually in play, plus the gate on push.

The pre-commit hook stays fast: `<stdio.h>` ban, clang-tidy, host ctest, and the flight target cross-compile for both roles. It does not touch the board.

**List completeness.** The build ID and the bench key hash only the `[elf]` paths. A file that the build uses but `[elf]` does not list could change, and an old PASS would still match. `scripts/ci/check_elf_paths.py` closes this gap. After each flight build it reads `ninja -t deps` (compiler dep files), `compile_commands.json`, `ninja -t inputs rocketchip.elf` (for example `.pio` programs and the linker script) and the `build.ninja` regeneration inputs (`CMakeLists.txt`, `*.cmake`). It fails when a repo file is not in `[elf]`. It ignores files outside the repo and generated files in build dirs. The pre-commit hook runs it after the cross-compile. The pre-push gate runs it again on the bench worktree build.

**Why the build ID is a firmware-tree hash.** An identity gate that hashes the whole tree (`git describe --dirty`) makes every commit a rebuild and a reflash, even a docs-only one. That cost brings back the `--no-verify` habit. The firmware-tree hash covers only `[elf]` paths, so a docs commit after the flash stays valid.

### Mechanical defenses (strongest first)

1. **Pre-push hook runs the gate itself.** `scripts/hooks/pre-push` + `scripts/ci/pre_push_gate.py` + `scripts/ci/pre_commit_matrix.py` do this for the bench_sim path (vehicle and station), once per push. The agent does not choose whether the gate runs - the hook runs it, observes the exit code, and blocks the push on failure. Install: `bash scripts/hooks/install.sh`.
2. **Flight-path canary** - During Session in `docs/agents/SESSION_CHECKLIST.md` (not a fixed numbered Session Start item): run `bench_sim.py` before editing flight-critical paths listed there, catching rot from commits that no gate benched.
3. **Bypass discipline** - agents never use `--no-verify`, on commit or on push. A bench failure is fixed in the unpushed commit (`git commit --fixup=<sha>`, then `git rebase -i --autosquash`).

What this means in practice:

- **If your push includes a firmware or gate path, the pre-push hook benches the pushed commit and blocks on failure.** You do not have to remember; the hook remembers.
- **Roles (Nathan, 2026-10-08).** Both roles build from one source list. A firmware (`[elf]`) change benches BOTH roles: vehicle (`bench_sim.py`) and station (`station_bench_sim.py`). A `[gate]` change benches BOTH roles. One exception: when every firmware path in the push is in `[station-only]`, only the station bench runs. A radio push benches both roles and the radio link. The role table in `scripts/ci/pre_push_gate.py` (`ROLES`) is the only place that maps a role to its build dir, preset, bench script and flash path.
- **If OpenOCD is not on `127.0.0.1:3333` when you push, the hook fails closed.** Start OpenOCD from the main checkout and push again - do not bypass. The gate prints the main checkout folder (the first line of `git worktree list`) and the start script. Never start OpenOCD from the bench worktree: the running process locks the folder it starts in.
- **If the chip is not this ELF, the hook fails closed.** Flash `build_flight/rocketchip.elf` (or the station ELF) with `scripts/flash_elf_halt_write.py`, wait LED+CDC per `docs/FLASHING.md`, then retry. A leftover banner or a rebuilt ELF with no new flash record is a failed gate, not a skip. `--record-only` is only for a flash that already verified by another documented method (picotool).
- **The ELF to flash is in the bench worktree** (`<main checkout>-bench/build_flight/rocketchip.elf`, or `build_station_flight/`). The gate prints the exact paths. Both setups flash by Path 1: `picotool load -f` over USB (`-f` returns to the app by itself), then `--record-only`, then wait for LED and CDC (CLI `k` if a fresh boot is needed; no `reset halt`, per `docs/FLASHING.md` rule 2). The debug probe does not flash in this procedure; it serves only the OpenOCD check. Path 2 (`flash_elf_halt_write.py`, probe halt-write) stays an alternative for the setup with the debug probe.
- **If the gate runs but you suspect a false PASS**, the positive-control signal was insufficient - open a Rule 1 strengthening followup; do not silently re-run until it "passes."
- **For gates that cannot be a hook-runnable script** (most field-test / stage-exit gates), the gate is structurally soft per Rule 4.

### Push shape and radio pushes

- **Bench notes stay local.** The PASS notes (`refs/notes/rc-bench`) are in the bench PC's clone. All pushes go from that PC, and its worktrees share `refs/notes`. Do not push the notes.
- **Bench key and note** (`scripts/firmware_tree.py`, `scripts/ci/pre_push_gate.py`):
  - The bench key (`rc-bench-key v2`) is a git blob id over the `arm-none-eabi-gcc --version`, `picotool version` and `pioasm --version` lines, the Pico SDK (path, `git describe --always --dirty --tags`, `git rev-parse HEAD`, `git submodule status --recursive`), the role CMake presets, and the `[elf]` and `[gate]` listings at the pushed commit.
  - The Pico SDK is the one the flight builds use (`PICO_SDK_PATH` in their CMakeCache.txt; else what CMake would pick). It must be a git checkout. If it is not, or a git query fails, the push stops. The key never falls back to a version name.
  - The gate finds each tool by `RC_ARM_GCC` / `RC_PICOTOOL` / `RC_PIOASM`, then by the flight build CMakeCache.txt, then by the Pico VS Code install that `CMakeLists.txt` pins, then by `PATH`. `RC_PICO_SDK` overrides the SDK. A missing tool or an empty version line blocks the push. The key never hashes an empty value.
  - After the build, the compiler, picotool, pioasm and Pico SDK named in `CMakeCache.txt` must give the same key lines. If not, the gate blocks and names the variable to set.
  - The PASS note (`rc-bench v2`) has a `boards:` line, for example `boards: vehicle=<USB serial> station=<USB serial>`. The note also records every `[tool]` key line (compiler, picotool, pioasm, Pico SDK, presets). The bench scripts print the serial (`board_usb_serial:`). The gate reads the attached serials by USB enumeration only. It opens no port. This line is a record. It is not a board-ID check before a flash.
  - The gate reuses a note only when it says PASS, its `roles:` line covers the push, and each recorded board serial is attached now.
  - The USB serial comes from the MCU board's flash chip. It does not change when the radio board changes. After a radio board swap, or a jumper, solder or antenna change, use fresh mode: `RC_BENCH_FRESH=1 git push` (`--plan --fresh` for a plan). Fresh mode ignores all notes.
- **Push shape is an agent rule** (`docs/agents/SESSION_CHECKLIST.md` 6b). The gate prints a WARNING; it does not block.
  - At most 5 firmware commits per push. One bench run covers the whole range, so a failure must stay traceable to one commit.
  - A pyro/deploy, guard, radio-adapter or turnaround commit goes in a push of its own (`[isolate:*]` in `scripts/ci/firmware_paths.txt`).
- **Radio push** (any `[radio]` path): the link has two ends, so BOTH boards are benched, even when only one image changed. Both roles build from one source list, so the file list decides only which board needs a fresh flash. The other board must run a known-good image: a clean build (no `-dirty`) whose firmware tree has a bench PASS note for that role, with the same air protocol (build ID check). Preconditions: (1) the station board is powered and runs a known-good image (clean build, bench PASS note for that role, same air protocol); (2) desk radio power stays at 2 dBm; (3) a forced radio HW TX timeout fires - a clean round trip alone does not prove the timeout works. Then the bench must show an actual radio HW TX/RX round trip between the boards. **Not built yet:** the two-board round-trip script (`scripts/radio_link_bench.py`), the forced TX-timeout hook and the 2 dBm readback. The blocking radio rule lands in the same commit as that script, not before. Until then the gate prints a WARNING, and the agent runs the two-board soak by hand and cites it (agent rule).

Station-only pushes: when every firmware path is in `[station-only]`, the gate benches the station only and does not need OpenOCD (the station setup has no debug probe). All other firmware and gate pushes bench both roles.

## Rule 6. Local-commit verification ≠ audit-suite regression credit

This rule distinguishes two levels of verification credit that commit messages and finding-closure claims must not conflate. Rules 1-3 already say *what* a gate must observe (positive-control signal) and *how* to cite it; Rule 6 says *at what level of credit* a given verification counts.

**Local-commit verification (level 1).** A single focused commit or push observes its own gate - the pre-push gate's bench_sim, the change's claimed positive-control signal, host ctest covering the touched path. This is what Rule 3 commit-message citation captures. It proves the change does what it claims at the moment of the commit, in isolation, on the author's bench.

**Audit-suite regression credit (level 2).** The original audit's scripted check suite re-runs end-to-end *after* a remediation cycle (or a category of remediations, per `standards/AUDIT_GUIDANCE.md` Appendix C.5) and observes the full positive-control set the audit defined. This is what closes a Problem Report in `docs/PROBLEM_REPORTS.md` from `verified` to `closed`, and what signs a dated audit report's `## Remediation` row.

**Why this matters.** Local-commit verification is necessary but not sufficient. The LL Entry 36 framing applies: a gate that passes locally can still be silently broken at the integration level (other findings touched the same file; a shared mechanism was changed; the audit's own check rotted). Audit-suite regression is the level that catches inter-finding interactions; local verification by definition cannot see them.

**Required form.** When citing verification:

- Use "Verified locally:" for level 1 - the change's own commit gates, captured per Rule 3.
- Use "Verified at audit regression:" for level 2 - the audit-suite re-run that confirms the change still holds in the as-shipped state.
- Use "Verified independently:" for the strongest form - a separate session, a re-audit, or a different agent ran the regression and reached the same verdict. Per DO-178C's independence principle, the fix author's local re-test has less credibility than an independent verifier's.

A commit message may legitimately cite *only* level 1 - that's the normal per-commit state. A dated audit report's `## Remediation` section row marked `closed` must cite at least level 2. A `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` entry derived from an audit finding must cite at least level 2 plus the user-acceptance signature.

**Anti-pattern.** A commit message that says "Verified: audit-suite PASS" when only the local commit gate ran. That is the failure mode this rule prevents - it overclaims level 2 credit for level 1 work. If level 2 hasn't actually run, the commit message says level 1 (and the audit-suite re-run happens later at the proper gate per AUDIT_GUIDANCE Appendix C.5).

**Why this didn't need a separate rule until now.** Before the 2026-05-07 audit cycle, the project had ad-hoc framing for verification levels; commits cited "Verified: ctest passes" and that was fine because the work was incremental. With audit-driven remediation cycles producing batches of related fixes, the local-vs-audit distinction becomes load-bearing - without it, the audit's `## Remediation` section can fill with claimed-PASS rows that only saw level 1 verification, and the audit-suite regression is never actually performed. Source: DO-178C verification-independence principle + LL Entry 36 pattern + static-analysis-tool community's distinction between merge-blocker and backlog credit.

---

## Rule 7. Bugs surfaced by verification are in-scope for the PR that surfaced them

**When verifying PR-X reveals bug-Y that PR-X's verification path depends on, the fix for bug-Y rides in the same commit (or back-to-back commits with the same PR ID) as PR-X.** Bug-Y is *not* a separate PR.

The reason: bug-Y was hidden by the same defect PR-X exists to fix. Splitting it off as a new PR pretends an independence that wasn't there and creates a verification gap - PR-X gets marked `closed` while its own verification path could not actually run.

**Test:** did fixing PR-X's primary defect *reveal* bug-Y, or was bug-Y merely coincident nearby code?

Surfaced (in-scope):
- PR-X's primary defect was masking bug-Y. Fixing X is what exposes Y.
- PR-X's verification recipe cannot produce its positive-control signal until bug-Y is also fixed.
- Bug-Y is in code PR-X is touching and a future reader following the PR-X trail will hit it.

Coincident (split into a new PR as normal):
- Bug-Y is in a different subsystem with no causal link to the masking defect.
- Bug-Y existed independently and PR-X just happens to be in the same file.
- Fixing bug-Y doesn't change PR-X's verification outcome.

**Commit-message form.** A PR-X commit that fixes a surfaced bug-Y names the surfaced bug, states why it was in-scope (one of the surfaced-indicators above), and cites the verification that proves both fixes work together. The verification citation per Rule 6 still applies - Level 1 local + Level 2 audit-suite-style, both on the combined scope.

**Example.** R-3 (audit 2026-05-07) replaced a halt-forever fault handler with capture-and-reset. Verification surfaced two pre-existing MPU bugs that the halt-forever handler had hidden because the handler was never actually exercised: an `AP=0b00` encoding mistake in `mpu_setup_stack_guard()` (which allowed privileged writes through the guard region - defeating the guard's purpose) and a missing `MEMFAULTENA` enable (which made MPU violations escalate to HardFault instead of invoking the dedicated MemManage handler). Both rode in the R-3 commit because R-3's verification path required them to be fixed - without the AP correction, `fault_force_hardfault()` would never produce a fault; without MEMFAULTENA, the fault wouldn't route to the new handler. R-3 closed with both fixes verified together; no separate "R-15 / MPU AP encoding" or "R-16 / MEMFAULTENA" was opened.

**Counter-example to avoid.** Treating each surfaced bug as a new PR means PR-X ships under-verified (its own verification recipe couldn't run on the current commit), the new PRs ship in isolation later with their own verification (proving the surfaced bug fix works but not proving PR-X works), and the audit's `## Remediation` section has a row that says `closed` for a PR whose verification never ran. That looks identical to a clean closure in diff history, which is precisely the kind of structural-soft-gate failure LL Entry 36 documents.

Source: project policy 2026-05-12, lived-experience case is R-3 above. Companion rule to AUDIT_GUIDANCE.md Appendix C.4a.

---

## Rule 8. Flight release is its own gate

Only a tagged, bench-passed commit flies. The pre-push gate refuses a release tag (`vMAJOR.MINOR.PATCH` exactly, the form in `standards/VERSIONING.md` "How to cut a release") on a commit with no bench PASS note. Other tags (`starcom-v*`, `pre-*`, `archive/*`) are not release tags. Before flight, run the soak test and the flight checklist on that exact binary (same ELF sha256). A 6.5 s bench pass is not flight qualification. **GAP:** no tracked record yet ties a soak run and a checklist run to an ELF sha256; until one exists, this part is a soft gate (Rule 4) and the repo owner signs it.

---

## How this interacts with existing standards

- **Extends `standards/CODING_STANDARDS.md` Pre-Commit Checklist** - Items 1-4 become "and the commit message cites the observed control signal."
- **Extends `docs/agents/SESSION_CHECKLIST.md`** - Item 9 (commit) requires citing the observed control signal per Rule 3. Item 6 (session-start canary) is the Rule 5 defense against pre-existing rot.
- **States the bypass rule** - never use `--no-verify`, on commit or on push (Rule 5 defense 3; `standards/GIT_WORKFLOW.md` "Local history").
- **Consolidates the gate-classification framing from `docs/plans/STAGE_P7_15_SHELVED_2026-04-11.md`** - the "verification gates claimed but not performed" history is now codified as a normative rule.
- **Does NOT modify `docs/CONFIG_TEST_MATRIX.md`** - the matrix specifies *which* changes trigger HW gates; this doc specifies *what* a HW gate must observe to pass and *what* discipline keeps the gate from being silently skipped.
- **Names the pre-push hook as the primary Rule-5 defense.** The pre-commit hook keeps the fast gates and the flight target cross-compile; the bench moved to pre-push (2026-10-08 council: bench once per push).

---

## When this rule does NOT apply

- **Pure-software changes** (host tests, doc-only, comment-only, build-system reformatting that doesn't produce a different compiled artifact). `scripts/ci/pre_commit_matrix.py` (paths in `scripts/ci/firmware_paths.txt`) classifies these, and the pre-push gate skips HW gates for them.
- **Off-line analysis** (replay-harness runs against logged data, post-flight log decoders, math-only refactors verified by host ctest). No live hardware, no HW gate.

---

## Open follow-ups (to be tracked outside this doc)

The following are deliberately not in this rule because they're tooling work, not standard:

1. **Pre-commit hook enforcement of "commit message cites control signal"** - possible regex check on the staged commit message body. Out of scope for this rule; logged as a candidate for `scripts/hooks/pre-commit` Tier 6c.
2. **Per-gate positive-control catalog** - the table in Rule 1 should grow as new gates are added. Catalog can live in `docs/CONFIG_TEST_MATRIX.md` (Tier 7?) or as a structured appendix to this doc.
3. **Automated 3-boot verification** - the probe can in principle reset the chip 3× with a script comparing positive-control signals across boots. Possible future enhancement.
