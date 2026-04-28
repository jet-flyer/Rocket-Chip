# Hardware Gate Discipline

**Status:** Normative. Applies to every IVP gate or session-end gate that involves external hardware (sensors, radios, GPS, USB, debug probe, soak runs).
**Authored:** 2026-04-27 (this session).
**Origin:** User ask 2026-04-17 after the IVP-140 false-positive + Fruit Jam GPS cable episode (`docs/plans/STAGE16C_STATION_DECOUPLING_STATUS.md`). Same meta-pattern as `LESSONS_LEARNED.md` Entry 36 (bench_sim silent rot).
**Related rules:** `standards/CODING_STANDARDS.md` (Code Verification Process), `.claude/SESSION_CHECKLIST.md` (build-parity check), `docs/CONFIG_TEST_MATRIX.md` (Tier 6b CI matrix).

---

## The problem this rule fixes

A hardware gate that says "build clean + MSP stable for 5 min" can pass while the hardware is actually broken. Two real incidents on this project:

- **IVP-140 (2026-04-15..17):** Station scaffolding gated on "no-op tick runs without crashing under `kRadioModeRx`." It passed. The hardware was delivering nothing useful — the I2C bus had a marginal SDA line, the onboard DAC at 0x18 was silently held in reset, and the GPS module wasn't ACKing PMTK writes. Several hours of firmware bisection later the root cause turned out to be a bad STEMMA QT cable. The firmware code-path-didn't-crash gate was satisfied; the *system actually working* gate was not.
- **bench_flight_sim.py rot (LL Entry 36):** A test script's regex went stale when a firmware log line was renamed. Five days of commits claimed "bench sim 9/9 PASS" without anyone running the script. The script's own self-consistency check was missing.

The unifying defect is **gates that say "X did not fail"** (negative evidence) rather than **gates that say "X delivered an observable signal that proves the hardware is working"** (positive evidence). Negative-evidence gates pass on broken hardware whenever the firmware happens to exit early, fall through to a default, or silently tolerate the failure.

---

## Rule 1. Every HW gate must name a positive-control signal

A gate definition is incomplete unless it specifies an observable signal that:

1. **Proves the bus, peripheral, or radio link is healthy** — independent of the device under test.
2. **Cannot be produced by firmware error-handling fallback** — must come from the hardware itself, observable in serial output or GDB.
3. **Is named explicitly in the gate text** — not "the soak ran fine" but "Hardware Status shows DAC at 0x18 ACK = OK *and* GPS PMTK round-trip = `[51,18,51]`."

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
- "5-minute MSP soak stable" — *silent firmware = stable MSP*
- "no QP assertion" — *firmware can run for hours after a sensor fault before triggering an assertion*
- "build compiles clean" — *says nothing about the hardware*
- "ctest passes" — *host tests don't touch hardware*
- "bench_sim.py PASS" without a regex audit — *bench_sim's own log-line tokens can rot* (LL Entry 36)

A gate that names *only* anti-examples is a **soft gate dressed as a hard gate** and should be downgraded.

---

## Rule 2. Gates that depend on external hardware require a 3-boot reseat protocol

Cable, connector, and EMI variance are higher-frequency than firmware variance. A single boot satisfies neither (a) a reasonable mean-time-between-spurious-pass nor (b) a fair sample of the population of plug-states the operator will encounter.

**Protocol:**
1. **Boot 1:** Power on, observe gate signals.
2. **Power off, fully reseat the cables** (unplug, count to 3, plug back in — counts as a single reseat).
3. **Boot 2:** Power on, observe gate signals.
4. **Power off, swap to a second cable from the bench inventory** (or, if only one cable available, reseat again). For STEMMA QT specifically: pull and re-insert until the click is unambiguous.
5. **Boot 3:** Power on, observe gate signals.

Gate **passes** only if all 3 boots produce the same positive-control signals.

**Why 3 and not 2:** Two boots tell you "twice in a row" (a single coin-flip can do that). Three identical results across two reseat events tells you "consistent behavior across mechanical disturbance" — that's the actual question.

**Exemption:** Pure-software changes (host tests, doc-only, comment-only, build-system reformatting that doesn't change a compiled artifact) skip the reseat protocol. The `pre_commit_matrix.py` triggers (Tier 6b) classify which changes need HW verification at all; this rule applies only to changes that already need HW verification.

---

## Rule 3. Commit messages must cite the observed control signal, not just the gate name

A commit message that says **"build clean, MSP stable, gate PASS"** records that the agent ran the gate but does not let a future agent (or the user) verify that the gate was satisfied for the right reason.

**Required form:**
> Verified: bench_sim 2/2 PASS, vehicle Hardware Status reports DAC=OK, RegVersion=0x12, GPS window_hit:1 across 3 reseat boots.

Or, if a probe-attached ctest run was the verification:
> Verified: 788/788 ctest, vehicle bench_sim 2/2 PASS (banner: vehicle v0.16.0 (kmenu)), 3-boot reseat clean.

Or, if reseat was waived:
> Verified: pure-software change, host ctest 788/788 PASS, no HW reseat required.

This requirement extends `standards/CODING_STANDARDS.md` Pre-Commit Checklist Item 4 (Documentation). The CHANGELOG entry inherits the same signal, and the WB row that depends on the gate inherits it transitively.

---

## Rule 4. Gates without a positive-control signal are soft gates

If a gate's definition only includes anti-evidence ("did not crash," "compiles clean," "no assertion fired"), it is **structurally soft** in the LL Entry 36 sense — it can pass while the system is broken. Two responses:

1. **Strengthen the gate:** add a positive-control signal that the gate must observe. Once added, the gate is hard.
2. **Accept the gate as soft and label it explicitly:** the plan doc / IVP entry / commit message says **"soft gate (no positive-control signal available — requires human attention)"**. This is fine for early-bring-up scaffolding where a positive-control signal genuinely doesn't exist yet, but it's never fine to *present* a soft gate as a hard one.

Plan documents (`docs/plans/*.md`, `docs/IVP.md`) should classify each gate as **hard** or **soft** explicitly. Hard gates are mechanically enforced (SPIN, host tests via pre-commit, clang-tidy, bench_sim.py with audited regex). Soft gates require the human to satisfy themselves the gate was met for the right reason.

---

## Rule 5. Gates do not get silently skipped — the hook is the line of defense

This rule addresses a separate failure mode from Rules 1-4: **gates that are claimed but never run.** The history is in `docs/plans/STAGE_P7_15_SHELVED_2026-04-11.md`: "plans list 'HW verify' as a gate, commits cite it in messages, but running the gate is honor-system and can be skipped silently. When skipped, downstream bit-rot accumulates until someone actually runs the gate." Same root cause as LL Entry 36 (bench_sim regex rot — the gate had been formally claimed across multiple commits without anyone running it).

The mechanical defenses against this, in order of strength:

1. **Pre-commit hook runs the gate itself.** `scripts/hooks/pre-commit` + `scripts/ci/pre_commit_matrix.py` already do this for the bench_sim path (vehicle and station). The agent does not get to choose whether the gate runs — the hook runs it, observes the exit code, and blocks the commit on failure. This is the strongest mechanical defense available for any gate that can be expressed as a script.
2. **Session-start canary** — `.claude/SESSION_CHECKLIST.md` item 6 runs `bench_sim.py` before any flight-critical work begins, catching rot introduced by previous sessions that committed via `--no-verify` when the probe was not available.
3. **Bypass discipline** — `git commit --no-verify` is permitted only with explicit repo-owner approval per `.claude/DEBUG_PROBE_NOTES.md`. Autonomous agents must not bypass the hook unless the human author instructed it. A commit that bypassed the hook must say so in its commit message and explain why.

What this means in practice:

- **If your change touches a path that the matrix classifies as flight-critical or station-relevant, the hook will run the corresponding bench_sim and block the commit on failure.** You don't have to remember; the hook remembers.
- **If OpenOCD is not on `127.0.0.1:3333` when you commit, the hook fails closed** with an explicit error. The right response is to start OpenOCD and re-attempt, *not* to bypass.
- **If the gate runs but you suspect the result was wrong** (e.g., hook said PASS but the firmware behavior on bench is different), the gate's positive-control signal was insufficient — open a Rule 1 strengthening followup, do not silently re-run until it passes.
- **For gates that cannot be expressed as a hook-runnable script** (most field-test gates, most plan-level "stage exit" gates), the gate is structurally soft per Rule 4. State that explicitly in the plan and require the agent's commit to cite the *content* of what they observed, not just the gate name.

The combination of these three defenses means an agent cannot pass a hard HW gate without either (a) actually running it via the hook, or (b) explicitly bypassing with documented user approval. There is no third path.

---

## How this interacts with existing standards

- **Extends `standards/CODING_STANDARDS.md` Pre-Commit Checklist** — Items 1-4 become "and the commit message cites the observed control signal."
- **Extends `.claude/SESSION_CHECKLIST.md`** — Item 9 (commit) requires citing the observed control signal per Rule 3. Item 6 (session-start canary) is the Rule 5 defense against pre-existing rot.
- **Consolidates the bypass discipline already in `.claude/DEBUG_PROBE_NOTES.md`** — the `--no-verify`-only-with-approval rule lives there; Rule 5 references it as one of the three Rule-5 defenses.
- **Consolidates the gate-classification framing from `docs/plans/STAGE_P7_15_SHELVED_2026-04-11.md`** — the "verification gates claimed but not performed" history is now codified as a normative rule.
- **Does NOT modify `docs/CONFIG_TEST_MATRIX.md`** — the matrix specifies *which* changes trigger HW gates; this doc specifies *what* a HW gate must observe to pass and *what* discipline keeps the gate from being silently skipped.
- **Does NOT modify the pre-commit hook itself** — the hook's mechanical triggers stay the same. This doc names the hook as the primary Rule-5 defense.

---

## When this rule does NOT apply

- **Pure-software changes** (host tests, doc-only, comment-only, build-system reformatting that doesn't produce a different compiled artifact). The pre-commit hook's `pre_commit_matrix.py` already classifies these and skips HW gates entirely.
- **Off-line analysis** (replay-harness runs against logged data, post-flight log decoders, math-only refactors verified by host ctest). No live hardware, no HW gate.
- **Emergency hotfixes with explicit `git commit --no-verify` and repo-owner approval** — the bypass discipline in `.claude/DEBUG_PROBE_NOTES.md` already covers this. Such commits should still cite *what* was verified, even if not by the standard gate.

---

## Open follow-ups (to be tracked outside this doc)

The following are deliberately not in this rule because they're tooling work, not standard:

1. **Pre-commit hook enforcement of "commit message cites control signal"** — possible regex check on the staged commit message body. Out of scope for this rule; logged as a candidate for `scripts/hooks/pre-commit` Tier 6c.
2. **Per-gate positive-control catalog** — the table in Rule 1 should grow as new gates are added. Catalog can live in `docs/CONFIG_TEST_MATRIX.md` (Tier 7?) or as a structured appendix to this doc.
3. **Automated 3-boot reseat verification** — the probe can in principle reset the chip 3× while the operator reseats cables, with the script comparing positive-control signals across boots. Possible future enhancement.
