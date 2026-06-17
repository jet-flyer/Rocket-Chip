# Session Checklist

**Purpose:** Prevent incomplete handoffs, lost work, and undocumented changes between agent sessions.

**How to use this checklist (aviation phase model):**

This document contains dedicated checklists for different phases of a session, modeled on aviation procedures (e.g. Before Start, Taxi, Takeoff, Approach, Landing, Short-Field Landing).

- Always start at the **top of the relevant phase section** for the actual phase you are in.
- Do not run checklists for phases you are not currently executing.
- Outer phases inherit the procedures and requirements from inner phases (every landing includes an approach; every short-field landing includes a normal landing).

The scopes are nested as:

```
Per Commit  ⊂  Per Push  ⊂  Session End  ⊂  Milestone Close
```

- **Per Commit** and **Per Push** are baseline procedures. They are inherited by any later phase that involves commits or pushes.
- **Normal session close**: Begin at the `## Session End` header. Satisfy the inherited Per Commit + Per Push rules for the work being closed, then complete the Session End items (9–12).
- **Milestone / stage close**: Begin at the `## Session End` header for the normal close items, then continue with the additional milestone-only items under `## Session End: Milestone` (13+). Not every session end is a milestone.

### Phase Entry Points (where to begin)

- **Starting or resuming a session** → Begin at `## Session Start`
- **Making a commit** → Begin at `## Per Commit` (for that commit)
- **Pushing** → Begin at `## Per Push` (includes all Per Commit rules)
- **Normal session close / handoff** → Begin at `## Session End`
- **Stage / milestone close** → Begin at `## Session End`, then continue at `## Session End: Milestone`

A separate **Trigger-Driven Documentation Edits** section captures rules that don't follow the inheritance pattern — they fire when a session's content directly contradicts a doc, regardless of which scope-event is happening at the time.

---

## Session Start

1. Read all files referenced in `CLAUDE.md` (or equivalent agent instructions)
2. Check `PROJECT_STATUS.md` for current phase and blockers
3. Check `CHANGELOG.md` for recent changes
4. Check `AGENT_WHITEBOARD.md` for flags from previous sessions
5. If resuming interrupted work: read whiteboard notes and verify repo state matches expectations
6. **If this session will touch flight-critical paths** (`src/flight_director/`, `src/active_objects/ao_flight_director.cpp`, `src/active_objects/ao_logger.cpp`, `src/cli/rc_os.cpp`, or firmware `[FD]` log messages), run the bench sim canary before making any changes:

       python scripts/bench_sim.py

   If the canary fails on unchanged main, the tool has rotted — fix the tool first, not the firmware. See LESSONS_LEARNED.md Entry 36. The pre-commit hook runs the same test automatically when the probe is available and the staged diff touches these paths; the session-start canary catches rot introduced by previous sessions that committed via `--no-verify` when the probe was not available.

---

## During Session

- **No arbitrary numbers.** Any numerical value (sample rates, buffer sizes, thresholds, timeouts) must be justified by a source (datasheet, SDK docs, reference implementation, forum post confirming it works). If no source exists, flag it and ask before proceeding.
- **No assumptions on unspecified decisions.** If a task or design choice isn't covered by existing docs and internet research doesn't definitively answer it, ask before implementing. "Definitively answered" means confirmed working in a credible source, not a speculative forum post.
- **Research before implementing.** Before writing code that touches hardware interfaces or sensor drivers, check relevant documentation, datasheets, and especially recent forum posts for guidelines and known issues.
- **Log as you go.** Don't batch all documentation to the end. If you discover something unexpected, note it immediately.

---

## Per Commit

These run on **every** `git commit`. Inherited by PUSH, SESSION_END, and MILESTONE.

1. **Verify build compiles clean.** For the relevant tier(s) given the staged paths. The pre-commit hook gates host ctest automatically; target builds (`cmake --build build/`) are the agent's responsibility for changes that affect them.
2. **No orphaned work.** Every file change being committed should be intentional. No half-finished edits left in the working tree from prior tool calls.
3. **No unintended deletions.** Run `git diff --stat` (or `git status --short`) and verify the diff matches what the commit is claiming to do. If files are being deleted, confirm that was intentional.
4. **Commit message.** Format: `[agent] brief description of what and why`. If a HW gate ran for this commit (bench_sim, ctest, soak, post-flash banner read), the message must cite the **observed positive-control signal**, not just the gate name. See `standards/HW_GATE_DISCIPLINE.md` Rule 3. Pure-software changes are exempt — say so explicitly in the commit message ("Verified: pure-software change, host ctest 788/788, no HW reseat required").

4a. **Hardware gate readiness.** When the pre-commit hook indicates that hardware verification is required (e.g. “HW VERIFICATION NEEDED” or “OpenOCD not on localhost:3333”), start OpenOCD using the commands in `docs/agents/DEBUG_PROBE_NOTES.md` (or `scripts/start_openocd_pico_sdk.ps1`) before proceeding with the commit. `--no-verify` may only be used with explicit repo-owner approval in the current session.
5. **Trigger-driven doc edits ride with the trigger** (see "Trigger-Driven Documentation Edits" section below). If this commit changed a fact a state-of-system protected doc states, the doc edit goes in the same commit, not a separate one.

5a. **Change Impact Analysis for non-trivial changes.** Before staging a commit that touches a flight-critical path, a state-of-system protected doc, or any mechanism that other code depends on, name the dependency footprint:
   - What files or modules does the change touch?
   - What other code (or open Problem Reports in `docs/PROBLEM_REPORTS.md`) depends on what the change is modifying?
   - Which audit gates, pre-commit checks, or scripted tests does the change interact with?

   For trivial changes (typo fixes, comment-only edits, isolated single-file additions with no callers yet), this is implicit and doesn't need to be written down. For non-trivial changes, capture the answers in either (a) the commit message body, (b) a `docs/PROBLEM_REPORTS.md` row if the change closes a tracked PR, or (c) a scratch note in the dated audit report if the change is part of an audit-driven remediation cycle. Source: DO-178C change-impact-analysis principle. Catches the failure mode where a fix to one file silently invalidates an assumption another file relied on. Inserted as 5a (not 6) to preserve existing cross-references to items 6-17 in other docs.

   **Symbol-removal sub-rule.** When the commit removes a function, file, or symbol (not adds — adds are caught by callers compiling against the new symbol), the impact analysis MUST also include: (a) **callees** — grep for functions that were called only by the removed symbol and are now potentially dead (one-line grep per callee; if the callee is `static`, the compiler's `-Wunused-function` will catch it on next build); (b) **protected-doc references** — the existing trigger-driven edit rule already covers state-of-system docs that mention the removed symbol by name. The commit message body cites both walks. Per `standards/CODING_STANDARDS.md` "Scratch Tools and Dead-Code Discipline" (project policy 2026-05-22, four-persona council). Origin: R-5 Unit D's 2026-05-16 discovery of `src/telemetry/telemetry_service.cpp` orphaned for ~6 weeks because no prior commit walked for orphaned callees when its caller was migrated.

---

## Per Push

Inherits all Per Commit rules. Adds the rules below — these run **once** before `git push`, regardless of how many commits accumulated since the last push.

6. **Station/vehicle build parity check.** If any commit being pushed modified `CMakeLists.txt`, `src/active_objects/`, `src/cli/`, `src/telemetry/`, or `src/drivers/rfm95w.cpp`, rebuild BOTH tiers for BOTH roles to confirm no build is broken:
   ```
   cmake --build build_flight/         # vehicle flight (single tier post-R-25-exec 2026-05-13)
   cmake --build build_station_flight/ # station flight (ROCKETCHIP_JOB_STATION=1)
   ```
   Station and vehicle share the same source tree gated by `ROCKETCHIP_JOB_STATION` / `kRadioModeRx` — a change that compiles on one role can silently break the other. If any build directory doesn't exist yet, create it with `cmake -B build_flight ..` (or `cmake -B build_station_flight -DROCKETCHIP_JOB_STATION=1 ..`). When hardware-verifying, both the vehicle Feather and the Fruit Jam station should exercise the changed path before the push. (Single flight binary per role with runtime test-mode gating via `rc::test_mode_active()`; see `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.)

7. **Triggered-doc edits are committed, not WIP.** Every CHANGELOG entry, WB row change, PROJECT_STATUS edit, or other trigger-driven doc edit that this push window produced must be in a committed state — not left as unstaged or staged-but-uncommitted edits. (The principle is "the diff and the doc are atomically consistent in git history" — uncommitted edits break that.)

8. **CHANGELOG entry/entries cover the push window.** Each significant unit of work being pushed has a CHANGELOG entry. Routine work commits between meaningful documented changes are fine without their own entry. The frequency rule is in CHANGELOG.md's own header — typically one entry per session, sometimes more if multiple significant units landed.

---

## Session End

**Dedicated checklist entry point for normal session closure.** Begin here when ending a session.

This phase always requires satisfying the inherited Per Commit and Per Push rules for any commits/pushes that are part of this close, then performing the items below.

These rules run **once** when the session is closing (in addition to the inherited baseline procedures).

9. **No broken code on main.** If any work in this session is incomplete, either stash it, abandon it, or commit it to a feature branch — never leave broken code on main.

**Session cleanup note:** At the end of sessions with heavy WSL or debug probe use, terminate lingering processes (e.g. `wsl --terminate Ubuntu-24.04` + kill OpenOCD). Not a formal gate.

10. **Note the exact state.** "Build compiles, sensor reads work, CLI untested" is better than "made progress" in any handoff note.

11. **Handoff notes (if applicable).** If the user has explicitly signaled that the session is being handed off (for example by saying “session handoff”, “continue tomorrow”, or similar), expand `AGENT_WHITEBOARD.md` with:
    - What was in progress
    - What's blocked and why
    - Any concerns or open questions
    - Specific files that were being worked on

12. **Push to remote** — `git push` so work is not stranded locally.

---

## Session End: Milestone

**Additional items for stage / milestone closes only.** This is the "short-field" version of a session end.

Begin the milestone-specific items here (after completing the normal `## Session End` items above).

Every milestone close includes a full normal Session End + these extra requirements. These rules run **only** when a stage closes (in addition to all earlier rules including a normal Session End).

13. **Update `docs/PROJECT_STATUS.md`** with milestone completion and next phase (per the trigger map — phase change is a state-of-system trigger).

14. **Architecture-doc drift check against protected docs.** When a stage closes, grep the state-of-system protected architecture docs (`docs/SAD.md`, `docs/SCAFFOLDING.md`, `docs/AO_ARCHITECTURE.md`) for symbols and module paths that this stage deleted, renamed, or re-homed. The Per-Commit trigger-driven edit rule should have kept these current, but **rot detection** at milestone close is the safety net for cases where prior sessions didn't apply the principle. Pattern: `grep -n "<deleted-symbol>|<old-module-path>" docs/SAD.md docs/SCAFFOLDING.md docs/AO_ARCHITECTURE.md`. This is the legitimate retroactive-fix point — if drift is caught here, fixing it as part of stage close is in-scope. Added 2026-04-22 after the watchdog deprecation left `kWatchdogSentinel` + the `watchdog/` directory tree entry in both protected docs.

15. **Build-system audit.** Run the checks in `docs/BUILD_SYSTEM_AUDIT.md` (P1-A through P6 — dev-tool gating, self-flagged dead code, `ROCKETCHIP_SOURCES` coverage, host/target split, vendor SYSTEM classification, IVP-era scaffolding comments, toolchain versions). Added 2026-04-23 after `ud_benchmark` went undetectedly broken for 2+ weeks and 14 files drifted outside the pedantic-warning gate. Companion to item 14 — both are content-level drift checks on protected or audit-gated infrastructure.

16. **Consider `LESSONS_LEARNED.md`.** If significant debugging occurred during the stage, document it. This is trigger-driven (significant content) not cadence-driven, so most stage closes won't add an entry; some will.

17. **Full-tree clang-tidy sweep.** The pre-commit hook only gates staged files, so latent JSF AV Rule 1 violations (function size > 60 lines) can accumulate in files that aren't touched stage-by-stage. At milestone/stage close, run the same checks the hook runs against EVERY `src/**/*.cpp` not on the exemption list (exemptions: `src/cli/**`, `eskf_codegen.cpp`). (`src/dev/**` no longer exists post-R-25-exec 2026-05-13.) Zero warnings required for milestone closure — any new violations must be decomposed or logged as an accepted deviation in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` before the stage can close. Pattern (Git Bash, adjust toolchain path if needed):

        for f in $(git ls-files 'src/*.cpp' | grep -v 'src/cli/' | grep -v eskf_codegen.cpp); do
          "C:/Program Files/LLVM/bin/clang-tidy.exe" "$f" -p build/ \
            --checks="-*,readability-function-size,readability-function-cognitive-complexity" \
            --extra-arg="--sysroot=C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi" \
            --extra-arg="--target=armv8m.main-none-eabi" \
            --extra-arg="-isystem" \
            --extra-arg="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi/include/c++/14.2.1" \
            --extra-arg="-isystem" \
            --extra-arg="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi/include/c++/14.2.1/arm-none-eabi/thumb/v8-m.main+fp/softfp" \
            --extra-arg="-Wno-format-security" \
            --extra-arg="-Wno-gnu-zero-variadic-macro-arguments" 2>&1 \
            | grep -E "warning:.*readability-function-(size|cognitive-complexity)"
        done

   Added 2026-04-18 after Stage L surfaced 5 latent violations (ao_logger, ao_telemetry, core1_sensor_loop, guard_evaluator_tick, flight_director_evaluate_guards) that had accumulated silently across earlier stages. See LL Entry 36 discipline — "gates that only check incremental change cannot catch pre-existing rot."

17a. **Scratch-tool stale review.** Walk `tools/scratch/` for any directory whose most recent commit is older than 4 months. For each: confirm `_PURPOSE` file present and current, decide keep-or-delete in one line. The flag means "review," not "auto-delete" — a scratch tool that's still occasionally useful stays with a noted bump in date. Per `standards/CODING_STANDARDS.md` "Scratch Tools and Dead-Code Discipline" (project policy 2026-05-22, four-persona council). Pattern (Git Bash):

        for d in tools/scratch/*/; do
          last=$(git log -1 --format="%ai" -- "$d" 2>/dev/null | cut -d' ' -f1)
          [ -z "$last" ] && continue
          if [ "$(date -d "$last + 4 months" +%s)" -lt "$(date +%s)" ]; then
            echo "STALE: $d (last touched $last)"
            cat "$d/_PURPOSE" 2>/dev/null || echo "  (no _PURPOSE file)"
          fi
        done

   ArduPilot panelist's point — calendar items don't run, so this must be a checklist item that actively surfaces stale dirs each milestone close. Cheap (~30 sec to run); decisions are 1-line "keep" or "delete" per row.

17b. **Dead-code inventory.** Run `python scripts/audit/find_dead_code.py --output docs/audits/DEAD_CODE_INVENTORY_<date>.md` and walk the findings. Catches: unbuilt orphans, DCE candidates, stale build-cache .obj files, and source-list drift between `add_executable(rocketchip)` and `ROCKETCHIP_SOURCES`. Companion script: `python scripts/audit/survey_test_subjects.py` for host-test orphan-subjects (the bench-tier deletion class — tests for code that doesn't ship). High false-positive rate by design (per ArduPilot panelist's false-positive-asymmetry framing); hand-walk each finding. Added 2026-05-22 per the dead-code policy in `standards/CODING_STANDARDS.md`.

17c. **Dual-toolchain exercise (WSL pivot).** At milestone close, confirm that both the WSL Linux-FS path and the native Windows path have seen at least one full host ctest run + one hardware gate (`bench_sim.py` or `station_bench_sim.py`) since the previous milestone. If the Windows path has not been exercised, explicitly note it and decide whether to force a Windows-native run before closing the stage. This prevents silent rot of the fallback under the "try until it breaks" policy (see the WSL steady-state decision doc). Added 2026-05-28 per council review of Phase 6.

---

## Trigger-Driven Documentation Edits

**Principle:** A protected/architecture document is edited only when the session's work directly contradicts its current contents. Not on cadence. Not on stage transitions. Not "at session end" by default. Edit when — and only when — the diff has made the doc wrong.

This is the inverse of "remember to update SAD at the end" which is the pattern that produced the watchdog-deprecation drift incident (2026-04-22) and the `kWatchdogSentinel` staleness in two protected docs. "Remember at the end" relies on memory; "edit only when content-affected" relies on diff-checking, which is mechanical.

**The edit rides with the trigger.** When a code or content change makes a state-of-system doc wrong, the doc edit goes in the **same commit** as the change that triggered it. That way `git log --follow` on the doc shows the same SHA as the code that changed it, and `git bisect` doesn't land on a "code drifted from doc" intermediate state.

### Two categories of protected docs

Protected docs come in two kinds with different edit rules.

**State-of-system docs** — content claims "this is how the system is currently structured." If code changes and the doc still says the old thing, the doc is now wrong. Forward-going edit applies (same commit as the trigger).

**Historical-record docs** — content describes what was decided / planned / audited / completed at a moment in time. New code doesn't make historical statements wrong; it just makes them history. Forward-going edit does **not** apply — write a new entry / new decision doc / supersession note instead. Editing the historical content directly is a special case (typo, factual error, deliberate supersession header marking).

### Per-doc trigger map

#### State-of-system docs (forward-going edit applies)

| Doc | When the edit is forced |
|---|---|
| `docs/SAD.md` | Architecture-level facts change: AO added/removed/priorities, signal added/renamed/rerouted, queue depths, fault-handling chain, code classification, build-system identity. Implementation refactors that don't change the architecture statement don't count. |
| `docs/SCAFFOLDING.md` | Directory structure changes (file/folder added/deleted/renamed/moved). Pure file-content edits don't trigger an edit here. |
| `docs/AO_ARCHITECTURE.md` | The AO inventory or its facts (rate, priority, queue depth, sub/pub list) change. Internal AO logic refactors don't count. |
| `docs/MULTICORE_RULES.md` | A multi-core rule changes (new lesson learned, new SDK behavior, new fault pattern). Code that *applies* an existing rule doesn't trigger an update. |
| `standards/CODING_STANDARDS.md` | A standard or classification changes. New file added to the file-classification table. New deviation added/resolved (with back-reference to `ACCEPTED_STANDARDS_DEVIATIONS.md`). |
| `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` (Active section only) | New active deviation added (post user-acceptance). Existing deviation's severity / scope / location changes. (Resolved section is historical-record — see below.) |
| `standards/RP2350_ERRATA.md` | New erratum encountered. Existing erratum's status (workaround applied / gap / verified) changes. |
| `standards/HW_GATE_DISCIPLINE.md` | A rule changes. New gate type covered. |
| `COUNCIL_PROCESS.md` | Council protocol changes (new persona, retired persona, process change). |
| `.claude/CLAUDE.md` | Auto-load index changes (doc added/removed from intake). |
| `standards/AK_GUIDELINES.md` | Behavioral guideline changes. |
| `docs/agents/PROTECTED_FILES.md` | Protection list changes (file added or removed from protection). |
| `docs/agents/SESSION_CHECKLIST.md` | A checklist item changes, the structure changes, or the trigger map gains/loses a doc. |
| `standards/AUDIT_GUIDANCE.md` | Master audit strategy, scope table, or file-location policy changes. |
| `README.md` | "Read First" / "Each Session" lists change. Key rules change. |
| `docs/PROJECT_STATUS.md` | Phase changes, new blocker discovered, blocker resolved, next-action changes. **Not** triggered by routine work commits within a stage. |
| `AGENT_WHITEBOARD.md` | Active state changes — new flag/issue surfaced, row resolved (erase per the IRL-whiteboard rule), or row's status materially changes. **Not** triggered by routine work. |
| `tools/spin/*.pml` (existing models) | Firmware behavior matching a candidate SPIN extension lands — see `AGENT_WHITEBOARD.md` "Station SPIN model extensions" (multi-pending-in-flight, RadioScheduler TX-window arbitration, MAVLink parser state, `station_idle_tick` GPS poll interleave) for the in-flight list. When such firmware lands, the `.pml` edit rides in the same commit; the master gate (`run_stage_o_ao_spin.sh`) auto-discovers LTL properties so new claims pick up automatically. Codifies a discipline that would otherwise rely on memory. R-13 (2026-05-07 audit). |

#### Historical-record docs (forward-going edit does NOT apply)

| Doc | Edit rule |
|---|---|
| `docs/decisions/*.md` | Each decision doc is frozen on commit. If a decision is later superseded, write a **new decision doc** that supersedes the old one. Edits to existing decision docs are typo-correction or supersession-header-only. |
| `docs/plans/*.md` | Per-stage plans freeze on commit. Once a stage executes, the plan is historical even if execution diverged from it. Edits are typo-correction only. |
| `docs/audits/*.md` | Each audit is dated and frozen. New audits are new files. |
| `docs/baselines/*` | Each baseline is a frozen snapshot. New baselines are new directories/files. |
| `CHANGELOG.md` | Append-only historical event log. The current entry being drafted in this commit is state-of-system until it ships; after commit, frozen. |
| `docs/agents/LESSONS_LEARNED.md` | Append-only. Each entry is historical. Existing entries get edits only for typo-correction or for explicit supersession headers (e.g., the LL Entry 25 "SUPERSEDED 2026-04-22" header is legitimate because it adds context to the historical record without rewriting it). |
| `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` (Resolved section only) | Historical record once a deviation is marked Resolved. The Active section above is state-of-system. |

#### Mixed-mode docs

| Doc | Rule |
|---|---|
| `docs/IVP.md` | Two trigger modes: (a) **Planning entry** — when a stage is being planned, the first step of the planning session amends `IVP.md` with per-IVP entries in the project's IVP format. (b) **Closing entry** — if significant changes from the planned scope occurred during execution, amend after all the work has landed. The stage-list and "next stage" sections are state-of-system; the per-IVP entries are historical-record once the IVP ships. |

### How to apply

Before each commit, ask:
- Did the staged diff change any fact stated in a **state-of-system** protected doc?
  - If yes → update the doc as part of THIS commit.
  - If no → leave it alone.
- Did the work create something new that warrants a **historical-record** entry (CHANGELOG entry, LESSONS_LEARNED entry, decision doc, audit doc)?
  - If yes → write the new entry as part of THIS commit (or a focused commit dedicated to the documentation, depending on scope).
  - If no → leave the historical record untouched.

### Retroactive amendment

If drift in a state-of-system protected doc is **caught later** — by a future agent, by a milestone audit, or by the user — a focused retroactive amendment is legitimate **with explicit user approval**. Without approval, the default is: flag the drift on `AGENT_WHITEBOARD.md` and continue. Don't silently fix protected-doc drift outside the trigger event.

The reason for the approval gate is that retroactive fixes look identical to "decided to start editing protected files because I felt like it" from a diff perspective. Requiring approval keeps the scope honest.

The Milestone-only Architecture-doc drift check (item 14) is a structurally-authorized retroactive-fix moment — drift caught during that check is in-scope to fix as part of the stage close, without per-edit approval, because the milestone authorization covers it.

---

## Red Flags (Stop and Ask)

- You're about to delete files you didn't create
- A build that was working now fails after your changes
- You're changing more files than the task requires
- You're picking numerical values without a source
- You're "fixing" something adjacent to the actual task
- You notice a conflict between documentation and code — flag it, don't silently resolve it
- The task would require changes to a protected file (forward-going trigger-driven edits to **state-of-system** protected docs are an exception — they don't require asking, but historical-record protected docs always do)

---

*This checklist is the minimum standard. Doing less risks losing work or introducing silent regressions.*
