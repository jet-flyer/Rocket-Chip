# Agent Whiteboard

**Purpose:** Cross-context / cross-agent communication channel for **active work only**.

> **Treat this like an IRL whiteboard — not a record of completed things.**
> When an item is done, **erase the row**. Don't add a "Resolved" section,
> don't strike it through, don't leave a "closed" marker. The CHANGELOG is
> the project's permanent record of what was done; this whiteboard's only
> job is to surface what's *still active*. A row's continued presence is
> the signal that it still needs attention. Stale "done" notes dilute that
> signal and bury the rows that actually matter.
>
> Before adding a row: check whether the work is already done elsewhere
> (CHANGELOG, git log, the relevant doc). Before acting on a row: spot-
> check it's still real — agent memory of "this needs doing" is exactly
> the failure mode that drives stale-row accumulation. If you reject an
> item after consideration, log the rejection rationale in CHANGELOG and
> erase the row, don't move it to a "rejected" section.

## Skills to add (OPEN)

Wanted skills — not written yet. Not a license to author them until scheduled.

- **Session-checklist skill** — grant add-only cadence writes (`CHANGELOG.md`, `PROJECT_STATUS.md`) only when that checklist scope is actually running (commit / push / wrap). Stops jumping the gun on a changelog because the list was *read*. If the protected-file hook is revived on Claude Code, this skill drives that gating. (Moved here from the graphify/hook note.)
- **Council skill** — when the user says “council review” / “panel check,” load `COUNCIL_PROCESS.md` (panel, stop conditions, incomplete-review offer) instead of relying on the agent to remember the file.

---

## bench_sim hook vs canary vs A/B recovery (OPEN) (2026-08-19)

**Hook ≠ canary.** Same script (`scripts/bench_sim.py`); different tree, different question. Flesh out in `standards/HW_GATE_DISCIPLINE.md` Rule 5 (still cites dead checklist item 6) + drop/retarget the During Session canary paragraph.

- **Hook (prevention of silent skip / LL 36):** pre-commit on firmware-affecting paths. Runs **after** edits are staged, **before** the commit exists. Needs OpenOCD `:3333`. Does **not** flash — talks to whatever image is already on the chip. Stops “claimed PASS, nobody ran it.”
- **Everyday pre-edit canary (not feasible here):** would need HW on **unchanged** HEAD *before* first edit. Probe is not up at session start; upload is after edits. First-`Read` of `src/` is the wrong moment (no probe; board ≠ HEAD).
- **Recovery (when a skip or hook-fail is ambiguous):** flash last-known / `HEAD~`, `bench_sim`; flash the new build, `bench_sim`. Attribution, not prevention. Do not put a last-run stamp in `PROJECT_STATUS.md` (LL 36-shaped lie unless only the hook writes it).

Owner: hook stays the land gate. A/B is the documented recovery. No boot canary.

---

## `rp400` git remote = Pi 400 keyboard clone (DEFER) (2026-08-20)

Not a radio chip and not WSL. Git remote `rp400` (`npow@192.168.1.233:~/Rocket-Chip.git`) is an early clone onto the **Raspberry Pi 400** keyboard computer (CYBERDECK HAT/Bonnet on hand — `docs/hardware/HARDWARE.md` Ground Station). Host was off/unreachable 2026-08-20. Local tracking of `claude/tender-banach` was dropped; that branch may still exist on the Pi (Feb 2026 SAD/ESKF, already an ancestor of `main`). **Do not chase it now.** Next time that machine is used — likely Stage 12B Yamcs / OpenMCT / advanced GCS — if the clone has not been fully redone, delete leftover branches there (at least `claude/tender-banach`). CHANGELOG `2026-08-20-004` is the land-time note.

---

## Project status (one-line snapshot)

**Stages 1-14 + 16A + 16B + 16C + L + T COMPLETE.** **L2-P5 itinerary 121/121 + owner WN-001–327; walk WB closed 2026-08-17.** Independent walks on main: Claude `CW-` (`L2P5_CLAUDE_WALK_FINDINGS.md`) and Grok **GWF-001–498** (`L2P5_GROK_WALK_FINDINGS.md`). Next: disposition plan. W-5/W-2 include/consumer + concurrency 3-question still OPEN (not the Grok/Claude semantic walks). Host ctest / SPIN: see last green gate. Tracking: `docs/AO_ARCHITECTURE.md`. **Stage 17** plan: `docs/plans/STAGE17_TAPERED_BUILDUP.md`. **CCSDS TC + COP-1 deferred post–Stage-17.**

---

## Agent re-walk checks (OPEN) (landed from walk WB W-5 + W-2, 2026-08-17)

**Venue:** planned agent re-walks (owner walk did not run these systematically). **Not** a new temp doc. Frozen walk guide is not the home.

**When questioning whether a header earns its own file** (sparseness, thin façade, parallel packs): attach, in the discussion or WN,

1. **Direct includes** of that path under `src/` + `include/` (+ `test/` if relevant)
2. **Symbol consumers** (may be fewer than includes; dead symbols happen)
3. **Indirect fan-in** (selector-only packs, façade-only types)

Do **not** couple later keep/fold decisions across “same class” thin headers. Owner walk applied this only a few times early (`radio_config_table`, `sensor_snapshot`); later “earns rent?” notes did not.

**Concurrency 3-question pass** — for each object below, name owner / mutator / barrier. Ambiguity is the finding. Enumerate from `volatile` / `std::atomic` / `multicore_*` / spinlock — **not** every `g_` (QP `l_`→`g_` rename made that hint useless).

*22 `volatile`:* `log/rc_log.cpp` `g_ring`/`g_head`/`g_tail`/`g_droppedBytes`/`g_highWater`; `drivers/gps_uart.cpp` `g_rxHead`/`g_rxTail`/`g_rxOverflow`; `safety/test_mode.cpp` `g_test_mode_arm_magic`/`g_test_mode_enabled`/`g_magicObservedAtBoot`; `cli/rc_os_commands.cpp` `g_t2_pending`/`g_t2_cmd`/`g_t2_p1`; `safety/fault_inject.cpp` `g_fault_core0_stall`/`g_fault_watchdog_skip`; `safety/station_fault_inject.cpp` `g_fault_station_rx_drop_remaining`/`g_fault_station_ack_suppress_remaining`; `safety/fault_protection.cpp` `g_inFaultHandler`; `safety/flight_in_progress.cpp` `g_flightInProgressMagic`; `safety/pyro_edge_logger.cpp` `g_count`; `flight_director/flight_director.cpp` `g_phaseObservablePair`.

*9 `std::atomic`:* `shared_state.cpp` `g_startSensorPhase`/`g_sensorPhaseDone`/`g_calReloadPending`/`g_core1PauseI2C`/`g_core1I2CPaused`/`g_core1LockoutReady`; `core1/sensor_core1.cpp` `g_bestGpsValid`; `cli/rc_os.cpp` `rc_os_mag_cal_active`; `drivers/spi_bus.cpp` `g_spi_error_count`.

Same test, not in the count: seqlock snapshot, PSRAM ring, AO static events.

---

## P10-9 live function pointers (OPEN) (landed from walk WB W-1, 2026-08-17)

**This is a disposition item.** It should have been **one project-wide WN** (same later decision: unlogged P10-9 sites). It was not filed: triage says Property C is Det-by-reference via the deviation log, and the log reads “all resolved 2026-05-13,” so the walk procedure would PASS without looking. `lm_solver` was explicitly not re-opened. Findings stay frozen — this row is the working list.

**Do not treat the empty deviation log as evidence of absence.** FP-1 (templates in `lm_solver`) really was fixed. These 18 declaration sites (7 files) are still live (spot-checked 2026-08-17):

- Typedef’d: `safety/test_mode.h` `FlightPhaseAccessor`; `fusion/eskf_runner.h` `EskfEventLogFn`; `cli/rc_os.h` `rc_os_read_accel_fn` / `rc_os_read_mag_fn` / `rc_os_reset_mag_staleness_fn`
- Raw: `shared_state` GPS hooks `g_gpsFnUpdate`/`g_gpsFnGetData`/`g_gpsFnHasFix`; `flight_director.h` 5 FD action callbacks; `action_executor.h` `set_led`/`log_pyro`; `logging/flash_flush.*` `void (*kick_watchdog)()` ×3

**When disposing:** accept (sign-off), fix (templates / other), or defer. FD + `action_executor` callbacks may wait on the existing **QP/C vs QP/C++** eval (they exist because the HSM is C). GPS hooks and `kick_watchdog` do **not** wait on QP. If P10-9 is accepted rather than banned, JSF-176 un-moots on the 13 raw decls — a second, separate question. Sweep was `src/`+`include/` regex only.

---

## Comment / Doxygen work — order (OPEN) (landed from walk WB W-10; W-6 folded into WNs)

**Do not start mass comment cleanup or the density/Doxygen policy edit until the inventory exists.** Already agreed; W-10 was the inventory so WN-081 would not grow a file list.

1. **Inventory** production headers/sources with Doxygen markup (`@file`/`@brief`/`@param`/`@return` / `/** … */` API blocks). Grep; include/exclude rules at the time (public API vs all `src/`). Seeds: `gps_pa1010d.h`, `i2c_bus.h`, `icm20948.h`, `baro_dps310.h`, `rfm95w.h`.
2. **Then** policy: header density exemption (**WN-054**) + Doxygen keep-consistently-or-drop (**WN-081**).
3. **Then** process-archaeology / “dev comment” cleanup — **WN-085** and the per-file comment WNs (W-6/W-16 were only the theme pointer; 79 WNs already cite W-6). Keep live invariants; IVP/Stage/session essays belong in docs.

---

## HW-agnostic rule — write it before HW-leakage WNs (OPEN) (landed from walk WB W-8)

**No product fork.** Stance is already apparent: domain code HW-agnostic within the compatible ARM class; HAL/board/job packs at the edge. What is missing is a **written rule** (CODING_STANDARDS and/or SAD — both protected; edit only when that work is scheduled).

**Write the rule first**, then dispose HW-reference / leakage notes (**WN-063**, **WN-068**, **WN-309**, and others in that family). Do not close those WNs against an unwritten standard. Spell out what must live in board/job packs vs domain headers, what “compatible ARM” means, and what is forbidden.

---

## Safety/ops criticality inventory (OPEN) (landed from walk WB W-15)

Optional project-wide map of **things the system does** (Go/No-Go, launch abort, pyro intent, confidence gate, ESKF healthy, FD HSM, …) → owning files/APIs. Review priority / gate scope / doc SSOT. Not a C++ or build tier.

**WN tie is weak.** **WN-182**’s real claim is Go/No-Go SSOT; the inventory is an explicit owner tangent. **WN-184** is a load-bearing comment-vs-type contract and only points at W-15 as safety-adjacent. **Do not block** disposing those WNs on creating this list. Seeds if/when built: WN-182, WN-142, WN-172, WN-176, ESKF brake, fault recovery.

---


## Local-LLM try-later shortlist (OPEN)

Not adopted. Detail: `docs/tools/LOCAL_LLM_COMPANION_RESEARCH.md` §5. WSL `.wslconfig` is **48GB** (Cookbook ~47 GB after refresh).

Cookbook scan-row Download for Devstral hits **official** `mistralai/Devstral-Small-2-24B-Instruct-2512` (no GGUF). Use Direct Download: `unsloth/Devstral-Small-2-24B-Instruct-2512-GGUF`.

- **Devstral Small 2 24B** — owner trying **Q8_0** first (`…-Q8_0.gguf`, ~23 GB). Q4 later for A/B. Do not pull the whole Unsloth repo.
- Qwen3.6-35B-A3B
- Gemma 4 31B QAT-Q4_0 (`google/gemma-4-31B-it-qat-q4_0-gguf`, not Cookbook’s Q4_K_M row)
- Nemotron 3.5 Lightning 30B

---

## Early-impl / rework-eval candidates (OPEN) (grouped 2026-08-05)

**What this is:** One place that lists systems which **work today** but were chosen early
(or sit on early HW/paths) and deserve a **deliberate re-evaluation** — not fire drills,
not mid-walk rewrites. Outcome of each eval is keep-with-written-*why*, or planned rework
(plan ± council before code).

**Rule:** Do **not** half-refactor live flight paths for “cleanup.” Detail lives in the
named sections / WNs / Research rows below — this header is the **index + grouping**, not
a replacement. If the set grows unwieldy, break out a dedicated doc later; **not now**.

**Shared constraint — PIO budget:** several candidates prefer or require PIO. Budget is
shared (watchdog + backup timers already on PIO2; beacon candidate wants PIO0/1). Eval
order for PIO-touching items should weigh **advantages vs remaining SM/instruction budget**
together, not in isolation.

| Candidate | Prefer / lean | Full detail |
|-----------|---------------|-------------|
| **I²C bus backend** | **Prefer PIO master** if advantages hold and **PIO budget** allows; keep thin `i2c_bus_*` façade either way. Flipper `i2c_master_pio` = working RP2350 prior art (license check before import). Driver residual / Fruit Jam GPS cold-boot is a trigger, not a mandate to switch tomorrow. | Research row *I²C bus backend rework-eval* below; `src/drivers/i2c_bus.*`; LL-28/41 |
| **Fault beacon (last-gasp)** | PIO beacon is the architecturally preferred path; eval with SPI last-gasp stop-gap in one session | Research row *PIO beacon + SPI last-gasp* below |
| **RC_OS / CLI “pseudo-OS”** | Structure as proper UX/OS layer (table-driven dispatch, ownership) | **§ RC_OS Rework** below |
| **Quaternion convention** | Re-check Hamilton vs alternatives — not keep only to avoid churn | **§ Quaternion convention re-eval** below |
| **Sensor seqlock (Stage 3)** | Still right path for Core0↔Core1 snapshot? | L2-P5 **WN-042** (`sensor_seqlock.h`) |
| **PCM onboard logging** | Still right shape vs Starcom/air vs Stage-17 log tier? | L2-P5 **WN-059** (`pcm_frame` + log path) |
| **Flash layout map** | Early feature; re-eval after multi-board / storage evolution (low priority) | L2-P5 **WN-062** (`flash_layout.h`) |
| **Radio / telem surfaces** | Many are **Starcom-gated** supersession candidates (not pure “early code smell”) | **WN-041**, **WN-046**, **WN-097** (RFM95W defer), Starcom / CCSDS rework rows |
| **CCSDS TC + COP-1** | Command reliability layer rework — deferred post–Stage-17 (council) | Project status line; high-priority deferred radio items |

**Not in this group:** pure process/tooling OPEN items (graphify re-pass, commit hygiene,
worktrees, IEEE 1028 mapping), accepted Gemini-tier PIO gaps, or active L2-P5 walk handoff.

**Add a member:** append a row here + keep/expand the detailed section or WN; do not scatter
new “maybe rework someday” bullets without listing them in this table.

---

## Regulatory / RF compliance safeguards (OPEN) (2026-08-05)

**Origin:** L2-P5 walk on `rfm95w` — casual `// ISM band (US, FCC Part 15)` + default
915 MHz / high TX power without a project compliance SSOT or risk-line warnings
(**WN-100**).

**Intent:** Project-wide discipline for knobs that can put firmware **out of
legal operation** if mis-set or bypassed (frequency, power, band, duty where
applicable): accurate scoped statements or explicit non-guarantee + **doc SSOT**;
warnings at the dangerous call sites/defaults; later audit sweep (radios first).

**Starcom special caveat:** When Starcom defines RF/PHY/config surfaces, it must
adopt the **same safeguards** (or stricter) — do not reintroduce silent “looks
legal” defaults without policy. Coordinate with RFM95W deferral (**WN-097**).

**Rule:** Not legal advice from agents; owner/legal review for any public
compliance claims. No mid-walk mass comment campaign.

**Refs:** L2-P5 **WN-100**, `src/drivers/rfm95w.*`, Starcom docs under `starcom/`.

---

## RC_OS Rework (OPEN) (2026-07-09, from CODE_TRIMMING §2)

**Group:** Early-impl / rework-eval candidates (see index above).

**Origin:** 2026-07-03 code-trimming / staleness survey noted CLI “morphed almost into a pseudo-OS” (`docs/audits/CODE_TRIMMING_AUDIT_2026-07-03.md` §2). Not a scheduled Stage/IVP yet.

**Intent:** Future workstream — structure RC_OS more like a proper UX/OS layer than accretion of single-key handlers: table-driven key→handler maps, clear UX vs domain ownership (AO_RCOS vs cal_manager vs FD), station/vehicle gating without copy-paste, host-testable dispatch. Entry docs: `docs/ROCKETCHIP_OS.md`, `docs/AO_ARCHITECTURE.md`, CODE_TRIMMING §2.

**Rule:** Do **not** half-refactor live menus for LOC first; proven-dead CLI symbols may still be deleted. Needs own plan + council before code.

---

## Quaternion convention re-eval — Hamilton vs alternatives (OPEN) (2026-08-04)

**Group:** Early-impl / rework-eval candidates (see index above).

**Origin:** L2-P5 walk on `math/quat.{cpp,h}` — project ships Hamilton product, scalar-first
`[w,x,y,z]`, body-to-NED (Sola 2017 / IVP Stage 5). Owner: **do not keep the choice only
to avoid churn**; re-check it is still the *right* convention for this product and
stack, not merely the historical one.

**Scope:** Quaternion algebra convention (Hamilton vs JPL product / related layout
choices) and how that decision is documented for implementers. **Not** the separate
UD/Bierman / Joseph ESKF update-path work (MCU numerics — different “JPL” story).

**Intent:** Dedicated re-read (prior art + current fusion/tests) → either reaffirm with
written *why*, or plan a deliberate convention change with full stack impact (math,
ESKF, Mahony, tests, logs). Needs plan before code; no mid-walk flip.

**Refs:** `src/math/quat.h`, IVP quat/ESKF sections, `docs/plans/PHASE5_ESKF_PLAN.md`,
`test/test_quat.cpp`.

---

## Graphify full re-pass (OPEN) — after L2-P5 formal close; owner-gated

L2-P5 walk WB drained and formal walk close **2026-08-17**. Full `/graphify` remains **owner-gated** (billed). Cheap `graphify update` + curate already runs post-commit. A WN cluster index (findings cites, not this graph) is the first disposition-prep step.

---

## Cross-agent commit hygiene — sweep-in convention (OPEN) (2026-06-29, Claude/Opus)

Recurring friction with Claude + Grok + Gemini on shared `main`: separate agents' work getting swept into each other's commits (Grok edited Claude's CHANGELOG entry; an AGENTS.md edit landed in an unexpected commit). **Open ask:** a lightweight convention so agents don't sweep each other's in-flight files — stage explicit paths, never `git add -A` / `git add .` across another agent's working files. Worth a short written rule in AGENTS.md or CROSS_AGENT_REVIEW.md.

**RESOLVED 2026-06-29 (CHANGELOG `2026-06-29-001`) — graphify-out churn no longer dirties the tree.** The post-commit hook rebuilds the graph every commit; the volatile root outputs are now gitignored + untracked (kept on disk, hook keeps them current locally, query/curate/verify unaffected). Protected snapshot subdirs stay tracked. This was the churn that made graph state easy to sweep into unrelated commits.

**Merge-driver note (evaluated, NOT installed):** graphify's README + issue [#1018](https://github.com/safishamsi/graphify/issues/1018) say `graphify hook install` sets up a git merge-driver (`graphify merge-driver %O %A %B` → `networkx.compose` union) so parallel `graph.json` commits union-merge instead of conflicting — but verified across v0.8.50 + upstream main/v8 that the installer does NOT actually wire it (real doc-vs-code gap; needs manual `.gitattributes merge=graphify` + `git config`). With graph.json now gitignored, parallel-conflict risk on it is moot, so the merge-driver is unnecessary unless graph.json is re-tracked. Left uninstalled.

---

## Session-scoped git worktrees for concurrent agents (PROPOSED) (2026-07-01, Claude/Opus)

**Lived trigger:** the L2-P5 manual walk guide (`docs/audits/l2p5_manual_walk/L2P5_MANUAL_WALK_GUIDE.md`) was authored across a long session but left **uncommitted and untracked** on `main`; the graphify tree-churn cleanup (`git clean` — same churn behind the gitignore fixes above) wiped it from the working tree while the repo owner was away. Recovered 2026-07-01 by replaying the session transcript's base Read + 44 Edits (0 misses, signatures + encoding verified), now committed on branch `claude/l2p5-walk-guide-c3757857`.

**Root-cause distinction (the actual lesson):** two separate problems, two separate fixes. **Durability** (don't lose work) is solved *only by committing* — a branch alone would NOT have saved an untracked file from `git clean`. **Isolation** (concurrent agents don't clobber each other) is solved by a dedicated branch — or better, a dedicated *working tree*.

**Proposed policy (evaluate; especially load-bearing for Starcom):**
- **Per-session branch + commit-often.** Substantive sessions work on `claude/<task>-<id>`, committing checkpoints frequently (even WIP), merged at a natural checkpoint. Branch for isolation, commit for durability — both, not either. This is already what `GIT_WORKFLOW.md` prescribes; the gap was execution, not policy. (Note: the line-~42 flag proposing GIT_WORKFLOW *deprecation* is in tension with this and should be reconsidered.)
- **`git worktree` per concurrent agent.** Give Claude / Grok / Gemini / Composer each an isolated working directory (`git worktree add`) sharing one object store. Then one agent's `git clean` / `reset --hard` / branch-switch physically **cannot touch** another agent's tree — the exact failure that just happened, and the structural fix for the "Cross-agent commit hygiene — sweep-in" item above (agents also stop sweeping each other's in-flight files into commits).
- **Starcom specifically:** the standalone CCSDS library is expected to see concurrent multi-agent dev — stand up the worktree model there from the start rather than retrofitting.

**Cost/caveats to weigh:** worktrees add disk + a little setup per agent; graphify's per-tree `graphify-out/` would rebuild per worktree (already gitignored, low risk); any automation that runs `git clean` should be scoped to never delete untracked files it didn't create. Owner decision.

**Owner leans ADOPT (2026-07-04) — applies even to doc-only sessions.** Fresh datapoint: a routine docs-only session hit a push-rejection because Grok pushed to `main` mid-session (clean rebase, but avoidable). **Precision on what fixes what:** a **per-session branch** removes remote-ref contention (the push-collision / rebase dance — this is what bit the doc session); a **worktree** *additionally* prevents local file-clobber (the original guide-wipe). Adopt **both** — branch as the floor (even for docs), worktree when agents run concurrently. Low-friction: the harness supports it natively (`EnterWorktree`/`ExitWorktree` tools + `Agent(isolation:"worktree")`), so the next substantive session should default to a `claude/<task>` branch in its own worktree and merge at a checkpoint.

**Implementation detail (owner idea 2026-07-04) — not done now:** wire this into `docs/agents/SESSION_CHECKLIST.md` **Session-Start** as a *prompt*, not a hard gate — "when substantial work is anticipated, suggest spinning up a new worktree/branch first." Mirrors item 17d's "surface state, owner decides" pattern (a recommendation the agent raises at session start, not an automatic action). Deferred to the same checklist rework that maps the IEEE 1028 review levels.

**Gate skip in a fresh worktree** lives in `standards/CODING_STANDARDS.md` → Code Verification Process (not a MERGE checklist scope).

**Lived failure (2026-08-20) — project log is not isolated-safe.** Isolation saved the Grok walk *findings* (`L2P5_GROK_WALK_FINDINGS.md` via merge `5841d16`). It did **not** save the walk's CHANGELOG entries (`2026-08-18-001` @ `d0166d2`, `2026-08-19-001` @ `392091a`): those lived only on `grok/l2p5-agent-walk`; main already had a different `2026-08-19-001`; merge conflict kept main's log. `GIT_WORKFLOW.md`'s "delete the branch, all merged work is on main" is false for any file the merge did not copy. **Rule:** before `worktree remove` / branch delete, `CHANGELOG.md` (and `LESSONS_LEARNED.md` if the sitting added an entry) must be on `main` — new date-NNN or a one-time suffix if IDs collide; do not "take ours" and drop the branch block. Detail: LL Entry 45. Recovered onto main as **2026-08-18-001W** / **2026-08-19-001W** under 2026-08-20-003 (one-time wrap; do not copy).

---

## IEEE 1028 review-level → decision-table mapping (PROPOSED / DEFERRED) (2026-07-04, Claude/Opus)

IEEE Std 1028-2008 recorded as a **review/audit-process** reference in `standards/AUDIT_GUIDANCE.md` Appendix B.5, kept deliberately distinct from the JSF/P10/JPL **coding** standards (how we review ≠ how we write). **Provisional, not a sole standard:** useful but broad, lightly vetted so far — complementary review standards may join it later; this is a starting point, not a settled adoption. **Open rework:** the "When to Do What" decision table in AUDIT_GUIDANCE.md sets review *scope* per trigger but leaves review *depth* implicit. IEEE 1028 names five review levels — management review, technical review, **inspection**, walk-through, audit. Map those onto the 7-tier procedure so each trigger states which depth applies (e.g. the L2-P5 manual walk = an **inspection** per Appendix B.4 "Phase 9"; a small change = a walk-through). Deferred — do when the audit procedure is next reworked. Owner decision on priority.

---

## Graphify — curated code+docs graph rebuilt; doc→code connectivity pass DEFERRED (2026-06-27, Claude/Opus)

Rebuilt the graphify graph at `graphify-out/` as a **curated current-state code+docs map** (supersedes the bloated bootstrap). **2448 nodes / 4521 edges / 186 communities, 71% in giant component, full-detail HTML (<5k).** Protected snapshot at **`graphify-out/claude-build-2026-06-27/`** (with README; safe from future `graphify .` runs, which only rewrite the root). Grok pass-3 snapshot `graphify-out/grok-build-pass3-2026-06-27/` untouched (its own protected folder). `.graphifyignore` updated — now excludes vendored (`EXTERNAL/` ETL, `lib/`), `test/`+`scripts/`, `mcps/`, `starcom/`, `logs/`, tooling configs, images, and historical churn (`docs/plans/`, `docs/audits/`, `docs/baselines/`, `CHANGELOG.md`). **Not committed** (tooling output; pending repo-owner direction).

**DEFERRED — doc→code connectivity pass (run when the API rate-limit window resets):** `standards/` (113 nodes) + ~431 `docs/` nodes are still their own islands — doc concepts vs code symbols live in different naming spaces, and chunk-parallel extraction dropped ~672 cross-refs on ID-mismatch (prune + fuzzy reconcile already lifted connectivity 50%→71%). **Next:** targeted linking pass — hand a subagent the AST code-symbol list + the orphaned `standards/`/`docs/` nodes and have it draw real doc→code bridge edges, then rebuild. **Rate-limit lesson:** dispatch graphify extraction subagents in **batches of ~4**, NOT all at once (17-at-once tripped a server-side rate limit this session; batches of 4 sailed through).

**WB note (2026-06-27):** Evaluate `standards/GIT_WORKFLOW.md` for outright deprecation. It is no longer referenced from AGENTS.md and its `claude/` branch prefix + immediate-delete rule are over-specific for general agent workflows.

**Protected-file hook DISABLED 2026-06-28 (Claude) — not deletable into a good cross-runner solution.** The category+`ask` hook (CHANGELOG `2026-06-28-002`) works on **Claude Code** (per-file approval prompts via `permissionDecision:"ask"`), but on **Grok Build CLI it can't prompt at all**: Grok's PreToolUse hook contract is `{"decision":"allow"|"deny"}` only — no `ask` — so the `ask` output is unrecognized → fail-open → the protected edit silently succeeds (critical false-positive, observed in a Grok session; see `temp/grok-hooks-claude-intake-notes.md` + Grok docs `~/.grok/docs/user-guide/10-hooks.md` lines 188-201). On Grok the only hook signal is hard-`deny` (no prompt). Grok's permission *rules* (`permissions.ask` path patterns in `.claude/settings.json`) DO prompt on both runners, but are static file-granularity only (can't see the diff, so no "CHANGELOG additions allowed" logic), and Grok's user-facing modes are just always-approve / normal with zero granularity. None of those was the solution Nathan wanted, so the hook is **disabled** (PreToolUse `search_replace|Edit|Write|MultiEdit` entry removed from both `.claude/settings.json` files; graphify Bash/Read|Glob hooks untouched). For now we rely on clear docs (`PROTECTED_FILES.md`) + the agent respecting them.

**Dormant assets (NOT deleted, ready to revive):** `scripts/hooks/protected-file-pretool.py` (category model: Hard-Protected / Historical / Checklist-cadence with add-only diff detection) + `tools/scratch/protected_hook/contract-test.py` (20/20) + the 3-category structure in `PROTECTED_FILES.md`. Re-wire by re-adding the PreToolUse entry when a viable approach exists.

---

## L2-P5 §CM to-implement backlog — mechanical checks surfaced by the walk-guide trim (2026-07-01, Claude)

Trimming `L2P5_MANUAL_WALK_GUIDE.md` down to human-judgment-only lenses (on branch `claude/l2p5-walk-guide-c3757857`, recovered + committed 2026-07-01) surfaced a set of **mechanically-decidable** checks that had been riding inside the manual walk. Per the coverage≠mechanical principle, each belongs in a gate/grep, not the eyeball pass. **Repo-owner direction: wire these when patching §CM *after* the manual walk — likely as part of re-doing the audit**, not now.

**To implement (currently ungated / partial):**
1. **Commented-out code** (CERT MSC04-C / JSF 127) — greppable (a block of real C++ inside `//` or `/* */`) + the nested-`/*` delimiter hazard. Manual residual kept in Class 3: delete-vs-deliberate-`#if 0` disposition.
2. **Per-function assertion count / >10-line trigger** (P10-5 / JPL-16 density floor) — countable/greppable; function length is already measured by `readability-function-size`, the per-fn assertion count is not.
3. **`template<>` explicit function-specialization** (CCG T.144) — grep the banned token; presence = finding.
4. **static_assert-guard presence** on POD types meant to model a concept (CCG T.150) — greppable; `misc-static-assert` is enabled but doesn't cover "a guard exists for type X."
5. **Per-instantiation test coverage** (JSF 102 / T.102) — compiler emits the instantiation list; needs a coverage manifest to auto-check.
6. **lock-in-A / unlock-in-B cross-function pairing** (JPL 9 / CON51) — grep or a custom clang-tidy matcher (triage's stated conversion). Live surface: 1 `save_and_disable_interrupts`/`restore_interrupts` pair in `psram_init.cpp`, 0 spinlock pairs. Manual residual kept in Class 10: every abnormal exit releases the lock.
7. **JSF AV 166 — side-effects-in-`sizeof`** (canon; = the `sizeof` case of CERT EXP52) — greppable; `bugprone-sizeof-expression` is enabled but targets broader suspicious-`sizeof`, not side-effect-in-operand. **0 current violations in `src/`** (zero-risk gap). Surfaced 2026-07-01 by the I.4/EXP52 canon re-check.

**Already gated (no action — recorded for the split):** the other trimmed items already hit enabled gates — `bugprone-assert-side-effect`, `-Wnon-virtual-dtor`, `cppcoreguidelines-special-member-functions`, `google-explicit-constructor`, `readability-container-size-empty`, `-Wall`→`-Wsequence-point`, `cppcoreguidelines-init-variables`+`clang-analyzer`, `readability-function-size`/`cognitive-complexity`, `bugprone-branch-clone`, `clang-analyzer-deadcode`, `[[nodiscard]]`, **`readability-magic-numbers` (Class 13 demoted 2026-07-28; residual notes on `.clang-tidy` + CODING_STANDARDS)**. **Class-6 narrowing** (56 `-Wconversion`/`-Wsign-conversion` findings) is already tracked in the L2-P5 handoff above + `L2P5_WCONVERSION_FINDINGS_2026-06-25.md`.

**Class-14 do-not-"fix" guards (cut from the walk guide 2026-07-01; preserved here — they're gated-class cautions, not walk criteria):** while remediating or gating the mechanical surface, do **not** (a) convert mandated `uintN_t` / hardware-register / bitmask code to signed, or (b) add redundant arithmetic parentheses (JSF AV 213 is deliberately disabled — LL Entry 26). These guarded against a reviewer over-"fixing" the gated Class-14 (expressions / evaluation-order) surface; kept in case that surface is touched.

---

## Use Cases
1. **Cross-agent review** — Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** — Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** — Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** — Flag items needing user input before code changes
5. **Deferred items** — Active intent kept visible until acted on (and then erased — see header rule)

---

## High priority

- **Codegen audit — verify every generated/codegen area is handled properly (2026-06-23, Claude).** Surfaced during the L2-P5 walk. `src/flight_director/mission_profile_data.h` is banner-marked "AUTO-GENERATED by `scripts/generate_profile.py` / Do not edit", yet was **hand-edited** in commit `b1f25ce` and has **drifted from its generator**. A rigorous diff (HEAD generator-output vs committed) confirms the post-gen hand-edits are **exactly two, both Stage-T radio, and nothing else differs** (the whole `MissionProfile` struct matches the generator): **(1)** a `#ifdef ROCKETCHIP_STAGE_T3_MAVLINK` protocol switch (MAVLink vs CCSDS), **(2)** the `// Stage T IVP-T6 sweep …` comment. The build does **not** regenerate the file (no `generate_profile` in CMake), so the documented "edit `profiles/rocket.cfg` + regenerate" workflow would **silently delete the MAVLink switch**. `test/test_hab_profile_data.h` also appears **stale** (omits current struct fields: `default_lat_deg`, `drogue_timer_s`, `phase_qr`, …). **Do NOT retire the generator** — it backs the user-facing profile UX + setup wizard (the wizard itself is due for a cleanup, ideally after the CCSDS work lands). **Proper fix (deferred — not blocking; committed files compile correctly):** (a) re-encode the two hand-patches into `profiles/rocket.cfg` + `generate_profile.py` so regeneration is lossless; (b) refresh the stale hab profile; (c) wire the generator into the CMake build so generated headers regenerate each build and can't drift (cfg = single source of truth); (d) **sweep for any OTHER "AUTO-GENERATED" file that has been hand-touched** (the ESKF SymPy codegen `eskf_codegen.cpp` is separately covered by deviation CG-1; check the rest). A standards rule was added this session (`CODING_STANDARDS.md` → "Auto-Generated Code": never edit post-gen output; edit the generator/input). Until the proper fix lands, **do not regenerate `mission_profile_data.h` without re-applying the two `b1f25ce` hand-patches above.** (This session applied the `#pragma once`→`#ifndef` guard + `has_default_location` field *directly* to the committed file + the generator to avoid clobbering, per the surfaced-issues rule.) Walk-WB W-14 landed here: when the audit actually runs, **start from scratch** — earlier method sketches (including the (a)–(d) list) are prior notes, not the audit spec. Lived facts on this row stay evidence.

- **AO Commandments source-citation audit.** Investigating R-27 (RfManager Commandment XII observation) surfaced that Commandment XII's `Source:` line cites LL Entry 36, but LL 36 is about test-tool rot (bench_flight_sim.py going stale), not AO state-transition logging or runtime observability. A research agent walked the doc's stated sources (Samek PSiCC2 Ch. 11, state-machine.com Active Object/RTEF/QP/C SRS pages, NASA F´ Code Style + State Machines doc) and confirmed **no clean substitute citation exists in any of those** — the rule is project-internal invention generalized from folklore, not inherited from external authority. This is an [LL Entry 37](docs/agents/LESSONS_LEARNED.md)-class citation-rot finding. Per Entry 37 discipline ("if one citation was wrong, check the rest"), audit all 12 Commandment `Source:` lines in `docs/decisions/AO_COMMANDMENTS.md` against their cited sources; fix XII's citation (either reframe as project-internal "Rationale:" or cite PSiCC2 Ch. 11 honestly as topical-but-tool-framing); reassess R-27's disposition once the rule's authority is correctly understood. Est. ~1-2 hrs. Block on this is open per user direction 2026-05-22 — address before closing R-27.

- **Four-cycle plan — Cycle 4: L2-P5 itinerary complete 2026-08-08** (121/121, WN-001–327); **walk WB drained + formal walk close 2026-08-17**. Next: WN cluster index, then disposition / Cycle-4 remediation; L2-P10 CLA-RBM. Cycles 1-3 closed. See CHANGELOG 2026-08-08-001 and 2026-08-17-001.


- **IVP-T13 LQ-adaptive retry — deferred until after the CCSDS command-
  layer rework.** Original Stage T Batch C plan was to port the ELRS
  LQCALC pattern (retry aggressiveness scales U-shape with LQ: fewer
  retries when LQ is high, more when marginal, pause when LQ is near-
  dead) behind `ROCKETCHIP_LQ_ADAPTIVE_RETRY`, default OFF. Decision
  2026-04-22: don't polish parametric tuning on top of a retry
  architecture we've already marked as STOP-GAP pending proper CCSDS
  TC-Layer + COP-1. T13 reopens once the CCSDS command path lands —
  the adaptive algorithm likely maps onto whatever flow-control / FARM
  machinery the CCSDS layer provides, not onto naive retry counts.
  `AO_RfManager_ok_to_retry()` API already exists in-tree, ready for
  use when T13 returns.

- **Station→vehicle radio health channel — deferred to CCSDS rework batch.**
  Council A3 asked for condensing station readiness to a single bit the
  vehicle's GO/NO-GO consumes via radio. Current channel is command-only,
  no periodic telemetry-back. Moved into the CCSDS rework batch (with
  IVP-T13 + Stage T re-baseline) on 2026-05-21 — the telemetry-back
  direction wiring depends on what TC-Layer / TM-Layer split the CCSDS
  rework lands, so designing it now would commit to an interface that
  the CCSDS work will likely change.

- **Re-evaluate Stage T "95% first-try" gate with correct baseline.**
  User observation 2026-04-22: the Stage T diagnostics measured
  operator-burst ACK rate (10 Hz retry over ~300 ms), which treats "3rd
  retry succeeded at 300 ms" as a failure. But on a half-duplex LoRa
  link with sparse station TX (no heartbeat before IVP-T14d), a
  3rd-retry ACK latency of ~300 ms is within ABORT's 250 ms budget +
  reasonable operational margin. The 6.7% first-try number conflates
  "link broken" with "expected half-duplex latency for a burst into a
  sparse RX window." Once CCSDS station beacon lands, re-measure: (a)
  steady-state first-try success when station is continuously TXing;
  (b) actual ACK-latency distribution (p50 / p95 / p99); (c) whether
  the anchor-station-TX-to-vehicle-RxDone architectural fix is still
  needed or was over-engineered for our actual link conditions.

## Medium (session-scale, 4–12 hours)

Scope is clear but touches multiple files, needs verification, or has small design questions.

- **QP/C vs QP/C++ framework evaluation (2026-06-23, Claude).** The project uses **QP/C** (C framework) from C++ TUs — confirmed by first-member event composition (`struct PhaseChangeEvt { QEvt super; ... }`, `include/rocketchip/ao_signals.h`) + the "Allocated from QP/C dynamic event pool" comment + free `QActive_publish_()` API. The FreeRTOS→QP migration was council-reviewed, but **no record exists of the QP/C-vs-QP/C++ sub-choice being weighed or surfaced to the user** — it appears to have been made implicitly during the migration. The choice seems sound and works, but an seemingly-arbitrary undocumented decision of this size is a red flag (user direction 2026-06-23). **Do a full QP/C vs QP/C++ evaluation**: what each buys/costs for this codebase (event ergonomics, type-safety, the `(void *)0`-vs-`nullptr` and reinterpret_cast-downcast idioms QP/C forces, framework-core C++ runtime footprint, JSF/MISRA posture, migration cost if switching), whether the implicit choice was correct, and **record the rationale** (back-fill a decision doc either way). Surfaced during the L2-P5 standards walk while dispositioning the QP/C `(void *)0` sender idiom. **Concrete standards data point for the eval:** QP/C's first-member event model forces `reinterpret_cast<const DerivedEvt*>(e)` downcasts of `QEvt const*` (~10 AO sites, being centralized into one `evt_cast<E>` helper) — a standing **JSF-182** residual (no Exception covers it; well-defined only via standard-layout first-member equivalence). In **QP/C++** these become real inheritance downcasts (`static_cast`, JSF-178-compliant), **eliminating the JSF-182 residual outright**. So the C++ edition would retire a coding-standard exception — weigh against migration cost in the eval.

- **QP/C naming-convention divergence — TRACKED (2026-06-24, Claude).** The L2-P5 naming pass renamed the project's AO/QP code from Samek/QP house conventions to the project's JSF house standard: QP **`l_`** module-static prefix → `g_` (JSF-209/CODING_STANDARDS:469 static convention); QP state-handler **`Xxx_initial`/`Xxx_running`** CamelCase → `lower_case` (JSF AV Rule 51); QP **`s_evt`/`tx_evt`** event statics → `g_`-prefixed. **Why this is safe (researched 2026-06-24, primary sources):** (1) QP/C consumes these as *function pointers* (`Q_STATE_CAST(&FdAo_initial)`) and *variable identifiers* (`&l_fdAo.super`) — names are arbitrary to the framework, only signatures + registration matter; (2) the AO code is **hand-written** (no `.qm` model, no QM-generated banners) so nothing regenerates QP names back; (3) Quantum Leaps publishes their own coding style (QL-C/C++:2022) as **editable markdown explicitly meant to be forked/customized** to a project's house standard. So QP naming is *guidance, not a hard line*, and JSF governs (no accepted-deviation needed). **Watch-items down the line (the reason this is tracked):** (a) QP forum/book examples + `docs/decisions/AO_COMMANDMENTS.md` use Samek naming — a QP-veteran reading our AO code sees house naming instead; (b) **if QM (the QP modeling tool) is ever adopted**, its generated code reimposes QP conventions → the divergence would resurface as a generated-vs-house conflict (revisit then); (c) pairs with the QP/C-vs-QP/C++ eval above. **Formalization TODO:** record this as an "Identifier naming (QP/Samek vs JSF)" bullet in `CODING_STANDARDS.md` → "Worked consolidation decisions" (alongside the function-pointer P10-vs-JSF and nullptr-vs-JSF-175 resolutions) — that file is PROTECTED, so needs repo-owner to name it for editing.

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). Extend when corresponding firmware behavior lands: multi-pending-in-flight, RadioScheduler TX-window arbitration (needed for the sync-gap fix), MAVLink parser state, `station_idle_tick` GPS poll interleave.

- **`LESSONS_LEARNED.md` cleanup + update pass (2026-08-20).** Medium: the file is the process journal agents actually re-read, and it has aged in place. **Not** a rewrite of historical entries (append-only / frozen existing text). Pass should: (a) refresh the header (still says "so Claude can learn"; format claims every entry has time-spent, many don't); (b) kill or update the stale **Plan Files Reference** (`virtual-scribbling-finch.md` / AP_Vehicle, 2026-01-31 — almost certainly dead); (c) navigability — Entry 34 sits after 35; process/tooling lessons (36–45) sit in a HW-debug journal with no index; superseded Entry 25 is marked, check for other rot; (d) confirm FreeRTOS-archived numbers 7–10 / 14 / 17–19 still make sense as holes vs a short "see branch" pointer. Owner names the file when the sitting runs. Do not start this unprompted.

## Large (multi-session, architectural)

Needs council review or planning doc before starting.

- **Real-World Accuracy Tests plan.** Bench-side ground-truth validation — IMU known-angle tilts, baro altitude vs reference, GPS stationary/moving baseline characterization, ESKF replay vs synthetic truth, Allan variance for gyro/accel. Doesn't need launch window or airframe. Complements Stage 18 field tuning. Needs dedicated plan doc with prior-art research (ArduPilot EKF tuning, PX4 calibration) and equipment assessment.
- **Launch procedure audit items.** Six future safety items from NASA/SpaceX/NAR procedure comparison, all requiring Mission Profile or hardware support:
  1. Angle-rate abort guard (BOOST bank threshold → ABORT; needs IMU attitude in BOOST)
  2. No-pyro-after-impact guard (landing guard before apogee guard → suppress pyro)
  3. Hung-fire / ignition timeout (track time since ARM, station-side exclusion timer)
  4. Igniter continuity check (station-side pre-arm check)
  5. Air-dropped vehicle profile (altitude-aware abort, no "stay on ground")
  6. Multi-engine / staging support (partial engine light, inter-stage hold, TRA 13-9)

## Research / Deferred

No code changes planned — kept as context for future decisions.

- **Datasheet RAG + spec-table→code transcription — evaluate (2026-07-28, Claude/Opus).** Two related ideas surfaced while scoping a local-LLM companion workflow; **both are worth visiting regardless of whether the model is local or cloud** — the value is in the pattern, not the hosting.
  1. **Datasheet RAG.** Stand up a retrieval index over the primary-source PDF set we already lean on (RP2350 datasheet, SX1276, Pico SDK docs, and for Starcom the CCSDS Blue Books) so register/bit/field lookups are a query instead of a page-hunt or a context-burning full-PDF read. Directly attacks the friction documented in [[l2p5-standards-walk-guide]]'s source-fetch method note (WebFetch's extractor fails on the compressed standards PDFs; pypdf + regex is the current workaround). Local embedding models are cheap enough (~0.6B, ~1.5 GB) that this is not a token-cost decision. **Caution:** RAG retrieval is a *pointer*, not an authority — per LL Entry 37 + 38, any rule/register value it surfaces still gets verified verbatim against the primary source before it lands in a doc or a citation. A summarizer already fabricated P10 quotes once.
  2. **Spec-table → code, compiler-verified.** Transcribe tabular layout specs (CCSDS Blue Book bit layouts; RP2350/SX1276 register bitfields) into C++ structs + `static_assert` on `sizeof`/`offsetof`/field masks. The interest here is that the output is **machine-checkable** — the compiler proves the layout, so this is delegation with a real gate rather than trust. **Bounded claim (important):** `static_assert` proves *layout*, NOT *semantics* — a field transcribed at the correct offset under the wrong name or meaning still passes. So the gate covers the mechanical half only; field identity/meaning stays a human/primary-source check. Do not let "static_assert clean" become a Rule-7-style over-claim (see `RULE_VERIFIABILITY_TRIAGE.md` severity model: over-claiming tool-claims are critical findings).

  **Not scheduled, no code planned.** Most load-bearing for Starcom (strict Blue Book bit-layout fidelity is the whole point of the library) — evaluate before Starcom's data-link framing code locks in.

- **ELRS on RP2350 — research item.** Running ExpressLRS natively on RP2350 with PIO-assisted frequency hopping. Current RFM95W (bare SX1276 on SPI) may be compatible if packet format + hopping schedule can be implemented in firmware. Telstar Booster Pack already describes CRSF/UART to a dedicated ELRS module as the alternative path. Future radio protocol investigation.
- **PIO hardware failure gap — Gemini tier only.** IVP-130 Scenario 5 confirmed: external PIO SM halt is undetectable by firmware (PIO watchdog IRQ only fires from PIO program itself; ARM-side monitoring defeats the independence point). Correct mitigation = physical redundancy (second independent timer on separate MCU). Gemini-tier feature (dual-core carrier board). Accepted gap for Core/Titan.

- **Fault-recovery rework follow-ups (commits `ed7c569` + `8baa18a` landed 2026-05-14/15; see `docs/decisions/FAULT_RECOVERY_2026-05-14.md` for the design):**
  - **PIO beacon + SPI last-gasp combined session** — see dedicated row below.
  - **AON-timer prior-uptime signal** — stubbed to 0 in the anomalous-boot confidence gate. Wiring it requires adding `pico_aon_timer` to target_link_libraries + explicit timer-start at boot. Marginal value (POWMAN reset register already carries the high-confidence signal for brownout; AON timer would corroborate for the watchdog-RSM / hazard-DP / glitch-detect / SWcore-PD reset classes only). Worth picking up if auto-zero-baro suppression false-positive rate during bench testing needs an extra corroborator. Otherwise deferred.

- **PIO beacon + SPI last-gasp beacon (B.5) — combined dedicated future session.** **Group:** Early-impl / rework-eval candidates (index above; shares **PIO budget** with I²C-backend eval). Council round 3 (NASA/JPL + Cubesat, 2026-05-15) unanimously deferred the SPI-based last-gasp beacon (commit (c) of the rework was scoped for this and *not* implemented). User direction 2026-05-15: "merge with the future PIO beacon" — the two questions evaluate together rather than pre-committing to an interface (compile-time `ROCKETCHIP_LAST_GASP_BEACON` + `radio_init_confirmed` semantics) that would constrain the PIO design choice. Reasons for deferral, fully captured in plan B.5 + council-round-3 transcript at `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a355e8caee0717e0b.md`:
  - **SPI peripheral state corruption** if fault occurred mid-byte/mid-burst — recovery is NOT bounded-cost (FIFO drain + CS deassert via GPIO function override + SX1276 hardware reset pulse + full cold re-init; each step has its own hang potential).
  - **`#ifdef`-scaffolding-rot pattern** per LL Entry 36 — code-shaped-but-never-exercised artifact creates false-confidence for future contributors.
  - **JPL precedent: always architect beacons as independent silicon** (Cassini LGA+USO, SMAP transponder). Cubesats that share the radio rely on modem-level autonomous beacon modes (e.g., FSK Beacon Mode); SX1276 LoRa lacks this in long-range mode.
  - **PIO beacon is the architecturally-correct answer** (Pico SDK has working PIO-SPI in 2-3 instructions per primary-source verification 2026-05-14; DIO0 wired to GPIO 6 on Adafruit Feather RP2350; DIO5 not currently routed but solder-jumperable on the RFM95W FeatherWing #3231).
  - **In-flight ARM-dead beacon-coverage gap is the trade** — accepted as a known gap for Core/Titan single-MCU pending this session.

  Scope of the combined session: (1) evaluate whether SPI-from-fault-handler is ever the right stop-gap given the failure-mode inventory; (2) design the PIO-driven beacon program (target: PIO0 or PIO1 — PIO2 already shared between watchdog SM0 + backup-timer SM1-3); (3) decide whether the design requires soldering the DIO5 jumper on the FeatherWing; (4) bench-verify on a known-faulted chip state if any stop-gap is in scope. Likely outputs a dedicated decision doc under `docs/decisions/`. User direction will determine sequencing relative to other open work.
- **RP2350B/Fruit Jam persistent bus-corruption hypothesis.** User hunch 2026-04-17: one boot during the Fruit Jam GPS debug had a transition not fully explained by the cable theory alone. Investigate whether RP2350B exhibits bus-corruption state that survives power cycles. Low priority — may be a dead end, keep passive.

- **I²C bus backend rework-eval (prefer PIO if advantages + budget) + Flipper prior art.** **Group:** Early-impl / rework-eval candidates (index above). **Owner lean (2026-08-05):** PIO master is **preferable** when advantages are real and **PIO budget** remains (coordinate with PIO beacon / watchdog allocation — do not eval in isolation). Current `i2c_bus` thin façade over DW_apb stays intentional either way; backend swap is the rework question, not deleting the project API. **Trigger / residual context:** Fruit Jam GPS cold-boot intermittency and related bus pain (`docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`) — not a mandate to switch mid-walk. **Prior art:** Flipper One MCU firmware (public 2026-05-21) has a working RP2350 PIO I²C master at `lib/drivers/i2c_master_pio/pio_i2c.c` in https://github.com/flipperdevices/flipperone-mcu-firmware — claim/init + unclaim/deinit (LL-42 pattern), mid-cycle recover via drain + jump-to-wrap + IRQ clear (no program-memory touch), acquire/release pad-mux (LL-28) in `furi_hal_i2c_config.c`. Same SDK/toolchain family as us. **License check required before any code import.** Eval session outputs: advantages vs HW I²C, SM budget map, keep/reaffirm DW_apb vs plan PIO backend under `i2c_bus_*`.

## Deferred (near-term, post-Stage 15)

- **Battery ADC monitoring.** Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS command authentication.** Telecommand auth for Rocket profile.

## Far-future

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export — all tracked in `docs/PROJECT_STATUS.md` future features.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** — AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 — evaluate justified vs. bloat).

**Stage 16: Field Tuning** — All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** — IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.
