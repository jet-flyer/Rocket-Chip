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

## Project status (one-line snapshot)

**Stages 1-14 + 16A + 16B + 16C + L + T COMPLETE.** **856** host `ctest` entries (852 C++ discovered + **4** Python `scripts/` gates), SPIN 11/11. Tracking: `docs/AO_ARCHITECTURE.md`. **Stage 17 (Field Testing & Avionics Airworthiness) restructured 2026-04-22** from 5-IVP direct-to-flight into 13-IVP tapered buildup (three council rounds, approved with amendments). First motor flight = step 13 of 14. Plan: `docs/plans/STAGE17_TAPERED_BUILDUP.md`. Execution awaits future session; starts with IVP-135a (pure-software log schema extension). **CCSDS TC-Layer + COP-1 rework deferred to post-Stage-17** (unanimous council) — field data will inform scoping.

---

## Session Handoff Notes (2026-05-26, Grok)

**User requested explicit session handoff + pause.**

**Work completed this session:**
- Deep dive into current CCSDS status (TM side implemented as pruned Space Packet; TC + COP-1 is the missing STOP-GAP).
- Created thorough current-state analysis of the command/retry/ACK reliability layer (the main area the CCSDS command rework would replace).
- Produced two primary artifacts for CCSDS prep:
  - `docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.md` (detailed textual map with file:line citations)
  - `docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.dot` + rendered `.svg` (Graphviz diagram matching project conventions from `docs/audits/cla_rbm/dot/`)
- Installed Graphviz via winget so the visualizer can be used going forward.
- Graphviz is now present on the machine (`C:\Program Files\Graphviz\bin\dot.exe`).

**Files touched (all new, untracked):**
- docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.md
- docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.dot
- docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.svg

**Current state of CCSDS prep work:**
- We now have a high-quality, citable "as-built" map of the existing command delivery + retry mechanism.
- This directly supports scoping, interface design, and migration planning for any future CCSDS TC-layer work (including the standalone library idea the user mentioned).
- 2026-06-22: Condensation of all CCSDS prelim docs (research/* + comparison + design_record) into starcom/docs/DESIGN.md completed on feat branch and merged to main. Includes agreement/conflict/gaps table, §0 verbatim, state machine summary + external Blue Book REF, D-1..D-5, unique data. Modern naming (no I-prefix). Detailed council verdict for D-2 sans-I/O deferred to design_record_claude.md Round 2. Historical docs untouched. Pure doc. (Grok)
- Multiple items remain explicitly parked pending this rework (see "High priority" section below: IVP-T13, station radio health channel, first-try metric re-baseline).

**What was discussed but not yet executed:**
- Next actions with the data flow artifacts (scoping doc, library contract definition, target-state diagram, design doc for council, etc.).
- User indicated they want to pause for the night.

**Handoff instructions from user:**
- **Check with user before any commit or push.** Do not commit these files or push without explicit approval in the next session.
- Working tree currently has only the three new decision files above as untracked changes.

**Open questions / suggested next steps for next session:**
- Which direction to take the data flow work (scoping options, library behavioral contract, target-state diagram, full design doc)?
- Any specific refinements needed to the .dot / MD before using them in a council or design review?
- Whether to create a dedicated CCSDS command layer planning document.

**Build / verification state:** N/A — pure documentation work. No src/ changes. No builds or tests affected.

---

**End of handoff note.** (Erase or update this section when work resumes.)

## Session Summary (2026-05-28, Grok) – WSL Soft Pivot

**Work delivered:**
- Full WSL_SOFT_PIVOT plan executed (Phases 0–6).
- Linux FS worktree (`~/Rocket-Chip`) created; clean vehicle + station firmware builds + both `bench_sim.py` (2/2) and `station_bench_sim.py` (3/3) proven from it with positive controls.
- Documentation delivered at docs root: `WSL_SETUP.md`, `WSL_QUICKSTART.md`, `WSL_ROLLBACK_CHECKLIST.md`.
- Steady-state policy ("try until it breaks") reviewed via council; recorded in `docs/decisions/WSL_STEADY_STATE_POLICY_2026-05-28.md`.
- Dual-toolchain exercise requirement added to milestone checklist (item 17c).

**State at close:**
- WSL (Linux FS) is now a fully functional primary environment.
- Windows remains supported fallback with explicit, lightweight rollback checklist.
- Repo clean. All plan deliverables delivered.

---



## Council review — Starcom (completed)

Council review of all Starcom research findings (Grok vs Claude) completed. Full details + naming suggestions recorded in `starcom/docs/comparison.md` (new "Council Review — Universal CCSDS Scope" section). Starcom library docs live under `starcom/` — see `starcom/docs/WORKING_HERE.md`.

**Key outcome:** Core should be called "Starcom Core". Use `ILink` (or similar) for the transport abstraction. PHY tiers (none / best-effort / full compliant) as optional adapters outside the core. Recommendations re-framed for universal CCSDS use.

**End of note.** (Erased per whiteboard rules after review.)

---

## Use Cases
1. **Cross-agent review** — Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** — Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** — Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** — Flag items needing user input before code changes
5. **Deferred items** — Active intent kept visible until acted on (and then erased — see header rule)

---

## High priority

- **Codegen audit — verify every generated/codegen area is handled properly (2026-06-23, Claude).** Surfaced during the L2-P5 walk. `src/flight_director/mission_profile_data.h` is banner-marked "AUTO-GENERATED by `scripts/generate_profile.py` / Do not edit", yet was **hand-edited** in commit `b1f25ce` and has **drifted from its generator**. A rigorous diff (HEAD generator-output vs committed) confirms the post-gen hand-edits are **exactly two, both Stage-T radio, and nothing else differs** (the whole `MissionProfile` struct matches the generator): **(1)** a `#ifdef ROCKETCHIP_STAGE_T3_MAVLINK` protocol switch (MAVLink vs CCSDS), **(2)** the `// Stage T IVP-T6 sweep …` comment. The build does **not** regenerate the file (no `generate_profile` in CMake), so the documented "edit `profiles/rocket.cfg` + regenerate" workflow would **silently delete the MAVLink switch**. `test/test_hab_profile_data.h` also appears **stale** (omits current struct fields: `default_lat_deg`, `drogue_timer_s`, `phase_qr`, …). **Do NOT retire the generator** — it backs the user-facing profile UX + setup wizard (the wizard itself is due for a cleanup, ideally after the CCSDS work lands). **Proper fix (deferred — not blocking; committed files compile correctly):** (a) re-encode the two hand-patches into `profiles/rocket.cfg` + `generate_profile.py` so regeneration is lossless; (b) refresh the stale hab profile; (c) wire the generator into the CMake build so generated headers regenerate each build and can't drift (cfg = single source of truth); (d) **sweep for any OTHER "AUTO-GENERATED" file that has been hand-touched** (the ESKF SymPy codegen `eskf_codegen.cpp` is separately covered by deviation CG-1; check the rest). A standards rule was added this session (`CODING_STANDARDS.md` → "Auto-Generated Code": never edit post-gen output; edit the generator/input). Until the proper fix lands, **do not regenerate `mission_profile_data.h` without re-applying the two `b1f25ce` hand-patches above.** (This session applied the `#pragma once`→`#ifndef` guard + `has_default_location` field *directly* to the committed file + the generator to avoid clobbering, per the surfaced-issues rule.)

- **Standards-currency audit (2026-06-17, Claude) — two cited standards are stale; fix before Starcom code locks coding definitions.** Web-verified (two research agents, high confidence, primary sources). Findings:
  - ⚠️ **CCSDS 131.0-B-3 (TM Sync & Channel Coding, 2017) is SUPERSEDED TWICE — current is 131.0-B-5 (Sep 2023).** ccsds.org marks B-3 historical. This is the doc that defines the convolutional/LDPC codes the Proximity-1 C&S sublayer (211.2-B-3) references. The Starcom research docs + `STARCOM_RESEARCH_COMPARISON.md` cite **131.0-B-3** — stale by two issues. **Action:** update references to 131.0-B-5 — BUT first check *which* 131.0-B issue 211.2-B-3 itself normatively points to (that issue, not "latest," governs PLTU coding fidelity). Current PDF: https://ccsds.org/Pubs/131x0b5.pdf
  - ⚠️ **MISRA C:2012 → superseded by MISRA C:2023** (Apr 2023; not withdrawn, but no longer latest). Project's MISRA-C is already "deferred-with-rationale" per `CODING_STANDARDS.md`; this only updates *which edition* "current MISRA C" means if/when the deferral is revisited. No action required now.
  - ✅ **All other standards confirmed current:** JSF AV C++ (2005, frozen but never officially withdrawn — our citation is the only/final version), Power of 10 (2006, canonical), JPL-C (2009, current per NASA SWEHB), and **all six other CCSDS Blue Books we cite** (211.1-B-4, 211.2-B-3, 211.0-B-6, 732.1-B-3, 232.1-B-2, 232.0-B-4 — all current issues).
  - **MISRA C++:2023 — confirmed PAYWALLED** (PDF ~£15, hardcopy ~£45, no free tier; only rule titles/IDs are free via tool-vendor pages). Per user direction 2026-06-17: **not adopting it** (paywall conflicts with the open-source-audit principle; ~90% overlap with JSF+P10+JPL already enforced; hobbyist tier). Stays a *compatible-with* target for Starcom's core, not an adopted standard. AUTOSAR C++14 is free but explicitly out of scope (use what we have). No further action.

- **AO Commandments source-citation audit.** Investigating R-27 (RfManager Commandment XII observation) surfaced that Commandment XII's `Source:` line cites LL Entry 36, but LL 36 is about test-tool rot (bench_flight_sim.py going stale), not AO state-transition logging or runtime observability. A research agent walked the doc's stated sources (Samek PSiCC2 Ch. 11, state-machine.com Active Object/RTEF/QP/C SRS pages, NASA F´ Code Style + State Machines doc) and confirmed **no clean substitute citation exists in any of those** — the rule is project-internal invention generalized from folklore, not inherited from external authority. This is an [LL Entry 37](docs/agents/LESSONS_LEARNED.md)-class citation-rot finding. Per Entry 37 discipline ("if one citation was wrong, check the rest"), audit all 12 Commandment `Source:` lines in `docs/decisions/AO_COMMANDMENTS.md` against their cited sources; fix XII's citation (either reframe as project-internal "Rationale:" or cite PSiCC2 Ch. 11 honestly as topical-but-tool-framing); reassess R-27's disposition once the rule's authority is correctly understood. Est. ~1-2 hrs. Block on this is open per user direction 2026-05-22 — address before closing R-27.

- **Four-cycle plan — Cycle 4 IN PROGRESS (L2-P5 manual standards-walk).** L2-P5 JSF AV walk + L2-P10 CLA-RBM re-collection. Cycles 1-3 closed; gate open (R-5 + Cycle-3 closed). See CHANGELOG for cycle-by-cycle history.

  **── SESSION HANDOFF / temp-record (2026-06-21, Claude Opus 4.8) — resume here. CHANGELOG entry now written (`2026-06-21-001`, post-written after a restart); this block is kept for the forward-looking resume state (built/pending classes, next steps). ──**

  ⚠️ **Provenance caution:** commit `5a6cf87` (below) never got a CHANGELOG entry, and its original WB temp-record (commit `d3efe29`) was overwritten by a later cross-agent WB commit. So this block re-carries that lost state *plus* this session's work. **Push state:** `5a6cf87` + `d3efe29` ARE on `origin/main` (carried up by a later agent's push); only this handoff commit `ab44feb` is unpushed.

  **What's DONE and committed (`5a6cf87`, 2026-06-17 — pushed to origin, but NOT CHANGELOGged):**
  - `docs/audits/RULE_VERIFIABILITY_TRIAGE.md` (NEW) — unified master rule-verifiability triage, workflow-generated, primary-source-verified. Corpus P10 (10) + JPL-C LOC-1..4 (31) + JSF AV (233 distinct IDs); 225 property-rows; LOC-5/6 (89 MISRA) deferred. §1 method · §2 P10 · §3 JPL · §4a-d JSF · §5 cross-ref · §6 deferred · §7 findings. This is the **driver** for everything below.
  - `standards/CODING_STANDARDS.md` (PROTECTED) — bounded, repo-owner-authorized amendment: "Cross-standard rule-equivalence map" table under Foundation (before "JPL Additions"). Table + note only.

  **What's DONE this session (`L2P5_MANUAL_WALK_GUIDE.md` rebuild — committed at this handoff as `ab44feb`, NOT pushed):**
  - **Re-scoped per user direction (2026-06-20):** the walk is NOT the post-`fcd3496` file delta. It's **everything the scripts can't fully+correctly evaluate**, across all of `src/`, driven by the triage's enforcement diagnostic. Three tiers: **Tier-3 semantic walk** (Grey/Manual/Split/over-claim) · **Tier-2 one-shot mechanical** (decidable but ungated — grep/flip-a-check once) · **Tier-1 script-covered** (cite gate, don't walk — only after confirming it actually runs + fully covers).
  - **Built:** Class 1 (return values, template) · Class 2 (naming + JSF supersession over-claim) · Class 3 (comments/doc-quality smell-cluster) · §LV (live unlogged violations) · §CM (one-shot mechanical checks/conversion moves) · §SC (script-covered, revalidated).
  - **Indexed, build-pending:** Classes 4-14 (assertions, scope/lifetime, pointers/casts, class design, templates, control-flow, concurrency, preprocessor, headers, magic numbers, expressions) — rules + triage refs assigned; build to the Class-1 template next.

  **§7 / live findings SURFACED, not actioned (disposition is Nathan's):** naming over-claim at `standards/CODING_STANDARDS.md:433` ("JSF AV 50-53" — 51/52 mandate all-lowercase, project uses k/g_/camelBack → record the supersession); P10-7 CANARY (`bugprone-unused-return-value.CheckedFunctions` unpopulated → bus/flash/gps/rc_log returns uncovered); live violations JSF-18 `offsetof` (`calibration_data.cpp:106,119`), JSF-27 `#pragma once` (`shared_state.h:16`), JSF-190 28× `continue`, JSF-202 `!=0.0f` (`eskf_runner.cpp:292-295`).

  **NEXT STEPS (recommended order):** (1) run **§CM** first — populating `CheckedFunctions` + enabling `-Wshadow` converts two hand-walks to script-covered and shrinks the walk; (2) walk Classes 1-3 + disposition the 4 **§LV** items (concrete, known sites); (3) build Classes 4-14 (start with **Class 6 / pointers** — carries critical over-claims JSF-175 nullptr + JSF-182 pointer-cast). Then the two still-pending derivative deliverables: condensed agent-facing triage, and the smell-first agent walk guide (external-canon-sourced, findings+draft-remediation, no edits).

  **Not blocked.** Low-priority loose end never done: reconcile the 225 property-rows vs 274 rule-IDs count line-by-line. Untracked `agent-tools/` + `mcps/` dirs in the tree are from the CCSDS detour, unrelated to L2-P5.

  **── SESSION HANDOFF (2026-06-23, Claude Opus 4.8) — the 2026-06-21 "NEXT STEPS" above are SUPERSEDED. Resume from the authoritative plan: `docs/plans/L2P5_WALK_PLAN.md` (read it first). ──**

  Plan 1 was council-reviewed + approved and is now consolidated into that plan doc (permanent). Big shape change since 2026-06-21: the walk became a **field manual** (`L2P5_MANUAL_WALK_GUIDE.md`) + **work plan** (`docs/plans/L2P5_WALK_PLAN.md`) + **itinerary** (`L2P5_WALK_ITINERARY.md`, 185-file coverage map) — deliberately kept distinct (building the guide ≠ using it for the walk). Governing principle re-set to **completeness over brevity** (every file walked, PASS coverage recorded; balloon expected — esp. remediation).

  **DONE this session:**
  - Field manual fully built: agent-blind-spot **lens** (catch what compiles/passes but is improper) + Classes 1–14 + §LV/§CM/§RP/§IT/§SC + 185-file itinerary companion.
  - **§CM measurement (WSL build of `/mnt/c` — Windows-native blocked by cmd-length/quoting; see memory `wsl-on-demand-builds`):** `-Wshadow`=**1** (`rc_os_commands.cpp:1332`), `-Wfloat-equal`=**4** (`eskf_runner` sentinels), confusable=**0**, reserved-id=**4**, param>6=**8**. **No `-Wshadow` balloon.**
  - **Adopted-code policy (3rd-party code) — council-approved, NEVER auto-accept:** `CODING_STANDARDS.md` new "Adopted (Third-Party/Vendored/Toolchain) Code" subsection + `ACCEPTED_STANDARDS_DEVIATIONS.md` register formalized + **TP-2** (`__StackBottom` accepted-via-vendoring: linker-defined, referenced-not-authored). Each vendor flag individually evaluated (sequestered + rule-reason-still-holds); scalable up to formal MISRA GRP/GCS for Nova/RC-Pro/cubesat/Starcom.
  - Reserved-id real finding fixed: `_test_mode_*` renamed (dropped reserved leading-underscore) in `test_mode.cpp`.
  - Triage corrections folded into field manual: `#pragma once`=**3** headers (not 1), `rc_log()` is `void` (not return-checkable), `flash_safe_execute` is **`int`** (return-checkable).

  **── SESSION HANDOFF (2026-06-24, Claude Opus 4.8) — Phase-C dispositions + the ENTIRE code remediation are DONE + committed. ──**

  **DONE since the 2026-06-23 handoff:**
  - **Decision-lock `6f2cf6b`** (pure-doc, host 857/857): all Phase-C dispositions resolved fix-oriented (no pre-judged accepts); JSF-175 `nullptr` supersession + naming supersession relocated into Foundation → Worked-consolidation-decisions; JSF-182 dispositioned per-category against the rule's verbatim Exc 1/2 → two native-code residuals logged (**CAST-1** MPU ptr→int, **CAST-2** `evt_cast` QEvt downcast); JSF-113 non-enforcement recorded; triage over-claim errata at source; Phase-C table refreshed.
  - **Code remediation batch (CHANGELOG `2026-06-24-001`, bench_sim commit):** mechanical (shadow, `#pragma once`×3→`#ifndef`, offsetof×2, `(void*)0`→`nullptr`×24); float== → `CAL_STATUS_WMM_SET` bit + `has_default_location`; JSF-182 (evt_cast×11 + helper, `uintptr_t`, CRC `const void*`, static_cast overlay×8); **`continue`×28 inversions** (council REVERSED its initial "keep" lean → fix-all: a violation is a defect this pass exists to fix, "passes tests" is not a disposition, P10 silence ≠ permission; 12 were in `eskf.cpp` flight math); param refactors (MavCmdParams, calibration→`cal_vec3_t`, CombinatorSpec, FlightEntryLayout); **`-Werror` gate finalized** (shadow/float-equal now fatal). **Verified: host 857/857; WSL firmware clean `-Werror` rebuild, 0 warnings; bit-exact equivalence vs HEAD on both flight-critical math paths (eskf+ud_factor, calibration).**
  - New findings logged: **codegen-drift audit** (High WB above — `mission_profile_data.h` was hand-edited vs its generator in `b1f25ce`) + new **"Auto-Generated Code" rule** in `CODING_STANDARDS.md`; **QP/C-vs-QP/C++ eval** (Medium WB).

  **NEXT (resume order):**
  1. **Phase B — gate-wiring DONE** (commits `d3c8965` complexity, `60ce731` eskf gate-hole, `44b360e` LL43 + coverage-check, + the canary/gate-wiring commit). Highlights: `build/` configured (clang-tidy native Windows, `-p build/`); **P10-7 return-value canary closed via `[[nodiscard]]` contracts** (council JPL/Prof/Cubesat — the return-check contract lives on the i2c_bus/gps API *declarations*, compiler-enforced under `-Werror`; `recover`/`reset` best-effort-unmarked so their 4 calls aren't findings; GPS drain `(void)`'d; `CheckedFunctions`→`flash_safe_execute` only — the rest we own are `[[nodiscard]]`); reserved-id `__Stack*` consolidated into `include/rocketchip/linker_symbols.h` (one documented suppression, not scattered — "a NOLINT lost in the static is worse than a logged deviation"); `scripts/audit/full_tree_clang_tidy.sh` is the single source of truth, called by both the pre-commit hook (Gate 2) and `SESSION_CHECKLIST` item 17 (LL-40 dual-hardcode fixed); `.clang-tidy` adds `ParameterThreshold=6`; planted-proofs + positive-controls all green. **Deferred (low value):** `misc-confusable-identifiers` not gated per-commit — measured 0 findings and O(n²) slow; run in a full milestone clang-tidy audit if ever wanted.
  2. **Phase D — §RP research**: primary-source-verified criteria for the Manual classes (3/7/8/10), fold per-class checklists into the field manual; stash sources in `docs/audits/L2P5_RP_SOURCES_<date>.md`.
  3. **Phase E — wrap**: field-manual `:433`→`:460` citation fix, LL-37 supersession note, plan/itinerary refresh.
  4. **Then** Plan-3 close-out: Cycle-4 remediation doc → close L2-P5 → L2-P10 (CLA-RBM).

  **DEFERRED (future):** full re-audit of existing TP/vendored exceptions vs the new discipline; Plan 2 (agent-facing condensed triage + smell-first agent walk guide); Plan 3 close-out (Cycle-4 remediation doc → close L2-P5 → L2-P10 CLA-RBM).

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

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). Extend when corresponding firmware behavior lands: multi-pending-in-flight, RadioScheduler TX-window arbitration (needed for the sync-gap fix), MAVLink parser state, `station_idle_tick` GPS poll interleave.

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

- **ELRS on RP2350 — research item.** Running ExpressLRS natively on RP2350 with PIO-assisted frequency hopping. Current RFM95W (bare SX1276 on SPI) may be compatible if packet format + hopping schedule can be implemented in firmware. Telstar Booster Pack already describes CRSF/UART to a dedicated ELRS module as the alternative path. Future radio protocol investigation.
- **PIO hardware failure gap — Gemini tier only.** IVP-130 Scenario 5 confirmed: external PIO SM halt is undetectable by firmware (PIO watchdog IRQ only fires from PIO program itself; ARM-side monitoring defeats the independence point). Correct mitigation = physical redundancy (second independent timer on separate MCU). Gemini-tier feature (dual-core carrier board). Accepted gap for Core/Titan.

- **Fault-recovery rework follow-ups (commits `ed7c569` + `8baa18a` landed 2026-05-14/15; see `docs/decisions/FAULT_RECOVERY_2026-05-14.md` for the design):**
  - **PIO beacon + SPI last-gasp combined session** — see dedicated row below.
  - **AON-timer prior-uptime signal** — stubbed to 0 in the anomalous-boot confidence gate. Wiring it requires adding `pico_aon_timer` to target_link_libraries + explicit timer-start at boot. Marginal value (POWMAN reset register already carries the high-confidence signal for brownout; AON timer would corroborate for the watchdog-RSM / hazard-DP / glitch-detect / SWcore-PD reset classes only). Worth picking up if auto-zero-baro suppression false-positive rate during bench testing needs an extra corroborator. Otherwise deferred.

- **PIO beacon + SPI last-gasp beacon (B.5) — combined dedicated future session.** Council round 3 (NASA/JPL + Cubesat, 2026-05-15) unanimously deferred the SPI-based last-gasp beacon (commit (c) of the rework was scoped for this and *not* implemented). User direction 2026-05-15: "merge with the future PIO beacon" — the two questions evaluate together rather than pre-committing to an interface (compile-time `ROCKETCHIP_LAST_GASP_BEACON` + `radio_init_confirmed` semantics) that would constrain the PIO design choice. Reasons for deferral, fully captured in plan B.5 + council-round-3 transcript at `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a355e8caee0717e0b.md`:
  - **SPI peripheral state corruption** if fault occurred mid-byte/mid-burst — recovery is NOT bounded-cost (FIFO drain + CS deassert via GPIO function override + SX1276 hardware reset pulse + full cold re-init; each step has its own hang potential).
  - **`#ifdef`-scaffolding-rot pattern** per LL Entry 36 — code-shaped-but-never-exercised artifact creates false-confidence for future contributors.
  - **JPL precedent: always architect beacons as independent silicon** (Cassini LGA+USO, SMAP transponder). Cubesats that share the radio rely on modem-level autonomous beacon modes (e.g., FSK Beacon Mode); SX1276 LoRa lacks this in long-range mode.
  - **PIO beacon is the architecturally-correct answer** (Pico SDK has working PIO-SPI in 2-3 instructions per primary-source verification 2026-05-14; DIO0 wired to GPIO 6 on Adafruit Feather RP2350; DIO5 not currently routed but solder-jumperable on the RFM95W FeatherWing #3231).
  - **In-flight ARM-dead beacon-coverage gap is the trade** — accepted as a known gap for Core/Titan single-MCU pending this session.

  Scope of the combined session: (1) evaluate whether SPI-from-fault-handler is ever the right stop-gap given the failure-mode inventory; (2) design the PIO-driven beacon program (target: PIO0 or PIO1 — PIO2 already shared between watchdog SM0 + backup-timer SM1-3); (3) decide whether the design requires soldering the DIO5 jumper on the FeatherWing; (4) bench-verify on a known-faulted chip state if any stop-gap is in scope. Likely outputs a dedicated decision doc under `docs/decisions/`. User direction will determine sequencing relative to other open work.
- **RP2350B/Fruit Jam persistent bus-corruption hypothesis.** User hunch 2026-04-17: one boot during the Fruit Jam GPS debug had a transition not fully explained by the cable theory alone. Investigate whether RP2350B exhibits bus-corruption state that survives power cycles. Low priority — may be a dead end, keep passive.

- **PIO I²C master reference implementation available (Flipper One MCU firmware).** If station I²C ever needs to leave DW_apb_i2c hardware (candidate direction for the Fruit Jam GPS cold-boot intermittency tracked in `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`), Flipper Devices published their RP2350 co-processor firmware on 2026-05-21 with a working PIO I²C master driver at `lib/drivers/i2c_master_pio/pio_i2c.c` in https://github.com/flipperdevices/flipperone-mcu-firmware. Pairs `pio_claim_free_sm_and_add_program_for_gpio_range` at init with `pio_remove_program_and_unclaim_sm` at deinit (LL Entry 42 discipline as a working pattern). Mid-cycle error recovery uses `pio_sm_drain_tx_fifo` + `pio_sm_exec` (jump-to-wrap) + `pio_interrupt_clear` — never touches program memory. They also use the acquire/release pad-mux discipline (LL Entry 28) as a first-class per-handle pattern (`Activate`/`Deactivate` callbacks in `targets/f100/furi_hal/furi_hal_i2c_config.c`: `i2c_init` + pad config on activate, `i2c_deinit` + pads-to-input on deactivate). Useful as **reading material** before any PIO-I²C migration evaluation; not actionable today. Same SDK (2.2.0) and toolchain (14_2_Rel1) as us. License check needed before any code import.

## Deferred (near-term, post-Stage 15)

- **Battery ADC monitoring.** Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS command authentication.** Telecommand auth for Rocket profile.

## Far-future

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export — all tracked in `docs/PROJECT_STATUS.md` future features.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** — AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 — evaluate justified vs. bloat).

**Stage 16: Field Tuning** — All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** — IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.
