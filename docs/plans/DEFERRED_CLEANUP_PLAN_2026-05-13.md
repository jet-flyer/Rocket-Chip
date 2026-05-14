# Deferred-Cleanup Cycle DC-2026-05-13 — Plan

**Status:** APPROVED WITH AMENDMENTS (council 2026-05-13 — main 4 personas, CONSENSUS, no tie-break required). Ready to execute.
**Cycle tag:** `DC-2026-05-13` (every commit in this cycle is prefixed `[DC-2026-05-13]`).
**Authored:** 2026-05-13 (post-2026-05-13 audit cycle close + audit-infrastructure follow-up).
**Procedure framing:** `standards/AUDIT_GUIDANCE.md` Appendix C 4-category ordering — **focused remediation cycle** per AUDIT_GUIDANCE scope-decision-table row 6, NOT a new master audit.
**Hardware available:** debug probe + vehicle Feather + station Fruit Jam.
**Expected total wall-clock budget:** 4–5 hours focused work across 2 active sessions (Session 1 + Session 3). (Original 8–10 hr estimate reduced over 2026-05-13: R-24 pulled out and merged into R-25-exec; CLA-RBM pulled out as its own dedicated session; **R-22 pulled out and merged into R-25-exec after 4 iterations + council redesign hit a flight-tier-firmware design wall — see Session 2 post-mortem below**.)

**Sessions explicitly out of this cycle (each its own dedicated session):**
- R-5 ETL vendoring + stdio removal (per `STDIO_REPLACEMENT_PLAN.md`).
- In-flight fault recovery architecture (per AGENT_WHITEBOARD).
- **L2-P10 / CLA-RBM re-collection** — pulled out 2026-05-13 (post-council) per user direction: the cycle-budget audit deserves a focused session, not parallel work during Session 2. Triggers organically at 90-day staleness (~2026-06-06) or sooner if scheduled.

---

## Purpose

The 2026-05-13 master audit cycle CLOSED with three findings DEFERRED to a follow-up tooling session (now REMEDIATED — F-001/002/003 shipped earlier today). That leaves the broader `docs/PROBLEM_REPORTS.md` queue, much of which has accumulated across the 2026-05-07 + 2026-05-13 cycles. This cycle organizes the remaining items per Appendix C and executes them session-by-session, with closure tracked as a focused remediation cycle.

The cycle was scoped via council review on 2026-05-13. Council verdict + the 5 question-answers + 5 independent amendments are recorded inline below. The verdict was CONSENSUS APPROVE WITH AMENDMENTS — no tie-break required.

---

## State inventory

### Bucket A — verified rows awaiting Phase 8 wrap regression (16 rows)

These are REMEDIATE rows where the fix has landed in a commit and per-commit verification passed. They've been waiting on the audit-suite regression to advance from `verified` → `closed`.

**The audit-suite regression already happened** at 2026-05-13 via Grok's Tier 5/6/7 independent verification (CHANGELOG `2026-05-13-004`). Per Appendix C.5, the cycle-close regression is the final gate. These can be advanced to `closed` and archived.

| ID | Title | Verification observed |
|---|---|---|
| R-1 | Bound Core 1 boot-wait loop | commit `273a2f9` + Grok Tier 5 PASS |
| R-3 | Capture-state-then-reset hardfault handler | commit `e4d222a` + Grok Tier 5 PASS |
| R-4 | MPU-config failure handling (AP=0b10 + MEMFAULTENA) | folded into R-3, surfaced-bug rule |
| R-6c | FP-1 template-dispatch | bench_sim 2/2 + 794 host tests |
| R-7 | Holzmann inverted-rule exemption doc | doc edit verified |
| R-9c | Log-on-change emit in health_monitor | commit `9a47ba6` + Tier 5.4 |
| R-11 | SPIN model: flash_safe_execute protocol | SPIN_OK_31 + Grok Tier 5 |
| R-12 | SPIN model: cross-core boot handshake | SPIN_OK_31 + Grok Tier 5 |
| R-13 | SESSION_CHECKLIST trigger-map row for SPIN | doc edit verified |
| R-15 | i2c_bus_reset after CLI log-flush + erase | code edit + R-17 wraps |
| R-16 | Systematic LL-entry freshness audit | audit table doc'd |
| R-17 | Cooperative pause around runtime flash_safe_execute | SPIN-verified protocol |
| R-18 | Clean up dead cal_pre_hook function-pointer table | tree clean |
| R-19 | Unconditional multicore_reset_core1 before launch | AIRCR + warm-reboot HW PASS |
| R-26 | Fix RP2350 datasheet citation §1.4.3 → §14.9.1 | comment-only edit |
| P8-FMEA-Pyro | Pyro intent/arm/disarm FMEA sub-check | F1-F9 review doc'd |

All 16 rows move to `closed` + archive in C4-1 of Session 1.

### Bucket B — DEFER rows with named blockers

These cannot close in isolation; each depends on a separate session.

| ID | Title | Blocker | Owner session |
|---|---|---|---|
| R-2 | GPS PMTK snprintf → const arrays | Absorbed into R-5 | R-5 ETL session |
| R-5 | Full `<stdio.h>` removal from src/ | Vendoring ETL | R-5 dedicated session |
| R-20 | kCrashReasonCore1BootWait AIRCR half-broken | R-3 + R-21 architecture | In-flight fault recovery session |
| R-21 | No automatic chip-reset on PIO watchdog timeout | R-19 + R-20 | In-flight fault recovery session |
| R-23 | Vehicle bench-tier HardFault on cold boot | R-25 (may subsume) | Bench-tier deprecation session |
| R-25 | Evaluate deprecating bench tier | R-23 (subsumed) | Bench-tier deprecation session |

### Bucket C — DEFER rows that are clear standalone work

| ID | Title | Estimated effort | HW gate? |
|---|---|---|---|
| R-10b | Stack-usage SDK coverage gap | ~1 hr (analysis + decision) | No |
| R-22 | Warm-reboot audit script | ~1-2 hr | Yes (probe AIRCR + picotool variants) |
| R-24 | Extend 4-tier build parity to flash+verify-banner | ~2 hr | Yes (probe + 4 boards) |
| L2-P5 | Tier 3 standards walk (JSF AV / JPL C / project-D / agent-E) | ~2-4 hr | No (dated audit) |
| L2-P6 | DEV_CODE re-audit | ~30 min | No |
| L2-P7 | VERSION_STRING re-audit | ~15 min | No |
| L2-P8 | AO_COMMANDMENTS re-audit | ~30 min | No |
| L2-P9 | TOOLCHAIN_VERSION re-audit | ~15 min (script exists now — F-003) | No |
| L2-P10 | CLA-RBM re-collection | — | **PULLED OUT** to own dedicated session 2026-05-13 |
| L2-P11 | Inventory-completeness check (meta-audit) | ~1 hr | No |

**Note:** L2-P5 through L2-P11 are AUDIT-CYCLE catchup items, not project-finding items. The 2026-05-13 audit cycle's procedure refactor (Tier 3.6, 3.7, 4.4, 4.5, 4.6) already specifies where these run in a future cycle. **Many are auto-absorbed** when the next master audit cycle runs.

### Bucket D — AGENT_WHITEBOARD policy items (not in PROBLEM_REPORTS yet)

| Item | Source | Resolution path |
|---|---|---|
| L2-P2/P3/P4 | 2026-05-07 cycle audit-policy doc edits (sampling, citations, scope language) | Roll into Tier 3.8 procedure edits at next cycle; doc-only |
| Station SPIN model extensions | WB | Trigger-driven when corresponding firmware lands |
| In-flight fault recovery architecture | WB (deep architectural session) | Owns R-20, R-21, R-23 indirectly |
| RP2350B persistent bus-corruption hypothesis | WB (low priority — likely dead end) | Watch for evidence |

---

## Sequencing (Appendix C 4-category)

### Category 1 — Gate Integrity

**This category is empty for this cleanup cycle.** Today's audit-infrastructure follow-up session (F-001/002/003) was itself Category 1 work for the 2026-05-13 cycle. No further gate-integrity items remain open.

### Category 2 — Shared Foundations

Items that introduce or refactor a mechanism that multiple later items reuse.

**SF-1: R-25 bench-tier deprecation evaluation**

Why Category 2: subsumes R-23, owns the decision tree for `src/dev/*` modules + the 40 `#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` gates + 12 host-side scripts depending on bench-tier symbols. Whichever direction R-25 chooses (A: flight-tier runtime test-mode flag; B: patch-based testing; C: tightened bench tier) is a foundation that R-23 + L2-P6 (DEV_CODE re-audit) inherit from.

Scope of the evaluation session:
- Inventory 23 distinct citations from audit § L2.6f (already done in dated report).
- Per-script migration plan for 12 host scripts.
- `replay_inject.cpp`'s fate (largest surface).
- Cleanup plan for 9 archived host scripts.
- **Deliverable:** decision doc `docs/decisions/BENCH_TIER_DEPRECATION_<approach>_2026-MM-DD.md` with council review.
- **HW gate:** probe-attached boot-verify on bench-tier ELF if approach C; no HW if approach B.

**SF-2: ~~R-5 ETL vendoring + stdio removal (subsumes R-2)~~ — Dedicated session, NOT in this cleanup.**

R-5 ETL vendoring + stdio removal stays as its own dedicated session per user direction 2026-05-13 and `docs/decisions/STDIO_REPLACEMENT_PLAN.md`. It would otherwise be a large Category 2 root within this cleanup, but the scope (vendoring infrastructure + 18-file migration + tier-by-tier HW gate) is appropriately bigger than the rest of this cleanup combined. R-2 stays absorbed into R-5.

**Inputs from this cleanup that R-5's dedicated session will consume when it runs:**
- C4-1 archive sweep (so R-5 doesn't have to rationalize about already-fixed gate work).
- C4-2 / L2-P9 toolchain-drift baseline (R-5's commit gates may depend on ETL vendor source version pinning).
- R-22 / R-24 warm-reboot + boot-parity audit scripts (R-5 changes touch USB CDC printf paths — warm-reboot is the relevant audit for those).

**SF-3: ~~In-flight fault recovery architecture~~ — Dedicated session, NOT in this cleanup.**

Per user direction 2026-05-13, this stays as its own session. R-20, R-21, R-23 indirectly depend on the architecture decision; they remain DEFERRED with the architecture session as named owner.

**Inputs from this cleanup that the architecture session will consume:**
- R-22 / R-24 audit scripts (the architecture decision needs warm-reboot + boot-parity audits as its verification baseline).
- C4-2 / L2-P9 toolchain-drift baseline (the architecture session may pin SDK / picotool to specific versions for its safe-mode reset primitive).
- C4-1 archive sweep (so R-19's "verified" status is finalized as `closed` — load-bearing input to the architecture session since R-19 partially unblocks R-21 SDK watchdog re-evaluation).

### Category 3 — Behavior Changes

Items that change observable program behavior. Ordered by mission-phase exposure × intrinsic severity.

(No active Category 3 items in the deferred queue today — R-1/R-3/R-4/R-9c/R-15/R-17/R-19 were Category 3 in their original cycles and are now `verified`.)

### Category 4 — Cleanup & Documentation

**C4-1: Advance the 16 "verified-awaiting-regression" rows to `closed`** (Bucket A)

Why Cat 4: pure documentation work. The fixes already shipped. The audit-suite regression already happened via Grok's Tier 5/6/7. This is bookkeeping that catches the queue up.

Scope:
- Walk `docs/PROBLEM_REPORTS.md` rows R-1, R-3, R-4, R-6c, R-7, R-9c, R-11, R-12, R-13, R-15, R-16, R-17, R-18, R-19, R-26, P8-FMEA-Pyro.
- For each: change state `verified` → `closed`, move row to `## Archived (Closed) Problem Reports` section with closure-commit SHA + verification gate observed.
- **No HW gate** — closure is bookkeeping over already-verified work.
- **Effort:** ~30-60 min.

**C4-2: L2-P5..P11 audit catchup items** (10 rows)

Why Cat 4: doc-only audit work. The 2026-05-13 procedure refactor moved most of these into the procedure structure (Tier 3.6, 3.7, 4.4, 4.5, 4.6). They'll be auto-walked on the next master audit cycle. **Recommendation: do not pre-walk in this cleanup cycle.** Save the energy for the next milestone audit.

Exception: **L2-P9 (TOOLCHAIN_VERSION re-audit) ~15 min** is now trivial — F-003 `check_toolchain_drift.py` shipped today. Running it produces the dated audit content. Worth doing as a 15-minute side task to demonstrate the new mechanism on a real walk.

**C4-3: R-10b stack-usage SDK coverage gap re-eval**

Why Cat 4: tooling gap, not safety gap. Project-side max stack 200 bytes vs 1024 threshold — well within margin. Schedule re-eval for next milestone close per the row's existing rationale.

**C4-4: R-22 warm-reboot audit script + R-24 boot-parity extension**

Why Cat 4 (despite needing HW): pure-additive audit infrastructure. ~1-2 hr each, sequential since both touch warm-reboot HW paths.

- R-22 first (standalone script). 4 gates: G-W1 AIRCR via probe + G-W2/G-W3/G-W4 picotool variants.
- R-24 second (extends `verify_build_parity.sh`).
- **HW gate:** probe + USB CDC enumerate + banner read for each gate.
- **Deliverable:** wired into `AUDIT_GUIDANCE.md` Tier 5.2 (Stack/Memory/Errata) or new Tier 2.7.

**C4-5: AGENT_WHITEBOARD policy items L2-P2/P3/P4**

Pure doc edits (sampling policy, citation inventory references, scope language) to `AUDIT_GUIDANCE.md` / `STANDARDS_AUDIT.md`. ~30 min. Could ride with C4-2's L2-P9 work as the same kind of audit-procedure follow-through.

---

## Session ordering (council-amended)

Per Appendix C category-order with council amendments. **This cleanup cycle covers Sessions 1-3 only.** Sessions 4 and 5 (R-5 ETL, in-flight fault recovery) stay as dedicated separate sessions per user direction 2026-05-13.

### Session 1 — Cat 4 closeout [~2-3 hr] | NO HW

**Open with A1 self-check:** Run `scripts/audit/pre_commit_fixture_test.py`, `scripts/audit/list_bench_sim_pass_tokens.py`, `scripts/audit/check_toolchain_drift.py` against current `origin/main`. Confirm new instruments produce honest output on real workload (not just synthetic-fixture inputs). **If any instrument fails, halt cycle.** ~10 min total. Defense-in-depth per LL Entry 36.

**Commit cadence: one commit per work item, in order:**

1. **C4-2 (L2-P9)** — Run F-003 output, package as `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-05-13.md`. *Commit*: `[DC-2026-05-13] L2-P9 REMEDIATED: toolchain version audit 2026-05-13`.
2. **C4-5** — L2-P2/P3/P4 audit-policy doc edits (sampling, citations, scope language) to `AUDIT_GUIDANCE.md` / `STANDARDS_AUDIT.md`. *Commit*: `[DC-2026-05-13] L2-P2/P3/P4 REMEDIATED: audit-policy doc edits`.
3. **C4-3** — R-10b decision-of-record in PROBLEM_REPORTS.md (re-eval deferred to next milestone close per existing rationale; row stays `analyzed` / DEFER). *Commit*: `[DC-2026-05-13] R-10b decision recorded: re-eval at next milestone`.
4. **C4-1** — Archive 16 verified rows in PROBLEM_REPORTS.md → `closed`. Each archive row cites Grok verification SHA (`2c68512` / `1a32103`) as audit-suite regression that closed it. *Commit*: `[DC-2026-05-13] C4-1: archive 16 verified rows (Grok Tier 5/6/7 regression)`.
5. **End-of-session sanity check:** host ctest + bench_sim 2/2 + SPIN_OK_31 — confirm bookkeeping commits didn't break anything. ~5 min. (No commit; failure halts cycle.)

Single push at end. CHANGELOG entry for the session.

### Session 2 — DEFERRED ENTIRELY to R-25-exec (post-mortem)

**Original Session 2 scope (executed but did not close):** R-22 (warm-reboot audit script) + R-24 (boot-parity extension, deferred mid-session).

**What happened:** Session 2 ran 4 iterations of `scripts/warm_reboot_audit.py` against the vehicle Feather:

1. **Iteration 1** — initial draft, blind-sleep approach. 1/4 PASS; G-W3 picotool burst caused USB CDC re-enumeration loops requiring physical replug.
2. **Iteration 2** — added `--chip-serial` plumbing + `_wait_for_device_serial()` polling. Python wedged in `serial.Serial()` C-library code on Windows; required `taskkill`.
3. **Iteration 3** — same wedge; same fix didn't help.
4. **Council redesign (NASA/JPL + Prof + ArduPilot + Cubesat)** — unanimous APPROVE WITH AMENDMENTS. Plan at `C:\Users\pow-w\.claude\plans\snoopy-wibbling-noodle.md`. Switch to `peek_banner()` from `_rc_test_common.py`, add Core-1 IMU/baro counter-increment positive control for G-W1/G-W4 per HW_GATE_DISCIPLINE Rule 1. R-24 pulled out and merged into R-25-exec (tier count depends on bench-tier deprecation outcome).
5. **Iteration 4** (post-redesign) — `peek_banner` integration worked; baseline banner captured cleanly. **Hit a fundamental design wall:** the council-required Core-1 counter check (via `q→s` Sensors / `q→b` Hardware Status in the Debug submenu) is **bench-tier-only**. The flight-tier firmware currently on the vehicle has no main-menu equivalent for IMU/baro read counters — `s` is unbound on main menu, and main-menu `b` is Beacon (Stage L 2026-04-18 reassignment, surfaced via R-27 doc drift). Flashing bench-tier firmware to verify R-22 is operationally heavy and conflicts with R-25-eval's pending bench-tier deprecation direction.

**Decision (user 2026-05-13):** defer R-22 entirely to R-25-exec session. Build R-22 once against the final tier model — banner-only gates if approach B (no bench tier), or full counter-increment gates if approach C (bench tier retained). The 4-iteration redesign work (plan + council verdict + research memos) is preserved at `snoopy-wibbling-noodle.md` and inherited by R-25-exec.

**Useful Session 2 deliverables (committed despite R-22 deferral):**
- **R-27** (commit `48b3f59`) — `docs/ROCKETCHIP_OS.md` main-menu key table drift fix. Surfaced during R-22 iteration 4 when 'b' main-menu key activated Beacon instead of Hardware Status. Doc edit verified against `print_help_menu()` source of truth. Same drift class as R-26.

**R-22 + R-24 status:** both `analyzed / DEFER` in PROBLEM_REPORTS with owner = R-25-exec.

### Session 3 — R-25-eval bench-tier deprecation evaluation [~2 hr]

**Split per council Q3 amendment:** R-25 splits into R-25-eval (this session) and R-25-exec (separate session, tracked as next-session PR).

1. Council-review evaluation session (likely Main 4 + Cubesat Startup Engineer aux for cost-pressure framing).
2. Pick approach A (flight-tier runtime test-mode flag) / B (patch-based testing) / C (tightened bench tier).
3. Write decision doc `docs/decisions/BENCH_TIER_DEPRECATION_<approach>_2026-MM-DD.md`.
4. **R-25-eval CLOSES when:** decision doc committed AND R-25-exec PR added to PROBLEM_REPORTS.md as next-session work with named owner (per council A5 amendment — closure requires naming the follow-on).
5. **Downstream noted:** if approach B chosen, L2-P6 (DEV_CODE re-audit) auto-resolves (dev tier goes away). Plan must record this transitive closure.
6. *Commit*: `[DC-2026-05-13] R-25-eval CLOSED: bench-tier deprecation decision <approach>`.

### Cycle close — after Session 3

1. **Final audit-suite regression** per Appendix C.5: clang-tidy, cppcheck, lizard, coverage, host ctest, SPIN, bench_sim 2/2 + station_bench_sim 3/3.
2. Write `docs/audits/DEFERRED_CLEANUP_CYCLE_2026-05-13.md` — cycle's dated audit artifact (cycle scope, commits, dispositions, regression results).
3. CHANGELOG rollup entry referencing every commit in the cycle.
4. PROBLEM_REPORTS.md Archive section gains the cycle row at closure.
5. Push.

### Other sessions — NOT in this cycle

- **R-5 ETL vendoring + stdio removal** — dedicated session per `STDIO_REPLACEMENT_PLAN.md`. Consumes this cycle's outputs (C4-1 archive, C4-2 toolchain-drift baseline, R-22/R-24 audit scripts).
- **In-flight fault recovery architecture** — dedicated session per AGENT_WHITEBOARD. Consumes same inputs plus R-19 closed status (load-bearing for SDK watchdog re-evaluation).
- **L2-P10 / CLA-RBM re-collection** — dedicated cycle-budget audit session. Per-tick wall time + jitter + CPU utilization + stack high-water-mark + execution paths + state transitions. Last data 2026-03-08. Stages L / T / 16C added 3+ new AOs since.
- **R-25-exec** — execution of the deprecation approach chosen in Session 3. Owns R-23 closure (approach B: removes bench tier; approach C: fixes INVPC). Owns L2-P6 closure (if approach B). Tracked as next-session PR per council A5.

---

## Council verdict (2026-05-13)

**CONSENSUS APPROVE WITH AMENDMENTS.** 4 main personas (ArduPilot Core Contributor, Retired NASA/JPL Avionics Lead, Embedded Systems Professor, Senior Aerospace Student). 4 rounds + convergence check. No tie-break required.

### Answers to the 5 plan questions

**Q1 (C4-1 within Appendix C.5):** YES, archival fits C.5's intent. Chronology inversion (Grok regression before cat-4 cleanup) is acceptable for pure-bookkeeping commits because the risk that bookkeeping invalidates Grok's verification is essentially zero. **Amendment (applied):** every C4-1 archive row cites Grok's specific verification SHAs (`2c68512` / `1a32103`). End-of-Session-1 sanity check (host ctest + bench_sim 2/2 + SPIN_OK_31) runs to confirm bookkeeping commits didn't break anything.

**Q2 (R-22 + R-24 timing):** Session 2 placement correct. **Amendment (applied):** re-labeled in plan from "Cat 4 (pure-additive)" to "Cat-1-equivalent audit-infrastructure" so future readers don't treat them as optional cleanup. Their dependency on R-25 (Session 3 needs warm-reboot + boot-parity audits as verification baseline) is the actual ordering constraint.

**Q3 (R-25 timing):** **SPLIT.** R-25 becomes R-25-eval (this cycle, Session 3, ~2 hr) and R-25-exec (separate session, tracked as next-session PR). **Amendment (applied):** R-25-eval closure requires naming the follow-on session and entering R-25-exec into PROBLEM_REPORTS.md.

**Q4 (CLA-RBM):** Council recommended PRE-TRIGGER in parallel during Session 2. **Superseded 2026-05-13 by user direction:** CLA-RBM is the cycle-budget audit and deserves its own dedicated session rather than parallel background work. Removed from this cleanup cycle's scope; tracked as a separate-session item in the header's "Sessions explicitly out of this cycle" list. The 90-day staleness threshold (~2026-06-06) is the organic trigger if not scheduled sooner.

**Q5 (cycle framing):** **FOCUSED REMEDIATION CYCLE** per AUDIT_GUIDANCE scope-decision-table row 6. **Amendment (applied):** named `DC-2026-05-13`. Commits prefixed `[DC-2026-05-13]`. Closure produces dated artifact at `docs/audits/DEFERRED_CLEANUP_CYCLE_2026-05-13.md` and CHANGELOG rollup. PROBLEM_REPORTS.md Archive section gains cycle row at closure.

### 5 independent amendments raised by personas (all applied)

- **A1** (Prof) — Session 1 opens with self-check of F-001/002/003 against current `origin/main`. ~10 min. Defense-in-depth per LL Entry 36. If any instrument fails, halt cycle.
- **A2** (Prof) — Session 1 commit cadence is one commit per item (4 commits total) plus final session-end push + CHANGELOG. Cycle tag in every commit message.
- **A3** (AP) — Plan must state expected total wall-clock budget (8–10 hours focused work + async CLA-RBM soak). Recorded in header.
- **A4** (NASA + Prof, raised in Q4) — Originally added CLA-RBM async collection to Session 2. **Superseded 2026-05-13 by user direction:** CLA-RBM pulled out as its own dedicated session (see Q4 above). A4 amendment no longer applies to this cycle.
- **A5** (Student) — Session 3 (R-25-eval) deliverable list notes downstream effect on L2-P6: if approach B chosen, L2-P6 auto-resolves (dev tier goes away). Recorded in Session 3 step 5.

### No-change items (council unanimous)

- Sessions 4 + 5 (R-5 ETL + in-flight fault recovery) staying out of scope — correct, per user direction.
- L2-P5 / L2-P7 / L2-P8 / L2-P11 deferred to next master audit cycle — correct.
- L2-P9 pre-walked in Session 1 via F-003 — correct.
- C4-3 (R-10b decision noted in PROBLEM_REPORTS, not actioned) — correct.

### Persona sign-offs

- **ArduPilot Core Contributor:** APPROVE WITH AMENDMENTS. Splitting R-25 is the most important change.
- **Retired NASA/JPL Avionics Lead:** APPROVE WITH AMENDMENTS. C.5 chronology bend acceptable for bookkeeping; cycle naming + audit-trail discipline essential.
- **Embedded Systems Professor:** APPROVE WITH AMENDMENTS. Cleanup-cycle framing per row 6 theoretically correct; F-001/002/003 self-check is LL-Entry-36 discipline applied to its own remediation.
- **Senior Aerospace Student:** APPROVE WITH AMENDMENTS. Future-readability matters most; cycle naming + tag-prefixed commits make this work as a referenced artifact.

---

## Out of scope (deliberate)

- **No flight tests.** Stage 17 is its own work, not part of cleanup.
- **No CCSDS TC-Layer / COP-1 rework.** Post-Stage-17 per unanimous council.
- **No new audit cycle.** This is cleanup of the existing queue; the next master audit triggers at next milestone close.
- **No Stage 17 tapered-buildup execution.** That awaits airframe + launch window.

---

## Verification cadence (per Appendix C.5)

- **Per-commit local:** host ctest + the change's own positive-control signal per HW_GATE_DISCIPLINE Rule 3.
- **Per session-end:** push to origin/main + CHANGELOG entry + WB row erase if applicable.
- **Per category transition:** audit-suite regression — same scripted check suite as the 2026-05-13 cycle (clang-tidy, cppcheck, lizard, coverage, host ctest, SPIN, bench_sim).
- **Cycle close:** when the cleanup plan's queue is empty, sign as `closed` in PROBLEM_REPORTS.md and write a CHANGELOG rollup entry naming this plan's commits.

---

*End of draft. Pending council review.*
