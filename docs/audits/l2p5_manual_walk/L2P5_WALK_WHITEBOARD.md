# L2-P5 Walk Whiteboard — temporary; must be EMPTY at walk close

**Walk-tier checkpoints (e.g. Tier 1 done 2026-08-05):** this board is a **soft**
check — skim for rows that need **action now** (process rules mid-walk, blocking
confusion) vs **end-of-walk** disposition only. Do **not** require empty mid-walk;
do **not** treat a tier tick as product CHANGELOG / PROJECT_STATUS milestone.

**Companion to** the walk pack (`L2P5_MANUAL_WALK_GUIDE.md` / `L2P5_WALK_ITINERARY.md` /
`L2P5_WALK_PLAN.md` / optional `L2P5_CONTRACT_SURFACE_HELPER.md` for thin hub files).
Scratch surface for items that surface during the L2-P5 manual walk but
have **no home yet** — the walk's own working memory, not a record.

> **Treat this like the project whiteboard (`AGENT_WHITEBOARD.md`): erase resolved rows, don't
> mark them done.** A row's continued presence is the signal that it still needs attention.
> Unlike the project whiteboard, this one has a **hard end-of-life** — see Lifecycle.

---

## Lifecycle

**This file must be empty when the walk closes.** A non-empty whiteboard at Plan-3 close-out means
either (a) an item never got dispositioned, or (b) an item belongs in a permanent doc and was never
moved there. Both block closure. Empty-at-close sits alongside the itinerary's
100%-ticked requirement as a coverage condition — the itinerary proves every file was *seen*, this
proves every out-of-place item was *landed*.

When the walk closes and this file is empty, delete it (its history is in git).

---

## What goes here

Only things whose home **doesn't exist yet or is out of reach right now**:

- **Pre-walk observations** — surfaced by incidental reading before the walk reached that file
- **Instrumentation defects** — the guide / itinerary / triage itself is wrong or misdirecting, found mid-walk
- **Cross-file items** — noticed while walking file A, belongs to file B which is still ahead
- **Questions awaiting owner disposition** — where continuing the walk doesn't depend on the answer
- **Deferred-venue items** — real, but the right place to resolve them is another workstream (QP/QF rework, Starcom, §CM patch pass)

## What does NOT go here

Anything with a home. Send it there instead:

| Item | Home |
|---|---|
| A file's verdict (PASS / FAIL / PARTIAL) | itinerary checkbox + one-line note |
| A `FAIL` / `PARTIAL` on a file already walked | that class's findings table in the field manual |
| A mechanically-decidable check | §CM backlog (`AGENT_WHITEBOARD.md` L2-P5 §CM row) |
| A fix decision | new `R-NN` in `docs/PROBLEM_REPORTS.md` |
| An accept decision | `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`, with sign-off |
| A durable debugging lesson | `docs/agents/LESSONS_LEARNED.md` |

**Rule of thumb: if it has a home, it goes home.**

## Row format

Every row carries a **disposition target** — the doc it lands in when resolved. That is what makes
"fully resolved by walk close" checkable rather than aspirational; a row with no target is a row
that will still be here at close-out.

```
### W-N — <one-line title>
**Surfaced:** <date> · <how — pre-walk reading / walking file X / etc.>
**What:** <the item, with file:line citations>
**Disposition target:** <where this lands when resolved>
**Blocking?** <no — walk continues / yes — walk stalls until answered>
```

## Mirror-walk note

If an independent second pass is run over the same tree, **this file is excluded from its inputs**,
along with the field manual's populated findings tables. The second pass's value is independent
detection; feeding it the first pass's conclusions converts it into verification of a note. Compare
after both are complete, don't cross-contaminate during.

---

## Rows

### W-1 — P10-9 function-pointer ban is classified "policed by deviation log," and the log says clean

**Surfaced:** 2026-07-30 · pre-walk reading (CppCoreGuidelines F.1 lambda question led to the
callback wiring in `ao_flight_director.cpp`, then to a full function-pointer sweep)

**What:** `RULE_VERIFIABILITY_TRIAGE.md:60` classifies P10-9 Property C (function-pointer ban) as
**Det-by-reference** — "No tidy check enforces … the fn-ptr ban directly; policed by deviation log
+ review" — and row 29's enforcement column reads *"none — FP-1 resolved."* The deviation log's
Active "Function Pointer Usage (P10 Rule 9)" section reads *"All entries resolved as of
2026-05-13."*

`src/` currently contains **18 function-pointer declaration sites across 7 files**:

*Typedef'd (JSF-176 satisfied):*
- `safety/test_mode.h:90` — `FlightPhaseAccessor`
- `fusion/eskf_runner.h:61` — `EskfEventLogFn`
- `cli/rc_os.h:112, 149, 158` — `rc_os_read_accel_fn`, `rc_os_read_mag_fn`, `rc_os_reset_mag_staleness_fn`

*Raw / untypedef'd:*
- `shared_state.cpp:31-33` — 3 **mutable global** GPS hooks (`g_gpsFnUpdate`, `g_gpsFnGetData`, `g_gpsFnHasFix`)
- `flight_director/flight_director.h:85-89` — 5 struct members (FD action callbacks)
- `flight_director/action_executor.h:115, 118` — 2 struct members (`set_led`, `log_pyro`)
- `logging/flash_flush.{h,cpp}` — 3 functions take a raw `void (*kick_watchdog)()` parameter

**Why it's a walk-instrumentation item, not just a code finding:** the Det-by-reference
classification tells the walk to decide this rule by consulting the log rather than by hand. With
the log reporting zero active entries, the procedure yields PASS on all 7 files without anyone
looking. Same shape as LL 43 ("clean from a static gate is negative evidence") and the LL 36/40
gate-scope family; the triage's own §7 severity model treats over-claiming tool-claims as critical.
Correcting the classification should precede the walk reaching these files.

Note also row 176: JSF-176 (typedef function-pointer decls) is marked *"moot under fn-ptr ban."* If
the P10-9 disposition lands on *accept*, JSF-176 un-moots and the 13 raw declarations become a
second, separate finding.

**Confirming datapoint (not a defect):** `calibration/lm_solver.h` has no function pointer left —
FP-1 was genuinely resolved via template dispatch, exactly as its remediation path stated. The
remedy pattern works and has in-tree precedent (`include/rocketchip/notify_backend.h:6-10`:
compile-time dispatch, cited to P10-9).

**Sweep caveats:** regex over `src/` + `include/` only. Will not catch member-function pointers,
arrays of function pointers declared through an already-found typedef, or `std::add_pointer`-style
spellings. `test/`, `ground_station/`, `tools/` not swept. Checked the triage and the deviation log
for prior disposition — **not** the `MASTER_STANDARDS_AUDIT_2026-05-07*` files, which also hit on
P10-9 and may carry one.

**Disposition target:** triage §7.3 (live unlogged violation) + row 29/60 enforcement correction;
then per-site → Class 6 findings table as the walk reaches each file; then fix (`R-NN`) or accept
(deviation log, with sign-off). The FD + `action_executor` callbacks may reasonably defer their
*disposition* to the QP/QF rework (they exist because the HSM is C — the QP/C-vs-QP/C++ evaluation
governs whether they become virtual dispatch or templates). `shared_state.cpp`'s globals and
`flash_flush`'s `kick_watchdog` are unrelated to QP and do not depend on it.

**Blocking?** No — walk continues. But the triage correction should land before the walk reaches
Tier 1 `safety/test_mode.h` or Tier 3 `shared_state.cpp` / `flight_director/`.

---

### W-2 — Shared-mutable inventory for the concurrency 3-question test (31 objects)

**Surfaced:** 2026-07-30 · pre-walk enumeration, per the field manual's own find-hint at `:527`
("every `volatile` is greppable — grep them first, then classify").

**What:** the complete worklist for `:519` — *"who owns it, who mutates it, what barrier protects
it. If you cannot name all three for an object, that ambiguity is itself the finding."*

*22 `volatile` objects* (declaration-shape only; MMIO pointer casts and `__asm volatile` excluded —
those are the JSF-205 PASS case):
- `log/rc_log.cpp:481-490` — `g_ring`, `g_head`, `g_tail`, `g_droppedBytes`, `g_highWater`
- `drivers/gps_uart.cpp:164-166` — `g_rxHead`, `g_rxTail`, `g_rxOverflow` (comments assert ISR-writes / consumer-reads)
- `safety/test_mode.cpp:36-46` — `g_test_mode_arm_magic`, `g_test_mode_enabled`, `g_magicObservedAtBoot`
- `cli/rc_os_commands.cpp:53-55` — `g_t2_pending`, `g_t2_cmd`, `g_t2_p1`
- `safety/fault_inject.cpp:32-33` — `g_fault_core0_stall`, `g_fault_watchdog_skip`
- `safety/station_fault_inject.cpp:29-30` — `g_fault_station_rx_drop_remaining`, `g_fault_station_ack_suppress_remaining`
- `safety/fault_protection.cpp:34` — `g_inFaultHandler`
- `safety/flight_in_progress.cpp:22,26` — `g_flightInProgressMagic` (two decls, `#if`-split)
- `safety/pyro_edge_logger.cpp:14` — `g_count`
- `flight_director/flight_director.cpp:147` — `g_phaseObservablePair`

*9 `std::atomic` objects:*
- `shared_state.cpp:39-44` — `g_startSensorPhase`, `g_sensorPhaseDone`, `g_calReloadPending`, `g_core1PauseI2C`, `g_core1I2CPaused`, `g_core1LockoutReady`
- `core1/sensor_core1.cpp:81` — `g_bestGpsValid` · `cli/rc_os.cpp:58` — `rc_os_mag_cal_active` · `drivers/spi_bus.cpp:29` — `g_spi_error_count`

*Not counted, same test applies:* the seqlock-protected sensor snapshot, the PSRAM ring buffer, and
the AO static event objects (structures rather than scalars).

**Why it's here and not only in the itinerary:** the per-file counts are now itinerary hot-spot
notes, but the object *names* are the worklist, and two observations have no other home:

1. **`:511`'s catch-all heuristic no longer discriminates.** It says "touch it anywhere a
   `g_`-prefixed object … appears" — but the 2026-06-24/25 naming remediation renamed QP's `l_`
   module-static prefix to `g_` tree-wide, so `g_` now means *any file-scope static*. The hint
   selects hundreds of non-shared statics. Enumerate from `volatile` / `std::atomic` /
   `multicore_*` / spinlock instead.
2. **`:511`'s "primary on" list names 7 sites; shared mutables live in ~15.** Not a coverage hole —
   the spine ADD at `:173` runs on every file and is the net — but it means the lens's own file
   list is not the surface, and noticing is doing the work a list should do.

**Disposition target:** the inventory retires into the Concurrency findings table as each object is
adjudicated (PASS with owner/mutator/barrier named, or FAIL). Observations 1 and 2 → a `:511`
find-hint correction in the field manual.

**Blocking?** No. But do the enumeration-then-classify pass rather than reading for a smell —
that is the field manual's own instruction at `:527`.

### W-3 — Itinerary complete ≠ exhaustive or “perfect” files

**Surfaced:** 2026-08-01 · walk-findings design (owner)

**What:** When finalizing walk findings / close-out prose, do **not** imply that a fully ticked
itinerary means exhaustive standards coverage or that reviewed files are “clean” / “perfect.”
A checked box means only that the **owner reviewed that path** in this walk sitting. Missing
findings under a path (or `nothing of note`) are not certificates of quality. Some existing pack
wording is stronger than that; don’t play semantic wack-a-mole mid-walk — correct implications at
**finalization** of findings/close-out so agents and future readers don’t over-claim.

**Disposition target:** close-out / findings finalization language (itinerary progress note, plan
close-out, and/or a short findings-header line if needed) — tone only: “reviewed,” not “certified.”

**Blocking?** No — walk continues. Apply when wrapping findings or writing completion claims.

### W-4 — After all board packs: UART GPS multi-file rollup WN

**Surfaced:** 2026-08-03 · walking `board_pico2.h` (owner)  
**Status: ADDRESSED 2026-08-03** — rollup written as **WN-029** (UART GPS + LoRa pin inventory
across all packs → **WN-024**). Erase at walk close after owner verification.

**What (original):** When the board HAL itinerary group is fully ticked, add a findings WN
listing every pack with the UART GPS block → cite **WN-024**. Extended at close to include
LoRa/radio pin defs (none onboard on current SKUs).

---

### W-5 — When questioning header existence: always attach include / consumer verification

**Surfaced:** 2026-08-03 · walking sparse public headers (`job_capabilities.h`,
`notify_backend.h`, job packs) — owner cannot evaluate “who needs this file?” before the
`.cpp` tier without agent help.

**What:** For any header where the walk questions **whether a separate file earns rent**
(sparseness, parallel packs, thin façade), the agent must **double-check and report**:

1. **Direct includes** — every `#include` of that path under `src/` + `include/` (+ `test/`
   if relevant); exclude vendored noise.
2. **Symbol consumers** — who uses the declared API (may be fewer than includes, or
   zero for dead symbols — e.g. `kRoleHasFullGoNogo` defined but unused at check time).
3. **Indirect fan-in** — e.g. packs only included by a selector (`job_*.h` ← `job.h` only);
   types only reached via a façade (`notify_intents` via `ao_notify.h`).

Put a short table or bullets in the **discussion and/or WN** so the owner is not blocked
on not having walked callees yet. Do **not** couple dispositions across “same class”
sparseness notes (capabilities vs backend may resolve differently).

**Datapoint at surface:** capabilities ≈ 1 production include (`health_monitor.cpp`);
backend ≈ 1 caller + 2 impl self-includes; job packs = 0 external includes (selector only).

**Disposition target:** walk process — fold into `L2P5_MANUAL_WALK_GUIDE.md` (and/or
findings Form row / contract-surface helper) as a standing check when existence of a
`.h` is questioned; erase this row once the guide (or equivalent) carries the rule.

**Blocking?** No — walk continues; agent applies the check from this row until the guide
lands it.

---

### W-6 — Sweep: momentary / process archaeology in code comments

**Surfaced:** 2026-08-04 · walking `mavlink_rx.h` + pattern across the walk

**What:** In-source comments that record **momentary development work** rather than a
live contract — e.g. IVP/step land lines, council ticket IDs, “Stage N Phase …”,
migration TODOs that are done, “remove after IVP-…”, tombstones for deleted symbols,
session/agent provenance. Those belong in **docs** (IVP, CHANGELOG, decisions, PRs), not
as permanent code noise. **Keep** real invariants (“dispatcher only”, DMB before memcpy,
bit layouts) with optional short pointer to a decision doc.

IVP-*/council tags are a **solid example class**, not the whole set — anything with the
same “this was a work item” smell qualifies.

**Addendum 2026-08-05 (owner):** triage / alternate-execution paths — a **brief** “why
this special path exists” is good; further history → **commit hash and/or CHANGELOG**
(not multi-line Grok/council/Tier essays). Full claim: **WN-085**. Seeds: **WN-083**,
**WN-084** (`gps_pa1010d.cpp`).

**Suggested later work (not mid-walk mass edit):** grep/heuristic pass over `include/` +
`src/`; triage keep-invariant vs delete/move. Seeds in findings: **WN-050**, **WN-044**,
**WN-033**, **WN-039**, deprecated-alias lines, encoder IVP tags, **WN-083**–**085**, etc.

**Disposition target:** comment-hygiene pass or standing note in CODING_STANDARDS / walk
guide; erase WB when booked.

**Blocking?** No

---

### W-7 — Re-evaluate comment-density header exemption (standards)

**Surfaced:** 2026-08-04 · ao_signals / density discussion after telemetry Doxygen Q

**What:** Blanket “`.h` excluded — Doxygen 60–85% OK” has let low-quality comments
through; structured Doxygen is rare vs archaeology/restatement. Full claim + provenance:
**WN-054** (Project-wide). **Also (2026-08-05):** explicit Doxygen *style* decision —
re-validate whether `@brief`/`@param` markup is still a project goal, then apply
**consistently or drop** — **WN-081** (pairs with density carve-out; not a second track).

**Disposition target:** post-walk (or dedicated) **CODING_STANDARDS** re-eval + optional
measurement/enforcement change; fold **WN-081** into same session; erase when policy
updated or consciously reaffirmed.

**Blocking?** No

---

### W-10 — Inventory: files with Doxygen-style comments

**Surfaced:** 2026-08-05 · owner — prefer walk-WB inventory over growing a WN-081
file list; policy stay in **WN-081** / **WN-054** / **W-7**.

**What:** Produce a **repo inventory** of production headers/sources that use
Doxygen markup (`@file` / `@brief` / `@param` / `@return` / `/** … */` API blocks),
so the later “keep and apply consistently **or** drop” decision has a complete set.
Do **not** treat partial walk evidence as the full list.

**Seed (walk so far — incomplete):** `gps_pa1010d.h`, `i2c_bus.h`, `icm20948.h`
(~6:1), `baro_dps310.h` (+ large OS table — **WN-090**), `rfm95w.h` (**WN-095**).
Expand via grep at disposition; include/exclude rules TBD (public API only vs
all `src/`).

**Disposition target:** inventory artifact (findings note, checklist, or standards
appendix) feeding **WN-081**; erase this row when inventory exists or is declined.

**Blocking?** No

---

### W-8 — Stronger HW-agnostic code guidance (+ optional audit)

**Surfaced:** 2026-08-04 · walking `version.h` / `flash_layout.h` (owner: multi-board
intent vs RP2350-era / Pico-specific assumptions)

**What:** Project direction is “domain code HW-agnostic within compatible ARM MCU
class; HAL/board packs at the edge,” but there is **no hard, specific written rule**
in CODING_STANDARDS (only related practice: board packs, Stage 7 board-agnostic
features, JSF portability soft factor). Walk evidence of leakage: **WN-063** (flash
layout SKU-era assumptions), **WN-068** (`PICO_BOARD` / board identity in `version.h`).
Need stronger / more specific **guidance** (what must live in board/job packs vs
domain headers, what “compatible ARM” means, what is forbidden). **Potentially** a
later audit for code written too board-/SDK-specifically when RP2350 was the only
target.

**Disposition target:** post-walk standards or architecture note (e.g. CODING_STANDARDS
or SAD multi-board section) stating HW-agnostic expectations; optional audit checklist
or workstream if owner wants a sweep. Erase when guidance is written (and audit booked
or declined).

**Blocking?** No

---

### W-11 — Pedagogy: header vs `.cpp` — good example leaf

**Surfaced:** 2026-08-05 · owner after `ws2812_status` walk (h vs cpp explanation)

**What:** Use `drivers/ws2812_status.{h,cpp}` as a **clear, concrete example** when
teaching or re-checking **what belongs in a header vs implementation**:
- **Header** ≈ public Legos (modes, colors, APIs callers snap together).
- **`.cpp`** ≈ assembly (default timings, state machines, PIO/HSV bodies).
- Nuance still fine to mention: private defaults; optional public setters that
  *change* the recipe; other files (`led_patterns.h`, notify) for *semantic*
  codes, not a second mode enum.

Not a defect. Point agents/owner here instead of inventing abstract lectures
mid-walk. Pairs with any future guide note on h/cpp pedagogy.

**Disposition target:** optional fold into walk guide / onboarding; erase when
documented elsewhere or owner declines.

**Blocking?** No

---

### W-9 — Findings volume: trend OK, watch bloat (snapshot 2026-08-04)

**Surfaced:** 2026-08-04 · owner Q after math leaves — pacing / total WN prediction

**What (snapshot at ~WN-075, Next WN-076; ~36/184 leaves):**
- **~2.1 WNs per leaf** so far → naive full-walk linear **~380** total; more realistic
  band **~250–400** if later tiers mix more “nothing of note” / thinner notes (early
  public headers were denser).
- **Length:** median **~9 lines** / mean **~11** per WN; p75 ~13; long tail ~20–38
  (ownership / fan-in writeups). Not essay-scale on average.
- Owner: **fine for now**, keep an eye so volume doesn’t get out of hand (avoid every
  leaf becoming a long multi-claim essay; keep “nothing of note” when appropriate).

**Disposition target:** re-check mid/late walk (or at close-out) against this snapshot;
if density explodes, tighten form or disposition process — not a mid-walk rewrite of
existing WNs. Erase when walk closes or owner reaffirms “still fine.”

**Blocking?** No

---

### W-12 — Brief overview (itinerary keywords → only matching sections)

**Surfaced:** 2026-08-06 · owner during `fusion/eskf_runner.h` walk  
**Clarified (same sitting):**
1. “File notes” = **only** the itinerary checkbox parenthetical (not agent themes).
2. Overview is **not** a full section-by-section file map.
3. If itinerary **keywords** show up in a concrete section of the open file, **show
   that section** (locus + short “what the itinerary note means” if needed). If a
   keyword does not appear in this half of the pair (e.g. header-only), say so
   briefly — don’t invent a TOC of every region.

**Example itinerary line:**  
`fusion/eskf_runner.{cpp,h}` — *(fusion; WMM / cal_flags — float-sentinel remediated
via `CAL_STATUS_WMM_SET`)*  
→ surface WMM / `cal_flags` / `CAL_STATUS_WMM_SET` (and the remediated-sentinel idea)
only where those tokens live; “fusion” alone is the module label, not a reason to
outline the whole file.

**What it is not:** Unsolicited pre-walk; full line-section breakdown; inventing
findings; agent-authored hot-spot lists.

**Disposition target:** fold into `L2P5_MANUAL_WALK_GUIDE.md` (or session handoff /
process note); erase when documented or owner declines.

**Blocking?** No — apply from this row until folded into the guide.

---

### W-13 — Condense WNs by disposition (esp. central / dense leaves)

**Surfaced:** 2026-08-06 · owner after `eskf.h` volume (~12 thin WNs) vs runner-style grouping

**What:** When filing walk findings, **merge by disposition**, not by line number:
- One `WN` per claim that would get a **different** later keep/fix/move-doc decision.
- **Same disposition** → **one WN**, multi-locus list, **full context retained** (quotes,
  line ranges, HW names, ticket ids). Do **not** drop detail to save lines.
- **Nuance inside one disposition** can wait for disposition-time split — walk filing
  should not pre-split “just in case.”
- **Especially** on central files that mint many notes (fusion/`eskf*`, `eskf_runner`,
  flight_director, logging rings, shared_state, …): prefer **fewer richer WNs**
  (ballpark ~2–5 per dense leaf when notes cluster) over ~10 thin ones. Volume watch
  **W-9** still applies.
- Recurring project themes (comment density, HW leakage, bare council tags): **cite**
  an earlier WN when this leaf only repeats; new ID only for a **new** twist or a
  locus inventory worth its own row.

**Example:** `eskf.h` owner-directed regroup — density/design-doc/ticket essays →
**WN-131**; leftover Bierman switch → **WN-132**; prototype-HW noise defaults →
**WN-133**; single-mission tuning (wind chute / rocket vel guard) → **WN-134**.

**Also:** findings form **Split vs merge** row updated to match (2026-08-06).

**Disposition target:** fold into findings header / `L2P5_MANUAL_WALK_GUIDE.md` process;
erase when documented.

**Blocking?** No — apply when filing WNs.

---

### W-14 — Codegen / generated-output inventory: audit done or planned?

**Surfaced:** 2026-08-06 · owner on `fusion/wmm_tables` (AUTO-GENERATED) + prior hand-edit
of generated code

**What:** Double-check whether a **codegen / generated-file audit** has already been run or
is **planned**, covering “outputs only come from generators; no silent hand-edits.” Lived
pain: generated code was edited by hand before (e.g. mission-profile header drift →
“Auto-Generated Code” rule in CODING_STANDARDS / CHANGELOG). Walk keeps hitting generated
or generator-coupled leaves:

| Leaf / artifact | Generator (claimed) |
|-----------------|-------------------|
| `fusion/eskf_codegen.{cpp,h}` | `scripts/generate_fpft.py` (CG-1) |
| `fusion/wmm_tables.{cpp,h}` | `scripts/generate_wmm_table.py` |
| `flight_director/mission_profile_data.h` | `scripts/generate_profile.py` (itinerary: confirm) |
| (others?) | sweep `AUTO-GENERATED` / `Do not edit` under `src/` + `include/` |

**Ask:** Is there an existing audit, CI gate, or booked work that (a) inventories generators
vs outputs, (b) detects hand-edits / drift, (c) policy for SPDX/headers on generated files
(**WN-004** noted missing SPDX on some)? If not, plan one post-walk or as a dedicated pass
— not mid-walk mass regen.

**Disposition target:** findings close-out note, PROBLEM_REPORTS / plan item, or confirm
existing audit and erase; optional CI later.

**Blocking?** No — walk continues; light/exempt leaves still only confirm DO-NOT-EDIT stamps.
