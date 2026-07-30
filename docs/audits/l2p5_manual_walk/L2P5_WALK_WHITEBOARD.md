# L2-P5 Walk Whiteboard — temporary; must be EMPTY at walk close

**Companion to** the walk triad (`L2P5_MANUAL_WALK_GUIDE.md` / `L2P5_WALK_ITINERARY.md` /
`L2P5_WALK_PLAN.md`). Scratch surface for items that surface during the L2-P5 manual walk but
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
