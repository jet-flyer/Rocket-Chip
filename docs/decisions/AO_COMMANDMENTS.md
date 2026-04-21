# AO Commandments — Active Object Design Rules for RocketChip

**Status:** Advisory. Authored 2026-04-21 as part of Stage T Batch B prelim (see plan at `.claude/plans/shimmering-twirling-thimble.md`).
**Scope:** Rules for designing and writing Active Objects (AOs) in this project, based on QP/C framework conventions, Miro Samek's published guidance, and project-specific lessons-learned.
**Authority:** Advisory (not JSF-AV-grade normative). Violations should be flagged in review and fixed unless there's a documented exception.
**Out of scope:** Internal QP/C framework requirements (covered by state-machine.com's SRS_QP_AO_* specs). This doc covers how *application* AOs are written, not how the framework behaves.

---

## Why this document exists

Before authoring `AO_RfManager` (Stage T Batch B), the Round 1 council flagged that RocketChip had per-AO rationale in `docs/AO_ARCHITECTURE.md` but no cross-cutting rules. We looked for external published rules we could adopt wholesale (Samek PSiCC2, QP framework docs, Douglass, F´, NASA cFS) and found philosophical guidance plus one project-adjacent normative ruleset — F´'s adoption of JPL Power-of-10 — but nothing JSF-AV-grade that covers application-level AO design specifically. This doc condenses the external guidance and names the project-specific rules with their LL Entry sources.

If a future council elevates this doc to normative (standards/), it becomes `standards/ACTIVE_OBJECT_RULES.md`. Until then it's advisory here.

---

## Sources

- **Samek, "Practical Statecharts in C/C++, 2nd ed." (PSiCC2)** — the canonical AO book.
- **state-machine.com** — Miro Samek's QP framework site:
  - [Active Object page](https://www.state-machine.com/active-object) — the 3 core principles.
  - [RTEF page](https://www.state-machine.com/rtef) — Real-Time Embedded Framework best-practices.
  - [QP/C SRS Active Objects](https://www.state-machine.com/qpc/srs-qp_ao.html) — framework-level requirements (not duplicated here).
  - [Active Objects for Embedded — App Note PDF](https://www.state-machine.com/doc/AN_Active_Objects_for_Embedded.pdf).
- **NASA F´** — [F´ Code Style](https://nasa.github.io/fprime/UsersGuide/dev/code-style.html); adopts JPL Power-of-10, which we already enforce via `standards/CODING_STANDARDS.md`.
- **RocketChip Lessons Learned** — `.claude/LESSONS_LEARNED.md` Entries 32 and 35 (project-specific).

---

## The Commandments

### I. Keep AO data private. No cross-AO mutable-state sharing.
**Source:** Samek, state-machine.com/active-object — "Keep data isolated and bound to threads internally, and not share them with the rest of the system."
**How to apply:** AO state is `static` inside the `.cpp` or hidden behind a `struct` whose definition is private. Other code touches it only via the AO's public API.
**Existing pattern:** `AO_HealthMonitor`'s internal struct is opaque; callers use `health_monitor_get_state()`.

### II. Communicate between AOs asynchronously, never synchronously.
**Source:** Samek, state-machine.com/active-object — "Communicate among threads asynchronously via event objects. Using asynchronous events keeps the threads running truly independently, without blocking on each other."
**How to apply:** Use `QACTIVE_POST` or publish-subscribe signals. Don't reach into another AO's internals to trigger work. Read-only state inspection is OK (see Commandment V).

### III. Event handlers run to completion, bounded by the tick period.
**Source:** Samek, state-machine.com/rtef + PSiCC2 Ch. 6. Also LL Entry 32.
**How to apply:** Each `SIG_*` handler must return within ~1 ms (well under a 10 ms tick period at 100 Hz). If a handler genuinely needs longer work, split into start/poll (e.g., non-blocking TX in `rfm95w_send_start` + `rfm95w_send_poll`) so each segment is short.
**Why:** Blocking in an AO handler starves every other AO's queue. Discovered the hard way at Stage 9 migration — see LL Entry 32.

### IV. Never block inside a handler. Period.
**Source:** Samek, Application Note PDF — "Active objects should not block internally, period."
**How to apply:** No `sleep_ms()`, no busy-wait on hardware, no I2C reads that could take >1 ms, no `flash_safe_execute()`, no serial writes that could stall. If the peripheral is slow, use start/poll. If you must wait on time, use `QTimeEvt`.
**Why:** Under our cooperative QV scheduler, a blocked handler stalls the entire dispatch loop. Timer ISRs keep firing and post events; queues fill up; we assert on `qf_actq` overflow. See LL Entry 32.

### V. Read-only `const*` accessors are safe **only** within cooperative single-core dispatch context.
**Source:** Council A6 decision (in `docs/AO_ARCHITECTURE.md`). Project-specific rule.
**How to apply:** If another AO needs to read your state (e.g., `AO_FlightDirector` reading `AO_RfManager_get_state()` as a pre-arm input), expose it via a `const Type* AO_X_get_state()` accessor. The caller must:
- Invoke the accessor synchronously from its own tick handler on Core 0.
- Never call it from an ISR.
- Never call it from Core 1.
- Never cache the pointer across AO dispatch boundaries (state can change between handlers).
Put a comment at the accessor prototype naming the invariant explicitly.
**Why:** Under QV cooperative dispatch, no two handlers run concurrently. Reads are coherent. Under a preemptive scheduler (QK), they wouldn't be. This rule does NOT hold for multi-core or preemptive use.

### VI. Posted events must outlive dispatch — use `static` or pool-allocated storage.
**Source:** LL Entry 35 (project-specific).
**How to apply:** For any `QACTIVE_POST(&ao, &evt, ...)` or `Q_PUBLISH`:
- **OK:** `static MyEvt s_evt = { ... };` at file scope (or function-scope static), populated per-post, then posted. Safe because the next post overwrites the same buffer only after the previous dispatch has completed (QV cooperative scheduling guarantees this).
- **OK:** Pool-allocated event via `Q_NEW(MyEvt, SIG_X)` (we don't use pools currently, but the pattern is supported).
- **NOT OK:** Stack-local `MyEvt evt; ... QACTIVE_POST(&ao, &evt, ...);` — the stack frame disappears after the caller returns; the queue holds a dangling pointer.
- **NOT OK:** `static` events accessed from an ISR (ISRs aren't in the cooperative dispatch order).
**Why:** QP stores the event *pointer* in the receiver's queue, not a copy. A stack-local event is a use-after-free landmine. Shipped as a latent bug from Stage 7 through Stage 14 before stack growth triggered it. See LL Entry 35.

### VII. One AO, one responsibility. Decisions outside your domain belong in another AO.
**Source:** Samek PSiCC2 Ch. 6 (AO = "thread of control with private data"); Councilor feedback on `AO_RfManager` (Round 1 T14 council).
**How to apply:** An AO owns its domain's state and the logic that operates on that state. It does NOT make decisions that belong to another AO's domain, even if it has enough information to.
- **Example (correct):** `AO_RfManager` owns link state; exposes `get_state()` for others to read. Does NOT expose `ok_to_arm()` — arming is `AO_FlightDirector`'s decision.
- **Example (wrong):** An `AO_Radio::fire_abort_if_link_down()` — that's mixing transport-layer detection with safety-layer action.
**Why:** Scattered veto points across AOs produce "refused with no explanation" user experiences (ArduPilot has war stories about this). Centralizing policy in the aggregator AO keeps reasoning traceable.

### VIII. One AO per textbook-pattern responsibility. No AOs for "just a function that runs periodically."
**Source:** Council A1 (ESKF stayed a module, not an AO) + Stage 13 council (HealthMonitor promoted after re-review).
**How to apply:** A thing is worth promoting to an AO only if all three hold:
1. It has its own tick rate, independent of callers.
2. It owns state that outlives any single call.
3. It has multiple consumers of that state (publish-subscribe is cheaper than N consumers polling a module).
If fewer than three hold, it's a module.
**Why:** Each AO costs a queue, a priority slot, a subscription surface, SPIN-model coverage. Don't spend those unless the AO earns it.

### IX. AO priorities are set once at `AO_*_start()` — never change during runtime.
**Source:** QP framework convention; state-machine.com/qpc examples.
**How to apply:** Priority is passed to `AO_Y_start(priority)` at boot. Higher numbers = higher priority. For RocketChip: FlightDirector=7, HealthMonitor=6, Logger=4, etc. (see `docs/AO_ARCHITECTURE.md`). Don't reorder AOs with runtime logic.
**Why:** Priority inversion is hard to reason about statically; runtime reordering makes it impossible. Cooperative scheduling reduces the risk but doesn't eliminate it on the event-queue drain order.

### X. Subscribe at boot, not mid-session. Publish events, not pointers.
**Source:** state-machine.com/qpc, publish-subscribe convention.
**How to apply:** Call `QActive_subscribe(&ao.super, SIG_X)` exactly once per AO, in its `initial` state handler or in `AO_X_start()`. Don't subscribe/unsubscribe dynamically. Events carry by value (small structs or pool-allocated); don't carry raw pointers to mutable state across event boundaries.
**Why:** Dynamic subscription invites "I posted before they subscribed" race conditions. Pointer-carrying events reopen Commandment VI.

### XI. Any `QTimeEvt` fires on its armed AO — not globally.
**Source:** QP/C docs.
**How to apply:** `QTimeEvt_armX(&me->tick_timer, ticks, period)` arms a timer that posts to `me` (the containing AO), not broadcast. A broadcast timer fan-out is explicit: post to a signal that multiple AOs have subscribed.
**Why:** Each AO has its own cadence; global timers defeat that.

### XII. Log what the AO did; instrument decisions.
**Source:** LL Entry 36 ("gates that only check incremental change cannot catch pre-existing rot").
**How to apply:** An AO that makes non-trivial decisions (e.g., `AO_RfManager`'s ACQ→TRACK transitions) emits diag log lines with context (`[RF] ACQ->TENTATIVE after 1 RX, LQ=65%`). Retry instrumentation (IVP-T14b) makes counter state externally visible. Don't let an AO's internals be a black box at debug time.
**Why:** Most AO bugs only reproduce under load. Post-hoc reasoning from CSV / serial log is faster than re-creating the scenario.

---

## How this doc is used going forward

- **New AO proposals:** cite which Commandments apply. Flag any Commandment the new AO breaks with a documented rationale.
- **Code review:** if a PR introduces a Commandment violation, the reviewer flags it. Merge requires either a fix or an explicit exception entry in `AGENT_WHITEBOARD.md` / `STANDARDS_DEVIATIONS.md`.
- **Council reviews:** council uses this doc as a shared language (e.g., "that's a Commandment VII violation").
- **Elevation path:** if over time the project wants a JSF-AV-grade normative ruleset, this doc is the input. It becomes `standards/ACTIVE_OBJECT_RULES.md`, numbered, with severity and remediation for each rule. Until then, advisory is enough.

---

## Revision history

- **2026-04-21** (this version) — Initial authoring. Stage T Batch B prelim.
