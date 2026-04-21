# Stage T Batch B — IVP-T14 Design Doc

**Status:** DRAFT — Round 1 (preliminary points) awaiting council review.
**Plan:** `.claude/plans/shimmering-twirling-thimble.md`
**Batch A baseline:** commit `acd399d` (IVP-T11 + T12 on main)
**Council:** NASA/JPL Avionics Lead, ArduPilot Core Contributor, Advanced Hobbyist Rocketeer, Cubesat Startup Engineer
**Process:** 2-round review. Round 1 = preliminary points (below, premise-level sanity check). Round 2 = full plan after Round 1 feedback is incorporated. No code until Round 2 GO.

---

## Round 1 — Initial Design Points (preliminary)

This is deliberately sparse. Goal of Round 1 is to catch premise-level
problems before I spend time fleshing out details that might be based
on a wrong premise. Panelists: say "yes go detail this" or "no the
premise is wrong, consider X instead" for each point.

### 1. Problem statement

IVP-T12 (Batch A) showed:
- At SF7/BW500/10Hz, first-try ACK is **83.3%**, not the 95% Batch A
  targeted. Eventual 96.7%.
- At SF7/BW125/5Hz (CCSDS default), first-try is **13.3%**, eventual
  37%. Collision-dominated per T1 diagnosis.
- Vehicle-to-station downlink (nav packets) works fine at all configs
  tested. Problem is **station→vehicle commands** not landing first
  try. 17% of commands at BW500 still need a retry.

Council's Batch A hypothesis (T11 driver hygiene alone) was correctly
predicted to be insufficient. The architectural gap is that **station
TX is not synchronized to vehicle RX windows** — station fires commands
on a free-running timer and the 4 ms SX1276 PLL-lock + preamble wake
on the vehicle side means collisions with vehicle's own TX or with
the tail of vehicle's previous RX are probable.

Every successful long-range half-duplex radio in the class below fixes
this the same way: station TX window is anchored to vehicle's RxDone
event, with a bounded guard for oscillator drift.

### 2. Proposed mechanism (RxDone-anchored TX)

- Vehicle transmits nav telemetry at `nav_rate_hz`. Every TX ends with
  TxDone flag set; software already polls it (no DIO0 IRQ edge used —
  per our architecture).
- Station listens. Every valid RX fires its own RxDone flag. Station
  computes: "vehicle just finished TXing at t=now. Vehicle's next TX
  window opens at t+period. The window between now and t+period-guard
  is safe for me to TX."
- Station's scheduled station→vehicle commands fire in that safe
  window, not free-running.
- Re-sync on every RxDone (not one-shot anchor): corrects cumulative
  XOSC drift between vehicle and station.

**RP2350 XOSC is ±30 ppm, not a TCXO.** Over a 10-min window at 5 Hz
nav that's about 30 ms of potential drift — bigger than the RX window.
Re-sync every RxDone keeps drift bounded to <1 ms regardless of session
length.

### 3. Proposed ownership — new AO `AO_LinkHealth`

New active object `AO_LinkHealth` owns:
- Link state enum: `{ kAcq, kTentative, kTrack, kTrackDegraded }`
- RSSI, SNR, LQ%, CRC rate, packets lost, last_rx_ms
- Retry stats per command type (T14b instrumentation, always-on)
- Gate API: `ok_to_retry()`, `ok_to_arm()`, `beacon_mode()`
- Radio mode enum skeleton: `RadioMode { kFlight }` (dual-mode deferred)

`AO_Radio` posts `SIG_RX_DONE` to `AO_LinkHealth` on every valid RX.
`AO_LinkHealth` computes the next safe TX window and exposes it.
`AO_Radio`'s station-side TX scheduler reads that window.
`AO_Telemetry` consults `ok_to_retry()` before each retry tick.

Pattern parallels existing `AO_HealthMonitor` in `src/safety/`. Same
file layout, same gate-accessor style.

### 4. Proposed state machine

```
       +--[1 valid RX]---+
       |                 v
   [kAcq] ---> [kTentative]
     ^  ^       |
     |  |       | [N consecutive RX with LQ≥60%, N≈5]
     |  |       v
     |  |     [kTrack] <---[LQ≥60% over 10 slots, hysteresis]---+
     |  |       |                                               |
     |  |       | [LQ in 20-60% over 10 slots]                  |
     |  |       v                                               |
     |  |     [kTrackDegraded] ----+                            |
     |  |       |                  |                            |
     |  |       | [M missed frames,|                            |
     |  |       |  M≈20 = 2s @10Hz]|                            |
     |  |       v                  |                            |
     |  +------ (any state) ----+  +---------+------------------+
     |                          |
     +--- [idle > N min, no RX, clock drift tripwire] ---+
          (TRACK->ACQ fallback to avoid TRACK-while-drifted)
```

All numbers preliminary. Council round 2 revises.

### 5. Preliminary list of features

1. **RxDone-anchored TX** (main point, above).
2. **Re-sync on every RxDone** (not one-shot).
3. **Guard budget:** 2× max airtime + 4 ms (SX1276 PLL/preamble) + 5 ms slack.
4. **Beacon-on-startup:** vehicle fires beacon burst from boot until
   first station packet received or 10 s timeout.
5. **ACQ-starvation fallback:** if ACQ never converges after M minutes,
   vehicle downshifts to reduced beacon rate; station dashboard shows
   "Vehicle not heard" banner.
6. **TRACK→ACQ idle-timeout:** if idle >N minutes with no RX, drop
   back to ACQ (clock drift would otherwise accumulate unchecked).
7. **Dual-mode enum skeleton** (3-of-4 panelists + user support):
   `RadioMode { kFlight }` with safety invariant "no mode transition
   while flight state is ARMED/BOOST/COAST/DESCENT". Full dual-mode
   (in-flight BW500/SF7/10Hz ↔ landed BW125/SF9/1Hz beacon) deferred
   to its own future IVP with its own mini-council.
8. **MAV_CMD dedupe** (T14a): in `AO_Telemetry::send_tracked_command`,
   if `s_pending_cmd.pending && incoming.cmd_id == pending.cmd_id`,
   replace in-place (new seq, new params, retries_left reset).
   ~20 LOC. Prevents operator-mashing self-collision.
9. **Retry instrumentation** (T14b): per-command-type counters
   `first_try_ack_rate`, `retry_ack_rate`, `retry_count_avg`. Exposed
   via CLI stats dump. Always on. Data feeds Batch C's T13 decision.
10. **SPIN formal model extension:** new states + transitions + 2 LTL
    properties:
    - Liveness: `eventually TRACK given healthy link`
    - Safety: `never silent after LOST` (ACQ-beacon fallback always
      reachable)
11. **Core1 vibration FMEA:** profile worst-case RxDone-to-TX-start
    latency with Core1 at full I2C load (LL Entry 24 territory).
12. **ACK-path timing measurement:** during bench soak, verify vehicle's
    pending ACK drain fits inside station's next anchored window; if
    not, we recreate the collision we're trying to fix.

### 6. Explicitly out of scope for Batch B

- **Dual-mode radio (full implementation):** own mini-council required.
  Skeleton only (enum + safety invariant).
- **CCSDS COP-1 / CLCW / HMAC auth:** council-killed in the continuation
  review. Dead-path doc already at `docs/decisions/COP1_NOT_PURSUED.md`.
- **LQ-adaptive retry (T13):** deferred to Batch C behind flag
  `ROCKETCHIP_LQ_ADAPTIVE_RETRY`, default OFF. Enable decision requires
  3 ground tests with T14b instrumentation showing retry-storm patterns.
- **Live retry indicator on dashboard (T14c):** moved to Batch C per
  Round 2 of the continuation council (ArduPilot: "UX, not correctness
  — buys Batch B a day").

### 7. Gate definition (Batch B exit)

- **Bench pre-check:** ≥98% first-try ACK over N=100 at SF7/BW500/10Hz.
  Must pass BEFORE field test.
- **Primary (Rocketeer):** ≥95% first-try ACK at **500 m open-field**
  with pad-side station geometry. ABORT round-trip <250 ms × 10 in a
  row.
- **No `--no-verify` on any Batch B commit.**
- **Tripwire:** if bench or field gate fails, HALT. Do not advance to
  Batch C. Investigate; don't tune over the cliff.

### 8. Rough effort estimate

2 weeks total (council was unanimous that 1 wk was optimistic by ~50%):
- Design doc + Round 1 + Round 2 council: 2-3 days
- `AO_LinkHealth` + wiring + host tests + SPIN: 5-7 days
- MAV_CMD dedupe + instrumentation: 1 day
- Bench soak + field test + writeup: 3-4 days

---

## Round 1 — Council Questions (please answer in-place)

Each panelist: please respond to each of these with GO / NO-GO / CHANGE.
If CHANGE, state what. Round 1 is premise-level — no hand-wringing on
numbers yet, that's Round 2.

### Q1. Is RxDone-anchored station TX the right mechanism?

Alternative: listen-before-talk (CAD) or pure statistical spreading.
Council's continuation review converged on RxDone anchoring because
every radio in class does it. Confirm or dissent?

- NASA/JPL: __
- ArduPilot: __
- Rocketeer: __
- Cubesat: __

### Q2. Is `AO_LinkHealth` the right ownership boundary?

Alternative: fold this into `AO_Radio` directly (no new AO). Adding an
AO is non-trivial — each AO has a queue, subscriptions, SPIN coverage.

- NASA/JPL: __
- ArduPilot: __
- Rocketeer: __
- Cubesat: __

### Q3. Is "re-sync on every RxDone" right, or should we re-sync less often?

Alternative: re-sync on every Nth RxDone to save cycles. Trade-off:
more re-syncs = less drift, but more work per RX.

- NASA/JPL: __
- ArduPilot: __
- Rocketeer: __
- Cubesat: __

### Q4. Is beacon-on-startup justified?

Before vehicle has received its first station packet, it has no RX
anchor — it can't know where to put station's TX. Beacon tells the
station "I exist, sync to me." Alternative: station always TXes first.

- NASA/JPL: __
- ArduPilot: __
- Rocketeer: __
- Cubesat: __

### Q5. Does the TRACK→ACQ idle-timeout add real safety or just churn?

If system sits idle for N minutes (e.g., on the pad waiting for launch
window), XOSC drift accumulates. Dropping back to ACQ re-acquires sync
but temporarily suspends retries. Trade-off: safer drift bound vs.
brief loss of service at the start of long idle.

- NASA/JPL: __
- ArduPilot: __
- Rocketeer: __
- Cubesat: __

### Q6. Does dual-mode skeleton (enum + invariant only, no transitions) belong in Batch B?

3-of-4 panelists and user preferred landing the skeleton now to avoid a
refactor later. But it adds ~50 LOC with no active use. Confirm or
drop to Batch C.

- NASA/JPL: __
- ArduPilot: __
- Rocketeer: __
- Cubesat: __

### Q7. Is the bench ≥98% + field ≥95% exit gate realistic or too
aggressive?

Batch A measured 83% at C2 on bench — we'd need to go from there to 98%
with just this change. Council's Round 2 in continuation review was
unanimous on "yes, realistic with RxDone anchoring" but numbers are
bench-hostile.

- NASA/JPL: __
- ArduPilot: __
- Rocketeer: __
- Cubesat: __

### Q8. Anything missing from the 12-item feature list?

Things I may have missed:
- (placeholder for council to add)

---

## Round 2 — (to be filled in after Round 1 feedback)

After Round 1 responses come in, I'll revise the design doc with:
- Concrete values for N, M, LQ thresholds, guard budget (Round 1 only
  had preliminary).
- SPIN model LTL formula specifics.
- Core1 FMEA protocol.
- ACK-path timing measurement procedure.
- Any additional features council asks for.

Then Round 2 reviews the full revised design. GO from all 4 = code starts.

---

## Round 1 — Council Review Output

### Sub-round 1 — Independent verdicts

#### NASA/JPL

Q1: **GO** — RxDone anchoring is the right mechanism and the one we can prove. CAD + statistical spreading are both probabilistic mitigations of a deterministic problem (non-stationary oscillators). Anchoring converts luck into a bounded timing contract we can write an LTL property against. Non-negotiable from an FMEA standpoint.

Q2: **GO-with-change** — New AO is correct. But the gate API `ok_to_retry() / ok_to_arm() / beacon_mode()` conflates three different consumers. `ok_to_arm()` is a flight-state concern that belongs to `AO_FlightDirector`; LinkHealth should only expose link-layer truth (state + LQ + next_tx_window). Let FlightDirector *read* LinkState and make the arm decision. Don't let the radio module veto ARM — that's a separation-of-concerns violation.

Q3: **GO-with-change** — Re-sync every RxDone is correct. But spell out what "re-sync" is mathematically: a first-order filter on `anchor = (1-α)*anchor_old + α*anchor_new`, not raw overwrite. Raw overwrite on a noisy RX timestamp (interrupt jitter, Core1 I2C contention) will make the window jitter in lockstep with the noise. Pick α based on expected jitter vs drift rate; write that choice down in Round 2.

Q4: **GO-with-change** — Justified, but the 10 s timeout is arbitrary. Needs a source. Also: "beacon burst from boot" can collide with station TX before vehicle knows the channel is occupied. Specify: beacon is LBT-gated by a cheap preamble-detect OR uses a fixed cadence the station dashboard knows to expect. The brittleness Cubesat pointed to in Round 2 of continuation is real.

Q5: **GO** — TRACK→ACQ on idle timeout is correct. XOSC ±30 ppm is documented. The fallback prevents a silent TRACK-while-drifted failure mode that would otherwise only surface at launch. Classic example of "convert latent failure into observable recovery."

Q6: **GO** — Skeleton is ~50 LOC and prevents a later structural refactor that touches every AO subscription. Cheap insurance. But the safety invariant must be machine-checkable (SPIN), not just documented.

Q7: **CHANGE** — The bench 98% / field 95% gates are **soft gates** in LL Entry 36's sense unless the pass criterion is defined with a binomial CI, a reset protocol between runs, and a control arm. 28/30 on bench is 93% lower CI at 95% confidence — a single 29/30 passes the letter of ≥95%, but with only N=30 the CI straddles the threshold. Round 2 needs: (a) N ≥ 100 for the bench pre-check (doc says so — keep it), (b) explicit Wilson CI lower bound ≥ 95%, (c) reseat + 3-boot protocol between runs per the HW_GATE_DISCIPLINE note in AGENT_WHITEBOARD.

Q8: **Add:** (a) RxDone-miss cascade FMEA — what happens when vehicle doesn't hear 3 in a row from station AND station doesn't hear vehicle? Both enter ACQ, both beacon, both potentially step on each other. (b) Watchdog interaction — no AO handler may block, but the RxDone-to-TX-scheduler path now carries a deadline. Add dead-man check: if `next_tx_window` is stale by more than 2 periods, scheduler must fault-fail-safe to free-running-with-random-jitter, not silently stop. (c) Boot audit log should show `RegSymbTimeout` and `RegPreambleDetect` if those are used — if not, justify.

#### ArduPilot

Q1: **GO** — Every half-duplex TX-master radio in my world does this. SiK does it, FrSky does it, Crossfire does it, mLRS does it. Free-running timers on both ends at non-overlapping rates is literally the textbook example of a two-clock coincidence problem. "Every radio in class does it" is true and sufficient.

Q2: **GO** — New AO is the right call. Folding into AO_Radio makes AO_Radio a god-object and makes LinkState untestable without a radio mock. SPIN coverage, host tests, and the dashboard row all get cleaner with a dedicated AO. Pattern-matches AO_HealthMonitor, which is already proven.

Q3: **GO** — Re-sync every RxDone. At 10 Hz nav with ±30 ppm, max drift per interval is ~3 µs. Cost per RX is trivial (read timestamp, update anchor). Skipping re-syncs to "save cycles" is premature optimization on a path that runs 10 Hz. Don't.

Q4: **CHANGE** — Beacon-on-startup yes, but don't call it a "burst." Burst means same-period high-rate TX which IS a collision generator. SiK's pattern: vehicle transmits at `nav_rate_hz` from boot unconditionally, station listens for any valid preamble, first RxDone establishes anchor. No special "beacon mode" — it's just the vehicle being on the air as soon as it boots. Deleting a mode is cheaper than specifying one.

Q5: **GO-with-change** — Agreed, but N minutes of idle at ±30 ppm is 30 ms × (N/10). The idle-timeout N should be derived from the actual RX window width minus guard, not a round number. At 10 Hz and ~25 ms SF7/BW500 airtime, you probably have a ~70 ms window — drift exhausts that in ~23 min. So N < 20 min is physically justified; N = 5 min with margin. Pick one and source it.

Q6: **GO** — Skeleton stays. 50 LOC and an enum are nothing. The refactor later costs days.

Q7: **CHANGE** — Batch A gave us 83% first-try at BW500/10Hz on bench at 3-4 ft with noise floor at -114 dBm and SNR ~9 dB. The delta from 83% → 98% is entirely in the collision math, not sensitivity. RxDone anchoring should flatten that. BUT: bench 98% is achievable only if the ACK-path timing measurement (item 12 in the doc) actually closes. If vehicle's pending ACK drain doesn't fit in station's next anchored window, we shift the collision, not eliminate it. The gate is realistic *conditional on* item 12 passing first — make item 12 a go/no-go checkpoint inside Batch B, not just a soak-time measurement.

Q8: **Add:** (a) Station-side RX window explicit measurement — vehicle has a TX window anchored to station RxDone, but what about the reverse? Station TX → vehicle RX: what is vehicle's RX-window width at 10 Hz nav minus processing? If vehicle's Core1 I2C read stalls the RX-switch by 1-2 ms occasionally (LL Entry 24), that's a tail of missed commands even with anchoring. (b) `tx_consec_fail` path — when anchored TX fails N times in a row, what does the station do? Currently the scheduler retries on a free-running timer which is exactly what T14 is supposed to eliminate. Define the recovery path inside the anchored model.

#### Rocketeer

Q1: **GO** — "Free-running timers on both ends is amateur hour." Confirmed my Round 2 position. Ship it.

Q2: **GO** — New AO fine by me. I won't touch the code; I care about whether the dashboard shows something human-readable. A state name on the screen — ACQ/TRACK/DEGRADED — is worth more than any retry counter for the guy standing next to the pad.

Q3: **GO** — No opinion on implementation. If re-sync keeps the link from going silent at minute 8 of a pad hold, do it.

Q4: **CHANGE** — Vehicle beacon from boot is right. But "vehicle not heard" banner on dashboard (feature 5) needs to fire LOUD. Color, sound, something I can't miss while I'm 100 ft downrange with the key. Right now "dashboard banner" sounds like a text line that'll scroll off. This is the "did I actually arm it" moment. Make it unmissable.

Q5: **GO** — The idle timeout is exactly the pad-hold scenario. I've stood next to a rocket for 40 minutes waiting for a range clear. If the link goes silent because the two oscillators drifted apart, I'm calling a no-go and going home. Keep the fallback.

Q6: **GO** — I don't care what enum you ship. I do care that post-landing beacon works. As long as the skeleton doesn't block that future IVP, fine.

Q7: **CHANGE** — Bench 98% and field 95% are fine. But the **field gate must specify antenna orientation, station GPS position, and ground surface**, per the test discipline logging we already agreed in Round 2 of continuation. Without that, a successful 500 m test on dry concrete doesn't predict wet grass. Explicitly require 2 consecutive passes in 2 different ground conditions before calling Batch B complete. One test = one data point = not a gate.

Q8: **Add:** (a) ABORT round-trip < 250 ms is in the gate but not in the design — what's the end-to-end budget breakdown? Station keypress → station TX → vehicle RX → vehicle ACK → station RxDone → dashboard update. If that doesn't add up to < 250 ms on paper before we field-test, we'll fail on arithmetic, not RF. (b) Pre-arm "link OK" indicator on dashboard. Before operator hits ARM, they need one glance at the screen that says "radio contract satisfied, go ahead." Right now the dashboard is a pile of numbers.

#### Cubesat

Q1: **GO** — Withdrew my skip-T14 position in continuation Round 2. Anchoring is correct. RP2350 XOSC ±30 ppm vs SiK TCXO ±5 ppm is the killer — luck runs out at 10 min at our drift rate. Anchoring makes this deterministic.

Q2: **GO-with-change** — New AO OK. But `AO_LinkHealth` is a terrible name — it conflates "state of the link" with "is the link healthy enough to do X." Rename `AO_Link` or `AO_LinkManager`. Health-gating is one responsibility among several. Also: NASA/JPL's point about separating arm-veto from the radio is right — our subsystem exposes state, FlightDirector decides policy.

Q3: **GO** — Re-sync every RxDone. 10 Hz is nothing. Cost per RX is dominated by the SPI read of the RSSI register, not the timestamp update.

Q4: **CHANGE** — Beacon-from-boot yes. 10 s timeout before downshift is too short for a cubesat-style deployment (post-separation tumble, antenna-nulls in the rotation period), but this is a rocket not a cubesat so fine for now. ACQ-starvation fallback rate should be sourced from the link budget, not picked. At BW125/SF9 we can beacon at 1 Hz with an 8-byte payload indefinitely — that number comes from the duty-cycle-vs-power budget, not a guess.

Q5: **GO** — Idle timeout is correct. 30 ms drift at 10 min is in the feasible region the doc already cites. Round 2 should pick N based on `(rx_window_ms - guard_ms) / (30 ppm × nav_period_ms)`.

Q6: **GO** — Skeleton. 50 LOC. Done.

Q7: **GO-with-change** — Numbers are fine, but state the reseat protocol and log the ambient RSSI at each field run start. The bench has a 70 dB margin at 3-4 ft which means cable and connector health dominates the variance, not RF. One bad SMA and the gate is meaningless. Already have this discipline from Round 2 — make sure it's in Round 2 of this doc.

Q8: **Add:** (a) **Power / duty-cycle check at new anchored schedule.** Anchoring means station TX happens right after vehicle RxDone. If station retries 3× at 250 ms intervals, and vehicle is TXing at 10 Hz, the station radio is at ~30% duty cycle during a retry storm. Check the RFM95W's continuous-TX limit and the Fruit Jam 3.3V LDO current budget — if we brown out, every other mitigation is moot. LL Entry 24 territory with a different mechanism. (b) MET + last-known-GPS in beacon payload (carry over from continuation Round 2). If beacon is 8 bytes, cost is near-zero and post-landing recovery gets GPS bearing.

### Sub-round 2 — Cross-talk

#### Q2 — ownership boundary

- NASA/JPL: "ok_to_arm() belongs to FlightDirector, not LinkHealth. LinkHealth is the sensor; FlightDirector is the policy."
- Cubesat: "Agree, and the name `AO_LinkHealth` implies it's a pass/fail predicate when it's actually a state machine. `AO_Link` or `AO_LinkManager`."
- ArduPilot: "Fine with either name. The substantive point — LinkHealth exposes state, FlightDirector decides ARM policy — is right. ArduPilot made this mistake once (radio layer vetoing arm) and we had to pull it back after operators complained about unexplained arm refusals. Do not repeat that."
- Rocketeer: "Name doesn't matter to me. But +1 on operator never getting 'arm refused' with no explanation. If something gates ARM, the dashboard tells me why, in English."
- **Convergence:** Rename `AO_LinkHealth` → `AO_Link` (or `AO_LinkManager` — Round 2 picks one). Drop `ok_to_arm()` gate from its API; expose `state()`, `lq()`, `next_tx_window()`, `last_rx_ms()` only. FlightDirector reads those and owns the ARM predicate. Any ARM refusal must be reflected on the dashboard with a human-readable reason.

#### Q4 — beacon semantics

- ArduPilot: "Don't build a 'beacon mode.' Vehicle just transmits at `nav_rate_hz` from boot. Station's first valid RxDone IS the anchor. Deleting a mode is cheaper than specifying one."
- NASA/JPL: "Agree on not calling it a burst. But we do need a specified ACQ-starvation path — if station never hears vehicle (power / antenna / position), vehicle must eventually slow down to conserve power. That's not a separate mode, it's a degraded rate policy."
- Cubesat: "Right — continuous nav-rate TX from boot. ACQ-starvation downshift should be sourced from power budget, not arbitrary time. On the rocket this is less critical than cubesat, but write it down."
- Rocketeer: "I only care about the 'vehicle not heard' banner. Loud. Colored. Sound. Not a text line."
- **Convergence:** No separate beacon mode. Vehicle TXes nav packets at `nav_rate_hz` from boot; station's first RxDone anchors. If station dashboard hasn't seen a valid RX within 10 s post-power (justify the 10 s from cold-start + first-nav-tick math, not pick it), dashboard shows unmissable "VEHICLE NOT HEARD" indicator. Vehicle-side ACQ-starvation downshift deferred — not Batch B critical path.

#### Q7 — gate rigor

- NASA/JPL: "98% at N=30 is not 98%. Wilson CI lower bound is ~85%. You need N ≥ 100 for the bench pre-check to be meaningful. Doc already says N=100 for bench — keep it, enforce it, don't let it slip."
- Cubesat: "Agreed, N=100 bench. Plus ambient RSSI logged per run start. Plus reseat protocol."
- Rocketeer: "For the field gate: two different ground conditions, two different pad-to-station geometries. Not one 500 m line on dry concrete. Recovery range is won or lost in the cornfield, not on the parking lot."
- ArduPilot: "All of the above, plus the ACK-path timing measurement (item 12) becomes a go/no-go checkpoint *inside* Batch B before field test, not a soak observation. If ACK doesn't fit the anchored window, we move the collision instead of eliminating it — and won't know until field."
- **Convergence:** Round 2 gate spec gets: (a) Bench pre-check N=100 with Wilson CI lower bound ≥ 95%, ambient RSSI logged, 3-boot reseat between runs. (b) Field gate is 2 passes in 2 ground-surface conditions. (c) ACK-path timing measurement is a hard checkpoint before field test, not a soak-time observation. (d) Exit gate references HW_GATE_DISCIPLINE.md once that doc exists (whiteboard open item).

#### Q8 — panel additions

Pulling together across panelists:

- NASA/JPL: mutual-beacon collision on both-in-ACQ, dead-man check on stale `next_tx_window`, `RegSymbTimeout` / `RegPreambleDetect` audit.
- ArduPilot: station-side RX-window measurement (`tx_consec_fail` path recovery must stay within the anchored model).
- Rocketeer: ABORT < 250 ms budget breakdown on paper, pre-arm "link OK" glance indicator.
- Cubesat: station duty-cycle + LDO budget during retry storm (anchored retries can pile up in a ~250 ms window at 10 Hz), MET + last-known-GPS in beacon payload.

No panelist dissents on any of these. All four get added to the Round 2 feature list.

### Sub-round 3 — Plan-as-whole

- **NASA/JPL:** GO-with-changes — AO rename + `ok_to_arm()` moves to FlightDirector; re-sync uses a filter not raw overwrite; gate rigor fixed per Q7 convergence; dead-man check for stale `next_tx_window`; both-in-ACQ collision FMEA written.
- **ArduPilot:** GO-with-changes — Delete "beacon mode" language (vehicle just TXes from boot); ACK-path timing (item 12) becomes a hard checkpoint pre-field-test; station-side RX-window width measured in Round 2; `tx_consec_fail` recovery defined inside the anchored model.
- **Rocketeer:** GO-with-changes — "VEHICLE NOT HEARD" indicator is unmissable (color + sound, not a text banner); field gate is 2 ground conditions not 1; ABORT < 250 ms is budgeted on paper before field; pre-arm "link OK" glance indicator on dashboard.
- **Cubesat:** GO-with-changes — Rename `AO_LinkHealth` → `AO_Link` or `AO_LinkManager`; ACQ-starvation rate sourced from power budget; station duty-cycle / LDO check for retry storm; beacon payload includes MET + last-known-GPS (8 bytes); ambient RSSI logged per run.

**Consensus summary:**

- **Agreed changes before Round 2 full design:**
  1. Rename `AO_LinkHealth` → `AO_Link` or `AO_LinkManager`; drop `ok_to_arm()` from its API (FlightDirector owns ARM policy, reads link state).
  2. Re-sync math is a filtered update (`α` to be picked in Round 2), not raw timestamp overwrite.
  3. Delete "beacon burst" / "beacon mode" wording. Vehicle transmits nav at `nav_rate_hz` from boot. First RxDone is the anchor. If no RX within 10 s (justify the 10 s), dashboard shows unmissable "VEHICLE NOT HEARD" indicator (color + sound, not text).
  4. Bench pre-check: N=100, Wilson CI lower bound ≥ 95%, ambient RSSI logged, 3-boot reseat between runs.
  5. Field gate: 2 passes in 2 ground-surface conditions (not 1 run on 1 surface).
  6. ACK-path timing measurement (item 12) is a hard go/no-go checkpoint before field test, not a soak-time observation.
  7. ABORT < 250 ms round-trip budget is broken down on paper before field (keypress → station TX → vehicle RX → ACK → station RxDone → dashboard).
  8. TRACK→ACQ idle timeout N sourced from `(rx_window_ms - guard_ms) / (30 ppm × nav_period_ms)` arithmetic, not a round number.
  9. Guard budget numerical values (4 ms PLL, 5 ms slack, 2× max airtime) each get a source — datasheet / SiK reference / measurement.
  10. Dual-mode skeleton's safety invariant is machine-checkable in SPIN, not just a doc comment.
  11. New additions to the feature list (see below).
- **Unresolved:**
  - Final AO name: `AO_Link` vs `AO_LinkManager`. Round 2 picks.
  - α for the re-sync filter. Round 2 sources or measures.
  - Whether vehicle-side ACQ-starvation downshift lands in Batch B or defers to a separate IVP. Panel leans defer; user to confirm.
- **Panel additions beyond the 12-item feature list:**
  - Both-in-ACQ mutual-beacon collision FMEA + avoidance (NASA/JPL).
  - Dead-man check: if `next_tx_window` stale > 2 periods, scheduler fault-fail-safe to a defined recovery path, not silently stop (NASA/JPL).
  - `tx_consec_fail` recovery path defined inside the anchored model, not a free-running fallback (ArduPilot).
  - Station-side RX-window width measured + documented at Batch A baseline (ArduPilot).
  - `RegSymbTimeout` / `RegPreambleDetect` included in boot audit log or justified as unused (NASA/JPL).
  - Pre-arm "link OK" glance indicator on dashboard (Rocketeer).
  - Station LDO + radio duty-cycle check during retry-storm worst case (Cubesat).
  - Beacon / nav payload includes MET + last-known-GPS (Cubesat; ~8 bytes).
  - Any ARM refusal tied to link state must show a human-readable reason on dashboard (all panelists via Q2 convergence).

**Overall panel verdict: GO-with-changes, unanimous, no structural dissent.** Same pattern as the continuation council: the premise is sound, the scaffolding is right, Round 2 needs to replace preliminary numbers with sourced ones and tighten the gate definitions to match LL Entry 36 / HW_GATE_DISCIPLINE discipline.

---

## Pre-Batch-B Paper Arithmetic (2026-04-21)

Plan consensus item #1 requires an ABORT < 250 ms round-trip budget broken down on paper before any field test. Plan consensus items #2 and #3 require ACK-path timing + α-filter jitter PDF and station LDO rail measurement during retry-storm. This section holds the paper pre-work; bench measurement comes after and is recorded separately in `logs/stage_t/t14_*.md`.

### ABORT round-trip paper budget — target < 250 ms end-to-end

Path: operator keypress on station → station TX of CCSDS ABORT → vehicle RX → vehicle command_handler dispatch → vehicle TX of CCSDS CmdAck → station RX → station dashboard redraw.

**LoRa airtime at SF7/BW500/10Hz, CR=4/5, CRC on, explicit header, 8-symbol preamble** (per SX1276 datasheet §4.1.1.6):

- `T_symbol = 2^SF / BW = 2^7 / 500_000 = 256 µs`
- `T_preamble = (n_preamble + 4.25) * T_symbol = 12.25 * 256 µs = 3.136 ms`
- `T_payload(PL bytes) = (8 + max(ceil((8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*SF)) * (CR+4), 0)) * T_symbol`

For the ABORT command packet (CCSDS command, 22 bytes — same layout as kCmdAckPacketLen after IVP-T5.5 sub 2e):
- `T_payload = (8 + ceil((176 - 28 + 28 + 16 - 0) / 28) * 5) * 256 µs = (8 + 7*5) * 256 µs = 43 * 256 µs = 11.008 ms`
- `T_abort_tx ≈ 14.14 ms` (preamble + payload)

ACK packet is the same 22-byte layout → `T_ack_tx ≈ 14.14 ms`.

**Budget breakdown (worst-case conservative):**

| Leg | Time (ms) | Source |
|-----|----------:|--------|
| 1. Keypress → station USB CDC poll → ao_rcos dispatch | ~50 | `kCliPollMs = 50 ms` upper bound (docs/ROCKETCHIP_OS.md) |
| 2. ao_rcos → AO_Telemetry::send_tracked_command → RadioTxEvt post | ~1 | Cooperative QV, single-tick handler |
| 3. AO_Radio picks up TX event → fifo write + set mode | ~1 | SPI FIFO write: 22 B * 8 bits / 10 MHz SPI ≈ 18 µs + mode switch |
| 4. **SX1276 PLL lock + ABORT packet airtime (TX on vehicle side waits on RxDone for T14-anchored slot)** | ~4 + 14.14 = **18.14** | datasheet §4.1.1.6 + §2.5 (PLL lock 40 µs is negligible vs preamble wake) |
| 5. Vehicle RxDone IRQ → AO_Radio handler → RadioRxEvt post → AO_Telemetry decode | ~2 | QV dispatch plus ~22 B CCSDS decode |
| 6. AO_Telemetry → command_handler::dispatch → FlightDirector ABORT signal → state transition | ~2 | Handler chain inside one tick |
| 7. Vehicle CmdAck encode + RadioTxEvt post | ~1 | Same path as #3, same ~1 ms |
| 8. **Vehicle TX airtime (ACK packet)** | ~**18.14** | Same as #4 |
| 9. Station RxDone → AO_Radio → AO_Telemetry ACK match → dashboard flag | ~2 | Same as #5 + one more tick to re-render dashboard |
| 10. Dashboard redraw | ~25 | ANSI redraw cadence ~40 Hz; worst case next-frame |
| **Total worst-case round-trip** | **~120 ms** | |
| **Margin against 250 ms gate** | **~130 ms** | |

**Observations:**

- Airtime (legs 4 + 8) is ~36 ms combined — the biggest single chunk. SF7/BW500 specifically chosen to keep this small.
- Leg 1 (CLI poll) is the second biggest single cost at 50 ms. That's a fixed cost of the polling CLI architecture; can't reduce without changing polling rate.
- Legs 2-3, 5-7, 9 are cooperative-QV tick costs. Under worst-case Core1 I2C load (the ACK-path timing bench session measures this), each can shift by a few ms. Current estimate is likely optimistic under full load; the bench measurement will refine.
- Leg 4 assumes **vehicle is in station-TX window** (T14 anchoring works). If anchoring fails and station TX collides with vehicle TX, the command is retried at `kAckRetryTimeoutMs = 500 ms` — blowing the budget. **Therefore the budget's credibility depends on the T14 anchoring holding.**
- Leg 10 (dashboard redraw) is user-visibility latency, not propagation latency. If "ABORT confirmed" is considered the moment the flag is set internally (leg 9), shave 25 ms.

**Budget is credible with ~50% margin.** Main risk is leg 4 under anchoring failure (retry storm) — T14's job to prevent. The ACK-path timing bench session will replace the worst-case estimates in legs 2-3, 5-7, 9 with measured values.

**Tracking:** bench-measured values per leg will be appended to this section or recorded in `logs/stage_t/t14_soak.md` once the instrumented session runs. Any leg measurement > estimate by >50% is a flag for review.

### ACK-path timing + α-filter jitter PDF — bench measurement procedure

Plan consensus item #2: "α sourced from jitter PDF measured alongside ACK-path timing." One instrumented bench session measures both; ACK drain must fit within next anchored window, and the anchor jitter distribution drives the α-filter coefficient.

**Setup:**
- Both boards on `bench-1591794` (or newer Batch B firmware). Worst-case Core1 load induced by enabling the nominal flight profile (IMU 1 kHz + baro 30 Hz + GPS 10 Hz on station side).
- Instrumentation: add temporary `[STAGE_T] rxdone_us={T} sched_tx_us={T} tx_start_us={T} txdone_us={T}` log lines to `ao_radio.cpp` around the RxDone → TX-start path. Removable via a compile-time flag, not shipped.
- Station runs in kMenu + `ack_stress_test.py` for 300 sends at 1 s interval (5 min run).

**What to measure:**
1. **RxDone-to-RxDone interval** on station side. Expected: `1000 / nav_rate_hz ± jitter`. At 10 Hz expected 100 ms. Record the jitter PDF (millisecond bucket histogram or mean/stddev).
2. **RxDone-to-TX-start on station**. Currently with free-running scheduler this is ~50-90 ms. With T14 anchoring it should be bounded by the anchored window offset.
3. **TX-start-to-TxDone on vehicle, then RxDone-on-station**. The "ACK drain" — how long between vehicle's ACK TX starting and station hearing it. Must fit inside the anchored window so the next station TX doesn't collide with it.
4. **Jitter PDF → α-filter coefficient**. If the RxDone jitter PDF has stddev σ ms, pick α such that the filter time constant ≈ 3-5σ — attenuates one-off jitter but tracks real drift. Reference: ArduPilot GPS PPS uses α=0.1 for 1 Hz samples with ~100 µs jitter on a TCXO; we have ~30 ppm XOSC and 10 Hz samples with likely 1-5 ms jitter. **If σ is ~1 ms, α in [0.1, 0.2] is a reasonable starting point; confirm against observed PDF.**

**Pass criteria:**
- ACK drain (leg 3) fits within next anchored window with >25% margin (i.e., ≤ 75% of the window width at the new BW/nav combo).
- α picked such that a 3σ jitter outlier shifts the anchor estimate by ≤1 ms.

**Output:** `logs/stage_t/t14_acktiming.csv` + `t14_acktiming.md` summary with the chosen α value and its source.

### Station LDO / 3.3V rail under retry-storm — bench measurement procedure

Plan consensus item #3. Half-day bench session with multimeter + oscilloscope.

**Setup:**
- Station on `bench-1591794` at BW500/10Hz, +20 dBm TX (kDefaultTxPowerDbm).
- Oscilloscope on station 3.3V rail (probe at the Fruit Jam 3.3V test point or directly at the RFM95W VIN pin).
- Induce retry storm: force T14-anchored scheduler into rapid retry by sending DISARM commands at 100 ms interval for 60 s while vehicle is powered off (every command retries 3× then fails). Each retry is a full +20 dBm TX burst (~14 ms airtime).
- Alternatively: force fault-injection path if T14 instrumentation supports it.

**What to measure:**
- 3.3V rail floor under worst-case TX duty (should be ≥ 2.5 V per RFM95W V_min practical limit for stable RF output).
- Droop duration during a single TX burst vs. inter-burst recovery.
- Compare between station on USB power (5V rail → 3.3V LDO) and station on battery (LiPo → 3.3V LDO) — different LDO headroom.

**Pass criteria:**
- Rail stays ≥ 2.5 V under 3× retry at +20 dBm TX on both USB and battery power.
- No observable TX RF output degradation in the ACK success rate during the measurement window.

**Fail modes documented:**
- If rail droops below 2.5V → LDO is the bottleneck, not the scheduler. Fix options: larger decoupling cap on RFM95W VIN, reduce TX power during retry storm (not just nominal), or redesign power rail. Report and halt Batch B until decided.

**Output:** `logs/stage_t/t14_ldo.csv` + `t14_ldo.md` summary with scope screenshots.

### Station-side RX-window width against shipped Batch A binary — bench measurement procedure

Plan consensus item #4. Before any Batch B code change, measure the station's observed RX-window width at each of C0/C0P/C1/C2 on the shipped Batch A firmware (commit `acd399d`).

**Setup:**
- Both boards on `bench-1591794`.
- Same instrumentation as the ACK-path timing session: `[STAGE_T] rxdone_us={T}` log lines on station.
- For each config (C0, C0P, C1, C2): run for 60 s, capture all station RxDone timestamps, compute inter-arrival distribution.

**What to measure:**
- **Nominal RX window:** inter-arrival mean should match `1000 / nav_rate_hz`. C0 ≈ 200 ms, C0P/C1/C2 ≈ 100 ms.
- **Jitter:** stddev and p95 of inter-arrival variation. This is the number T14's filtered re-sync must track.
- **Gap patterns:** identify any bursts of missed RX that would collapse the next-window computation.

**Output:** `logs/stage_t/t14_rxwindow.csv` with per-config inter-arrival PDFs. Feeds the guard-budget calculation in T14's state machine (plan line 187: "2 × max airtime + 4 ms + 5 ms slack" — the slack number should be derived from measured p95 jitter, not picked).

### Summary of pre-Batch-B work completeness

- ✅ ABORT budget — **on paper, ~120 ms worst case, 130 ms margin against 250 ms gate.** Legs 2-3, 5-7, 9 to be refined by ACK-path timing measurement.
- ⏳ ACK-path timing + α-filter — procedure defined, bench session pending.
- ⏳ Station LDO rail — procedure defined, bench session pending.
- ⏳ Station RX-window width — procedure defined, bench session pending.

Three bench sessions. All can be done in one extended bench day with shared instrumentation. Until all four are complete, Batch B code does not start.
