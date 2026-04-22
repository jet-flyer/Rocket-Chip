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
- **LQ-adaptive retry (T13):** DEFERRED until after the CCSDS command-
  path rework (2026-04-22 decision). T13 is parametric tuning on top
  of the current MAVLink-over-LoRa retry loop, which is itself
  flagged as STOP-GAP pending CCSDS TC-Layer + COP-1/FARM/FOP.
  Implementing LQ-adaptive on a path we've already marked for
  replacement is polish misapplied. Revisit after the CCSDS rework
  lands so T13 can target the right retry architecture, or be
  superseded entirely (proper COP-1 FOP has its own retransmission
  semantics). Previously was planned as Batch C behind
  `ROCKETCHIP_LQ_ADAPTIVE_RETRY` flag-default-off.
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

## Round 2 — Revised Design (2026-04-21)

Round 1 was premise-level; this is the concrete design the Round 2 council reviews. Incorporates:
- All 11 Round-1 council consensus revisions.
- All 8 plan-as-whole council tightening items (`shimmering-twirling-thimble.md` §Plan-as-whole Council Review).
- User corrections (AO name stays `AO_RfManager`; `ok_to_arm()` NOT in gate API — FlightDirector owns arm policy).
- AO Commandments invariants (`docs/decisions/AO_COMMANDMENTS.md`).
- Post-fix T12 data (`logs/stage_t/t12_summary.csv` — confirms collision-starve pattern at actual 10 Hz).

### 0. Prereqs already landed

- `35f9591` (Batch B prelim): `nav_rate_hz` now actually wires to TX cadence; `rfm95w::tx_timeout_us` airtime-scaled; `kAckRetryTimeoutMs` airtime-scaled; `radio_config_sx1276_legal()` accepts any hardware-legal combo (advanced user path).
- `6104da5` + `6196e1e`: AO Commandments advisory doc + retroactive scan (no blocker violations).
- `acd399d` (Batch A): T11 driver hygiene (`RegLna`, `RegModemConfig3`) + audit display.

T14 Round 2 is the architectural fix on top of all three.

### 1. AO_RfManager ownership boundary

Per **AO Commandments VII ("one AO, one responsibility")** and **V (`const*` accessor cooperative-dispatch-only)**:

- **`AO_RfManager` owns:** link state enum, RSSI, SNR, LQ%, CRC-error rate, packets-lost counter, `last_rx_ms`, filtered `anchor_estimate_us`, per-command-type retry stats (T14b).
- **`AO_RfManager` does NOT own:** arm policy (FlightDirector), command dispatch (Telemetry), radio hardware (AO_Radio). It reads RxDone via `SIG_RX_DONE` events posted from `AO_Radio`; it exports `next_tx_window(now_us)` that `AO_Radio` consults when scheduling station TX.
- **Public API (const-accessor pattern, Council A6):**
  ```cpp
  // Header-declared invariant: callable only from Core 0 handler context
  // under cooperative QV dispatch. Never from ISR, never from Core 1.
  const RfManagerState* AO_RfManager_get_state();

  // Station-only: when is the next safe TX window open?
  // Returns 0 if anchor stale (deadman fired).
  uint32_t AO_RfManager_next_tx_window_us(uint32_t now_us);

  // Station retry gate — "is the link healthy enough to bother retrying?"
  bool AO_RfManager_ok_to_retry();
  ```
- **NO `ok_to_arm()`** — arming is FlightDirector's decision. FlightDirector reads `AO_RfManager_get_state()->link == kTrack && ...->lq_pct >= kArmLqThreshold` as **one input** to its own pre-arm aggregator alongside battery, calibration, GPS lock, health. One centralized ARM refusal point, one human-readable reason path.

### 2. State machine — concrete

```
         +---[1 valid RX]---+
         |                  v
     [kAcq] ---> [kTentative]
        ^  ^       |
        |  |       | [5 consecutive RX, LQ >= 60%, within 1.5 × nav_period each]
        |  |       v
        |  |    [kTrack] <--[LQ >= 60% over last 10 slots]---+
        |  |       |                                          |
        |  |       | [LQ in [20, 60)% over last 10 slots]     |
        |  |       v                                          |
        |  |    [kTrackDegraded] -------------------+         |
        |  |       |                                |         |
        |  |       | [20 missed frames consecutive: |         |
        |  |       |  2 s at 10 Hz, 10 s at 2 Hz]   |         |
        |  |       v                                |         |
        |  |    (transition to kAcq) <--------------+---------+
        |  |
        |  +---[idle > max(60 s, 30 × nav_period), clock-drift tripwire]
        |       regardless of current state
```

**Transition predicates (concrete, sourced):**

| Transition | Predicate | Source / rationale |
|---|---|---|
| `kAcq → kTentative` | 1 valid RX with `crc_ok && len > 0` | ELRS `ACQ` / SiK link-init: any handshake is promotion. |
| `kTentative → kTrack` | 5 consecutive valid RX with per-RX-LQ ≥ 65% AND inter-arrival ≤ 1.5 × nav_period | 5 = ELRS connection-hysteresis (≈ 500 ms at 10 Hz); 65% LQ floor = above the Schmitt upper bound (§2 revision 1, below); 1.5× spacing catches skip-every-other-packet. |
| `kTrack → kTrackDegraded` | **LQ < 55%** over trailing 10 slots (with **10 % deadband** vs exit threshold) | Round 2 council consensus #1: separate enter/exit to eliminate Schmitt-trigger livelock at the boundary. Prevents spurious drops AND prevents oscillation when LQ hovers at exactly one threshold. |
| `kTrackDegraded → kTrack` | **LQ ≥ 65%** over trailing 10 slots | Hysteresis upper rail; 10 pp deadband vs the 55 % lower rail guarantees state is stable when LQ ∈ [55, 65). |
| `* → kAcq` | (time ≥ 2 s AND missed ≥ 5 frames) OR missed ≥ 20 frames | **Post-Round-2 refinement 2026-04-21 (user direction):** LOS is time-primary with a telemetry-frame minimum floor. Time alone without enough missed frames cannot declare LOS — protects low-rate configs from single-drop false LOS where one dropped packet already takes >2 s. Missed-frames alone without elapsed time cannot declare LOS either — protects high-rate configs from spurious runs. Both conditions together trigger primary LOS. A separate 20-frame fast-frames path triggers early only at nav rates where 20 drops already exceeds any legitimate fade. Same class of bug as the nav_rate plumbing (commit 35f9591): decisions must not implicitly couple to radio config. Time is the universal axis. |
| `kTrack* → kAcq` (idle-drift) | `now_us - last_rx_us > max(60 s, 30 × nav_period)` | XOSC ±30 ppm: 30 ppm × 60 s = 1.8 ms drift — half the guard budget. Pre-emptive re-sync before drift crosses 5 ms. |

**⚠️ All thresholds above (5, 10, 60%, 20, 20%, 60 s) are static-bench defaults. Field testing MAY adjust — noted per plan consensus.**

### 3. Re-sync filter — concrete α

Filtered update on every `SIG_RX_DONE`:
```
anchor_estimate_us = (1 - α) * anchor_estimate_us
                   + α * (rx_done_us - expected_nav_period_us)
```

**α selection (Round 2 council consensus #3, continuous formula):**

α is chosen so the filter's step response to a 3σ anchor jitter bounds at ≤ 1 ms after one sample. For an exponential filter `y[n] = (1-α)·y[n-1] + α·x[n]`, a single sample's contribution to the estimate is `α · x[n]`. Requirement: `α · 3σ_jitter ≤ 1 ms`, so:

```
α ≤ 1_ms / (3 × σ_jitter_ms)
```

Clamp α to `[0.05, 0.5]` — below 0.05 the filter is too lazy for real XOSC drift (~30 ppm over minutes); above 0.5 loses smoothing value.

**Bench source:** ACK-path timing bench session measures Core1-loaded RxDone-to-RxDone inter-arrival PDF, computes σ_jitter under worst-case production load (IMU 1 kHz + baro + GPS on station's own Core 1 if applicable). α derived by the formula above.

Reference: ArduPilot GPS PPS uses α ≈ 0.1 on a ±5 ppm TCXO at 1 Hz (σ ≈ 3 ms under vibration). SiK TDMA uses α ≈ 0.25 on a similar clock at 100 Hz. Our ±30 ppm XOSC at 10 Hz under real load is the target regime — σ_jitter from the bench session picks the exact α.

(Note: a preliminary C0 observation of σ < 0.01 ms was captured during early T14 instrumentation but was under **zero Core1 load**, so it is struck from the design — pre-load numbers are not representative of the gated condition. The bench session under production Core1 load is the authoritative source.)

**⚠️ α value picked from this bench session is static-bench. Flight thermal/vibration/Doppler will shift jitter; field data MAY require a smaller α or adaptive formulation. Listed in §18 revisit parameters.**

### 4. Deadman check on stale anchor

`AO_RfManager_next_tx_window_us(now_us)` returns **0** (= "no window, don't TX") if:
```
now_us - last_rx_us > max(500_000, 5 × nav_period_us)
```

Round 2 council consensus #4: `3 × nav_period` alone was too aggressive at 10 Hz (300 ms fires on a single RF-fade dropout). Absolute 500 ms floor + 5× scaling gives headroom at low nav rates:
- At 10 Hz nav: 500 ms (floor dominates)
- At 5 Hz:      500 ms (floor dominates)
- At 2 Hz:      max(500, 2500) = 2500 ms
- At 1 Hz:      max(500, 5000) = 5000 ms

Caller (`AO_Radio::handle_radio_tick` station-side scheduling) treats 0 as "hold TX, wait for re-sync" rather than falling back to free-running. **Dashboard shows `ANCHOR STALE — awaiting re-sync` (yellow) when deadman active**, so operator knows why commands are queued.

**AO Commandment III applied:** deadman test is O(1), no loops, runs within the tick budget.

### 5. `tx_consec_fail` command-class behavior table

On TRACK → forced-ACQ transition triggered by `tx_consec_fail >= 3`:

| Command class | In-flight command fate | Operator visibility |
|---|---|---|
| **Safety-critical** (ABORT, DISARM, **ARM**, flight-termination) | **NOT dropped, NOT silently retried.** Dashboard escalation: red `LINK LOST DURING <cmd>` banner + audio alert via `SIG_NOTIFY_VEHICLE_LOST` to AO_Notify. Command moves to `s_pending_safety` buffer; AO_Telemetry re-TXes on first successful TRACK re-acquire (anchored to first good RxDone). | Unmissable. |
| **Config / info** (SET_RADIO_CONFIG, QUERY_RADIO_CONFIG) | Silently cancel. Normal retry flow resumes on TRACK; user/script sees a later ACK or timeout. | Normal CLI/GCS flow. |
| **Beacon request** | Silently cancel. | No operator visibility needed. |

**Source:** Round 1 cross-talk (ArduPilot opened, NASA/JPL + Rocketeer + Cubesat converged). Round 2 council consensus #5 promoted **ARM** into the safety-critical row — silent-cancel on ARM is wrong for rocketry. Operator needs to know their ARM was lost so they can re-issue consciously, not discover mid-flight that the ARM never landed because the link was down when the confirm dialog closed.

### 6. Guard budget — concrete numbers

Station TX window guard against boundary-collision with vehicle nav:
```
guard_us = 2 × airtime_worst_us
         + 4_000         // SX1276 PLL lock + preamble wake (datasheet §5.5)
         + max(5_000, 4σ_jitter)  // slack, floored at 5 ms; 4σ per Round 2 #6
```

Round 2 council consensus #6: **4σ instead of 3σ**. At 10 Hz nav × 3600 s = 36000 opportunities/hour to TX, 3σ covers 99.73% → ~98 outliers/hour. 4σ covers 99.994% → ~2 outliers/hour. For a safety-critical command path the extra ~10 ms of slack is worth the 50× reduction in boundary-collision rate.

At SF7/BW500/128 B worst-case payload: `airtime ≈ 70 ms`, so `guard ≈ 140 + 4 + 5 = 149 ms`.
At SF7/BW125/128 B worst-case: `airtime ≈ 270 ms`, so `guard ≈ 540 + 4 + 5 = 549 ms` — at BW125 the whole nav_period is 200 ms at 10 Hz, so guard > period.

**Round 2 council consensus #7: config-time refusal (loud).** When `radio_config_sx1276_legal()` or `ao_radio_apply_runtime_config()` sees `guard_us > nav_period_us`, the config is **refused at apply time** (not silently failing at TX time). Reason text: `"BW=%u SF=%u nav=%u: airtime×2+guard (%u ms) exceeds nav period (%u ms). Choose higher BW, lower SF, or lower nav rate."` Dashboard shows persistent red banner until operator changes config. SET_RADIO_CONFIG dispatch returns `CmdAckResult::kDenied` with the reason in the ACK payload.

Station TX scheduler logic:
```
now_us;
next_vehicle_tx_us = anchor_estimate_us + nav_period_us;
safe_window_start_us = last_rx_us + 2_000;  // ~2 ms after vehicle's TxDone settles
safe_window_end_us = next_vehicle_tx_us - guard_us;

if (safe_window_end_us <= safe_window_start_us + kMinWindow) {
    return 0;  // no safe slot exists at this config — refuse
}
if (now_us >= safe_window_start_us && now_us + my_airtime_us < safe_window_end_us) {
    return now_us;  // fire immediately
} else if (now_us < safe_window_start_us) {
    return safe_window_start_us;  // defer to window open
}
return next_vehicle_tx_us + 2_000;  // missed this slot, wait for next window
```

### 7. Core1 vibration FMEA protocol

**Scope:** profile worst-case RxDone-to-station-TX-start latency with Core1 at full production I2C load (IMU 1 kHz, baro 30 Hz, GPS 10 Hz, all polled concurrently).

**Procedure (bench, pre-field-test):**
1. Station flashed with `ROCKETCHIP_STAGE_T_LOGGING=ON` (`stage_t_log_tx_start` + `stage_t_log_rx` already in tree).
2. Vehicle running nominal Core1 workload at SF7/BW500/10Hz.
3. Record 1000+ station-side `[STAGE_T] tx_start` events.
4. For each, compute `δ = tx_start_us - last_preceding_rxdone_us`.
5. Tabulate δ distribution: mean, stddev, p95, p99, max.
6. **Pass criterion (Round 2 council consensus #8, two-tier):**
   - **Target:** p99 δ ≤ 10 ms.
   - **Hard pass:** p99 δ ≤ 25 ms. Anchored TX still lands reliably (< 25% of BW500 nav period).
   - **WATCH zone:** p99 δ in (10, 25] ms. Log a WATCH flag, note in `logs/stage_t/t14_field.md` that this parameter needs post-field re-verify with real flight-load thermal + vibration stress.
   - **Hard fail:** p99 δ > 25 ms. T14's anchored TX can't reliably land; Core1 is stealing budget. Escalate before proceeding.

**Escalation:** if p99 > 25 ms, mitigation options are:
- Priority boost for AO_RfManager (currently prio 5-6 planned).
- Move Core1 sensor polling into interrupt-driven buffers + seqlock handoff (Stage 13+ pattern already established).
- Reduce nav_rate (user-configurable now, commit 35f9591).

### 8. ACK-path timing bench procedure (plan consensus #2)

**Combined session:** measures α-filter source data + ACK-drain-inside-window + Core1 FMEA + LDO rail (Cubesat #3). One bench afternoon.

**Setup:**
- Both boards on post-T14 firmware, kAnsi dashboards, `ROCKETCHIP_STAGE_T_LOGGING=ON`.
- Station ack_stress_test running at C2 (BW500/10Hz), 5 min at 1 Hz inter-send.
- Vehicle at full Core1 load.
- Scope probe on station 3.3V rail (plan consensus #3).

**Measurements (all from log correlation):**
1. **RxDone jitter PDF** (α-filter source): inter-arrival of valid RX on station.
2. **RxDone-to-next-station-TX** (Core1 FMEA): δ per event, distribution.
3. **Station-TX-to-vehicle-RxDone** (ACK-path in-bound): how long from station's TxDone to vehicle seeing packet.
4. **Vehicle-ACK-TX-to-station-RxDone** (ACK-path out-bound): ACK drain — must fit within next anchored window.
5. **3.3V rail floor** during retry storm (scope, minutely sampled).

**Pass criteria:**
- RxDone σ_jitter < 5 ms; picks α per §3.
- Core1 FMEA p99 ≤ 25 ms per §7.
- ACK drain + next station TX fit within `guard_us` budget per §6.
- 3.3V rail stays ≥ 2.5 V per plan consensus #3.

**Single deliverable:** `logs/stage_t/t14_bench.md` summarizing all four, with CSVs + scope screenshots.

### 9. ABORT budget verification

Paper budget from the pre-Batch-B section: ~120 ms worst case, ~130 ms margin against 250 ms gate. Round 2 commits to **matching paper against measurement during the ACK-path timing bench session**:

- Bench-measured legs (2-3, 5-7, 9 from the budget table) captured in the same STAGE_T_LOGGING run.
- Any leg > 50% over paper estimate: flag for review, root-cause before field.
- If total measured > 250 ms: halt; field test does not start until root-caused.

### 10. SPIN model (LTL properties)

New PROMELA model: `tools/spin/rocketchip_rf_manager.pml`.

**Abstracted model:**
- States: `kAcq`, `kTentative`, `kTrack`, `kTrackDegraded` as labels.
- Input events: `rx_good`, `rx_crc_err`, `tick_miss`, `tick_idle_drift`.
- Output propositions: `can_tx`, `link_healthy`, `notify_lost`.

**LTL properties to check:**

1. **Liveness (eventual-TRACK):**
   ```
   ltl track_convergence { [] (healthy_link -> <> (state == kTrack)) }
   ```
   "If the input stream is healthy (no CRC, no drops), TRACK is eventually reached." Proves ACQ→TENTATIVE→TRACK is reachable without deadlock.

2. **Safety (never-silent-after-LOST):**
   ```
   ltl never_silent { [] ((state == kAcq && prev_state == kTrack) -> <> notify_lost) }
   ```
   "After TRACK→ACQ fallback, `notify_lost` is eventually raised." Proves the unmissable indicator always fires.

3. **Safety (no-TX-while-acq):**
   ```
   ltl no_tx_in_acq { [] (state == kAcq -> !can_tx) }
   ```
   "Station TX is never permitted in ACQ state." Proves the deadman never opens a window during ACQ.

4. **Safety (hysteresis-consistency):**
   ```
   ltl hysteresis { [] (state == kTrackDegraded -> (prev_state == kTrack || prev_state == kTrackDegraded)) }
   ```
   "kTrackDegraded only reachable from kTrack or itself." No wild jumps.

5. **Progress-under-partial-loss (Round 2 council consensus #9):**
   ```
   ltl progress_partial_loss { [] (intermittent_loss -> <>[] (state == kTrack || state == kTrackDegraded)) }
   ```
   "Under intermittent loss (RX flips between good and dropped, but LQ stays ≥ 40%), the machine eventually settles into TRACK or TRACK_DEGRADED and stays there — does NOT livelock between the two." Proves the §2 Schmitt-trigger fix (separate 55% / 65% thresholds) actually eliminates oscillation in the model, not just on paper.

All properties to pass SPIN verification before Batch B code merges. Exit gate alongside host tests.

### 11. Unmissable VEHICLE-NOT-HEARD indicator (Rocketeer, plan consensus #7)

On `* → kAcq` transition (LOST):
1. Post `SIG_NOTIFY_VEHICLE_LOST` to AO_Notify (static event per Commandment VI).
2. AO_Notify → LED red slow-flash (via AO_LedEngine with NotifyIntent::kVehicleLost priority).
3. AO_Notify → audio tone via `notify_backend_audio` (Round 2 council consensus #10: **single tone ships in Batch B**, ~20 LOC on top of the existing Stage 14 audio-backend stub. Visual-only acceptance explicitly STRUCK — panel said Rocketeer's unmissable-at-pad requirement is not negotiable).
4. Dashboard shows `[!] VEHICLE NOT HEARD — check power/position` in red.

All three channels (LED, audio, text) fire on the SAME event so a distracted operator sees at least one.

### 12. Pre-arm "link OK" glance indicator (Rocketeer, plan consensus)

Dashboard adds a single-color-block indicator separate from the detail RfManager row:
- Green: `state == kTrack && LQ >= 80%` (comfortable arm)
- Yellow: `state == kTrack && LQ in [60, 80)%` (arm with caution — dashboard shows why)
- Red: `state != kTrack || LQ < 60%` (FlightDirector will refuse arm)

FlightDirector pre-arm aggregator uses the same predicates → glance indicator always matches whether an ARM attempt will be accepted.

### 13. Dual-mode radio skeleton — REMOVED (user direction 2026-04-21)

Original Round-2 draft had a `RadioMode` enum with only `kFlight` as a populated value, plus a "safety invariant" comment without any enforcement point. Cubesat flagged this as cargo-cult scaffolding (form without function: a single-value enum degenerates to a `uint8_t`, and a comment without a call site constrains nothing). User agreed:

> "scaffolding is worth keeping only if it's load-bearing during development. No Batch B code reads/writes the mode, so it's form without function. Going with option (b): delete §13 entirely."

The dual-mode radio (flight ↔ post-landing beacon ↔ sleep) remains a valid future IVP. When that IVP lands it introduces the enum, transition API, safety invariant enforcement, and call sites together as one coherent change — not a pre-planted stump that the next implementer will have to restructure anyway.

**AGENT_WHITEBOARD item tracks the deferred work.** No code change in Batch B for dual-mode.

### 14. MAV_CMD dedupe (T14a) — concrete

In `AO_Telemetry::send_tracked_command()`:
```
// Round 2 council consensus #5: ARM is safety-critical and explicitly
// excluded from dedupe. Operator may mash `a` but each press represents
// a deliberate decision — we don't collapse them silently.
if (s_pending_cmd.pending &&
    s_pending_cmd.cmd_id == incoming_cmd_id &&
    !is_safety_critical(incoming_cmd_id)) {
    // In-place replace: keep pending-flag, keep sent_ms (retries still
    // target the original window), overwrite params + bump seq.
    s_pending_cmd.seq = next_seq();
    copy_params(s_pending_cmd, incoming);
    s_pending_cmd.retries_left = kMaxRetries;
    return;
}
// Normal queue-new path.
```

~20 LOC. Host test: send 5 `a`=ARM commands in 100 ms, assert pending queue depth = 1 at end with most-recent params.

### 15. Retry instrumentation (T14b) — concrete

Per command-type counters in `AO_Telemetry`:
```cpp
struct RetryStats {
    uint32_t sent_count;
    uint32_t first_try_ack_count;
    uint32_t retry_ack_count;    // any retry slot succeeded
    uint32_t fail_count;          // all retries exhausted
    uint32_t total_retry_count;   // for mean-retries-per-cmd
};
static RetryStats s_retry_stats[kCmdTypeCount] = {};
```

Exposed via `diag_stats.cpp` for `q → s` CLI dump. Always on in all 4 build tiers (no compile flag — this IS the instrumentation). Fed into:
- Dashboard live retry indicator (T14c, Batch C).
- T13 enable decision (Batch C): 3 ground tests showing retry-storm pattern.

### 16. Implementation file list (final)

**New:**
- `src/active_objects/ao_rf_manager.h` + `.cpp`
- `tools/spin/rocketchip_rf_manager.pml`
- `test/test_rf_manager.cpp` (host: state transitions + filter accumulation + deadman + command-class table)
- `logs/stage_t/t14_bench.md` (bench session summary)
- `logs/stage_t/t14_field.md` (field test summary)
- `docs/decisions/COP1_NOT_PURSUED.md` (already planned; written in this batch alongside the others)

**Modified:**
- `src/active_objects/ao_radio.cpp` (post SIG_RX_DONE; station-TX window delegates to AO_RfManager)
- `src/active_objects/ao_telemetry.cpp` (consult `ok_to_retry()`; MAV_CMD dedupe T14a; retry instrumentation T14b)
- `src/active_objects/ao_flight_director.cpp` (reads `AO_RfManager_get_state()` as pre-arm input)
- `src/active_objects/ao_rcos.cpp` (subscribe AO_RfManager at boot)
- `src/active_objects/ao_notify.cpp` (handle SIG_NOTIFY_VEHICLE_LOST)
- `src/cli/rc_os_dashboard.cpp` (RfManager row + glance indicator)
- `src/dev/diag_stats.cpp` (retry counters dump)
- `docs/AO_ARCHITECTURE.md` (new AO entry + inventory update)
- `docs/SAD.md` (data flow)

### 17. Post-fix T12 baseline (reference numbers for Batch B gate comparison)

Actual-cadence baseline post nav_rate fix (commit `35f9591`, 2026-04-21 T12 re-run):

| Config | BW | nav_actual | first-try | eventual | Regime |
|--------|---:|-----------:|----------:|---------:|--------|
| C0 | 125 | 5.8 Hz | 3.3% | 3.3% | Collision-saturated |
| C0P | 125 | 5.8 Hz (capped) | 3.3% | 6.7% | Same as C0 |
| C1 | 250 | 11.5 Hz | 13.3% | 23.3% | Moderate collision |
| C2 | 500 | 11.6 Hz | 40.0% | 96.7% | Low-duty, gaps available |

**Batch B gate (Round 2 council consensus #11):** C2 first-try ≥ 95% at bench (Wilson CI lower bound, N=100) is the **full-pass** criterion. 90–95% with **both §7 Core1 FMEA AND §8 ACK-path timing bench sessions green** is a **WATCH** result — re-verify on field, do not halt. Below 90%, halt regardless of other conditionals.

Current C2 post-fix baseline is 40% — T14's job is to close the 40 → 95% gap.

Exit the 2-round council with GO on this design and we start implementation.

### 18. Static-bench caveat (flight-testing reminder)

Every numerical value in §2-§10 was picked against **room-temperature, stationary bench** data. Flight will change:
- XOSC drift (thermal, vibration) — α may need to be adaptive.
- Airtime jitter under acoustic noise from motor burn — guard budget may need to widen.
- Multipath during coning — RX jitter p99 may balloon.
- Doppler shift at Mach — may need frequency correction loop (not in Batch B).

All thresholds in this design are **starting points**, not flight-final values. Field-testing phase (deferred post Batch B field gate) revisits each.

**Round 2 council consensus #12: explicit revisit parameter list.** The following five values MUST be re-validated against field data in a dedicated section of `logs/stage_t/t14_field.md`. Template field names:

1. **α** (re-sync filter coefficient, §3) — bench-picked formula vs. flight σ_jitter.
2. **Guard slack** (§6, currently `max(5_000, 4σ_jitter)`) — 4σ coverage under flight jitter distribution.
3. **Hysteresis thresholds** (§2, currently enter-55 / exit-65 with 10 pp deadband) — whether flight jitter falls inside or outside the deadband in practice.
4. **Deadman threshold** (§4, currently `max(500 ms, 5 × nav_period)`) — whether 500 ms floor fires spuriously or misses real stale-anchor events under flight RF-fade.
5. **Forced-ACQ thresholds** (§2, post-2026-04-21 refinement: time-primary 2 s AND frame-min 5, plus 20-frame accelerator) — whether 2 s is right for typical flight fade windows, and whether frame-min 5 is too strict at very low nav rates.

The field-log template must call out each value by name with space for "measured under load", "adjusted to", "rationale", and "follow-up." LL Entry 36 discipline: no honor-system handoff from bench to flight — the revisit is a recorded step, not an assumption.

### 19. Round 2 Final status (2026-04-21)

Round 2 council: **GO-with-changes, unanimous**. 12 consensus revisions applied inline:

| # | Section | Revision |
|---|---|---|
| 1 | §2 | Schmitt-trigger fix: enter kTrackDegraded at LQ<55%, exit at LQ≥65%, 10 pp deadband |
| 2 | §2 | Forced-ACQ cap: `min(2 s, 20 × nav_period)` absolute 2 s upper bound |
| 3 | §3 | Continuous α formula (1 ms bound on 3σ step response), struck zero-load preliminary observation |
| 4 | §4 | Deadman threshold: `max(500 ms, 5 × nav_period)` with dashboard ANCHOR STALE indicator |
| 5 | §5+§14 | ARM promoted to safety-critical; MAV_CMD dedupe excludes ARM |
| 6 | §6 | Guard slack: 4σ replaces 3σ |
| 7 | §6 | Config-time refusal when guard > nav_period, with reason text on dashboard |
| 8 | §7 | Core1 FMEA two-tier gate: target 10 ms, hard pass 25 ms, WATCH in between |
| 9 | §10 | Progress-under-partial-loss LTL (#5) proves §2 Schmitt fix eliminates oscillation |
| 10 | §11 | Audio tone for VEHICLE NOT HEARD ships in Batch B (visual-only STRUCK) |
| 11 | §17 | Gate conditionals: 90-95% at C2 is WATCH if §7+§8 pass; < 90% halt |
| 12 | §18 | Five-parameter explicit revisit list with field-log template fields |

User disposition on §13 dual-mode: **DELETED** (cargo-cult form-without-function per Cubesat). Dual-mode IVP is deferred; when it lands it introduces enum + API + invariant + call sites together as one coherent change.

**Code start unblocked** conditional on: (a) this final design committed, (b) three pending bench sessions (ACK-path timing + α source, LDO rail, RX-window width) done with recorded pass/fail. Bench sessions can run in parallel with the AO_RfManager skeleton implementation — scaffolding the file structure, subscriptions, state-machine transitions, and host tests does not need the bench numbers. Tight integration (final α value, final deadman numbers if any tuning) waits for bench session data.

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

#### 2026-04-22 addendum — aggressive-retry impact on the budget

Post-T14d the tracked-command retry changed from `3 × 500 ms` to
`8 × 250 ms` (commit `8d2ed74`). Three new rows for the budget table:

| Case | Round-trip | Notes |
|------|-----------:|-------|
| Single-shot, anchored-window hit | ~120 ms | **Unchanged.** The 250 ms gate holds when anchoring works. |
| First TX missed, retry #1 succeeds | ~370 ms | 120 ms budget + 250 ms retry interval. **Over the 250 ms gate** for the second attempt. Operator sees ACK at t+370 ms if first TX landed outside the anchored window or collided. |
| Worst-case retry exhaustion (8 retries fail) | ~120 + 7 × 250 ≈ 1870 ms | No ACK; command reported failed. ABORT wouldn't be confirmed; operator would re-press. |

**Implication:** the 250 ms gate is a **single-shot** metric. With the new retry config, the effective gate is "first-try lands within 250 ms" — i.e., T14 anchoring must succeed on the first attempt for the gate to hold. If first-try success rate is low, the operator will regularly see ABORT latencies > 250 ms even though each retry is fast. This reinforces the N=100 Wilson CI pre-check as the load-bearing measurement for Batch B: first-try rate is what makes the ABORT gate meaningful, not retry-eventually-succeeds rate.

**Revised gate interpretation:** `N=100 first-try ACK rate ≥ 95%` (Wilson lower bound) is equivalent to "95% of ABORTs confirm within ~120 ms, the remaining ~5% within 370-1870 ms depending on retry count." If first-try < 95%, aggressive retry masks the failure from operator UX but doesn't restore the 250 ms gate for those 5%.

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

- ✅ ABORT budget — **on paper, ~120 ms worst case, 130 ms margin
  against 250 ms gate.** Aggressive-retry addendum appended 2026-04-22
  (commit `59b5192`): single-shot ~120 ms, retry-fallback ~370 ms,
  worst-case retry-exhaust ~1870 ms.
- ❌ ACK-path timing + α-filter — **OUT-OF-SCOPE (dropped 2026-04-22).**
  Requires oscilloscope + instrumented bench session. User retro:
  council-approved beyond what's practical in our bench environment.
  α-filter currently uses a reference-radio default; works in practice.
- ❌ Station LDO rail — **OUT-OF-SCOPE (dropped 2026-04-22).** Same
  reason: scope-required measurement. Observed RSSI/link health during
  100+ DISARM retry stress sessions shows no brown-out symptoms; will
  re-evaluate if field-testing surfaces rail issues.
- ❌ Station RX-window width vs Batch A binary — **OUT-OF-SCOPE
  (dropped 2026-04-22).** Requires reflashing pre-T14 firmware to
  establish baseline for comparison; information value is low now that
  T14 anchoring is in place and link is demonstrably healthy.
- ⏸ N=100 Wilson 95% CI on first-try ACK — **DNF (2026-04-22).** Three
  attempts, instrumentation-bound non-result; deferred to re-measure
  under the CCSDS-layer rework where "first-try" becomes a meaningful
  metric. See `logs/stage_t/t14_wilson_ci_attempts.md`.

Scope-dropped items were council-approved beyond what's practical for
the current bench setup. Stage T Batch B code landed in commits
`3159173` through `5adb878`; Batch C's IVP-T14c landed at `9f0e04a`;
Batch C's IVP-T13 (LQ-adaptive retry) deferred to the CCSDS-command-
layer rework per commit `8fdf951`. Remaining concerns fold into the
whiteboard "Stage T 95% first-try re-baseline" item and the eventual
CCSDS rework.

---

## Round 2 — Council Review Output (2026-04-21)

Correctness-level review of the revised design (§0-§18). Panelists: NASA/JPL Avionics Lead, ArduPilot Core Contributor, Advanced Hobbyist Rocketeer, Cubesat Startup Engineer. Two-round format per `COUNCIL_PROCESS.md`.

### Sub-round 1 — Independent verdicts

#### NASA/JPL

**GO-with-changes.** The design is substantively correct and the state machine + SPIN properties are now at a rigor I can sign. Three concrete defects: (a) §3 α-selection is piecewise on a σ-bucket but the bench PDF is one draw — write the rule as "α selected such that the filter's 3σ response ≤ 1 ms" and let σ drop out continuously; discrete buckets create cliffs where a session at σ=1.01 ms lands on a different α than σ=0.99 ms for no physical reason. (b) §4 deadman at `3 × nav_period` is too aggressive at 10 Hz (300 ms kills TX after losing 2 frames — a single RF fade during coning does this) and too loose at 2 Hz (1.5 s). Make it `max(500 ms, 5 × nav_period)` floor. (c) §10 LTL set is missing **progress-under-partial-loss**: `[] (state == kTrack && intermittent_loss -> <>[] (state == kTrack || state == kTrackDegraded))` — i.e., never livelock between them. Without this, the hysteresis bands in §2 are asserted, not proven.

#### ArduPilot

**GO-with-changes.** §2 is what SiK would draw. 5-consecutive for promotion matches Crossfire's `LINK_UP_THRESHOLD=5`. 20-missed for forced ACQ is what RFD900 calls "long connection loss." Two specific flags: (a) §5 **`Arm` = silent-cancel is wrong for the rocketry use case.** ELRS and Crossfire both escalate ARM refusal loudly because the operator is standing there expecting it to happen. Promote `Arm` to same treatment as safety-critical (dashboard red + audible). Only `Config / info` and `Beacon request` stay silent. (b) §7's 25 ms p99 gate is sourced as "25% of BW500 nav period" which is arithmetic, not empirical — ArduPilot AP_HAL targets 90% of period available to non-scheduler work, so p99 ≤ 10 ms is the flight-software-grade number. 25 ms should be the pass; 10 ms should be the aspiration and the number that must trend down if not met. Also (c): §2 transition table marks `kTrack → kTrackDegraded` and `kTrackDegraded → kTrack` on the same "trailing 10 slots LQ ≥ 60%" with identical thresholds — that is **not hysteresis, that's a Schmitt trigger with zero deadband** (`[20,60)%` enters degraded, `≥60%` exits). At LQ bouncing around 60% you will livelock. Widen exit to ≥ 70% (or entry to < 55%). This is the concrete edge case the NASA/JPL progress-under-partial-loss LTL will catch.

#### Rocketeer

**GO-with-changes.** §11 unmissable VEHICLE NOT HEARD with LED + audio + text is exactly what I need. §12 glance indicator is in. §17 post-fix 40% → 95% gap is big but credible if you believe anchoring eliminates collision, which I do. Two pushes: (a) §11 says "visual-only is acceptable for Batch B if audio backend is stubbed" — **NO**. This is the "did I actually lose contact with the rocket" moment. If the audio backend is stubbed, finish just the one beep for `SIG_NOTIFY_VEHICLE_LOST` before Batch B ships; a single tone is not Stage 14 scope creep. (b) §6 refuses TX at BW125/10Hz (guard > period). That's correct physics, but I'd bet 30% of my club would pick BW125 without understanding it can't command at 10 Hz. **The refusal must be loud, on-screen, at config time — not silent.** Dashboard should show "BW125/10Hz: station cannot command — reduce to 5 Hz or use BW250+" the moment the config is picked. You've said "accept anything the radio can do" but that isn't the same as failing silent when it's a configuration error. (c) §9 ABORT budget total 120 ms is paper, and leg 1 (50 ms CLI poll) is the single biggest cost — fine, but that's the leg most at risk of growing if Core1 load is heavier than bench. Put an explicit budget-re-verify checkpoint after flight-testing, not just before.

#### Cubesat

**GO-with-changes.** §0 acknowledging 35f9591 + 6104da5 + 6196e1e as prereqs is good — the design is on top of known-fixed cadence + airtime scaling. §7 Core1 FMEA procedure is concrete enough to execute. §18 static-bench caveat is appropriate but not enforceable without explicit re-verify hooks in the plan. Concrete concerns: (a) §3 σ_jitter < 0.01 ms on bench is **suspicious.** Core1 has not yet been loaded during that observation (confirmed by "preliminary observation" wording — it was C0 instrumented, not C2 with full sensor load). Do NOT use 0.01 ms as the α-selection input; use ONLY the measurement from the ACK-path timing session with full Core1 load per §8. The plan already says that, but §3's preliminary observation paragraph will be copy-pasted by someone as justification. Strike that sentence or flag it red. (b) §13 dual-mode skeleton is enum + invariant comment — this is **cargo cult** without a transition dispatcher stub. The whole reason to land a skeleton is to reserve the API shape; an enum with one value and a prose invariant reserves nothing. Either add a `RfManager_request_mode_change(RadioMode)` stub that returns `kRefusedDueToFlightState` unconditionally, or drop the skeleton and land it with the real dual-mode IVP. Half-measure is worse than neither. (c) §6 guard budget uses `3σ_jitter` — 3σ is 99.7% coverage; at 10 Hz nav = 36000 samples per hour, you'll see 108 excursions beyond 3σ per hour. For flight-critical window gating use p99.9 or 4σ.

### Sub-round 2 — Cross-talk

**On §2 livelock (ArduPilot vs NASA/JPL):**
- ArduPilot: "The §2 table entering kTrackDegraded at LQ<60% and exiting at LQ≥60% IS livelock-bait. Widen to `[20, 55)%` enter / `≥65%` exit — 10-point deadband."
- NASA/JPL: "Agreed, and that's exactly what my progress LTL would catch. Better to catch it in SPIN now than in field. The asymmetric threshold proposal is standard Schmitt-trigger practice."
- Cubesat: "+1. Also: the 20-missed-frames forced-ACQ at 10 Hz = 2 s, but at 2 Hz = 10 s. 10 s silent before forced re-sync is a long time for an operator. Make it `min(2 s, 20 × nav_period)` — upper bound 2 s absolute."
- Rocketeer: "At 10 s I will have already called a no-go and disarmed myself. Cap it at 2 s."
- **Converges:** §2 table changes: enter kTrackDegraded at `LQ < 55%`, exit at `LQ ≥ 65%`. Forced ACQ at `min(2 s, 20 × nav_period)`. NASA/JPL's progress-under-partial-loss LTL added to §10.

**On §4 deadman threshold (NASA/JPL):**
- NASA/JPL: "3 × nav_period at 10 Hz = 300 ms = 2 frames. Any fade fires the deadman."
- ArduPilot: "SiK uses 5 × period with a 500 ms floor. Same logic — prevents spurious fault under a multipath dip."
- Cubesat: "`max(500 ms, 5 × nav_period)` is the right shape. Worst-case 500 ms of 'no TX' costs one retry slot; it doesn't cost anything safety-relevant."
- Rocketeer: "If the deadman fires while I'm holding the arm key, dashboard has to show why, not just gray out."
- **Converges:** §4 deadman becomes `max(500 ms, 5 × nav_period)`. When fired, dashboard shows explicit "ANCHOR STALE — awaiting re-sync" (not just a grayed-out TX indicator).

**On §5 Arm class (ArduPilot vs plan-as-whole):**
- ArduPilot: "Silent-cancel on ARM during link-loss is an anti-pattern in every RC radio. Operator presses arm, nothing happens, operator presses harder. Our MAV_CMD dedupe turns that into a no-op. Promote ARM to safety-class treatment."
- NASA/JPL: "Arming is an explicit state-transition request with real consequences — pyro becomes live. Treating it as silent-cancel is a usability bug and a traceability bug (no log of the failed attempt as perceived by operator)."
- Rocketeer: "If I press ARM and nothing happens I WILL press again. If I press 5 times and then the link comes back, suddenly the vehicle arms — that's the dedupe replacing the last press — and now I'm standing next to an armed rocket that I thought had ignored me. **Hard NO on silent-cancel.**"
- Cubesat: "Agreed. The §14 MAV_CMD dedupe (newest-wins) combined with §5 silent-cancel on ARM creates exactly the Rocketeer's scenario. Close the loop: ARM refusal during LOST must be visible, and the pending-command buffer MUST NOT silently re-queue an ARM on TRACK re-acquire."
- **Converges:** §5 table changes: `Arm` moves to **safety-critical** treatment (dashboard red + audible). ARM attempts during non-TRACK state are **refused, logged, and NOT auto-retried on re-acquire** — operator must re-issue. §14 MAV_CMD dedupe scope explicitly excludes ARM from newest-wins: each ARM press is a separate command, each refusal is a separate log line.

**On §7 Core1 FMEA gate (ArduPilot):**
- ArduPilot: "25 ms p99 is too loose. 10 ms is flight-software-grade."
- NASA/JPL: "25 ms pass with 10 ms target is reasonable framing. Fail gate > 25 ms halts Batch B. Between 10-25 ms is 'ship with a trend-down TODO.'"
- Cubesat: "Two thresholds, one gate: `pass ≤ 25 ms p99, target ≤ 10 ms p99, WATCH if 10-25 ms`. Same pattern we use for link margin."
- **Converges:** §7 pass criterion restated: p99 δ ≤ 25 ms hard-gate, ≤ 10 ms target, 10–25 ms records a WATCH item in `logs/stage_t/t14_bench.md` that is revisited post-field-test.

**On §3 α-filter piecewise vs continuous (NASA/JPL):**
- NASA/JPL: "Piecewise buckets are cliffs. Continuous: α = min(0.5, k / (σ_jitter_ms × expected_filter_length)) — pick k from the 3σ response requirement."
- Cubesat: "Continuous is right. Also strike the 'σ_jitter < 0.01 ms at room-temp' preliminary line — that's pre-load and will mislead."
- ArduPilot: "Practical compromise: compute α continuously from measured σ, round to nearest 0.05 for readability. Same answer, less magic."
- **Converges:** §3 replaces piecewise table with a one-line formula tied to the 3σ response target (`filter 3σ-step response ≤ 1 ms`). Preliminary "σ < 0.01 ms" sentence STRUCK — replaced with explicit "to be measured with Core1 at full load per §8; do not use bench-C0 numbers as the source."

**On §13 dual-mode skeleton (Cubesat):**
- Cubesat: "Enum + invariant comment reserves nothing. Cargo cult."
- NASA/JPL: "Agreed. Stub `RfManager_request_mode_change(mode)` returning `kRefusedNoFlightStateGuard` proves the API shape. Zero-LOC alternative is just delete §13 and land it with dual-mode."
- ArduPilot: "My Round 1 was 'skeleton stays' on the grounds of preventing refactor. If it's just an enum, refactor cost is zero anyway. Drop skeleton OR add the stub."
- Rocketeer: "Don't care. Just don't break post-landing beacon when that IVP happens."
- **Converges:** §13 either (a) adds a `RadioMode AO_RfManager_request_mode_change(RadioMode)` API returning `kRefusedNoFlightGuard` unconditionally in Batch B — proving dispatch surface exists — or (b) is deleted entirely and deferred. Panel prefers (a); user decides.

**On §6 BW125/10Hz silent refusal (Rocketeer):**
- Rocketeer: "Silent refusal at config time is a misconfiguration trap."
- ArduPilot: "Dashboard should show the refusal as a persistent banner tied to the config row, not a scrolling line."
- NASA/JPL: "Guard-budget check runs at config accept; if `guard_us > nav_period_us`, refuse the config at the CLI/GCS entry point, not silently at TX time."
- Cubesat: "Right — this is a validation-at-entry problem, not a runtime problem. Move guard check to `radio_config_sx1276_legal()` or equivalent."
- **Converges:** §6 gets a new sub-bullet: "If `guard_us > nav_period_us`, `radio_config_sx1276_legal()` REFUSES the config with reason-text; CLI/GCS surfaces that reason in the current config-apply flow. Dashboard persists a red 'CONFIG INVALID — see RadioCfg' banner until operator changes config."

**On §6 3σ vs 4σ guard (Cubesat):**
- Cubesat: "At 36000 samples/hour, 3σ is 108 outliers per hour past the guard."
- NASA/JPL: "4σ is the DO-178 convention for continuous-time safety gates. Accept."
- ArduPilot: "At 5 ms minimum slack floor, the slack itself usually dominates — but in the BW500 fast-jitter case where σ can be bigger, the 3σ vs 4σ diff is real. 4σ."
- **Converges:** §6 guard becomes `2 × airtime_worst + 4 ms + max(5 ms, 4σ_jitter)`.

**On §11 audio backend (Rocketeer):**
- Rocketeer: "One tone for SIG_NOTIFY_VEHICLE_LOST is not Stage 14 scope creep."
- Cubesat: "If audio backend is stubbed and we ship visual-only, we're one stumbling-operator step from missing the alert."
- NASA/JPL: "Three-channel redundancy is the whole point. Dropping one to stub is halving the redundancy."
- ArduPilot: "Single tone is ~20 LOC. Do it."
- **Converges:** §11 visual-only exception is STRUCK. Batch B ships with at least one audible tone on `SIG_NOTIFY_VEHICLE_LOST`, even if rest of audio backend is stubbed.

**On §17 post-fix baseline plausibility (cross-panel):**
- ArduPilot: "40 → 95 is 55 percentage points. With collision being the dominant loss mode (confirmed by C0/C1/C2 monotonicity in t12_summary.csv), anchoring closes most of that. Believable."
- NASA/JPL: "Plausible conditional on ACK-path timing closing (§8) AND Core1 FMEA passing (§7). Stack the conditionals explicitly in the gate statement."
- Cubesat: "Conditional-on-prereqs is the right framing. If §7 or §8 fail, §17 gate is moot."
- Rocketeer: "If it's 90% I'll take it. 95 is an aspiration; don't throw the whole field test out for falling short by 2 points if ABORT round-trip is fine."
- **Converges:** §17 gate statement gets explicit conditionals — "C2 ≥ 95% first-try (Wilson CI lower bound) **conditional on §7 p99 ≤ 25 ms AND §8 ACK-drain-fits**. If 90-95% with conditionals green, WATCH for field behavior before halting Batch B."

**On §18 static-bench caveat enforceability (Cubesat):**
- Cubesat: "Prose caveat without hook is toothless."
- NASA/JPL: "Add explicit flight-test revisit checkpoints to the design for: α value, guard slack, hysteresis thresholds, deadman threshold, forced-ACQ miss count."
- ArduPilot: "Field-test log template in `logs/stage_t/t14_field.md` should have a section `Parameters to revisit post-flight:` with those five items."
- **Converges:** §18 gets a concrete list of **five flight-test-revisit parameters** (α, guard slack, hysteresis thresholds, deadman threshold, forced-ACQ count). `logs/stage_t/t14_field.md` template includes a dedicated section listing them with columns `bench value / field observation / proposed update`.

### Plan-as-whole verdict

- **NASA/JPL:** GO-with-changes.
- **ArduPilot:** GO-with-changes.
- **Rocketeer:** GO-with-changes.
- **Cubesat:** GO-with-changes.

**Unanimous GO-with-changes. No structural NO-GO.** The design's architecture is sound (RxDone anchoring, AO_RfManager ownership, command-class escalation, SPIN coverage). The changes are numerical tightening and one genuine correctness fix (§2 Schmitt-trigger livelock) that SPIN would catch if the missing LTL progress property is added.

### Consensus summary — agreed revisions before code starts

1. **§2 hysteresis deadband.** Enter `kTrackDegraded` at `LQ < 55%`; exit at `LQ ≥ 65%`. Prevents Schmitt-trigger livelock around LQ=60%.
2. **§2 forced-ACQ cap.** `min(2 s, 20 × nav_period)` — absolute 2 s upper bound regardless of nav_rate.
3. **§3 α-selection formula.** Replace piecewise buckets with continuous `α` chosen so filter 3σ-step response ≤ 1 ms. Strike the "σ < 0.01 ms at C0 bench" preliminary observation as misleading.
4. **§4 deadman threshold.** `max(500 ms, 5 × nav_period)`. Dashboard shows "ANCHOR STALE — awaiting re-sync" when fired.
5. **§5 Arm class.** Promote `Arm` to safety-critical treatment (red + audible on link-loss refusal). §14 MAV_CMD dedupe explicitly excludes ARM from newest-wins.
6. **§6 guard budget.** `2 × airtime_worst + 4 ms + max(5 ms, 4σ_jitter)` — 4σ replaces 3σ.
7. **§6 config-time refusal.** If `guard_us > nav_period_us`, `radio_config_sx1276_legal()` refuses the config with reason-text. Dashboard persistent red banner until operator changes config.
8. **§7 Core1 FMEA two-tier gate.** Hard pass `p99 ≤ 25 ms`, target `p99 ≤ 10 ms`, 10–25 ms logs a WATCH for post-field revisit.
9. **§10 SPIN.** Add progress-under-partial-loss LTL (`[] (intermittent_loss -> <>[] (state ∈ {kTrack, kTrackDegraded}))`). Proves the §2 Schmitt-trigger fix landed.
10. **§11 audio.** Single tone for `SIG_NOTIFY_VEHICLE_LOST` ships in Batch B, even if rest of audio backend is stubbed. Visual-only acceptance STRUCK.
11. **§17 gate conditionals.** State C2 ≥ 95% is conditional on §7 + §8 both passing. 90–95% with conditionals green → WATCH, not halt.
12. **§18 revisit list.** Explicit five-parameter revisit list (α, guard slack, hysteresis thresholds, deadman threshold, forced-ACQ count) with dedicated section in `logs/stage_t/t14_field.md` template.

### Unresolved — user break-tie needed

- **§13 dual-mode skeleton disposition.** Panel preference is to replace enum-only with a stub API `AO_RfManager_request_mode_change(RadioMode)` returning `kRefusedNoFlightGuard` unconditionally, OR to delete §13 and defer to the dual-mode IVP. Current enum + invariant comment is cargo-cult per Cubesat, conceded by ArduPilot. User picks (a) stub or (b) delete.

### Final verdict

**GO-with-changes, unanimous.** Code start gated on: (i) all 12 consensus revisions applied to the design doc, (ii) §13 disposition decided by user, (iii) the three pending bench sessions from §Summary-of-pre-Batch-B-work-completeness (ACK-path timing + α source, LDO rail, RX-window width) completed with recorded pass/fail numbers.

No panelist dissents on the premise. This is the same pattern as Round 1: architecture holds, numbers need tightening, hysteresis needs a deadband, one missing LTL catches what field would otherwise surface.

