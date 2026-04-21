# Stage T Continuation — Council Review Record

**Date:** 2026-04-21
**Panelists:** NASA/JPL Avionics Lead, ArduPilot Core Contributor, Cubesat Startup Engineer, Advanced Hobbyist Rocketeer
**Process:** Round 1 (independent verdicts) → Round 2 (cross-talk, panelists respond to each other) → Plan-as-whole review (GO-with-changes unanimous)

Raw agent outputs are ephemeral JSONL transcripts. This file is the durable record of what was decided and why.

---

## Background

Stage T original plan (T5-T10, `shimmering-twirling-thimble.md`) hit a wall: T7 retry-timer change couldn't be validated because BW250/BW500 links became brittle in a later session after achieving 100% first-try at C2 earlier. Council was convened to review a pivot to T11+ based on new research (link-budget feasibility, SX1276 driver audit, mLRS + ELRS deep dive, NASA/CCSDS prior-art search).

## Round 1 — Independent verdicts (summary)

### NASA/JPL

- **T11 (register adds):** hygiene, not load-bearing for T1 collision failure. "Don't dress it up as a range improvement."
- **T13 (LQ-adaptive retry):** addresses a DIFFERENT failure mode than T1 diagnosed. Be explicit.
- **T14 (RxDone-anchored TX):** "the actual fix for T2's diagnosis... the one I'd put my name on."
- Prior art: "all systems that work use either strict scheduled TDM or persistence-CSMA with deep backoff. You picked neither."
- Gaps: clock-drift FMEA missing, RxDone miss cascade, IQ inversion audit, CRC-on audit, unexplained T7 session brittleness ("loose thread — don't paper over with more fixes"), Core1 vibration FMEA.
- Process: land T11+T12 standalone first. Don't bundle. LL Entry 36.

### ArduPilot

- **T11:** "Fixing a long-standing bug, not innovating." AP_Radio has had LnaBoost set since 2018. Shifts cliff out ~500 m.
- **T13:** Matters for survivability. Retry storm is itself a collision amplifier.
- **T14:** "Every successful long-range TX-master radio in rocketry and UAV does this. There is no silver bullet."
- Critical new finding: **RP2350 XOSC is ±30 ppm, not SiK's ±5 ppm TCXO.** 30 ms drift over 10 min. **T14 needs re-sync on every RxDone, not one-shot anchor.**
- SiK gotcha: reserves 2× max airtime as guard, plus ~4 ms for SX1276 PLL lock + preamble wake.
- Other: MAVLink-over-serial on half-duplex does NOT "just work." MAV_CMD dedupe (newest-wins by cmd type) — ArduPilot's standard.
- "Ship it, soak it, fly it. Don't add a T16."

### Cubesat

- **T11:** 30 min, log it, move on. Doesn't matter for range at HPR scale. Matters for bench floor + multipath nulls.
- Real fade risk: antenna pattern nulls during descent (15-20 dB fades, 200 ms).
- Missing techniques: MET in every downlink (already have), **beacon-on-startup / ACQ → TRACK state machine** (reframe of T14 as explicit states).
- **T14 skeptical:** "you're engineering for a problem your instrument doesn't have" — skip unless T12 regression shows need.
- COP-1: "absurd ceremony" for 5 Hz commanding.
- Duty-cycle sanity check: retries at 10 Hz hammer antenna match + LDO.

### Rocketeer

- **Post-landing recovery hole:** BW500 wrong for beaconing — 30 dB ground attenuation eats margin.
- **Dual-mode radio proposal** (NEW): in-flight BW500/SF7/10Hz, post-landing BW125/SF9/1Hz. Mission Profile flips on LANDED. RFD900 does this. "Saved my L3 bird in Kansas cornfield."
- **T13 UX concern:** if operator hits `a`=ARM and sees nothing for 800 ms while retries backoff, operator mashes `a` again. **Show retry count live.**
- **T14: most important item.** "Free-running timers on both ends is amateur hour."
- **Skip T13 until field data** — "CU Boulder spent a whole semester on an adaptive algorithm they never needed."
- MVP exit: ARM/DISARM/ABORT ≥95% at 1 km field, post-landing beacon receivable at 2 km on grass, ABORT cycle <250ms × 10.

## Round 2 — Cross-talk (all 4 panelists responded to each other)

### T14 necessity debate

- ArduPilot → Cubesat: "Skip T14 unless T12 shows need" is how you get a system that passes regression and fails at range day. Free-running timer is non-stationary. T12 will pass on a warm bench and fail in a cold pad box.
- NASA/JPL → Cubesat: A system whose success depends on two un-disciplined oscillators happening not to collide is a latent failure, not a working design. T14 converts luck into deterministic behavior.
- Cubesat → both: **Conceded.** "My 'skip unless T12 shows need' was wrong — right for a cubesat with thermal-vac characterized link budget, wrong for a rocket with 70 dB geometry change in 10 min." Withdraws skip.
- Cubesat's reframe accepted by all: **T14 built as ACQ → TRACK state machine, not "one-shot anchor"** — named states, explicit transitions, re-sync on every RxDone, testable in SPIN.

### T13 sequencing

- ArduPilot → NASA/JPL: "Move toward your position. Ship T11+T14 first as determinism fix. T13 after we have soak data on deterministic baseline."
- Rocketeer → all: "Build it, ship it disabled. Don't debut adaptive algorithms on launch day."
- Cubesat: "Instrument the retry layer regardless of whether T13 is enabled. Log retry_count, first_try_ack_rate, retry_ack_rate. Telemetry is free."
- **Consensus: T13 implemented behind flag (ROCKETCHIP_LQ_ADAPTIVE_RETRY), default OFF. Enable decision deferred to post-field-data addendum.**

### Dual-mode radio (Rocketeer's proposal)

- NASA/JPL: "I like this more than I expected. Spacecraft do this routinely (cruise vs safe-mode beacon). Mode transition itself is safety-critical. Needs bounded retry on mode-switch ACK and autonomous fallback."
- ArduPilot: "RFD900 does this with air-data-rate switching. Crossfire does telemetry-rate downshift. Don't rely on station command to switch after landing — vehicle auto-switch on LANDED with long dwell (60 s), AND beacon on BOTH old and new config during overlap."
- Cubesat: "Beacon should include MET + last-known GPS and nothing else. 8-byte payload, massive range margin at BW125/SF9."
- Rocketeer: "Beacon announces its own config in plaintext on first 5 transmissions so operator can manually re-tune if desynced. Saved three friends' birds."
- **Consensus: separate future IVP with own mini-council on mode-switch safety contract. Skeleton in Rev 2 — define enum, transition predicates, safety invariant. Full implementation later.**

### T7 hypothesis

- NASA/JPL: TX-window overlap at boundary SNR. T4 had 70 dB margin; T7 had lower margin + different RF. Collisions saved by retries when margin was high, cliff when margin dropped. **Tripwire: if post-T14 brittleness recurs, halt.**
- Cubesat: second hypothesis — antenna-proximity de-tuning by operator body.
- Rocketeer: third hypothesis — station physical position shifted 2 m → multipath shift (wet grass vs dry concrete = 15+ dB delta). Log station GPS, antenna orientation, ground surface per test.
- **Consensus: all three hypotheses in written doc. NASA's tripwire active. Logging discipline added to bench test procedure.**

### MAV_CMD dedupe

- All 4 panelists agreed. 20 LOC change. Pairs with live retry indicator. Operator mashes button → newest replaces oldest in pending queue, seq stays monotonic.

### COP-1 / COP-1-lite

- Cubesat: "Absurd ceremony" for 5 Hz commanding. "I had to google it" (Rocketeer).
- ArduPilot: "Never missed it on ArduPilot's telemetry layer. If we ever want auth, HMAC-SHA256 over command frame is lighter ceremony and solves threat model (spoofing), not wrong one (packet ordering)."
- NASA/JPL: "Keep in whiteboard bin as 'if we move to SDLS authentication.' For now, dead."
- **Consensus: dead. File `docs/decisions/COP1_NOT_PURSUED.md`. HMAC-SHA256 if auth needed later.**

## Round 2 Consensus Summary

### Resolved (all 4 panelists agreed)

1. **T14 ships.** Framed as ACQ → TRACK state machine with re-sync on every RxDone.
2. **T11 + T12 land first** as standalone commit (LL Entry 36 discipline — don't bundle).
3. **T13 demoted** from "ship" to "implement behind flag, disabled by default, decide after T14 soak + 3 ground tests."
4. **MAV_CMD dedupe** ships with T14, paired with live retry indicator.
5. **Dual-mode radio** approved as separate IVP after T-plan lands. Skeleton in Rev 2.
6. **COP-1 dead.** HMAC-SHA256 replacement if auth needed later.
7. **Retry instrumentation ships regardless of T13 state.**
8. **T7 hypothesis written** with tripwire.
9. **Test discipline logging:** station GPS, antenna orientation, ground surface, operator-body proximity.

### Unresolved

1. T13 enablement criterion — defer to post-soak data.
2. Mode-switch safety contract for dual-mode — needs own mini-council when drafted.

## Plan-as-whole review verdict

**GO-with-changes. All 4 panelists. No dissent on structure.**

Changes applied to `stage-t-continuation.md`:
1. Rev 2 estimate bumped 1 week → 2 weeks (unanimous "original was optimistic by ~50%").
2. Dual-mode skeleton (enum + safety invariant) lands in Rev 2.
3. ACQ-starvation fallback + TRACK clock-drift budget + TRACK→ACQ fallback criteria = gap-IVPs in Rev 2 design doc.
4. T14c dashboard indicator moved Rev 2 → Rev 3 (UX, not correctness).
5. Rev 1 lands to main but does NOT fly until Rev 2 gates pass.
6. 500 m field gate is primary Rev 2 exit; bench ≥98% is pre-check.
7. COP-1-dead decision filed in `docs/decisions/`.
8. Field test only at Rev 2 gate, not between Rev 1 and Rev 2.

## Execution order

Per `stage-t-continuation.md` §10:

1. **Now:** Rev 1 implementation. ~1 hour.
2. **After Rev 1:** Rev 2 design doc drafting (2-3 days) → design council review (2 rounds, 2-3 days).
3. **After design GO:** Rev 2 implementation (5-7 days) → bench soak → 500 m field test.
4. **After Rev 2 gate:** Rev 3 code lands (flag OFF). ~4 hours.
5. **Post-field-data:** Rev 3 enablement decision.
6. **Much later:** Dual-mode future IVP.

Stage T closes when Rev 2 field gate passes.
