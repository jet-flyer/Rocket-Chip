# Stage T Fix Council — Transcript

**Date:** 2026-04-19
**Convened to:** design the Stage T fix based on T1-T4 data
**Panelists:** NASA/JPL Lead, ArduPilot Core, Hobbyist Rocketeer, Cubesat Engineer
**Input:** `logs/stage_t/FIX_COUNCIL_REPORT.md` (T1-T4 synthesis)

---

**NASA/JPL Lead:** Let me anchor on the FMEA before we talk fixes. The §1 data is unambiguous. Pooled N=114 at 3-4 ft, first-try 6.1%, zero CRC errors across ~435 RF events, vehicle RX count exactly equals station ACK count per run. That's not a degraded link — that's a collision pattern with a perfect return path. And §2 nails the geometry: **140 ms TX airtime into a 100 ms RX window**. There is no scheduling policy that makes a 140 ms peg fit a 100 ms hole. I want to start by saying Option E (COP-1 + CLCW) alone does not close this gap, and Option F alone does not close this gap. The primary fix has to be A or B.

**ArduPilot Core:** Agreed, and I want to reinforce why T2 going 0/30 is the most important data point in the whole report. The plan expected cheat-mode to hit >95% and we got 0%. That's the council's prior being wrong, and it's right to update hard. The `send_after_rxw_ms ≈ 67 ms` with 140 ms airtime means the station TX extended ~107 ms past the vehicle RX window end — smack in the middle of the next `kTxActive`. Every single packet got clipped. The 6.1% in T1 isn't the floor of a well-tuned system; it's the sliver of randomly-timed TXes that happened to start early enough in RXW to complete before the vehicle flipped to TX. Any fix that doesn't move either the 140 or the 100 is cosmetic.

**Rocketeer:** So can I ask the blunt question — at 2 Hz nav, what does the user actually see? My launch window is Saturday. If I'm watching a dashboard and altitude updates every 500 ms instead of every 200 ms, do I care? For a rocket going 0 → apogee → main deploy → landing, I don't think I do. Apogee detection is onboard, not station-side. Descent rate is the only ground-visible number that matters and 2 Hz is plenty for that. I'll trade 3 updates/sec for "DISARM actually works on the first press."

**Cubesat:** I want to push on the B option before everyone converges on A. BW 125 → 250 kHz halves airtime to ~70 ms, which fits the 100 ms window with 30 ms margin at 5 Hz. The -3 dB sensitivity hit — let's put numbers on that. SF7/BW125 sensitivity is ~-123 dBm per the T4 data. -3 dB → -120 dBm. At 20 dBm TX power with decent antennas, your link budget at 1 km LOS is still comfortable; even at 3-5 km you're fine for a rocket. The report at §6 frames this as "at 1 km field range it could matter" — does it, actually? What's the worst-case range in the Rocket Mission Profile?

**Rocketeer:** Worst case for a hobby HPR flight is maybe 3 km slant range at apogee, and that's an L or M motor flying tall. Most flights are under 1.5 km AGL and slant range to ground station maybe 2 km total. -120 dBm sensitivity at 20 dBm TX on dipole antennas closes that easily — probably 10+ dB margin.

**NASA/JPL Lead:** I'll accept the link-budget argument for B at short range, but I want to flag an FMEA concern. B keeps the 5 Hz cadence, which means the station TX window margin is only 30 ms. That's tight. Any jitter in station TX scheduling — ISR latency, QF tick variance, SX1276 fifo-write delay — eats into that 30 ms. At 2 Hz the margin is 220 ms; it's not even close to a failure mode. I lean A for margin. The rule on flight hardware is you want the biggest margin you can afford, not the minimum that works on a bench.

**ArduPilot Core:** That's the right framing. And there's another wrinkle — we haven't talked about **question 7** yet, which I think is the sharpest point in the whole packet. The report's §5 says T2 and T3 both broke the station RX in the same way with compile-flag-gated radio-path changes. "Station boots, RF front-end tracks carrier, decoder silently fails." That's two data points of the same failure. If IVP-T6 is "drop nav cadence 5→2" and that change is anywhere near the radio path, we could ship a "fix" that looks clean at compile time and silently breaks RX in the field. That is the red flag.

**NASA/JPL Lead:** That's exactly why I endorse the report's §7 ordering: **T5 FIRST, before any firmware config change lands.** You cannot responsibly ship A, B, or C until you understand why T2/T3 broke. Otherwise you're flying blind into the same landmine.

**Cubesat:** I'll push back a tiny bit — A is a change to `kDefaultRocketRadioConfig.nav_rate_hz`. That's a **data-driven cadence change**, not a code-path change. The scheduler reads the value, multiplies a period. T2 added a new `#ifdef` branch in `handle_rx_packet` and new queue variables. T3 flipped a `constexpr` field that feeds modem setup. Both are structural. A is a number. The risk surface for A is much smaller than for T2 or T3.

**ArduPilot Core:** Fair. I agree the **numeric** change (A) is lower risk than T2's structural change. But I still want T5 to happen because if we're going to do E (COP-1 + CLCW) later, that IS a structural change on the hot path. I'd rather root-cause the silent-RX bug once than chase it four times.

**Rocketeer:** Question for the group — does T5 need to happen before T6, or in parallel? From a "Saturday launch" view, T6 is the tiny change that unblocks bench testing. T5 is the deeper dive. If I can do T6 as a clean number change while someone else investigates T5, I'm shipping sooner.

**NASA/JPL Lead:** I'll give you that, but with one condition. T6 lands, we measure first-try, we confirm station RX is still alive. If RX counter advances and first-try > 80%, T6 is fine. If station RX goes dead on T6 the way it did on T2, we stop, and T5 becomes urgent because a number-only change shouldn't be able to break RX, and if it does we have a bigger problem than nav cadence.

**Cubesat:** That's a clean gate. T6 is the canary for T5 — if a numeric config change is safe, the surface area for the T2/T3 bug is narrower than we feared.

**ArduPilot Core:** Agreed. So the order becomes: T6 ships first with a *post-flash RX-alive sanity check* as the go/no-go, T5 runs in parallel (or immediately after) to diagnose the silent-RX class bug for future structural work. T7 (shorter retry timer) and T8 (COP-1 + CLCW) come after.

**Rocketeer:** On T7 — the report asks whether F is a distraction. My take: F is **not** a distraction for bench testing and operator UX, but it is a distraction if it's presented as a root-cause fix. At 1.8% fail-after-3-retries and 3 s between retries, mean successful-command latency is ~6 s. Shorten to 500 ms and that drops to ~1-2 s. That's the difference between "DISARM feels responsive" and "did I even press it?" But only do F *after* A, because dropping retry interval from 3 s to 500 ms at 5 Hz nav means six retries inside one ~3 s period at 50% collision probability each — more RF congestion. At 2 Hz nav with the new ~360 ms clear window, 500 ms retry is well-matched.

**NASA/JPL Lead:** I like that dependency logic. F only makes sense after A. Let me restate: **A → F → E**, and **T5 is a parallel track that must resolve before any structural radio-path work (which E definitely is)**.

**Cubesat:** On Option D (FDD) — everyone agrees skip, right? The report correctly identifies it as significant work, and A closes the gap at a one-line cost. D is the right answer if we'd ever been unable to solve this at the firmware layer, but we can.

**ArduPilot Core:** Skip D for now. Flag it in the whiteboard as a "if we ever need full-duplex command channel for a Gemini tier or for a crewed-adjacent application, here's the path." Don't build it against the current problem.

**Rocketeer:** On question 5 from the brief — is 2 Hz too slow for the Rocket Mission Profile? No. Rocket apogee detection is on-vehicle, not ground-dependent. Station sees altitude/velocity every 500 ms, which is fine for range-safety monitoring. The only place 5 Hz would matter is if a human operator needed to see sub-second attitude for a live abort decision, and we don't have that in the profile. HAB profile is even less demanding — altitude changes over minutes, 2 Hz is luxurious.

**NASA/JPL Lead:** I'd want 2 Hz validated against the Mission Profile's abort-guard timeouts. If the station-side abort logic needs a fresh packet within, say, 250 ms to trigger something, 500 ms period doesn't give it. But based on the current architecture, abort logic is vehicle-side with optional station ABORT relay command — the relay is station→vehicle, not vehicle→station. So 2 Hz downlink doesn't degrade safety. Agreed, 2 Hz is fine.

**ArduPilot Core:** I want to nail down exit criterion (question 8). What do we want to see at T10 before declaring Stage T closed? The report doesn't propose a number. I suggest: **first-try ACK ≥ 80% pooled over N ≥ 100, at operational geometry (3-4 ft), with fail-after-3-retries ≤ 1% and zero CRC errors**. 80% is conservative — the math says 2 Hz with 140 ms TX in 360 ms RXW should be closer to 100% first-try, but bench reality has jitter. 80% is a clean PASS with margin for surprises.

**Cubesat:** Also add: **station RX packet counter advances continuously across the soak** (no silent-RX bug regression). And: **average successful command latency < 2 s** after F lands. That ties the IVPs together.

**NASA/JPL Lead:** And a regression check — we should re-run T4 ambient after T6 to confirm nothing in the config change altered the station's noise floor behavior. Cheap test, high confidence.

**Rocketeer:** Three exit criteria for operator-facing quality: (a) first-try ≥ 80%, (b) fail-after-retries ≤ 1%, (c) successful-command latency < 2 s. Those are the numbers I'd report to the team.

**ArduPilot Core:** One more architectural nit before we wrap. Any structural work in T8 (COP-1 + CLCW) **must respect LL Entry 32** — no blocking calls in AO handlers. COP-1 state machines can be tempting to write with spin-waits. Do not. The sliding-window ARQ timer must post events, not block. And **LL Entry 35** — any new `QEvt` in the ARQ path must be `static` or pool-allocated, never stack-local. These aren't optional; they're the gates that stopped the system from crashing before.

**NASA/JPL Lead:** Second that. Add it to the IVP-T8 acceptance: code review includes an explicit LL-32 / LL-35 compliance check. Grep `QACTIVE_POST` in any new ARQ code and verify the event arg is `static` or `Q_NEW`.

**Cubesat:** On the parallelism question — I think T5 and T6 can run sequentially in practice because T6 *is* the cheapest test of whether T5's class of bug exists. If T6 passes RX-alive, T5's diagnostic work can continue in parallel with T7 and T8 implementation, because T7 is also a numeric tweak and T8 is where T5's findings really matter. So: T6 (gated by post-flash RX-alive), then T7 and T5-investigation in parallel, then T8 only after T5 resolves.

**ArduPilot Core:** That's tight. I'd accept that.

**NASA/JPL Lead:** Let me also name a red flag for the implementer. The report's §5 lists three hypotheses for the T2/T3 silent-RX bug. I want a fourth explicitly considered: **the modem re-init doesn't happen cleanly on a warm boot with different config**. If there's any path where `SX1276::begin()` returns without applying the new config register writes under certain flag combinations, you get exactly the symptom — RF front-end runs on whatever it had last, decoder expects new config, nothing decodes. That's a single-boot-path audit, not a hypothesis-tree search.

**ArduPilot Core:** That's the one I'd bet on. Check the register-write sequence in the driver's init and confirm every config register is written unconditionally on `begin()`, not cached against previous state. ArduPilot's SX1276 driver explicitly rewrites the full config on every init for exactly this reason.

**Rocketeer:** Can I ask one more thing — question 6, the range trade. If we do A (2 Hz, stay at BW 125 kHz), we keep the -123 dBm sensitivity and don't touch range. Right? No -3 dB hit?

**Cubesat:** Correct. A is pure cadence, doesn't touch the RF config. You keep full sensitivity. B is where the range trade happens, and we're not recommending B as primary.

**Rocketeer:** Then I'm done. A wins on every axis I care about.

**NASA/JPL Lead:** Consensus appears to converge. Let me state it as a verdict.

---

## Consensus verdict

**Fix package:** A (primary) + F (transitional after A) + E (long-term ARQ), with T5 (firmware-variant RX-break diagnosis) as a parallel investigation that must resolve before T8 lands.

**IVP ordering:**

1. **IVP-T6 (Option A — primary fix):** Change `kDefaultRocketRadioConfig.nav_rate_hz` 5 → 2. Single-value config change, no structural code edits. Post-flash acceptance: station RX counter advances continuously within 60 s AND ack_stress first-try ≥ 80% pooled N ≥ 100 at 3-4 ft geometry. **If station RX goes silent after T6, STOP — do not proceed to T7. Escalate to T5 root-cause.**

2. **IVP-T5 (parallel to T6, mandatory before T8):** Diagnose T2/T3 silent-RX bug. Primary hypothesis: SX1276 driver's `begin()` does not unconditionally write full modem config on re-init. Audit the register-write sequence. Confirm ArduPilot's full-rewrite pattern. Write a regression test that builds with a benign `#ifdef` gating a constexpr value and verifies RX works — this becomes the canary for future structural changes.

3. **IVP-T7 (Option F — transitional palliative, after T6):** Shorten retry interval 3 s → 500 ms. Only valid after A lands (500 ms retry makes no sense at 5 Hz nav where the 200 ms period is already close to the collision window). Post-T6 the ~360 ms clear window gives 500 ms retry a healthy match. Target: mean successful-command latency < 2 s.

4. **IVP-T8 (Option E — long-term ARQ, only after T5 resolves):** Wire CCSDS COP-1 + CLCW. Structural change on hot path; must wait for T5 diagnosis. Mandatory code review includes LL-32 (no blocking in AO handlers) and LL-35 (static events for QACTIVE_POST) compliance verification.

5. **IVP-T9 (contingent, SKIP unless T6 fails):** Option B (250 kHz BW) only invoked if T6 shows 2 Hz is too slow for an unforeseen reason. Not expected to be needed.

6. **IVP-T10 (stage exit):** Soak test at final config (A + F + E). Exit criteria: first-try ACK ≥ 80% pooled N ≥ 100 at 3-4 ft, fail-after-3-retries ≤ 1%, mean successful-command latency < 2 s, zero CRC errors, station RX counter advances continuously across soak, T4 ambient re-run shows noise floor unchanged.

**Option C (A+B hybrid):** DEFER. A alone gives 220 ms margin. B on top buys nothing we need.

**Option D (FDD):** SKIP. Not justified by current data. Note in whiteboard as potential Gemini-tier work.

---

## Dissents

None on the primary fix package or ordering. One soft divergence remained: Cubesat argued B was viable on link-budget grounds at hobby-rocket ranges. NASA/JPL and ArduPilot Core preferred A for the larger margin (220 ms vs 30 ms). Rocketeer broke the tie on operational UX grounds (2 Hz nav is operationally fine; 30 ms timing margin is fragile). B stays in reserve.

---

## Exit criterion for Stage T

Four conditions, all required, measured at IVP-T10:

1. **First-try ACK ≥ 80%** pooled over N ≥ 100 commands at 3-4 ft operational geometry with the final config (A + F + E).
2. **Fail-after-3-retries ≤ 1%.**
3. **Mean successful-command latency < 2 s.**
4. **Station RX packet counter advances continuously** across the N ≥ 100 soak (no silent-RX regression).

Plus instrumentation: zero CRC errors across all RF events, T4 ambient re-run confirms noise floor unchanged.

---

## Red flags for the implementer

1. **T6 post-flash RX-alive check is load-bearing.** If the station RX counter is stuck at boot after T6 lands, DO NOT continue. A numeric config change must not break RX. If it does, the T2/T3 silent-RX bug class is wider than suspected and T5 becomes urgent.

2. **T5 primary hypothesis (per NASA/JPL + ArduPilot):** `SX1276::begin()` may not unconditionally rewrite full modem config on re-init. Audit before searching wider. Cross-check against ArduPilot's `AP_Radio_sx1276.cpp` full-rewrite pattern.

3. **LL Entry 32 compliance on T8:** COP-1 state-machine timers must post events, not block. Any spin-wait or sleep inside an AO handler is a crash waiting to happen (reference: ao_telemetry / rfm95w_send 150 ms block → qf_actq id=130).

4. **LL Entry 35 compliance on T8:** Every `QACTIVE_POST` in new ARQ code must pass a `static` or `Q_NEW` event, never a stack-local `QEvt` subclass. Grep `QACTIVE_POST` in the diff, inspect each call site.

5. **Do not conflate "compiled and linked clean" with "RX works."** T2 and T3 both compiled and linked clean. Both broke RX. Post-flash runtime verification (station RX counter check, ack_stress smoke test) is mandatory before marking any radio-path IVP complete.

6. **Exit criterion #4 (RX counter advances continuously) must be measured independently of ack_stress.** Run a 60-second ambient/idle poll at the start of T10 soak before command stress begins. If RX counter is stuck before the first command, the silent-RX bug regressed.

7. **2 Hz nav rate validation:** before committing T6, confirm no Mission Profile abort guard or station-side timeout depends on receiving vehicle nav updates at ≥ 5 Hz. Grep `nav_rate_hz` and any hardcoded `200` ms expectations in station-side handlers. NASA/JPL's FMEA nit from the transcript.
