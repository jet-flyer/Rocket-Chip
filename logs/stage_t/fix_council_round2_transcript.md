# Stage T Fix Council — Round 2 Transcript

**Date:** 2026-04-19 (same session, second convening)
**Convened to:** re-rank fix options under user's new hard constraints
**Panelists:** NASA/JPL Lead, ArduPilot Core, Hobbyist Rocketeer, Cubesat Engineer
**Input:** round 1 transcript + user's round-2 constraints (10 Hz nav target, 2 km LOS, T3 retest, wider sweep, E endorsed)

---

**NASA/JPL Lead:** Let me open by owning round 1's mistake. We converged on Option A (5→2 Hz) in 90 minutes and everyone signed it. The user came back and said 2 Hz is operationally inadequate — and he's right. I anchored on "apogee is vehicle-side, station just watches" and missed that **boost phase is ~3 seconds total for medium HPR**. At 2 Hz that's 5-6 samples spanning ignition, max-Q, burnout. The operator and post-flight analysis both lose the most dynamic 3 seconds of the flight. At 10 Hz that's 30 samples. This is the phase where you most need resolution, and round 1 threw it away for scheduler convenience. My FMEA framing was wrong — it measured margin on the radio, not information margin on the mission. Let me reset.

**ArduPilot Core:** Same mea culpa here. The 140 ms / 100 ms window geometry is still the constraint, but we need to attack it from the RF side now, not the cadence side. If 10 Hz nav is the target, vehicle TX period is 100 ms, and of that roughly half needs to be `kRxWindow` for station commands. So we need **station TX airtime < ~40-50 ms** to fit comfortably with jitter margin. At SF7/BW125/CR4-5 with 54-byte payload we're at 140 ms. We need to cut airtime by ~3×. That's the target number, and it tells us where to look.

**Cubesat:** Good, let me put numbers on the RF design space. LoRa symbol time `T_sym = 2^SF / BW`. Airtime scales roughly linearly with `T_sym` for a fixed payload. Let me walk through it for a 54-byte CCSDS frame with 8-symbol preamble, explicit header, CR4/5, CRC on:

| Config | T_sym | Payload symbols | Total airtime | Sensitivity (datasheet) |
|--------|-------|-----------------|---------------|-------------------------|
| SF7 / BW125 (current) | 1.024 ms | ~115 | **~140 ms** | -123 dBm |
| SF7 / BW250 | 0.512 ms | ~115 | **~70 ms** | -120 dBm |
| SF7 / BW500 | 0.256 ms | ~115 | **~35 ms** | -117 dBm |
| SF6 / BW125 | 0.512 ms | ~95 | **~60 ms** | -118 dBm (implicit hdr only) |
| SF6 / BW250 | 0.256 ms | ~95 | **~30 ms** | -115 dBm |
| SF5 / BW125 | 0.256 ms | ~80 | **~35 ms** | -115 dBm (SX1276 doesn't support SF5 cleanly — SX1262 only) |

SF5 is out — SX1276 silicon doesn't support it. SF6 is supported but forces implicit header mode, meaning both ends must agree on payload length a priori. That's a big protocol change.

**Rocketeer:** Can I interject on SF6 before we go further? Implicit header means fixed-length packets, and our command frames aren't all the same length — DISARM is short, telemetry-ack might be longer, future commands might vary. Going to implicit header to save 40 ms of airtime locks us into one frame size forever. That's a bad trade for the platform. I'd say SF6 is off the table on protocol grounds, not just complexity.

**ArduPilot Core:** Agreed. ArduPilot's SX1276 driver stays at SF7 explicit header for exactly this reason — the flexibility is worth more than the airtime. So we're choosing between BW 250 and BW 500, both at SF7.

**Cubesat:** Link budget time. 2 km LOS at 915 MHz:

- **Free-space path loss:** `FSPL = 32.45 + 20·log10(f_MHz) + 20·log10(d_km)` = 32.45 + 59.22 + 6.02 = **97.7 dB at 2 km**.
- **TX power:** +20 dBm (SX1276 max, legal under FCC Part 15 at 915 MHz ISM with duty cycle).
- **Antenna gain:** ~0 dBi each end (dipole; realistic for hobby).
- **Received power at 2 km:** 20 + 0 - 97.7 + 0 = **-77.7 dBm.**

Now margin against sensitivity:

| Config | Sensitivity | Received @ 2 km | Margin |
|--------|-------------|-----------------|--------|
| SF7/BW125 | -123 dBm | -77.7 dBm | **45.3 dB** |
| SF7/BW250 | -120 dBm | -77.7 dBm | **42.3 dB** |
| SF7/BW500 | -117 dBm | -77.7 dBm | **39.3 dB** |

All three close 2 km LOS with 39+ dB of margin. Even with 20 dB of fade/multipath/antenna-pattern loss in a real field environment, you're at 19-25 dB margin. **At 2 km LOS, BW choice is not limited by link budget.** It's limited by interference susceptibility and regulatory footprint.

**NASA/JPL Lead:** That shifts my thinking significantly. If BW 500 closes 2 km with ~40 dB margin, the trade is no longer sensitivity — it's **spectrum occupancy and interference vulnerability**. A 500 kHz channel is 4× wider than 125; you cross four times as many potentially-interfering signals, and you're 4× more vulnerable to narrowband blockers. That matters at a launch range where HAMs, weather balloons, or other HPR teams might share the ISM band.

**Rocketeer:** True at a busy launch site. At TRA regional launches there's always somebody running 915 MHz telemetry. But — and this is where it gets interesting — **with 35 ms airtime at BW 500, your duty cycle drops to ~7% at 10 Hz** (35/500 = assuming half-duplex with vehicle TX similarly short). That means even if you're sharing spectrum, you're only on-air 7% of the time instead of 28%. Collision probability with other users drops roughly proportionally. BW 500 is actually MORE polite to shared spectrum in duty-cycle terms even though the channel is wider.

**Cubesat:** That's the counterintuitive point. Wider BW + shorter airtime net out to roughly the same energy-on-band but much better time-sharing. I've seen this play out on cubesat LoRa links — 500 kHz with fast transmission beats 125 kHz slow in a contested environment.

**ArduPilot Core:** Let me propose the lead candidate, then we stress-test it. **BW 500 kHz, SF7, keep CR4/5, keep 10 Hz nav.** Station airtime ~35 ms, fits in a ~50 ms RX window with 15 ms jitter margin. Vehicle airtime also ~35 ms, so at 10 Hz (100 ms period) we have 35 ms vehicle TX + ~50 ms RX window + 15 ms guard = comfortable. Link budget closes 2 km with 39 dB margin.

**NASA/JPL Lead:** FMEA concern on 15 ms margin. QF tick jitter, SX1276 FIFO-write latency, ISR tail — those add up. At SF7/BW125 round 1 wanted 220 ms margin. 15 ms is 15× tighter. I want data, not theory, before I sign this.

**Cubesat:** So we sweep it. 30 samples at BW 250, 30 at BW 500, re-measure airtime and jitter under load. That's 10 minutes of bench time. And while we're at it, we measure actual airtime on the probe — LoRa datasheet numbers assume no implementation overhead, but SX1276 FIFO setup + DIO polling adds real time.

**Rocketeer:** Let's talk about **decoupled nav/TX rate**. The brief flags this as architectural. My read: the ESKF runs at 200 Hz internally already, so "nav rate" in the current config is really just the TX cadence. Vehicle logs at whatever rate its logger wants — doesn't have to match radio TX. If we internally log 10+ Hz nav to flash and TX nav at e.g. 5 Hz over the air, the post-flight analysis resolution is unchanged and the RF pressure drops.

**ArduPilot Core:** That's architecturally clean but it's not what the user asked for. He said "nominal nav rate target ~10 Hz" — the important word is the **operator sees 10 Hz**, not just the flash log does. If the station dashboard updates at 5 Hz while the flash has 10 Hz, the operator can't see boost-phase resolution live. Post-flight yes, live no. That may be acceptable but it's a downgrade from "10 Hz nav rate" as stated.

**NASA/JPL Lead:** Semantic question worth pinning down. What does the user actually need at 10 Hz — bits-in-the-air or bits-on-the-dashboard? If it's the dashboard, decoupled doesn't help. If it's mission data fidelity, decoupled is the right answer. Let's flag this for the user and not decide in council.

**Rocketeer:** I'd lean dashboard. A rocketeer's perception of the flight IS the telemetry stream. If you tell me "don't worry, it's in the flash log" during boost, I'm going to say "yeah but my LCD is frozen at the most exciting 3 seconds of the flight." Decoupled is a fallback if we can't make 10 Hz work over air — not the primary fix.

**Cubesat:** Good, decoupled stays as a contingency. Primary path is close 10 Hz at the air layer.

**ArduPilot Core:** Back to T3 retest. Round 1 let T3 go because it was blocked by the same silent-RX bug that hit T2. Round 1's disposition was "framing is third-order, window geometry dominates." That's still true for collision rate. But T3 measures something important: **does MAVLink framing's shorter payload reduce airtime enough to matter at BW 125?** MAVLink v2 minimum frame is ~14 bytes overhead; our CCSDS frame has ~20 bytes overhead. At 54-byte total, CCSDS is ~34 bytes payload, MAVLink equivalent might be ~48 bytes payload → ~40 byte frame → airtime drops from 140 ms to ~110 ms. Still doesn't fit 100 ms window. So T3 at BW 125 doesn't save us. **At BW 500 it's moot** — both protocols fit easily.

**NASA/JPL Lead:** So T3 retest sequencing: does it add value? At BW 500 with SF7 the airtime argument disappears for framing. T3 becomes pure "validate MAVLink framing works as a fallback protocol" — useful for ecosystem compatibility but not for solving the collision problem.

**Rocketeer:** I'd still retest T3 because QGroundControl compatibility matters for adoption. That's orthogonal to Stage T's goal though. Move it out of Stage T and into a future stage? Or do it as an independent IVP after T5 unblocks it, not gated on the fix.

**ArduPilot Core:** Agreed. T3 retest happens AFTER T5 unblocks the silent-RX bug, but it's parallel to the fix, not a dependency. Moved to low-priority.

**NASA/JPL Lead:** Let me re-rank. Here's my proposed order:

1. **T5 — silent-RX bug diagnosis.** Still mandatory prerequisite. If a `#ifdef` config change can silently break RX, any RF config change risks the same. Round 1 conclusion stands.
2. **T6-revised — BW sweep.** Three configs: current SF7/BW125, SF7/BW250, SF7/BW500. N=30 per config at 3-4 ft. Measure first-try ACK, airtime, station RSSI, CRC count, RX-alive canary. 15 minutes bench time total.
3. **Pick winner from sweep.** Expect BW 500 wins on first-try; measure the jitter margin to confirm.
4. **T7 — F (retry timer 3s → 500ms)** at new config. Re-measure first-try + latency.
5. **T8 — E (COP-1 + CLCW).** Structural change; protected by T5 findings.
6. **T9 — T3 retest (MAVLink A/B).** Unblocked by T5; validates ecosystem. Doesn't gate stage exit.
7. **T10 — stage exit soak.**

**Cubesat:** Wider sweep question. User said 5 configs = 25 min, 10 configs = 50 min, where's the cutoff? I'd say: SF7 at three BWs plus a repeat of winner-config for variance = 4 runs = 20 minutes. Don't do SF6 (implicit header rules it out). SF5 not supported. CR sweep is second-order; stick with CR4/5 which is the LoRa conservative default. Four configs is plenty.

**ArduPilot Core:** Agreed. Adding a control re-run of the winner is discipline, not overkill. One data-point sweeps lie.

**Rocketeer:** Exit criterion update. Round 1 said ≥80% first-try. At BW 500 with 35 ms in a ~50 ms window we should expect **≥95% first-try**. But bench reality — I'd set the bar at **≥85% pooled N≥100** for stage exit. 80% was too forgiving given the new timing margin.

**NASA/JPL Lead:** Fair. Exit criterion: **≥85% first-try ACK, pooled N≥100, at 3-4 ft bench.** Fail-after-3-retries ≤ 1%, mean latency < 2 s, station RX counter advances continuously, zero CRC errors, ambient noise floor unchanged. And one add: **T4 ambient re-run at new BW** — wider channel means more noise; confirm we're still 9+ dB above sensitivity at BW 500.

**Cubesat:** The user flagged E (COP-1 + CLCW) as "very likely worth implementation costs." Round 1 sequenced it last. Should it move up? Argument for: E benefits apply at any RF config; you could implement E while the RF sweep is happening. Argument against: E is structural, protected by T5, can't ship before T5 resolves. **If T5 resolves early, E and T6 can parallelize.** Otherwise E stays last.

**ArduPilot Core:** I'd say T5 is the critical path. Once T5 resolves — whether that takes a day or a week — E implementation can start immediately and doesn't block the sweep. I wouldn't sequence E before the sweep though because E doesn't change the collision rate; it changes retry efficiency. The user sees "first-try actually works" from the sweep; they see "retries are smarter" from E. First-try is the more visible problem.

**Rocketeer:** And the cost of E compared to the sweep is dramatically different. Sweep is 20 minutes of bench time plus analysis. E is 2-3 sessions of protocol implementation plus tests plus LL-32/LL-35 audit. The user gets 95% of the benefit from the sweep alone. E is the polish layer.

**NASA/JPL Lead:** I accept that. E lands after the sweep resolves the primary collision issue. Concurrent with T5 if T5 drags. So revised:

- **Critical path:** T5 → T6-sweep (4 configs) → pick winner → T7 (retry timer) → T10 exit soak.
- **Parallel track:** T3 retest once T5 unblocks it (informational, not gating).
- **Follow-on track:** T8 (COP-1 + CLCW) once T5 resolves AND sweep picks winner. Lands before T10 exit soak if time permits; otherwise becomes Stage T.1 or a dedicated stage.

**ArduPilot Core:** One more architectural point. The brief asked whether early E implementation could inform T5 diagnostics. I don't think so — E is a new AO or new state machine inside ao_telemetry. T5 is a driver-layer bug. They touch different code. No cross-benefit.

**Cubesat:** Agreed, skip that angle. Keep T5 focused on SX1276 driver `begin()` path.

**Rocketeer:** Final question — at BW 500 with 35 ms airtime, retry timer 500ms. That's 7× the airtime, lots of slack. Can we push retry timer even shorter? Like 200 ms?

**Cubesat:** I'd not go below 500 ms as initial config. The ESKF and vehicle-side nav TX at 10 Hz = 100 ms period. Retry 500 ms = 5 vehicle cycles. That's 5 opportunities to catch an RX window per retry — plenty. Going to 200 ms means only 2 vehicle cycles per retry, and if one got clipped you're re-TXing into the same collision. 500 ms is the sweet spot for now; tune later.

**ArduPilot Core:** Concur. 500 ms stays.

**NASA/JPL Lead:** Consensus forming. Let me state it.

---

## Consensus Verdict (Round 2)

**Primary fix:** **Option B-500** — SF7 / BW 500 kHz / CR4/5, keep 10 Hz nav cadence. Station TX airtime drops from ~140 ms to ~35 ms, fits comfortably inside a ~50 ms vehicle RX window at 10 Hz (100 ms period) with ~15 ms jitter margin. Link budget closes 2 km LOS with ~39 dB margin — well above any realistic field-degradation scenario.

**Secondary fix:** **Option F** — retry timer 3 s → 500 ms at new RF config. Quality-of-life improvement; doesn't gate exit.

**Tertiary fix:** **Option E** — CCSDS COP-1 + CLCW for efficient ARQ. Lands after T5 resolves and RF sweep completes. Strongly endorsed by user; real work but real benefit for robustness.

**Rejected / deferred:**
- **Option A (5→2 Hz nav)** — REJECTED by user. 2 Hz is operationally inadequate for boost-phase resolution (3 s boost = 5-6 samples at 2 Hz vs 30 at 10 Hz).
- **Option C (A+B hybrid)** — rejected with A.
- **Option D (FDD)** — overkill; keep in whiteboard as Gemini-tier candidate.
- **SF6 or SF5** — SF5 unsupported on SX1276; SF6 requires implicit header (fixed-length payloads), unacceptable protocol constraint.
- **Decoupled nav/TX rate** — contingency only, not primary. Re-evaluate if RF layer can't hit 10 Hz.

---

## Revised IVP List (Round 2)

1. **IVP-T5 — Silent-RX-bug diagnosis (prerequisite, carried from round 1).**
   Audit SX1276::begin() register-write sequence. Verify full modem config rewrite on re-init per ArduPilot AP_Radio_sx1276 pattern. Create regression canary: build with a benign `#ifdef` gating a `constexpr` value, verify RX works. This canary gates future structural radio-path work. **Must resolve before T8.**

2. **IVP-T6 — RF sweep (4 configs).**
   N=30 per config @ 3-4 ft geometry, 10 s spacing. ~25 min bench time total.
   - SF7/BW125 (current, control)
   - SF7/BW250
   - SF7/BW500
   - SF7/BW500 repeat (variance confirmation)
   Measure: first-try ACK %, measured airtime (probe), station RSSI mean/stddev, CRC error count, station RX-counter-advances canary across all 4 runs. Select winner on first-try % with jitter margin ≥ 10 ms.
   **Gate:** station RX counter advances on every config. If any config regresses RX, halt → escalate to T5.
   **Also gates:** re-run T4 ambient (60 s noise floor) at winner-config BW. Must show ≥ 9 dB above sensitivity.

3. **IVP-T7 — Retry timer tune.**
   `kAckRetryTimeoutMs` 3000 → 500. Post-sweep config. Re-measure first-try + mean successful-command latency.
   **Gate:** mean latency < 2 s; first-try unchanged from T6 winner.

4. **IVP-T8 — CCSDS COP-1 + CLCW.**
   Only after T5 resolves. Structural change on ao_telemetry hot path. Mandatory LL-32 (no blocking in AO handlers) and LL-35 (static events for QACTIVE_POST) audit in code review. Council spec-check included in IVP acceptance.

5. **IVP-T9 — T3 retest (MAVLink A/B, informational).**
   Unblocked by T5. Validates MAVLink framing as fallback protocol. Not gating Stage T exit — informational only.

6. **IVP-T10 — Stage T exit soak.**
   Final config: SF7/BW500 + retry timer 500 ms + (E if T8 landed in time).
   **Exit criteria (all required):**
   - First-try ACK ≥ 85% pooled N ≥ 100 at 3-4 ft
   - Fail-after-3-retries ≤ 1%
   - Mean successful-command latency < 2 s
   - Station RX packet counter advances continuously across soak
   - Zero CRC errors
   - T4 ambient re-run noise floor ≥ 9 dB above sensitivity at operational BW

---

## Dissents

None on the primary technical direction. Two soft open items flagged for user decision, not council decision:

1. **Decoupled nav/TX rate semantics.** Rocketeer and NASA/JPL noted ambiguity: does "10 Hz nav" mean 10 Hz over the air (what the operator sees on dashboard) or 10 Hz internal rate (what the flash log captures)? Council assumed the former. If the user wants the latter, decoupled architecture becomes primary and RF sweep priority drops.

2. **T8 (COP-1 + CLCW) timing.** Cubesat wanted it potentially parallelized with RF sweep (concurrent with T5). Others preferred sequential — sweep first, then E. Final order: sequential. E lands after the sweep demonstrates first-try at ≥ 85%.

---

## Red Flags for Implementer

1. **BW 500 kHz jitter margin is ~15 ms — tight.** Measure actual airtime on probe, not just datasheet. SX1276 FIFO-write + DIO-polling latency adds real overhead. If measured airtime > 40 ms, reconsider at BW 250 (airtime ~70 ms, fits a ~50 ms window only with a lot of luck) or accept decoupled architecture.

2. **T5 gate is load-bearing (same as round 1 red flag #1).** If the BW config change silently breaks station RX the way T2/T3 did, all subsequent IVPs stall until T5 resolves. Run the RX-alive canary after every config change in T6.

3. **Ambient interference vulnerability at BW 500.** A 4× wider channel catches 4× more potential blockers. T4 ambient re-run is mandatory before accepting the winner config. If the noise floor creeps above -114 dBm at BW 500 in any realistic environment, the margin story changes.

4. **LL Entry 32 on T8 (carried from round 1).** COP-1 state machine timers post events, never block. No spin-waits in AO handlers.

5. **LL Entry 35 on T8 (carried from round 1).** Every new `QACTIVE_POST` in ARQ code uses static or pool-allocated events.

6. **"Compiled clean ≠ RX works" (carried from round 1).** Post-flash runtime verification mandatory on every radio-path IVP. Station RX counter advances is the canary.

7. **2 km is the FLOOR, not a target.** Don't optimize to exactly 2 km margin. At BW 500 we have ~39 dB margin at 2 km → degrades gracefully to ~19 dB at 4-5 km in adverse conditions. That's the real operational window and it's comfortable. If future Mission Profiles push to 5+ km, re-evaluate BW choice.

8. **No combination moves fit in one IVP.** B-500 is T6. Don't bundle B-500 + retry-timer + COP-1 into one mega-change — if it breaks, you can't bisect. Separate IVPs, separate commits, separate verification.
