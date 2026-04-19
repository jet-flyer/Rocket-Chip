# Stage T — Fix Council Briefing Report

**Date:** 2026-04-19
**Purpose:** Hand the fix council the diagnostic data collected in Stage T
IVPs T1–T4 so they can design the fix (Stage T continuation: IVPs T5+).
**Plan:** `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md`
**Raw data:** `logs/stage_t/*.csv`, `*.log`, per-IVP summaries.

---

## TL;DR for the council

1. **6.1% first-try station→vehicle ACK is real and reproducible.** Pooled
   N=114 at operational geometry (3-4 ft, healthy RF link). Matches the
   original IVP-132a.5 measurement (6.7%) within noise.
2. **The failure is collision-dominant.** Zero CRC errors across 435+ RF
   events. Every packet that reaches the vehicle gets ACKed. The loss is
   entirely station→vehicle delivery.
3. **Sync-only fixes cannot close the gap.** T2 cheat-mode (host with
   perfect knowledge of vehicle RX windows) scored 0% first-try vs 6.1%
   baseline. Root cause: **station LoRa TX airtime (~140 ms) exceeds
   vehicle kRxWindow (~100 ms).** No sync choice makes a 140 ms packet
   fit in a 100 ms window.
4. **Interference is not a factor.** T4 confirmed zero detectable LoRa
   activity on the band with vehicle off, noise floor at -114 dBm
   (9 dB above SF7 sensitivity threshold).
5. **The fix must include an RF or cadence change**, not just a protocol
   change. See §6 for options.

---

## 1. Baseline characterization (IVP-T1)

**Protocol:** 5 runs × N=30 DISARM commands @ 10 s spacing. Run 1 at
<1 ft (control, matches IVP-132a.5 conditions). Runs 2-5 at 3-4 ft
operational.

**Vehicle instrumentation (`ROCKETCHIP_STAGE_T_LOGGING=ON`):** every
RadioPhase state transition + per-RX metadata (RSSI, SNR, CRC status,
state at arrival, seq) logged to serial with µs timestamps.

### Per-run results

| Run | Geometry | N  | ACK | First-try | r1 | r2 | r3 | Failed | First-try |
|-----|----------|----|----:|----------:|---:|---:|---:|-------:|----------:|
| 1   | <1 ft    | 28 | 10  | 2         | 1  | 3  | 4  | 0      | 7.1%      |
| 2   | 3-4 ft   | 28 | 9   | 1         | 2  | 2  | 4  | 0      | 3.6%      |
| 3   | 3-4 ft   | 28 | 14  | 3         | 2  | 5  | 4  | 0      | 10.7%     |
| 4   | 3-4 ft   | 29 | 5   | 1         | 0  | 3  | 1  | 1      | 3.4%      |
| 5   | 3-4 ft   | 29 | 11  | 2         | 1  | 3  | 5  | 1      | 6.9%      |

### Pooled 3-4 ft (Runs 2-5, N=114)

| Metric     | Value           |
|------------|-----------------|
| Total ACK  | 39 (34.2%)      |
| First-try  | 7 (**6.1%**)    |
| Retry 1    | 5               |
| Retry 2    | 13              |
| Retry 3    | 14              |
| Failed (3 retries) | 2 (1.8%) |

Run-to-run variance: first-try mean 6.2%, sd 3.4%, range [3.4, 10.7]%.

### Vehicle ground truth

| Run | Vehicle RX CRC-OK | Vehicle RX CRC-err |
|-----|------------------:|-------------------:|
| 1   | 10                | 0                  |
| 2   | 9                 | 0                  |
| 3   | 14                | 0                  |
| 4   | 5                 | 0                  |
| 5   | 11                | 0                  |

**Vehicle RX count exactly matches station ACK count per run.** Zero CRC
errors in 5 runs × ~140 commands × up to 4 attempts ≈ 435 RF events.
Every received packet got ACKed. Return path (vehicle→station) is
perfect. All loss is station→vehicle.

### Hypothesis-by-hypothesis disposition

| Hypothesis | Status |
|------------|--------|
| Collision-dominant (vehicle in TX during station TX) | **Confirmed.** Retry distribution uniform/back-loaded (matches ~50% TX duty cycle random collision). |
| Link-budget / geometry | **Refuted.** <1 ft (-33 dBm saturated) and 3-4 ft (-67 dBm clean) give indistinguishable rates. |
| CRC errors from weak link | **Refuted.** Zero CRC errors across ~435 RF events. |
| Parser/handler dropping clean packets | **Refuted.** 49 received = 49 ACKed. Perfect 1:1. |
| Retry aliasing with vehicle cycle | **Refuted.** Flat retry-slot distribution (attempts walk across ~45 phase positions over 9 s). |
| Interference / shared spectrum | **Refuted (T4).** Zero detectable LoRa activity at noise floor -114 dBm. |

---

## 2. Sync ceiling (IVP-T2)

**Protocol pivot:** original plan called for station firmware cheat-mode
(`ROCKETCHIP_STAGE_T2_CHEAT=ON`). Firmware compiled and linked cleanly
but broke station RX at runtime (station booted, RSSI LED tracked
carrier, but decoder never fired). Same class of bug hit T3 (below).
Pivoted to **host-side cheat** (`scripts/stage_t2_cheat.py`) — host tails
vehicle serial for `[STAGE_T] state TX->RXW` transitions and fires `X` on
station serial within a configurable delay. No firmware change. Station
ran production build throughout.

**Result (N=30, delay_target=10 ms, actual ~67 ms due to Python slop):**

| Metric | T2 cheat | T1 baseline pooled |
|--------|---------:|-------------------:|
| First-try ACK | **0 (0.0%)** | 7 (6.1%) |
| Total ACK | 1 (3.3%) | 39 (34.2%) |
| Failed (3 retries) | 1 (3.3%) | 2 (1.8%) |

**Cheat-mode did WORSE than random.** Council had predicted >95%. The
gap between prediction and reality is the finding.

### Why cheat-mode failed

| Quantity | Measured value |
|----------|----------------|
| Vehicle kRxWindow duration | ~100 ms (p10=99.9, p50=100.3, p90=101.4 ms) |
| Station LoRa TX airtime (SF7/BW125/CR4-5, 54 B) | ~140 ms |
| Overlap into next vehicle kTxActive | ~40–107 ms depending on timing |

T2 fired at `TX→RXW + 67 ms`, guaranteeing station TX extends 107 ms past
the vehicle kRxWindow end into vehicle's next kTxActive. Vehicle radio is
in TX mode during that ~40 ms overlap, cannot receive, packet truncated.
This is worst-case.

Physical confirmation from the operator during the test: station RSSI
bar fluctuated 4-5 bars (vs solid 6 bars during T1) — synchronized
collisions were visibly disrupting station's reception of vehicle downlink.

### The fundamental constraint

**Station LoRa TX airtime (140 ms) > vehicle kRxWindow (100 ms).** No
single station TX fits inside a single vehicle kRxWindow. At least
~40 ms of every station TX extends into the vehicle's next kTxActive
and is lost.

The ~6% first-try success in T1 baseline happens because packets that
**start early enough in the vehicle kRxWindow** complete before the next
vehicle TX begins. T1 saw that sliver randomly; T2 forced the opposite.

---

## 3. Protocol A/B (IVP-T3)

**BLOCKED.** Same failure mode as T2 firmware cheat. Station built with
`ROCKETCHIP_STAGE_T3_MAVLINK=ON` (flips `kDefaultRocketRadioConfig.protocol`
from `kCcsds` to `kMavlink`) compiled and linked clean, booted, tracked
RF carrier, but `rx_count = 0` at decode layer. Not root-caused.

**Both T2 and T3 firmware variants share the pattern:** compile-flag-
gated change on the radio hot path → RSSI OK, decode broken. Worth
investigating together in a future session, but not blocking the fix.

**Impact on fix design:** minor. T2 already showed window geometry
dominates; framing is third-order. Without T3 data we lose a post-fix
benchmark comparison point but not a foundation for the fix decision.

---

## 4. Ambient RF (IVP-T4)

**Reduced scope** (CAD feasibility deferred — requires firmware driver
additions, same risk class as T2/T3 firmware variants, not attempted).

**Ambient RSSI sweep done:** 60 s with vehicle powered off, station in
kRxContinuous.

| Metric | Value |
|--------|-------|
| New packets in 60 s | **0** |
| CRC errors | 0 |
| Live noise floor | **-114 dBm** |
| SF7/BW125 sensitivity | ~-123 dBm (datasheet) |

9 dB above sensitivity threshold, zero packet decode activity. **No
preamble-compatible LoRa transmitter on band. Interference ruled out as
a contributor to the 6.1% baseline.**

---

## 5. Firmware-variant reliability gap (cross-cutting finding)

Both T2 and T3 exposed a separate issue that the fix council should be
aware of: **compile-flag-gated changes to the radio-path code (ao_radio,
ao_telemetry, radio config) can break runtime RX without producing link
errors or CRC errors**. Station boots, RF front-end works, decoder
silently fails. The pattern:
- T2: added `#ifdef ROCKETCHIP_STAGE_T2_CHEAT` branch to `handle_rx_packet`
  + new queue variables — station RX broke.
- T3: flipped `kDefaultRocketRadioConfig.protocol` value via `#ifdef`
  — station RX broke.

Both code changes look benign; both fail identically. Root cause unknown.
Before the fix council signs off on any station firmware change, this
should be diagnosed — otherwise the fix itself may silently break RX.

---

## 6. Fix options for council consideration

Given the constraint **TX airtime (140 ms) > RX window (100 ms)**, the
fix MUST address at least one of:

### Option A — Reduce vehicle nav cadence
Change `kDefaultRocketRadioConfig.nav_rate_hz` from 5 → 2. Period goes
200 ms → 500 ms, with TX airtime ~140 ms leaving ~360 ms kRxWindow.
Station TX fits with 220 ms margin.

**Pros:**
- One-line config change, already parameterized.
- No protocol work, no new code paths.
- Telemetry bandwidth still adequate for a rocket (apogee nav at 2 Hz
  is plenty; post-apogee altitude descent even less demanding).

**Cons:**
- Lower nav update rate on station dashboard (operator sees ~500 ms
  latency instead of 200 ms).
- Affects every tier equally.

**Risk:** Low. Config was 2 Hz before the 5 Hz bump. Well-exercised code path.

### Option B — Widen LoRa bandwidth
Change `kDefaultRocketRadioConfig.bandwidth_khz` from 125 → 250 kHz.
Cuts airtime roughly in half (~140 → ~70 ms). Fits comfortably in 100 ms
kRxWindow even at 5 Hz nav.

**Pros:**
- Keeps 5 Hz nav cadence.
- One-line config change.

**Cons:**
- Sensitivity drops ~3 dB (range penalty). At 3-4 ft bench this is
  invisible; at 1 km field range it could matter.
- Higher BW = more susceptible to narrowband interference if it shows up
  (T4 showed none today, but we're in an anechoic-ish basement-equivalent
  setup).

**Risk:** Low-medium. Sensitivity trade should be modeled against worst-
case field range.

### Option C — A+B hybrid
2 Hz nav + 250 kHz BW. Very comfortable timing margin, retains range.

**Pros:** Most conservative timing, keeps nav usable.
**Cons:** Loses both faster nav AND sensitivity.

### Option D — Frequency-division duplex
SX1276 supports frequency hopping; use separate freq channels for
vehicle→station and station→vehicle, no half-duplex constraint.

**Pros:** Fully eliminates the collision class.
**Cons:**
- Doubles the spectrum footprint (regulatory consideration).
- Requires radio config change per direction — significant firmware work.
- Either needs TWO SX1276s (cost, complexity) or hop on one (timing
  complexity on top of the existing scheduler).

**Risk:** High. Non-trivial design work. Overkill if A or B solves it.

### Option E — CCSDS COP-1 + CLCW (ARQ improvement)
Does NOT solve single-packet first-try. Does solve retry efficiency:
station retransmits based on downlinked CLCW acknowledging windows, no
blind 3-second retry timeouts. Sliding window improves throughput.

**Pros:**
- Operationally transparent to the user (commands "just work" once the
  window opens).
- Standard protocol — leveraged flight-proven pattern.
- Orthogonal to A/B — can be added on top of any RF change.

**Cons:**
- Significant protocol implementation work.
- Won't help if RF config is still broken (140 ms in 100 ms window).

**Risk:** Medium protocol complexity. Worth doing AFTER A or B.

### Option F — Accept multi-attempt, tune current retry timer
Current retry timer is 3 s. At 1.8% fail-after-3-retries, average
delivery latency on successful commands is ~6 s. Shorten retry interval
to ~500 ms (~half the TX cycle period) — this reduces average latency
to ~1-2 s without changing RF config.

**Pros:**
- Trivial change (one constant in `ao_telemetry.cpp`).
- No firmware-variant risk (well-known code path).
- Improves perceived responsiveness without touching RF.

**Cons:**
- Still leaves 6% first-try rate — doesn't fix the fundamental problem,
  just masks it better.
- More RF airtime consumed per command (more retries per second).

**Risk:** Low. Could be a short-term palliative while A/B is being tested.

---

## 7. Recommended fix package (pre-council, for discussion)

From T1+T2 findings, a single good fix package is:

1. **Option A (2 Hz nav cadence)** — definitive, low-risk, restores
   timing margin. Primary fix.
2. **Option E (COP-1 + CLCW)** — efficient ARQ on top. Secondary fix.
3. **Option F (shorten retry timer)** — short-term palliative while A+E
   are being tested on the bench. Tertiary/transitional.

Each of A, E, F is independent and reversible. Build and test in order.
Each lands as its own IVP-T<n+1> step in the Stage T continuation.

Before any station firmware change: **diagnose the T2/T3 firmware-variant
RX-break issue** (see §5). If the fix is a radio-path config change, we
need confidence it won't silently break RX in the same way.

---

## 8. Proposed continuation IVPs for the council to pick from

Numbering continues Stage T from T4:

- **IVP-T5:** Diagnose T2/T3 firmware-variant RX-break (prerequisite).
- **IVP-T6:** Option A — drop vehicle nav cadence to 2 Hz, re-baseline
  ack_stress (target: first-try > 80%).
- **IVP-T7:** Option F — shorten retry timer 3 s → 500 ms, re-baseline
  (target: average successful delivery latency < 2 s).
- **IVP-T8:** Option E — wire CCSDS COP-1 + CLCW, re-baseline (target:
  efficient ARQ, no redundant retries on already-received commands).
- **IVP-T9** (contingent): Option B (250 kHz BW) if range penalty
  acceptable — revise post-IVP-T6 if 2 Hz is too slow for the use case.
- **IVP-T10** (stage exit): soak test at final config, document fixes,
  update PROJECT_STATUS / CHANGELOG / IVP.md.

Council to confirm / adjust / reorder.

---

## 9. Files / data index

All under `logs/stage_t/`:

| File | Contents |
|------|----------|
| `README.md` | Schema documentation |
| `t1_summary.md` | Full T1 classification and analysis |
| `t1_run{1..5}.csv` | Per-command records |
| `t1_run{1..5}.log` | Full station transcripts |
| `t1_vehicle_run{1..5}.log` | Vehicle `[STAGE_T]` captures |
| `t2_summary.md` | T2 cheat-mode analysis + window-geometry finding |
| `t2_cheat.csv` | N=30 per-command records |
| `t2_cheat.log` | Station transcript |
| `t2_cheat_vehicle.log` | Vehicle state-transition log |
| `t3_summary.md` | T3 block documentation |
| `t4_summary.md` | Ambient RSSI / interference analysis |
| `t4_ambient.csv` | 23-sample poll, 60 s |

Harness + analysis scripts (in `scripts/`):
- `stage_t_run.py` — T1 parallel station+vehicle capture
- `stage_t_summarize.py` — T1 aggregation
- `stage_t_rx_timing.py` — RX window timing extraction (T2 gate sizing)
- `stage_t2_cheat.py` — T2 host-side cheat harness
- `stage_t4_ambient.py` — T4 ambient poller
- `ack_stress_test.py` — extended with CSV output + per-retry counters

Firmware code (Stage T instrumentation + disabled variants):
- `src/active_objects/ao_radio.cpp` — `ROCKETCHIP_STAGE_T_LOGGING` gated
  state + RX logging (production-ready diagnostic)
- `src/cli/rc_os_commands.cpp`, `src/active_objects/ao_telemetry.cpp` —
  `ROCKETCHIP_STAGE_T2_CHEAT` gated queue (OFF, broken, for future debug)
- `src/flight_director/mission_profile_data.h` — `ROCKETCHIP_STAGE_T3_MAVLINK`
  gated protocol flip (OFF, broken, for future debug)
- `CMakeLists.txt` — option definitions

---

*End of report. Council convenes with this data to design the fix.*
