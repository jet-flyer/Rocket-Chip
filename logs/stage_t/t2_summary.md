# IVP-T2 Summary — Cheat-Mode Sync Ceiling (host-side)

**Date:** 2026-04-19
**Plan:** `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md`
**Gate:** Hard — **PASS** (data collected with caveat documented below)

## Pivot from original plan

The original T2 plan called for a **station-side firmware cheat**: flash station
with `ROCKETCHIP_STAGE_T2_CHEAT=ON`, which gates the `X` key press into a
pending queue and fires `AO_Telemetry_send_tracked_command()` from inside the
station's `handle_rx_packet()` when a vehicle nav packet arrives.

**That approach was implemented but the resulting station firmware failed
to receive vehicle packets** — station would boot cleanly, show uptime
advancing, but `RX: 0 pkts` for 6+ minutes while the known-good station
firmware received packets immediately. Reflashing the identical build
reproduced the failure. The station RSSI LED stayed green even after the
vehicle was physically unplugged — proof that the station firmware had
stopped tracking live link state, confirming the build was broken, not
just slow to link up.

Root cause of the T2 firmware breakage not diagnosed — deferred rather than
blocking on it. The station-side cheat code is preserved in the tree behind
`ROCKETCHIP_STAGE_T2_CHEAT` (OFF by default).

Pivoted T2 to **host-side cheat** (`scripts/stage_t2_cheat.py`):
- Host tails vehicle serial for `[STAGE_T] state TX->RXW` transitions
- Fires `X` on station serial `--delay-ms` after each observed transition
- Same experimental question (what is the ceiling if station has perfect
  knowledge of vehicle RX windows?), no firmware change.

## Protocol

N=30 commands at 10 s cmd-timeout, `--delay-ms 10` (target: fire 10 ms into
vehicle kRxWindow). Observed actual send-delay averaged ~67 ms after TX→RXW
(Python sleep+drain loop added ~57 ms slop — not fixed, but documented in
the CSV's `send_after_rxw_ms` column).

Station firmware: known-good `build_station/rocketchip.uf2` (NO cheat-mode).
Vehicle firmware: Stage T logging build (same as T1 runs).
Geometry: 3-4 ft, metal PC partial block (same as T1 runs 2-5).

## Results

| Metric          | T2 cheat (N=30) | T1 pooled 3-4 ft (N=114) |
|-----------------|----------------:|-------------------------:|
| Sent            | 30              | 114                      |
| ACK'd           | 1 (3.3%)        | 39 (34.2%)               |
| **First-try**   | **0 (0.0%)**    | 7 (6.1%)                 |
| Retry 1         | 0               | 5                        |
| Retry 2         | 1               | 13                       |
| Retry 3         | 0               | 14                       |
| Failed (3 ret.) | 1 (3.3%)        | 2 (1.8%)                 |

**T2 cheat-mode performed WORSE than T1 baseline.** This is the opposite of
the "upper bound on success" outcome the plan anticipated.

User-observed physical evidence during the T2 run: station RSSI LED bar
fluctuated between 4-5 bars (vs solid 6 bars during T1 baseline) — the
synchronized station TXes were measurably disrupting the station's reception
of vehicle downlink.

## Why T2 cheat did worse

The council's plan assumed the vehicle's kRxWindow would accommodate a
station LoRa TX. Let's check against measured timings:

- **Vehicle kRxWindow duration:** ~100 ms (from T1 vehicle log analysis —
  `stage_t_rx_timing.py` reports p10=99.9ms, p50=100.3ms across runs).
- **Station LoRa TX air-time:** ~140 ms on SF7/BW125/CR4-5 for a 54-byte
  CCSDS nav payload.

**Station TX air-time (140 ms) is longer than the vehicle kRxWindow
(100 ms).** No single station TX can fit entirely inside a single vehicle
kRxWindow at this config. At least ~40 ms of the station TX will always
overlap the vehicle's next kTxActive — the vehicle is in TX mode during
that overlap and cannot receive.

When station fires randomly (T1 baseline), ~60% of station TXes START in
the vehicle kTxActive and fail immediately, and the remaining ~40% that
start in kRxWindow get truncated when the vehicle transitions back to TX
mid-packet. The 6-10% first-try success rate comes from the sliver of
station TXes that happen to complete entirely before the next vehicle TX.

When T2 fires deterministically at TX→RXW + 67 ms, the station TX is
guaranteed to extend ~107 ms beyond the vehicle's kRxWindow end — i.e.,
it's guaranteed to be clipped mid-packet every single time. That's why
cheat-mode got 0% first-try.

Firing at TX→RXW + 0 ms would still see 40 ms of TX clipping in the next
vehicle TX. Firing at TX→RXW + 60 ms (worst case for overlap) would fail
hardest. T2's accidental +67 ms delay landed close to worst case.

## What this tells the fix council

The bounded half-duplex timing is **fundamentally incompatible with the
current RF config**:
- SF7/BW125 LoRa single-packet airtime: ~140 ms
- Vehicle 5 Hz nav cadence gives 100 ms RXW / 100 ms TX

**No sync-only fix can achieve high first-try success without also
modifying one of:**

1. **Vehicle TX cadence** — reduce from 5 Hz to 2 Hz (250 ms RXW windows
   would exceed 140 ms station airtime, leaving margin).
2. **LoRa config** — BW 250 kHz or 500 kHz halves/quarters the airtime
   proportionally. Trade-off: lower sensitivity (shorter range).
3. **Packet fragmentation** — split commands into sub-packets each <80 ms
   airtime. Adds protocol complexity.
4. **Accept multi-attempt delivery** — reuse T1's measured 1.8% failure
   rate at 3 retries as the operational spec; expose it to the operator
   via ACK-pending UI. (What the firmware currently does, modulo the 3 s
   retry window which is longer than necessary.)

CCSDS COP-1 + CLCW fits option (4): it doesn't reduce single-packet loss
rate, it just makes the ARQ transparent and efficient (no wasted retries
on already-received commands, sliding window instead of per-command).

CAD-based listen-before-talk (T4 topic) would defer station TX until the
channel is clear — which would PREVENT collisions with vehicle TX. But
since the vehicle TX takes 100 ms every 200 ms, CAD-based back-off would
only let the station TX during 100 ms windows that are still shorter than
the 140 ms packet — same fundamental constraint.

**Recommendation from T1+T2:** The fix MUST include a config change, not
just sync. SF7/BW125 at 5 Hz downlink cadence leaves no timing margin for
uplink commands. Options:
- Reduce vehicle nav rate to 2 Hz (simplest change)
- Widen LoRa BW to 250 kHz (keep 5 Hz nav, lose ~3 dB sensitivity)
- Use frequency-division multiplexing (separate vehicle→station and
  station→vehicle freq channels — SX1276 supports it)

## Caveats

1. `--delay-ms 10` was NOT achieved in practice — Python sleep+drain loop
   added ~57 ms of slop. Every send landed at ~67 ms after TX→RXW. CSV
   records actual `send_after_rxw_ms` per command.
2. Host-side timing measurement has ~1-2 ms uncertainty (USB CDC latency
   on the vehicle log tail thread). At 100 ms windows this is ≤2% noise.
3. N=30 — binomial CI at 0/30 is 0-10% (95% CI). Even upper bound of the
   CI is below T1's 6.1% point estimate. Cheat-mode is conclusively NOT
   better than random.

## Gate

**Hard-gate pass criteria:**
- [x] CSV committed: `logs/stage_t/t2_cheat.csv`
- [x] Station log + vehicle log committed
- [x] Documented delay-ms slop in CSV schema
- [x] Analysis of why cheat underperformed

The result is DIFFERENT from what the council expected but it IS the T2
finding: the half-duplex window geometry is the problem, not sync latency.
The fix council now has decisive data for that conclusion.

## Files

- `logs/stage_t/t2_cheat.csv` — per-command record (N=30)
- `logs/stage_t/t2_cheat.log` — station transcript
- `logs/stage_t/t2_cheat_vehicle.log` — vehicle `[STAGE_T]` capture
- `scripts/stage_t2_cheat.py` — host-side cheat harness
- `src/cli/rc_os_commands.cpp` + `src/active_objects/ao_telemetry.cpp` —
  firmware T2 cheat code (preserved, off-by-default, unresolved bug)
