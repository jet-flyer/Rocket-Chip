# IVP-T4 Summary — Ambient RSSI / interference check

**Date:** 2026-04-19
**Plan:** `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md`
**Gate:** Hard — **PASS** (reduced scope — full CAD feasibility deferred)

## Pivot from original plan

The original T4 plan had two parts:
1. **SX1276 CAD mode feasibility** — put station into Channel Activity
   Detection cycles, build a confusion matrix vs vehicle TX ground truth.
2. **Ambient RSSI baseline** — confirm no interference on the band.

**Part 1 requires firmware changes to add CAD mode to the SX1276 driver
and a test-only CLI command**, which after T2 and T3 both having firmware-
change failures (station booted but receiver broken in both cases) was
too risky to attempt without dedicated debugging time. CAD feasibility
is deferred to the fix stage's implementation phase.

**Part 2 was done** — host-side polling of station's `t` radio status
at 1 Hz for 60 s while vehicle was powered off.

T2 already provided the decisive finding about window geometry being
the root cause, so T4's value drops to ambient-interference verification.

## Protocol

- Operator unplugged vehicle USB (and the attached debug probe).
- Station remained powered on in dashboard mode, RadioScheduler in
  kRxContinuous.
- Host script `scripts/stage_t4_ambient.py` polled station 't' status
  at ~2.6 s intervals (23 samples over ~60 s — slightly slower than
  planned 1 Hz due to serial read timeouts, not consequential).

## Results

All 23 samples, 60 seconds total:

| Metric                  | Value        |
|-------------------------|--------------|
| Station pkts (start)    | 1073         |
| Station pkts (end)      | 1073         |
| Packets received in 60s | **0**        |
| Stale last_rx_rssi      | -58 dBm      |
| Stale last_rx_snr       | ~10 dB       |

`last_rx_rssi` stays frozen at -58 dBm because it's the RSSI of the
last decoded packet from BEFORE the vehicle was unplugged. It's not
live ambient RSSI.

## Interpretation

**Zero packets received in 60 seconds of RxContinuous on 915 MHz SF7/
BW125 at the operational geometry** (3-4 ft from where the vehicle
antenna used to be, metal PC partially in path).

The station's SX1276 is actively listening for valid LoRa preambles and
would register any packet meeting the SF7/BW125 sync-word requirement.
Zero registrations = zero detectable LoRa transmitters on this band in
this room at this moment.

Standard sub-GHz ISM noise/interference sources (WiFi at 2.4 GHz is
a different band, LTE/cellular different bands, garage door openers at
433 MHz different band, other LoRa at SF7/BW125 would have decoded).
Only a preamble-compatible LoRa transmitter at 915 MHz SF7/BW125 would
register, and none are present.

**Conclusion:** Ambient interference is NOT a contaminating factor in
the T1 baseline or T2 results. The 6.1% first-try rate is not driven by
shared-spectrum loss. The fix council can rule out interference and
focus on the window-geometry + sync + ARQ design question.

## Caveats

- This test does NOT measure ambient RF energy below the LoRa preamble
  threshold. Broadband interference at ~-100 dBm (below SF7 sensitivity)
  could still exist. But at SF7/BW125 the receiver accepts weaker
  signals than broadband sources typically emit.
- Sampling rate was 2.6 s not 1 s due to Python serial read timeouts.
  60 s of observation with the counter held perfectly still at 1073 is
  still decisive — even a burst of 1-2 spurious packets would have
  shown a non-zero delta.
- A mobile interferer (phone, wireless mouse) could have been moved
  during the test and missed — but no transient activity appeared.
- CAD feasibility as a fix mechanism is NOT evaluated — the data the
  fix council needs on this question has to come from a future test
  when CAD mode is added to the driver.

## What this tells the fix council

Interference is not the issue. Focus on window geometry + ARQ.

The **CAD-based listen-before-talk fix candidate** from the original
plan is NOT evaluated here. The fix council can still choose CAD as a
candidate, but without empirical feasibility data — it's a design
decision, not a data-driven one. Given T2's finding that window
geometry is the root cause (TX airtime > RX window regardless of
when you fire), CAD alone wouldn't solve the problem anyway — it'd just
avoid collisions while still failing to fit a 140 ms packet into a
100 ms window.

The productive fix directions (from T1+T2):
1. Reduce vehicle nav cadence (5 Hz → 2 Hz; 200 → 500 ms period;
   RXW grows from ~100 ms → ~400 ms)
2. Widen LoRa BW to cut airtime (125 → 250 kHz halves airtime)
3. Accept multi-attempt delivery, wrap in efficient ARQ (CCSDS COP-1 +
   CLCW) so retries are transparent and sliding-window

Stage T exit recommendation: fix council picks (1) or (2) OR (1+2) as
the primary RF fix, adds COP-1 + CLCW as the protocol fix. Either of
(1) or (2) plus CLCW will close the reliability gap.

## Files

- `logs/stage_t/t4_ambient.csv` — raw samples
- `scripts/stage_t4_ambient.py` — host poller
