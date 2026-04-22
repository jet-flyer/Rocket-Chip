# Stage T Batch B IVP-T14d — Wilson CI attempts (NON-RESULT)

**Status: DNF.** Three attempts, none produced a trusted Wilson 95% CI
number for the first-try-ACK gate. Recording the attempts here so
future re-runs start from clear ground, not from "why wasn't this
done yet?"

## Why it's a non-result, not a pass or fail

The Stage T plan's gate is "N=100 Wilson 95% CI lower bound ≥ 95%
first-try ACK rate." All three measurement approaches hit a different
instrumentation limitation before producing clean data. Quoted
"first-try rate" numbers below are under-reports or artifacts of the
harness, not a real link-health verdict.

Operational reality observed informally:
- RSSI LEDs green, TRACK state, LQ 100%, 0 packet loss over minutes
- Dashboard RF Link row reports kTrack consistently
- Manual `X` (DISARM) from kMenu usually ACKs within ~2 retries
- 13/14 commands that *fully resolved* in attempt 3 were ACK'd (92.9%)
- The CCSDS-rework STOP-GAP framing means we'd want to re-measure
  under the eventual CCSDS command layer anyway

## Attempt 1 — `scripts/ack_stress_test.py`, 1 s interval

**CSV:** `t14_wilson_n100.csv`

- Sent 100 commands, script reported 3 ACK'd, 0 failed, 97 unaccounted.
- **Root cause of non-result:** script's `RE_RETRY` regex was stale
  (`[CMD] Retry N (seq=N)` format, not `Retry N/M (seq=N)` after
  T14d). Regex fix landed; this run is the pre-fix baseline.

## Attempt 2 — same script, regex fixed, 3 s interval

**CSV:** `t14_wilson_n100_v3.csv` (and `_v2.csv` at 1 s interval for
completeness)

- Sent 100, script captured 286 retry log lines, 3 ACK'd, 0 failed,
  97 unaccounted.
- **Root cause of non-result:** serial-output races between station
  [CMD] prints and diag-dump stream. Script scraped lines reliably
  for first few commands but fell behind under the retry-log burst.
  Raw-log grep showed 99 commands reached "Retry 1/8", 95 reached
  "Retry 2/8", 92 reached "Retry 3/8", 0 reached "Retry 4+/8" or
  "No ACK after 8 retries" — implying station was ACKing somewhere in
  the retry-3-to-retry-4 window but the scraper missed it.

## Attempt 3 — `scripts/stage_t_wilson_ci.py` via firmware counters, 5 s interval

**CSV:** `t14_wilson_fw.csv`

Deterministic approach: snapshot `AO_Telemetry_get_retry_stats()` DISARM
row before/after via `q -> d` diag dump; diff.

Test-window delta:
| metric              | delta |
|---------------------|------:|
| fw_sent             | 100   |
| first_try           | 0     |
| retry_rescued       | 13    |
| failed              | 1     |
| total_retries_used  | 49    |

- 14 commands resolved cleanly, 86 "lost": dedupe-replaced by the
  subsequent send before they finished their retry window.
- 5 s interval was NOT long enough for each command to resolve before
  the next arrived. Retries + ACK wait + retry timer interactions mean
  typical resolution takes >5 s for a non-first-try ACK.
- The T14a MAV_CMD dedupe (newest-wins) is working as designed — it
  just means sending commands faster than they resolve silently
  overwrites the previous pending cmd with no counter accounting for
  the displaced one.
- **Root cause of non-result:** our test cadence was shorter than
  average command resolution latency under aggressive retry
  (8 × 250 ms). Would need ≥ 10 s interval, 15+ min run for N=100,
  with a dedupe-bypass flag or safety-class promotion of DISARM for
  this test purpose.

## Calculated numbers (provisional, DO NOT cite as the gate)

Wilson 95% CI on first-try / resolved: n=14, k=0 → [0.00%, 21.53%].

Eventually-ACK'd rate: 13/14 = 92.9%.

Neither is the metric the plan's gate specifies.

## What would work on re-run

1. Instrument a `dedupe_replaced` counter in `s_retry_stats` so
   displaced commands are tracked (simple addition in `populate_pending`).
2. OR bypass dedupe entirely for the test (temp-promote DISARM to
   safety-class, or add a `--no-dedupe` compile flag).
3. OR wait until CCSDS command layer lands and re-run against that —
   the flow-control semantics will make "first-try" meaningful in a
   way they aren't under MAVLink-over-LoRa naive retry.

Path 3 is consistent with the STOP-GAP framing already applied to
this whole command path. Deferring to that.
