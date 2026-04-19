# Stage T data logs

Raw diagnostic data from Stage T — RadioScheduler Timing Diagnostics.
See `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md` for the full plan.

## IVP-T1 — Baseline repeatability + failure classification

5 runs × N=30 commands at 10 s interval. First run at <1 ft (control,
reproduces IVP-132a.5 conditions), runs 2-5 at 3-4 ft with metal PC
partially in path.

### Files (per run N)

- `t1_runN.csv` — per-command record from station side:
  `seq, sent_host_t, ack_host_t, ack_retry_slot, retry_count, denied, failed`
- `t1_runN.log` — full station transcript including `[CMD]` firmware lines.
- `t1_vehicle_runN.log` — vehicle serial capture with `[STAGE_T]` state-
  transition and `[STAGE_T] rx` per-RX metadata lines (RSSI, SNR, CRC,
  state at arrival, seq). Host wall-clock timestamp prefixes every line.

### Summary

See `t1_summary.md` for headline metrics and the failure-classification
analysis the fix council uses.

### Reproducing

1. Build vehicle with `cmake -DROCKETCHIP_STAGE_T_LOGGING=ON ..`.
2. Flash vehicle via debug probe (`monitor reset halt; load; monitor resume`).
3. Station uses normal firmware (Stage T logging is vehicle-side only).
4. Run `python scripts/stage_t_run.py --run N --count 30 --interval 10.0`.
5. Use `python scripts/stage_t_summarize.py` to regenerate the summary.

## IVP-T2 — Cheat-mode sync ceiling (upcoming)
## IVP-T3 — CCSDS vs MAVLink protocol A/B (upcoming, soft gate)
## IVP-T4 — SX1276 CAD feasibility + ambient RSSI (upcoming)
