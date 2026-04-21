# IVP-T12 Manual Sweep Runbook

**Purpose:** measure first-try and eventual ACK rate at C0 / C1 / C2 with
both boards on bench-1591794 (T11) firmware. The `stage_t6_sweep.py`
harness can't complete the switch-verification step because the debug
menu suspends the station dashboard — this runbook does it by hand so we
can actually land a Batch A result.

**Preconditions (verify before starting):**

- Vehicle on COM7, `bench-1591794`, audit `IQ=0x27(OK) LNA=0x23(OK) CFG3=0x04(OK) CRC on`.
- Station on COM9, `bench-1591794`, Fruit Jam banner, dashboard streaming.
- Link green (RSSI -40 to -60 dBm, 0 CRC err, Pkts climbing).

## Station input-mode gotcha (critical)

Per `src/active_objects/ao_rcos.cpp:223-230`, when station is in kAnsi
(dashboard) mode, **only two keys are accepted**: `m` (cycle output
mode) and `x`/`X` (exit to kMenu). All other keystrokes are silently
dropped. The station boots into kAnsi by default.

This means `q<idx>z` (debug-menu → local-cfg → exit) does NOT work on
station from kAnsi — `q` is discarded. The station must first be
switched to kMenu via `x` before `q<idx>z` will do anything.

## Procedure (per config)

For each config {C0 idx=0, C1 idx=2, C2 idx=3}:

1. **Start each config with station in kAnsi (dashboard).** If not
   already there, send `m` (kMenu → kAnsi) until you see dashboard
   rows. Goal is a known starting state.
2. **Put station in kMenu so debug keys are accepted:** send `x` (or
   `X`) on COM9. Watch for `[main]` prompt. Station is now in kMenu.
3. **Switch vehicle:** `q<idx>z\r` on COM7. Expect response
   `[debug] [cfg] local radio -> BW<bw> <nav>Hz SF7 CR5 pwr20 (idx <idx>)`
   followed by `RADIO: config applied ...`.
4. **Switch station:** `q<idx>z\r` on COM9. Expect similar `[cfg]`
   response. Station stays in kMenu afterward.
5. **Return station to dashboard:** send `m` on COM9. Cycles kMenu →
   kAnsi. Dashboard rows resume within 1 s.
6. **Settle 5 s.** Both boards have the new config.
7. **Confirm station saw the switch:** dashboard `Radio:` row shows
   `BW<bw> <nav>Hz SF7 CR5` on BOTH the station-side (left) and
   `Vehicle:` (right) halves. If they don't match each other, STOP
   and report (real settings mismatch — one board didn't apply).
8. **Confirm link alive at new BW:** `Pkts:` climbing over 3 s AND
   RSSI green. If Pkts stops climbing at new BW, HALT (real
   T7-brittleness symptom).
9. **Send 30 commands, 10 s apart:** put station in kMenu again via
   `x`, use `a`=ARM (or other CCSDS command) on COM9, count ACKs from
   the `[CMD]` / `ACK` log lines. Record: total sent, first-try ACKs,
   eventual ACKs, denied, no-ack.
10. **Tabulate results. Return station to kAnsi via `m`.**

**Rule:** "ABORT" from a harness means the *procedure is broken*, not
that the test failed. Always root-cause the ABORT cause before claiming
gate pass or fail.

After all configs tested:
- Restore both boards to C0 (`q0z<CR>` on each).
- Write `logs/stage_t/t12_summary.csv`.
- Update `docs/RADIO_TESTING_RESULTS.md`.

## Config table

| Label | idx | BW (kHz) | Nav (Hz) | Purpose |
|-------|----:|---------:|---------:|---------|
| C0 | 0 | 125 | 5 | T1 baseline reproduction (collision-limited) |
| C1 | 2 | 250 | 10 | Halfway config |
| C2 | 3 | 500 | 10 | Primary candidate (T6 100% first-try) |

## Pass criterion (Batch A gate)

**C2 ≥ 95% first-try** over N=30. C0 should reproduce baseline
(0-6% first-try). C1 informational.

## If a switch fails

- **Vehicle shows config applied, station dashboard silent after
  settle:** send `\r` to station to confirm it's at main prompt. If
  yes, dashboard pause issue — re-read COM9 for a few seconds, it
  should start streaming again.
- **Station `Radio:` row shows wrong BW:** settings didn't apply.
  Check station `[cfg]` response from step 2 for errors.
- **Link dies at new BW (Pkts stops climbing):** report and halt.
  This IS a real T7-brittleness symptom.

## Why not the harness

`stage_t6_sweep.py` relies on reading `Pkts:` from the streaming
dashboard as the switch-verification signal. Because `q<idx>z`
suspends dashboard output briefly, the harness's Pkts readback
sometimes returns nothing and it ABORTs — even when the switch
actually worked. Manual confirmation avoids the ambiguity.
