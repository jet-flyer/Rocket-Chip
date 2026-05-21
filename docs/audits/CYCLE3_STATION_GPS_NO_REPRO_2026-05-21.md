# Cycle 3 — Station GPS cold-boot diagnostic (5-cycle experiment) — NO REPRO

**Date:** 2026-05-21
**Origin:** Cycle 3 of the four-cycle plan (`C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md`), stashed in `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`. Triggered by user-reported failure: "Fruit Jam on power-on reports `hardware: 10/11 ok [fail] gps`; warm reboot immediately after = 11/11 ok."
**Disposition:** NO-REPRO across 5 cold-boot cycles. Stash remains open pending user re-encounter; this doc captures the 2026-05-21 negative result so the next session has a baseline to compare against.

## Procedure

Per the stash plan's Step 1 single-experiment design. Each cycle:

1. Unplug Fruit Jam USB cable.
2. Wait ~5-10 seconds.
3. Plug USB back in.
4. Wait 2s for USB CDC enumeration.
5. Programmatically re-find the COM port via banner classification.
6. Send `x` to enter kMenu.
7. Send `p` to capture preflight output (`GPS: GO/FAIL`, `VERDICT`).
8. Send `q` then `b` to capture Hardware Status block, which renders `gps_pa1010d_get_debug_status()` showing `PMTK writes: [N,N,N]  window_hit:N  init:N`.
9. Send `z` to back out, `x` to return to dashboard.

Five iterations of this procedure, no other changes between cycles. Bench conditions: indoor, ambient ~22 C, no other USB devices on the same hub (vehicle Feather on COM7, station Fruit Jam on COM9, both via different USB ports). Station firmware: `flight-704f102` (from 2026-05-16 — pre-this-session; the GPS init code path is functionally identical to current HEAD per `git log` comparison).

## Data

| Cycle | Uptime (s) at capture | PMTK writes (3 values) | window_hit | init | Preflight GPS | Bug repro? |
|---|---|---|---|---|---|---|
| Baseline (warm) | 331 | [-1, -1, 51] | 1 | 1 | GO | no |
| 1 (cold) | 25 | [-1, 18, 51] | 1 | 1 | GO | no |
| 2 (cold) | 19 | [-1, -1, 51] | 1 | 1 | GO | no |
| 3 (cold) | 29 | [-1, 18, 51] | 1 | 1 | GO | no |
| 4 (cold) | 22 | [-1, -1, 51] | 1 | 1 | GO | no |
| 5 (cold) | 20 | [-1, 18, 51] | 1 | 1 | GO | no |

Hardware Status block (every cycle, except for the PMTK values varying as noted):

```
[PASS] I2C bus initialized at 400kHz (SDA=20, SCL=21)
[PASS] GPS init (I2C at 0x10, 500us settling delay)
[DBG ] GPS early-init: PMTK writes: [<varies>,<varies>,51]  window_hit:1  init:1
[PASS] Radio: RFM95W LoRa 915 MHz SF7 20dBm (CS=10 RST=6 IRQ=5)
  Best GPS: no fix acquired yet
```

Dashboard `GPS:` line during cycles 3-5 reported `3D (4-5 sat)` — that's the *vehicle's* GPS fix arriving via radio telemetry, not the station's onboard GPS. The station's own onboard GPS reads "no fix acquired yet" because cycle uptime is ~20-30s and PA1010D cold-start to fix is 35-60s per datasheet (~2 minutes outdoors, longer indoors).

## Pattern observations

- **PMTK write 1 always failed** (`-1`) on every cold boot (5/5) AND on the warm baseline. The MT3333 chip is not ACKing its first I2C write immediately after `gps_pa1010d_init()` starts. This is consistent with a wake-up window of >0ms but <50ms.
- **PMTK write 2 succeeded 2/5** times on cold boots (when it did succeed, returned 18 bytes = `kPmtk220_1HzSentence` length). It failed (`-1`) on 3/5 cold boots and the warm baseline. The 50ms inter-write sleep is on the boundary of MT3333 wake.
- **PMTK write 3 succeeded 5/5** on cold boots (returned 51 bytes = `kPmtk314Sentence` length). By the third write the chip is reliably ACKing.
- The **8-retry × 150ms post-config probe loop** (the "aggressive probe retry" at `gps_pa1010d.cpp:242-255`) found NMEA every time. `window_hit:1` and `init:1` on every cycle.

## Hypothesis attribution per stash-plan Step 2

The stash plan defined 5 hypotheses (H1-H5) with attribution rules:

- **H1 (MT3333 cold-start latency > 1.2s):** Would predict PMTK never succeeds on cold boot. Refuted: write 3 succeeded 5/5.
- **H2 (Fruit Jam-specific I2C electrical):** Would predict PMTK never succeeds at t+3s, bus stuck LOW. Refuted: writes succeed, bus is functional.
- **H3 (POWMAN reset-cause influence):** Would need correlation between reset-cause and failure. No failures observed, so no correlation possible to evaluate.
- **H4 (GPS LDO not stabilized in 1.2s):** Would predict GPS-LED-off period > 1.2s. Not directly measured (no LED logging). Refuted indirectly because writes succeed in <600ms.
- **H5 (Stale I2C bus state):** Would predict inconsistent bus state across t=0 readings. Refuted: same outcome across 5 cold boots + warm baseline.

**No hypothesis wins** because the underlying failure did not occur in 5 attempts. The data shows the current init pattern works reliably under tested bench conditions.

## Why the user-observed failure could still be real

5/5 success does not rule out the user's earlier observation. The failure was reported as a real datapoint, and possible explanations for the non-repro this session:

1. **Intermittent / timing-dependent at lower than 20% rate.** A bug that fires 10% of the time would pass 5 in a row with ~59% probability; even 20% gives ~33%. The user observed it once; sample size of 1 vs. 5 doesn't statistically prove the bug is gone.
2. **Different bench condition.** Things that could matter that weren't varied: power-off interval (we did ~5-10s; the user's observation might have been after several minutes off → MT3333 fully cold), USB hub topology (we used distinct ports for vehicle + station), ambient temperature, USB cable / connector freshness.
3. **Different USB-power character.** A USB hub busy with other devices, or a cable with marginal voltage drop, could cause VBUS sag during boot that delays the MT3333's POR.

Item 2 is the most-likely candidate to vary in a future re-attempt. If the user encounters the bug again, a useful first datapoint is "how long was the device powered off before the failing cold boot."

## Recommendation

- **No code change this session.** Per `LESSONS_LEARNED.md` Entry 38, do not ship a speculative fix to close off a future option based on hypothesis without data. The proposed H1 fix (extend window to 2.5s, gated on `HAD_POR`) would commit code for a failure mode we could not reproduce on this bench in 5 attempts.
- **Stash row remains open** in `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md` (Cycle 3) — but the trigger to re-open is the user encountering the bug again, not us re-attempting on the same bench.
- **WB row updated** to reflect 5/5 no-repro + request bench-condition context on next re-encounter.

## Future-session protocol for re-encounter

When the user re-encounters `hardware: 10/11 ok [fail] gps` on Fruit Jam:

1. **Do not power-cycle**. The warm reboot would mask the cold-boot state.
2. **Immediately enter kMenu** (`x`), send `q b` to capture Hardware Status WITH the failing state in `gps_pa1010d_get_debug_status()`. That captures `g_pmtkWriteResults[]` + `g_pmtkWindowHit` + `g_initialized` from the failing init.
3. **Record bench context**: how long was the device powered off; what other USB devices were on the bus; ambient temp.
4. **Then** power-cycle and re-test to see if warm-reboot recovers (per the user's original observation pattern).
5. If a failing snapshot is captured, this artifact's data table is the comparison baseline.

## Related

- Stash plan: [`docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`](../plans/CYCLE_RESIDUALS_AFTER_R5.md).
- Init code: `src/drivers/gps_pa1010d.cpp` (`gps_pa1010d_init()` at line 204-263, `gps_pa1010d_get_debug_status()` at line 265-294).
- Boot sequence: `src/main.cpp` `init_gps_early()` at line 218-226, called from `init_early_hw()` line 237.
- LL Entry 31 — flash_safe_execute corrupts I2C peripheral on RP2350B. Same family of issues, *different* failure pattern.
- LL Entry 41 — RP2350B GPIO pads start isolated; pad de-isolation now runs in `i2c_bus_init()` before recovery.
- LL Entry 38 — "code shows current-state, primary sources show possibility-space" — the meta-rule that says don't ship hypothesis-driven fixes without data.
- Flipper Zero I2C handling research (2026-05-21 agent transcript) — confirmed Flipper has no applicable patterns; their HAL is more aggressive (NACK-as-fatal) than ours.
