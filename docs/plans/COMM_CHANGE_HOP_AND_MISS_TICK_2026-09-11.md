# COMM_CHANGE hop + RfManager miss-tick (2026-09-11)

**Status:** Closed on desk (2026-09-11). Landed on `main`.
**Worktree:** `C:\Users\pow-w\Documents\Rocket-Chip-main` (`main`)
**Library:** `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`); nested `starcom/` is what RC CMake links.
**CHANGELOG:** `2026-09-11-001`

This file is the sitting itinerary as executed. It is historical once
committed. Do not amend it to add later work; write a new plan.

## What this sitting was

Four steps, in order:

1. Stay on the **old PHY until the remote applies** (one SX1276: do not
   treat the initiator's own COMM_CHANGE as E69 after E68 moves S62→S60).
2. Prove desk **500 → 125** with matching catalogs/CFG and climbing
   `Pkts` / `Last` 0.
3. Commit that hop (`2f88744`).
4. Chase the **miss-tick**: pad `Lost` must hold and LQ must recover
   after a 10 Hz → 2 Hz hop.

Product boot stays **250 kHz / 10 Hz SF7 / 2 dBm**. USB MAVLink,
`dispatch_command`, and the SX1276 driver stay. Do **not** restore
`AO_RfManager_next_tx_window_us` / RadioScheduler. 211.1 stays 0.

## What this sitting was not

Not FPV scan. Not COP-P-lock LED vs RSSI. Not OTA radio settings.
Not STOP-GAP CMake strip. Those remain whiteboard leftovers.

## How to hop on the desk

NAV_PRESET `n` = next catalog COMM_CHANGE. Catalog idx2 is boot 250/10;
idx3 is 500/10; idx4 is 125/2 (the 500→125 cell). 125/10 is refused
(`radio_config_nav_fits_hz`).

Hop from **vehicle settings with the station on the pad**. Station
main `z` is pad readout. Do not drag the station through settings/CLI
during `n` (misses the SPDU). Do not send vehicle main `b` (find-me
beacon).

Do not claim a hop passed until **both** catalogs/CFG match **and**
`Pkts` climb.

## MAC (step 1–3) — `2f88744`

Initiator keeps radiating on the old PHY until the remote echo
(`local_comm_change` / `peer_comm_change`). E69 sets `y=2`; E41 auto-E65
queues COMM_CHANGE into S56; FIFO-empty E66→S58→E67 S62. PHY apply
reinits MAC and reloads hail from the **live** catalog (not always boot
250). Nested `starcom/` MAC was copied from `starcom_dev`.

Desk: 250→500 both CFG BW500; 500→125 both CFG BW125 2 Hz, pad
`Pkts` 624→626, `Last` 0.0 s, RSSI −54 dBm.

## Miss-tick (step 4)

`AO_RfManager` 10 Hz tick was charging misses against the boot 10 Hz
period after COMM_CHANGE moved nav to 2 Hz, so `Lost` climbed and LQ
went TRACK_DEGRADED.

Wire:

- `AO_RfManager_set_nav_period_ms` from `ao_radio_apply_runtime_config`
  (`1000U / nav_rate_hz`). Clears LQ window, `next_miss_due_ms`,
  `consec_missed_rx`. Logs `[RF] nav period %u ms`.
- `rf_charge_miss_slot` / `rf_miss_grace_ms` in `src/safety/rf_link_health.h`:
  one miss per nav slot; grace **2 × period** (T14 §2 skip-every-other
  is 2×). 1.5× false-fired on 2 Hz half-duplex (one send contact is
  about one period off-air).
- Host `RfMissSlot.*` 3/3.

Desk after 2× grace (this wrap's threshold): boot Lost 0 / LQ 100% at
250/10; 250→500 TRACK 100%; **500→125 both CFG BW125 2 Hz**, **Lost 0
held** (`Pkts` 474→497), **TRACK LQ 100% [OK]**, `Last` 0.0 s,
`[RF] nav period 500 ms` on vehicle and station.

Pad `RX n/desired Hz` is a **boot-lifetime** average
(`g_radioRateCounters.rx_crc_ok_n / uptime_s`; denom is CFG
`nav_rate_hz`). After a 10 Hz soak then hop to 2 Hz it stays ~7–8 until
uptime dilutes. Instantaneous air is `Pkts` / `Last` / LQ. Not a hop
fail and not on this sitting's list.

## Leftovers (not this file)

See `AGENT_WHITEBOARD.md` Also-open: FPV scan, COP-P lock vs RSSI bar,
oMCT live board, passive chute-detect. Plus SET OTA / ARM leftover.
