# Two-board LoRa soak — Pass A / Pass B (2026-09-03)

**Tree:** `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`), keepers `6dcea56`.  
**Procedure:** `starcom/docs/integration/TWO_BOARD_SOAK.md` (matrix only).  
**Capture:** `scripts/soak_two_board.py` (DTR-low CDC). Raw logs `logs/soak/` (gitignored).  
**Power:** 2 dBm both ends the whole matrix. **Cell PHY = reflash**, not SET.

Vehicle Feather `02FB` COM5. Station Fruit Jam `BEC71` COM7. Probe COM4.

---

## Owner notes this sitting

- **Received telem Hz vs commanded.** Even cells at ~40–50% paper airtime delivered ~4–6 Hz when commanded 10 Hz. Owner: that is acceptable for GCS **if** the stream can be synced to true time for a 3D display / Open MCT. MET on the nav SDU plus GPS on both vehicle and station is the intended sync; oMCT interpolation is assumed, not wired (`docs/gcs/openmct/` still empty).
- **U.FL** was unseated for A1–A5 first pass (connector-only). Later cells used the TBS 900 MHz RX antenna. ~10 dB RSSI step (−67 → −55). Layout B with the antenna on was **stronger** than Class A without it — not a range test.
- **Default boot** stays **SF7 / BW125 / 5 Hz / CR 4/5 / 2 dBm**. Do not retire BW125 from this desk. 10 Hz as a product rate needs **expedited nav** (Pass B), not seq + PLCW-every-nav.

---

## What Pass A / B are

| Pass | Nav QoS | What it measures |
|------|---------|------------------|
| **A** | Seq `pump_submit_sdu(..., false)`; station may PLCW every nav | Paper ToA, 5/10 Hz fit, command leftover |
| **B** | Expedited nav `true`; commands stay seq | Station TX/PLCW drop, leftover, ARM first-try. Cells **B3/B4/B5** = A3/A4/A5 PHY |

Not this sitting: hail, FSK, adaptive power, **Class D outdoor** (100 m then 500 m / 1 km, start +20 dBm). B5r (PLCW every 4th) not run — B5 still LQ-dropped after ARM.

---

## Results (authoritative cells)

First A3–A5 were **no U.FL**. Prefer `*_ufl` / `*_ufl2` / `*_exp` for antenna-on claims. Keep the no-U.FL logs.

| Cell | PHY | Layout | Paper | Result |
|------|-----|--------|-------|--------|
| A1 | 125/5 SF7 | A, no U.FL | 80% duty, ARM leftover OK | Lock + 2 dBm PASS. ARM `[fd]` FAIL |
| A2 | 125/10 SF7 | A, no U.FL | 159% expect FAIL | FAIL as expected |
| A3 | 250/5 SF7 | A, no U.FL | leftover fat | Vehicle lock; station waiting PLCW (transient) |
| **A3 ufl** | 250/5 | A, U.FL | same | **Station lock**, ~4.1 Hz, **`[fd]` ARM**. End LQ 10% after ARM |
| A4 | 250/10 SF7 | A, no U.FL | 82%, cmd squeeze | Lock both, ~5.6 Hz RX, no `[fd]` ARM |
| A4 ufl | 250/10 | A, U.FL | same | **Invalid leftover** (~0.8 Hz, LQ 20%) — do not use |
| **A4 ufl2** | 250/10 | A, U.FL | same | ~3.3 Hz, lock, **`[fd]` ARM**, LQ 10% after ARM (squeeze) |
| A5 | 500/10 SF7 | A, no U.FL | 41%, leftover | Lock, LQ 100%, ~5 Hz, no `[fd]` (`V(S)=0`) |
| **A5 ufl** | 500/10 | A, U.FL | same | RSSI −55, ~5.1 Hz, LQ 100%, **`[fd]` ARM** |
| A6 | 125/5 **SF8** | A, U.FL | >100% expect FAIL | Did **not** die: lock, LQ 100%, ~4.2 Hz. SF8 not in SET table |
| A7 | 250/10 | B, U.FL | vs A4 geometry | RSSI −57 (not harsh). Squeeze after ARM |
| A8 | 500/10 | B, U.FL | vs A5 geometry | RSSI −58. **`[fd]` ARM**, LQ 90–100% |
| **B3** | 250/5 exp | A, U.FL | drop PLCW | ~5 Hz until ARM, **`[fd]`**, end LQ 10% |
| **B4** | 250/10 exp | A, U.FL | drop PLCW | ~4.6 Hz, **LQ 100% through ARM**, **`[fd]`**. Contrast vs A4 |
| **B5** | 500/10 exp | A, U.FL | drop PLCW | ~5 Hz, **`[fd]`**, end LQ 10% |

SET hop never moved PHY on this desk (`V(S)` often 0). Every cell was a boot-tuple reflash.

---

## Defaults (no change from keepers)

| Use | Setting |
|-----|---------|
| Flight / range default | **125 kHz, 5 Hz, SF7, 2 dBm** until Class D |
| 10 Hz TM if wanted | **250/10 + expedited nav** (B4), not seq A4 |
| Comfortable seq-nav | **250/5** |
| Desk PA | **2 dBm**. +20 is Class D / PA smoke |

Do not default 500 kHz. Do not put SF8 in `kRadioConfigTable` from A6.

---

## Next (fresh sitting)

See `AGENT_WHITEBOARD.md`. In short: commanded vs delivered Hz (GCS/MET/GPS/oMCT), command AD / SET hop, Class D outdoor BW rank, FSK, hail, adaptive power.

---

## Hardware left on the desk

Boards still hold the **B5 500/10 expedited** image. Tree is restored: `kDefaultRocketRadioConfig` 125/5/2, nav submit `false`. Do not treat the chips as the tree default.
