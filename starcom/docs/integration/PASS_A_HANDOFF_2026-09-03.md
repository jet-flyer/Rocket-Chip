# Pass A soak — handoff 2026-09-03

**Tree:** `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`).  
**Procedure:** `starcom/docs/integration/TWO_BOARD_SOAK.md`  
**Capture:** `scripts/soak_two_board.py` (DTR-low open, then SETDTR; never peek_banner).  
**Results so far:** `logs/soak/2026-09-02_PASS_A_RESULTS.txt`

Owner going to bed. Dual-board picotool targeting was already solved in an earlier sitting — **do not reinvent; pick it up later.**

---

## Exact hardware state at handoff

| Role | Board | USB | Live image | Radio |
|------|-------|-----|------------|--------|
| Vehicle | Feather | COM5 `02FBDDB8E1CA1281` | `flight-a88ed79` wrap-free, 2 dBm | last scored CFG **BW=125/5/2**; A3 needs a **reflash** to 250/5/2 |
| Station | Fruit Jam | COM7 `BEC71B8EDC6AEBD1` | `flight-a88ed79` wrap-free, 2 dBm table | USB `-f --bus/--address` after mapping serial. Never station UF2 onto `02FB` |
| Probe | CMSIS-DAP | COM4 `E663AC91D3487137` PID 000C | — | On Feather. Do not OpenOCD during soak |

A3–A5 blocked on **real PHY**, not on 2 dBm image. SET does not hop cells on this desk (`V(S)` stays 0). Reflash `kDefaultRocketRadioConfig` per cell.

---

## What already scored (do not redo as if new)

**A1** SF7/BW125/5 Hz / 2 dBm / 120 s — **RF hear + lock + 2 dBm PASS. ARM leftover FAIL.**  
Paper ~82% duty (nav 118 ms + PLCW 46 ms / 200 ms). Vehicle 63 B nav TX-busy; station 12 B PLCW TX-busy. CRC 0. RX ~4.6 Hz.

**A2** SF7/BW125/10 Hz — **FAIL as expected** (paper 159%). Vehicle CLI `r` only (telem rate). CFG radio field can still say nav=5. Station SET was not used.

**A3–A5 2026-09-03 captures are invalid.** Vehicle never left BW125. Do not treat those logs as BW250/500 cells.

**A6** skip — SF8 is not in `kRadioConfigTable`.  
**A7/A8** need layout B (vehicle behind chassis) after A4/A5 PHY is real.

Pass B (expedited nav) is after Pass A.

---

## SET vs lock (desk 2026-09-03) — do not hop cells with SET

Vehicle does **not** “just fail to re-lock after changing over.”

| Attempt | Station | Vehicle | Lock |
|---------|---------|---------|------|
| First SET idx 1 (old image) | stayed 125/5 | **did** apply 125/5 → 125/10 | died (station hops only on ACK) |
| Hop-on-send firmware | **did** apply 125/10 + COP-P reinit | **stayed** 125/5 | died (split brain). Reverted. |
| After revert, `x` then `r` | SET log idx 1, **V(S) stays 0** | no `[CMD] SET accepted`, CFG 125/5/2 | **survives** (LQ ~80–90% after TX hold) |

Root: command AD never airs (`V(S)=0`). Dashboard eats `r` until `x` (kMenu). Radio was dropping 12/14 B ACK/PLCW while 63 B nav occupied TX.

**Pass A cell PHY = flash `kDefaultRocketRadioConfig`, not SET.** Score A3 only if vehicle `CFG: BW=250 … nav=5Hz pwr=2dBm`.

---

## How to complete Pass A

1. Keep 2 dBm table + wrap-free USB `-f`. Jam: `picotool load <uf2> -f --bus/--address` after mapping `BEC71`. Never station UF2 onto Feather `02FB`.
2. **A3:** set default to BW250 / 5 Hz / 2 dBm, rebuild vehicle + station, probe-flash Feather, USB `-f` Jam. Confirm CFG then:
   `python scripts/soak_two_board.py --cell A3 --tag bw250_5hz_clsA --duration 120 --bw 250 --nav-hz 5`
3. **A4/A5:** same reflash with 250/10/2 and 500/10/2. Skip A6. A7/A8 layout B.
4. Do not OpenOCD during soak. Do not desk-SET +20. Do not score BW125 as A3.

CDC open: DTR low at open, then SETDTR, `dsrdtr=False` (`soak_two_board.py` `open_cdc`).

---

## Wanted (not this soak)

- Station LED: keep RSSI colour, **0.5 Hz blink when not COP-P locked**.
- FSK after LoRa Pass A, and only after the 2 dBm station image is actually on the Jam.
