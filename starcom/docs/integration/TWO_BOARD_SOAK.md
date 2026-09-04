# Two-board RF soak (reusable)

**Audience:** Rocket-Chip first; any Starcom consumer with two SX1276/RFM95 ends.  
**Status:** living procedure. Not an IVP increment. Not a Blue Book PICS.  
**Library:** `starcom-v0.2.25`. Consumer flag `ROCKETCHIP_USE_STARCOM=ON`.  
**Worktree (this lab):** `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`).

Host codec proof stays in [`TESTING.md`](../TESTING.md). This file is **over-the-air**: airtime, half-duplex leftover, COP-P PLCW cadence, command ACK. Range ranking is a *layout class*, not a PHY claim from a quiet desk.

---

## What this soak is for

Reuse it whenever the **working air** changes: PLTU size, nav QoS (seq vs expedited), nav rate, SF/BW, TX power, or “is there leftover for a command.”

| Proves | Does not prove |
|--------|----------------|
| TxDone span vs `rfm95w_airtime_us` | Field range / HAB margin |
| 5 Hz / 10 Hz fit with PLCW-as-is vs expedited nav | FSK bitstream vs LoRa wrap |
| Command leftover (ARM / SET) in the hole | Hail SF/BW (owner pick, not this matrix) |
| Full `TelemetryState` (45 B user) still one FIFO PLTU | 50–400 Hz onboard log on air |
| Desk CRC under a **harsh** layout | Adaptive TX power (RC WB; needs RSSI return path) |

Owner ops locked for rocket TM: **live nav, onboard log is truth, ACK GCS commands.** Pass A measures the cost of today’s seq-nav + PLCW-every-nav. Pass B is expedited nav (commands stay seq). If leftover remains, a little nav retry is optional — not required.

Do **not** mute PLCWs while nav is still sequence-controlled. Window is 4; nav stalls after four unacked frames.

---

## Hardware (this lab)

| Role | Board | Radio | Serial (USB) |
|------|-------|-------|----------------|
| Vehicle | Feather RP2350 HSTX + RFM95 FeatherWing | SPI0, CS10 RST11 IRQ6 | `02FBDDB8E1CA1281` COM5 |
| Station | Fruit Jam + RFM95 | as wired | `BEC71B8EDC6AEBD1` COM7 |
| Probe | CMSIS-DAP | — | COM4 PID 000C |

Flash: vehicle `scripts/flash_elf_halt_write.py` (never OpenOCD `program` / extra `reset halt`). Station `picotool load <uf2> -f --bus/--address` after mapping chip serial (Jam `BEC71` COM7 only — never station UF2 onto Feather `02FB`). OpenOCD long-lived background (`scripts/start_openocd_pico_sdk.ps1` must detach; inherited stdin EOF kills telnet). No BOOTSEL unless stuck. GPS wing seated on the UART row (offset row scrambles LoRa CS/SPI — `RegVersion=0xFF`).

**Cell PHY is the boot default, not SET.** Desk 2026-09-03: station SET CLI log can fire while COP-P `V(S)` stays 0 — the command AD never airs, vehicle CFG never moves. Dashboard eats `r` (`x` then `r`). For Pass A, set `kDefaultRocketRadioConfig` to the cell tuple, rebuild both roles, flash both, score only if vehicle `t` shows that CFG. Do not desk-SET +20.

Air identity: 915.0 MHz, sync `0x12`, CR 4/5, preamble 8. ON nav PLTU = 18 B wrap + **45 B** user (`TelemetryState` packed, including `met_ms` and `flags`). On-wire **63 B**. PLCW PLTU ≈ 14 B.

**TX power (desk default = legal min 2 dBm both ends).** ToA, leftover, and PLCW cadence **do not depend on PA**. +20 dBm at 1–2 m slams RSSI (~−40 to −50 dBm historically) and the SX1276 packet-SNR register (~+10 dB cap), so every SF/BW looks identical. 2 dBm is enough to close 1–2 m and leaves ~18 dB of RSSI headroom so cells can actually differ in the log. +20 dBm is for Class D (outdoors) and an optional PA smoke, not the paper-check matrix.

---

## Physical layout classes

915 MHz λ ≈ 33 cm. 1.5 m is several wavelengths (not reactive near-field), but a **metal PC chassis under the station** is a reflector and the USB bundle is clutter. Historical desk (3–4 ft, metal in path): RSSI about **−40 to −50 dBm**, SNR ~9 dB, noise ~−114 dBm — see `docs/RADIO_TESTING_RESULTS.md`. That is **~70 dB above** SF7/BW500 sensitivity (~−117 dBm). At +20 dBm every legal cell looks CRC-clean. Use layout class to say what the run is allowed to claim.

| Class | Setup | Typical RSSI (expect) | Use |
|-------|--------|------------------------|-----|
| **A — protocol desk** | Station on the metal PC (as parked). Vehicle ~1.5 m, antennas roughly vertical. **2 dBm both ends.** | maybe −55 to −75 dBm; SNR may still sit on the ~+10 dB cap | Paper ToA, PLCW cadence, 10 Hz geometry, command leftover. Log RSSI/SNR deltas between BW cells. **Not** a range rank. |
| **B — harsh desk** | Same 2 dBm. Vehicle on the **floor behind the chassis** (no LOS). Antennas **orthogonal**. | want −80 to −100 if you can get it | CRC / first-try vs BW on the same PA. Still may not reach the floor. |
| **C — clean short LOS** | Station **lifted off metal**. 2 m LOS, antennas vertical. **2 dBm.** | stronger than A | Repeat a Pass A cell if Class A CRC is dirty and you need a protocol-only number. |
| **D — range (later)** | Outdoors, 100 m then 500 m / 1 km. Start +20 dBm, then step down. | toward sensitivity | Only class that ranks BW125 vs BW500 on margin. |

**Tonight:** Class **A at 2 dBm** for the full matrix (validate paper airtime). Then harsh-B (still 2 dBm, worse geometry) if time. Optional one-shot +20 dBm on A5 only if you want a PA smoke — not required for the paper check. Do not treat Class A SNR as “BW500 is as good as BW125.”

Log for every run: class letter, estimated range, antenna orientation, TX dBm, what the station is sitting on.

---

## Paper budget (SF7, CR 4/5, 60 B nav / 14 B PLCW)

Formula: `rfm95w_airtime_us`. Combined = vehicle nav + station PLCW every nav. Leftover is for a cmd PLTU (~42 B).

| Cell | Nav ToA | PLCW ToA | 5 Hz veh+PLCW | 10 Hz veh+PLCW | Cmd leftover @ 10 Hz | Paper |
|------|--------:|---------:|--------------:|---------------:|---------------------:|-------|
| SF7/BW125 / 5 Hz (rocket default) | 113 ms | 46 ms | 80% | — | 41 ms @ 5 Hz | **expect PASS** protocol |
| SF7/BW125 / 10 Hz | 113 ms | 46 ms | — | **159%** | none | **expect FAIL** (past expected) |
| SF7/BW250 / 5 Hz | 56 ms | 23 ms | 40% | — | lots | PASS |
| SF7/BW250 / 10 Hz | 56 ms | 23 ms | — | 80% | **20 ms** (cmd ~41 ms **does not fit** if PLCW every nav) | PASS nav; **FAIL or squeeze** command |
| SF7/BW500 / 10 Hz | 28 ms | 12 ms | — | 40% | **60 ms** | PASS nav + command |
| SF8/BW125 / 5 Hz | ~205 ms nav | — | >100% | — | none | **expect FAIL** (past expected) |

Pass B (expedited nav, no per-nav PLCW): drop the PLCW column. 10 Hz leftover ≈ period − nav ToA (BW250 ≈ 44 ms, BW500 ≈ 72 ms). That is the improvement to measure.

Full 45 B `TelemetryState` user → 63 B PLTU: ~3 ms more at SF7/BW250 vs the old 42 B prefix. Same pass/fail as 60 B.

---

## Matrix (run in order)

Fixed unless a row says otherwise: ON firmware, 915.0 / sync `0x12` / CR 4/5 / 8-sym preamble, **2 dBm**. Nav SDU is the full 45 B `TelemetryState` (already packed; still one FIFO frame).

### Pass A — PLCW as-is (seq nav)

Nav `pump_submit_sdu(..., false)`. Station may PLCW every accepted nav.

| ID | Layout | SF | BW | Nav Hz | Power | Duration | Expect |
|----|--------|---:|---:|-------:|------:|----------|--------|
| A1 | A | 7 | 125 | 5 | **2** | 2 min | lock, nav ~5 Hz, PLCW seen, ARM leftover OK |
| A2 | A | 7 | 125 | 10 | **2** | 1 min | **geometry fail** (over 100% TX). Stop when obvious. |
| A3 | A | 7 | 250 | 5 | **2** | 2 min | lock, leftover fat |
| A4 | A | 7 | 250 | 10 | **2** | 2 min | nav OK; ARM during soak — note if cmd collides |
| A5 | A | 7 | 500 | 10 | **2** | 2 min | nav + ARM both OK |
| A6 | A | 8 | 125 | 5 | **2** | 1 min | **past expected**; ToA too long. Stop when obvious. |
| A7 | B | 7 | 250 | 10 | **2** | 2 min | CRC/RSSI vs A4 (geometry only; same PA) |
| A8 | B | 7 | 500 | 10 | **2** | 2 min | CRC/RSSI vs A5 |

### Pass B — reduce PLCW (expedited nav)

Nav `pump_submit_sdu(..., true)`. Commands stay seq. One PLCW at lock is OK; not one per nav.

Repeat **A3, A4, A5** only (B3/B4/B5), same layout A, 2 min each. Compare station TX count, leftover, ARM first-try.

If leftover is fat after B5: optional B5r — leave seq nav but PLCW every 4th (window). Only if A/B already closed.

### Command probe (each PASS cell that is expected to fit)

Dashboard ARM once the station has an RfManager window (heard nav, not ACQ). DISARM when done. Do not hail. Do not SET a hail PHY.

---

## Metrics (log every cell)

Capture vehicle COM5 + station COM7 (DTR-low / `open_classified_port`). Name files `logs/soak/<date>_A4_bw250_10hz_clsA.txt` (and `_stn`).

| Metric | Where | Gate |
|--------|--------|------|
| `RegVersion` | CLI `d` / Hardware | `0x12` both boards or abort cell |
| TxDone span | radio / `[STAGE_T]`-style tx_done if present | within ~20% of `rfm95w_airtime_us` for that PL |
| Nav rate | station `rx` count / window | within 20% of commanded Hz on PASS cells |
| `cop-p: lock` / `plcw_heard` | vehicle | yes on A1/A3–A5; Pass B still locks |
| Station TX count | station `tx` | Pass A ≈ nav rate (PLCWs); Pass B ≪ nav rate |
| CRC errors | radio | Class A ~0; Class B log only |
| last RSSI / SNR | station dashboard (local chip, **not** in SDU) | log every cell; 2 dBm should sit well below historical −40 dBm @ +20. SNR may still peg ~+10 dB on LOS |
| ARM | station dashboard `a` then `ARM\r` | PASS cells with leftover: vehicle `[fd]` sees it |
| TX-busy / window 0 drops | station | 0 on PASS; expected on A2/A6 |

Positive-control: if both `tx=0` after 10 s, stop (radio/seating), do not score the cell.

---

## Tonight (this instance)

1. Confirm GPS wing on the UART row; vehicle `RegVersion=0x12`. Station on the metal PC is **Class A**. Vehicle ~1.5 m, antennas up. **Both radios 2 dBm** (legal min) for the whole matrix.
2. Confirm ON image has the 45 B nav SDU (`kNavSduUserBytes == 45`); rebuild vehicle + station if the flashed ELF is older.
3. Run A1 → A5 by **reflashing** each cell’s boot tuple (A1 already scored). Skip A6. Do not treat a BW125 log as A3. Gate: TxDone vs paper ToA (~20%), not RSSI. Confirm `CFG: BW=… nav=…Hz pwr=2dBm` before the capture.
4. If energy left: A7/A8 (same 2 dBm, vehicle behind chassis). Then Pass B on A3–A5.
5. Hail, FSK, adaptive power, outdoor range: **not tonight.**

Desk at 2 dBm is **good enough to check the paper numbers** (ToA, 10 Hz fit, PLCW cost). It is **not** a substitute for an outdoor field test and must not retire BW125 for range. Class B is a stress toy, not a km stand-in.

---

## Later reuse

Copy the matrix, change only: payload size, QoS, layout class, power. New working-air change = new dated log dir, same IDs. Outdoor range = Class D rows added under the same IDs (A5-D, …). Do not mix Class A SNR into a range claim.

Related: RC [`AGENT_WHITEBOARD.md`](../../../AGENT_WHITEBOARD.md) adaptive TX power; Starcom [`AGENT_WHITEBOARD.md`](../../AGENT_WHITEBOARD.md) LoRa Hail (not this matrix).
