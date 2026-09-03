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
| Vehicle | Feather | COM5 `02FBDDB8E1CA1281` (was **unplugged** for station flash) | `flight-4819102` | CFG **BW=125 SF=7 CR=5 nav=5Hz pwr=2dBm**, RegVersion `0x12`, Hardware 13/13 |
| Station | Fruit Jam | COM7 `BEC71B8EDC6AEBD1` | **`flight-2f7096d`** | SET table **pwr=20**. Last good banner after operator reset. COP-P waits PLCW until vehicle USB is back |
| Probe | CMSIS-DAP | COM4 `E663AC91D3487137` PID 000C | — | On Feather. Do not OpenOCD during soak |

**The Fruit Jam still needs the 2 dBm station image.** Desk SET at +20 is not allowed. A3–A5 are blocked until the banner is `flight-4819102` and SET prints `pwr=2`.

UF2 waiting (built, **not on the chip**):

`C:\Users\pow-w\Documents\starcom_dev\build_station_flight\rocketchip.uf2`  
(`flight-4819102`, Fruit Jam, table 2 dBm; disk mtime 2026-09-02 21:09)

Uncommitted overlay that UF2 was built from (do **not** `git clean`):

- `include/rocketchip/radio_config_table.h` — all six SET rows `power_dbm = 2` (HEAD is 20)
- `src/flight_director/mission_profile_data.h` — `kDefaultRocketRadioConfig.power_dbm = 2` (HEAD is 20)
- `src/cli/rc_os_commands.cpp` — CLI `t` prints `CFG: BW=… pwr=…dBm`

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

## Why SET did not change PHY

1. Live Jam `flight-2f7096d` CLI `r` sends table **pwr=20**.
2. Vehicle dispatcher `dispatch_set_radio_config` (`ao_telemetry.cpp`): ground + SX1276-legal + **±6 dB vs current**. 20 vs 2 → **kDenied** if the cmd arrives.
3. Station hops its own radio **only on ACK-accepted** (`station_on_set_radio_ack`).
4. SF7/BW125/5 Hz seq-nav+PLCW is already TX-busy, so many cmds get **no ACK at all** (leftover), which is not the same as denied.
5. Hear ≠ lock ≠ command accept. RSSI bar = last LoRa FIFO RSSI. COP-P lock = valid peer PLCW. ARM leftover is the soak command probe; SET ACK is how we know PHY moved.

`g_cycleIdx` advances on every station `r` even without ACK.

---

## How to complete Pass A (next sitting)

1. **Flash the Jam** with the 2 dBm UF2. Owner rule: **unplug the board you are not flashing** (Feather out, Jam only), flash, then plug both back in for soak. Dual-plug `picotool -f --ser` on this Jam was not working this sitting; dual-board targeting is parked for later.
2. Prove flash: COM7 banner **`flight-4819102`**, SET log **pwr=2**. `rc=0` with no load/verify text and no COM drop is **not** a write.
3. Plug Feather back. Wait COP-P **lock**, RX climbing, CRC 0, vehicle CFG still 125/5/2.
4. **A3 hop (do not soak until CFG says BW=250):**
   - On busy 5 Hz air, station `r` **four** times (idx 1–4). Expect no ACK; both stay 125/5.
   - Vehicle `r` until **2 Hz** leftover.
   - One more station `r` → idx 5 = **BW250 / 5 Hz / 2 dBm**.
   - Score A3 only if vehicle `CFG: BW=250` and station Radio row matches. Then 2 min + ARM.
5. BW250 has leftover: SET on to **A4** idx 2 (250/10) and **A5** idx 3 (500/10). Skip A6. A7/A8 layout B.
6. Capture: `python scripts/soak_two_board.py --cell A3 --tag bw250_5hz_clsA --duration 120`

Do not desk-SET to +20. Do not kill Python during COM7 `CreateFile`. Do not `pnputil /restart-device`. Do not OpenOCD during soak. Do not extra `picotool reboot -f` / `info -f` as counted boots. USB unplug is FLASHING recovery, not a soak step.

CDC open: DTR low at open, then SETDTR, `dsrdtr=False` (`soak_two_board.py` `open_cdc`).

---

## Wanted (not this soak)

- Station LED: keep RSSI colour, **0.5 Hz blink when not COP-P locked**.
- FSK after LoRa Pass A, and only after the 2 dBm station image is actually on the Jam.
