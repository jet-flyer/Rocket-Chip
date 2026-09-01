# Early-implementation rework (2026-08-31)

**Status:** ranked redo closed except items skipped below.
**Tree:** `C:\Users\pow-w\Documents\Rocket-Chip-i2c-bus` on `grok/core1-health`.
**Not this file:** Starcom house-standards. PIO full pass is later (owner 2026-08-31).
RadioScheduler is not a Starcom MAC stub and is not rewritten — ON air is Starcom COP-P.

Outcome of each eval is **keep-with-why** or a named sitting. No cleanup rewrite.

## Ranked list — outcomes

### Redo (landed)

| Item | Outcome | Desk |
|------|---------|------|
| `i2c_bus` blank slate | Stretch-aware wait; recover 9-clock **only if SDA/SCL stuck** (init *and* runtime). `kI2cTimeoutUs` stays 10000. | Fruit Jam GPS; Feather STEMMA IMU+PA1010D at **end of chain**. |
| PA1010D I2C path | Rides the bus. 255-byte reads, 50 ms call timeout, GSA=0 PMTK314. | Same. PMTK `[51,18,51]`. |
| Core1 USB FAULT/RECOVERED | Torn seqlock is unknown, not instant FAULT. `peek_banner` sends `h` if role unknown. | COM5 FAULT=0; `bench_sim` classifies vehicle. |

Slip on the I2C slate: “9-clock only if stuck” lived in `i2c_bus_init()` only. `i2c_bus_recover()` still always pulsed; timeout and Core 1 called that. ICM `stuck_slave_recovery` 27-clocked on idle NACK (out of bus-layer scope). Fixed in `112d0a7`.

### After a live RF pass

| Item | Outcome |
|------|---------|
| SPI + RFM95W | **KEEP.** Live STOP-GAP sitting 2026-08-31: vehicle `TX sent 0 fail`; station `TRACK LQ 100%`, Lost 0, ~5 Hz `BW125 SF7 CR5`, RSSI ~−40 dBm / SNR 9 dB (desk range). Air `stop-gap`. No on-air failure to rewrite. WN-097 / Starcom ON PHY still later, not this driver. |

### Second pass

| Item | Outcome |
|------|---------|
| PIO watchdog + backup timer | **Skip.** Full PIO sitting later (owner). |
| Ring buffer / flash flush / PCM packing | **KEEP** the onboard log shape. See below. |
| RadioScheduler | **Do not rewrite.** Surpassed by Starcom COP-P on the ON path (`byte_pump`). STOP-GAP still uses the RC half-duplex scheduler. Not a Starcom MAC stub. Banned from `starcom/`. |

### Leave (unchanged)

Starcom codecs, ESKF math, lwGPS, WS2812 PIO, DPS310, ICM mag-bypass while LSM6 is the IMU. RC_OS / QP are product sittings, not this audit.

## PCM / ring / flash — why keep

Onboard log is **not** the air protocol. PCM (`include/rocketchip/pcm_frame.h`) is 55-byte IRIG-style frames (sync `0xEB90` + MET + type + `TelemetryState` + CRC-16) into a PSRAM ring, then `flash_flush` sectors + flight table. STOP-GAP air is CCSDS/MAVLink; Starcom ON air is COP-P PLTUs. Starcom has no log container today (CFDP is wanted post-mission offload, not a 50 Hz ring).

WN-059 asked whether Starcom should supersede PCM. Answer this sitting: **no.** Dual path is intentional. Stage 17 still routes through this logger.

`frame_count` is uint32 and wraps ~2.7 y at 50 Hz; after wrap `stored_count` under-reports (header comment, GWF-291). That is not a flight-day bug. Saturating the counter is a later one-line if this ring is still the logger at year-scale. Not a format rewrite.

`ring_recover` has no firmware caller (AO_Logger only `ring_init`). Crash recovery of volatile PSRAM is not claimed. Flash flight table is the durable path (HW-verified Stage 6 flush).

No blank-slate of packing, ring, or flush without a log that will not come off the pad.

## Remaining WB early-impl table (2026-08-31)

Walk of every row still on `AGENT_WHITEBOARD.md` after the ranked redo. No code.

| Item | Verdict | Why |
|------|---------|-----|
| I²C PIO master | **KEEP DW_apb.** | Stretch-aware bus + end-of-chain PA1010D already desked. PIO I2C is a backend swap for SM budget, not a hole in the current driver. Rides the later PIO sitting if budget remains. |
| Fault beacon (last-gasp) | **Sitting.** | Not implemented. Council: do not SPI-from-fault-handler; PIO beacon + SPI stop-gap in one session. Missing coverage, not a rewrite of bad code. |
| RC_OS | **Sitting.** | In progress: `grok/rcos-rework` / `docs/plans/RCOS_REWORK.md`. Still a product console, not “good enough accretion.” |
| Quaternion (Hamilton) | **KEEP.** | Scalar-first `[w,x,y,z]`, Hamilton product, body-to-NED, Sola 2017, `test/test_quat.cpp`. Matches Eigen/robotics ESKF. JPL left-multiply would retouch quat, ESKF, Mahony, logs, goldens. No fusion bug to justify that. |
| Sensor seqlock | **KEEP.** | 1 kHz Core 1 → 10 Hz Core 0 snapshot. Seqlock + DMB + 4 retries is the right tool. Vitality already treats torn as unknown. No SPSC ring needed. |
| Flash layout | **KEEP.** | Offsets derived from `PICO_FLASH_SIZE_BYTES` with overlap asserts. Revisit when a second flash size SKU exists. Not an 8 MB Feather hole. |
| PIO backup pyro timers | **KEEP the design.** | Deliberate IVP-130 independent countdown on PIO2. Remaining is pyro HW + `pyro_edge_logger` consumer, not a timer-architecture slate. Full PIO pass can pick LL-42. |
| Radio / telem | **Driver KEEP; sittings remain.** | RFM95W live STOP-GAP pass. RadioScheduler not rewritten. Still open: Starcom ON two-board soak, WN-100 compliance SSOT, radio-settings OTA. |
| CCSDS TC + COP-1 | **KEEP defer.** | Council: post–Stage-17. Starcom library already has COP-P/COP-1; RC ON uses COP-P. Do not restack STOP-GAP command retry. |

## Out

- Do not treat RadioScheduler as a Starcom MAC stub.
- Do not start PIO WDT / backup / last-gasp beacon in this eval.
- Do not rewrite RFM95W after a passing live RF sitting.
- Do not flip Hamilton → JPL without a measured fusion defect.
- Do not swap DW_apb I²C for PIO because the old “prefer PIO” lean predates the stretch-aware bus.
