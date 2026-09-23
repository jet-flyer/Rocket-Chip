# Starcom Whiteboard

**Purpose:** Active flags for **Starcom-only** work. Rocket-Chip firmware leftovers stay on the repo-root [`AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md).

> Same IRL-whiteboard rule as the root board: when an item is done, **erase the row**. The library log is [`CHANGELOG.md`](CHANGELOG.md); this file only surfaces what still needs attention.

Phase / next increment: [`STATUS.md`](STATUS.md). Sequence: [`docs/IVP.md`](docs/IVP.md) (through 25). Living locks: SAD / ICD / CONFORMANCE / IVP. Do not duplicate the IVP table here.

Consumer how-to: [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md).

Worktree: `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`).

---

## LoRa Hail (OPEN) (2026-09-02)

**Starcom** Prox-1 §6 / Best-effort hail on RFM95/915 — not a 211.1 PICS tick. Wire `MacSession`; do not reimplement §6. **Coupled with RC FPV-style scan/find** (repo-root `AGENT_WHITEBOARD.md`): same first-impl slot, **FPV scan won**. Hail is not cancelled. Stash `wip phy-scan leds docs` had both in one row. Not this sitting.

---

## FSK bitstream bearer (WANTED) (2026-09-02)

Moved from RC board. SX1276 FSK (packet and/or continuous bitstream) as a Best-effort bearer — not 211.1. Not hail. Not a license to mint LoRa SF/BW. Sit after station 2 dBm ELF is on the Jam so SET is not a +20 dBm desk shot. STATUS.md future line already names this. Not a license this sitting.

---

## Station readiness bit on air (WANTED)

Moved from RC board. Council A3: one bit the vehicle GO/NO-GO can consume (station ready). Today the channel is command-only; no periodic TM-back. COP-P is the air; do not invent a second ARQ. Not IVP 0–25. Not a license this sitting.

---

## SDLS 355.0 telecommand auth (WANTED)

Moved from RC board. CCSDS 355.0 for the Rocket profile — frame auth, not CFDP checksum. Library increment when scheduled, above `starcom::ccsds`. Not IVP 0–25. Not a license this sitting.

---

## Radio settings OTA (WANTED) (2026-08-31)

Owner-wanted. Next RC consumer feature: apply `RadioConfig` (SF / BW /
CR / power / nav rate) over Starcom ON air as a cmd SDU (IVP 22
`cmd_sdu`, APID 0x003). USB/local `SET_RADIO_CONFIG` already exists
(MAVLink COMMAND_LONG + `kRadioConfigTable` + ToA gate). RC's
implementation of Starcom — not a new library increment or air dialect.
Not IVP 0–25. Not CFDP. Not a license to start this sitting. Desk:
two-board ON soak, legal table row, ACK + reconfig, link stays up.

---

## CFDP post-mission offload (WANTED) (2026-08-27)

Owner-wanted. **CCSDS 727.0** file delivery with a checksum over the blob — data offload after a mission, not live TM. Rides in Space Packet user data. Not IVP 0–25. Not SDLS 355.0 (that is telecommand frame auth; RC whiteboard has SDLS for the Rocket profile). Not PLTU CRC-32.

When scheduled: own stack module above `starcom::ccsds`, not a codec sitting.

---

## FPGA compliant PHY + decode port (HELD) (2026-08-28)

Held until **base-level verification on the FPGA board**. Not increment 19. Not a license to start without that sitting.

**Already in (do not redo):** increment 18 uncoded `PhyTier` none / best_effort on the host path. Increment 19 is host encode (conv K=7 r=1/2 with G2 inversion, LDPC (2048,1024) + CSM + codeword randomize). Host goldens close 19. Buzz: Forgix / Snickerdoodle / Pi stay off the bench while 19 is host encode.

**Held — cut these as their own sittings, not as 19 leftovers:**

1. **`PhyTier::compliant` / 211.1 waveform / FPGA bitstream** (the 18 claim we did not make). HDL sim before bitstream (Researcher). Same codec vectors as the host uncoded path. No Electra / UT product claim.

2. **Decode port.** Researcher 2026-08-27: projection, not P&R. Encode is tiny. Viterbi K=7 decode is the wrong class for Forgix T8 (~7.4k LE), let alone LDPC (2048,1024) decode. Snickerdoodle on hand is the one (~17.6k LUT). Same call as Pluto: not in the LDPC decode ballpark.
   - Honest default for this stack: **Pi as GCS decode** (can hang on HW Nathan already has, after encode goldens exist).
   - Hook the Snickerdoodle when we want a **real utilization number** for a decode port, not to close 19.

**Board order (Nathan 2026-08-28):** Forgix first, Snickerdoodle later. Snickerdoodle was only in play because it is on hand (the one, ~17.6k LUT). Forgix T8 is the encode / `PhyTier::compliant` vehicle (encode is tiny).

Owner split: Researcher FPGA / Blue Book / sim-before-bitstream. Hamilton decode as a later software port (Pi first). Buzz bench bring-up when Nathan says.

---
