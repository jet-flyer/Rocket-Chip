# Starcom Whiteboard

**Purpose:** Active flags for **Starcom-only** work. Rocket-Chip firmware leftovers stay on the repo-root [`AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md).

> Same IRL-whiteboard rule as the root board: when an item is done, **erase the row**. The library log is [`CHANGELOG.md`](CHANGELOG.md); this file only surfaces what still needs attention.

Phase / next increment: [`STATUS.md`](STATUS.md). Sequence: [`docs/IVP.md`](docs/IVP.md) (through 25). Living locks: SAD / ICD / CONFORMANCE / IVP. Do not duplicate the IVP table here.

Consumer how-to: [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md).

Worktree: `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`).

---

## House identifiers (OPEN) (2026-09-02)

No Starcom accepted-deviation row. NASA F´: functions/locals lower camel case, types PascalCase; enum/constants ALL_CAPS in F´ — house already chose `k`CamelCase over that for constants (CODING_STANDARDS supersedes JSF 45/51/52). cFS prefers CamelCase for terms. Neither NASA tree keeps Blue Book prose as C++ snake_case.

Remaining on this tree:

1. `Error` enumerators (`uslp_truncated` → `uslpTruncated`). ICD table moves with the tokens. Tests move. Not a handshake of *values* — names only.
2. JSF AV 151 named field masks + mechanical `u`→`U`. Grey map: [`docs/audits/IVP23_REPORT.md`](docs/audits/IVP23_REPORT.md). Reuse existing `k*`. Do not widen `IgnoredIntegerValues`. Do not split `macTick`.

Public verbs are already camelBack (IVP 23 initial pass).

---

## Radio settings OTA (WANTED) (2026-08-31)

Owner-wanted. Next RC consumer feature: apply `RadioConfig` (SF / BW /
CR / power / nav rate) over Starcom ON air as a cmd SDU (IVP 22
`cmd_sdu`, APID 0x003). STOP-GAP already has `SET_RADIO_CONFIG`
(MAVLink COMMAND_LONG + `kRadioConfigTable`). RC's implementation of
Starcom — not a new library increment or air dialect. Not IVP 0–25.
Not CFDP. Not a license to start this sitting. Desk: two-board ON soak,
legal table row, ACK + reconfig, link stays up.

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

2. **Decode port.** Researcher 2026-08-27: projection, not P&R. Encode is tiny. Viterbi K=7 decode is the wrong class for Forgix T8 (~7.4k LE), let alone LDPC (2048,1024) decode. Snickerdoodle 7020 (~53k LUT) is the first fabric even in the decode ballpark. 7010 / Pluto is not.
   - Honest default for this stack: **Pi as GCS decode** (can hang on HW Nathan already has, after encode goldens exist).
   - Hook the Snickerdoodle when we want a **real utilization number** for a decode port, not to close 19.

**Board order (Nathan 2026-08-28):** Forgix first, Snickerdoodle later. Snickerdoodle was only in play because it is on hand. Forgix T8 is the encode / `PhyTier::compliant` vehicle (encode is tiny). Snickerdoodle 7020 remains the first fabric in the decode-P&R ballpark.

Owner split: Researcher FPGA / Blue Book / sim-before-bitstream. Hamilton decode as a later software port (Pi first). Buzz bench bring-up when Nathan says.

---
