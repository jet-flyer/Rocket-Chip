# Starcom Whiteboard

**Purpose:** Active flags for **Starcom-only** work. Rocket-Chip firmware leftovers stay on the repo-root [`AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md).

> Same IRL-whiteboard rule as the root board: when an item is done, **erase the row**. The library log is [`CHANGELOG.md`](CHANGELOG.md); this file only surfaces what still needs attention.

Phase / next increment: [`STATUS.md`](STATUS.md). Sequence: [`docs/IVP.md`](docs/IVP.md) (through 25). Living locks: SAD / ICD / CONFORMANCE / IVP. Do not duplicate the IVP table here.

---

## copp_init stack smash — log at wrap, not the fix commit (2026-08-28)

`copp_init` / `copp_init_uslp` / `cop1_init` memset in place. `e = CoppEndpoint{}` was an ~18 KiB stack temp; Pico Core 0 is 4 KiB; first ON UF2 reset-looped. **Home at wrap:** [`CHANGELOG.md`](CHANGELOG.md) (Starcom library bugfix). Root `CHANGELOG.md` only if that wrap also covers the Pico ON sitting — confirm (checklist item 8). Do not mint a row on the copp-init commit.

---

## Host dissect demo — teaching tool (SIDETRACK) (2026-08-27)

Break-time only. Not mainline IVP. Lives in `starcom/examples/` when scheduled — host CLI, not `starcom::ccsds`, not Rocket-Chip firmware (RC’s station dashboard / `telemetry_encoder` stay in RC until IVP 20–22).

**Input is a telem source or the line**, not typed junk as the whole package. Pipe/file/canned encoder output of PLTUs (and later a stream for `hunt_pltu`). Typing is only the Space Packet *user field* (the small N in 18+N). Helper text per section cites the Blue Book (SAD field maps as a working copy).

| When | Extra on the same demo |
|------|-------------------------|
| Now (through IVP 12) | Dissect one PLTU; hunt a stream of several |
| IVP 9 | Truncated USLP, Insert Zone, FECF labeled |
| COP already in | FSN / PLCW / CLCW as “why this octet exists” |
| IVP 15 | File/UDP in, same display |

Do not invent a GUI. Do not couple to RC pins or SX1276.

---

## CFDP post-mission offload (WANTED) (2026-08-27)

Owner-wanted. **CCSDS 727.0** file delivery with a checksum over the blob — data offload after a mission, not live TM. Rides in Space Packet user data. Not IVP 0–25. Not SDLS 355.0 (that is telecommand frame auth; RC whiteboard has SDLS for the Rocket profile). Not PLTU CRC-32.

When scheduled: own stack module above `starcom::ccsds`, not a codec sitting.

---

## FPGA compliant PHY + decode port (HELD) (2026-08-27)

Not 19. Not a license to start tonight. Nathan 2026-08-27: flesh this out next sitting so it is not a skip.

**Already in (do not redo):** increment 18 uncoded `PhyTier` none / best_effort on the host path. Increment 19 is host encode (conv K=7 r=1/2 with G2 inversion, LDPC (2048,1024) + CSM + codeword randomize). Host goldens close 19. Buzz: Forgix / Snickerdoodle / Pi stay off the bench while 19 is host encode.

**Held — cut these as their own sittings, not as 19 leftovers:**

1. **`PhyTier::compliant` / 211.1 waveform / FPGA bitstream** (the 18 claim we did not make). HDL sim before bitstream (Researcher). Same codec vectors as the host uncoded path. No Electra / UT product claim.

2. **Decode port.** Researcher 2026-08-27: projection, not P&R. Encode is tiny. Viterbi K=7 decode is the wrong class for Forgix T8 (~7.4k LE), let alone LDPC (2048,1024) decode. Snickerdoodle 7020 (~53k LUT) is the first fabric even in the decode ballpark. 7010 / Pluto is not.
   - Honest default for this stack: **Pi as GCS decode** (can hang on HW Nathan already has, after encode goldens exist).
   - Hook the Snickerdoodle when we want a **real utilization number** for a decode port, not to close 19.

**Board order (Nathan 2026-08-28):** Forgix first, Snickerdoodle later. Snickerdoodle was only in play because it is on hand. Forgix T8 is the encode / `PhyTier::compliant` vehicle (encode is tiny). Snickerdoodle 7020 remains the first fabric in the decode-P&R ballpark.

Owner split: Researcher FPGA / Blue Book / sim-before-bitstream. Hamilton decode as a later software port (Pi first). Buzz bench bring-up when Nathan says.

---
