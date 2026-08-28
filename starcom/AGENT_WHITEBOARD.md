# Starcom Whiteboard

**Purpose:** Active flags for **Starcom-only** work. Rocket-Chip firmware leftovers stay on the repo-root [`AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md).

> Same IRL-whiteboard rule as the root board: when an item is done, **erase the row**. The library log is [`CHANGELOG.md`](CHANGELOG.md); this file only surfaces what still needs attention.

Phase / next increment: [`STATUS.md`](STATUS.md). Sequence: [`docs/IVP.md`](docs/IVP.md) (through 25). Living locks: SAD / ICD / CONFORMANCE / IVP. Do not duplicate the IVP table here.

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
