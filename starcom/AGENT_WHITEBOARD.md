# Starcom Whiteboard

**Purpose:** Active flags for **Starcom-only** work. Rocket-Chip firmware leftovers stay on the repo-root [`AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md).

> Same IRL-whiteboard rule as the root board: when an item is done, **erase the row**. The library log is [`CHANGELOG.md`](CHANGELOG.md); this file only surfaces what still needs attention.

Phase / next increment: [`STATUS.md`](STATUS.md). Living locks: SAD / ICD / CONFORMANCE / IVP. Do not duplicate those here.

---

## PLTU repeater — early RC capability (OPEN) (2026-08-27)

Owner: RP2350 + LoRa should be able to **repeat a PLTU** (regenerative: check envelope, same octets out, no payload decode). Not a Prox-1/long-haul gateway.

**Not the first `.cpp`.** Codecs (Annex C CRC + PLTU) come first — you cannot honestly repeat without the envelope check. Repeater is in mind from day one; it is not a LoRa-shaped fork in the core.

**Grade:** bent-pipe (one unit) vs buffered (caller-owned queue; 133.0 §2.4 store-and-forward). Owner: full/buffered can follow immediately, or skip bent-pipe if buffered is cheap enough in RAM/CPU. Measure at implementation; do not invent a depth now. Dedup key not locked.

Last night’s “MVP lock in every living doc” overstated a missing whiteboard. This row is the home until a sitting implements it.

---

## First codec: PLTU from 211.2 Annex C (OPEN) (2026-08-27)

Ready to code when scheduled. Increment 0+1. Handshake: `docs/ICD.md`. Spec: 211.2-B-3 §3.2.5 + Annex C + Fig C-1.

The open-source *library* gap is real; the *spec* is not. CRC-32 for a PLTU is 211.2-B-3 §3.2.5 + Annex C (normative) + Fig C-1. One procedure. Starcom writes that encoder. Do not invent a different polynomial. Do not stitch Ethernet/`crc32()`, TM FECF CRC-16, or TC frames into it. Do not wait for Electra source.

Green Book 210.0-G-2 Annex B restates the same G(X). Interop-test lessons are in that Green Book’s annex on test campaigns. Mars papers are operational color.

Space Packet / CLCW / COP-1 still use cFS, Yamcs, OSDLP, `cop1.c` as prior art for *those* layers.

---

## Duplex / §6 / radios (OPEN) (2026-08-27)

Do **not** couple the core to RC’s current half-duplex LoRa. Other radios (dual-radio, possible full duplex) stay in mind.

211.0 DUPLEX = full / half / simplex lives in **§6 MAC**, not in the PLTU/V-3 codecs. 211.1 is the Prox-1 UHF PHY (not SX1276). Adapters declare what the hardware can do. Full §6 vs a small turnaround helper vs consumer-only: not picked. RC integration may go first **if it does not lock the core out of full/simplex/dual-radio**.

ELRS Gemini is **not** CCSDS full duplex (see DESIGN note). Hardware with two transceivers *can* TX on one and RX on the other; that is a port, not a codec fork.

---

## PIO / FPGA port seams (OPEN) (2026-08-27)

Core stays byte-level. When special features land, they attach as **ports** (IVP increment 5 or a dedicated sitting), same codec vectors:

- **PIO** — ASM hunt, symbol timing, Manchester / FSK-continuous on RP2350.
- **FPGA** — 211.1 / C&S (conv, PLTU on the wire; LDPC/Viterbi as fabric allows). HDL sim before bitstream.

Do not bake PIO or FPGA types into `include/starcom`. SAD “Later port seams” is the map.

---

## Simplex + bitstream (FUTURE) (2026-08-27)

Keep in mind. Not increment 0+1. Not a stub.

Simplex is one-way (211.0 §6; no hailing). A one-way **bitstream** / user-defined data path (V-3 DFC `11`, Odyssey “Unreliable Bitstream” as a *mission* receiver mode — Annex F, not the general codec) is a later feature. Do not couple it to LoRa FSK-continuous unless a port sitting says so. Codecs stay the same PLTU.

---
