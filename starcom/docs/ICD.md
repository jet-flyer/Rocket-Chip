# Starcom core ICD

**Status:** Draft. Principles plus named verbs. Signatures land with the first codec, not invented here. Namespace `starcom::ccsds`.

This is the handshake at the core boundary. The SAD is the map. Conformance is the claim table. Do not treat example spellings below as the API.

## Two layers

Codecs (Phase 1–2) are pure encode/decode. They take spans and return a value or an error. They do not own a radio, a clock, or a session.

The engine verbs exist once there is state (COP, or a framing pump that retains parse state). A CRC helper does not grow `receive_bytes`.

## What a host test may call

A host test of the core needs no hardware. It may:

- Call codec functions with canned octets and assert the result (golden PLTU / V-3 / USLP / Space Packet vectors).
- Once the engine exists: feed octets in, drain octets out, pass `now`, poll events, submit an SDU.

It may not: open a socket, poke SPI, sleep inside the core, or include Rocket-Chip types.

## Settled principles

- Sans-I/O. The core holds no I/O object. The consumer owns radio, clock, and event loop.
- Caller owns buffers. Input is a non-owning span of caller memory. Output is written into a caller span. The core does not allocate after init.
- Caller owns state. Protocol state lives in caller-provided static structs. The core is functions over that state.
- Public API is value-or-error. Default `tl::expected`. Compile knob to `std::expected` or `starcom::Result`. No exceptions across the core API. Error objects trivially copyable, no heap.
- C++20. `std::span` is the buffer type. Do not vendor a span backport.
- Time: the core never reads a clock. The caller passes `now`. The C++ typedef lands with COP-P timers.
- Strong IDs (`Scid`, `Vcid`, `MapId`, …). Version-3 uses PCID / Port ID; those are not USLP VCID / MAP.

## Named verbs

From DESIGN / library_craft, for the engine, not for Phase 1 helpers:

| Verb | Job |
|------|-----|
| `receive_bytes` | Caller gives inbound octets. |
| `bytes_to_send` | Caller drains outbound octets. |
| `poll_event` | Caller takes semantic events. |
| `handle_timeout` / `tick` | Caller passes `now`. Core owns no clock. |
| `submit_sdu` | Caller gives a Space Packet (or equivalent SDU) to send. |

Exact types, error codes, and whether output is a drain or a write-into-span land with the first codec / COP-P caller. Do not invent them here.

**Repeater path (MVP with codecs, not with COP-P).** Same verbs, different use: `receive_bytes` of a candidate PLTU, `bytes_to_send` of that same span if ASM + CRC-32 pass and the V-3 FSN is new. No `submit_sdu`. No COP timers. The consumer still owns "radio is free" (half-duplex). Signatures land with the PLTU codec, same rule as the rest of this file.

A later buffered grade still uses those verbs; the caller passes a queue (span of slots). Depth and backing store (PSRAM on an RC relay profile, SRAM on a dual-use board, host test array) are not core types.

## Not this document

PHY, adapters, COP tables, CMake, and Blue Book field maps. Wire picture: `SAD.md`. Claims: `CONFORMANCE.md`.
