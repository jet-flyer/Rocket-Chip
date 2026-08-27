# Starcom status

Library-scoped phase and next work. Lighter than Rocket-Chip `docs/PROJECT_STATUS.md`.

**Phase:** increment 0+1 in progress. CRC-32 + PLTU + Version-3 + Space Packet + `Starcom::starcom` + host `ctest` (`starcom.unit`). Not in the Pico firmware build. Starcom graph: `starcom/graphify-out/`.

## Next

`Plcw16` / `Clcw32` pack/unpack (same increment). Then COP-P.

Owner-open (whiteboard): duplex/§6; repeater grade (after remaining 0+1 codecs).

1. `Plcw16` / `Clcw32` pack. Handshake: `docs/ICD.md`. Tests: `docs/TESTING.md`.
2. COP-P procedures (FOP-P/FARM-P). That is the Prox ARQ, not optional.

After codecs + COP-P: USLP, COP-1, adapters. Gates: `docs/IVP.md`. FPGA sim is later (Researcher / Buzz).

## Phase sketch

Transcribed from `docs/research/library_craft_claude.md` §7, with this sitting's MVP cut. Grok's library-craft doc has no numbered 0–6 list; cite it for host-first testing, size reporting, and "F' is an integration target" (Grok §8, §10). Do not treat Claude's module filenames as the only cut.

| Phase | Job | Notes |
|-------|-----|--------|
| 0 | Skeleton | CMake static + export, `version` / `result` / span seams, empty host test. Lands with the first codec, not alone. |
| 1 | Codecs | Pure functions. PLTU (ASM+CRC-32), Version-3 frame, Space Packet SDU, PLCW and CLCW field pack/unpack. Golden vectors. |
| 2 | COP-P | FOP-P / FARM-P. This is implementing Prox reliability, not extra. Engine verbs become real here. |
| 3 | USLP | Version-4 frame + VC/MAP in the same PLTU. Can host COP-P. |
| 4 | COP-1 | FOP-1 / FARM-1. The other ARQ, not a substitute for COP-P. |
| 5 | Adapters | Host loopback first. Generic radio port in `starcom/adapters/`. RC pins/AO stay in RC. |
| 6 | Hardening | Sanitizers, longer fuzz, docs, first `0.1.0`. |

**MVP cut (2026-08-25):** Phases 0–2: CMake-with-first-codec, codecs, COP-P (endpoints only). USLP and COP-1 are in, sequenced next. Order of implementation, not a maybe. 131.0 long-haul coding is not this MVP. PHY / 211.1 is a later port. Prox-1 §6 hailing/MAC is not decided. **PLTU repeater is not decided** — awareness on `AGENT_WHITEBOARD.md` (last night’s MVP lock walked back). No stub.

## Blockers

- None for V-3. RC half-duplex flight pain drives *when* RC integrates; it does not block the host core.

## Done this sitting

- Annex C CRC-32 + PLTU encode/decode, `Starcom::starcom`, host `ctest` (`starcom.unit`), D-5 heap trap. Golden `v3-header-only` remainder `BCC004E7`.
- Host testing procedure: `docs/TESTING.md`.
- Version-3 `decode_v3` / `encode_v3`.
- Space Packet `decode_sp` / `encode_sp`; IVP `v3-one-sp-n` is 18+N.
- Docs cut `db1465c`. Graph snapshot `952b913`.
