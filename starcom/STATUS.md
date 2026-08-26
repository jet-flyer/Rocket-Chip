# Starcom status

Library-scoped phase and next work. Lighter than Rocket-Chip `docs/PROJECT_STATUS.md`.

**Phase:** docs cut on `docs/starcom-sad-draft`. No library code, no CMake targets, not in the root build.

## Next

1. Codecs: PLTU, Version-3, Space Packet SDU, PLCW/CLCW field pack. CMake (`Starcom::starcom` + host ctest + `tl::expected` / span seams) lands with the first `.cpp`, not as a solo sitting.
2. COP-P procedures (FOP-P/FARM-P). That is the Prox ARQ, not optional.

After codecs + COP-P: USLP, COP-1, adapters, then a Starcom IVP fed from this STATUS and `docs/CONFORMANCE.md`. FPGA sim is later (Researcher / Buzz).

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

**MVP cut (2026-08-25):** Phases 0–2: CMake-with-first-codec, codecs, COP-P. USLP and COP-1 are in, sequenced next. Order of implementation, not a maybe. 131.0 long-haul coding is not this MVP. PHY / 211.1 is a later port. Prox-1 §6 hailing/MAC is not decided (full module vs out); decide when we implement it. No stub.

## Blockers

- None for the first codec. RC half-duplex flight pain drives *when* RC integrates; it does not block the host core.

## Done this sitting

- SAD with on-the-wire PLTU figure (`starcom/docs/SAD.md`).
- ICD, CONFORMANCE, this STATUS, WORKING_HERE vocabulary.
- Identity README and folder READMEs un-placeholdered. DESIGN pointer so stale D-4 is not read as current.
