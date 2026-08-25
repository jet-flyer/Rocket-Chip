# Starcom status

Library-scoped phase and blockers. Lighter than Rocket-Chip `docs/PROJECT_STATUS.md`.

**Phase:** pre-Phase-0 docs. SAD draft is on `docs/starcom-sad-draft`. ICD, conformance, and this file land with it. No library code, no CMake targets, not in the root build.

## Next

1. Phase 0 CMake (`Starcom::starcom`), host ctest shell, `tl::expected` + span seams. Header-vs-static default is a spike, not a lock.
2. Phase 1 codecs: PLTU, Version-3, Space Packet SDU, PLCW/CLCW field pack.
3. Phase 2 COP-P procedures (FOP-P/FARM-P). That is the Prox ARQ, not optional.

README identity is drafted in chat; not required to start Phase 0.

## Phase sketch

Transcribed from `docs/research/library_craft_claude.md` §7, with this sitting's MVP cut. Grok's library-craft doc has no numbered 0–6 list; cite it for host-first testing, size reporting, and "F' is an integration target" (Grok §8, §10). Do not treat Claude's module filenames as the only cut.

| Phase | Job | Notes |
|-------|-----|--------|
| 0 | Skeleton | CMake static + export, `version` / `result` / span seams, empty host test. Prove the target exists. |
| 1 | Codecs | Pure functions. PLTU (ASM+CRC-32), Version-3 frame, Space Packet SDU, PLCW and CLCW field pack/unpack. Golden vectors. |
| 2 | COP-P | FOP-P / FARM-P. This is implementing Prox reliability, not extra. Engine verbs become real here. |
| 3 | USLP | Version-4 frame + VC/MAP in the same PLTU. Can host COP-P. |
| 4 | COP-1 | FOP-1 / FARM-1. The other ARQ, not a substitute for COP-P. |
| 5 | Adapters | Host loopback first. Generic radio port in `starcom/adapters/`. RC pins/AO stay in RC. |
| 6 | Hardening | Sanitizers, longer fuzz, docs, first `0.1.0`. |

**MVP cut (2026-08-25):** Phases 0–2: CMake, codecs, COP-P. USLP and COP-1 are in, sequenced next. Order of implementation, not a maybe. 131.0 long-haul coding is not this MVP. PHY / 211.1 is a later port. §6 hailing/MAC is later or absent.

## Blockers

- None for Phase 0 CMake. RC half-duplex flight pain drives *when* RC integrates; it does not block the host core.
- Open, not blockers: Prox-1 §6 later vs absent; header-vs-static default.

## Done this sitting

- SAD draft with on-the-wire PLTU figure (`starcom/docs/SAD.md`).
- Identity README drafted in chat, not landed.
