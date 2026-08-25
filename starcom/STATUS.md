# Starcom status

Library-scoped phase and blockers. Lighter than Rocket-Chip `docs/PROJECT_STATUS.md`.

**Phase:** pre-Phase-0 docs. SAD draft is on `docs/starcom-sad-draft`. ICD, conformance, and this file land with it. No library code, no CMake targets, not in the root build.

## Next

1. Phase 0 CMake (`Starcom::starcom`), host ctest shell, `tl::expected` + span seams. Header-vs-static default is a spike, not a lock.
2. Phase 1 codecs, Blue Book order: PLTU, Version-3, Space Packet SDU, then USLP.

README identity is drafted in chat; not required to start Phase 0.

## Phase sketch

Transcribed from `docs/research/library_craft_claude.md` §7, with this sitting's MVP cut. Grok's library-craft doc has no numbered 0–6 list; cite it for host-first testing, size reporting, and "F' is an integration target" (Grok §8, §10). Do not treat Claude's module filenames as the only cut.

| Phase | Job | Notes |
|-------|-----|--------|
| 0 | Skeleton | CMake static + export, `version` / `result` / span seams, empty host test. Prove the target exists. |
| 1 | Codecs | Pure functions. PLTU (ASM+CRC-32), Version-3 frame, Space Packet SDU. Golden vectors. No state machines. |
| 2 | USLP | Version-4 frame + VC/MAP. Same PLTU. Table tests. |
| 3 | COP-1 | FOP-1 / FARM-1 as sans-I/O machines. Engine verbs become real. |
| 4 | COP-P | FOP-P / FARM-P + PLCW. Hailing/session is not this phase by default. |
| 5 | Adapters | Host loopback first. Generic radio port in `starcom/adapters/`. RC pins/AO stay in RC. |
| 6 | Hardening | Sanitizers, longer fuzz, docs, first `0.1.0`. |

**MVP cut (2026-08-25):** Phases 0–2 (both frame types in one PLTU). COP-P is deferred unless pulled forward. 131.0 long-haul coding is not this MVP. PHY / 211.1 is a later port.

## Blockers

- None for Phase 0 CMake. RC half-duplex flight pain drives *when* RC integrates; it does not block the host core.
- Open, not blockers: COP-P in or after the frame MVP; Prox-1 §6 later vs absent; header-vs-static default.

## Done this sitting

- SAD draft with on-the-wire PLTU figure (`starcom/docs/SAD.md`).
- Identity README drafted in chat, not landed.
