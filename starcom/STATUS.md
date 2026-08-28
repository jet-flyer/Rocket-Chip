# Starcom status

Library-scoped phase and next work. Lighter than Rocket-Chip `docs/PROJECT_STATUS.md`.

**Phase:** increment 6 hardening in (prefix smoke + `STARCOM_SANITIZE`). Product `0.6.0-dev`. Core still sans-I/O. ASan not run on this MinGW (no libasan). No tag. No SC-NNN.

## Next

Whiteboard: duplex/§6; PLTU repeater grade; coding-standards clang-tidy/naming audit. RC integration when scheduled. FPGA/PIO later.

1. Owner-open rows on `AGENT_WHITEBOARD.md`. Consumer map: `docs/integration/CONSUMERS.md`. Handshake: `docs/ICD.md`.

Gates: `docs/IVP.md`.

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

- None for whiteboard/RC-integration start. This MinGW g++ has no libasan/libubsan; sanitizer option waits for Clang/Linux.

## Done this sitting

- Annex C CRC-32 + PLTU encode/decode, `Starcom::starcom`, host `ctest` (`starcom.unit`), D-5 heap trap. Golden `v3-header-only` remainder `BCC004E7`.
- Host testing procedure: `docs/TESTING.md`.
- Version-3 `decode_v3` / `encode_v3`.
- Space Packet `decode_space_packet` / `encode_space_packet`; IVP `v3-one-sp-n` is 18+N.
- `Plcw16` / `Clcw32` pack/unpack.
- FOP-P / FARM-P (`copp.hpp` / `copp.cpp`): RE0–RE6, SE0–SE4/SE7, canned PLTU→PLCW host loop, `copp_take_sdu` (7.3.3). SET V(R) persistent/MAC omitted.
- USLP Version-4 (`uslp.hpp` / `uslp.cpp`): non-truncated primary header + TFDF; `decode_pltu` locates CRC-32 via 16-bit Frame Length. Truncated / Insert / FECF omitted.
- COP-1 (`cop1.hpp` / `cop1.cpp`): FARM-1 E1–E11; FOP-1 E23 + AD ack + E8 retransmit; USLP+OCF host loop. S4/S5 omitted.
- Host loopback + `RadioPort` mailbox (`adapters/host/`, `include/starcom/adapters/`). No UDP/SPI.
- Versioning: `STARCOM_VERSION` + generated `starcom/version.hpp` (RC 2026-08-26 scheme). Product `0.6.0-dev` after increment 6. No tag this sitting.
- Increment 6: codec prefix smoke (`test_fuzz.cpp`); `-DSTARCOM_SANITIZE=ON` (not exercised here — no libasan).
- Consumer map: `docs/integration/CONSUMERS.md`. Bent-pipe `repeat_pltu`. Buffered repeater still whiteboard.
- Docs cut `db1465c`. Graph snapshot `952b913`.
