# Starcom status

Library-scoped phase and next work. Lighter than Rocket-Chip `docs/PROJECT_STATUS.md`.

**Phase:** increment 15 host UDP / file replay in. Product `0.15.0-dev`. **Sans-I/O data-link core cut (0–14); host file/UDP port (15).** Next is 16 (generic SPI/GPIO radio port). ASan not run on this MinGW (no libasan). No tag. No SC-NNN.

## Next

IVP increment 16 (generic SPI/GPIO radio port). Sequence through 25 is in `docs/IVP.md`. RC integration (IVP 20–22) when scheduled. Host dissect demo is a Starcom WB sidetrack. CFDP (727.0) is wanted post-mission offload, not 0–25.

1. Owner-open rows on `AGENT_WHITEBOARD.md`. Consumer map: `docs/integration/CONSUMERS.md`. Handshake: `docs/ICD.md`. Plan: `docs/IVP.md`.

Gates: `docs/IVP.md`.

## Phase sketch

Transcribed from `docs/research/library_craft_claude.md` §7, then numbered as this tree’s IVP. Grok's library-craft doc has no numbered 0–6 list; cite it for host-first testing, size reporting, and "F' is an integration target" (Grok §8, §10). Do not treat Claude's module filenames as the only cut.

| Phase | Job | Notes |
|-------|-----|--------|
| 0+1 | Skeleton + codecs | CMake with first codec. PLTU, V-3, Space Packet, PLCW, CLCW. |
| 2 | COP-P | FOP-P / FARM-P. SET V(R) persistent waits for 13. |
| 3 | USLP | Non-truncated V-4 in the same PLTU. Remainder is 9. |
| 4 | COP-1 | FARM-1 + FOP-1 subset. Remainder is 10. |
| 5 | Adapters | Host loopback + `RadioPort` mailbox. |
| 6 | Hardening (option) | Prefix smoke + `STARCOM_SANITIZE`. Close is 24. |
| 7 | Bent-pipe repeater | `repeat_pltu`. Buffered is 12. |
| 8 | ASM hunt | 211.2 C&S stream search. |
| 9 | USLP remainder | Truncated, Insert Zone, FECF (MIB). |
| 10 | COP-1 remainder | S4/S5 + remaining Table 5-1. |
| 11 | COP-P on USLP VC | Same procedures, V-4. |
| 12 | Buffered repeater | Caller-owned queue; no invented depth. |
| 13 | §6 MAC / DUPLEX | Full module (2026-08-27). SET V(R) persistent. |
| 14 | User-defined DFC 11 | Opaque U-frame data. Not Annex F. Simplex already increment 13. |
| 15–18 | Ports | UDP/file, SPI/GPIO, PIO, PHY/FPGA tiers. |
| 19 | Conv / LDPC | 211.2 PICS. |
| 20–22 | RC consumer | Host link, Pico+AO, replace `telemetry_encoder` with COP. |
| 23–25 | Audit, hardening close, tag | Standards; ASan/fuzz/size; `starcom-v*`. |

**MVP cut (2026-08-25, historical):** Phases 0–2 were the first code cut. USLP and COP-1 followed in order. Superseded by the IVP 0–25 table: repeater is 7/12 (not “not decided”); §6 is 13 (still a decision, not a stub). 131.0 long-haul TM C&S is not a Starcom increment. PHY / 211.1 is increment 18 as adapter tiers, not a blanket claim. No stub.

## Blockers

- None for increment 15. This MinGW g++ has no libasan/libubsan; sanitizer *run* waits for increment 24 on Clang/Linux.

## Done this sitting

- Annex C CRC-32 + PLTU encode/decode, `Starcom::starcom`, host `ctest` (`starcom.unit`), D-5 heap trap. Golden `v3-header-only` remainder `BCC004E7`.
- Host testing procedure: `docs/TESTING.md`.
- Version-3 `decode_v3` / `encode_v3`.
- Space Packet `decode_space_packet` / `encode_space_packet`; IVP `v3-one-sp-n` is 18+N.
- `Plcw16` / `Clcw32` pack/unpack.
- FOP-P / FARM-P (`copp.hpp` / `copp.cpp`): RE0–RE6, SE0–SE4/SE7, canned PLTU→PLCW host loop, `copp_take_sdu` (7.3.3). SET V(R) persistent/MAC is increment 13.
- USLP Version-4 (`uslp.hpp` / `uslp.cpp`): non-truncated primary header + TFDF; truncated (annex D), Insert Zone, FECF Annex B via caller `UslpMib`.
- COP-1 (`cop1.hpp` / `cop1.cpp`): FARM-1 E1–E11; FOP-1 E23 + S4/S5 BC-init (E24/E25/E27) + E29 terminate; USLP+OCF host loop.
- Host loopback + `RadioPort` mailbox (`adapters/host/`, `include/starcom/adapters/`). UDP/file is increment 15; SPI is 16.
- Versioning: `STARCOM_VERSION` + generated `starcom/version.hpp` (same scheme as RC's 2026-08-26 `RC_VERSION` close — Starcom file is `STARCOM_VERSION`). Product `0.12.0-dev` after increment 12. No tag this sitting.
- Increment 6: codec prefix smoke (`test_fuzz.cpp`); `-DSTARCOM_SANITIZE=ON` (not exercised here — no libasan). Close is 24.
- Consumer map: `docs/integration/CONSUMERS.md`. Bent-pipe `repeat_pltu` (increment 7).
- Increment 8: `hunt_pltu` (211.2 §3.6 exact-ASM search, caller leftover, no library buffer).
- Increment 9: truncated USLP (annex D), Insert Zone, FECF CRC-16 (Annex B). `UslpMib` from caller.
- Increment 10: COP-1 S4/S5 BC-init (E24/E25/E27) + E29 terminate. Unlock/Set V(R) on the wire.
- Increment 11: `copp_init_uslp` — COP-P on Version-4 VC/MAP; PLCW still SPDU. V-3 path unchanged.
- Increment 12: caller-owned `PltuRepeatQ` / `enqueue_pltu` / `dequeue_pltu`. No invented depth.
- IVP sequence through increment 25 (rest of the stack). No Starcom stop-gap. RC `telemetry_encoder` is RC firmware until increment 22.
- Increment 13: full §6 MAC (`mac.hpp` / `mac.cpp`). SET V(R) persistent. Simplex S71/S72.
- Increment 14: V-3 DFC `11` user-defined (`encode_v3_user_defined`, `copp_submit_user_defined`). Opaque octets, no reassembly. Not Annex F4.
- Increment 15: host file replay + UDP (`replay_pltu_file`, `udp_*`). Sockets only in `adapters/host`. No Starcom service port.
- Docs cut `db1465c`. Graph snapshot `952b913`.
