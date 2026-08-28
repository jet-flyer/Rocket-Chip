# Starcom conformance claims

**Status:** Draft. Claims, not implementation rows. No Implemented / stub column until there is code. No JPL User Terminal table.

Honesty: a claim is in-scope, deferred, not decided, or out of scope. "Best effort" is not a claim. PHY tiers belong here, not in the README.

Book cites here are pointers. The Blue Book is the claim; this table is the index.

| Claim | Book | Status | Notes |
|-------|------|--------|--------|
| PLTU: ASM `FAF320` + transfer frame + CRC-32, uncoded | 211.2-B-3 Fig 3-1 | In scope (MVP; hunt IVP 8) | Envelope. One frame version per stream. `decode_pltu` / `hunt_pltu` (`tests/unit/test_pltu.cpp`). Hunt is 211.2 §3.6 exact-ASM search. No SC-NNN. |
| PLTU repeater (bent-pipe and/or buffered) | Not a Blue Book product. Related: 211.2 C&S check; 133.0-B-2 §2.4 (subnetwork storage/forwarding assumed, not an SPP procedure) | In scope (IVP 7 bent-pipe; 12 buffered) | `repeat_pltu` / `enqueue_pltu` / `dequeue_pltu` (`tests/unit/test_pltu.cpp`). Caller-owned queue. No COP on this path. |
| Version-3 transfer frame | 211.0-B-6 Fig 3-2 | In scope (MVP) | First insides of PLTU. 5-octet header, 2 KiB cap. Tests: `tests/unit/test_v3.cpp`. |
| Version-4 / USLP transfer frame in the same PLTU | 732.1-B-3 Fig 4-1 | In scope (MVP + IVP 9 remainder) | Non-truncated + truncated (annex D) + Insert Zone + FECF Annex B (`tests/unit/test_uslp.cpp`). Not nested in the V-3 data field. No SC-NNN. |
| Space Packet as SDU | 133.0-B-2 Fig 4-1 | In scope (MVP) | 6-octet header + user data. Not a Starcom product name. Tests: `tests/unit/test_space_packet.cpp`. |
| PLCW 16-bit SPDU field codec | 211.0-B-6 §3.2.4.3.2.1.1 | In scope (MVP codecs) | Pack/unpack only. Not the ARQ. Distinct from CLCW. No generic OCF. Tests: `tests/unit/test_ocf.cpp`. |
| CLCW 32-bit field codec | 232.0-B-4 §4.2.1 | In scope (MVP codecs) | Pack/unpack only. Lives in a USLP OCF later; still a pure codec now. Tests: `tests/unit/test_ocf.cpp`. |
| COP-P procedures (FOP-P / FARM-P) | 211.0-B-6 §7 | In scope (MVP + IVP 11 USLP VC) | Tables + `CoppEndpoint` / `copp_init_uslp` (`tests/unit/test_copp.cpp`). SET V(R) persistent is MAC — increment 13. No SC-NNN. |
| COP-1 procedures (FOP-1 / FARM-1) | 232.1-B-2 | In scope (MVP + IVP 10 S4/S5) | FARM-1 Table 6-1 + FOP-1 E23/S4/S5/E29 (`tests/unit/test_cop1.cpp`). Suspend/resume E30–E34 and LLIF E41–E46 not this sitting. No SC-NNN. |
| Prox-1 session / MAC / hailing | 211.0-B-6 §6 | In scope (IVP 13 full module) | Owner pick 2026-08-27: full §6, not turnaround helper, not consumer-only. Tables 6-2–6-13 + SET V(R) 7.2.3.2 (`tests/unit/test_mac.cpp`). No radio objects in the core. No SC-NNN. |
| Convolutional or LDPC coding | 211.2 → 131.0-B-3 | Deferred (IVP 19) | 211.2 PICS: at least one of uncoded / conv / LDPC. MVP is uncoded PLTU. |
| Long-haul TM C&S (131.0 ASM / FECF path) | 131.0 | Out of scope for this MVP | Different coding sublayer than PLTU. |
| 211.1-B-4 Physical Layer | 211.1-B-4 | Out of scope as a blanket claim (IVP 18 tiers) | No Electra/UT product claim. Adapters declare none / best-effort / compliant. |
| JPL User Terminal / Electra interop as a product claim | — | Out of scope | Prox-1 V-3 is the interop *frame*, not a UT claim. |
| Mixed V-3 and V-4 on one PLTU stream | 211.2-B-3 §3.2.4 | Out of scope | Forbidden by the book. |
| F' as a Starcom dependency | — | Out of scope | Integration target only (Grok §10). |
| CFDP file delivery (post-mission data offload) | 727.0-B-5 | Deferred (wanted; not IVP 0–25) | Checksummed file transfer in Space Packet user data. Owner-wanted after the data-link core. Not SDLS (355.0 is TC frame auth). Not `starcom::ccsds` codecs. |

When a row is implemented, add a test pointer. Do not retcon status to Implemented without that pointer.
