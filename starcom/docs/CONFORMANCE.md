# Starcom conformance claims

**Status:** Draft. Claims, not implementation rows. No Implemented / stub column until there is code. No JPL User Terminal table.

Honesty: a claim is in-scope, deferred, or out of scope. "Best effort" is not a claim. PHY tiers belong here, not in the README.

| Claim | Book | Status | Notes |
|-------|------|--------|--------|
| PLTU: ASM `FAF320` + transfer frame + CRC-32, uncoded | 211.2-B-3 Fig 3-1 | In scope (MVP) | Envelope. One frame version per stream. |
| PLTU repeater: valid unit in, same octets out | 211.2-B-3 §3.5–3.6 (C&S send/receive). Dedup: 211.0-B-6 V-3 FSN | In scope (MVP, with codecs) | Regenerative range-extend. Bit-exact pass-through after ASM + CRC-32. No Space Packet parse. No COP-P/COP-1 on this path. Not a 211.0 session and not a second-link gateway. Half-duplex "radio is free" stays with the consumer. Dual-use board (already doing another job): this path. |
| Buffered PLTU repeater (caller-owned queue) | 133.0-B-2 §2.4 (subnetwork storage and forwarding is assumed, not an SPP procedure). Same C&S envelope as the row above | Deferred | Dedicated relay node. Core stays sans-I/O: caller provides the queue. Rocket-Chip may back it with PSRAM on a relay mission profile (no IMU/fusion working set). Not DTN/Bundle Protocol (734.2) unless a later sitting says so. Do not invent a depth now. |
| Version-3 transfer frame | 211.0-B-6 Fig 3-2 | In scope (MVP) | First insides of PLTU. 5-octet header, 2 KiB cap. |
| Version-4 / USLP transfer frame in the same PLTU | 732.1-B-3 Fig 4-1 | In scope (after COP-P) | In lieu of V-3, whole stream V-4. Not nested in the V-3 data field. Newer frame; can host COP-P. |
| Space Packet as SDU | 133.0-B-2 Fig 4-1 | In scope (MVP) | 6-octet header + user data. Not a Starcom product name. |
| PLCW 16-bit SPDU field codec | 211.0-B-6 §3.2.4.3.2.1.1 | In scope (MVP codecs) | Pack/unpack only. Not the ARQ. Distinct from CLCW. No generic OCF. |
| CLCW 32-bit field codec | 232.0-B-4 §4.2.1 | In scope (MVP codecs) | Pack/unpack only. Lives in a USLP OCF later; still a pure codec now. |
| COP-P procedures (FOP-P / FARM-P) | 211.0-B-6 §7 | In scope (MVP, after codecs) | Prox ARQ. USLP can host this; it does not replace it. Sequenced immediately after V-3/PLTU can pack a frame. |
| COP-1 procedures (FOP-1 / FARM-1) | 232.1-B-2 | In scope (after COP-P) | The other ARQ. Distinct from COP-P. Not a substitute. |
| Prox-1 session / MAC / hailing | 211.0-B-6 §6 | Not decided | Decide at implementation (full module vs out). No stub now. |
| Convolutional or LDPC coding | 211.2 → 131.0-B-3 | Deferred | 211.2 PICS: at least one of uncoded / conv / LDPC. MVP is uncoded PLTU. |
| Long-haul TM C&S (131.0 ASM / FECF path) | 131.0 | Out of scope for this MVP | Different coding sublayer than PLTU. |
| 211.1-B-4 Physical Layer | 211.1-B-4 | Out of scope | No blanket PHY claim. Adapters may later declare none / best-effort / compliant. |
| JPL User Terminal / Electra interop as a product claim | — | Out of scope | Prox-1 V-3 is the interop *frame*, not a UT claim. |
| Mixed V-3 and V-4 on one PLTU stream | 211.2-B-3 §3.2.4 | Out of scope | Forbidden by the book. |
| F' as a Starcom dependency | — | Out of scope | Integration target only (Grok §10). |

When a row is implemented, add a test pointer. Do not retcon status to Implemented without that pointer.
