# Starcom conformance claims

**Status:** Draft. Claims, not implementation rows. No Implemented / stub column until there is code. No JPL User Terminal table.

Honesty: a claim is in-scope, deferred, or out of scope. "Best effort" is not a claim. PHY tiers belong here, not in the README.

| Claim | Book | Status | Notes |
|-------|------|--------|--------|
| PLTU: ASM `FAF320` + transfer frame + CRC-32, uncoded | 211.2-B-3 Fig 3-1 | In scope (MVP) | Envelope. One frame version per stream. |
| Version-3 transfer frame | 211.0-B-6 Fig 3-2 | In scope (MVP) | First insides of PLTU. 5-octet header, 2 KiB cap. |
| Version-4 / USLP transfer frame in the same PLTU | 732.1-B-3 Fig 4-1 | In scope (MVP) | In lieu of V-3, whole stream V-4. Not nested in the V-3 data field. |
| Space Packet as SDU | 133.0-B-2 Fig 4-1 | In scope (MVP) | 6-octet header + user data. Not a Starcom product name. |
| PLCW 16-bit SPDU field codec | 211.0-B-6 §3.2.4.3.2.1.1 | In scope (MVP codecs) | Pack/unpack only. Not the ARQ. Distinct from CLCW. No generic OCF. |
| CLCW 32-bit field codec | 232.0-B-4 §4.2.1 | In scope (MVP codecs) | Pack/unpack only. Lives in a USLP OCF later; still a pure codec now. |
| COP-P procedures (FOP-P / FARM-P) | 211.0-B-6 §7 | Deferred | State machines, timers, window. USLP can *host* this; it does not replace it. Claude phase 4. |
| COP-1 procedures (FOP-1 / FARM-1) | 232.1-B-2 | Deferred | State machines. Distinct from COP-P. Claude phase 3. |
| Prox-1 session / MAC / hailing | 211.0-B-6 §6 | Deferred or absent | Full module later, or not at all. No stub. |
| Convolutional or LDPC coding | 211.2 → 131.0-B-3 | Deferred | 211.2 PICS: at least one of uncoded / conv / LDPC. MVP is uncoded PLTU. |
| Long-haul TM C&S (131.0 ASM / FECF path) | 131.0 | Out of scope for this MVP | Different coding sublayer than PLTU. |
| 211.1-B-4 Physical Layer | 211.1-B-4 | Out of scope | No blanket PHY claim. Adapters may later declare none / best-effort / compliant. |
| JPL User Terminal / Electra interop as a product claim | — | Out of scope | Prox-1 V-3 is the interop *frame*, not a UT claim. |
| Mixed V-3 and V-4 on one PLTU stream | 211.2-B-3 §3.2.4 | Out of scope | Forbidden by the book. |
| F' as a Starcom dependency | — | Out of scope | Integration target only (Grok §10). |

When a row is implemented, add a test pointer. Do not retcon status to Implemented without that pointer.
