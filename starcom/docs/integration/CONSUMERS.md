# What a consumer can do with Starcom

**Audience:** Rocket-Chip first, then any other stack (cubesat, HAB, GCS). Not a wiring sitting — this is the map. Handshake verbs: [`ICD.md`](../ICD.md). Sequence: [`IVP.md`](../IVP.md). Phase: [`STATUS.md`](../../STATUS.md).

Dependency is always **consumer → Starcom**. Starcom never includes consumer headers, AO/QP types, board pins, or radio objects.

Starcom does **not** ship a stop-gap command/retry layer. RC's `telemetry_encoder` is pre-Starcom firmware (council deferred CCSDS past Stage 17). It is replaced at IVP increment 22 by COP, not by another temporary path.

## Now (`0.10.0-dev`, through IVP 10)

Link `Starcom::starcom` (and optionally `Starcom::adapters_host`). Call with `std::span` and, for COP, caller `now`. The core does not key a transmitter.

| Point | Who can use it | What it does | Not yet (IVP) |
|-------|----------------|--------------|----------------|
| Codecs | Anyone | PLTU, V-3, USLP (incl. truncated/Insert/FECF), Space Packet, PLCW, CLCW; `hunt_pltu` | — |
| COP-P | Anyone who owns a loop | `copp_*` on Version-3 PLTUs | SET V(R) persistent / §6 MAC (13); COP-P on USLP VC (11) |
| COP-1 | Anyone who owns a loop | `cop1_*` on USLP+CLCW-in-OCF in a PLTU (incl. S4/S5 BC-init) | Suspend/resume E30–E34; LLIF E41–E46 |
| Host loopback / `RadioPort` | Tests, desktop sims | One-PLTU mailboxes | UDP/file (15); SPI/GPIO (16) |
| PLTU repeater | Anyone with bytes in/out | `repeat_pltu`: envelope check, same octets out | Buffered queue / FSN dedup (12) |
| Version | Anyone | `#include "starcom/version.hpp"` | Annotated tag (25) |

**Rocket-Chip specifically:** an AO *may* call those verbs. RC CMake does **not** `add_subdirectory(starcom)` until increment 20. Pre-Starcom `telemetry_encoder` remains the flight path until increment 22. RadioScheduler / SX1276 stay in RC.

**Others (cubesat / HAB / GCS):** same library. They bring their own event loop and radio port. No RC types required.

## Rest of the stack (IVP 11–25)

| Increment | Point | Owner |
|-----------|--------|--------|
| 11 | COP-P on USLP VC | Starcom core |
| 12 | Buffered repeater / dedup | Starcom core (caller-owned queue; no invented depth) |
| 13 | Prox-1 §6 MAC / DUPLEX + SET V(R) | Owner decision, then Starcom (no stub of the unchosen cut) |
| 14 | Simplex / user-defined bitstream | Starcom core |
| 15–18 | UDP/file, SPI/GPIO, PIO, PHY/FPGA tiers | `adapters/` |
| 19 | Convolutional / LDPC | Starcom (211.2 PICS; not a 131.0 long-haul product) |
| 20 | Host `add_subdirectory(starcom)` | RC |
| 21 | Pico link + first AO byte pump | RC |
| 22 | Replace `telemetry_encoder` with COP | RC (real replacement) |
| 23 | clang-tidy / camelBack vs ICD snake_case | Starcom audit |
| 24 | ASan host, longer fuzz, size report | Starcom |
| 25 | First `starcom-v*` tag | Owner cut |

## How to call (shape)

```
bytes in  →  hunt_pltu / copp_receive_bytes / cop1_receive_bytes / decode_*
now       →  copp_tick / cop1_tick
bytes out →  copp_bytes_to_send / cop1_bytes_to_send / encode_* / repeat_pltu
events    →  copp_poll_event / cop1_poll_event
```

Sans-I/O: if it is not in that list, the consumer owns it (radio, GPIO, QP, clocks).
