# What a consumer can do with Starcom

**Audience:** Rocket-Chip first, then any other stack (cubesat, HAB, GCS). Not a wiring sitting — this is the map. Handshake verbs: [`ICD.md`](../ICD.md). Sequence: [`IVP.md`](../IVP.md). Phase: [`STATUS.md`](../../STATUS.md).

Dependency is always **consumer → Starcom**. Starcom never includes consumer headers, AO/QP types, board pins, or radio objects.

Starcom does **not** ship a stop-gap command/retry layer. RC's `telemetry_encoder` is pre-Starcom firmware (council deferred CCSDS past Stage 17). It is replaced at IVP increment 22 by COP, not by another temporary path.

## Now (`0.19.0-dev`, through IVP 22 on `grok/sc-dev`)

Link `Starcom::starcom` (and optionally `Starcom::adapters_host` / `Starcom::adapters_rp2350`). Call with `std::span` and, for COP, caller `now`. The core does not key a transmitter.

| Point | Who can use it | What it does | Not yet (IVP) |
|-------|----------------|--------------|----------------|
| Codecs | Anyone | PLTU, V-3, USLP (incl. truncated/Insert/FECF), Space Packet, PLCW, CLCW; `huntPltu`; conv / LDPC encode | Decode (GCS/Pi) |
| COP-P | Anyone who owns a loop | `copp_*` on V-3 or USLP (`coppInitUslp`) PLTUs | — |
| COP-1 | Anyone who owns a loop | `cop1_*` on USLP+CLCW-in-OCF in a PLTU (S4/S5, Resume, setup, null-LLIF Ready) | TC 232.0 frames |
| Host loopback / UDP / file / SPI-GPIO / PIO / PHY tiers | Tests, desktop sims | Mailboxes, `udp_*`, `BusOps`, `pio_shift_*`, `PhyDecl` | — |
| PLTU repeater | Anyone with bytes in/out | `repeatPltu`; buffered `enqueuePltu` / `dequeuePltu` (caller-owned slots) | — |
| Version | Anyone | `#include "starcom/version.hpp"` | Annotated tag (25) |

**Rocket-Chip specifically:** host CMake `addSubdirectory(starcom)` and links `Starcom::starcom` when `ROCKETCHIP_USE_STARCOM=ON` (20). Pico + `byte_pump` (21). COP-P on the ON air path (22). Default OFF stays STOP-GAP. RadioScheduler / SX1276 stay in RC. No RC types in `include/starcom`.

**Others (cubesat / HAB / GCS):** same library. They bring their own event loop and radio port. No RC types required.

## Rest of the stack (IVP 13–25)

| Increment | Point | Owner |
|-----------|--------|--------|
| 13 | Prox-1 §6 MAC / DUPLEX + SET V(R) | Landed |
| 14 | User-defined DFC 11 | Landed |
| 15 | Host UDP / file replay | Landed |
| 16 | Generic SPI/GPIO radio port | Landed |
| 17 | PIO PLTU symbol pipe | Landed |
| 18 | PHY adapter tiers (uncoded host; FPGA later) | Landed |
| 19 | Convolutional / LDPC encode | Landed |
| 20 | Host `addSubdirectory(starcom)` | Landed (RC host) |
| 21 | Pico link + first AO byte pump | Landed (`grok/sc-dev`) |
| 22 | Replace `telemetry_encoder` with COP | Landed (`grok/sc-dev`) |
| 23 | Initial clang-tidy / camelBack pass (not exhaustive) | Landed (`688ff00`; no Starcom deviation row) |
| 24 | ASan host, longer fuzz, size report | Landed (WSL ASan; sizeof in IVP.md) |
| 25 | First `starcom-v*` tag | Owner cut |

## How to call (shape)

```
bytes in  →  huntPltu / coppReceiveBytes / cop1ReceiveBytes / decode_*
now       →  coppTick / cop1Tick
bytes out →  coppBytesToSend / cop1BytesToSend / encode_* / repeatPltu
events    →  coppPollEvent / cop1PollEvent
```

Sans-I/O: if it is not in that list, the consumer owns it (radio, GPIO, QP, clocks).
