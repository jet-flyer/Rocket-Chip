# What a consumer can do with Starcom

**Audience:** Rocket-Chip first, then any other stack (cubesat, HAB, GCS). Not a wiring sitting — this is the map. Handshake verbs: [`ICD.md`](../ICD.md). Phase: [`STATUS.md`](../../STATUS.md).

Dependency is always **consumer → Starcom**. Starcom never includes consumer headers, AO/QP types, board pins, or radio objects.

## Now (`0.6.0-dev`)

Link `Starcom::starcom` (and optionally `Starcom::adapters_host`). Call with `std::span` and, for COP, caller `now`. The core does not key a transmitter.

| Point | Who can use it | What it does | Not |
|-------|----------------|--------------|-----|
| Codecs | Anyone | PLTU, V-3, USLP, Space Packet, PLCW, CLCW pack/unpack | Stream ASM hunt; truncated USLP; FECF |
| COP-P | Anyone who owns a loop | `copp_*` on Version-3 PLTUs | SET V(R) persistent / §6 MAC |
| COP-1 | Anyone who owns a loop | `cop1_*` on USLP+CLCW-in-OCF in a PLTU | FOP-1 S4/S5 BC init; TC 232.0 frames |
| Host loopback / `RadioPort` | Tests, desktop sims | One-PLTU mailboxes | Sockets, SPI, Pico SDK |
| PLTU repeater | Anyone with bytes in/out | `repeat_pltu`: envelope check, same octets out | Buffered queue, FSN dedup, COP on the repeat path |
| Version | Anyone | `#include "starcom/version.hpp"` | A tagged release (still `-dev`) |

**Rocket-Chip specifically:** an AO *may* call those verbs. RC CMake does **not** `add_subdirectory(starcom)` on this branch (host or Pico). STOP-GAP `telemetry_encoder` is still the flight path. RadioScheduler / SX1276 stay in RC.

**Others (cubesat / HAB / GCS):** same library. They bring their own event loop and radio port. No RC types required.

## Later (not this tree’s numbered IVP)

| Point | Owner | Note |
|-------|--------|------|
| Pico `target_link_libraries(rocketchip Starcom::starcom)` | RC sitting | Flash size; first AO byte pump |
| Replace STOP-GAP retry/ACK with COP | RC | Consumer work, not a codec |
| Half-duplex turnaround | RC scheduler **or** later Starcom MAC — **not decided** | Do not lock the core to LoRa |
| Prox-1 §6 hailing/session | Whiteboard | Full module vs out |
| Buffered repeater / dedup | Whiteboard | Bent-pipe `repeat_pltu` is in; queue depth not invented |
| SPI/GPIO / PIO / FPGA ports | `adapters/` or RC | Same codec vectors |
| clang-tidy / camelBack vs ICD snake_case | Whiteboard | Standards apply; rename is a later audit |

## How to call (shape)

```
bytes in  →  copp_receive_bytes / cop1_receive_bytes / decode_*
now       →  copp_tick / cop1_tick
bytes out →  copp_bytes_to_send / cop1_bytes_to_send / encode_*
events    →  copp_poll_event / cop1_poll_event
```

Sans-I/O: if it is not in that list, the consumer owns it (radio, GPIO, QP, clocks).
