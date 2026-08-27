# Core implementation (`src/ccsds/`)

Sans-I/O protocol core. No Pico SDK, sockets, SPI, GPIO, or Rocket-Chip includes.

| File | Job |
|------|-----|
| `crc32.cpp` | 211.2 Annex C CRC-32 |
| `pltu.cpp` | PLTU envelope (ASM + frame + CRC-32) |
| `v3.cpp` | Version-3 Transfer Frame (5-octet header) |
| `sp.cpp` | Space Packet primary header (133.0) |

Next in increment 0+1: PLCW, CLCW pack. SAD field maps are working copies — open the cited Blue Book first. See `STATUS.md`.
