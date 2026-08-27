# Core implementation (`src/ccsds/`)

Sans-I/O protocol core. No Pico SDK, sockets, SPI, GPIO, or Rocket-Chip includes.

| File | Job |
|------|-----|
| `crc32.cpp` | 211.2 Annex C CRC-32 |
| `pltu.cpp` | PLTU envelope (ASM + frame + CRC-32) |
| `v3.cpp` | Version-3 Transfer Frame (5-octet header) |
| `sp.cpp` | Space Packet primary header (133.0) |
| `plcw.cpp` | 16-bit PLCW SPDU pack/unpack |
| `clcw.cpp` | 32-bit CLCW pack/unpack |

Increment 0+1 codecs are in. Next: COP-P. SAD field maps are working copies — open the cited Blue Book first. See `STATUS.md`.
