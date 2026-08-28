# Core implementation (`src/ccsds/`)

Sans-I/O protocol core. No Pico SDK, sockets, SPI, GPIO, or Rocket-Chip includes.

| File | Job |
|------|-----|
| `crc32.cpp` | 211.2 Annex C CRC-32 |
| `pltu.cpp` | PLTU envelope, `repeat_pltu`, `hunt_pltu` (211.2 §3.6) |
| `v3.cpp` | Version-3 Transfer Frame (5-octet header) |
| `space_packet.cpp` | Space Packet primary header (133.0) |
| `plcw.cpp` | 16-bit PLCW SPDU pack/unpack |
| `clcw.cpp` | 32-bit CLCW pack/unpack |
| `copp.cpp` | FOP-P / FARM-P (211.0 §7) |
| `uslp.cpp` | Version-4 transfer frame (732.1 §4.1) |
| `cop1.cpp` | FOP-1 / FARM-1 (232.1 Tables 5-1 / 6-1) |

Next: IVP 11 (COP-P on USLP VC). SAD field maps are working copies — open the cited Blue Book first. See `STATUS.md`.
