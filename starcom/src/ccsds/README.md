# Core implementation (`src/ccsds/`)

Sans-I/O protocol core. No Pico SDK, sockets, SPI, GPIO, or Rocket-Chip includes.

Order: PLTU / Version-3 / Space Packet / PLCW / CLCW pack, then COP-P procedures, then USLP, then COP-1. SAD field maps are working copies — open the cited Blue Book first. See `STATUS.md`.
