# Unit tests

Procedure: [`../../docs/TESTING.md`](../../docs/TESTING.md).

| File | Job |
|------|-----|
| `test_pltu.cpp` | Annex C CRC-32 + PLTU; IVP `v3-header-only` and PLTU rejects |
| `test_v3.cpp` | Version-3 pack/unpack; composition with PLTU |
| `test_space_packet.cpp` | Space Packet; IVP `sp-idle`, `v3-one-sp-n` (18+N) |
| `test_ocf.cpp` | `Plcw16` / `Clcw32`; IVP `plcw-zero-report`, `clcw-cop1` |
| `heap_trap.cpp` | D-5 malloc/`operator new` trap (positive control + codecs allocate nothing) |
| `test_user_defined.cpp` | IVP 14 DFC `11` user-defined + simplex without hailing |
| `test_host_io.cpp` | IVP 15 host file replay + UDP |
| `test_radio_bus.cpp` | IVP 16 generic SPI/GPIO port on a fake bus |
| `test_pio_port.cpp` | IVP 17 PIO bit pipe; 0+1 PLTU octets |
| `test_phy.cpp` | IVP 18 PHY tiers; uncoded 0+1 PLTU; compliant not offered |
