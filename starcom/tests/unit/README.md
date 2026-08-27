# Unit tests

Procedure: [`../../docs/TESTING.md`](../../docs/TESTING.md).

| File | Job |
|------|-----|
| `test_pltu.cpp` | Annex C CRC-32 + PLTU; IVP `v3-header-only` and PLTU rejects |
| `test_v3.cpp` | Version-3 pack/unpack; composition with PLTU |
| `test_sp.cpp` | Space Packet; IVP `sp-idle`, `v3-one-sp-n` (18+N) |
| `test_ocf.cpp` | `Plcw16` / `Clcw32`; IVP `plcw-zero-report`, `clcw-cop1` |
| `heap_trap.cpp` | D-5 malloc/`operator new` trap (positive control + codecs allocate nothing) |
