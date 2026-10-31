# Public headers (`include/starcom/`)

Consumers' only search path. Core is `starcom::ccsds` (no Rocket-Chip types, no hardware). Ports are `starcom::adapters` (byte mailboxes; still no Pico SDK).

| Header | Job |
|--------|-----|
| `version.h` | Generated. `starcom::kVersionString` / git identity. Template `version.h.in`. |
| `error.h` | Closed `enum class Error` |
| `result.h` | `Result<T>` = `tl::expected<T, Error>` |
| `ccsds/crc.h` | Annex C `crc32` |
| `ccsds/pltu.h` | `decodePltu` / `encodePltu` / `repeatPltu` / `huntPltu` / `enqueuePltu` |
| `ccsds/types.h` | `Scid`, `Pcid`, `PortId`, `Apid`, `UslpScid`, `Vcid`, `MapId`; frame min/max |
| `ccsds/v3.h` | `decodeV3` / `encodeV3` / `encodeV3UserDefined` |
| `ccsds/space_packet.h` | `decodeSpacePacket` / `encodeSpacePacket` |
| `ccsds/plcw.h` | `Plcw16` pack/unpack |
| `ccsds/clcw.h` | `Clcw32` pack/unpack |
| `ccsds/copp.h` | FOP-P / FARM-P + `CoppEndpoint` / `coppSubmitUserDefined` |
| `ccsds/uslp.h` | Version-4 `decodeUslp` / `encodeUslp` |
| `ccsds/cop1.h` | FOP-1 / FARM-1 + `Cop1Endpoint` |
| `adapters/loopback.h` | `HostLoopback` two `FrameSlot`s |
| `adapters/radio_port.h` | One TX + one RX mailbox |
| `adapters/file_replay.h` | Host file hunt into a PLTU sink |
| `adapters/udp.h` | Host UDP bearer (opaque handle; caller host/port) |
| `adapters/radio_bus.h` | Generic SPI/GPIO `BusOps` + RadioPort byte pump |
| `adapters/pio_port.h` | PIO-shaped PLTU bit pipe (MSB first; not 211.1) |
| `ccsds/conv.h` | Rate 1/2 K=7 conv encode (`convEncode` / `ConvEnc`) |
| `ccsds/ldpc.h` | LDPC (2048,1024) encode, CSM, 211.2 randomizer |
| `adapters/phy.h` | PHY tiers (`none` / `best_effort` / `compliant`); uncoded PLTU |

Handshake: `docs/ICD.md`. Clause walk: `docs/COVERAGE.md`.
