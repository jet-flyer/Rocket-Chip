# Public headers (`include/starcom/`)

Consumers' only search path. Core is `starcom::ccsds` (no Rocket-Chip types, no hardware). Ports are `starcom::adapters` (byte mailboxes; still no Pico SDK).

| Header | Job |
|--------|-----|
| `version.hpp` | Generated. `starcom::kVersionString` / git identity. Template `version.hpp.in`. |
| `error.hpp` | Closed `enum class Error` |
| `result.hpp` | `Result<T>` = `tl::expected<T, Error>` |
| `ccsds/crc.hpp` | Annex C `crc32` |
| `ccsds/pltu.hpp` | `decode_pltu` / `encode_pltu` / `repeat_pltu` / `hunt_pltu` / `enqueue_pltu` |
| `ccsds/types.hpp` | `Scid`, `Pcid`, `PortId`, `Apid`, `UslpScid`, `Vcid`, `MapId`; frame min/max |
| `ccsds/v3.hpp` | `decode_v3` / `encode_v3` |
| `ccsds/space_packet.hpp` | `decode_space_packet` / `encode_space_packet` |
| `ccsds/plcw.hpp` | `Plcw16` pack/unpack |
| `ccsds/clcw.hpp` | `Clcw32` pack/unpack |
| `ccsds/copp.hpp` | FOP-P / FARM-P + `CoppEndpoint` |
| `ccsds/uslp.hpp` | Version-4 `decode_uslp` / `encode_uslp` |
| `ccsds/cop1.hpp` | FOP-1 / FARM-1 + `Cop1Endpoint` |
| `adapters/loopback.hpp` | `HostLoopback` two `FrameSlot`s |
| `adapters/radio_port.hpp` | One TX + one RX mailbox |

Handshake: `docs/ICD.md`.
