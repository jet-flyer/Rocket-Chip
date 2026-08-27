# Public headers (`include/starcom/`)

Consumers' only search path. Namespace `starcom::ccsds`. No Rocket-Chip types, no radio objects, no hardware.

| Header | Job |
|--------|-----|
| `error.hpp` | Closed `enum class Error` |
| `result.hpp` | `Result<T>` = `tl::expected<T, Error>` |
| `ccsds/crc.hpp` | Annex C `crc32` |
| `ccsds/pltu.hpp` | `decode_pltu` / `encode_pltu` |
| `ccsds/types.hpp` | `Scid`, `Pcid`, `PortId`; frame min/max |
| `ccsds/v3.hpp` | `decode_v3` / `encode_v3` |
| `ccsds/sp.hpp` | `decode_sp` / `encode_sp` |

Handshake: `docs/ICD.md`.
