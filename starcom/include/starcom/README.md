# Public headers (`include/starcom/`)

Consumers' only search path. Namespace `starcom::ccsds`. No Rocket-Chip types, no radio objects, no hardware.

| Header | Job |
|--------|-----|
| `error.hpp` | Closed `enum class Error` |
| `result.hpp` | `Result<T>` = `tl::expected<T, Error>` |
| `ccsds/crc.hpp` | Annex C `crc32` |
| `ccsds/pltu.hpp` | `decode_pltu` / `encode_pltu` |

Handshake: `docs/ICD.md`.
