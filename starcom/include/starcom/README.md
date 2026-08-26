# Public headers (`include/starcom/`)

Consumers' only search path. Mirrors `starcom::ccsds`. No Rocket-Chip types, no radio objects, no hardware.

Real headers arrive in Phase 0 (`version`, `error`/`expected`, `span`). Codecs after that. See `docs/ICD.md`.
