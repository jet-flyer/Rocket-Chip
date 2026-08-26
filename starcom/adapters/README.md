# Adapters (`adapters/`)

First-party ports. They depend on the core; the core never depends on them.

- `host/` — desktop transports (UDP, file replay, loopback)
- `rp2350/` — generic SPI/GPIO radio port (pins and AO stay in Rocket-Chip)

Hardware-specific drivers live only here. PHY honesty: `docs/CONFORMANCE.md`.
