# Starcom

Starcom is a CCSDS comms **stack** incubating inside Rocket-Chip until it extracts to its own repository.

The portable piece is the **Starcom library** (`starcom::ccsds`): a sans-I/O data-link core. Rocket-Chip is the first consumer, not the owner.

PHY and claim language live in [`docs/CONFORMANCE.md`](docs/CONFORMANCE.md). There is no blanket 211.1 Physical Layer claim.

Read [`docs/WORKING_HERE.md`](docs/WORKING_HERE.md) first. Map: [`docs/SAD.md`](docs/SAD.md). Handshake: [`docs/ICD.md`](docs/ICD.md). Phase: [`STATUS.md`](STATUS.md). Locks and research freeze: [`docs/DESIGN.md`](docs/DESIGN.md).

```
starcom/
  include/starcom/   public API (empty of real headers until Phase 0)
  src/ccsds/         core implementation
  adapters/          first-party ports (host, radio)
  tests/             host tests, no hardware
  docs/              DESIGN, SAD, ICD, CONFORMANCE, WORKING_HERE
  CMakeLists.txt     scaffold until Phase 0
```

CMake still has no library target. That lands with the first codec, not as a solo sitting.
