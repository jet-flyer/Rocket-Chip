# Starcom

Starcom is a CCSDS comms **stack** incubating inside Rocket-Chip until it extracts to its own repository.

**Core library** (`starcom::ccsds`): sans-I/O data-link — caller feeds octets and `tick(now)`, the core returns octets and events. **Ports** (this tree) attach radios, host loopback, later PIO/FPGA. **Rocket-Chip** is the first consumer.

| Term | One line | More |
|------|----------|------|
| **PLTU** | On-air unit: ASM `FAF320` + one transfer frame + CRC-32 | [Glossary](docs/GLOSSARY.md), 211.2 Fig 3-1 |
| **Version-3** | Native Prox-1 frame (5-octet header) inside a PLTU | 211.0 Fig 3-2 |
| **USLP** | Version-4 frame in the same PLTU, in lieu of V-3 | 732.1 |
| **Space Packet** | 6-octet header + user data; usual payload inside the frame | 133.0 Fig 4-1 |
| **COP-P / COP-1** | Reliability machines (Proximity vs Earth TC) | 211.0 §7, 232.1 |

Full acronym list with Blue Book section cites: [`docs/GLOSSARY.md`](docs/GLOSSARY.md).

**Using it:** [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md). Agents: [`docs/WORKING_HERE.md`](docs/WORKING_HERE.md). Map: [`docs/SAD.md`](docs/SAD.md). Handshake: [`docs/ICD.md`](docs/ICD.md). Proof order: [`docs/IVP.md`](docs/IVP.md). Phase: [`STATUS.md`](STATUS.md). Claims: [`docs/CONFORMANCE.md`](docs/CONFORMANCE.md).

```
starcom/
  include/starcom/   public API
  src/ccsds/         core
  adapters/          first-party ports
  tests/             host tests
  docs/              DESIGN, SAD, ICD, IVP, GLOSSARY, …
  CMakeLists.txt     Starcom::starcom lands with the first codec
```

