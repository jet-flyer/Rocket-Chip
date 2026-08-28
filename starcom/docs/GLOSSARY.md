# Starcom glossary

Blue Book and stack terms. **The cited section is the definition.** This file is a working index.

Key concepts for a first read also sit in [`../README.md`](../README.md). Agent vocabulary: [`WORKING_HERE.md`](WORKING_HERE.md).

| Term | Meaning | Primary source |
|------|---------|----------------|
| **Starcom** | The comms **stack** (this tree): core + first-party ports + tests + docs. | `DESIGN.md` note 2026-08-21 |
| **Core** | `starcom::ccsds` — sans-I/O library. Bytes and `tick(now)` in; events and bytes out. | same |
| **Port / adapter** | First-party I/O (host loopback, generic radio, later PIO/FPGA). Depends on the core. | same |
| **Integration** | A consumer (Rocket-Chip first). Pins, AO, mission profile. | same |
| **Sans-I/O** | Protocol as synchronous functions over caller buffers. No sockets/SPI/GPIO in the core. | ICD; sans-io pattern |
| **Prox-1** | Proximity-1 Space Link Protocol. Short-range space-to-space / lander–orbiter family. | 211.0, 211.1, 211.2 |
| **C&S** | Coding and Synchronization sublayer. Builds/checks the PLTU. | 211.2 |
| **PLTU** | Proximity Link Transmission Unit: ASM + one transfer frame + CRC-32. | 211.2 §3.2, Fig 3-1 |
| **ASM** | Attached Synchronization Marker. 24 bits, `FAF320`. Marks the start of a PLTU. | 211.2 §3.2.3 |
| **CRC-32** | 32-bit cyclic redundancy check on the **transfer frame** (not the ASM). | 211.2 §3.2.5, **Annex C (normative)** |
| **Annex C** | Normative CRC-32 encoding/decoding procedure. Part of 211.2, not an optional essay. | 211.2 Annex C; CCSDS A20 (normative annexes) |
| **TFVN** | Transfer Frame Version Number. V-3: first two bits `10`. USLP: `1100`. | 211.0 §3.2.2.2; 211.2 §3.6.4 |
| **Version-3 / V-3** | Native Prox-1 transfer frame. 5-octet header, max 2048 octets. | 211.0 §3.2, Fig 3-2 / 3-3 |
| **USLP / Version-4** | Unified Space Data Link Protocol. On Prox-1 it sits *in* a PLTU in lieu of V-3. | 732.1; 211.0 §3.3 |
| **SCID** | Spacecraft Identifier (10 bits on V-3). | 211.0 §3.2.2.6 |
| **PCID** | Physical Channel Identifier (1 bit on V-3). | 211.0 §3.2.2.7 |
| **Port ID** | V-3 output port (3 bits), 0–7. | 211.0 §3.2.2.8 |
| **FSN** | Frame Sequence Number (8 bits). Independent counters per PCID for Sequence Controlled vs Expedited. | 211.0 §3.2.2.11 |
| **QoS** | Quality of Service bit: Sequence Controlled (`0`) vs Expedited (`1`). | 211.0 §3.2.2.3 |
| **U-frame / P-frame** | User-data frame vs supervisory/protocol frame (PDU Type ID). | 211.0 §3.2.2.4 |
| **SPDU** | Supervisory Protocol Data Unit (PLCW and directives). | 211.0 §3.2.4 |
| **PLCW** | Proximity Link Control Word. 16-bit SPDU for COP-P. | 211.0 §3.2.4.3.2, Fig 3-5 |
| **CLCW** | Communications Link Control Word. 32-bit report for COP-1. | 232.0 §4.2.1, Fig 4-6 |
| **OCF** | Operational Control Field (USLP/TM trailer). Can carry a CLCW. Prox-1 V-3 has no OCF. | 732.1 §4.1.5; 211.0 §6.4 discussion in research |
| **FECF** | Frame Error Control Field. USLP optional CRC-16. | 732.1 |
| **Space Packet** | 133.0 PDU: 6-octet primary header + data field. The usual SDU in a transfer frame. | 133.0 §4.1, Fig 4-1 / 4-2 |
| **APID** | Application Process Identifier (11 bits in the Space Packet header). | 133.0 §4.1.3.3.4 |
| **SDU / PDU** | Service Data Unit / Protocol Data Unit (what the user hands in vs what the layer emits). | 211.0 / 133.0 definitions |
| **COP-P** | Communications Operation Procedure — Proximity. | 211.0 §7 |
| **FOP-P / FARM-P** | Frame Operation Procedure / Frame Acceptance and Reporting Mechanism (Proximity). Sender / receiver of COP-P. | 211.0 §7.2 / §7.3 |
| **COP-1** | Communications Operation Procedure-1 (Earth TC reliability). | 232.1 |
| **FOP-1 / FARM-1** | Sender / receiver of COP-1. | 232.1 Tables 5-1 / 6-1 |
| **CFDP** | CCSDS File Delivery Protocol. Post-mission data offload with a checksum over the file. Rides in Space Packet user data. Wanted; not IVP 0–25. | 727.0 |
| **SDLS** | Space Data Link Security. Authenticated/encrypted *frames* (typically telecommand). Not file offload. | 355.0 |
| **MIB** | Management Information Base — long-lived parameters (max frame length, SCIDs, timers). | 211.0 Annex C (normative); 133.0 §5 |
| **DUPLEX** | MAC variable: full, half, or simplex. | 211.0 §6 |
| **Full duplex** | Both directions at once (book PHY: two frequencies). | 211.0 §6; 211.1 |
| **Half duplex** | Both directions, not at the same time. | 211.0 §6 |
| **Simplex** | One direction only. No hailing on that session. | 211.0 §6 |
| **DFC ID** | Data Field Construction Identifier. `11` = user-defined data (opaque octets; not Annex F). | 211.0 Table 3-1 |
| **Repeater** | Regenerative forward of a valid PLTU (same octets out). Starcom/RC capability; not a Blue Book product name. | 211.2 C&S check; 133.0 §2.4 (subnetwork store-and-forward) |
| **T / A / R / I** | Verification methods: Test, Analysis, Review-of-design, Inspection. | ECSS-E-ST-10-02; this IVP |
