# FPGA and PIO

Hub for when to use RP2350 PIO vs a small FPGA, which boards exist in Nathan's Feather stack, and what the FPGA is not. Starcom PHY claims stay in the Starcom docs.

PIO first. FPGA second. Never for ESKF / fusion math.

---

## Role split

| Layer | Owns |
|-------|------|
| **MCU (M33)** | ESKF / fusion, flight director, health *policy*, log schema, radio protocol, multi-IMU voting (vector subtract) |
| **PIO** | Pin-timed small FSMs: WS2812, WDT kick, extra UART / DShot, FSK DCLK, stamping GPIO edges, bit-piping PCM |
| **FPGA** | Work that must not share the flight core, or that needs parallel MACs: inhibit / voting (dumb monitor), dual-clock mailbox, tiny FIR / correlator, RF bit-clock if PIO is exhausted |

Treat PIO as a tiny pin CPLD: 12 state machines, 32 instructions, no multipliers - a few hundred LUTs of GPIO FSMs. It is not a T8.

### Definitely not (T8 / UP5K / PIO)

- ESKF / fusion float matrices
- Video (needs SERDES + frame RAM)
- Viterbi / LDPC decode (too large for T8)
- GNSS baseband
- Filesystems
- Analog front ends - UVLO and overcurrent stay analog comparators so the voter still works if the 3.3 V rail dies

---

## FPGA Horizons intended use

Forgix exists because PIO almost makes a small FPGA redundant. The FPGA is still wanted for what PIO cannot do: true parallel fabric, MACs, and a dissimilar monitor. That is PIO-first, not FPGA-first.

They picked the T8F49 (3x3 mm, 49-ball) so the FPGA fits next to an RP2354 in a Teensy / stamp outline. A fatter-RAM part (iCE40UP5K-SG48 is ~8x8 mm) would not. Package size is the honest reason the T8 has little block RAM, not a mistake.

A Forgix T8 bought only so Starcom has an FPGA job is a red herring. T8 is justified by project-wide uses plus lockstep encode / correlator the Pico cannot keep - not so the FPGA has a job.

The same size argument is why T8 is a candidate for the Core module / stamp: it can sit on the stamp without becoming the board. That is gated. Put it on the stamp only if a job PIO cannot do (dissimilar inhibit, MAC FIR / correlator) proves worth the fabric, Efinity, and power. Not for Starcom comms. Forgix on the bench is the eval vehicle.

---

## Chips

LUT vs LE is roughly 1:1 for counting. T8 has ~1.4x more cells than UP5K if those cells stay logic. T8 LEs are also used as routing, so congested designs lose logic. MCU / PSRAM cannot replace on-FPGA block RAM.

### Efinix Trion T8

| | |
|--|--|
| Package | T8F49, 3x3 mm 49-ball (the stamp-fit reason) |
| Logic | 7384 LEs (4-LUT+FF-ish) |
| DSP | 8 x 18x18 MACs |
| RAM | ~123 kbit (24 blocks) |
| Tools | Efinity (closed) |

### Lattice iCE40UP5K

| | |
|--|--|
| Logic | 5280 4-LUTs |
| DSP | 8 x MAC16 (accumulate, add/sub, cascade, split 8x8) |
| RAM | 1 Mbit SPRAM + 120 kbit DPRAM |
| Tools | IceStorm / Yosys (open) |

Fabric RAM is the real split vs T8.

### Lattice iCE5LP4K

3520 4-LUTs, few MAC16s, 80 kbit RAM. Used on Oak RPGA Feather and the Lattice FeatherWing.

---

## Boards / form factor

Nathan's Feather stack plus FPGA eval boards in play this sitting. No invented pin maps or SKUs beyond what is listed here.

The table splits **mechanical form** (hole pattern / board outline) from **electrical pin functions** (what those holes actually carry). Do not mix them.

Mechanical systems: Teensy 4.0 0.600" 2x14 hole pattern; Feather / FeatherWing; Pico-width outline (pico2-ice is **not** Pico pinout); Digilent PMOD 2x6 0.1" dual-row (0.1" **between** rows); UPduino custom dual-inline 24-pin (two single 0.1" rows, one per long edge).

Electrical: Forgix is RP2354+T8 on Teensy holes, **not** Teensy 4.0 pin functions. pico2-ice is Digilent PMOD 2x6, **not** Pico / Cowbell. Oak is true Feather pin functions on the carrier. RPGA is Feather mechanical+electrical (skip: RP2040). Galaxia edge plugs are real Digilent PMOD 2x6. UPduino electrical is UP5K GPIO on DIP holes; "PMOD compatible" is pin-order on a 1x6, not a 2x6. Pmod detail: `docs/hardware/FPGA/standards/PMOD.md`.

### Master comparison

| Board | FPGA (+ MCU) | Mechanical form | Electrical pin functions | RAM/MAC | Verdict for Rocket Chip |
|-------|--------------|-----------------|--------------------------|---------|-------------------------|
| **Forgix** | RP2354 + Efinix T8F49 (3x3 mm) | Teensy 4.0 0.600" 2x14 hole pattern / board outline. Adafruit 3200 mates the holes. Not Feather. | **Not** Teensy 4.0 pin functions. Header 11/12/13 and 18/19 are T8-only (G5/G2/F5, B7/A7); no copper to the RP, so Teensy 4 SPI0/I2C0 are not on those holes. GPS can live on RP UART1; LoRa SPI is FPGA-side. A 3200 therefore dumps Feather SPI/I2C onto the FPGA. Debug: Tag-Connect TC2030, not Teensy USB/debug. | T8 ~123 kbit, 8x 18x18 MACs (little BRAM; package-size reason) | $50. On desk (unsoldered). Eval vehicle for T8-on-stamp. T8 on core stamp is **gated**: only if inhibit/MAC work PIO cannot do proves worth fabric/Efinity/power. Not for Starcom comms. T8 not on the base RC board. |
| **pico2-ice** | RP2350B + iCE40 UP5K | Pico-**width** outline. Not a Pico-header / Cowbell drop-in. | Digilent PMOD 2x6 0.1" dual-row electrical (ICE A, ICE B, RP-ICE, RP) — **not** Pico pinout. Buy soldered vs not (4 headers; no 90 vs vertical SKU); docs solder example is female 2x6 right-angle. J5 is 3-pin JST-SH BM03B-SRSS-TB Raspberry Pi Debug Probe SWD, **not** Qwiic. | UP5K 1 Mbit SPRAM + 120 kbit DPRAM, 8x MAC16 | $50. Not a Pico-header drop-in. |
| **RPGA Feather** | RP2040 + iCE5LP4K | Feather | Feather pin functions (true Feather). | iCE5LP4K: 3520 LUTs, few MAC16s, 80 kbit | **SKIP** (RP2040; no RP2350 SKU). |
| **Oak Lattice FeatherWing** | iCE5LP4K (no MCU on the wing) | FeatherWing | True Feather pin functions on the carrier (mechanical **and** electrical). | Same iCE5LP4K | Most practical FPGA add-on on the current Feather stack. |
| **Galaxia** | Certus-NX LFD2NX-40 + Pico | Bench / space-dev board (not Teensy / Pico / Feather / rocket FC). | Edge 2x6 90 deg plugs **are** real Digilent PMOD 2x6 (80 I/O = ten ports: 8 I/O + 3.3 V + GND each). | 40k LUTs, 2.5 Mbit BRAM, 56 DSP 18x18 | ~$988 bench/space-dev. Not a rocket FC. |
| **UPduino v3.1** | iCE40 UP5K only (FTDI programmer; no RP) | Custom DIP stick ~2.2 cm x 6.2 cm. Two **single** 0.1" rows, one on each long edge (~Pico-like **row spacing**, not 0.1" dual-row). Not Teensy/Pico/Feather/PMOD 2x6. Tindie $36, quantity only; bag of two unsoldered 24-pin 0.1" headers. **No PMOD SKU on Tindie.** | UP5K GPIO on those DIP holes. "PMOD compatible" is pin **order** on a 1x6 (27/26/25/23/GND/3V3), not Teensy/Pico/Feather/Digilent 2x6. | UP5K 1 Mbit SPRAM + 120 kbit DPRAM, 8x MAC16 | FPGA-only eval, not Feather/Pico/Teensy drop-in. |

### Forgix - RP2354 + T8

**Mechanical:** Teensy 4.0 0.600" 2x14 hole pattern / outline. ~$50. Not a Feather. On desk (unsoldered). Adafruit 3200 mates the holes.

**Electrical:** not Teensy 4.0 pin functions.

- Header pins 11/12/13 and 18/19 are T8-only (T8 G5/G2/F5 and B7/A7). No copper to the RP. Teensy 4 SPI0/I2C0 are therefore not on those holes. GPS can live on RP UART1; LoRa SPI is FPGA-side (safe: not an RP radio). A 3200 dumps Feather SPI/I2C onto the FPGA, not a Teensy-standard SPI/I2C.
- RP pins on the Teensy header: 0/1 UART1 (GPIO8/9), 2/3 GPIO22/23, 7/8 UART0 (GPIO12/13). Everything else digital 4-6 and 9-23 is T8 fabric.
- RP-T8 chip link is passive config SPI: CS/SCK/MOSI only (RP GPIO1/2/3 to T8 G3/F3/F2). MISO is unconnected. After DONE, those three wires can be SPIBone (3-wire) or UART on CLK/MOSI. Not a 4-wire MCU SPI you can mux header pins onto.
- Practical: T8 owns SPI on 11/12/13 and I2C on 18/19 as a real bus; mailbox to RP over the 3-wire link. Not practical: bitstream-mux those header pins onto the RP hardware SPI/I2C (no copper). Do not treat T8 as a Teensy-SPI compatibility shim.
- Debug: Tag-Connect TC2030, not Teensy USB/debug.

T8 is not on the base Rocket-Chip board for comms. Forgix is the eval vehicle for T8-on-stamp. T8 on the core stamp is gated: only if inhibit / MAC work PIO cannot do proves worth fabric, Efinity, and power.

### pico2-ice - RP2350B + iCE40UP5K

- **Mechanical:** Pico-width outline. ~$50.
- **Electrical:** all expansion pins are Digilent PMOD 2x6 (ICE A, ICE B, RP-ICE, RP) — not Pico 0.1" header pinout, not Adafruit Pi Cowbell. PMOD is a separate expansion system (`docs/hardware/FPGA/standards/PMOD.md`).
- Buy option is soldered vs not (4 headers; no 90 vs vertical SKU). The documented solder example is female 2x6 right-angle on the long edges.
- J5 is **not** Qwiic / STEMMA QT. It is a 3-pin JST-SH SWD debug header (BM03B-SRSS-TB: SWD, SWCLK, GND). STEMMA sensors do not plug in; use the RP I2C pins on the headers if you need a stopgap.

### RPGA Feather and Oak Lattice FeatherWing

- **RPGA Feather** (RP2040 + iCE5LP4K): Feather mechanical **and** electrical (true Feather pin functions). No RP2350 SKU. Skip.
- **Oak Lattice FeatherWing** (iCE5LP4K): FeatherWing mechanical **and** electrical — true Feather pin functions on the carrier. Most practical path to a small FPGA on the current Feather stack if not buying Forgix as the FC.

Shrike / FireAnt are also-rans (ForgeFPGA / T8-only). No pinout verified this sitting.

### Galaxia Space Development Board (FPGA Horizons / Adiuvo)

Different class from T8 / iCE40. Ground / bench / space-dev, not a rocket FC.

- Lattice Certus-NX LFD2NX-40 (40k LUTs, 2.5 Mbit BRAM, 56 DSP 18x18)
- Commercial equivalent of Certus-NX-RT (CAES)
- ~$988
- **Electrical:** 80 I/O via PMOD = ten real Digilent 2x6 right-angle PMODs along the edge (8 I/O + 3.3 V + GND each)
- Also has an RPi Pico

### UPduino v3.1 (tinyVision.ai)

FPGA-only eval (UP5K + FTDI programmer). **Mechanical:** custom DIP; not a Feather, Pico, Teensy, or Digilent 2x6 drop-in.

Tindie listing (verified this sitting): **no PMOD SKU**. Quantity only, $36. The listing includes a bag of two unsoldered 24-pin 0.1" headers — that is the whole header story on Tindie.

The board is a custom DIP stick ~2.2 cm x 6.2 cm with **two single 0.1" rows**, one on each long edge. Row spacing is ~Pico-like (the two rows face each other across the stick). It is **not** a 0.1" dual-row connector. A Digilent PMOD is 2x6 with 0.1" **between** rows. There is no hidden 2x6 PMOD footprint on this board.

**Electrical:** UP5K GPIO on those DIP holes. "PMOD compatible" means the **pin order** on a 1x6 of those holes (27, 26, 25, 23, GND, 3V3), so you can jumper to a PMOD accessory — not Teensy/Pico/Feather/PMOD 2x6. tinyVision's own shop (not Tindie) will solder a 6-pin PMOD for a fee. That is a shop service, not a Tindie option.

---

## Redundancy

Dissimilar (FPGA watching MCU) catches design bugs. Identical copies catch random faults. The in-real-life pattern is stacked identical copies plus a dissimilar monitor.

T8 = dumb discrete voter (heartbeat, disagree, cutoff), not a third flight computer and not a fusion brain. Multi-IMU voting stays on the M33.

Gemini + Titan could be dissimilar triplicate if someone truly wants that. Gemini pyro voting today is discrete AND/OR gates, not fabric - see `docs/hardware/GEMINI_CARRIER_BOARD.md`.

UVLO / overcurrent stay analog, outside the fabric, so the voter still works if the 3.3 V rail dies.

---

## CNN / anomaly (optional Feather experiment)

Not a flight-computer default. Optional FeatherWing / Forgix experiment for PID tuning / anomaly handling.

Project: https://github.com/kazunori279/fpga-open-vocab (`docs/fit.md` is the honesty screen).

SigLIP 2 distilled to a 1.40M int4 CNN on Forgix T8:

| Resource | Used / total | Note |
|----------|--------------|------|
| LE | 6,265 / 7,384 | 85% |
| Multipliers | 8 / 8 | none left |
| RAM | 21 / 24 | 3 blocks left |

282 ms/frame camera encoder, 5.97x vs MCU-only. Decision rule is an enrolled one-scene discriminator, not a general detector (held-out pairs 95.8 / 90.8 / 50 / 34%).

85% of logic is acceptable if vital functions don't need much *and* the net provides a notable advantage. Leftover ~1.1k LEs can do a tiny voter. Leftover 0 MACs and 3 RAM blocks cannot do FIR / mailbox / correlator at the same time as this CNN. Reconfigure (swap bitstream) is how those share the chip.

A PID / IMU anomaly net could be smaller than this vision CNN, but still consumes MACs. Do not treat vision CNN as a flight-computer default.

---

## Starcom

T8 is not on the base RC board for comms. FSK / bitstream is the RFM95 lab path.

Max on existing RFM95: RP 211.0 plus T8 211.2 encode and FSK continuous bitstream as the lab path. Bit-bang / FSK is not 211.1. Full 211.1 PHY is not offered. Hamilton: no PHY approximations.

Living claims and order live in the Starcom docs, not here:

- `starcom/docs/CONFORMANCE.md` - PICS levels; PIO bit pipe and FSK are Best effort, not 211.1
- `starcom/docs/DESIGN.md` - Forgix T8 C&S fit; T8 not on the base board
- `starcom/docs/WORKING_HERE.md` - don't put Forgix T8 on the base board for comms
- `starcom/docs/IVP.md` - increment 17 PIO pipe, 18 PHY tiers; FSK path not a new IVP number yet
- `starcom/docs/SAD.md` - PIO vs FPGA adapter roles
- `starcom/STATUS.md` - FSK / T8 future path; `compliant` / FPGA HDL not offered this sitting

---

## See also

Paths from repo root. One-line why; do not treat this hub as a copy of those docs.

### Hardware and voting

- `docs/hardware/HARDWARE.md` - Gemini carrier, madflight FC3v2 reference, Tindie FPGA sourcing, PIO NeoPixel / PWM notes
- `docs/hardware/GEMINI_CARRIER_BOARD.md` - discrete AND/OR pyro voting (not FPGA)
- `docs/icd/GEMINI_PROTOCOL_ICD.md` - hardware voting still works if both MCUs hang
- `docs/decisions/TITAN_BOARD_ANALYSIS.md` - Gemini voting is essential; PIO servo hard limits

### PIO safety

- `docs/decisions/WATCHDOG_SAFETY_SOURCES.md` - Ingenuity FPGA watchdog as a gate; PIO clk_sys / WDSEL cites
- `docs/decisions/WATCHDOG_SAFETY_ARCHITECTURE.md` - three-layer safety; PIO timers survive degrade
- `docs/PIO/PIO_WATCHDOG.md` - PIO heartbeat / lockout design (tiny pin CPLD)
- `docs/benchmarks/STAGE11_PIO_WATCHDOG.md` - PIO watchdog size / timing bench (IVP-91)
- `docs/SAD.md` - PIO SM allocation (WS2812 on PIO0 SM0); PIO watchdog; future SpaceWire PIO DS
- `docs/IVP.md` - Stage 11 PIO safety (IVP-87 through IVP-91)
- `standards/protocols/SPACEWIRE_LITE.md` - PIO DS encode / decode; LVDS needs an external transceiver

### Starcom PHY / T8 / FSK

- `starcom/docs/CONFORMANCE.md` - PICS; PIO pipe and FSK Best effort; T8 may clock bits; not 211.1
- `starcom/docs/DESIGN.md` - Forgix T8 C&S fit; T8 not on the base board; FPGA pedagogy
- `starcom/docs/WORKING_HERE.md` - do not put Forgix T8 on the base RC board for comms
- `starcom/docs/IVP.md` - increment 17 PIO pipe, 18 PHY tiers; FSK path not numbered yet
- `starcom/docs/SAD.md` - PIO vs FPGA adapter roles
- `starcom/STATUS.md` - FSK / T8 future path; FPGA HDL not offered this sitting
- `starcom/docs/ICD.md` - PHY tiers; FPGA HDL sim before bitstream
- `starcom/docs/GLOSSARY.md` - port / adapter includes later PIO / FPGA
- `starcom/docs/TESTING.md` - FPGA testbench uses the same goldens
- `starcom/docs/integration/CONSUMERS.md` - increment 18 FPGA later
- `starcom/README.md` - ports attach later PIO / FPGA
- `starcom/adapters/rp2350/README.md` - PIO-shaped PLTU bit pipe; FPGA bitstreams stay out
- `starcom/include/starcom/README.md` - `pio_port.hpp` (MSB first; not 211.1)
- `starcom/AGENT_WHITEBOARD.md` - Forgix-first / decode-port hold
- `starcom/CHANGELOG.md` - PICS + FSK path; T8 not on the base board
- `starcom/docs/comparison.md` - ComBlock / Pluto as true-PHY paths (not T8)
- `starcom/docs/research/ccsds_domain_claude.md` - SDR / FPGA IP survey (historical; DESIGN is the living note)
- `starcom/docs/research/ccsds_domain_grok.md` - same survey, Grok side (historical)

### FPGA handbooks / other

- `docs/hardware/FPGA/standards/README.md` - FPGA / space-electronics index (pointers only; not a cert kit; folder rename TBD)
- `docs/hardware/FPGA/standards/PMOD.md` - Digilent Pmod 6-pin vs 2x6; pico2-ice / Galaxia / UPduino
- `standards/starcom/README.md` - NASA / ESA FPGA handbook shelf (not a cert kit)
- `standards/starcom/fpga/README.md` - PDF index (NASA-HDBK-4008/4011, ESA ASIC/001, GSFC 500-PG)
- `docs/decisions/Telem+logging/telemetry_comparison.md` - FPGA + LVDS SpaceWire as a Nova concept
