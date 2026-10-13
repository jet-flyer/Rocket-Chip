# Onboard neural networks on an FPGA co-processor (T8 sizing)

- **Date:** 2026-10-12
- **Author:** Goddard
- **Scope:** Whether small deterministic neural networks (adaptive control, sensor anomaly detection, fault classification, flight-phase detection) fit on the Forgix's Efinix T8, and how to work around its memory limit. Taken from Nathan's DM with Goddard and finalized in the Rocket Chip group chat.
- **Status:** Frozen once committed. A later report replaces this one.

Every item below is labeled. **[Fact]** means it comes from a cited source. **[Inference]** means it is Goddard's estimate or judgment and has not been checked on hardware.

## 1. Hardware facts
- **[Fact]** The Forgix FPGA is an Efinix T8F49. It has 7,384 logic elements, 122.88 Kb of on-chip RAM, and 8 hard 18x18 multipliers (Efinix T8 data sheet; Adiuvo Forgix schematic).
- **[Fact]** On the Forgix, only RP2350 GPIO1-6 reach the T8, plus GPIO19 for the oscillator enable. GPIO1-3 are the configuration lines (SPI CS, CLK and MOSI) and can be reused as a 3-wire user link after configuration. GPIO4 and GPIO5 are CRESET and CDONE. There is no shared clock; the T8 runs from its own 32 MHz oscillator (Adiuvo schematic).
- **[Fact]** The RP2350 has 520 KB of SRAM (RP2350 data sheet).

## 2. Sizing
- **[Inference]** Small networks for anomaly detection, fault classification and flight-phase detection, run at flight-loop rates, would keep the T8's multipliers only about 1-2% busy. Compute is not the limit.
- **[Inference]** On-chip RAM is the limit. About 15 KB holds only small weight sets once buffers are counted.

## 3. Tiered memory
- **[Inference]** Keep hot weights in T8 RAM, stream the rest from RP2350 SRAM over a PIO link, and use flash or PSRAM as the slowest tier.
- **[Inference]** On the Forgix, that link is the 3-wire configuration bus after configuration, plus GPIO22 and GPIO23 if they are jumpered over. A Rocket Chip board could add cut-or-solder jumper traces between spare RP2350 GPIOs and pins on whatever FPGA it uses, to widen the bus (Nathan's idea).

## 4. Safety independence
- **[Inference, Nathan's point]** A watchdog built only from RP2350 PIO is not independent, because it shares the die, clock and power with the MCU. Real independence needs a separate chip such as the FPGA. A PIO heartbeat is at most one extra signal for the FPGA to watch.

## 5. References for deterministic, checkable networks
- NASA IFCS: self-healing flight control.
- ACAS Xu: aircraft collision-avoidance networks checked with the Reluplex and Marabou solvers.
- TaxiNet: a vision network used with run-time assurance (a simple trusted controller that takes over when the network misbehaves).
- EASA AI guidance, and SAE ARP6983.
- **[Fact]** Network solvers prove properties of a trained network's input-output behavior. SPIN checks state machines. They do different jobs.
- FPGA network tools: hls4ml, FINN and LogicNets. Neuroevolution (NEAT) can be used to shrink a network.

## 6. Open items
- Measure a real network's weight size and how much PIO bandwidth it needs. Nothing in sections 2-4 has been run on hardware.
