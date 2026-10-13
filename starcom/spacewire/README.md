# SpaceWire notes (Starcom side folder)

- **Date:** 2026-10-13
- **Author:** Hamilton (docs), from the "Rocket Chip" room (Buzz, Goddard, Duke, Nathan), 2026-10-13
- **Scope:** Collect the SpaceWire facts, cable checks, hardware options and use ideas from the 2026-10-13 room in one place. This folder is self-contained. It is a working shelf, not a design decision, and it can be removed later without breaking links elsewhere (no other file depends on it; the whiteboard FPV row only names it as background).
- **Status:** Preliminary placeholder. LOOK only. Nothing here is adopted for Rocket Chip or Starcom. This is hobby-grade use; no flight qualification is claimed.

**Labels.** **FACT** = stated by a named source (clause, datasheet section or page). **INFERENCE** = room reasoning, not checked against a source. **DATASHEET** = vendor paper value, not measured on a bench. **OVERVIEW** = standard's overview text, not a "shall" requirement. **NOT CHECKED** = open; nobody has verified it.

Main standard: **ECSS-E-ST-50-12C Rev.1, "SpaceWire - Links, nodes, routers and networks", 15 May 2019** (replaces the 2008 issue). <https://ecss.nl/standard/ecss-e-st-50-12c-rev-1-spacewire-links-nodes-routers-and-networks-15-may-2019/>. No PDFs are stored in this folder. Clause numbers below are from that issue (Duke's extraction). Commas in quoted numbers are ECSS decimal commas.

## 1. Physical layer (50-12C Rev.1)

| Topic | Label | Clause | What it says |
|---|---|---|---|
| LVDS required between units | FACT | 5.3.6.1a | Links between units use LVDS signalling. |
| Inside one unit | FACT | 5.3.6.1b | Plain logic-level (LVCMOS) pins are allowed inside one unit, with no cable rules. |
| What a "unit" is | FACT | 3.2.114 | Examples given: "instrument, processor or mass memory". |
| "Unit" = one PCB or one metal case | INFERENCE | (from 3.2.114) | Duke's reading. The standard gives no board or enclosure rule. |
| Driver amplitude | FACT | 5.3.6.2.2c | 247 mV to 454 mV differential into 100 Ω. |
| Driver common mode | FACT | 5.3.6.2.2a | 1,125 V to 1,45 V. |
| Common-mode noise note | FACT | 5.3.6.2.2b note | "Large common-mode noise is an indication that something is wrong with the circuit design or grounding arrangement." |
| Receiver | FACT | 5.3.6.2.3c | Receiver input "as defined in TIA-644-A:2012". |
| TIA-644-A receiver levels | FACT (secondary sources for TIA-644-A: TI SLLA038B <https://www.ti.com/lit/an/slla038b/slla038b.pdf>; SN65LVDS32 datasheet <https://www.ti.com/lit/ds/symlink/sn65lvds32.pdf>) | 5.3.6.2.3c | The receiver must switch at ±100 mV differential or less and work up to ±600 mV. Common mode 0,05 V to 2,35 V; inputs within 0 V to 2,4 V. Works with a 1 V ground shift (matches DS90LV048A ±1 V). |
| Receiver margin | INFERENCE (Goddard, Duke) | 5.3.6.2; 5.3.4.2 | With 247 mV to 454 mV driver swing, the margin is about 147 mV over the 100 mV threshold only on a short, low-loss cable. At the 7 dB cable insertion-loss limit, 247 mV minimum gives about 110 mV at the receiver: about 10 mV margin. |

## 2. Cable and cable assembly (50-12C Rev.1)

| Topic | Label | Clause | What it says |
|---|---|---|---|
| Construction | FACT | 5.3.2.2b | "Four screened twisted-pairs with an overall shield". |
| Other constructions | FACT | 5.3.2.2c | Type B may use another construction only if it meets 5.3.2.3 to 5.3.2.6. |
| Impedance | FACT | 5.3.2.3a/b | Each pair 100 Ω ± 6 Ω, verified by TDR (time-domain reflectometer). |
| Cable skew | FACT | 5.3.2.4a/b/c | 0,1 ns/m. |
| Assembly skew and loss | FACT | 5.3.4.2c/d | Intra-pair skew under ±0,5 ns; insertion loss under 7 dB up to 1,5 × data rate. |
| Length | FACT (5.3.4.2a); OVERVIEW (5.3.2.1 note, note 2) | 5.3.4.2a; 5.3.2.1 note; note 2 | Length "determined by data to strobe skew, jitter, and signal attenuation". Limits assume "up to 10 m length at 200 Mbps". Longer cables are OK at slow rates if signal levels and edge separation are met. |
| Shield bonding | FACT | 5.3.4.1b-g | Shield bond to shell 10 mΩ class; pigtails under 2 cm. |
| Type A | FACT | 5.3.4.3.2 | Standard 9-pin Micro-D assembly; crosstalk -20 dB class. |
| Type AL | FACT | 5.3.4.3 | Legacy assembly type; do not use for new work. |
| Type B: per-pair shields | FACT | 5.3.4.4c | "The individual shields of each of the four differential signal pairs shall be bonded to the connector shell." |
| Type B: crosstalk | FACT | 5.3.4.4d | Under -50 dB up to 1 GHz. |
| Skew budget | FACT | 5.3.7, Table 5-10 | Worked example gives 251 Mbps. |
| Rate for 2 m R&M Cat7 | INFERENCE | (from 5.3.7) | About 228 Mbps with a 2 m R&M Cat7 S/FTP cable. Not measured. |
| Unshielded Cat6A (U/UTP) | INFERENCE | (from 5.3.4.4c) | Fails 5.3.4.4c (no per-pair shield). Only pair-foiled S/FTP is a Type B candidate. |

## 3. Link behaviour (50-12C Rev.1)

| Topic | Label | Clause | What it says |
|---|---|---|---|
| Disconnect detection | FACT | 5.4.8c | No edge for more than 727 ns is a disconnect, flagged within 1 µs. |
| Time-codes | OVERVIEW | 4.2.3.5 | Time-codes synchronise units "with low jitter", sent with high priority, "alleviating the possible need for a separate time distribution network". |
| Time-code rules | NOT CHECKED | 5.6.4 | The binding time-code rules are in 5.6.4; not read yet. |
| Broadcast interrupts | OVERVIEW | 4.2.3.6 | "Up to 32 different interrupts can be broadcast over the network with high priority." |

## 4. ECSS SpaceWire books

| Standard | Date | Link |
|---|---|---|
| ECSS-E-ST-50-12C Rev.1 - Links, nodes, routers and networks | 15 May 2019 | <https://ecss.nl/standard/ecss-e-st-50-12c-rev-1-spacewire-links-nodes-routers-and-networks-15-may-2019/> |
| ECSS-E-ST-50-53C - CCSDS packet transfer protocol (protocol ID 0x02) | 5 February 2010 | <https://ecss.nl/standard/ecss-e-st-50-53c-spacewire-ccsds-packet-transfer-protocol-5-february-2010/> |
| ECSS-E-ST-50-51C - Protocol identification | 5 February 2010 | <https://ecss.nl/standard/ecss-e-st-50-51c-spacewire-protocol-identification-5-february-2010/> |
| ECSS-E-ST-50-52C - RMAP (Remote Memory Access Protocol) | 5 February 2010 | <https://ecss.nl/standard/ecss-e-st-50-52c-spacewire-remote-memory-access-protocol-5-february-2010/> |

50-51C, 50-52C and 50-53C are listed only; their clauses are not extracted.

## 5. Cable findings (hobby options)

| Item | Label | Finding |
|---|---|---|
| R&M Cat.7 S/FTP | DATASHEET | 100 ± 5 Ω mean: passes 5.3.2.3 on paper. NEXT ≥ 61 dB at 600 MHz, 78 dB typical at 1 GHz: clears -50 dB. |
| Digitus Cat7 S/FTP | DATASHEET | 100 ± 5 Ω mean: passes on paper. |
| Barpa, Infinique Cat7 | DATASHEET | Only ±15 Ω (1-100 MHz) / ±25 Ω (to 600 MHz): fail on paper. "Cat7" on the label is not enough. |
| Insertion loss at 1-2 m | INFERENCE | Fraction of a dB at SpaceWire rates; passes easily. |
| Connector | DATASHEET | Metz RJ45 field plug pro 130E405032-E named as the candidate plug. |
| Per-pair foil bonding to shell (5.3.4.4c) | INFERENCE | Standard RJ45 plugs bond only the overall shield; per-pair bonding is a hand modification. GG45 / TERA connectors NOT CHECKED. |
| Plastic (unshielded) RJ45 plug | INFERENCE | Fails the shield-bonding rules of 5.3.4.1b. |
| Naddod cheap patch cords | INFERENCE | Braid only, no per-pair foil: avoid. |
| Official route | FACT | Type A: 9-pin Micro-D (MIL-DTL-83513) with ESCC 3902/003 cable. STAR-Dundee sells lab cables. |
| Micro-D assembly prices | NOT CHECKED | Glenair and Axon are quote-only. |
| Ready-made compliant RJ45 cable | OPEN | None named and verified. |

## 6. Hardware notes

| Item | Label | Note |
|---|---|---|
| AT86RF215 I/Q interface | FACT (Atmel-42415 §4.5, datasheet-only) | Bidirectional LVDS: TXCLK/TXD into the radio, RXCLK/RXD out of it. |
| Lowest-BOM route | INFERENCE | Resistor network for the TX pairs plus one receiver IC (DS90LV048A) for RX, unless the FPGA has true LVDS inputs. "Resistors only" covers TX only. |
| DS90LV048A receiver | FACT (TI datasheet §8.1) | Detects 100 mV over ±1 V common mode around 1,2 V (inputs 0 to 2,4 V). |
| DS90LV047A | FACT | Matching quad LVDS driver. |
| CAN levels | DATASHEET | CAN common-mode range -2 to +7 V, classic CAN 1 Mb/s (TI SLOA101B, TIDUCF3, SLLA337A, SLLA486C, SLLA270). Several times the LVDS ground-shift margin. |
| CAN FD rate | INFERENCE | A few Mb/s. |
| LVDS between boards | INFERENCE | Needs a solid shared ground or shield return, not just the pair. |
| Efinix Trion T8 (DS v5.5) | DATASHEET | Q144: 6 TX / 6 RX hard LVDS, 600 Mb/s, 100 Ω on-die termination. F49 (Forgix T8F49I2X): no LVDS. |
| Emulated LVDS on Trion | INFERENCE | Possible on F49 with resistors; not checked. |
| Q144 package vs Feather | DATASHEET + INFERENCE | T8 Q144 (LQFP-144): 20 × 20 mm body, about 22 mm across the leads (T8 datasheet v5.5, datasheet). Doesn't fit a Feather or Feather+ (about 22.9 mm wide) once header rows sit on both sides (inference, no layout done). Fine on a larger bespoke board. On a Feather-width board, use a small package plus resistor-network TX and a DS90LV048A for RX. |
| iCE40 LP/HX | DATASHEET | LVDS25E emulated output with 3-resistor network (Fig 3-1, TN-02213) on any bank; true LVDS inputs on bank 3 only. |
| iCE40 UP5K | DATASHEET | No LVDS. |
| RP2350 HSTX | DATASHEET | Output-only, GPIO12-19. |
| HSTX driving LVDS pairs | INFERENCE | Via resistor network or a DS90LV047A; needs a scope test before any claim. |

## 7. Uses

| Use | Label | Note |
|---|---|---|
| Spacecraft onboard networks | FACT (room sources) | JWST, Gaia, GOES-R, ExoMars cameras. |
| Multi-node rocket (TVC, self-landing booster) | INFERENCE | Good fit: time-synced vehicle state, TVC commands, booster guidance. Time-codes, routing, flow control earn their keep. |
| Staging | INFERENCE | Separation pulls a connector; 5.4.8c gives a clean "link lost" event within 1 µs. Open: breakaway connectors that keep 100 Ω pairs. |
| Staging / abort events | INFERENCE | 32 broadcast interrupts (4.2.3.6) fit. |
| Radio far from flight electronics | INFERENCE (Nathan's idea) | High-rate link from the flight computer to a remote radio. |
| FPV camera ribbon (MIPI D-PHY) | DATASHEET + INFERENCE | D-PHY: 140-270 mV swing, 150-250 mV common mode (Tektronix D-PHY note). Inference: the ribbon is the weak link in a digital FPV stack. |
| Digitizing analog video (ADV7180) | INFERENCE | Adds latency of a few lines to a frame. |
| FPV stack | INFERENCE | Separate boards count as separate units, so LVDS rules apply. Use CAN/DroneCAN for control and telemetry, LVDS/SpaceWire for high-rate links. |

Analog FPV video is not a SpaceWire job. See the Rocket Chip `AGENT_WHITEBOARD.md` row "Differential analog FPV video (Telestar FPV) (LOOK)".

## Prior art: open and university SpaceWire (Goddard, 2026-10-13)

| Item | Label | Note |
|---|---|---|
| SpaceWire Light <https://github.com/bc-iwf/spacewire-light> | FACT (repo) | GPL. 50-12C codec, 200 Mb/s on Spartan-3. No RMAP, no routing. |
| SpaceWire Reloaded <https://github.com/lcapossio/spacewire_reloaded> | FACT (repo) | LGPL. AXI, VHDL/Verilog, cocotb tests, Arty A7. Most hobby-friendly of the set. |
| Shimafuji 6-port router <https://github.com/shimafujigit/SpaceWireRouterIP_6PortVersion> | FACT (repo) | VHDL router with RMAP, 2014. |
| PoliTo thesis <https://webthesis.biblio.polito.it/8488/1/tesi.pdf> | FACT (thesis) | University SpaceWire implementation. |
| Chalmers / RUAG thesis <https://publications.lib.chalmers.se/records/fulltext/157904.pdf> | FACT (thesis) | Small codec, under 500 cells. |
| Tallinn thesis <https://a-lab.ee/edu/sites/default/files/MAG_Andrei_Maalberg.pdf> | FACT (thesis) | SpaceWire-to-SPI bridge on ProASIC3E. |
| 2025 Portuguese thesis <https://exa.ai/library/publication/lypr32k3d82> | FACT (thesis) | University SpaceWire work, 2025. |
| Twins4Space, Würzburg <https://www.montenegros.de/sergio/public/iaa23-t4s.pdf> | FACT (paper) | Meshed SpaceWire on-board computer on RODOS. Closest to the multi-node rocket idea. |
| MCU-only hobby implementation | NOT CHECKED | None found. |
| Light / Reloaded on iCE40 HX or Trion T8 | INFERENCE | Small enough to fit; no port found. |
| Maturity: SpaceWire Light | FACT (Goddard) | Complete since 2011. Used in Kayser-Threde VIBANASS (ground qualification at ESA EPOS). Its manual says it is "not designed for mission critical applications". |
| Maturity: Shimafuji open IP (codec, RMAP target, router) | FACT (Goddard) | Used in JAXA ASTRO-H ground test equipment. Last updated 2014. |
| Maturity: SpaceWire Reloaded | FACT (Goddard) | New; no track record. |
| Interop test: SpaceWire Light (original, OpenCores) | FACT (OpenCores, Manual.pdf) | Tested on Spartan-3 and Virtex-5 against "a commercial SpaceWire product". Manual.pdf says only "a commercial SpaceWire interface product" (XC3S2000-4). The product is not named. |
| Interop test: IWF Graz fork <https://github.com/bc-iwf/spacewire-light> | FACT (fork's own report only) | Conformance report PLATO-IWF-PL-TR-0100, Issue 1.0 (22 Jul 2021). Core as used in the PLATO ICU RDCU FPGA, on a development board, tested against a STAR-Dundee SpaceWire Conformance Tester (v2.4.2) and a STAR-Dundee SpW USB Brick Mk2. §3: the core "is in line with the SpW standard definition"; listed failures are "caused only by the specific test conditions". |
| IWF report: standard issue | FACT (report) | Applicable standard is ECSS-E-ST-50-12C (07/2008), not Rev.1. |
| Rev.1 (15 May 2019): backward compatibility | FACT (50-12C Rev.1, per Duke) | Change log: Rev.1 "aims to be compliant to the previous issues". Annex A: "without making substantial technical changes". |
| Rev.1: 2008-issue devices | FACT (50-12C Rev.1 Annex A, 5.5.7.8, per Duke) | Annex A: 2008-issue devices "are interoperable with a new device", and "clause 5.5.7.8 ensures that older devices remain compliant". 5.5.7.8 allows the 2008 LinkDisabled behaviour for a device "developed prior to this current revision". |
| Rev.1: time-code rule | FACT (50-12C Rev.1 5.6.4.5b, 5.6.4.8, 5.6.4.9, per Duke) | 5.6.4.5b: a time-code is valid only if it is "one more, modulo 64" than the register. 5.6.4.8: only valid time-codes go to the host. 5.6.4.9: an invalid time-code still updates the register. |
| IWF result carry-over to Rev.1 | INFERENCE (Duke) | The link-level conformance result carries to Rev.1 through 5.5.7.8. Rev.1 distributed interrupts (5.6.5) are not in the core, but they are optional ("shall be optionally implemented", 5.6.5.1). Router-only changes are not checked; they do not apply to a point-to-point codec. |
| SpaceWire Light: tick_out deviation | FACT (Manual.pdf; 50-12C 2008 §8.12; Rev.1 5.6.4.5b/5.6.4.9) | `tick_out` pulses on every received time-code. 50-12C (2008) §8.12 wants a pulse only when the value is the previous value + 1; Rev.1 keeps this rule. The manual says to filter time-codes outside `spwstream`. The filter is still needed for Rev.1, and it must also load the register with invalid values (5.6.4.9). |
| Interop test: Shimafuji open IP | FACT (repo) | Used in SpaceWire-to-GigabitEther, built for "ground tests of flight modules which use SpaceWire interfaces" on ASTRO-H <https://github.com/yuasatakayuki/SpaceWireToGigabitEther>. |
| Interop test: SpaceWire Reloaded | FACT (repo) | Internal loopback on Arty A7 only. No test against external equipment. |
| pbauza Ultra96 transceiver <https://github.com/pbauza/SpaceWire_Transmitter_Receiver> | FACT (repo) | Checked with a STAR-Dundee Link Analyser Mk3. |
| Maturity order | INFERENCE (Goddard, Duke) | IWF Graz fork first, then Shimafuji, then Reloaded. Start from the bc-iwf fork, not the original OpenCores copy, and filter time-codes as the manual says. |
| Which core to use | INFERENCE | SpaceWire Light for point-to-point links. Add the Shimafuji RMAP target / router only if the multi-node idea goes ahead. |

## 8. Ideas (not decided)

- **Feather+ layout:** a Feather-compatible SpaceWire expansion board; an FPGA on the board could serve other MCUs (M0, ESP32).
- **Resistors-only TX BOM option:** keep as a low-cost variant (still needs one RX receiver IC, see §6).

## 9. Open items

- TIA-644-A receiver text: closed from secondary sources (TI SLLA038B, SN65LVDS32 datasheet). The TIA-644-A document itself is still not in hand.
- 50-12C 5.6.4 binding time-code rules not read.
- No verified compliant RJ45 cable; no bench TDR or skew measurement.
- HSTX pseudo-differential scope test.
- Micro-D assembly prices (quote-only).
- Prior-art cores: interop testing closed (see Prior art). Remaining gap: the commercial product used to test the original SpaceWire Light is not named in OpenCores or Manual.pdf. Our own bench tests are still required.
- GG45 / TERA connectors.

Related in-repo draft: [`../../standards/protocols/SPACEWIRE_LITE.md`](../../standards/protocols/SPACEWIRE_LITE.md) (not changed by this file).
