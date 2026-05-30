# Starcom CCSDS Library Research Document

**Purpose:** Comprehensive external research and analysis to support the design and implementation of Starcom, a standalone, high-fidelity, modern C++ CCSDS Data Link (and eventually Physical) Layer library. Intended for release independently of Rocket-Chip while remaining cleanly integrable as a dedicated Active Object.

**Status:** Living document — research phase only. No implementation or execution plan yet.

**Usage by Multiple Agents:**  
This document is explicitly structured for handoff between agents (Grok, Claude, future reviewers, council personas). Every major section **must** carry clear authorship and date attribution. When editing or extending a section written by another agent, add a new sub-section or "Update" block rather than silently rewriting. Repo files are authoritative.

**Research Principles (per user direction 2026-05-30):**
- Primary focus: External (web) standards, prior art, hardware feasibility, and library craft patterns.
- Official CCSDS compliance requirements come first.
- Only use Rocket-Chip repository as light reference for integration fit (AO model, event posting patterns, radio driver constraints).
- Current Rocket-Chip CCSDS/telemetry code is acknowledged as an incomplete STOP-GAP and is not used as a design base.

**Document Structure & Attribution Rule:**
- Top-level sections are owned by the authoring agent.
- Add `**Written by:** Agent Name (interface) — YYYY-MM-DD` at the start of each major section.
- Use `**Updated by:**` blocks for subsequent contributions.

**Last Major Update:** 2026-05-30 (initial skeleton + first external research wave)

---

## Corrections & Clarifications from User (2026-05-30)

**Added by:** Grok 4.3 (Build CLI) — 2026-05-30

The following two points were explicitly clarified by the user and must be respected in all future research and writing in this document:

1. **Physical Layer Scope**: When the user asks about "Physical Layer" feasibility with PIO and current hardware, they mean **the Physical Layer as formally defined by the CCSDS standards** (primarily CCSDS 211.1-B-4 for Proximity-1), not a generic or loosely-defined software physical layer. 
   - The CCSDS Physical Layer is narrow: it modulates a *coded symbol stream* coming from the Coding & Synchronization Sublayer (211.2-B) and reports only `CARRIER_ACQUIRED` and `SYMBOL_INLOCK_STATUS` to the MAC Sublayer.
   - Many functions commonly associated with "PHY" work (ASM/sync marker detection, PLTU construction, detailed symbol synchronization) actually reside in the C&S Sublayer.

2. **JPL User Terminal Attribution**: The S-band Proximity-1 work associated with Blue Ghost Mission 2 is the **JPL User Terminal** (a JPL-developed SDR payload, built with Vulcan Wireless as the radio vendor). It flies on the Firefly Blue Ghost 2 lander. It is **not** "Firefly's protocol." JPL sources should be prioritized in research.

All future sections and updates must reflect these distinctions clearly.

---

## 1. Official CCSDS Requirements for Full Compliance (Data Link + Physical Layer)

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30

### 1.1 Core Documents (Authoritative Blue Books)

- **COP-1**: CCSDS 232.1-B-2 (with Technical Corrigendum 1, 2019) — Communications Operation Procedure-1.
  - Source: https://ccsds.org/Pubs/232x1b2e2c1.pdf
  - Critical content: FOP-1 and FARM-1 state machines (Tables 5-1 and 6-1), CLCW exact bit layout and semantics, Managed Parameters (MIB), inter-procedure interfaces, service primitives.

- **Proximity-1 Data Link Layer**: CCSDS 211.0-B-6 (2020) — https://ccsds.org/Pubs/211x0b6e1.pdf
  - COP-P (Proximity-1 equivalent), PLCW, timing services, hailing, session management.

- **Proximity-1 Physical Layer**: CCSDS 211.1-B-4 (2013) — https://ccsds.org/Pubs/211x1b4e1.pdf
  - PLTU (Proximity Link Transmission Unit), Acquisition Sequence Marker (ASM), modulation, frequencies, hailing channel, bit synchronization, etc.

- **USLP**: CCSDS 732.1-B-3 (June 2024) — https://ccsds.org/Pubs/732x1b3e1.pdf
  - Modern unifying frame format that can carry both COP-1 and COP-P services on different Virtual Channels.

- Supporting: CCSDS 132.0-B / 232.0-B (TC/TM Space Data Link Protocols) for frame structures and OCF usage.

### 1.2 What "Full Official Compliance" Requires (Data Link Layer)

**CLCW (Command Link Control Word)**: Exact 32-bit structure carried in TM OCF (Type-1 report):
- Control Word Type, Version Number, COP in Effect, VCID, Status fields (Retransmit Flag, Lockout Flag, Wait Flag, No RF Available, etc.), FARM-B Counter, Report Value (N(R) — next expected TC frame sequence number).

**FARM-1 (Receiver / Spacecraft side)**: Must implement the complete state table (Table 6-1 in 232.1-B-2), sliding window logic (W, PW, NW widths), and CLCW generation rules.

**FOP-1 (Sender / Ground side)**: Must implement the complete state table (Table 5-1), retransmission logic (go-back-N ARQ), timer management, and interpretation of incoming CLCWs.

**COP-P / Proximity-1 Data Link**: Equivalent state machines using PLCW, with additional concepts for proximity links (hailing, session establishment, intermittent connectivity).

**Managed Parameters (MIB)**: All parameters listed in §7 of COP-1 and equivalent sections in 211.0 must be configurable/exposed (not hard-coded).

**USLP Mapping**: Frames must be usable over USLP with proper VC multiplexing so the same library instance can support both long-haul COP-1 and proximity COP-P.

**Error Control, Segmentation, etc.**: Depending on the exact service (AD, BD, etc.).

**Documentation Requirement**: Every class/method must cite the exact Blue Book section/table it implements.

### 1.3 Physical Layer Requirements (for Reference — Full Native Compliance)

**From CCSDS 211.1-B-4** (to be extracted in detail):
- PLTU structure (including Acquisition Sequence + Start Sequence Marker).
- Specific ASM patterns (commonly 0xFAF320 or mission-specific variants).
- Bit synchronization, modulation (e.g., PCM/PSK/PM, subcarrier, etc.).
- Hailing channel and acquisition procedures.
- Timing services and time tagging requirements.
- Radio control interfaces (frequency, power, mode switching).

Full native Prox-1 Physical Layer compliance is **not** achievable with generic COTS packet radios without significant external hardware or custom silicon.

---

## 2. Physical Layer Feasibility with RP2350 PIO + Current Hardware (915 MHz LoRa / RFM95W)

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30  
**Status:** Substantial external research completed (standards + hardware datasheets + real PIO examples). Summary below.

### 2.1 Official Requirements Summary (from CCSDS 211.1-B-4)

Proximity-1 Physical Layer is intentionally thin by design:
- Accepts coded symbol stream from the Coding & Synchronization Sublayer.
- Modulates onto the RF carrier.
- Provides two key status signals back to the MAC Sublayer: CARRIER_ACQUIRED and SYMBOL_INLOCK_STATUS.
- PLTU (Proximity Link Transmission Unit) = Attached Sync Marker (ASM) + Version-3 Transfer Frame + CRC-32.
- Hailing channel + acquisition procedures are first-class concepts for session establishment in intermittent/proximity environments.
- The 2013 Blue Book is primarily UHF/Mars-focused. Lunar S-band adaptations (new profiles, additional coding/modulation options for spectrum efficiency) were developed ~2024 and are documented in LunaNet Interoperability Specification v5 and SSTL Lunar Pathfinder materials.

Full native compliance with a specific Prox-1 Physical Layer profile requires:
- Support for the exact modulation scheme(s) and indices defined for that profile.
- Support for the hailing channel waveform and timing.
- Precise bit/symbol timing and (in some cases) ranging support.

### 2.2 Current Hardware Reality (SX1276 / RFM95W)

From the official Semtech SX1276 datasheet (Rev. 4, March 2015):
- Supports LoRa + high-performance (G)FSK, MSK, GMSK, OOK.
- In FSK/OOK modes: Built-in bit synchronizer for clock recovery, preamble detector, SyncAddress recognition, and explicit Manchester encoding/decoding support in the packet engine (see datasheet Figure 36 and related registers).
- Continuous Mode vs Packet Mode: Continuous Mode exposes DCLK and DATA on DIO pins — this is the closest the chip gets to raw bitstream access without the full packet engine.
- Strong RSSI, AFC, AGC, and channel filter programmability.
- **Fundamental limitation**: The radio is optimized for its own packet formats and LoRa chirps. It does not natively generate or demodulate arbitrary CCSDS Physical Layer waveforms (specific PCM/PM schemes with exact modulation indices required by some Prox-1 profiles, custom hailing waveforms, etc.).

### 2.3 RP2350 PIO + Software-Defined Physical Layer Feasibility (Strongly Positive)

Real-world evidence from the Raspberry Pi ecosystem and community (all portable to RP2350):

- Official `pico-examples/pio/manchester_encoding` and `differential_manchester` — complete bidirectional examples with precise timing state machines (one SM for TX encoding with side-set, one for RX edge detection and decode). This is production-quality starting material.
- Real RF deployments:
  - 433 MHz OOK Manchester decoders (PIO handles edge detection + Manchester decode + streams bits; CPU does sliding-window header/sync detection).
  - ADS-B (1090 MHz) receiver using **two PIO state machines**: one dedicated to preamble/sync pattern correlation on the pulse train from an RF frontend, second for Manchester decoding of the message. This is an extremely close analog to what we need for ASM detection + PLTU framing.
- High-speed bit-banged protocols (e.g., 100BASE-TX Ethernet on PIO) demonstrate that RP2350 PIO can handle complex encoding, preamble/SFD detection, and multi-level signaling at impressive rates with DMA offload.

**Conclusion for Starcom (with current 915 MHz hardware):**

A high-value "software Physical Layer" is realistically achievable:
- Use FSK Continuous Mode (or a carefully configured LoRa mode) on the SX1276.
- PIO state machines for: ASM / sync word detection (including the classic 0xFAF320 or mission-specific markers), Manchester / Bi-Phase-L encoding/decoding, precise bit timing and edge capture, PLTU assembly/disassembly.
- CPU / DMA handles higher-level framing, CRC, and handoff to the pure Data Link Layer (COP-1 / COP-P / USLP).
- This delivers excellent, standards-respecting COP-1 and COP-P performance over the existing legal 915 MHz link without requiring a new radio module immediately.
- Full native Prox-1 Physical Layer (specific lunar S-band waveforms, certain high-performance modulation indices, hardware-accelerated ranging) will still require a more capable SDR or dedicated transceiver in the future.

**Honest documentation recommendation:** Clearly label the library as "Data Link Layer + Software-Defined Physical Layer over COTS packet/continuous radios" with a well-defined IRadio / IPhysicalLayer abstraction boundary. This allows future drop-in replacement of the PHY module for true native Prox-1 when hardware is available.

### 2.4 Recommended Next Concrete Steps for This Section

- Extract the exact ASM patterns, hailing procedures, and modulation tables from 211.1-B-4 + LunaNet v5 appendices.
- Prototype-level PIO experiments (start with the official Manchester example + DIO continuous mode on an SX1276 test board).
- Power / range / timing characterization of FSK continuous + PIO vs current LoRa packet mode.

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30 (initial deep synthesis)

### 2.6 Commercial Radios Capable of Full CCSDS 211.1-B-4 Physical Layer Implementation

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30

**Goal of this section**: Identify commercially available (non-space-qualified / non-rad-hard) radios or implementations that can do the *full* official CCSDS Physical Layer as defined in 211.1-B-4, within a reasonable budget for research/HPR/cubesat-class use (not thousands of dollars per radio if possible).

#### Key Findings

True turnkey, affordable, COTS hardware radios that implement the complete CCSDS 211.1-B-4 Physical Layer (with the exact Bi-Phase-L + residual carrier at 60° ±5%, required phase noise performance, acquisition behavior, etc.) are extremely rare. This is because Proximity-1 is a specialized deep-space / proximity operations protocol.

**Best realistic commercial option found:**

- **ComBlock COM-1852SOFT** (VHDL IP Core)
  - Explicitly implements the full CCSDS 211.1-B-4 Physical Layer.
  - Supports PCM/Bi-Phase-L phase modulation with programmable symbol rates.
  - Designed as a portable FPGA IP core (tested on Xilinx, portable).
  - Price: Approximately **$2,500** for an unlimited-use license.
  - This is the clearest "affordable commercial" path for a research-grade full implementation. You would pair the IP core with a suitable commercial FPGA board + RF front-end (UHF or S-band as needed).
  - Source: Direct from comblock.com product page.

**Other relevant options (higher cost or requiring customization):**

- **Vulcan Wireless NSR-SDR-S/S** (S-band SDR transponder)
  - Supports Proximity-1 waveforms (the same class of hardware used in the JPL User Terminal context).
  - Full software-defined radio, reprogrammable, flight-proven heritage.
  - Pricing: Not publicly listed (quote only). Expect mid-to-high four figures or more per unit. Positioned for smallsat / cislunar use.
  - Not "cheap," but one of the few commercial products that actually supports the protocol family.

- **Syrlinks / GomSpace / EmTroniX ecosystem**
  - Syrlinks EWC31-NG and similar S-band transceivers have been considered or customized for Proximity-1 in lunar CubeSat studies (e.g., LUMIO).
  - GomSpace / EmTroniX have done ESA-funded work on autonomous Proximity-1 SDR transceivers.
  - These are professional smallsat-grade radios. Pricing is quote-only, typically in the $10k–$50k+ range depending on configuration and qualification. Not turnkey "full 211.1 out of the box" without customization in most cases.

- **L3Harris Electra-lite / similar heritage UHF transceivers**
  - True native CCSDS Proximity-1 support (Mars relay heritage).
  - Professional / mission-grade pricing (well into six figures). Not relevant for the budget constraint.

**Overall Assessment for Research / Non-Space Use**

- The **ComBlock IP core at ~$2,500** is currently the most practical way to get a *full, standards-compliant* CCSDS 211.1-B-4 Physical Layer implementation without spending serious money on space hardware.
- Everything else that can do the real thing tends to be either:
  - Custom FPGA development on top of COTS SDR platforms (USRP + custom code), or
  - Professional smallsat radios that require customization and cost significantly more.

For your stated constraints ("not going to the Moon" and "not spending thousands on just the radio"), the ComBlock route + a commercial FPGA + appropriate RF frontend (or even a modified COTS transceiver in FSK mode) is the most realistic path to a true full Physical Layer if you ever decide you need 100% compliance instead of the best-effort approximation over the SX1276.

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30 (commercial radio survey)

### 2.7 Practical In-Field Impact of Using a Best-Effort Physical Layer Approximation (for <50 km Use Cases)

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30

**Question being answered:** If we cannot implement the full CCSDS 211.1-B-4 Physical Layer with current hardware (SX1276 + RP2350 PIO), what does that actually cost us in real operations at rocket-typical ranges (<50 km)?

#### What the Full CCSDS Physical Layer Actually Buys You

The specific waveform and requirements in 211.1-B-4 (residual carrier Bi-Phase-L at ~60° modulation index, tight phase noise, low residual AM, precise stability, etc.) are optimized for:

- Operating reliably at **very low SNR** (weak signals, long slant ranges, low power assets).
- Fast and robust **carrier and symbol acquisition** after long periods of silence or when the link geometry changes rapidly.
- Predictable behavior that the MAC and Data Link layers (hailing, session management, COP-P) can depend on.
- Support for coherent modes and ranging when needed.

This design comes from Mars relay experience, where an orbiter may only have a short pass over a lander/rover with limited power and unknown orientation after landing.

#### Realistic Assessment for Your Use Case (<50 km, 915 MHz, High-Power Rocketry)

At distances under 50 km on 915 MHz with reasonable transmit power (even 1–5 W) and decent antennas, you are usually operating with **significantly higher link margins** than a typical Mars proximity link.

**Areas where the difference is likely small:**
- Normal flight telemetry downlink while the link is solid.
- Command uplink during pre-launch, boost, or coast phases when the vehicle is still relatively close and oriented reasonably.
- Bulk data offload when the rocket is on the ground with good antenna visibility and you can wait for the link to stabilize.

**Areas where you will feel the limitation (even under 50 km):**

1. **Post-landing / recovery scenarios (the most relevant for Starcom goals)**
   - After landing the vehicle may be on its side, partially buried, or have a compromised antenna.
   - There can be long periods of silence.
   - You want the ground station to be able to reliably "wake up" the vehicle and pull the flight log with minimal operator intervention.
   - This is exactly the kind of intermittent, blackout-recovery use case Proximity-1 was designed for. A best-effort FSK approximation will generally have:
     - Longer or less reliable acquisition time after silence.
     - Higher chance of failing to acquire in marginal orientations/power levels where a proper compliant PHY would succeed.
     - Less efficient use of the available link margin.

2. **Edge-of-coverage or degraded conditions during descent**
   - Tumbling, changing aspect angle, or increasing range can push the link into lower SNR territory.
   - The full Physical Layer (especially the specific modulation + acquisition design) gives you more margin and more predictable behavior here.

3. **Future expansion**
   - If you ever want ranging, more sophisticated timing services, or move to S-band / true proximity operations, the gap becomes larger.

#### Rule of Thumb for Your Current Hardware + Best-Effort Approach

- **Strong links** (good antenna visibility, <10–15 km, decent power): The practical difference is small. Your current SX1276 + PIO approximation should work well for most telemetry and command needs.
- **Stressed links** (post-landing recovery, longer range, poor orientation, low power): You will notice reduced robustness compared to a true CCSDS Physical Layer implementation. Acquisition after long blackouts and performance at the edge of the link will be the main pain points.
- The **Data Link Layer + MAC** (COP-1/COP-P state machines, CLCW/PLCW, hailing logic, session management) still provide enormous value even with a non-compliant PHY. Many of the reliability benefits people associate with Proximity-1 come from those upper layers, not just the waveform.

**Bottom line recommendation:**
For your stated use cases and distance, a well-designed best-effort Physical Layer adapter (FSK continuous mode + PIO for symbol handling) is a very reasonable engineering choice today. It will not be as good as a full 211.1-B-4 implementation in the hardest recovery scenarios, but the gap is manageable and the alternative (buying or building real compliant hardware) is expensive.

Document the limitations clearly in Starcom so future users (or future hardware upgrades) know exactly where the approximation starts to cost them performance.

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30 (practical impact assessment)

---

### 2.5 Corrected Analysis: Official CCSDS Physical Layer (211.1-B-4) vs Current Hardware + PIO (Updated 2026-05-30)

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30

**Scope note:** This analysis is strictly limited to the Physical Layer as defined in CCSDS 211.1-B-4 (Proximity-1 Space Link Protocol — Physical Layer). It does **not** address the Coding & Synchronization Sublayer (211.2-B) or Data Link Layer functions.

#### What the CCSDS Physical Layer Actually Is (per 211.1-B-4)

From the standard:

- The Physical Layer sits directly below the Coding & Synchronization Sublayer.
- **Transmit path**: It receives a stream of *coded symbols* from the C&S Sublayer. It modulates those symbols onto the RF carrier according to configured parameters (Bi-Phase-L encoding is required, with residual carrier at a modulation index of 60° ±5%).
- **Receive path**: It provides only two binary status signals to the MAC Sublayer:
  - `CARRIER_ACQUIRED`
  - `SYMBOL_INLOCK_STATUS`
- When `SYMBOL_INLOCK_STATUS` is true, it delivers the received coded symbol stream upward to the C&S Sublayer.
- It is responsible for basic transceiver control (on/off, frequency, modulation state, etc.) via control variables passed from the MAC.

The Physical Layer is **not** responsible for:
- ASM / sync marker detection or PLTU construction (those are in the C&S Sublayer).
- Higher-level hailing logic or session state (MAC Sublayer).

It is essentially a controlled RF modulator/demodulator with carrier and symbol lock detection.

#### Specific Requirements That Are Difficult or Impossible with Current Hardware

Key normative requirements from 211.1-B-4 that the SX1276 + RP2350 PIO combination struggles to meet:

- **Modulation (3.3.5)**: PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier with residual carrier at exactly 60° ±5% modulation index. The SX1276 can do Manchester/Bi-Phase in packet mode, but achieving the precise residual carrier characteristics and modulation index with the required spectral purity on a LoRa/FSK transceiver is not supported in the way the standard expects.

- **Performance (3.4)**:
  - Long-term oscillator stability ≤ 10 ppm.
  - Short-term stability ≤ 1 ppm over 1 minute.
  - Residual AM < 2% RMS.
  - Strict phase noise masks (especially in non-coherent mode).
  - Precise coded symbol rate accuracy and stability.

- **Acquisition behavior**: The receiver must sweep and acquire carrier lock, then symbol lock, and report the two status signals with behavior the upper layers rely on.

The SX1276 datasheet shows it has good FSK performance and some built-in sync features, but it is not designed as a CCSDS Proximity-1 Physical Layer transceiver. It lacks the fine-grained control and the RF characteristics needed for full compliance.

#### Realistic Assessment for Rocket-Chip Hardware

With the current 915 MHz RFM95W (SX1276) + RP2350:

- It is **not possible** to implement a conformant CCSDS 211.1-B-4 Physical Layer.
- It **is possible** to build a useful approximation of the *interface services* the Physical Layer is supposed to provide to the Data Link/MAC layers (i.e., a way to send and receive symbol streams with some carrier and symbol lock indication) using FSK continuous mode + PIO for timing and encoding.

This approximation can be very effective for reliable COP-1 and COP-P Data Link operation over the existing legal 915 MHz link. However, it should be documented as a **best-effort adaptation layer**, not as an implementation of the CCSDS Physical Layer.

#### Recommendation for Starcom

The cleanest architecture is to define the library's `IProximityPhysicalLayer` interface based strictly on the services described in 211.1-B-4. Then provide:

- A "COTS Radio Approximation" implementation for current hardware (clearly labeled as non-conformant at the PHY level).
- Future "Native PHY" implementations when better RF hardware is paired with the system.

This approach respects the standard while still delivering high value today.

**End of corrected analysis.**

### 2.1 Hardware Constraints (Current Rocket-Chip Setup)

- Primary radio: RFM95W (SX1276 LoRa transceiver) on 915 MHz ISM band (legal in US for unlicensed use at appropriate power).
- Capabilities (from SX1276 datasheet):
  - LoRa mode (chirp spread spectrum) — excellent range, but packet-oriented with built-in preamble + CRC + whitening.
  - FSK/OOK continuous and packet modes.
  - Limited raw bit-level control; the chip handles a lot of framing internally.
  - DIO pins for interrupts (RxDone, TxDone, etc.), but not arbitrary bit-stream access in all modes.
  - No native support for CCSDS-specific Physical Layer formats (PLTU, specific ASMs, Proximity-1 hailing waveforms).

- RP2350: Dual Cortex-M33 + PIO (Programmable I/O) blocks. PIO is extremely powerful for bit-banging, state machines, and precise timing (up to ~125-150 MHz effective).

### 2.2 What PIO + Software Can Realistically Do (Early Assessment)

**Feasible (with effort):**
- Custom ASM (sync word) detection on top of FSK or even LoRa in certain configurations (PIO can watch DIO or bit-banged SPI-like streams).
- Manchester / Bi-Phase encoding/decoding (common in legacy CCSDS and Prox-1).
- Precise bit-level timing and time-tagging of edges (PIO excels here).
- PLTU framing / de-framing once raw bits are recovered.
- Software state machines for hailing sequence detection (at Data Link + limited Physical).
- Custom preambles or low-level acquisition aids in FSK continuous mode.

**Very Difficult or Impossible without Hardware Changes:**
- True Proximity-1 Physical Layer waveforms (specific modulation schemes, subcarriers, or spread-spectrum patterns defined in 211.1-B).
- High-rate S-band or UHF native Prox-1 (requires appropriate RF frontend + up/down conversion).
- Full "fast hailing" channel with the exact timing and modulation specified for native Prox-1.
- Bit-synchronous operation at the precision needed for some legacy or high-performance modes without external clock recovery hardware.

**Promising Hybrid Path:**
- Use PIO + DMA to implement a "bitstream mode" wrapper around the SX1276 in FSK continuous or a modified LoRa mode.
- This gives a soft Physical Layer that is "CCSDS-inspired" and sufficient for reliable COP-1/COP-P Data Link operation over COTS 915 MHz hardware.
- Document clearly what is "Data Link Layer over COTS radio" vs "full native Physical Layer compliant."

### 2.3 Recommended Research Next Steps (for this section)

- Detailed extraction of every Physical Layer primitive from 211.1-B-4.
- SX1276 register-level analysis for rawest possible bit access.
- Existing RP2350 PIO protocol examples (Manchester, sync word, bit-bang radios, IRIG, etc.).
- Power / timing / range trade-offs of FSK vs LoRa for a software PL.

---

## 3. JPL User Terminal (flying on Firefly Blue Ghost Mission 2) – S-band Proximity-1 Details

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30  
**Updated by:** Grok 4.3 (Build CLI) — 2026-05-30 (corrected attribution per user feedback: this is JPL-developed hardware/protocol work, not Firefly's)

**Important Note on Attribution:** The S-band Proximity-1 implementation in question is part of the **JPL User Terminal** (a JPL-managed SDR payload developed with Vulcan Wireless). It is flying as a payload on Firefly Aerospace’s Blue Ghost Mission 2 lander. It is not a "Firefly protocol."

### 3.1 Confirmed Context
- JPL User Terminal (compact S-band SDR + antenna) will be used to commission ESA Lunar Pathfinder using the updated lunar S-band version of CCSDS Proximity-1 (profile specified ~2024).
- This represents the first flight demonstration of that specific S-band Prox-1 variant.
- Primary near-term use: surface-to-orbit relay for far-side assets (including LuSEE-Night).

### 3.2 Best Public Technical Sources (as of 2026-05-30)

- **NASA LunaNet Interoperability Specification v5 Baseline (2025)**: https://www.nasa.gov/wp-content/uploads/2025/02/lunanet-interoperability-specification-v5-baseline.pdf
  - Contains detailed physical layer profiles (PFS1a/b/etc.) for lunar S-band Proximity-1.
  - Waveforms: BPSK, PCM/PM, PCM/PSK/PM, GMSK + PN ranging, filtered OQPSK, spread-spectrum options.
  - Coding: LDPC (multiple rates), convolutional.
  - ASM usage in the sync/coding layer.
  - Hailing and session control via Proximity-1 + CCSDS 235.1.
  - Explicitly SDR-friendly with variable rates/coding/modulation.

- **SSTL Lunar Pathfinder Service Guide V2.2**: https://www.sstl.co.uk/getmedia/edaaa697-405c-458d-8394-97f83130d54a/Lunar-Pathfinder-Service-Guide-V2-2.pdf
  - Concrete hailing procedure (carrier-only forward hailing channel addressed by SCID; only matching asset responds).
  - S-band frequencies (forward ~2025–2110 MHz, return ~2200–2290 MHz).
  - Additional modulation/coding "above the baseline Proximity-1 standard" for lunar spectrum efficiency.
  - Performance tables (EIRP, G/T, data rates from 0.5 kbps to 2 Mbps).

- **Vulcan Wireless SmallSat 2024 Paper**: "Vulcan Wireless is Heading to the Moon" (https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=5937&context=smallsat)
  - Directly describes the JPL User Terminal hardware on Blue Ghost 2 and its role in instituting the new S-band Prox-1 standard.

- JPL public pages (2025) confirm delivery and integration of the User Terminal payload.

### 3.3 Implications for Starcom

The emerging lunar S-band Prox-1 is **not** a radical departure from the classic Mars UHF version — it reuses the same Data Link concepts (COP-P style, hailing, PLCW) with updated Physical Layer profiles for spectrum and efficiency reasons. 

A well-designed Starcom library that cleanly separates Data Link (COP-1 + COP-P via USLP) from an abstract Physical Layer interface will be directly relevant to these missions once the PHY module is extended (or when users plug in a compatible S-band SDR).

No public open-source implementation of the exact lunar S-band waveform/hailing parameters has been located yet (as expected for operational missions). The LunaNet v5 + SSTL guide + CCSDS 211.x documents are the practical references for anyone wanting to build compatible equipment.

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30 (initial synthesis of available sources)

### 3.1 Known Public Context (Confirmed)

- Blue Ghost Mission 2 (Firefly + Elytra) will carry JPL User Terminal (S-band SDR).
- Goal: Commission ESA Lunar Pathfinder using S-band Proximity-1 (updated lunar profile, ~2024 specification).
- First flight heritage for this specific S-band Prox-1 variant.
- Hardware: Compact low-cost S-band software-defined radio + antenna.

### 3.2 Current Research Status on Implementation Details

Public information as of 2026-05-30 is still mostly high-level mission descriptions (JPL press releases, Firefly mission pages). Very little low-level protocol implementation detail (exact ASM patterns used, hailing parameters, managed parameters chosen, waveform specifics, or software architecture) has surfaced in open sources yet.

**Promising leads to investigate further:**
- JPL CLPS / User Terminal technical papers or conference presentations (2024–2026).
- Any open-source components from Vulcan Wireless (SDR partner mentioned).
- CCSDS working group materials or presentations on the lunar S-band Prox-1 profile.
- Firefly or JPL GitHub / open data releases post-launch (if any).

This is valuable context for Starcom: even "short range" lunar relay missions are adopting (updated) Proximity-1 at the Data Link + Physical layer. A library that cleanly supports both COP-1 and COP-P will have immediate relevance.

---

## 4. Other Research Threads (Broad Starcom Vision)

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30

(Sections to be expanded with external research on prior-art libraries, modern C++ patterns for spec-faithful state machines, licensing/packaging for standalone release, etc.)

- Prior-art deep analysis (OSDLP, NASA cFS, CCSDSPack, etc.)
- C++17+ library craft (exceptionless design, constexpr state tables, IRadio abstraction, conformance testing)
- AO integration patterns (light reference only from Rocket-Chip QP usage)
- ...

---

## Appendix: Key External Sources & Links

**Written by:** Grok 4.3 (Build CLI) — 2026-05-30

- CCSDS 232.1-B-2 (COP-1): https://ccsds.org/Pubs/232x1b2e2c1.pdf
- CCSDS 211.0-B-6 (Prox-1 Data Link): https://ccsds.org/Pubs/211x0b6e1.pdf
- CCSDS 211.1-B-4 (Prox-1 Physical): https://ccsds.org/Pubs/211x1b4e1.pdf
- CCSDS 732.1-B-3 (USLP): https://ccsds.org/Pubs/732x1b3e1.pdf
- Blue Ghost Mission 2 / JPL User Terminal references (JPL, Firefly sites)

---

*End of initial skeleton. This document will grow through continued external research.*