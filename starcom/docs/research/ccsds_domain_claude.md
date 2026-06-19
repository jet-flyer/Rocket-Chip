# Starcom CCSDS Library — Independent External Research

**Written by:** Claude — 2026-05-31
**Scope:** Independent external research (conducted without reading the prior STARCOM_CCSDS_LIBRARY_RESEARCH.md, to avoid bias).

**Verification status:** Findings were adversarially fact-checked against primary sources in two passes (initial multi-agent run + a focused re-verification of the load-bearing claims). **0 claims were refuted.** A handful needed nuance; those corrections are folded inline and flagged with *"Verification correction (re-verify 2026-05-31)"* call-outs. The most consequential correction: **Proximity-1 has no Operational Control Field — the PLCW is a supervisory-channel SPDU, not a 32-bit OCF payload** (see §6.4).

---

## 1. Executive Summary / Verdict

**Question:** Can the named candidate hardware — a Raspberry Pi RP2350 microcontroller plus a Semtech SX1276 / HopeRF RFM95W LoRa transceiver operating in the ~915 MHz ISM band, with RP2350 PIO assistance — implement the CCSDS **211.1-B-4** Proximity-1 *Physical Layer*?

**Short, honest answer: No. It cannot, and the failure is fundamental — not a matter of firmware effort.**

[CCSDS 211.1-B-4](https://ccsds.org/Pubs/211x1b4e1.pdf) (Proximity-1 Space Link Protocol — Physical Layer, December 2013, with Editorial Change 1, January 2018) mandates a single, specific RF waveform: PCM data **Bi-Phase-L (Manchester) encoded and phase-modulated directly onto a residual carrier at a modulation index of 60° ± 5%** (§3.3.5), Right-Hand Circular Polarization on both links (§3.3.4), in the **UHF Mars bands** (forward 435–450 MHz, return 390–405 MHz; §3.3.2.2), with tight oscillator stability and optional coherent transponder turnaround for ranging/Doppler (§3.4).

The SX1276/RFM95W modem is fixed in silicon to **LoRa CSS, (G)FSK, (G)MSK, and OOK** only. It has no phase-modulation / residual-carrier path and no arbitrary-I/Q baseband input, so it physically cannot emit the mandated 60°-index residual-carrier Bi-Phase-L waveform regardless of firmware. The RP2350 PIO can shape *digital* bit/symbol timing and line coding (Manchester/Bi-Phase-L bit shaping, framing, CRC), but PIO drives only digital I/O — it cannot change the transceiver's hardwired analog RF modulator. Separately, **915 MHz is the 902–928 MHz ISM band, which is not a CCSDS-authorized Proximity-1 band** at all.

**Where Starcom's real value lands:** one layer up. The CCSDS **211.2-B-3** Coding & Synchronization Sublayer (PLTU framing, the 24-bit ASM = hex `FAF320`, CRC-32, convolutional/LDPC coding, idle data, frame sync) and the **USLP / COP-1 / COP-P** Data Link machinery are *digital and transport-agnostic*. A standards-faithful Starcom library can implement all of that over any COTS link — including a 915 MHz LoRa/FSK bearer — and that is exactly where a software library should focus. The honest framing: **this hardware can run a Proximity-1-inspired data-link stack over a COTS 915 MHz link, but it is not, and cannot be, a 211.1-B-4-compliant Physical Layer.**

A genuine 211.1-B-4 waveform is achievable on a research/HPR budget only with a general-purpose **SDR** (e.g., ADALM-PLUTO) or a dedicated **VHDL modem IP core** (e.g., ComBlock COM-1852SOFT), where the modulation is synthesized in software/FPGA — the same architectural approach used by JPL's flight radios.

---

## 2. CCSDS 211.1-B-4 Physical Layer — Exact Requirements and the Layer Boundary

### 2.1 Document identity

[CCSDS 211.1-B-4](https://ccsds.org/Pubs/211x1b4e1.pdf), *Proximity-1 Space Link Protocol — Physical Layer*, Blue Book, **Issue 4, December 2013**, with **Editorial Change 1 (January 2018)** (which repaired a broken cross-reference and improved the Table-of-Contents formatting). It superseded 211.1-B-3 (March 2006), B-2 (May 2004), B-1 (April 2003), and the original combined 211.0-B-1 (October 2002). It is also published as **ISO 21460:2015**. The technical specification (Section 3) is deliberately thin — roughly 11 pages.

### 2.2 Scope: UHF / Mars only

Per §1.2: *"Currently, the Physical Layer only defines operations at UHF frequencies for the Mars environment."* §3.3.3 reinforces: *"Other frequency bands are intentionally left unspecified until a user need for them is identified."* A full-text search of the Blue Book returns **zero** mentions of S-band or any 2 GHz frequency. Any S-band Proximity-1 operation (see §7) is outside the 211.1-B-4 PHY.

### 2.3 Frequency bands and channel assignments (PHY-owned)

| Parameter | Value (211.1-B-4) |
|---|---|
| Total UHF range | 60 MHz spanning 390–450 MHz, with a 30 MHz guard-band (§3.3.2.1) |
| Forward band (relay/orbiter → surface) | 435–450 MHz (§3.3.2.2.1) |
| Return band (surface → relay/orbiter) | 390–405 MHz (§3.3.2.2.2) |
| Default hailing channel | **Channel 1**: forward 435.6 MHz / return 404.4 MHz, turnaround ratio 1348/(44×33) (§3.3.2.3.1) |
| Legacy fixed channel | **Channel 0**: 437.1 / 401.585625 MHz, turnaround ratio 147/160 (first-generation Mars transceivers, e.g., MRO/Odyssey ↔ landers) |
| Other tabulated channels | Ch2 439.2/397.5 (1325/(24×61)); Ch3 444.6/393.9 (1313/(38×39)); Ch4–7 selectable within the bands |
| Multi-frequency selection | 20 kHz steps; **Channels 8–15** reserved by CCSDS, enabled via the `SET PL EXTENSIONS` directive defined in the Data Link Layer book (211.0-B) Annex A (§3.3.2.5) |

**Nuance (corrected per verification):** the channels reserved for the 20 kHz-step multi-frequency mechanism are **8–15**, not "4–15." Channels 0–7 are fixed single forward/return pairs. Also, the warning that frequencies near 430 MHz cannot be used in the vicinity of Earth lives in the descriptive §3.3.1 (Background) — it is *not* a flat "ITU-reserved / prohibited" normative ban; the standard states these frequencies are designed to avoid interference with ITU-allocated services and that near-Earth use would require defining *different* frequencies in strict compliance with the ITU Radio Regulations. Frequency selection is subject to SFCG recommendations.

### 2.4 Modulation (PHY-owned — the load-bearing requirement)

- §3.3.5.1: *"The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier."*
- §3.3.5.2: *"Residual carrier shall be provided with modulation index of 60° ± 5%."*
- §3.3.5.3: Bi-Phase-L mark-to-space symmetry between 0.98 and 1.02.
- §3.3.5.4: a symbol `1` advances the RF carrier phase at the start of the symbol interval; a symbol `0` delays it.
- §3.3.4: both links use **Right-Hand Circular Polarization (RHCP)**.

This is a **residual-carrier phase-modulation (PCM/PM with Bi-Phase-L)** waveform. There is **no subcarrier**, no GMSK/OQPSK/8-PSK option, and no suppressed-carrier mode anywhere in the Blue Book. Annex A PICS Table A-1 lists *Modulation* (ref §3.3.5) as Status **M** (mandatory) with the only allowed value `Bi-Phase-L`, and *Polarization* (ref §3.3.4) as RHCP. (Note: GMSK/OQPSK/8-PSK appear only in the rationale Green Book CCSDS 210.0-G as *future* candidates, not in the normative 211.1-B-4.)

### 2.5 Rates, stability, and RF performance (PHY-owned)

- **Coded symbol rates (Rcs)** — 13 discrete values (§3.3.6.1): 1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000, 512000, 1024000, 2048000, 4096000 sym/s. Support for at least one is mandatory (PICS). Because Bi-Phase-L maps one coded symbol to one channel symbol, the channel symbol rate equals the coded symbol rate (Rchs = Rcs); the standard ties this equality to the §3.3.5.1 modulation scheme — not to any "1:1 line code" property (Bi-Phase-L is in fact ~2× the NRZ bandwidth with two transitions per symbol).
- **Oscillator stability**: long-term 10 ppm over mission life (§3.4.1.1); short-term 1 ppm over 1 minute (§3.4.1.2).
- **Symbol-rate stability**: channel symbol period within 1% of nominal (§3.3.6.2); rate offset < 0.1% measured over > 10000 symbol periods (§3.3.6.3).
- **Residual AM**: < 2% RMS of the phase-modulated signal (§3.4.2).
- **Spectral purity templates**: Figure 3-2 is the oscillator phase-noise template (non-coherent mode, normalized at 437.1 MHz); Figure 3-3 is the discrete-spurious-spectral-line template (normalized by A = 2·Rcs, the factor of 2 owing to Bi-Phase-L). No absolute EIRP or hard dBm emission mask is mandated; link budgets are left to mission MoAs.

### 2.6 Coherency, Doppler, ranging (PHY-owned)

Radio categories (Table 3-1): **E1** (transmit-only), **E2n** (non-coherent), **E2c** (coherent transmit/receive with channel turnaround ratio, for range/range-rate), **E2d** (descoped microprobe class — *receives* FSK but *transmits* PSK; not required for cross-support). Doppler frequency range ±10 kHz; Doppler rate 100 Hz/s non-coherent, 200 Hz/s coherent (§3.4.5.1). Coherent ranging uses the per-channel turnaround ratio.

### 2.7 State model and signaling (PHY-owned)

Duplex, half-duplex, and simplex are all supported (§1.5.1.2). The PHY accepts four control variables from the MAC Sublayer — MODE, DUPLEX, TRANSMIT, MODULATION (§3.2.1.1). Transmit is **state-driven** (transmitter state, Table 3-2, is governed by MODE/TRANSMIT/MODULATION); **DUPLEX drives the receiver state** (Table 3-3 — e.g., in half-duplex the receiver is off while TRANSMIT is on). Receive is **modeless** once the receiver is on (§2.1). The receiver sweeps its assigned channel to acquire carrier lock (§3.2.3.2) and signals **CARRIER_ACQUIRED** (RF lock) and **SYMBOL_INLOCK_STATUS** (symbol lock) up to the MAC (§3.2.3.3/3.2.3.4).

### 2.8 The critical boundary: 211.1-B-4 PHY vs. 211.2-B-3 Coding & Synchronization Sublayer

211.1-B-4 keeps coding and sync **out** of the Physical Layer by delegation: the PHY *"accepts a coded symbols stream from the Coding & Synchronization Sublayer (reference [2]) … for modulation onto the radiated carrier"* (§2.1), and that stream *"contain[s] Proximity Link Transmission Units (PLTUs) and Idle data"* (§2.2). The detailed coding/sync content is **defined in** [CCSDS 211.2-B-3](https://ccsds.org/Pubs/211x2b3.pdf) (October 2019), the C&S Sublayer Blue Book — a sublayer of the Data Link Layer, not the PHY.

The one subtle exception worth stating precisely: **the Bi-Phase-L (Manchester) line code is applied by the Physical Layer (211.1-B-4 §3.3.5.1)**, downstream of the C&S encoder, immediately before the RF modulator (Figure 1-1). The C&S Sublayer's output is the *coded* symbol stream (post convolutional/LDPC/uncoded); the channel coding is C&S, the Manchester symbol formatting is PHY.

#### Feature → owning document map

| Feature | Owning document | Reference |
|---|---|---|
| RF carrier, residual-carrier PM | **211.1-B-4 (PHY)** | §3.3.5 |
| Bi-Phase-L (Manchester) line code / symbol formatting | **211.1-B-4 (PHY)** | §3.3.5.1, Fig 1-1 |
| Modulation index (60° ± 5%) | **211.1-B-4 (PHY)** | §3.3.5.2 |
| UHF frequency bands & channel/hailing assignments | **211.1-B-4 (PHY)** | §3.3.2 |
| RHCP polarization | **211.1-B-4 (PHY)** | §3.3.4 |
| Coded symbol rates (Rcs), oscillator/symbol-rate stability | **211.1-B-4 (PHY)** | §3.3.6, §3.4.1 |
| Ranging/Doppler, coherency, turnaround ratios | **211.1-B-4 (PHY)** | §3.1.2, §3.4.5 |
| Carrier/symbol lock signaling, sweep | **211.1-B-4 (PHY)** | §3.2.3 |
| 24-bit ASM = `FAF320` | **211.2-B-3 (C&S)** | §3.2.3 |
| PLTU structure (ASM + Transfer Frame + CRC-32) | **211.2-B-3 (C&S)** | §3.2.2 |
| CRC-32 (G(X)=X³²+X²³+X²¹+X¹¹+X²+1) | **211.2-B-3 (C&S)** | §3.2.5, Annex C |
| Convolutional code (rate-1/2, K=7) | **211.2-B-3 (C&S)** → CCSDS 131.0-B-3 | §3.4.3 |
| LDPC code (n=2048, k=1024, rate-1/2) | **211.2-B-3 (C&S)** → CCSDS 131.0-B-3 | §3.4.4 |
| LDPC Codeword Sync Marker (CSM, 64-bit `034776C7272895B0`) | **211.2-B-3 (C&S)** | §3.4.4.6 |
| LDPC codeword randomization (h(x)=x⁸+x⁶+x⁴+x³+x²+x+1) | **211.2-B-3 (C&S)** | §3.4.5 |
| Idle/Acquisition/Tail data (PN `352EF853`) | **211.2-B-3 (C&S)** | §3.3 |
| Frame synchronization (ASM search) | **211.2-B-3 (C&S)** | §3.6.3 / §3.6.4 |
| Transfer Frame (Version-3), MAC sublayer, directives, MIB | **211.0-B-6 (Data Link)** | — |
| USLP (Version-4) Transfer Frame | **732.1-B-3** | — |
| COP-1 procedures (FOP-1/FARM-1) | **232.1-B-2** | — |
| CLCW bit format | **232.0-B-4** | §4.2 |
| COP-P, FOP-P, FARM-P, PLCW | **211.0-B-6** | §7 |

The interface between C&S and PHY is the coded-symbol stream delivered at a constant rate Rcs (211.1-B-4 §2.1; 211.2-B-3 §2.4).

---

## 3. Hardware Feasibility: RP2350 + SX1276/RFM95W (915 MHz) + PIO vs. 211.1-B-4

### 3.1 What the hardware actually is

- **RP2350** (Raspberry Pi, product brief Aug 2024): dual Arm Cortex-M33 *or* dual Hazard3 RISC-V @ 150 MHz; **3 PIO blocks (12 state machines)**; 16-channel DMA. PIO state machines are clocked from sysclk through a fractional divider and feed/drain via TX/RX FIFOs (DMA-fed). Excellent for deterministic digital I/O and line coding.
- **SX1276 / RFM95W** (Semtech datasheet Rev. 7, May 2020): "137 MHz to 1020 MHz Low Power Long Range Transceiver." Modulation set: **LoRa CSS, FSK, GFSK, MSK, GMSK, OOK** — the block diagram shows exactly two TX modulator paths (LoRa and FSK/OOK), with **no PSK/PM or arbitrary-I/Q path**. FSK is SPI-programmable (deviation 0.6–200 kHz, bit rate 1.2–300 kbps, 32 MHz FXOSC, ~61 Hz PLL resolution), but programmability only configures the *fixed* modems — it cannot synthesize a new modulation topology.

### 3.2 Requirement-by-requirement verdict

| 211.1-B-4 requirement | Status on RP2350 + SX1276 @ 915 MHz | Why |
|---|---|---|
| Bi-Phase-L residual-carrier PM, 60° ± 5% index (§3.3.5) | **UNMEETABLE** | SX1276 has no phase-modulation path; only LoRa/(G)FSK/(G)MSK/OOK. No firmware can add a PM modulator. |
| UHF band 435–450 / 390–405 MHz (§3.3.2.2) | **UNMEETABLE (as configured)** | Device runs in the 902–928 MHz ISM band. The chip's PLL *can* tune 390–450 MHz, but COTS RFM95W modules are band-filtered for 868/915 MHz, and the waveform is wrong there anyway. |
| RHCP polarization (§3.3.4) | **UNMEETABLE** | Antenna-domain requirement; not a transceiver function (and typical COTS antennas are linear). |
| Coherent transponder / ranging turnaround (§3.1.2, §3.4.5) | **UNMEETABLE** | SX1276 has no coherent transponder or ranging capability. |
| Oscillator 10 ppm / 1 ppm, residual AM < 2% (§3.4.1/3.4.2) | **PARTIAL / UNVERIFIED** | Module crystals/TCXOs are typically ±2–10 ppm but not specified as a coherent reference to this regime; moot given the modulation blocker. |
| Coded symbol rates & symbol-clock stability (§3.3.6) | **PARTIAL (digital only, moot)** | RP2350 PIO fractional divider can generate the symbol clocks precisely; but SX1276 FSK tops out ~300 kbps, so Rcs ≥ 512 ksym/s is unreachable, and rate compliance is irrelevant without the correct modulation. |

### 3.3 What the RP2350 PIO can and cannot do

**Can (digital domain):** generate Bi-Phase-L/Manchester bit timing, drive precise symbol clocking, build PLTUs (ASM + frame + CRC-32), correlate the ASM on receive, run idle/acquisition/tail sequence generation, time-tag, and manage half-duplex turnaround timing. PIO is well-suited to feeding a digital data/clock pin into the transceiver.

**Cannot (analog domain):** change the SX1276's hardwired analog RF modulator. PIO outputs digital logic levels; the chip then re-modulates per its fixed FSK/OOK/LoRa path. To produce a true 211.1-B-4 residual-carrier PM waveform you need an I/Q modulator / DAC + upconverter (an SDR) where firmware defines the carrier phase trajectory — **the gap is the SX1276 modem, not the RP2350.**

### 3.4 Where Starcom's value actually sits

The framing, coding, and ARQ are digital and transport-agnostic:

- **211.2-B-3 C&S Sublayer**: PLTU = 24-bit ASM (`FAF320`) + Transfer Frame + CRC-32; idle PN `352EF853`; convolutional/LDPC coding + CSM; ASM frame sync.
- **Data Link**: native Version-3 Proximity-1 frames (211.0-B-6) or Version-4 **USLP** frames (732.1-B-3), carrying **COP-1** (CLCW) and **COP-P** (PLCW) ARQ.

All of this is pure digital logic that RP2350 + library code can execute over **any** COTS link. Starcom can therefore deliver a standards-faithful PLTU/USLP/COP transport over a non-CCSDS-compliant 915 MHz LoRa/FSK bearer — useful for telemetry/HPR/cubesat link experiments — provided the documentation is honest that the *Physical Layer* is not 211.1-B-4-compliant.

---

## 4. Survey: COTS Non-Space-Qualified Radios That Can Emit a Full CCSDS PHY

The decisive question is whether a part can produce the **residual-carrier PCM/PM Bi-Phase-L waveform at a 60° index** (UHF Prox-1), or the residual-carrier PCM/PM/SP-L waveform of the S-band variant. Almost no *fixed-modem* COTS chip can. "CCSDS-compliant" on most cubesat radio datasheets refers to TM/TC **framing and FEC**, not the Proximity-1 PHY waveform.

| Vendor / Part # | Capability vs. CCSDS PHY | Form factor | USD price (approx.) | Practicality verdict |
|---|---|---|---|---|
| **Semtech SX1276 / HopeRF RFM95W** | Band-reachable (137–1020 MHz) but **fixed LoRa/(G)FSK/(G)MSK/OOK** — no PM/residual-carrier. **Cannot** emit Prox-1 waveform. | IC / ~1″×0.6″ module | IC ~$3–6; RFM95W module ~$8–15; Adafruit RFM95W breakout (PID 3072) ~$20 | **Not viable** for the PHY; fine as a COTS *bearer* for upper-layer Starcom logic. |
| **TI CC1200** | Bands 164–190 / 410–475 / 820–950 MHz (435–450 reachable) but **2/4-(G)FSK, MSK, OOK only** — no PM, no S-band. | QFN IC | ~$5–9 | **Not viable** for the waveform; band-correct but modulation-wrong. |
| **Silicon Labs Si446x** | Sub-GHz (G)FSK/OOK; same blocker — no residual-carrier PM. | QFN IC | ~$2–5 | **Not viable** for the waveform. |
| **GomSpace NanoCom AX100** | 430–440 MHz, **GFSK/GMSK**, 0.1–38.4 kbps. In-band UHF but not Prox-1 PM. | CubeSat module | Quote-only | **Not viable** for the waveform; it is a TT&C packet radio. |
| **EnduroSat UHF Transceiver II** | 400–403 / 435–438 MHz, **2/4-(G)FSK/GMSK/OOK**, up to ~19.6 kbps. | CubeSat module | Quote-only | **Not viable** for the waveform. |
| **ISIS TRXVU** | VHF up / UHF down, **BPSK(G3RUH)/GMSK**. | CubeSat module | ~€8,500 | **Not viable** for the waveform. |
| **NanoAvionics SatCOM UHF** | UHF digital transceiver, **(G)FSK/GMSK**. | CubeSat module | from ~€6,350 | **Not viable** for the waveform. |
| **Syrlinks EWC31-class S-band transponder** | **Genuinely CCSDS-compliant for RF/Modulation/Coding**: PCM/PM/**SP-L** (Split-Phase-L = Bi-Phase-L = Manchester) uplink **8–256 kbps**, **coherent transmission + ranging**, in the 2 GHz space TT&C bands 2025–2110 (Rx) / 2200–2290 MHz (Tx) — bands *shared* by the Space Operation, Space Research, and Earth-Exploration-Satellite services. | Space-grade unit | Quote-only (tens of $k) | **Compliant but space-grade** — S-band TT&C, not UHF Mars Prox; out of HPR/hobby budget. |

**Bottom line for §4:** no low-cost, fixed-modem COTS radio can emit a 211.1-B-4 residual-carrier Bi-Phase-L/PM waveform. The only COTS parts that produce a true CCSDS PCM/PM/SP-L emission with ranging are space-grade S-band transponders (Syrlinks-class), which are quote-only and expensive — and they are S-band TT&C, not the UHF Mars Prox-1 band of 211.1-B-4.

---

## 5. Survey: SDR Platforms and FPGA IP Cores for a Full CCSDS PHY

Because the SDR/FPGA path generates the waveform in software/firmware, it *can* synthesize the residual-carrier Bi-Phase-L/PM waveform — this is how JPL's flight radios do it (JPL **Electra**, an FPGA-based SDR at 390–450 MHz, transmits BPSK in carrier-only, suppressed-carrier NRZ, or **residual-carrier Manchester** modes — the last being exactly 211.1-B-4 §3.3.5).

| Product | Capability | USD price / tier | Practicality |
|---|---|---|---|
| **ComBlock COM-1852SOFT** (VHDL Prox-1 modem IP core) | **Explicitly 211.1-B-4-compliant**: Bi-Phase-L + 60° residual-carrier PM, all 13 rates (1k–4.096M), Doppler 200 Hz/s; TX/RX; outputs I/Q to any DAC/SDR; ~10–27% of an Artix-7-100T; royalty-free bitstream license. | ~$2,500 (core) + FPGA/DAC | **Best low-cost faithful PHY.** You still supply an FPGA carrier + RF front-end (UHF PA/filter/RHCP antenna). |
| **ADALM-PLUTO / Pluto+ (AD9363/AD9361 SDR)** | AD9363, factory-tunable 325 MHz–3.8 GHz (firmware-"hackable" to emulate AD9364 → 70 MHz–6 GHz); covers UHF Prox (native, no hack) + S-band; **waveform fully software-defined**, so residual-carrier Bi-Phase-L/PM at 60° is implementable in GNU Radio/MATLAB. | genuine ADALM-PLUTO ~$186–230; Pluto+ clones ~$150–260 *(corrected re-verify 2026-05-31 — the earlier "$99" floor reflected only historical launch/student promo or bare clone boards, not current retail)* | **Best low-cost path to a real waveform.** Needs external UHF PA, filtering, RHCP antenna, and a high-stability reference (stock TCXO ±25 ppm; needs ~0.5 ppm VCTCXO or a 10 MHz GPSDO/OCXO) to meet the 10 ppm/1 ppm requirement; coherent ranging is extra DSP work. |
| **bladeRF 2.0 micro xA4** | 47 MHz–6 GHz, full-duplex; covers UHF + S-band. | ~$720 | Viable; more RF performance than Pluto. RF front-end still external. |
| **LimeSDR Mini / USB** | Wideband; covers 390–450 MHz. | ~$399 / ~$625 | Viable; community DSP support. |
| **Ettus USRP B200 / B205mini** | 70 MHz–6 GHz, best RF of the hobby class. | ~$1,460–1,576 | Viable; strong UHD/GNU Radio ecosystem. |
| **HackRF One** | 1 MHz–6 GHz but **half-duplex**; discontinued in 2025. | ~$299 | Limited (half-duplex); usable for one-way experiments. |
| **AD-FMCOMMS3-EBZ (AD9361) on Zynq carrier** | 70 MHz–6 GHz, arbitrary modulation incl. Prox-1 waveform. | board few-hundred to ~$1k + FPGA carrier (ZC706/ZedBoard) | Lab-grade; more integration effort. |
| **EVAL-ADRV9002 on FPGA motherboard** | 30 MHz–6 GHz, narrow/wideband; arbitrary modulation. | ~$1,920–3,770 radio card + FPGA motherboard | Best RF, costliest; overkill for HPR. |

**Bottom line for §5:** the **ComBlock COM-1852SOFT** is the most direct route to a *labeled-compliant* 211.1-B-4 modem; the **ADALM-PLUTO** is the cheapest route to *generating* the waveform from scratch. Either way you supply the RF front end (UHF PA, filtering, RHCP antenna, stable reference). All SDR figures are RF transceiver only.

---

## 6. Supporting Standards Detail (Data Link Layer)

All revisions below were verified from the primary Blue Book PDFs on ccsds.org.

### 6.1 USLP — CCSDS 732.1-B-3 (June 2024)

**Correction to any stale recollection:** the current USLP issue is **732.1-B-3 (June 2024)** — it supersedes 732.1-B-2 (Oct 2021) and B-1 (Oct 2018). Do **not** cite B-2 as latest.

USLP non-truncated Transfer Frame Primary Header (§4.1.2), bit 0 = MSB = first transmitted:

| Field | Bits | Notes |
|---|---|---|
| TFVN | 4 | Transfer Frame Version Number (= 0b1100 / Version-4) |
| SCID | 16 | Spacecraft ID |
| Source-or-Destination ID | 1 | 0 = SCID is source; 1 = SCID is destination |
| VCID | 6 | Virtual Channel ID (0–62; 63 reserved) |
| MAP ID | 4 | up to 16 MAP Channels per VC |
| End-of-Frame-Primary-Header Flag | 1 | 1 = truncated header (first 6 fields only) |
| Frame Length | 16 | value C = total octets − 1 (max 65536 octets) |
| Bypass/Sequence Control Flag | 1 | 0 = Sequence-Controlled (FARM checks); 1 = Expedited (bypass) |
| Protocol Control Command Flag | 1 | — |
| Reserve Spares | 2 | — |
| OCF Flag | 1 | signals presence of the 4-octet OCF |
| VCF Count Length | 3 | — |
| VC Frame Count | 0–56 (variable) | — |

Frame components (§4.1.1): Primary Header (4–14 octets) + optional Insert Zone + mandatory Transfer Frame Data Field + optional OCF (4 octets) + optional FECF (2 octets, CRC-16). Multiplexing hierarchy: Physical Channel ⊃ Master Channel (MCID = TFVN+SCID) ⊃ Virtual Channel (GVCID = MCID+VCID) ⊃ MAP Channel (GMAP ID = GVCID+MAP ID).

**USLP carries both COP-1 and COP-P transparently**, per VC, via generic FOP/FARM abstractions; QoS is selected per frame by the Bypass/Sequence Control Flag (§2.1.6/2.2.3). USLP does **not** itself specify the COP procedures — references [9]=232.1-B (COP-1) and [10]=211.0-B (COP-P) are normative.

### 6.2 COP-1 + CLCW

- **Procedures** — **CCSDS 232.1-B-2 (September 2010, incorporating Technical Corrigendum 1, April 2019)**. Closed-loop Go-Back-N ARQ. **FOP-1** (sending) has 6 states: S1 Active, S2 Retransmit-without-Wait, S3 Retransmit-with-Wait, S4 Initialising-without-BC-Frame, S5 Initialising-with-BC-Frame, S6 Initial. **FARM-1** (receiving) has 3 states: S1 Open, S2 Wait, S3 Lockout. Service types: Type-A (AD sequence-controlled user data; BC sequence-controlled control commands `Unlock`/`Set V(R)`) and Type-B (BD expedited, bypasses FARM checks). 8-bit Frame Sequence Number; FOP arithmetic mod-256.
- **CLCW bit format** — defined **NOT** in the COP-1 doc but in the **TC Space Data Link Protocol, CCSDS 232.0-B-4 (incorporating Technical Corrigendum 1, October 2023), §4.2.1**:

| Bits | Field | Value/meaning |
|---|---|---|
| 0 | Control Word Type | `0` |
| 1–2 | CLCW Version Number | `00` (Version-1) |
| 3–5 | Status Field | mission-specified |
| 6–7 | COP in Effect | `01` for COP-1 |
| 8–13 | Virtual Channel ID | 6 bits |
| 14–15 | Reserved Spare | `00` |
| 16 | No RF Available Flag | 1 = no RF |
| 17 | No Bit Lock Flag | — |
| 18 | Lockout Flag | — |
| 19 | Wait Flag | — |
| 20 | Retransmit Flag | — |
| 21–22 | FARM-B Counter | 2 LSBs |
| 23 | Reserved Spare | `0` |
| 24–31 | Report Value | N(R) = next expected Frame Sequence Number = FARM V(R) |

For Starcom: cite **232.0-B** for the CLCW field format and TC frame format; cite **232.1-B** for the procedures/state machines.

### 6.3 COP-P + PLCW (Proximity-1)

COP-P, FOP-P, FARM-P, and the PLCW are all defined in the **Proximity-1 Space Link Protocol — Data Link Layer, CCSDS 211.0-B-6 (July 2020)** (supersedes 211.0-B-5, Dec 2013). COP-P lives in the Data Services Sublayer (§7): FOP-P (§7.2) sending, FARM-P (§7.3) receiving; it provides Sequence-Controlled (Go-Back-N ARQ, report = PLCW) and Expedited QoS.

The **PLCW is a 16-bit fixed-length SPDU** comprising **seven contiguous fields** (§3.2.4.3.2.1.1, Fig 3-5) — structurally completely different from the 32-bit CLCW. Fields: Report Value 8 bits (bits 8–15) = V(R); Expedited Frame Counter 3 bits (bits 5–7, mod-8); Reserved Spare 1 bit (bit 4, set to `0`); PCID 1 bit (bit 3, selects one of two redundant receivers); Retransmit Flag 1 bit (bit 2); SPDU Type Identifier 1 bit (bit 1, `0`); SPDU Format ID 1 bit (bit 0, `1`). **Key differences vs. CLCW:** PLCW is 16 bits not 32; has **no** No-RF/No-Bit-Lock/Lockout/Wait flags and **no** COP-in-Effect/Version/Status/VCID fields; uses PCID + a 3-bit mod-8 Expedited Frame Counter instead of the CLCW's 2-bit FARM-B counter. Both carry an 8-bit Report Value = V(R) and a Retransmit flag. PLCW is transmitted only with Expedited QoS, on the supervisory channel (not in a Transfer-Frame OCF — see §6.4).

> **Verification correction (re-verify 2026-05-31):** the original draft listed six PLCW fields; the standard enumerates **seven** — the 1-bit Reserved Spare (bit 4) was added above. Verified against CCSDS 211.0-B-6 §3.2.4.3.2.1.1.

Proximity-1 session model (§6): session establishment → data services (with optional resynchronization / reconnect subphases) → session termination. **Hailing** establishes a link over the forward/return hailing-channel pair (not used in simplex). **Resynchronization** forces the responder to readjust Sequence-Controlled frame numbers via the `Set V(R)` directive (carried by a Type-1 SPDU); **reconnect** rehails mid-session without resetting FOP-P/FARM-P counters.

### 6.4 OCF (Operational Control Field) — and why Proximity-1 has none

In the **TM, AOS, and USLP** Space Data Link Protocols, the OCF is a **4-octet (32-bit) optional Transfer-Frame trailer field**, signaled by the **OCF Flag** in the frame primary header (TM 132.0-B-3 §4.1.5, §4.1.2.4; USLP 732.1-B-3 §4.1.1, USLP_MC_OCF Service). When it carries a **Type-1 Report** it holds the 32-bit **CLCW** for COP-1 status reporting on the return link (it may alternatively carry a Type-2 Report, e.g., an SDLS Frame Security Report). Its 32-bit width matches the CLCW because both are exactly 32 bits.

> **Verification correction (re-verify 2026-05-31) — important:** the original draft said the OCF carries "the CLCW (or, for Proximity-1, the PLCW)." **That is wrong.** **Proximity-1 (CCSDS 211.0-B) defines no Operational Control Field at all.** The PLCW is **not** carried in an OCF and is **not** 32 bits — it is the 16-bit fixed-length SPDU of §6.3, sent on the **supervisory channel** with Expedited QoS, not appended to a Transfer-Frame trailer and not signaled by any OCF Flag. So the OCF↔CLCW width identity holds **only** for the TM/AOS/USLP CLCW; it does **not** extend to the Proximity-1 PLCW. For Starcom this means COP-1 return-link reporting (CLCW-in-OCF) and COP-P return-link reporting (PLCW-as-SPDU) are **two distinct mechanisms**, not one parameterized field — a real design implication, not a cosmetic note.

### 6.5 Revision roll-up (verified)

| Document | Current issue | Date |
|---|---|---|
| Proximity-1 Physical Layer | **CCSDS 211.1-B-4** (+ EC1 Jan 2018) | Dec 2013 |
| Proximity-1 Coding & Synchronization Sublayer | **CCSDS 211.2-B-3** | Oct 2019 |
| Proximity-1 Data Link Layer (COP-P, PLCW) | **CCSDS 211.0-B-6** | Jul 2020 |
| USLP | **CCSDS 732.1-B-3** | Jun 2024 |
| COP-1 procedures | **CCSDS 232.1-B-2** (+ TC1 Apr 2019) | Sep 2010 |
| TC Space Data Link Protocol (CLCW format) | **CCSDS 232.0-B-4** (+ TC1 Oct 2023) | — |
| Convolutional/LDPC code definitions | **CCSDS 131.0-B-3** | Sep 2017 |

**Decision point for Starcom:** whether to use USLP Version-4 frames over Proximity-1 (211.0-B-6 / 732.1 annexes) versus native Version-3 Proximity-1 frames affects which integrity check applies — the USLP **FECF (CRC-16)** vs. the PLTU's **CRC-32** — and should be decided early.

---

## 7. Context Note: JPL User Terminal on Blue Ghost Mission 2; 915 MHz Legality

### 7.1 Attribution (corrected and made precise)

The **User Terminal** flying on Firefly's **Blue Ghost Mission 2 (BGM2)** is **JPL-led, JPL-developed hardware** — a compact software-defined radio + antenna. The radio itself was **built by Vulcan Wireless (Carlsbad, CA)** under JPL; JPL manages the project and supported development of the new S-band radio standard. **It is not "Firefly's protocol."** Firefly supplies the Blue Ghost lander and the **Elytra Dark** orbital transfer/relay vehicle that deliver **ESA's Lunar Pathfinder** relay (built by **Surrey Satellite Technology Ltd**, with ESA as anchor customer) to lunar orbit. The terminal will commission Lunar Pathfinder and later relay LuSEE-Night far-side data through it to commercial Earth ground stations. Launch is targeted for **2026** (far side).

The terminal implements a **new S-band two-way version of CCSDS Proximity-1**, needed because on the lunar far side **UHF is protected for radio astronomy** (the Shielded Zone of the Moon), so the Mars UHF Prox-1 band cannot be reused there.

**Distinguish UHF Proximity-1 from S-band Proximity-1 throughout:** 211.1-B-4 is **UHF-only** (390–450 MHz, Mars). The S-band variant is a *separate, newer* development. JPL describes it as a new lunar standard JPL is *instituting*; CCSDS working-group traffic (SLS-SLP / SLS-CC mailing lists, Nov 2023 – May 2024) shows the S-band Prox-1 profile still under refinement at that time — S-band LDPC code selection [LDPC(2048,1024) r½, LDPC(6144,4096) r⅔, LDPC(8160,7136) r⅞], session-control parameters with TBDs, a 1 ksps hailing channel being validated, and modulation index discussion (the Lunar Pathfinder mission using a π/3 = 60° index).

> **Verification corrections (re-verify 2026-05-31):**
> 1. **"Specified in 2024" is a primary-source quote, not loose paraphrase.** JPL's own release (PIA26596) states the new lunar S-band standard "*was specified in 2024.*" The earlier draft flagged this as unverifiable — that flag was **too strong** and is withdrawn. What remains genuinely unconfirmed is narrower: whether a *ratified, publicly published CCSDS Blue Book* carrying the S-band profile existed by that date (none appears in the CCSDS public catalog), and the exact finalized S-band frequencies/EVM.
> 2. **The S-band work spans more than the Physical Layer.** The new LDPC codes belong to the **Coding & Synchronization Sublayer (211.2-B)**, and session-control parameters touch the **Data Link Layer (211.0-B)** — so it is an extension across the Proximity-1 211.x *series*, not a change confined to the 211.1-B Physical Layer. Calling it "a revision of the 211.1-B series" alone would understate its scope.

### 7.2 915 MHz legality and band status

- **915 MHz ISM is not a CCSDS-authorized Proximity-1 band.** Prox-1 PHY frequencies are UHF 390–450 MHz (211.1-B-4, Mars) or the new S-band (~2 GHz). A Starcom 915 MHz LoRa link is a COTS bearer chosen for cost/availability — it can carry CCSDS data-link/COP framing, but it is **not** operating a CCSDS-compliant Physical Layer.
- **FCC Part 15.247 (47 CFR 15.247)** governs unlicensed 902–928 MHz use. The *digital-modulation* path (§15.247(a)(2)) requires ≥ 500 kHz of 6 dB bandwidth and is capped at **1 W (30 dBm) conducted** (§15.247(b)(3)); the FHSS path (§15.247(a)(1)/(g)) caps at 1 W (≥ 50 hop channels) or 0.25 W (25–49) and has *no* 500 kHz minimum. Antenna/EIRP: at ≤ 6 dBi no reduction is required and above 6 dBi the conducted power must be reduced dB-for-dB (§15.247(b)(4); no point-to-point relaxation in 902–928 MHz — that relaxation applies only to the 2.4 GHz and 5.8 GHz bands), so EIRP is effectively pinned at ~36 dBm. **Practical caveat (corrected re-verify 2026-05-31):** many cheap LoRa configs use 125/250 kHz bandwidth (< 500 kHz). This does **not** automatically relegate them to §15.249 — narrowband LoRa is routinely certified under §15.247's **FHSS** provisions (no bandwidth floor). §15.249 (a far lower ~50 mV/m field-strength limit, ≈ −1.2 dBm EIRP) is only *one* alternative route for non-hopping narrowband devices. Operators must verify their actual 6 dB bandwidth **and** which §15.247 sub-path their radio is certified under.
- **FCC Part 97 (amateur)** allows far higher power (e.g., 33 cm 902–928 MHz and 23 cm 1240–1300 MHz up to 1500 W PEP; 70 cm 420–450 MHz overlaps the Mars Prox UHF forward band) **but 97.113(a)(4) prohibits "messages encoded for the purpose of obscuring their meaning."** Documented public codes (CCSDS framing, FEC, CRC, randomization, Manchester) are fine; **payload content must not be encrypted.** If you encrypt on 902–928 MHz you must operate as a Part 15 user, not Part 97. The 33 cm band is secondary/shared with ISM and federal radiolocation.

### 7.3 COTS pricing anchors (HPR/cubesat audience)

RP2350 silicon ~$0.80 (reel) / ~$1.10 (single); Raspberry Pi Pico 2 board ~$5; Adafruit RFM95W (SX1276) 915 MHz breakout (PID 3072) ~$20; bare RFM95W modules ~$8–15. Attractive for HPR/research budgets — but, per §3, they deliver a LoRa/(G)FSK COTS link, not a 211.1-B-4 PHY.

---

## 8. Integration Note (Light) — Framework-Agnostic Starcom as an Active Object

This section is informed by a local scout review of an existing radio Active Object (the Rocket-Chip `AO_Radio`). It is **explicitly not** a design base — the Rocket-Chip telemetry/CCSDS/RF-manager specifics are out of scope, and Starcom must be framework-agnostic. The patterns below are generic run-to-completion / AO concepts, useful as integration *seams*, kept deliberately high-level.

- **Protocol-agnostic transport seam.** A radio transport AO should move opaque byte buffers in/out and **never decode payloads**; protocol parsing (PLTU/USLP/COP) belongs in a separate consumer AO. Starcom should sit at this seam, exchanging frames, so it can plug into any radio AO without coupling.
- **Thin AO lifecycle contract.** Expose a `start(priority, queue_storage, queue_len, init_flag)` entry that constructs the AO, arms a periodic tick, zero-inits state, and starts a fixed-size event queue the library owns internally.
- **Single tick cadence + private signals.** Drive all time-sliced work (slot scheduling, retransmit timers, FOP/FARM timeouts, state evaluation) from one periodic tick, divided into per-tick sub-functions so handlers stay run-to-completion (no blocking). Define a private tick signal above the public signal range.
- **Non-blocking radio-abstraction interface.** Define the radio driver as `begin_tx()/poll_tx()` returning a tri-state (busy/done/timeout), `rx_ready()/read_rx()`, and pure config setters — never a single blocking send (a blocking LoRa airtime call would overflow other AOs' queues under a cooperative scheduler). A ~200 ms backstop should guard against a TX-complete that never arrives.
- **Statically-allocated, caller-owned events.** Post/publish from file-scope (or pooled) event storage, never stack-local structs (a cooperative AO framework typically stores the event pointer without copying). Design frame events around a fixed-max-payload byte array + length + optional RSSI/SNR/link-quality side-channel.
- **Publish-subscribe fan-out.** Subscribe to an outbound-frame signal; publish an inbound-frame signal so multiple consumers (decoder, link manager) attach without the producer knowing them.
- **Pluggable TX-slot arbitration (TDMA seam).** Before a TX starts, consult an external `ok_to_tx(now)` / `next_tx_window()` hook so an external TDMA/anchor manager can veto/defer; the producer must tolerate dropped sends and rely on its own retransmit cadence. This separates *when a frame is built* from *when it physically leaves the antenna* — a natural home for COP-1/COP-P retransmit timing and the half-duplex `MODE/DUPLEX/TRANSMIT` discipline of the Prox-1 MAC.
- **Owned state, const views, deferred-apply config.** Keep state in one owned struct, hand out const views for status/telemetry, and apply config changes at a safe boundary (e.g., after a frame leaves) rather than mid-transmission.
- **Role parameterization.** Express endpoint behavior (continuous-listen vs. scheduled-transmit vs. forward) through a small role enum + an RX-continuous flag, keeping one codepath.

**Architectural open question:** whether a standalone Starcom AO should be a *peer* of an existing radio AO (subscribing to inbound-frame, publishing outbound-frame) or *wrap/replace* it is a project decision, not resolvable from the scout findings alone.

---

## 9. Open Questions / Low-Confidence Areas

1. **S-band Proximity-1 PHY publication status (low confidence).** Whether CCSDS has published the S-band variant as a formal Blue Book (number, year), and its exact finalized parameters (carrier frequencies, residual vs. suppressed carrier, modulation index, pulse-shaping, EVM, symbol rates), could not be confirmed from a primary CCSDS publication. Treat "specified in 2024" as paraphrase pending a primary citation.
2. **Module oscillator stability (moot but unverified).** Per-module TCXO/crystal stability of specific RFM95W/SX1276 boards vs. the 211.1-B-4 10 ppm/1 ppm regime was not measured. Academic only, given the modulation blocker.
3. **Coherent ranging on Pluto-class SDR.** Feasibility of implementing the E2c coherent turnaround (211.1-B-4 §3.4.5) on hobbyist SDR hardware was not evaluated in depth.
4. **Quote-only pricing.** USD list prices for GomSpace AX100, EnduroSat UHF Transceiver II, and Syrlinks EWC31 are quote-only; only NanoAvionics SatCOM UHF (~€6,350) and ISIS TRXVU (~€8,500) had public anchors.
5. **USLP vs. native Version-3 framing choice** (and the resulting CRC-16 FECF vs. CRC-32 PLTU integrity check) is an unresolved Starcom design decision.
6. **211.2-B-3 cover date** was taken as October 2019 from cross-references and search results; if exact dating is load-bearing, confirm directly from the PDF cover.

---

## 10. Sources

**CCSDS Blue Books (primary)**
- [CCSDS 211.1-B-4 — Proximity-1 Physical Layer (Dec 2013, EC1 Jan 2018)](https://ccsds.org/Pubs/211x1b4e1.pdf) — §1.2 scope; §3.3.2 bands/channels/hailing; §3.3.4 RHCP; §3.3.5 modulation; §3.3.6 rates; §3.4 stability/Doppler; Annex A PICS; Fig 1-1.
- [CCSDS 211.2-B-3 — Proximity-1 Coding & Synchronization Sublayer (Oct 2019)](https://ccsds.org/Pubs/211x2b3.pdf) — §3.2 PLTU/ASM(`FAF320`)/CRC-32; §3.3 idle data (`352EF853`); §3.4 convolutional/LDPC/CSM(`034776C7272895B0`)/randomization; §3.6 frame sync.
- [CCSDS 211.0-B-6 — Proximity-1 Data Link Layer (Jul 2020)](https://ccsds.org/Pubs/211x0b6e1.pdf) — COP-P, FOP-P, FARM-P, PLCW (§3.2.4.3.2, §7); session/hailing (§6).
- [CCSDS 732.1-B-3 — USLP (Jun 2024)](https://ccsds.org/Pubs/732x1b3e1.pdf) — frame header §4.1.2; multiplexing; COP-1/COP-P transparency §2.1.6/2.2.3; OCF service.
- [CCSDS 232.1-B-2 — COP-1 (Sep 2010, TC1 Apr 2019)](https://ccsds.org/Pubs/232x1b2e2c1.pdf) — FOP-1/FARM-1 state machines.
- [CCSDS 232.0-B-4 — TC Space Data Link Protocol (TC1 Oct 2023)](https://ccsds.org/Pubs/232x0b4e1c1.pdf) — CLCW field format §4.2.1.
- CCSDS 131.0-B-3 — TM Synchronization and Channel Coding (Sep 2017) — convolutional & LDPC code definitions.

**Hardware datasheets**
- [Semtech SX1276/77/78/79 datasheet](https://cdn.sparkfun.com/assets/7/7/3/2/2/SX1276_Datasheet.pdf) and [Semtech SX1276 product page](https://www.semtech.com/products/wireless-rf/lora-connect/sx1276) — modulation set (LoRa/FSK/GFSK/MSK/GMSK/OOK; no PM/PSK), 137–1020 MHz.
- [Raspberry Pi RP2350 datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) / [product brief](https://datasheets.raspberrypi.com/rp2350/rp2350-product-brief.pdf); [RP2350 product page](https://www.raspberrypi.com/products/rp2350/); [Pico 2](https://www.raspberrypi.com/products/raspberry-pi-pico-2/); [Adafruit RFM95W PID 3072](https://www.adafruit.com/product/3072).
- [TI CC1200 datasheet](https://www.ti.com/lit/ds/symlink/cc1200.pdf).

**SDR / FPGA**
- [ComBlock COM-1852SOFT (Prox-1 modem IP core)](https://www.comblock.com/zc/index.php).
- [ADALM-PLUTO specs](https://wiki.analog.com/university/tools/pluto/devs/specs); [AD9361](https://www.analog.com/en/products/ad9361.html); [ADRV9002](https://www.analog.com/en/products/adrv9002.html); [Ettus B205mini](https://www.ettus.com/all-products/usrp-b205mini-i/).
- [JPL Electra (radio) overview](https://en.wikipedia.org/wiki/Electra_(radio)).

**COTS cubesat radios**
- [GomSpace NanoCom AX100](https://gomspace.com/UserFiles/Subsystems/datasheet/gs-ds-nanocom-ax100-33.pdf); [EnduroSat UHF Transceiver II](https://catalog.orbitaltransports.com/uhf-transceiver-ii/); [ISIS TRXVU](https://www.cubesatshop.com/product/isis-uhf-downlink-vhf-uplink-full-duplex-transceiver/); [NanoAvionics SatCOM UHF](https://nanoavionics.com/cubesat-components/cubesat-uhf-digital-radio-transceiver-satcom-uhf/); [Syrlinks EWC31](https://www.satcatalog.com/component/ewc31-ss-band-transponder/).

**JPL / ESA / Firefly (BGM2)**
- [NASA-JPL: testing future commercial lunar spacecraft](https://www.nasa.gov/centers-and-facilities/jpl/nasa-jpl-shakes-things-up-testing-future-commercial-lunar-spacecraft/) and [JPL mirror](https://www.jpl.nasa.gov/news/nasa-jpl-shakes-things-up-testing-future-commercial-lunar-spacecraft/) — User Terminal is JPL-developed (radio by Vulcan Wireless); new S-band Prox-1; far-side UHF reserved for radio astronomy.
- [JPL: User Terminal payload delivered to Firefly (PIA26596)](https://www.jpl.nasa.gov/images/pia26596-jpls-user-terminal-payload-delivered-to-firefly/).
- [Firefly Blue Ghost Mission 2](https://fireflyspace.com/missions/blue-ghost-mission-2/); [SSTL Lunar Mission Services](https://www.sstl.co.uk/what-we-do/lunar-mission-services); [ESA Lunar Pathfinder](https://bsgn.esa.int/service/lunar-pathfinder/).
- [CCSDS SLS-CC mailing list, S-band Prox-1 progress (May 2024)](https://mailman.ccsds.org/pipermail/sls-cc/2024-May/000588.html).

**FCC / RF legality**
- [47 CFR 15.247 (Cornell)](https://www.law.cornell.edu/cfr/text/47/15.247) / [eCFR](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-C/subject-group-ECFR2f2e5828339709e/section-15.247).
- [47 CFR 97.113 (Cornell)](https://www.law.cornell.edu/cfr/text/47/97.113); [ARRL Part 97 text](https://www.arrl.org/part-97-text).
