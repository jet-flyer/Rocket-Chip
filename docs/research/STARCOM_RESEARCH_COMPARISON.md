# Starcom Research — Cross-Document Comparison

**Purpose:** A shared, multi-agent comparison of the Starcom research documents. Any agent that performs a cross-document comparison records its findings here. This is **not** owned by one agent — it is the common place where pairwise comparisons, adjudications, and resulting decisions are logged, with explicit attribution for every claim.

**What is being compared:** Four research documents in `docs/research/`, in two pairs:

| Pair | Claude doc | Grok doc |
|---|---|---|
| **CCSDS domain** (what the standards say + hardware feasibility) | `CLAUDE_STARCOM_CCSDS_LIBRARY_RESEARCH.md` | `STARCOM_CCSDS_LIBRARY_RESEARCH.md` |
| **Library-dev craft** (how to build the C++ library) | `CLAUDE_STARCOM_LIBRARY_DEVELOPMENT_RESEARCH.md` | `STARCOM_LIBRARY_DEVELOPMENT_RESEARCH_GROK.md` |

**Attribution convention (mandatory for this doc):**
- Every finding names its **source agent** and the **document state** it was made against (commit or date), because these docs change over time.
- When a finding is later overtaken by an edit to one of the source docs, do **not** delete it — append a `Status:` line recording what changed and when. This preserves the record (per `CROSS_AGENT_REVIEW.md`: flag, don't silently rewrite).
- "Claude doc" / "Grok doc" refer to the **documents**; "Claude (comparison)" / "Grok (author)" refer to **agents acting**.

---

## Entry 1 — Claude comparison (2026-06-01)

**Run by:** Claude (Opus 4.8, Code) — 2026-06-01.
**Method:** Multi-agent workflow (16 agents): one structured pairwise diff per pair → adversarial verification of each substantive conflict against the canonical CCSDS Blue-Book facts (the same two-pass-verified fact set behind `CLAUDE_STARCOM_CCSDS_LIBRARY_RESEARCH.md`) → synthesis. Workflow script retained at `C:\Users\pow-w\.claude\projects\C--Users-pow-w\ae0277c8-e1ad-46a1-9e3a-d220af358ff4\workflows\scripts\starcom-research-compare-wf_1f275e60-d55.js`.

> ### IMPORTANT — document state this entry was made against
>
> The comparison agents read the Grok docs in their **2026-05-30 state** (the cached state at run time). **Shortly before this comparison, Grok committed a fact-check remediation (`5b33e39`, CHANGELOG `2026-05-30-001` follow-up, 2026-05-30) that already fixed most of the factual errors recorded below.** Per the user's direction, the findings below are recorded **as originally found, against the pre-`5b33e39` Grok docs**, so they are on the record. Each affected finding carries a `Status (post-5b33e39):` line stating what Grok's remediation changed. **Readers checking the current files should expect the factual items to already be corrected.** The *judgment* divergences (sans-I/O vs in-core `IPhysicalLayer`, header-vs-static default, framing choice, etc.) are **not** affected by `5b33e39` and remain live.

**Headline:**
1. **CCSDS domain:** On every load-bearing CCSDS fact, the two docs now agree or have converged. Where they differed in the 5/30 state, the **Claude doc was correct** and the Grok doc was outdated/imprecise — and Grok's `5b33e39` has since corrected those. The Claude doc is the stronger *spine* for the standards facts; the Grok doc contributes substantial hands-on enablement (RP2350 PIO prior art, <50 km field-impact analysis, LunaNet/SSTL S-band sourcing) that the Claude doc lacks.
2. **Library-dev craft:** Mostly two reasonable engineering views landing in the same place. A handful of genuine judgment forks remain open (below). The Claude doc is the stronger spine for *architecture*; the Grok doc is stronger on *shipping/operational* mechanics and concrete prior art. Merge wants both.
3. **No claim in either Claude doc was refuted** by the adversarial pass. Two Claude-flagged uncertainties (CRC-32 polynomial, 211.2-B-3 cover date) have since been confirmed by Grok against the 211.2-B-3 PDF (Issue 3, October 2019) — see Entry 1.E.

---

## 1.A — CCSDS domain: what both docs agree on (settled — build on it)

Both the Claude CCSDS doc and the Grok CCSDS doc assert these the same way. Treat as settled ground for the implementation plan.

- **Three-Blue-Book split of Proximity-1.** Physical Layer = **CCSDS 211.1-B-4** (Dec 2013); Coding & Synchronization Sublayer = **211.2-B-3** (Oct 2019); Data Link Layer (COP-P, PLCW, hailing/session) = **211.0-B-6** (Jul 2020). *(Both — Grok's explicit naming of 211.2-B-3 as distinct was added in `5b33e39`; see 1.C-(E).)*
- **The PHY is intentionally thin.** It modulates a coded symbol stream handed up from the C&S Sublayer and reports only `CARRIER_ACQUIRED` and `SYMBOL_INLOCK_STATUS` to the MAC. Functions colloquially called "PHY work" (ASM/sync detection, PLTU construction, symbol sync) live in the **C&S Sublayer**, not the PHY. *(Both.)*
- **The PHY waveform.** 211.1-B-4 §3.3.5 mandates PCM **Bi-Phase-L (Manchester) residual-carrier phase modulation at index 60° ±5%**; §3.4 oscillator/RF reqs: long-term ≤10 ppm, short-term ≤1 ppm/min, residual AM <2% RMS. *(Both.)*
- **The SX1276/RFM95W cannot natively produce the 211.1-B-4 waveform** (LoRa/(G)FSK/(G)MSK/OOK only; no residual-carrier PM path). Full native PHY compliance is **not achievable** on current hardware. *(Both — though they framed the *consequence* differently; see 1.B-(A).)*
- **Starcom's real value is the digital, transport-agnostic data-link machinery** (PLTU/USLP framing, COP-1, COP-P) runnable over any COTS bearer including 915 MHz LoRa/FSK. *(Both.)*
- **Data-link standards & revisions:** COP-1 procedures = **232.1-B-2** (2010, +TC1 2019); COP-P/PLCW = **211.0-B-6** (2020); USLP = **732.1-B-3** (Jun 2024). *(Both.)*
- **COP-1 = two coupled state machines per VC** — FOP-1 (sender/ground: go-back-N retransmission, sliding window, T1 timers, CLCW interpretation) and FARM-1 (receiver/spacecraft: frame acceptance, sliding window, CLCW generation with Lockout/Wait/Retransmit flags + FARM-B counter + Report Value N(R)). Managed Parameters (T1, Transmission_Limit, window widths) must be configurable, not hard-coded. *(Both.)*
- **USLP (732.1-B-3) is the forward-looking unifier** that carries both COP-1 and COP-P on different Virtual Channels. *(Both.)*
- **PLTU** = ASM (`0xFAF320`) + Version-3 Transfer Frame + CRC-32. *(Both.)*
- **RP2350 PIO is well-suited to the digital bitstream work** (Manchester encode/decode, symbol timing, ASM correlation, PLTU assembly with DMA) — a real reason to keep it in the system. *(Both; Grok far more detailed — see 1.D.)*
- **Hardware upgrade paths for a real PHY:** ComBlock **COM-1852SOFT** VHDL IP (~$2,500 + FPGA/RF front end) for a genuinely compliant PHY; **ADALM-PLUTO** SDR (Claude: ~$186–230; Grok: ~$200–300) for waveform research. *(Both.)*
- **915 MHz / 902–928 MHz ISM (FCC Part 15) is a legal COTS bearer but NOT a CCSDS-authorized Proximity-1 band.** *(Both.)*
- **The Blue Ghost Mission 2 radio is the JPL User Terminal** (JPL-developed SDR, radio by Vulcan Wireless), implementing a **new lunar S-band** Proximity-1 profile (~2024) to commission ESA Lunar Pathfinder — **not "Firefly's protocol."** No public open-source implementation of the exact S-band waveform/hailing parameters exists. *(Both.)*
- **Prior-art libraries:** OSDLP, NASA cFS, CCSDSPack, EmbeddedSpacePacket — follow the CCSDS state tables literally, keep COP engines independent of PHY/framing. *(Both.)*

---

## 1.B — CCSDS domain: decision-changing divergences

### (A) "Software PHY" on SX1276+PIO — achievable? — *adjudicated: not a real conflict; both converge on NO*

- **Claude doc says:** Emphatic NO; the failure is fundamental, not a firmware-effort problem. Frame the deliverable as a "Proximity-1-INSPIRED data-link stack over a COTS 915 MHz link." PHY is binary — compliant or not.
- **Grok doc (5/30 state) said:** §2.3 — "A high-value software Physical Layer is realistically achievable" via FSK Continuous Mode + PIO; **but §2.5 (a later self-correction in the same doc) reversed this** and agreed full conformance is impossible, recommending a best-effort adapter behind `IProximityPhysicalLayer`.
- **Claude (comparison) verdict:** Claude-doc-correct (high confidence), but **mostly NOT a real contradiction** — Grok's own §2.5 supersedes §2.3 and converges with the Claude doc. The only surviving genuine difference is a naming/framing call (see decision D-1).
- **Decision implication:** Starcom ships **no "211.1-B-4 Physical Layer."** Headline = a Proximity-1/CCSDS data-link + C&S stack over an honestly-labeled non-compliant COTS 915 MHz bearer. Define an abstract radio boundary so a true PHY (Pluto/ComBlock) drops in later, **named neutrally — `IRadio`/`IBearer`, not `IProximityPhysicalLayer`** — to avoid implying CCSDS PHY conformance.
- **Status (post-`5b33e39`):** Grok edited §2.3 to mark the optimistic language as superseded by §2.5 and removed the scrambled duplicate tail. The two docs are now explicitly aligned on the NO. The naming/framing fork (D-1) remains open.

### (B) "Closer to compliant" embedded chip: onsemi AX5043 — *adjudicated: Claude-doc-correct (was a verified Grok error)*

- **Claude doc says:** No low-cost fixed-modem COTS radio (CC1200, Si446x, AX100, …) can emit the residual-carrier Bi-Phase-L/PM waveform — all "not viable for the waveform." Does not offer the AX5043 as a compliance path.
- **Grok doc (5/30 state) said:** Recommended the **onsemi AX5043** as the strongest integrated chip "closer to compliant PHY behavior" because of native PSK + clean symbol-clock pins + 390–450 MHz coverage.
- **Claude (comparison) verdict:** Claude-doc-correct (high confidence) — this was a **factual category error**, not a judgment difference. 211.1-B-4 requires **residual-carrier, partial-index (60°)** PM; the AX5043's PSK is **suppressed-carrier, fixed 180° BPSK** — wrong on both load-bearing axes, with no register to make it residual-carrier or partial-index. A 211.1-B-4 receiver cannot lock an AX5043 emission; it is no "closer" than the SX1276. The AX5043's clean clock/data pins are a legitimate but *separate, lower-priority* raw-bitstream ergonomics point (which RP2350 PIO already provides).
- **Decision implication:** Do **not** put the AX5043 (or any fixed-modem chip) on the roadmap as a "closer to PHY compliance" step. True PHY = SDR (Pluto) or FPGA IP (ComBlock).
- **Status (post-`5b33e39`):** Grok corrected the AX5043 entries (§2.9) to state explicitly that it implements "suppressed-carrier BPSK/PSK … not the residual-carrier phase modulation at a precise 60° index required by … 211.1-B-4 … a good Data Link / MAC enabler but still a best-effort (non-conformant) approximation." The error is **fixed**; the chip now correctly carries the caveat.

---

## 1.C — CCSDS domain: factual divergences (5/30 state) — Claude-doc-correct, since corrected by Grok

These were true factual contradictions in the 5/30 Grok doc. The adversarial pass found the **Claude doc correct** in every case. Grok's `5b33e39` has since fixed all of them — recorded here for the record.

### (C) Does Proximity-1 have an OCF, and how is the PLCW carried?
- **Claude doc says (§6.3/§6.4):** Proximity-1 defines **NO Operational Control Field**. The PLCW is a **16-bit fixed-length SPDU of seven fields** (211.0-B-6 §3.2.4.3.2.1.1), sent on the supervisory channel with Expedited QoS — not in a Transfer-Frame OCF, not 32 bits. COP-1 (CLCW-in-OCF) and COP-P (PLCW-as-SPDU) are two distinct mechanisms. *(Note: the Claude doc records that its own earlier draft wrongly said "OCF carries the CLCW or, for Prox-1, the PLCW" and corrected it — see its §6.4 verification call-out.)*
- **Grok doc (5/30) said:** Called the PLCW the "Proximity-1 equivalent" of the CLCW; listed "232.0-B … for frame structures and OCF usage" loosely; gave no bit layout. Functionally right at a high altitude, imprecise where it matters.
- **Claude (comparison) verdict:** Claude-doc-correct (high confidence); conflict was overstated — Grok never actually asserted the PLCW is in an OCF.
- **Decision implication:** Model `Clcw32` and `Plcw16` as **two distinct types with separate generate/parse paths** (FARM-1 vs FARM-P). **No generic OCF abstraction hosting both.** OCF-flag logic lives only on the TM/AOS/USLP side. Use the **Claude doc §6.3** for the PLCW bit layout.
- **Status (post-`5b33e39`):** No substantive change needed on Grok's side (it was imprecise, not wrong). Decision stands.

### (D) Which CCSDS doc defines the CLCW bit format?
- **Claude doc says:** The **bit format** is in **232.0-B-4** (TC Space Data Link, +TC1 Oct 2023) §4.2.1; **232.1-B-2** owns only the procedures/state machines.
- **Grok doc (5/30) said:** Attributed "CLCW exact bit layout and semantics" to the COP-1 doc **232.1-B-2** (§1.1).
- **Claude (comparison) verdict:** Claude-doc-correct (high confidence) — true factual contradiction. The CCSDS TC stack splits this: the Space Data Link Protocol Blue Book (232.0-B) owns the data-structure/field formats; the COP-1 Blue Book (232.1-B) owns the procedures and references 232.0-B normatively.
- **Decision implication:** Encode the CLCW struct/serializer against **232.0-B-4 §4.2.1**; encode FOP-1/FARM-1 semantics against **232.1-B-2**.
- **Status (post-`5b33e39`):** **Fixed.** Grok's §1.2 now reads "CLCW … defined in CCSDS 232.0-B-4 (TC Space Data Link Protocol)" with 232.1-B-2 owning the state machines/procedures. The docs now agree.

### (E) Is the C&S Sublayer a distinct standard (211.2-B-3), and who owns PLTU/ASM/CRC-32/coding?
- **Claude doc says:** **211.2-B-3** (Oct 2019) is the named separate Blue Book owning PLTU/ASM(`FAF320`)/CRC-32/convolutional+LDPC/CSM(`034776C7272895B0`)/idle PN(`352EF853`)/randomization/frame-sync, with a full feature-ownership table.
- **Grok doc (5/30) said:** Referred to "211.2-B" generically with no issue/date, and **attributed PLTU/ASM to the 211.1-B-4 PHY doc** (§1.3).
- **Claude (comparison) verdict:** Claude-doc-correct (high confidence) — true contradiction on the load-bearing point. Verified against the 211.1-B-4 PDF: it cites the C&S sublayer as normative reference [2] (proving they are distinct docs); the PHY doc mentions PLTU only as a glossary courtesy and does **not** specify the ASM, CRC-32, or coding.
- **Decision implication:** Use the **Claude doc's document/feature-ownership map.** Model 211.2-B-3 as a distinct module (PLTU framing + coding + CSM + frame sync), separate from the 211.1-B-4 PHY and the 211.0-B-6 Data Link.
- **Status (post-`5b33e39`):** **Fixed.** Grok's §1.1 now lists "Proximity-1 Coding & Synchronization Sublayer: CCSDS 211.2-B-3 (October 2019) … PLTU construction, ASM, CRC-32 …" as its own bullet; §1.3 reframed as a "for reference" PHY summary. The docs now agree.

### (I) SX1276 datasheet revision
- **Claude doc says:** Semtech datasheet **Rev. 7, May 2020**.
- **Grok doc (5/30) said:** **Rev. 4, March 2015**.
- **Claude (comparison) verdict:** Claude-doc-correct (use the later revision). Minor.
- **Status (post-`5b33e39`):** **Fixed** — Grok's §2.2 now cites Rev. 7, May 2020.

---

## 1.D — CCSDS domain: legitimate judgment differences (both partially right) — NOT affected by `5b33e39`

### (G) Is the SX1276's built-in Manchester meaningful?
- **Claude doc:** Bi-Phase-L is a PHY function, but the chip's fixed FSK/OOK re-modulation makes the *waveform* wrong regardless; built-in Manchester is not meaningful for compliance.
- **Grok doc (§2.2/§2.8):** SX1276 has explicit Manchester support and Continuous Mode exposes DCLK/DATA — usable building blocks for a best-effort symbol-stream interface.
- **Verdict (both-partially, high confidence):** Both state true facts at different layers. The chip yields Manchester *bits* but the wrong *waveform* (FSK, not residual-carrier PM). The Claude doc answers the compliance question (no); the Grok doc answers the best-effort-adapter question (yes). Minor Grok over-weight: the packet-engine Manchester is coupled to Packet Mode and would largely be **bypassed** in the very Continuous-Mode design Grok advocates.
- **Decision implication:** Best-effort design = **SX1276 FSK Continuous Mode, packet engine OFF**, RP2350 PIO owning Bi-Phase-L + ASM(`FAF320`) correlation + symbol timing, behind a clearly-labeled `IRadio` best-effort adapter. Do **not** model built-in Manchester as a PHY capability.

### (H) Primary source for the lunar S-band parameters
- **Claude doc:** Anchors on the CCSDS catalog + SLS mailing-list traffic + JPL release PIA26596 ("specified in 2024"); flags that **no ratified public S-band Blue Book** appears in the catalog.
- **Grok doc (§3.2):** Anchors on **NASA LunaNet Interoperability Spec v5 (2025)** + **SSTL Lunar Pathfinder Service Guide V2.2**, listing a concrete waveform menu (BPSK, PCM/PM, PCM/PSK/PM, GMSK, OQPSK, spread-spectrum), S-band freqs (fwd ~2025–2110 / ret ~2200–2290 MHz), 0.5 kbps–2 Mbps, hailing via CCSDS 235.1.
- **Verdict (both-partially, high confidence):** Claude is right on *sourcing principle* (for a fidelity-claiming library the normative source is the CCSDS spec; LunaNet/SSTL are mission *profiles* — secondary). But because the authoritative CCSDS S-band params aren't public, Grok is right that LunaNet v5 + SSTL are the *only currently-published concrete numbers*. Grok's specific values are **profile-sourced, not CCSDS-primary-verified** — flag them as such.
- **Decision implication:** Adopt a **two-tier source hierarchy**: (1) NORMATIVE = CCSDS Blue Books (treat the S-band CCSDS profile as not-yet-ratified-public; do not hardcode S-band params into the portable core); (2) PROFILE = LunaNet v5 + SSTL, cited only as current best-available numbers for a pluggable S-band PHY/profile module, labeled mission-derived.

---

## 1.E — CCSDS domain: Claude-doc items flagged uncertain — since confirmed

The Claude CCSDS doc's own Open-Questions section flagged two items as taken from cross-references rather than the PDF cover:
- **CRC-32 generator polynomial** (`X³²+X²³+X²¹+X¹¹+X²+1`).
- **211.2-B-3 cover date** (stated as October 2019).

**Status (per CHANGELOG `2026-05-30-001` follow-up, commit `5b33e39`):** Grok re-confirmed both against the actual **211.2-B-3 PDF (Issue 3, October 2019)** — cover date confirmed; CRC-32 confirmed via the annex reference to the standard CCSDS CRC-32 procedure. Both Claude-doc uncertainties are now **resolved**. *(Recorded by Claude comparison; verified item authored by Grok.)*

**One residual TODO for the plan author (Claude comparison):** the **211.2-B-3 issue is "B-3"** but its cover is **"Issue 3, October 2019"** — the Claude doc's revision roll-up writes "211.2-B-3 … Oct 2019," consistent. No action; noted only so a future reader doesn't re-flag it.

---

## 1.F — CCSDS domain: minor divergences and coverage notes

- **(J) USLP header detail:** Claude doc gives the full Primary Header bit table + supersession history ("do not cite B-2 as latest"); Grok conceptual only. **Use the Claude doc's table.**
- **(K) FCC depth:** Claude doc details Part 15.247 (≥500 kHz / 1 W) vs 15.249, ~36 dBm EIRP ceiling, and the **Part 97 amateur 97.113(a)(4) encryption prohibition** (framing/FEC/CRC/Manchester OK; encrypted payload not). Grok mentions only "Part 15, license-free." **Use the Claude doc's analysis** — the encryption prohibition is a real design constraint if amateur bands are ever used.
- **(L) UHF channel/freq detail:** Claude doc gives full tables (fwd 435–450 / ret 390–405 MHz, hailing Channel 1, turnaround ratios, RHCP, 13 symbol rates, radio categories E1/E2n/E2c/E2d); Grok says only "UHF/Mars-focused." **Use the Claude doc's tables.**
- **(M) Vulcan Wireless:** Claude doc treats it as the JPL-radio builder (attribution); Grok additionally lists a buyable "Vulcan NSR-SDR-S/S" (quote-only, space-grade). Grok's product lead is worth keeping as a contact.

### Coverage gaps — what each doc should absorb from the other
**Absorb from the Claude CCSDS doc:** the feature-to-owning-document map; 211.2-B-3 coding internals (CRC-32 poly, idle/acq PN, conv K=7, LDPC n=2048/k=1024 via 131.0-B-3, 64-bit CSM); full USLP Primary Header table + multiplexing hierarchy + FECF(CRC-16)-vs-PLTU(CRC-32) decision; complete FOP-1 six-state and FARM-1 three-state enumerations; full CLCW-32 and PLCW-16/7-field bit tables; UHF channel/oscillator tables; the "Proximity-1 has NO OCF" finding; full FCC analysis; COTS-radio + SDR/FPGA survey tables with prices; JPL Electra as the FPGA-SDR proof-of-concept; ISO 21460:2015 equivalence; the framework-agnostic Active-Object seam guidance; the explicit Open-Questions/low-confidence section.

**Absorb from the Grok CCSDS doc:** concrete **RP2350 PIO prior art** (pico-examples `manchester_encoding`/`differential_manchester`, 433 MHz OOK decoders, **ADS-B dual-PIO preamble+Manchester as an ASM-detection analog**, 100BASE-TX on PIO); the §2.8 enumeration of six PIO opportunities (synchronous DCLK-edge sampling, custom Bi-Phase-L, transition-density/eye SYMBOL_INLOCK logic, TX timing, acquisition-state assist, sync correlation); the **§2.7 in-field impact assessment** for <50 km HPR (link margin far exceeds a Mars link → gap is small for normal telemetry, felt mainly in post-landing wake-up / edge-of-coverage descent / future ranging); LunaNet PFS1a/b profile labels + SSTL hailing procedure (carrier-only forward hailing addressed by SCID); concrete next prototyping steps.

### Tone
- **Claude CCSDS doc:** single-author, standards-forensic, adversarially-verified reference with dated in-place corrections.
- **Grok CCSDS doc:** exploratory multi-wave engineering log, richer on hands-on PIO/datasheet enablement; in the 5/30 state it was looser on provenance and internally un-reconciled (§2.3 vs §2.5) — `5b33e39` substantially tightened both.

---

## 2.A — Library-dev craft: what both docs agree on (settled)

- **Error handling = `std::expected`/`tl::expected`, NOT exceptions** in the core; small strongly-typed `enum class` error domain; optional exception-throwing facade **desktop-only** behind a compile switch. *(Both.)*
- **MCU path is the binding constraint:** compiles/runs under `-fno-exceptions -fno-rtti`, freestanding-friendly, **zero dynamic allocation after init** (JSF AV Rule 206), large objects statically allocated; desktop is secondary. *(Both.)*
- **Zero-copy buffers** via `std::span<[const] std::byte>` with an `etl::span`/CETL polyfill for freestanding/C++17. *(Both.)*
- **Strong typing** over primitives (`Vcid`, `Apid`, `FrameSeqNumber`, `Scid`, `MapId`). *(Both.)*
- **Compile-time configurability** of buffer sizes/queue depths/window+VC+MAP counts via templates or a `config.hpp` profile (ETL `etl_profile.h` model). *(Both.)*
- **Modern target-based CMake** with full install/export (`CMakePackageConfigHelpers`, `configure_package_config_file`, generated `*Config.cmake.in`) and **FetchContent via the hybrid `find_package` + `FIND_PACKAGE_ARGS`** pattern. *(Both; FetchContent "non-negotiable" per Grok.)*
- **Layout:** `include/` = public API only; private impl under `detail/`; namespace mirrors directory, rooted at `starcom`. *(Both.)*
- **Apache-2.0** license. *(Both.)*
- **Host-heavy testing:** bulk = fast host unit tests under ASan/UBSan + coverage; **property-based + coverage-guided/grammar-aware fuzzing of frame/packet parsers**; on-target smoke only for the irreducible hardware slice. Both single out **COP-1/FARM-1** (window, retransmission, CLCW feedback, timeouts) as the key stateful-fuzz/PBT target with thin public coverage. *(Both.)*
- **CI cross-compiles against `arm-none-eabi-gcc`** to prove the core stays freestanding-clean. *(Both.)*
- **Narrow swappable PHY/transport abstraction** isolates data-link logic from any radio; SX1276+PIO adapter documented as best-effort; **RadioLib's `PhysicalLayer`** cited as precedent. *(Both — but they disagree on WHERE the seam lives; see 2.B-(B).)*
- **Docs are a first-class deliverable:** Doxygen + conformance/limitation statements (esp. PHY non-compliance) + runnable examples + explicit allocation-free/exception-free contract; CLI encoder/decoder/validator tools as living docs. *(Both.)*
- **Rocket-Chip integration = thin Active-Object wrapper only:** a new AO owns the Starcom instance and translates to/from QP/C events (`CommandAccepted`, `LinkLost`, …) via `QACTIVE_POST`; **the core takes no QP/C dependency.** *(Both.)*
- **Prior art:** CCSDSPack (closest direct CCSDS model), OSDLP (embedded COP-1 heritage), cFS `cop1.c` (flight-heritage reference, not code to copy). *(Both.)*
- **SemVer** + `version.hpp` macros + CHANGELOG. *(Both.)*

---

## 2.B — Library-dev craft: divergences

### (A) Header-only (INTERFACE) vs static library as the default — *adjudicated: lean Claude doc (medium confidence); Grok's load-bearing argument was an error, since retracted*
- **Claude doc:** **Static by default** — Starcom's "real state machines with sizable `.cpp` logic" argue against header-only; lists it as an open question warranting a measurement spike.
- **Grok doc (5/30 state) said:** **Header-mostly (INTERFACE) default**, optional static — argued for portability to "weird" toolchains, **"full LTO + inlining,"** and the dominant 2024–2026 aerospace trend.
- **Claude (comparison) verdict:** Lean Claude doc (medium confidence). Both docs converge on the *same two artifacts*; the split is which is "primary." Static-by-default is sounder here because: (1) **code shape governs** — Starcom is non-template state-machine logic, the opposite of the template/codegen libs Grok cites; Grok's own headline example **CCSDSPack is static-on-MCU**; (2) **Grok's LTO claim was incorrect** — a static `.a` built with `-flto` + the linker plugin gets identical cross-boundary inlining; LTO is a function of `-flto`, not packaging; (3) Starcom's flight target is one fixed toolchain (arm-none-eabi GCC 14), so "weird toolchains" is generic-library folklore here. Medium (not high) because both forms yield essentially the same flight binary and the Claude doc itself honestly flags it as open.
- **Decision implication:** **Static library = default/primary**; header-only INTERFACE as an opt-in alias behind an option. Build with `-flto -ffunction-sections -fdata-sections -Wl,--gc-sections`. **Run the Claude doc's compile-both-ways spike in Phase 0/1** (arm-none-eabi flash + compile-time comparison); flipping the default later is a one-line CMake change.
- **Status (post-`5b33e39`):** Grok's §3 now explicitly retracts the LTO argument ("LTO … is controlled by compiler/linker flags and works with both header-mostly and properly-built static libraries; it is not a unique property of header-only packaging"). The **factual error is fixed.** Grok still *prefers* header-mostly as the stated default — so the **header-vs-static default remains a live (now purely judgment-based) fork**, to be settled by the spike + the CI size report (decision D-5).

### (B) Sans-I/O pure core vs `IPhysicalLayer` under the core — *adjudicated: both-partially; Claude doc's call is better under Starcom's constraints* — NOT affected by `5b33e39`
- **Claude doc:** Names **sans-I/O** as THE foundation — core API `receive_bytes()`/`bytes_to_send()`/`poll_event()`/`handle_timeout(now)`/`submit_sdu()`; argues the core **shouldn't even call an abstract `IRadio`** — it hands the app bytes-to-send. Cites sans-io.readthedocs + h11/hyper-h2/webrtc-rs/Firezone.
- **Grok doc:** Reaches a neighboring place (state in user structs, alloc-free hot path, data-link with "zero knowledge of any radio," high-level events to the AO) but puts an **`IPhysicalLayer::send_frame/receive_frame` seam *in/under* the core** (§7; prior art opendnp3 `openpal::IPhysicalLayer`, RadioLib).
- **Claude (comparison) verdict:** Both-partially (high confidence), but **the Claude doc's call is better under Starcom's constraints.** The one load-bearing difference: Grok's core **calls out through a virtual seam** = vtable dispatch = a function pointer, in direct tension with **P10 Rule 9** (load-bearing on Rocket-Chip per `CODING_STANDARDS.md` + `LESSONS_LEARNED` Entry 37); `-fno-rtti` + no-heap-after-init also make an injected `IPhysicalLayer`'s lifetime awkward. A pure core has nothing to allocate or dispatch through, and the "100% branch coverage via public API, no mocks" testability (which Grok's *own* fuzz-the-FARM-1 plan depends on) is easier with no seam to stub. Grok is **not wrong about the goal** (it calls transport-agnosticism "the single most important architectural decision") — it just stops one level short of removing I/O from the core. Grok's cited prior art (opendnp3 = heap/exception desktop DNP3; RadioLib `PhysicalLayer` = the *adapter* layer) imports an I/O-coupling pattern from libs that don't share Starcom's constraints.
- **Decision implication:** Adopt **sans-I/O as the named foundational decision**; keep the core strictly I/O-free. Fold in Grok's two compatible contributions: (a) **state in caller-owned statically-allocated structs**; (b) Grok's **high-level semantic events** (`CommandAccepted`, `LinkLost`, `LogOffloadComplete`) become the `poll_event()` vocabulary the AO consumes. **Demote `IPhysicalLayer` to the adapter layer** (`starcom-radio-sx1276`/`starcom-host`) — where the Claude doc already puts it and where RadioLib's `PhysicalLayer` actually belongs. If any in-core seam is ever needed, use the **templated/policy (devirtualized) form, not a virtual ABC.**
- **Status (post-`5b33e39`):** Unaffected — this is a genuine architecture fork, not a fact error. **Remains the single most important open architectural decision** for the plan.

### (C) vcpkg / package-manager timing — *adjudicated: both-partially; settle by superset* — partly softened by `5b33e39`
- **Claude doc:** **Defer the vcpkg/Conan port to post-1.0** (design install/export now so it's ready).
- **Grok doc (5/30 state) said:** Ship a **`vcpkg.json` + `portfile.cmake` in v1 defaults** (manifest mode + baseline + binary caching + **custom triplets for the `-fno-exceptions -fno-rtti` bare-metal flags**) — "near-non-negotiable" for reproducibility/air-gapped/certification builds.
- **Claude (comparison) verdict:** Both-partially (high confidence); settle by superset. Both already agree the **install/export machinery is mandatory near-term**. Grok is right on a real hook the Claude doc omits — the **custom triplet is a clean home for the bare-metal flag set** (a vcpkg-specific config-management win), and CCSDSPack already does roughly this. But Grok's "non-negotiable for air-gapped/certification" overreached against the project's stated hobbyist/educational scope (cert deferred to a future tier), and a near-zero-dependency leaf library gets little from vcpkg's transitive-dep machinery. The Claude doc is right that **maintaining a published registry port is real ongoing work** that can wait until the API is 1.0-stable.
- **Decision implication:** **Phase 0 includes the install/export both agree on.** Additionally adopt Grok's two low-cost wins now: a **checked-in (NOT-yet-upstreamed) `vcpkg.json` + `portfile.cmake`** that just runs the installed config, and a **custom triplet** carrying the freestanding flight flags. **Defer to post-1.0:** registry publication, any Conan recipe, binary-cache/CI infra, locked-baseline maintenance.
- **Status (post-`5b33e39`):** Grok softened the "non-negotiable" framing (§1/§5 now: "valuable even in hobbyist/educational projects. Air-gapped or formally certified builds would typically use vendoring, submodules, or a locked private registry…"). The overreach is **fixed**; the v1-vs-defer timing nuance is resolved by the superset decision above.

### (D) Resource (flash/RAM/stack) reporting in CI — *adjudicated: both-partially; Grok's items are additive* — NOT affected by `5b33e39`
- **Claude doc:** CI = GCC/Clang/MSVC × Debug/Release, arm cross-compile, sanitizers, fuzz-smoke. Silent on size/RAM/stack and a malloc shim.
- **Grok doc:** Size/RAM/stack reporting per config (1 VC COP-1, 4 VCs, max window) is "non-negotiable for MCU users," plus a "fail on any malloc after init" shim and an RTOS (FreeRTOS/Zephyr) example build.
- **Claude (comparison) verdict:** Both-partially (high confidence); Grok's items are additive, not opposed. The **malloc-after-init shim is the mechanical enforcement** of the no-heap contract both treat as load-bearing — an untested "no-heap" claim is exactly the structurally-soft gate `HW_GATE_DISCIPLINE` Rule 1 + `LESSONS_LEARNED` 36/40 warn against (require a positive control, not negative evidence). It also produces the data the Claude doc's *own* header-vs-static open question needs. Grok overstates only that **absolute flash/RAM numbers are not a portable merge gate** (toolchain/`-O`/profile dependent); the defensible form gates on **regression/delta**. The RTOS build is the most negotiable item.
- **Decision implication:** Add to Phase-0 CI: (1) a **`-fno-exceptions -fno-rtti` no-heap-after-init shim** (trapping `malloc`/`operator new` after init) as a **HARD pass/fail gate**; (2) a **flash/RAM/stack report for 1–2 fixed configs** (1 VC COP-1; 4 VCs + max window) with pinned arm flags, published as a CI artifact and **flagged on delta-vs-baseline, not absolute threshold** (also resolves the header-vs-static open question with data). One RTOS example build = nightly/"should," not a merge blocker.

### (E) C89/C-ABI (Hourglass) bindings — *adjudicated: NOT a real conflict*
- **Claude doc (§4.4):** Discusses the **Hourglass C89 ABI seam** (Du Toit, CppCon 2014) — recommends it over per-class pimpl IF a binary desktop `.so`/`.dll` is ever shipped, but explicitly says "not needed for the static/embedded core … skip both."
- **Grok doc:** Silent on Hourglass/C-ABI/pimpl; packaging focus is source-distribution.
- **Claude (comparison) verdict:** Not a real conflict (high confidence). Both effectively *decline* the technique now. ABI stability is strictly a pre-built-binary-crossing-toolchains problem; a C++17 lib compiled fresh on both targets has no such boundary.
- **Decision implication:** Do **not** add a C89/C-ABI seam to the core or Phase 0. Keep the Claude doc §4.4 as a documented, deferred "if we ever ship a binary `.so` + Python bindings" appendix (preferring Hourglass over per-class pimpl at that future point).

### Minor (library-dev)
- **(F) pimpl:** Claude doc "no pimpl in the core" (heap cost; ABI moot when statically linked); Grok silent. Not a conflict — **no pimpl in core.**
- **(G) `tl::expected` vendor vs hand-rolled `Result`:** Claude doc poses it as an open question (vendor = faster; hand-roll = MISRA/JSF reviewers own the layout, zero external dep); Grok picks **`tl::expected` default** with a `STARCOM_USE_STD_EXPECTED` knob. **Resolve as a config knob:** ship `tl::expected` as default, expose the knob so a zero-dependency `starcom::Result` is a one-flag swap (satisfies the Claude doc's MISRA concern without blocking 1.0).
- **(H) Error-object size bound:** Grok states a concrete invariant — error objects **trivially copyable, ≤16–32 bytes, core handling `noexcept`**; Claude doc gives no number. **Adopt Grok's bound** (a useful MCU invariant the Claude doc doesn't contradict).
- **(I) Repo/package name:** Claude doc `starcom` (`Starcom::starcom`, `include/starcom/`, `starcom::ccsds`); Grok `starcom-ccsds` (`StarcomCcsdsConfig.cmake`, `find_package(StarcomCcsds CONFIG)`, same `starcom::ccsds` namespace). Cosmetic — **recommend `starcom` as the umbrella repo, `starcom::ccsds` as the protocol namespace.**
- **(J) Test framework:** Claude doc names **GoogleTest + FuzzTest** via CTest (also Catch2/doctest); Grok generic. **Adopt the Claude doc's picks** for consistency with the existing Rocket-Chip host suite.

### Coverage gaps — library-dev
**Absorb from the Claude lib-dev doc:** the four-persona portability council (unanimous "one core + adapters"); sans-I/O as a named/sourced pattern with full API surface; the Hourglass/pimpl ABI discussion (as a deferred appendix); `-fvisibility=hidden` + generated `STARCOM_EXPORT` via `generate_export_header`; Google's breaking-change policy + Keep-a-Changelog as concrete `VERSIONING.md`/`CHANGELOG` models; the **7-phase build plan** (Phase 0 skeleton → Phase 6 hardening, pure codecs first to build the test scaffolding before state machines); Blue-Book-clause→file citations; governance (CONTRIBUTING.md + DCO, CODEOWNERS, issue/PR templates with frame hexdumps); the "what makes a library adopted" synthesis; the AO-wrapper-location open question.

**Absorb from the Grok lib-dev doc:** the **F' (F Prime) feasibility ruling** (§10: a full component framework with FPP/codegen/OSAL/topology, **NOT viable as a lightweight plugin** — integration target only, do not design Starcom around F' internals); **fetched CCSDSPack CMake excerpts** (`option(CCSDSPACK_BUILD_MCU OFF)`, STATIC + `ccsdspack::mcu` alias, `-Os -ffunction-sections -fdata-sections -Wl,--gc-sections`, `POSITION_INDEPENDENT_CODE OFF`, `CROSSBUILD.md`); `CMakePresets.json` with arm-none-eabi cross presets; `FILE_SET HEADERS` (CMake 3.23+); the concrete **`IPhysicalLayer` method surface** (`send_frame`/`receive_frame`, CARRIER_ACQUIRED/SYMBOL_INLOCK best-effort signals, eye-quality/transition-density/lock-metric reporting — to live in the *adapter*, per 2.B-(B)); the named PHY adapter roadmap (Pluto, AX5043 *as raw-bitstream only*, ComBlock); the broader prior-art roster with lessons (**Glaze** `glz::expected` under `-fno-exceptions`; **OpenCyphal/CETL**; **opendnp3**; **EmbeddedSDLP** C11 zero-heap TM/TC; **OSDLP's** "expose the queue/timer hooks, don't hide them"); concrete fuzzing precedents (**NASA cFS Space Packet AFL** found length/APID/fragmentation memory corruption; **FlatSat** RP2040+SX1262 grammar-based RF fuzzing of `spp_unpack_packet`); the two-tier API (high-level monadic + low-level zero-copy span); the **alignment checklist tied to Rocket-Chip `CODING_STANDARDS.md` item numbers** (Rule 206 = item 408, no-exceptions = item 410, large-object >1 KB static, `<stdio.h>` banned in `src/*.cpp` → `rc::rc_log`/`rc::rc_snprintf`).

### Tone
- **Claude lib-dev doc:** pedagogical library-craft tutorial (fixed 5-part section shape, named architectural ideas, persona council, phased plan, deliberately-open trade-offs).
- **Grok lib-dev doc:** denser catalog-driven survey grounded in concrete 2024–2026 prior art with fetched CMake/dated notes; more decisive on defaults, more tightly bound to the Rocket-Chip standards file.

---

## 3 — Net assessment & decisions teed up (Claude comparison, 2026-06-01)

**Stronger spine per domain:**
- **CCSDS domain → the Claude doc is the spine.** It is the adversarially-verified reference for what the standards say and whether the hardware can comply. On every *verified-fact* divergence in the 5/30 state the Claude doc was right; Grok's `5b33e39` has since corrected those. **Merge in from Grok:** the hands-on enablement layer — RP2350 PIO prior art, the six PIO opportunities, the <50 km in-field impact intuition, LunaNet/SSTL S-band numbers (as profile-tier annotations), concrete prototyping next-steps.
- **Library-dev craft → split; lean the Claude doc for architecture.** The Claude doc is the spine for the *foundational decisions* (sans-I/O, static-default, build-phase plan, portability council, ABI reasoning). **Merge in from Grok:** the *shipping/operational* layer — the F' ruling, fetched CCSDSPack CMake patterns, CMakePresets/`FILE_SET HEADERS`/vcpkg-triplet mechanics, CI size-reporting + the malloc-shim gate, the broader prior-art roster, the Rocket-Chip standards line-number alignment checklist.

**Concrete decisions for the implementation plan:**

- **D-1 — PHY honesty / compliance claim (settled → Claude doc).** Starcom ships **NO "211.1-B-4 Physical Layer."** Headline deliverable = a Proximity-1/CCSDS **data-link + Coding & Sync** stack (211.2-B-3 PLTU, 211.0-B-6 COP-P/PLCW, USLP/COP-1 per 732.1-B-3 / 232.1-B-2) over an **honestly-labeled non-compliant COTS 915 MHz bearer**. Documentation states PHY non-compliance explicitly. **Do not claim any PHY compliance; do not add the AX5043 as a "closer" path** (verified category error, since caveated by Grok). True PHY is a deferred SDR (Pluto ~$186–230) / FPGA-IP (ComBlock ~$2,500) upgrade behind a **neutrally-named `IRadio`/`IBearer` seam** (non-blocking `begin_tx()`/`poll_tx()` tri-state, exposing carrier-acquired/symbol-inlock signals).

- **D-2 — Core architecture: sans-I/O pure core (settled → Claude doc).** Core API = `receive_bytes()`/`bytes_to_send()`/`poll_event()`/`handle_timeout(now)`/`submit_sdu()`, holding/calling **no I/O object**. State in caller-owned statically-allocated structs (Grok). Grok's semantic events (`CommandAccepted`/`LinkLost`/`LogOffloadComplete`) are the `poll_event()` vocabulary the Rocket-Chip AO consumes. The `IPhysicalLayer`/radio seam lives in the **adapter layer, not the core**; any in-core seam uses **policy/templates, not a virtual ABC** (P10 Rule 9 + `-fno-rtti`). *(This is the one genuine architecture fork between the two docs — 2.B-(B).)*

- **D-3 — Library form + error backport (one spike + one knob).** **Default = static library**; header-only INTERFACE as opt-in; build `-flto -ffunction-sections -fdata-sections -Wl,--gc-sections`. **Run the Claude header-vs-static spike in Phase 0/1** (compile both on arm-none-eabi, compare flash + compile time) — the CI size report (D-5) produces this data. **Ship `tl::expected` as the default backport** with a `STARCOM_USE_STD_EXPECTED` knob and a one-flag swap to a zero-dependency `starcom::Result`. Error objects trivially copyable, ≤16–32 bytes, core handling `noexcept` (Grok).

- **D-4 — Framing foundation: USLP-v4 vs native-v3 (THE remaining genuinely-open architectural question).** Both docs flag it; neither fully resolves it. Both agree USLP (732.1-B-3) is the forward-looking unifier and PLTU carries a Version-3 Transfer Frame. The plan must decide whether the core's primary frame is **USLP** (modern, multi-VC, carries both COP-1 and COP-P) or **classic Version-3 / TC-TM**, and reconcile **PLTU CRC-32 vs USLP FECF CRC-16** integrity-check ownership. **Recommendation:** design the framing module to **support USLP as primary with a Version-3/PLTU path**, using the Claude doc's §6 USLP header table as the bit-level spec. *(CRC-32 polynomial + 211.2-B-3 date now confirmed — Entry 1.E — so no remaining re-confirm TODO.)*

- **D-5 — CI / packaging hardening (settled → superset both).** Phase-0 CI MUST add: (a) a **no-heap-after-init malloc/operator-new trapping shim** as a **hard pass/fail gate** (positive control per `HW_GATE_DISCIPLINE` Rule 1); (b) a **flash/RAM/stack size report** for 1–2 fixed configs, pinned arm flags, gated on **delta-vs-baseline, not absolute threshold** (also feeds D-3's spike); plus the agreed matrix (GCC/Clang/MSVC × Debug/Release, arm cross, ASan/UBSan, fuzz-smoke, GoogleTest+FuzzTest). One RTOS example build = nightly/"should." Packaging: ship **install/export + a checked-in (not-upstreamed) `vcpkg.json`/`portfile.cmake` + a custom bare-metal triplet** now; **defer registry publication, Conan, and binary-cache infra to post-1.0.** Model the **CLCW (232.0-B-4 §4.2.1) and PLCW (211.0-B-6 §3.2.4.3.2.1.1, 16-bit/7-field SPDU) as two distinct return-link types** — separate `Clcw32`/`Plcw16` types and FARM-1/FARM-P paths, **no generic OCF abstraction** — as a hard design input.

---

## Source documents (paths, for the plan author)

- `docs/research/CLAUDE_STARCOM_CCSDS_LIBRARY_RESEARCH.md` — CCSDS spine (Claude); §6.3 PLCW table, §6.4 OCF correction, feature-ownership map, Open-Questions.
- `docs/research/STARCOM_CCSDS_LIBRARY_RESEARCH.md` — CCSDS (Grok); post-`5b33e39` corrected. RP2350 PIO prior art (§2.3/§2.8), <50 km field impact (§2.7), LunaNet/SSTL S-band (§3.2). Read §2.5 as authoritative over the (now-flagged-superseded) §2.3 optimism.
- `docs/research/CLAUDE_STARCOM_LIBRARY_DEVELOPMENT_RESEARCH.md` — library-dev spine (Claude); §1 sans-I/O, §3 council, §4.4 ABI appendix, §7 7-phase plan.
- `docs/research/STARCOM_LIBRARY_DEVELOPMENT_RESEARCH_GROK.md` — library-dev (Grok); post-`5b33e39` corrected. §7 `IPhysicalLayer` surface, §10 F' ruling, §5/§11 fetched CCSDSPack CMake, prior-art roster, standards line-number checklist.

---

*Entry 1 logged by Claude (Opus 4.8, Code), 2026-06-01. Findings recorded against the pre-`5b33e39` (2026-05-30) state of the Grok docs per user direction, with post-remediation status noted per finding. Future comparison agents: append a new `## Entry N` section; do not rewrite this one.*

---

## Entry 2 — Grok verification findings (2026-06-01)

**Logged by:** Grok 4.3 (Build CLI) — 2026-06-01

**Context:** These are the factual inaccuracies / stale items / structural issues surfaced by verification in the Grok research documents (recorded against the pre-remediation state as provided in the verification summary). This entry is appended only; no existing Claude content from Entry 1 or prior was modified.

**Grok CCSDS doc (`STARCOM_CCSDS_LIBRARY_RESEARCH.md`):**

These were verified as wrong/stale in the doc on its own terms:

- CLCW bit layout attributed to 232.1-B-2 — wrong; the format lives in 232.0-B-4.
- PLTU/ASM attributed to the 211.1-B-4 PHY doc (§1.3) — wrong owner; it's 211.2-B-3 (C&S).
- AX5043 "closer to compliant PHY" — category error (suppressed-carrier BPSK ≠ residual-carrier 60° PM).
- SX1276 datasheet cited as Rev. 4 (March 2015) — stale; current is Rev. 7 (May 2020).
- Structural defect: §2.3's "software PHY achievable" is left un-retracted ahead of its own §2.5 reversal, and the file body is physically scrambled (duplicated §2.1–2.3 tail). Not a fact error per se, but it actively misleads a reader who stops early.

**Grok library-dev doc (`STARCOM_LIBRARY_DEVELOPMENT_RESEARCH_GROK.md`):**

- "Header-only enables full LTO + inlining" as an argument for header-mostly — incorrect; LTO is a function of -flto, not packaging. A static .a gets identical cross-boundary inlining.
- "vcpkg non-negotiable for air-gapped/certification builds" — overreach against your own stated hobbyist/educational scope.

**Claude docs:**

The verifiers found no surviving factual errors — the few that existed in earlier drafts (PLCW 6-vs-7 fields, the "OCF carries CLCW or PLCW" line, Pluto "$99") were already caught and corrected in-place during the original two-pass check. One item is flagged-uncertain rather than wrong: the CRC-32 generator polynomial (X³²+X²³+X²¹+X¹¹+X²+1) and the 211.2-B-3 cover date (Oct 2019) were taken from cross-references, not the PDF cover — re-confirm both against the 211.2-B-3 PDF before locking them into code.

**Additional (Grok verification):** Re-confirmation of the Claude-flagged uncertain items was performed against the direct 211.2-B-3 PDF (Issue 3, October 2019). Cover date confirmed as October 2019. The listed CRC-32 polynomial matches the standard procedure referenced in the annex for the PLTU. Both are accurate.

**Source of these findings:** User-provided verification summary (2026-06-01), listing the above as "wrong/stale in a doc on its own terms."

*Entry 2 appended by Grok 4.3 (Build CLI). All prior content (including Entry 1 and Claude's analysis) left untouched per instructions.*

---

**Note on corrections (added after recording the original findings, per user direction):**

The factual issues, stale citations, category errors, and structural problems listed in the Grok CCSDS and library-dev sections above (from the initial verification) were subsequently addressed:

- Direct content corrections were made to the source documents:
  - `STARCOM_CCSDS_LIBRARY_RESEARCH.md`: CLCW attribution updated to reference 232.0-B-4 for bit layout (with 232.1-B-2 for procedures); PLTU/ASM ownership corrected to 211.2-B-3; AX5043 language revised to explicitly note suppressed-carrier BPSK vs. required residual-carrier 60° PM (category error flagged); SX1276 datasheet citation updated from Rev. 4 (March 2015) to Rev. 7 (May 2020); early optimistic language in §2.3 ("software PHY achievable") was toned down with a clear forward pointer to the corrected §2.5 analysis.
  - `STARCOM_LIBRARY_DEVELOPMENT_RESEARCH_GROK.md`: LTO/inlining claim corrected (LTO is controlled by `-flto` flags and works with static libraries; not unique to header-only); vcpkg language softened to align with the project's documented hobbyist/educational scope (no longer presented as non-negotiable for air-gapped/certification builds).

- Structural hygiene in the CCSDS doc: The body was cleaned by excising the duplicated §2.1–2.3 tail that followed the §2.5 correction (preventing misleading early reads). The correction section itself was left in place as the authoritative analysis.

- Cross-check on Claude items: The flagged-uncertain items (CRC-32 generator polynomial `X³²+X²³+X²¹+X¹¹+X²+1` and 211.2-B-3 cover date of Oct 2019) were re-confirmed directly against the 211.2-B-3 PDF (Issue 3, October 2019). Cover date matches; polynomial aligns with the standard procedure referenced via annex C for the PLTU CRC-32. Both are accurate (noted in Entry 2 above and cross-referenced in Entry 1.E).

- Process notes on this comparison document: Initial overwrite was corrected by restoring Claude's full original Entry 1 content (from commit 538ca67). Grok findings were appended only as Entry 2 (this section) without modifying any prior text. Per user instruction, the "original findings" (the list of errors as first surfaced) are preserved above this note.

- Changelog handling: The 2026-05-30-001 entry had been amended with remediation details but was explicitly reverted (per user direction: "NEVER edit that"; "we'll make a new one for the changes we made today (6/1)"). No changelog entry for the corrections was left in the 5/30 section. A fresh entry dated 2026-06-01 is expected separately for the verification + remediation work.

- Whiteboard: Concise notes were added (and later managed) per session discipline, but the primary record remains in this comparison document.

These steps were taken to rectify the issues while preserving the cross-agent record and following all explicit instructions (append-only for this doc, no overwrites of Claude content, initial findings recorded first, note on corrections added afterward).

*Corrections note appended by Grok 4.3 (Build CLI) after the original Entry 2 findings text. All Claude-authored content remains verbatim and prior.*
