# Starcom primary sources

This folder holds **primary-source PDFs** used while designing Starcom FPGA HDL and the CCSDS stack. It is a working reference shelf, **not** a certification library, an ECSS/NASA compliance kit, or a claim that Rocket-Chip is flight-qualified.

Files here are public, freely redistributable specs that verified as real PDFs (`%PDF` magic, not an HTML error page, size > 50 KB). Markdown research stays in [`starcom/docs/`](../../starcom/docs/); do not treat this tree as a relocation of that work.

## Layout

| Path | What |
|---|---|
| [`fpga/`](fpga/) | NASA / ESA / GSFC FPGA–HDL handbooks |
| [`ccsds/`](ccsds/) | CCSDS Blue Books used this sitting |
| [`../protocols/SPACEWIRE_LITE.md`](../protocols/SPACEWIRE_LITE.md) | SpaceWire-Lite draft (stays put; not copied here) |

CCSDS notes and research remain in [`starcom/docs/`](../../starcom/docs/) (especially `research/ccsds_domain_*.md`). This `ccsds/` directory is the Blue Books themselves.

## HDL pedagogy (JPL C analog)

[`../CODING_STANDARDS.md`](../CODING_STANDARDS.md) already absorbs the JPL C institutional rules. For HDL, the pedagogical analog is:

- **NASA-HDBK-4008 §7.3** — FPGA/PLD design practices (synchronous design, clocks, FSMs, resets, testability)
- **NASA-HDBK-4011** — VHDL style (readability, reviewability, maintainability)

Together they play the same role for VHDL that JPL C / Power of Ten play for flight C: a short, risk-oriented subset you can actually check.

Holzmann *The Power of Ten* is C, not FPGA — URL only: <https://spinroot.com/gerard/pdf/P10.pdf>

JPL C coding standard (lars-lab) did not serve a public PDF from this environment — URL only: <http://lars-lab.jpl.nasa.gov/JPL_Coding_Standard_C.pdf>

## Catalog

| Filename | Official title | Date / issue | Official URL | Why we have it |
|---|---|---|---|---|
| [`fpga/NASA-HDBK-4011.pdf`](fpga/NASA-HDBK-4011.pdf) | VHDL Style Handbook (VHSIC Hardware Description Language) | Baseline, 2022-06-06 | <https://standards.nasa.gov/standard/NASA/NASA-HDBK-4011> | VHDL style rules; HDL analog of JPL C |
| [`fpga/NASA-HDBK-4008.pdf`](fpga/NASA-HDBK-4008.pdf) | Programmable Logic Devices (PLD) Handbook | Baseline w/ CHANGE 2, revalidated 2025-06-18 (approved 2013-12-02) | <https://standards.nasa.gov/standard/NASA/NASA-HDBK-4008> | PLD/FPGA lifecycle + §7.3 design practices |
| [`fpga/ESA-ASIC-001.pdf`](fpga/ESA-ASIC-001.pdf) | VHDL Modelling Guidelines (ASIC/001 Issue 1; Sinander) | Issue 1, September 1994 | <https://microelectronics.esa.int/vhdl/doc/ModelGuide.pdf> | ESA VHDL modelling / testbench requirements |
| [`fpga/GSFC-500-PG-8700.2.7.pdf`](fpga/GSFC-500-PG-8700.2.7.pdf) | Design of Space Flight Field Programmable Gate Arrays (500-PG-8700.2.7B) | Rev B, 2012-08-13 (admin extension copy) | <https://soma.larc.nasa.gov/lws/pdf_files/3.17%20500-PG-8700%202%207%20B%20Admin%20Ext%202%20(2019-20)%20(1).pdf> | GSFC space-flight FPGA design criteria |
| [`ccsds/CCSDS-211.0-B-6.pdf`](ccsds/CCSDS-211.0-B-6.pdf) | Proximity-1 Space Link Protocol—Data Link Layer | Blue Book, Issue 6, July 2020 | <https://ccsds.org/Pubs/211x0b6e1.pdf> | Proximity-1 framing / MAC / session (current issue; 211.0-B-5 404) |
| [`ccsds/CCSDS-211.1-B-4.pdf`](ccsds/CCSDS-211.1-B-4.pdf) | Proximity-1 Space Link Protocol—Physical Layer | Blue Book, Issue 4, December 2013 (e1 file) | <https://public.ccsds.org/Pubs/211x1b4e1.pdf> | Proximity-1 PHY used this sitting |
| [`ccsds/CCSDS-211.2-B-3.pdf`](ccsds/CCSDS-211.2-B-3.pdf) | Proximity-1 Space Link Protocol—Coding and Synchronization Sublayer | Blue Book, Issue 3, October 2019 | <https://ccsds.org/Pubs/211x2b3.pdf> | Proximity-1 C&S (current issue; 211.2-B-2 404) |
| [`ccsds/CCSDS-131.0-B-5.pdf`](ccsds/CCSDS-131.0-B-5.pdf) | TM Synchronization and Channel Coding | Blue Book, Issue 5, September 2023. **Superseded** by 131.0-B-6 (Issue 6, April 2026; not on this shelf; URL below) | <https://public.ccsds.org/Pubs/131x0b5.pdf> | TM coding, including LDPC (2048,1024) and convolutional. Starcom cites now use B-6 section numbers with the B-5 number alongside |
| [`ccsds/CCSDS-133.0-B-2.pdf`](ccsds/CCSDS-133.0-B-2.pdf) | Space Packet Protocol | Blue Book, Issue 2, June 2020 (e2 file) | <https://public.ccsds.org/Pubs/133x0b2e2.pdf> | Space Packet used this sitting |
| [`ccsds/CCSDS-232.0-B-4.pdf`](ccsds/CCSDS-232.0-B-4.pdf) | TC Space Data Link Protocol | Blue Book, Issue 4, October 2021 (TC1 Oct 2023) | <https://public.ccsds.org/Pubs/232x0b4e1c1.pdf> | CLCW field format §4.2.1 |
| [`ccsds/CCSDS-232.1-B-2.pdf`](ccsds/CCSDS-232.1-B-2.pdf) | Communications Operation Procedure-1 | Blue Book, Issue 2, September 2010 (TC1 Apr 2019) | <https://public.ccsds.org/Pubs/232x1b2e2c1.pdf> | FOP-1 / FARM-1 tables |
| [`ccsds/CCSDS-732.1-B-3.pdf`](ccsds/CCSDS-732.1-B-3.pdf) | Unified Space Data Link Protocol | Blue Book, Issue 3, June 2024 | <https://public.ccsds.org/Pubs/732x1b3e1.pdf> | Version-4 frame in a PLTU |
| *(URL only)* [`../protocols/SPACEWIRE_LITE.md`](../protocols/SPACEWIRE_LITE.md) | SpaceWire-Lite draft (Rocket-Chip) | 0.1 Draft, 2026-01-19 | (in-repo) | Draft stays under `standards/protocols/`; not duplicated here |

## URL-only (no PDF in tree)

Download failed, was gated, was undersize, or is C-not-FPGA. Official URLs only; no placeholder files.

| Document | Date / issue | Official URL | Why URL-only |
|---|---|---|---|
| CCSDS 131.0-B-6 TM Synchronization and Channel Coding | Blue Book, Issue 6, April 2026 (Editorial Correction 1, May 2026) | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2026/06/131x0b6ec1.pdf> (listing: <https://ccsds.org/publications/allpubs/entry/4803/>) | Current issue. Not downloaded into the tree; `ccsds.org/Pubs/131x0b6.pdf` and `131x0b6e1.pdf` are 404 |
| CCSDS 235.1-R-1 Space Communications Session Control | **Red Book draft** (not a Blue Book), Issue 1, May 2026 | <https://ccsds.org/review/ccsds-235-1-r-1/> | Draft under agency review; Starcom makes no claim against it |
| CCSDS 211.0-B-5 Proximity-1 Data Link Layer | Blue Book, Issue 5, December 2013 | <https://public.ccsds.org/Pubs/211x0b5.pdf> | 404; superseded. Current issue stored as `CCSDS-211.0-B-6.pdf` |
| CCSDS 211.2-B-2 Proximity-1 C&S | Blue Book, Issue 2, December 2013 | <https://public.ccsds.org/Pubs/211x2b2.pdf> | 404; superseded. Current issue stored as `CCSDS-211.2-B-3.pdf` |
| Gaisler, *A structured VHDL design method* (`structdes.pdf`) | lecture / paper | <https://gaisler.com/doc/structdes.pdf> | URL redirects to homepage; no public PDF >50 KB |
| Gaisler two-process chapter (`vhdl2proc.pdf`) | ~1997 | <https://download.gaisler.com/research_papers/vhdl2proc.pdf> | Public PDF exists but 32 KB (below 50 KB keep threshold) |
| Holzmann, *The Power of Ten* | IEEE Computer, June 2006 | <https://spinroot.com/gerard/pdf/P10.pdf> | C rules, not FPGA; already informs `CODING_STANDARDS.md` |
| JPL Institutional Coding Standard for C | v1.0, 2009-03-03 | <http://lars-lab.jpl.nasa.gov/JPL_Coding_Standard_C.pdf> | lars-lab did not serve a PDF here (TLS/timeout); already informs `CODING_STANDARDS.md` |
| **ECSS-E-ST-50-12C Rev.1** SpaceWire — Links, nodes, routers and networks | 15 May 2019 | <https://ecss.nl/standard/ecss-e-st-50-12c-rev-1-spacewire-links-nodes-routers-and-networks-15-may-2019/> | ECSS typically requires free registration; not treated as freely redistributable. Use with SpaceWire-Lite draft above |
| **ECSS-E-ST-20-40C** ASIC, FPGA and IP Core engineering | 11 October 2023 | <https://ecss.nl/standard/ecss-e-st-20-40c-asic-fpga-and-ip-core-engineering-11-october-2023/> | Gated / registration; no PDF stored |
| **ECSS-E-HB-20-40A** Engineering techniques for radiation effects mitigation in ASICs and FPGAs handbook | 11 October 2023 | <https://ecss.nl/home/ecss-e-hb-20-40a-engineering-techniques-for-radiation-effects-mitigation-in-asics-and-fpgas-handbook/> | Gated / registration; no PDF stored |
| **CCSDS 401.0-B** Radio Frequency and Modulation Systems — Part 1: Earth Stations and Spacecraft | Issue 32, October 2021 (later issues may exist) | <https://ccsds.org/Pubs/401x0b32.pdf> | Earth-station PHY. Not Proximity-1. Not downloaded this sitting. Detail: `starcom/docs/DESIGN.md` note 2026-10-01 |
| **CCSDS 231.0-B** TC Synchronization and Channel Coding | 231.0-B-3, Issue 3, September 2017 (Historical); 231.0-B-4 is the later issue | <https://ccsds.org/Pubs/231x0b3s.pdf> (also <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/03//231x0b3s.pdf>; Wayback <https://web.archive.org/web/20210210221900/https://public.ccsds.org/Pubs/231x0b3.pdf>; do not cite `ccsds.org/Pubs/231x0b3.pdf`, 404) | Telecommand coding companion to 401. Not on this shelf. Only the 6.2 / 6.3.1 randomizer text was read (cited by 211.2-B-3 as [E3]); the rest is not walked |
| **RCC 106** Telemetry Standards (IRIG 106) | 106-22, May 2022; wiki tracks later issues | <https://www.irig106.org/wiki/irig_106-22> | US range ascent telemetry. Chapter 2: PCM/FM, SOQPSK-TG, ARTM CPM. Not CCSDS |
| **RCC 319** Flight Termination Systems Commonality Standard | 319-25, June 2025 | <https://www.trmc.osd.mil/wiki/download/attachments/113019893/319-25_FTS_Commonality.pdf> | Dedicated termination uplink, independent of the telemetry radio. Tones and ordnance stay in that book |
| **NASA/TP–20230006855** NASA Sounding Rockets User Handbook (810-HB-SRP) | May 2023 | <https://www.nasa.gov/wp-content/uploads/2015/03/sounding-rocket-user-handbook.pdf> | Umbilical circuit classes and Wallops PCM encoders. No universal pin number list. Circuits: `docs/hardware/HARDWARE.md` Regulatory Notes |

## Provenance notes

- NASA handbooks: “Approved for public release — distribution is unlimited.”
- CCSDS Blue Books: public recommended standards from <https://public.ccsds.org/Pubs/> (some hosts now redirect to `ccsds.org/Pubs/`).
- ESA ASIC/001: public from ESA Microelectronics (`microelectronics.esa.int`).
- GSFC 500-PG-8700.2.7B: public copy hosted by NASA LaRC SOMA / LWS.
- Do not scrape ECSS logins. If a future sitting needs those PDFs, download them yourself from ecss.nl after registration.

Collected 2026-08-25. Do not commit from the collector session unless a later human asks.
