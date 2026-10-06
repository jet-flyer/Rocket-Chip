# CCSDS / Link-Layer Deviation Survey: Open CubeSat and Ground-Station Projects

## Issue check (2026-10-05, Nathan's rule)
Every rule here rests on the current Blue Book: 732.1-B-3 (no draft on ccsds.org/review), 211.0-B-6, 211.1-B-4, 232.0-B-4 incl. Cor 1, 232.1-B-2 incl. Cor 1. Drafts checked: 211.0-P-6.2, 211.1-P-4.2.
Any older issue in this file (732.1-B-1/B-2, 131.0-B-3/B-4, etc.) is HISTORY: it records what a project itself cites, not a rule we follow.
- 232.1-B-2 §5.1.12 (1 ≤ K ≤ PW): holds in the current issue. Cor 1 adds "NOTE – Although 1 ≤ K ≤ PW, the value of K may never exceed 255."
- USLP FECF 16 bits: current rule is 732.1-B-3 §4.1.6.2.2. B-2 is named only because Yamcs/OreSat/spacepackets-py cite it.
- CHANGED IN DRAFT: Prox-1 near-Earth text. 211.1-B-4 §3.3.1 has both "430 MHz cannot be used ... in the vicinity of the Earth" and the layering provision for near-Earth use. 211.1-P-4.2 §3.3.1 drops BOTH sentences. It keeps "designed primarily for use in a Proximity link space environment far from Earth" and adds "the frequency bands for which the use of Proximity-1 is foreseen (UHF and S-Band)".

Prepared for: Starcom / Rocket Chip (Nathan Powell)
Date: 2026-10-05 (CT)
Status: **REFERENCE ONLY. Nothing here is decided for Starcom.**
Readers: Duke maps clauses. Hamilton logs LOOK rows. Buzz owns PHY and radio.

## Rules used
- A deviation is listed only when the project states it in writing (docs, README, spec, paper, wiki).
- Every quote has a direct URL. Commit hashes and dates are pinned where possible. Dates are in CT.
- If code looks non-compliant but the docs do not say so, the project goes under "no stated deviation found."
- PHY and band are noted only when the project states them. Buzz will go deeper.
- License is noted in one line. GPL, AGPL, or no license = **reference-only flag** (do not copy code).

---

## Framing question: did OreSat/UniClOGS get around Prox-1's "not for Earth" limit by changing band?

Short answer from sources: **No. OreSat never used Prox-1, so it had no Prox-1 limit to get around.** The OreSat EDL docs say: "The EDL uses USLP (Unified Space Link Protocol) from CCSDS". They list USLP, CFDP, COP-1, TC SDLP, and the 130.0 Green Book as references. Prox-1 (211.x) is not in that list. [1]

USLP is in scope for this kind of link. 732.1-B-3 §1.1 says USLP is "to be used over space-to-ground, ground-to-space, or space-to-space communications links by space missions." [2] OreSat runs it over its own GFSK radio, not a CCSDS PHY. UniClOGS says: "The emission type is GFSK at 50 kb/s". The bands are UHF 435–438 MHz and L-band 1260–1270 MHz uplink. [3]

One nuance for Duke. The Prox-1 "not near Earth" text is about the 430 MHz frequencies in the Prox-1 **Physical Layer** (211.1-B-4 §3.3.1). The same section says "provision is made to change only the Physical Layer by adding other frequencies to enable the same protocol to be used in near Earth applications". [4] So the Prox-1 standard itself allows a band change. We just have no source saying OreSat considered it.

**Current USLP issue (checked on ccsds.org 2026-10-05):** CCSDS 732.1-B-3, Blue Book, Issue 3, June 2024. [2][5] The OreSat EDL docs still cite **732.1-B-2**. [1] An open SLP WG thread (Oct 2025) proposes edits to B-3. It is not a new issue. [6]

---

## 1. OreSat C3 EDL / UniClOGS (Portland State Aerospace Society)

**Primary URLs**
- EDL spec: https://oresat-c3-software.readthedocs.io/en/latest/edl.html
- Doc source pinned: https://github.com/oresat/oresat-c3-software/blob/55eedb42ca47a29ece299f023854e89b97bb61bf/docs/edl.rst (last commit touching edl.rst: 2026-08-02 CT, "docs: update EDL docs with COP-1 details")
- UniClOGS paper SSC24-WII-01: https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=5829&context=smallsat (PDF created 2024-07-26, modified 2025-07-30)
- UniClOGS hardware README: https://github.com/oresat/uniclogs-hardware/blob/5c0cd713d93a6fb207da5bc97d79922ea797971a/README.md (2026-03-01 CT)

**Stack claimed**
- "The EDL uses USLP (Unified Space Link Protocol) from CCSDS ... It uses SDLS (Space Data Link Security) for authentication and COP-1 (Communication Operation Procedure-1) for frame retransmission."
- File transfer: "The EDL uses CCSDS File Delivery Protocol (CFDP) for file transfer. The CCSDS PDU packets will be used as the payload of the main USLP packet."
- References listed: 130.0-G-4, **732.1-B-2**, 727.0-B-5, 232.1-B-2 (COP-1), 232.0-B-4 (TC SDLP).
- The paper says only "Engineering Data Link (EDL) packets based on CCSDS standards". The beacon is "based on APRS/AX.25 packet standards". The paper does not name USLP.

**Stated deviations (quoted, edl.rst lines 80 and 107)**
- "Though out of spec, the SDLS Header is currently implemented using the USLP insert zone."
- "Though out of spec, the SDLS Trailer is currently inserted into the end of data zone."
- Stated profile choices. These are stated, but the project does not call them deviations:
  - "MAP ID: 4 bits. Not used by OreSat (will always be 0b0000)."
  - "VC Frame Count Length ... set to 0b000 for no VCF Count bits."
  - SDLS trailer = "32 octets HMAC". SDLS SPI = 1 means "the oresat sdls implemenation" (sic).
  - Anti-replay: a 32-bit sequence number. Any EDL packet "must have a higher number that the C3 internal count, otherwise the C3 will ignore it."
  - VC 2 is an "IDLE channel carrying frames with CLCWs, but no payload or SDLS."
- For Duke: 732.1-B-3 §6.3.4 says the Security Header "shall follow, without gap, the Transfer Frame Insert Zone if a Transfer Frame Insert Zone is present". §6.3.6 says the Security Trailer "shall follow, without gap, the TFDF." [2] That is the clause context for the two "out of spec" notes. Duke should map the exact clauses.
- Doc wording oddities (not stated deviations; listed only so Duke does not trip on them): TFVN is described as "Always 'C' in ASCII" in a 4-bit field. UPID is given as `0b000101` (6 digits) for a 5-bit field.

**PHY/band (stated)**
- EDL diagram: "EDL UHF Uplink", "EDL L Band Uplink", "EDL UHF Downlink" (edl.rst lines 17–19).
- UniClOGS README: UHF "435 to 438 MHz"; L-band "1260 to 1270 MHz ... designated for uplink only"; "The emission type is GFSK at 50 kb/s, and was chosen for the capability of the Engineering transceiver radio on the spacecraft, an acceptable performance indicated by the link budget, and is somewhat common in the amateur satellite community."
- Paper: "For the OreSat CubeSat system modes L/u and U/u are used."

**License:** oresat-c3-software = GPL-3.0. uniclogs-hardware = GPL-3.0. **Reference-only flag.**

**Relevance to Starcom:** A fielded example of USLP + COP-1 + SDLS-style HMAC over a non-CCSDS GFSK PHY. It writes down its own "out of spec" SDLS placement.

---

## 2. SatNOGS-COMMS (Libre Space Foundation)

**Primary URLs**
- ICD (in the design doc repo): https://gitlab.com/librespacefoundation/satnogs-comms/satnogs-comms-design-doc/-/blob/f9deed9c29c8bee9744968fc0c6c445a844c9bbe/ICD.tex#L438-443 (HEAD 2026-06-05 CT)
- MCU software: https://gitlab.com/librespacefoundation/satnogs-comms/satnogs-comms-software-mcu (HEAD 060025a0, 2026-09-30 CT)
- Yamcs integration README: https://gitlab.com/librespacefoundation/satnogs-comms/satnogs-comms-software-mcu/-/blob/master/contrib/yamcs/README.md

**Stack claimed (ICD §"RF Functionality", lines 439–443)**
- "Modulation: GMSK/GFSK, BPSK, QPSK"
- "Tx: ECC, CCSDS CC1/2 w/ and w/o RS, Rx: only CCSDS RS"
- "Framing encapsulation: CCSDS, IEEE 802.15.4"
- "Data Rates: UHF, up to 50 kbps, S-Band, up to 1 Mbps"
- TM/TC definitions: "The default firmware of the SatNOGS-COMMS utilizes the CCSDS XTCE to describe the available telecommands and telemetry responses, as well as the internal structure of each frame."

**Stated deviations / subsets**
- No CCSDS book or issue numbers are given for framing. "CCSDS" framing is not tied to TM, AOS, or USLP in the docs I found.
- One stated asymmetry: "Rx: only CCSDS RS" (Tx offers CC 1/2 with or without RS).
- No "out of spec", subset, or PICS text found. I searched the ICD, design-development plan, and the READMEs of the design-doc, MCU, libsatnogs-comms, satnogs-comms-yamcs, and gateway repos. **None stated.**

**PHY/band (stated):** The ICD has UHF test data at "401 and 435MHz" and S-band at "2200MHz and 2290MHz". Buzz to deepen.

**License:** MCU software GPL-3.0-or-later. libsatnogs-comms GPL-3.0-only. satnogs-comms-yamcs GPL-3.0-or-later. Design doc CC-BY-SA-4.0. **Reference-only flag** (software).

**Relevance to Starcom:** An open radio that offers both CCSDS and 802.15.4 framing. It states the coding split but not which CCSDS book it uses. Treat it as a "claims CCSDS, no PICS" example.

---

## 3. AcubeSAT ccsds-space-data-link-protocols

**Primary URLs**
- GitHub mirror README: https://github.com/AcubeSAT/ccsds-space-data-link-protocols/blob/d55722f2d474ce6dc952cddbb34b6c39b8d06466/README.md (main HEAD, 2024-09-10 CT)
- GitLab origin: https://gitlab.com/acubesat/comms/software/ccsds-data-link-layer (same HEAD d55722f2; the old path `ccsds-telemetry-packets` redirects here)

**Stack claimed**
- "Implementation of the CCSDS TM and TC Data Link standards (232.0-B-4, CCSDS 232.1-B-2, 132.0-B-3)."

**Stated deviations / subsets**
- "(Note: There is no planned support for SDLS)"
- Wiki: the README points to `.../-/wikis/Creating-a-Service-Channel` "(WIP)". The GitLab wiki API returns **zero pages** for this project (checked 2026-10-05). There are no public wiki notes to capture. **Nothing else stated.**

**PHY/band:** Not stated in this repo.

**License:** MIT. Fine to copy with attribution.

**Relevance to Starcom:** A clean, explicit statement of which books and issues it covers, plus an explicit "no SDLS" exclusion. A good model for a one-line scope statement.

---

## 4. LibreCube: Reference Architecture docs

**Primary URLs**
- Communication protocols: https://gitlab.com/librecube/librecube.gitlab.io/-/blob/8739a6ffcb4e6e7650f809713c88ed66a3222a2d/docs/reference_architecture/communication_protocols/index.md (HEAD 2026-09-15 CT; rendered at https://librecube.gitlab.io)
- Remote segment: https://gitlab.com/librecube/librecube.gitlab.io/-/blob/8739a6ffcb4e6e7650f809713c88ed66a3222a2d/docs/reference_architecture/remote_segment/index.md
- Related libs: https://gitlab.com/librecube/lib/python-spacepacket, https://gitlab.com/librecube/lib/python-cfdp, https://gitlab.com/librecube/prototypes/proto-uhf-groundstation

**What they claim**
- Remote segment: the comms receiver decodes "the contained CCSDS telecommand frames". Downlink: "CCSDS telemetry frames are received through UART A or B and then encoded and modulated". No book or issue is named.
- python-cfdp: "It supports all features as outlined in the latest version of the CFDP Blue Book" (links 727x0b5). "The protocol is tested for cross-support as outlined in CCSDS CFDP Yellow Book."
- python-spacepacket: links 133x0b1c2. "Currently, only transport over UDP is implemented."
- proto-uhf-groundstation: "send and receive CCSDS Packets over radio link. It uses SDRs ... interfaces the Yamcs mission control system". No framing detail.

**What is "to be written" (quoted from communication_protocols/index.md)**
- "Comms Bus Protocol — To be written..."
- "Space Data Link Protocols — To be written..."
- "Space Packet Protocol — To be written..."
- "CCSDS File Delivery Protocol (CFDP) — To be written..."
- "ECSS Packet Utilization Protocol (PUS-C) — To be written..."
- "Coding, Modem, RF — To be written..."
- "Space Link Extension Protocol — To be written..."
- Payload bus: "To protocol for data sending over this bus is **TO BE DEFINED**."
- proto-uhf-comms README: "UHF Communications Module, Version 0 — To be written..."

**Stated deviations:** **None stated.** The link-layer sections are placeholders.

**License:** python-spacepacket MIT. python-cfdp MIT. proto-uhf-groundstation README says MIT (the GitLab API shows no license detected). proto-uhf-comms CERN-OHL-W.

**Relevance to Starcom:** An architecture that intends to use CCSDS framing but has no written link-layer profile yet. It shows the gap we should avoid. Its MIT CFDP and SPP libraries are copy-friendly.

---

## 5. OpenLST (Planet heritage)

**Primary URLs**
- USERS_GUIDE: https://github.com/OpenLST/openlst/blob/b996935a516967936859634887fdf7b4d48dcc7c/open-lst/USERS_GUIDE.md (master HEAD, 2018-08-03 CT)
- README: https://github.com/OpenLST/openlst/blob/master/README.md

**Stack claimed**
- "packet-based communication 437MHz at 3kbps and includes forward error correction (FEC), CRC-based integrity checks, basic radio telemetry, as well as support for in-situ or over-the-air firmware updates."
- Its own header fields: HWID ("a 2-byte serial number") and SEQNUM ("a 2-byte number used to match commands and responses").
- Non-features: "Message encryption or authentication; users who need this will need to implement it at the application or protocol layer." Also no beaconing and no full duplex.

**Stated deviations / reasons for not using CCSDS:** The USERS_GUIDE and README never mention CCSDS or AX.25. They give **no stated reason** for choosing a non-CCSDS stack. Its value is heritage: "Planet has used its LST UHF radio on over 200 Dove satellites."

**PHY/band (stated):** 437 MHz, "2FSK", "7.4 kBaud" in the spec table. Note: the intro says "3kbps" and the table says 7.4 kBaud. Buzz should reconcile.

**License:** GPL-3.0. **Reference-only flag.**

**Relevance to Starcom:** A flight-proven non-CCSDS stack with no stated rationale. Use it only as the "chose non-CCSDS" contrast. Its auth-free link is a stated non-feature.

---

## 6. OpenCCSDS (oresat/OpenCCSDS)

**Primary URLs**
- https://github.com/oresat/OpenCCSDS (master HEAD 87cae72e, 2022-03-30 CT)

**Stack claimed:** Repo description "CCSDS Protocol Stack". There is no README (404). The build file lists `uslp.c`, `cop.c`, `spp.c`, `sdls.c`, `frame_buf.c`.

**Stated deviations:** **None stated.** There are no docs. (Per the rules, I made no code review for compliance.)

**License:** **No license file.** GitHub reports none. **Reference-only, read-only.** Stale since 2022.

**Relevance to Starcom:** An early OreSat USLP/COP/SDLS attempt. Read for history only.

---

## 7. Other open peers found

### 7a. Yamcs: CCSDS Frame Processing docs (strongest "honest PICS-like" source found)
**URLs:** https://docs.yamcs.org/yamcs-server-manual/links/ccsds-frame-processing/ ; source pinned: https://github.com/yamcs/yamcs/blob/006c0d963a89ca19e581d9015fa81b225172e350/docs/server-manual/links/ccsds-frame-processing.rst (last edit 2026-02-04 CT)

**Stack claimed:** "Yamcs support for parts of" TM 132.0-B-3, AOS 732.0-B-4, TC 232.0-B-4, **USLP 732.1-B-2**, TC S&C 231.0-B-4, TM S&C 131.0-B-4, COP-1 232.1-B-2, SPP 133.0-B-2, Encapsulation 133.1-B-3, SDLS 355.0-B-2.

**Stated deviations / subsets (quoted)**
- "Yamcs supports to a certain extent all three of them [AOS, TM, USLP]. The main support is around the 'packet service'".
- "The MAP service is only supported for TC, not for USLP."
- "Currently Yamcs ignores any data in the insert zone."
- "For the moment only the Reed-Solomon codec is supported." (raw frame decoder)
- "Yamcs does not support segmentation (i.e. splitting a TC packet over multiple frames)".
- TC VC service: "Currently the only supported option is PACKET".
- `skipRandomizationForVcs`: "This is not as per CCSDS standard which specifies that the randomization is enabled/disabled at the physical channel level."
- Per-VC `errorDetection`: "This is not according to the CCSDS standard which specifies the frame error detection shall be configured at physical channel level."
- `cltuStartSequence` / `cltuTailSequence` can be set "if different than the CCSDS specs."
- Built-in SDLS: AES-256-GCM-128 only. It has a fixed list of managed parameters (IV 12 octets, sequence number 4 octets, MAC 16 octets, "padding is not used").

**License:** AGPL-3.0. **Reference-only flag.**
**Relevance to Starcom:** A model for how to write deviations. Each one is marked "not as per CCSDS" and names the clause owner (physical channel vs VC). It is also the likely ground software, and it ignores insert-zone data. That matters if anyone copies OreSat's SDLS-in-insert-zone layout.

### 7b. NASA CryptoLib (SDLS / SDLS-EP)
**URLs:** https://github.com/nasa/CryptoLib ; wiki https://github.com/nasa/CryptoLib/wiki (Home last edited 2024-07-17 CT)
**Stack claimed:** "C-based software-only implementation of the CCSDS Space Data Link Security Protocol (SDLS), and SDLS Extended Procedures (SDLS-EP)". "Specific communications protocols that are supported include: Telecommand (TC), Telemetry (TM), Advanced Orbiting Systems (AOS)". The wiki links 355x0b1 and 355x1b1.
**Stated deviations:** **None stated.** USLP is not in its stated supported list. That is a scope statement, not a stated deviation.
**License:** NASA Open Source Agreement 1.3. Not GPL, but NOSA has its own terms. Flag for review before copying.
**Relevance to Starcom:** A reference SDLS implementation. Its stated scope is TC/TM/AOS, not USLP.

### 7c. Bifrost (CubeSat ground services built on NASA AIT)
**URL:** https://github.com/Mejiro-McQueen/Bifrost (main, last commit 2024-04-14 CT)
**Stated:** "adhering to the CCSDS standards as much as possible." Also: "many of the Bifrost libraries are not fully CCSDS compliant, bug free, or feature complete." It does not list which items are non-compliant.
**License:** MIT.
**Relevance to Starcom:** A blanket non-compliance disclaimer with no list. A counter-example of what a deviation log should look like.

### 7d. SPUTNIX SX-USP (FEC/framing protocol)
**URL:** https://github.com/sputnixru/SX-USP (master README)
**Stated:** Design goals include "Max. CCSDS-compatibility is a plus." and "AX.25-compatibility is a plus." It uses its own 64-bit sync `0x5072F64B2D90B1F5` and a DVB-S2/CCSDS 131.2 PLS code. RS, scrambling, and convolutional coding are "as per CCSDS 131.0-B-3". The payload is AX.25 in an EtherType-like tag: "No bit-stuffing, no HDLC, no CRC besides RS." Also: "No interleaving yet."
**License:** None detected on GitHub. **Reference-only flag.**
**Relevance to Starcom:** CCSDS coding blocks reused under a non-CCSDS frame and sync. Mostly Buzz territory.

### 7e. PicSat (Obs. de Paris) packet format
**URL:** https://picsat.obspm.fr/communication/ccsds-packets?locale=en
**Fetch status:** **Live fetch failed.** WebFetch returned HTTP 500, and curl failed on an expired TLS certificate. The Wayback Machine has no snapshot. The quote below is from the search-engine index only. **Verify before citing.**
**Stated (index snippet):** "The packet format used within the PicSat mission is largely based on the CCSDS ... standard. However, the standard has been modified for the particular needs of a small CubeSat mission." Fields like `sequence_flag` are listed as "Unused for PicSat. Set to 0b11."
**License:** N/A (web doc).
**Relevance to Starcom:** A packet-layer (SPP) deviation stated openly at field level.

### 7f. spacepackets-py / spacepackets-rs (IRS Univ. Stuttgart)
**URLs:** https://github.com/us-irs/spacepackets-py ; https://github.com/us-irs/spacepackets-rs
**Stated:** spacepackets-py: "Unified Space Data Link Protocol (USLP) frame implementations according to CCSDS Blue Book 732.1-B-2." I checked the README and the readthedocs USLP API page. **No stated deviation found.**
**License:** Apache-2.0 (LICENSE file checked; spacepackets-rs also ships LICENSE-APACHE).
**Relevance to Starcom:** A permissive USLP frame library. It cites B-2, not the current B-3.

---

## No stated deviation found (summary)
| Project | What docs claim instead |
|---|---|
| SatNOGS-COMMS | "Framing encapsulation: CCSDS, IEEE 802.15.4"; no book/issue; only stated subset is "Rx: only CCSDS RS" |
| LibreCube ref. arch. | Link-layer sections "To be written..." |
| OpenLST | Own packet format; no CCSDS mention; no stated rationale |
| OpenCCSDS | No README, no license |
| CryptoLib | SDLS for TC/TM/AOS (USLP not listed) |
| spacepackets-py | USLP per 732.1-B-2; no deviations listed |

---

## Summary table (LOOK only, nothing decided)

| Project | Stated deviation | Source URL | Suggested Starcom parallel (LOOK only, not decided) |
|---|---|---|---|
| OreSat C3 EDL | "Though out of spec, the SDLS Header is currently implemented using the USLP insert zone." | https://github.com/oresat/oresat-c3-software/blob/55eedb42ca47a29ece299f023854e89b97bb61bf/docs/edl.rst#L80 | LOOK: if we put USLP + SDLS on a custom PHY, log any SDLS field placement against 732.1-B-3 §6.3.4 |
| OreSat C3 EDL | "Though out of spec, the SDLS Trailer is currently inserted into the end of data zone." | https://github.com/oresat/oresat-c3-software/blob/55eedb42ca47a29ece299f023854e89b97bb61bf/docs/edl.rst#L107 | LOOK: trailer vs TFDF boundary, §6.3.6 |
| OreSat C3 EDL | Cites USLP 732.1-B-2 (current is B-3, June 2024); MAP ID unused; VCF count length 0; HMAC-32 octets; SPI=1 "oresat sdls implemenation" | https://oresat-c3-software.readthedocs.io/en/latest/edl.html | LOOK: pin the issue number in our own profile; list managed-parameter choices |
| UniClOGS | None stated (paper says "EDL packets based on CCSDS standards"; GFSK 50 kb/s stated in hardware README) | https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=5829&context=smallsat ; https://github.com/oresat/uniclogs-hardware/blob/5c0cd713d93a6fb207da5bc97d79922ea797971a/README.md | LOOK: USLP over non-CCSDS GFSK PHY (hand to Buzz) |
| SatNOGS-COMMS | None stated; subset "Rx: only CCSDS RS"; framing "CCSDS, IEEE 802.15.4" with no book | https://gitlab.com/librespacefoundation/satnogs-comms/satnogs-comms-design-doc/-/blob/f9deed9c29c8bee9744968fc0c6c445a844c9bbe/ICD.tex#L438-443 | LOOK: name the CCSDS book/issue for any framing we claim |
| AcubeSAT SDLP | "(Note: There is no planned support for SDLS)"; scope = 232.0-B-4, 232.1-B-2, 132.0-B-3 | https://github.com/AcubeSAT/ccsds-space-data-link-protocols/blob/d55722f2d474ce6dc952cddbb34b6c39b8d06466/README.md | LOOK: one-line scope + exclusions statement format |
| LibreCube | None stated; "Space Data Link Protocols — To be written..." | https://gitlab.com/librecube/librecube.gitlab.io/-/blob/8739a6ffcb4e6e7650f809713c88ed66a3222a2d/docs/reference_architecture/communication_protocols/index.md | LOOK: avoid placeholder link-layer docs |
| OpenLST | None stated (non-CCSDS; no rationale). Non-feature: no encryption/authentication | https://github.com/OpenLST/openlst/blob/b996935a516967936859634887fdf7b4d48dcc7c/open-lst/USERS_GUIDE.md | LOOK: contrast case for "non-CCSDS stack" |
| OpenCCSDS | None stated (no docs, no license) | https://github.com/oresat/OpenCCSDS | LOOK: read-only history |
| Yamcs | "The MAP service is only supported for TC, not for USLP." / "Currently Yamcs ignores any data in the insert zone." / "Yamcs does not support segmentation" | https://github.com/yamcs/yamcs/blob/006c0d963a89ca19e581d9015fa81b225172e350/docs/server-manual/links/ccsds-frame-processing.rst | LOOK: ground-side limits that constrain the onboard profile (no USLP MAPs, insert zone ignored) |
| Yamcs | "This is not as per CCSDS standard which specifies that the randomization is enabled/disabled at the physical channel level." / per-VC errorDetection "not according to the CCSDS standard" | same as above | LOOK: deviation-log wording template |
| NASA CryptoLib | None stated; scope "TC, TM, AOS" | https://github.com/nasa/CryptoLib/wiki | LOOK: check SDLS library scope vs USLP |
| Bifrost | "many of the Bifrost libraries are not fully CCSDS compliant, bug free, or feature complete." | https://github.com/Mejiro-McQueen/Bifrost | LOOK: counter-example (blanket disclaimer, no list) |
| SPUTNIX SX-USP | Own sync + AX.25 payload over CCSDS 131.0 coding; "No interleaving yet." | https://github.com/sputnixru/SX-USP | LOOK: Buzz, CCSDS coding under non-CCSDS framing |
| PicSat | "the standard has been modified for the particular needs of a small CubeSat mission" (UNVERIFIED: live fetch failed) | https://picsat.obspm.fr/communication/ccsds-packets?locale=en | LOOK: field-level SPP deviation table format |
| spacepackets-py | None stated; USLP per 732.1-B-2 | https://github.com/us-irs/spacepackets-py | LOOK: permissive USLP lib, check B-3 delta |

---

## References
1. OreSat EDL: https://oresat-c3-software.readthedocs.io/en/latest/edl.html (source commit 55eedb42, 2026-08-02 CT)
2. CCSDS 732.1-B-3 USLP, June 2024: https://ccsds.org/Pubs/732x1b3e1.pdf (§1.1, §6.3.1, §6.3.4, §6.3.6)
3. UniClOGS hardware README: https://github.com/oresat/uniclogs-hardware/blob/5c0cd713d93a6fb207da5bc97d79922ea797971a/README.md
4. CCSDS 211.1-B-4 Prox-1 Physical Layer §3.3.1: https://ccsds.org/Pubs/211x1b4e1.pdf ; Prox-1 DLL 211.0-B-6 §1.1: https://ccsds.org/Pubs/211x0b6e1.pdf
5. ccsds.org publication entry: https://ccsds.org/publications/allpubs/entry/3287/
6. SLS-SLP list, "Proposed Changes to 732.1-B USLP" (Oct 2025): https://mailman.ccsds.org/pipermail/sls-slp/2025-October/001315.html

## Fetch log / gaps
- AcubeSAT GitHub page via WebFetch returned only metadata. The raw README worked (main branch).
- SatNOGS-COMMS GitLab web page returned no content via WebFetch. I used the GitLab API raw files instead.
- AcubeSAT GitLab wiki: the API lists 0 pages. No public wiki content.
- PicSat: HTTP 500 plus an expired TLS cert. No Wayback snapshot. The quote is from the search index only.
- GitHub REST API hit its rate limit mid-session. I used raw.githubusercontent and the GitHub connector for pins.
- No OreSat document mentions Prox-1. I searched the oresat org code: the only hits are a SANA UPID table in OpenCCSDS `uslp.h`.

---

## Then vs now (2026-10-06)

Reference only. Nothing here is decided.

Prepared for: Nathan Powell. Date: 2026-10-06 (CT). This section adds three checks to the rows above. It does not change the rows above.

### How to read this section

- Each row checks one stated project item. The check compares the issue the project followed with the current Blue Book issue and with a draft.
- Row IDs 1–41 come from deviation-clause-map.md (Duke). X1 and X2 come from check2-deviation-by-issue.md (Duke). IDs that start with "R" are new in this section. An R row holds a stated reason or a survey row that has no map row.
- The "Row key" table below links each row of the summary table above to these IDs.
- Sources: **G** = ccsds-deviation-rationale.md, last section (Goddard: check 1 and check 3) and its project tables. **D** = check2-deviation-by-issue.md (Duke: check 2). **B** = check3-radio-reasons.md (Buzz: radio reasons). **M** = deviation-clause-map.md. **D2** = check2-R-rows.md (Duke: check 2 for rows R2, R3, R4, R6, R7, R8; posted 2026-10-06). **Room** = the facts in the next list.
- Check 1 column: the project date, the issue the project followed, and the current issue.
- Check 2 column: verdict under their issue / verdict under the current issue / draft. Words: Allowed, Deviation, Open, N/A. "Open" means the book text is clear, but the verdict needs a project fact that the project does not state, or the verdict rests only on INFERENCE (D). D2 adds: or the only clashing text is informative. "UNVERIFIED" means the source could not get the text or fact. Then the clause.
- Draft: D re-read https://ccsds.org/review/ on 2026-10-06 01:30 CT. It holds no 732.1, 355.0, 132.0, 231.0 or 232.x draft. 131.0-P-5.1 review closed before 131.0-B-6. So every row in Table A has "no draft". D2 re-read the review page on 2026-10-06 10:43 CT. It lists no 732.1, 132.0, 231.0 or 232.x draft. So every R row that D2 checked also has "no draft".
- Check 3 column: Yes, No, Partly, or Judgment call. Then one short note and the basis.
  - "N/A (no stated reason)" means the source files found no stated reason. There is nothing to re-check.
  - "not checked" means no source file has data for that cell.
  - Basis "source-verified" means a team member read the project code or package. Basis "doc-only" means docs, a datasheet, a poster, or a web list. B: nothing is board-measured, and datasheet revisions are not pinned.
- **INFERENCE** marks reasoning, not a checked fact. This section keeps every INFERENCE label from its source.
- D caveat: the 732.1-B-2 text comes from a Wayback copy (capture 20220717230159). CCSDS serves no live B-2 copy. It applies to rows 1–4, 7, 8, 10–19, 23–26, 29, 41 and X1.

### Room facts used (2026-10-06)

1. Goddard confirmed row X2 in current AcubeSAT code: branch Tc-Rx-Unit-Testing, HEAD 3bd340b (2026-09-18), `inc/SecurityAssociation.hpp` line 62, `HMAC_SHA256_40_BIT` `macFieldLength = 5`. Their own `inc/CcsdsDefinitions.hpp` lines 426–427 set `MinMACLength = 8`. AcubeSAT does not call this a deviation.
2. Yamcs 5.13.6 (2026-10-01) still cites 732.1-B-2, 131.0-B-4 and 732.0-B-4. The current issues are 732.1-B-3, 131.0-B-6 and 732.0-B-5.
3. The OSDLP claim that TM and TC are "superseded by" USLP has no support. ccsds.org lists 132.0-B-3 and 232.0-B-4 as active.
4. spacepackets-py 0.32.0 (May 2026) still has no SDLS in USLP frames.
5. The Yamcs packet service still rejects UPID ≠ 0. Yamcs has a VCA service with a custom handler. The OreSat "1000 lines" figure is not verified.
6. Goddard confirmed row R2-b in OreSat code at HEAD 1136e31 (2026-08-17). `make_frame` in `oresat_c3/protocols/uslp.py` line 129 always sets `SPACE_PACKETS_ENCAPSULATION_PACKETS`. The code comment is "yamcs does not support user defined octet stream". `docs/edl.rst` line 87 says `0b000101`. So R2-b is a split between docs and code (source-verified).
7. D2 correction: the AcubeSAT design doc of 2021-05-17 names 232.0-B-3, not 232.0-B-4. Row R7 check 1 is fixed.

### Row key: summary table rows → row IDs

| Summary table row (project: stated item) | Row IDs |
|---|---|
| OreSat C3 EDL: SDLS Header in the insert zone | 1–6 |
| OreSat C3 EDL: SDLS Trailer in the data zone | 7–9 |
| OreSat C3 EDL: cites 732.1-B-2; MAP ID; VCF count; HMAC; SPI | 13, 14, 15, 21, 20 (map also adds 10–12, 16–19, 22 from edl.rst); R2, R2-b |
| UniClOGS | R1, R3 |
| SatNOGS-COMMS | 34–36 |
| AcubeSAT SDLP | 30–33, X2, R7 |
| LibreCube | R9 |
| OpenLST | R5 |
| OpenCCSDS | R10 |
| Yamcs: MAP service, insert zone, segmentation | 23–29 |
| Yamcs: randomization per VC, errorDetection per VC | R8, R8-a, R8-b, R8-c (CRC32 FECF option: X1, not in the summary table) |
| NASA CryptoLib | 40 |
| Bifrost | R11 |
| SPUTNIX SX-USP | 37–39, R6 |
| PicSat | R12 |
| spacepackets-py | 41 |
| OSDLP (no row in the summary table; from G) | R4 |

### Table A: map rows 1–41, X1 and X2

| ID | Project: item | Project date + issue followed (check 1) | Deviation under their issue / current issue / draft (check 2) | Stated reason still true today? (check 3) |
|---|---|---|---|---|
| 1 | OreSat: SDLS Header in the Insert Zone | edl.rst pin 2026-08-02. Cites 732.1-B-2 (2020 plan: 732.1-B-1). Current: 732.1-B-3 (June 2024). OreSat docs never moved to B-3 (G). | Deviation / Deviation / no draft. 732.1 §6.3.4. Same sentence in B-1, B-2 and B-3. OreSat writes "Though out of spec". | **Yes** (one of two reasons checked). The code note says "return the header to be put in the insert zone (see previous note.)". That note is the spacepackets reason (row 7). spacepackets-py 0.32.0 still has no SDLS in USLP frames. Source-verified (G: 0.32.0 wheel, `spacepackets/uslp/frame.py`). PR #74 reason "so the packets from YAMCS are properly recognized.": not checked. |
| 2 | OreSat: same item, §6.3.3 | Same as row 1. | Deviation / Deviation / no draft. 732.1 §6.3.3. Same text in B-1, B-2, B-3. | **Yes**. Same as row 1. |
| 3 | OreSat: same item, use of the Insert Zone | Same as row 1. | Allowed / Allowed / no draft. 732.1 §4.1.3.1, §4.1.3.3 (use of the zone). The content of the zone is the row 1 deviation. | **Yes**. Same as row 1. |
| 4 | OreSat: Insert Zone with TFDZ rule ‘111’ | Same as row 1. | Open / Open / no draft. 732.1 Table 5-1 note 5; §4.1.4.2.2.2.8; Table 5-3 note 2. INFERENCE only. OreSat does not state its Physical Channel Frame Type. Fact: note 5 is not in 732.1-B-1. B-2 added it. | **N/A** (no stated reason). G: no stated rationale found at OreSat. |
| 5 | OreSat: same item as row 1, SDLS mask rules | edl.rst pin 2026-08-02. Names no 355.0 issue. 355.0-B-2 by date (D). Current: 355.0-B-2 (same book). | Deviation / Deviation / no draft. 355.0-B-2 §4.2.2.6.2 g), h). INFERENCE (M). Fact: 355.0-B-1 has the mask rule "(AOS only)" and no USLP rule. | **Yes**. Same as row 1. |
| 6 | OreSat: same item as row 1, §5.4 and §2.2.1 | Same as row 5. | Deviation / Deviation / no draft. 355.0-B-2 §5.4 b), §2.2.1. Fact: 355.0-B-1 §5 covers TM, TC and AOS only. | **Yes**. Same as row 1. |
| 7 | OreSat: SDLS Trailer inside the data zone | Same as row 1. | Deviation / Deviation / no draft. 732.1 §6.3.6. Same first sentence in B-1, B-2, B-3. B-3 NOTE 1 adds "or MAP". OreSat writes "Though out of spec". | **Yes**. Reason: "As the spacepackets protocol does not support sdls". spacepackets-py 0.32.0 (May 2026) still has no SDLS in USLP frames. G: docstring "This is the format without the SDLS option". Source-verified. |
| 8 | OreSat: same item, TFDF length and OCF | Same as row 1. | Deviation / Deviation / no draft. 732.1 §6.3.5.2 b), §6.3.7.2. INFERENCE (M). | **Yes**. Same as row 7. |
| 9 | OreSat: trailer present (32-octet HMAC) | Same as row 5. | Allowed / Allowed / no draft. 355.0-B-2 §4.1.2.2, §4.1.2.3. Same text in 355.0-B-1. Only the position is the deviation (row 7). | **N/A** (no stated reason). G: no stated rationale found for HMAC over GCM or encryption. |
| 10 | OreSat: uses SDLS | Same as row 1. | Allowed / Allowed / no draft. 732.1 §2.1.2.5. | **N/A** (no stated reason for a deviation; the item is allowed). |
| 11 | OreSat: same, §6.2 and §6.3.1 | Same as row 1. | Allowed / Allowed / no draft. 732.1 §6.2, §6.3.1. B-3 adds SDLS per MAP. Fact: 732.1-B-2 ref [15] is 355.0-B-1. B-3 ref [15] is 355.0-B-2. | **N/A** (no stated reason; the item is allowed). |
| 12 | OreSat: same, managed parameters and PICS | Same as row 1. | Allowed / Allowed / no draft. 732.1 PICS USLP-127/128/129 (B-2) = USLP-151/152/153 (B-3). | **N/A** (no stated reason; the item is allowed). |
| 13 | OreSat: cites 732.1-B-2 | Same as row 1. | N/A / N/A / no draft. Issue fact. B-2 became superseded in June 2024. No verdict change. | **N/A** (no stated reason). |
| 14 | OreSat: MAP ID always 0 | Same as row 1. | Allowed / Allowed / no draft. 732.1 §4.1.2.5.2. | **not checked** |
| 15 | OreSat: VCF Count Length ‘000’ | Same as row 1. | Allowed / Allowed / no draft. 732.1 §4.1.2.12.1, Table 4-2 (allowed on its own; see row 16). | **N/A** (no stated reason). G: "No stated rationale found." |
| 16 | OreSat: COP-1 with no VCF Count | edl.rst pin 2026-08-02. 732.1-B-2 and 232.1-B-2 + Cor. 1. Current: 732.1-B-3; 232.1-B-2 + Cor. 1 (same). | Open / Open / no draft. 732.1 Table 5-3, §4.1.2.12 NOTE 4; 232.1-B-2 §2.1. INFERENCE only. OreSat does not say which VCs send Type-A frames. | **N/A** (no stated reason). G: no stated rationale found. |
| 17 | OreSat: Source/Destination labels | Same as row 1. | Open / Open / no draft. 732.1 §4.1.2.3.3. INFERENCE: the bit values match; the labels differ. Code not checked. | **not checked** |
| 18 | OreSat: PCC flag polarity | Same as row 1. | Open / Open / no draft. 732.1 §4.1.2.8.2.2. INFERENCE: the doc text is the reverse of the book. It can be a doc error. Code not checked. | **N/A** (no stated reason). G: no issue or commit mentions it. |
| 19 | OreSat: IDLE VC 2 with CLCWs | Same as row 1. | Allowed / Allowed / no draft. 732.1 §4.1.5.2.1 (CLCW in OCF). A non-OID "idle VC" is not defined in B-2 or B-3. | **not checked**. G has a stated reason (PR #74: CLCW every 1 s, inside the Yamcs 3000 ms FOP-1 delay). No file re-checks it. |
| 20 | OreSat: SPI = 1 | Same as row 5. | Allowed / Allowed / no draft. 355.0-B-2 §4.1.1.2.3, Table 6-1. Same in 355.0-B-1. | **N/A** (no stated reason). G: PR #60 describes the profile but does not say why. |
| 21 | OreSat: 6-octet header, 4-octet SN, 32-octet HMAC | Same as row 5. | Allowed / Allowed / no draft. 355.0-B-2 §4.1.1.1.4, Table 6-1. Same values in 355.0-B-1. | **N/A** (no stated reason). G: no stated rationale found for HMAC-only. |
| 22 | OreSat: anti-replay rule | Same as row 5. | Allowed (h) and Open (i, window) / same / no draft. 355.0-B-2 §4.2.4.4 h), i). OreSat states no window. | **not checked** |
| 23 | Yamcs: MAP service only for TC, not USLP | Docs pin 2026-02-04 cite 732.1-B-2 (the link goes to B-3). Yamcs 5.13.6 (2026-10-01) still cites 732.1-B-2. Current: 732.1-B-3. | See row 24 / See row 24 / no draft. 732.1 §3.3.1, §3.5.1, §3.7.1 (Overview text, no keyword). | **N/A** (no stated reason). The limit is still in the docs (G check 3: STILL TRUE). Doc-only. |
| 24 | Yamcs: same, PICS | Same as row 23. | Deviation / Deviation / no draft (from full conformance). 732.1 PICS: MAP items are M in B-2 and B-3. INFERENCE: it needs a PICS exception (§A1.3). | **N/A** (no stated reason). Same as row 23. |
| 25 | Yamcs: ignores Insert Zone data | Same as row 23. | Allowed / Allowed / no draft. 732.1 §4.1.3.1. INFERENCE: if a mission uses the Insert Service, the receiver skips a "shall" (B-2 §4.3.10.1.4 = B-3 §4.3.11.1.4). | **N/A** (no stated reason). The limit is still in the docs: "Currently Yamcs ignores any data in the insert zone." (G check 3: STILL TRUE). Doc-only. |
| 26 | Yamcs: same, PICS | Same as row 23. | Deviation / Deviation / no draft (from full conformance). 732.1 PICS: Insert items are M in B-2 and B-3. INFERENCE. | **N/A** (no stated reason). Same as row 25. |
| 27 | Yamcs: no TC segmentation | Docs pin 2026-02-04. 232.0-B-4 (Cor. 1, October 2023, existed then). Current: 232.0-B-4 + Cor. 1 (same). | See row 28 / same book / no draft. 232.0-B-4 §4.1.3.2.2.1.2. Cor. 1 does not touch these clauses. | **Yes**. 2021 status: "No, that is not supported yet; currently one TC packet = one TC frame." Docs today: "Yamcs does not support segmentation (i.e. splitting a TC packet over multiple frames)". G check 3: STILL TRUE. Doc-only. |
| 28 | Yamcs: same | Same as row 27. | Allowed / Allowed / no draft. 232.0-B-4 Table 5-4, PICS TC-114. INFERENCE (M): allowed when segmentation is "Prohibited" and no packet is larger than one frame. | **Yes**. Same as row 27. |
| 29 | Yamcs: USLP segmentation counterpart | Same as row 23. | Allowed (sender) and Open (Yamcs receive side) / same / no draft. 732.1 §4.1.4.2.2.1.3. B-3 adds VCA_SDU and per-SAP wording. | **not checked** |
| 30 | AcubeSAT: no SDLS (TC) | README HEAD 2024-09-10. 232.0-B-4 (current). DDJF_TTC 2021 names 232.0-B-3 (D2). | Allowed / Allowed / no draft. 232.0-B-4 §2.1.2.3, PICS TC-119. | **N/A** (no stated reason). G: no stated rationale for the original "no SDLS". Fact: the README note "(Note: There is no planned support for SDLS)" is out of date. Work item 102 (2024-12-01) plans the SDLS authentication service. Branch Tc-Rx-Unit-Testing (HEAD 3bd340b, 2026-09-18) has SDLS HMAC code (Room fact 1). |
| 31 | AcubeSAT: no SDLS (TM) | README HEAD 2024-09-10. 132.0-B-3 (current). | Allowed / Allowed / no draft. 132.0-B-3 §2.1.2.2, PICS TM-89. | **N/A** (no stated reason). Same fact as row 30. |
| 32 | AcubeSAT: 232.1 and SDLS | README HEAD 2024-09-10. 232.1-B-2 (current). | N/A / N/A / no draft. 232.1-B-2 has no SDLS text. | **N/A** (no stated reason). |
| 33 | AcubeSAT: scope names 232.0-B-4, 232.1-B-2, 132.0-B-3 | README HEAD 2024-09-10. All three are current (G, D). | N/A / N/A / no draft. Issue fact. The README does not name the TC corrigenda. | **N/A** (no stated reason). |
| 34 | SatNOGS-COMMS: Rx only CCSDS RS | ICD line last changed 2024-12-07 (commit 5b65b0f863). No issue named. 131.0-B-5 by date (D). Current: 131.0-B-6. G: the MCU moved to USLP via OSDLP in 2025 (commit b887165, 2025-07-26). | Allowed / Allowed / no draft. 131.0-B-5 §4.1 = 131.0-B-6 §5.1; §12.3, Table 12-1. | **not checked**. G: closest statement is the AT86RF215 baseband limits ("CC FEC available only for downlink"). G warns it is not tied directly to "Rx RS only". B did not check SatNOGS. |
| 35 | SatNOGS-COMMS: E, I and randomizer clauses | Same as row 34. | Open / Open / no draft. **Rule changed.** 131.0-B-5 §4.2.1 has an escape clause and Table 12-1 lists "Absent". 131.0-B-6 §5.2.1 has no escape clause and no "Absent". SatNOGS does not say if it randomizes. | **N/A** (no stated reason). |
| 36 | SatNOGS-COMMS: Rx path vs 231.0 | Same as row 34. 231.0-B-4 (July 2021). Current: 231.0-B-4 + Cor. 1 (July 2026). | N/A or Open (conditional) / same / no draft. 231.0-B-4 has no RS text. Cor. 1 changes §5.2.4.2 only. It matters only if "Rx" is a 231.0 uplink. | **N/A** (no stated reason). |
| 37 | SX-USP: no interleaving | README initial commit 5934376d (2021-03-16), unchanged at f72bdcf4 (2024-04-14). Cites 131.0-B-3. Current: 131.0-B-6 (April 2026). | Allowed / Allowed / no draft. 131.0-B-3 §4.3.5.1 = 131.0-B-6 §5.3.5.1 (I=1). | **N/A** (no stated reason). G: "No interleaving yet", no stated rationale found. |
| 38 | SX-USP: same item, current-book row | Same as row 37. | Allowed / Allowed / no draft. 131.0-B-6 Table 12-3. | **N/A** (no stated reason). |
| 39 | SX-USP: "Scrambling as per CCSDS 131.0-B-3 ... Section 10." | Same as row 37. | Allowed / **Open** / no draft. **Changed.** 131.0-B-3 §10.4.2: 255-bit sequence. 131.0-B-6 §10.4.1: 131071 bits is the "shall". 131.0-B-6 §10.4.2: "For backward compatibility with legacy systems, a 255-bit pseudo-random sequence may be generated instead". B-6 does not define "legacy systems". INFERENCE (D): SX-USP uses the 255-bit sequence. The change dates from 131.0-B-5 (September 2023). | **N/A** (no stated reason for the scrambler choice). |
| 40 | NASA CryptoLib: scope TC/TM/AOS, no USLP | Wiki Home 2024-07-17. Links 355.0-B-1. Current: 355.0-B-2. Names no USLP issue. | N/A / N/A / no draft. Scope statement. The scope gap appeared with 355.0-B-2 (July 2022), which adds USLP. | **N/A** (no stated reason). G: no USLP mention in docs, commits, or issues. |
| 41 | spacepackets-py: cites 732.1-B-2 | 0.32.0 (2026-05-03). Cites 732.1-B-2. Current: 732.1-B-3. | N/A / N/A / no draft. Issue fact. No stated deviation. Library code not reviewed against the B-3 changes (M). | **N/A** (no stated reason). G: no reason for no SDLS or for the fixed CRC16 FECF. Fact: 0.32.0 still has no SDLS in USLP frames (Room fact 4). |
| X1 | Yamcs: USLP FECF option CRC32 | Added in commit be8cf80a9 (2019-02-15), message "USLP - CCSDS 732.1-B-1". Docs today cite 732.1-B-2. Yamcs 5.13.6 (2026-10-01) still cites 732.1-B-2. Current: 732.1-B-3. | **Allowed** (B-1) / **Deviation** (B-2, B-3) / no draft. **Changed.** 732.1-B-1 §4.1.6.2.2 allows 16-bit or 32-bit. 732.1-B-2 and B-3 §4.1.6.2.2: "If present, the FECF shall occupy the last 16 bits of every Transfer Frame transmitted within the same Physical Channel throughout a Mission Phase." INFERENCE (D): it applies when a link is set to CRC32. | **N/A** (no stated reason). G: no stated rationale found. |
| X2 | AcubeSAT: 40-bit (5-octet) HMAC tag for SDLS authentication | DDJF_TTC v2.0 (2021-05-17): 355.0-B-1 by date. Work item 102 (2024-12-01): 355.0-B-2 by date. No issue named. Code: branch Tc-Rx-Unit-Testing, HEAD 3bd340b (2026-09-18). Current: 355.0-B-2. | Deviation / Deviation / no draft. 355.0-B-1 and B-2 Table 6-1: MAC length "8-64 octets". INFERENCE (D): 5 octets is below 8. Room fact 1: the code still sets `macFieldLength = 5`. Their own `MinMACLength = 8`. AcubeSAT does not call it a deviation. | **Judgment call**. Reason (DDJF 2021): "the chance of guessing a 40-bit HMAC tag is really insignificant". No file re-tests this claim. Source-verified only that the 5-octet tag is still in the code (Room fact 1). |

### Table B: stated reasons and survey rows with no map row

Sub-rows (R2-b, R8-a, R8-b, R8-c) come from D2. Row R8 is now a parent row for R8-a to R8-c.

| ID | Project: item | Project date + issue followed (check 1) | Deviation under their issue / current issue / draft (check 2) | Stated reason still true today? (check 3) |
|---|---|---|---|---|
| R1 | OreSat / UniClOGS: GFSK 50 kb/s, chosen for the spacecraft radio | uniclogs-hardware README @5c0cd713 (2026-03-01). Radio reason. The EDL docs cite 732.1-B-2 (row 1). | N/A (no stated deviation; PHY reason, not in the map). | **Partly**. Reason: "was chosen for the capability of the Engineering transceiver radio on the spacecraft". B marks it "Changed". It was true for the flown AX5043. That chip was "discontinued in 2022". OreSat1 moves to Microchip AT86RF215. Doc-only (OSU Space Grant poster, spring 2026). |
| R2 | OreSat: UPID = Space Packet because of Yamcs | Commit ad50e8d (2026-05-15). Current text: `oresat_c3/protocols/uslp.py` L130, commit 9e41c5af (2026-06-09). EDL docs cite 732.1-B-2. Current: 732.1-B-3 (it was current on both code dates, D2). | Allowed / Allowed / no draft. 732.1 §4.1.4.2.3.2, §4.1.4.2.3.3, §4.1.4.3.3 (same text in B-2 and B-3). Table 5-4 (MAP Channel): "UPID supported Integer (see reference [14])". PICS USLP-123 (B-2) = USLP-147 (B-3). SANA (a registry, not a book): "0b00000 Space Packets or Encapsulation packets are contained within the TFDZ." INFERENCE (D2): one fixed UPID is a managed-parameter choice per MAP. See R2-b for the content match. | **Partly**. Reasons: "Needs to be this, otherwise we would need to rewrite at least 1000 lines in YAMCS for 5 bits." and "Needs to be this, yamcs does not support user defined octet stream and this functions." The Yamcs packet service still rejects UPID ≠ 0. Source-verified (G: Yamcs master `UslpFrameDecoder.java`). Yamcs has a VCA service with a custom handler (docs). INFERENCE (G): a VCA handler is a documented path for other UPIDs. "1000 lines": not verified. |
| R2-b | OreSat: does UPID 0 match what OreSat puts in the TFDZ? (docs vs code) | Docs: `docs/edl.rst` line 87 says `0b000101`. Code: OreSat HEAD 1136e31 (2026-08-17), `oresat_c3/protocols/uslp.py` line 129: `make_frame` always sets `SPACE_PACKETS_ENCAPSULATION_PACKETS` (Room fact 6). EDL docs cite 732.1-B-2. Current: 732.1-B-3. | Open / Open / no draft. 732.1 §4.1.4.2.3.2 and §4.1.4.3.3: "The TFDZ shall contain the data defined by the UPID." Docs and code disagree (source-verified, Room fact 6). edl.rst: "UPID (USLP Protocol Identifier): 5 bits. Set to ``0b000101`` to mark the protocol in the TFDZ is mission specific." SANA: "0b00101 Mission-specific Information-1 as a MAPA_SDU or VCA_SDU is contained within the TFDZ." COP-1 protocol-control frames also go out with UPID 0 (D2: `scripts/edl_cmd_shell.py` L99). SANA: "0b00001 COP-1 Control Commands are contained within the TFDZ." D2 found no SpacePacket/SpHeader/APID use in `oresat_c3/`. INFERENCE (D2): if the TFDZ holds data other than Space or Encapsulation Packets, UPID 0 does not match §4.1.4.3.3. Bytes on the wire: not checked. | **Partly**. Same reason as R2. The code comment still reads "yamcs does not support user defined octet stream" at HEAD 1136e31. Source-verified (Room fact 6). The Yamcs part of the reason has the same status as R2. |
| R3 | OreSat / UniClOGS: Yamcs as mission control | SSC24-WII-01 paper (PDF 2024-07-26). Link: 732.1-B-2 (EDL docs). No book for the tool choice. Current: 732.1-B-3. | N/A / N/A / no draft. No clause governs a ground software product. 732.1-B-2 and B-3 §1.2: "It does not specify: a) individual implementations or products;" INFERENCE (D2): the §1.2 words cover this case. | **Judgment call**. Reason: "for its open source and simple to operate nature". G: cannot re-test. |
| R4 | OSDLP: no TM/TC SDLP, USLP only | README commit cffd5a90 (2026-09-28); local HEAD b08b733 (2026-10-01). Cites 732.1-B-3 and 232.1-B-2. The README also lists 232.0-B-4 as "Supported (CLCW & COP-1 Commands)". All current. | N/A / N/A / no draft (no deviation found). 732.1-B-3 §1.1; §A1.1: "An implementation claiming conformance must satisfy the mandatory requirements referenced in the RL." No 732.1-B-3 PICS item names the TM or TC SDLP. INFERENCE (D2): a USLP-only implementation can conform. Whether OSDLP meets every M item: not checked. The "superseded" claim (D2 row R4-b): not supported by the book text. 732.1-B-3 Document Control lists only 732.1-B-1 and 732.1-B-2 as superseded. | **No**. Reason: "as they are superseded by the Unified Space Link Protocol (USLP, CCSDS 732.1-B-3)". ccsds.org lists 132.0-B-3 and 232.0-B-4 as active Blue Books (Room fact 3; G). G found no "supersede" text in 732.1-B-3 except its own issue history. Doc-only (ccsds.org list). Not checked: SLS WG slides or mailing lists. |
| R5 | OpenLST: non-CCSDS stack | USERS_GUIDE @b996935a (2018-08-03). No CCSDS issue (non-CCSDS). | N/A (no stated deviation). | **N/A** (no stated reason). B note: the rate mismatch is reconciled (INFERENCE). About 7416 Bd with rate-1/2 FEC gives about 3.7 kb/s. That fits "3kbps". The sync word is 16 bits (D3 91). A CCSDS ASM cannot be matched in hardware on the CC1110. Datasheet/doc-only. |
| R6 | SX-USP: own sync word for low-power single-chip transceivers | README initial commit 5934376d (2021-03-16). Cites 131.0-B-3 for RS, scrambling and convolutional coding. No book named for the sync field. Current: 131.0-B-6. | Deviation (INFERENCE) / Deviation (INFERENCE) / no draft. 131.0-B-3 §4.3.8.1 b), §9.3.1, §10.3.1: ASM 1ACFFC1D. 131.0-B-6 §5.3.8.1 b), §9.3.1, §10.3.1: CSM 1ACFFC1D. The name changed (ASM → CSM in B-6). The pattern and the "shall" did not change. Fact: 5072F64B2D90B1F5 is not a marker in B-3 or B-6. INFERENCE (D2): the clash holds only if one measures the sync field against 131.0. SX-USP claims 131.0 only for RS, scrambling and convolutional coding, and it carries no TM/AOS/USLP frames. | **Partly**. Reason: "Optimized for the first 32 bits being detected by the single-chip transceiver in low-power receive mode". B: it holds only for chips with a true 32-bit sync detector. CC1101/CC1110 cannot match the first 32 bits in hardware. SX1276/RFM95 FSK sync can match all 64 bits. SX-USP does not name the chip. Datasheet-only. |
| R7 | AcubeSAT: MAP ID not used for response routing | DDJF_TTC v2.0 (2021-05-17) names "CCSDS‐232.0‐B‐3" (§3.1, p. 14). This is the date of the MAP ID decision. Library README (HEAD 2024-09-10) names 232.0-B-4. Current: 232.0-B-4 + Cor. 1. | Allowed / Allowed / no draft. 232.0-B-3 and B-4 §4.1.3.2.2.1.2: "The Segment Header is optional; its presence or absence shall be established by management for each Virtual Channel." Also §4.1.3.2.1.4, §4.1.3.2.2.3.2, Table 5-3 (same text in both). B-4 adds the PICS ("TC-101 Presence of Segment Header Table 5-3 M Present(‘1’)/ Absent (‘0’)"). INFERENCE (D2): no clause requires the MAP ID to carry response-routing data. AcubeSAT does not state if a VC has more than one MAP or segments SDUs. Not checked: whether the library implements the MAP functions that the B-4 PICS marks M. | **Judgment call**. Reason: "considered too complex". G: cannot re-test. |
| R8 | Yamcs: randomization skip per VC; error detection per VC (TC uplink) | Commits 1cde8f849 (2022-01-30) and 805b9cce8 (2022-01-31). Docs name 231.0-B-4, 232.0-B-4, 131.0-B-4, 732.1-B-2, 132.0-B-3. Yamcs 5.13.6 (2026-10-01) cites 131.0-B-4 and 732.1-B-2. On the commit dates 131.0-B-3 was current; 131.0-B-4 came in April 2022 (D2). Current: 231.0-B-4 + Cor. 1, 232.0-B-4 + Cor. 1, 131.0-B-6, 732.1-B-3, 132.0-B-3. | See R8-a, R8-b, R8-c. D2 corrects the scope: both options are in the Yamcs docs section "Telecommand Frame Processing". So the books are the TC uplink books, 231.0 and 232.0. "CLTU" has 0 matches in 131.0-B-4 and 131.0-B-6. Neither commit patch contains "USLP". | **N/A** (no stated reason). G: no stated rationale found. |
| R8-a | Yamcs: skip CLTU randomization per VC (`skipRandomizationForVcs`, BCH only) | Commit 1cde8f849 (2022-01-30). Docs: 231.0-B-4 (and 131.0-B-4). Current: 231.0-B-4 + Cor. 1. | 231.0-B-4: Open / Open / no draft. The only per-Physical-Channel text is informative. §6.1 OVERVIEW: "Its use is fixed for a Physical Channel and is managed (i.e., its presence or absence is not signaled but is known a priori by the receiver), per Table 8-2." Table 8-2: "Randomizer Used, Not used". 231.0-B-4 has no PICS. Cor. 1 changes §5.2.4.2 only. 131.0: N/A for this option (no CLTU in 131.0). Yamcs calls it "not as per CCSDS standard". INFERENCE (D2): a per-VC skip clashes with informative text only. | **N/A** (no stated reason). Same as R8. |
| R8-b | Yamcs: error detection (FECF NONE/CRC16) per VC on the TC uplink | Commit 805b9cce8 (2022-01-31). Docs: 232.0-B-4 (TC) and 732.1-B-2 (USLP). Current: 232.0-B-4 + Cor. 1; 732.1-B-3. | 232.0-B-4: Deviation / Deviation / no draft, when VCs on one Physical Channel differ. §4.1.4.1.3: "If present, the Frame Error Control Field shall occur within every Transfer Frame transmitted within the same Physical Channel throughout a Mission Phase." Also Table 5-1 (Physical Channel), PICS TC-86 M, §4.3.8.2 b). INFERENCE (D2): the clash exists only when the setting of one VC differs from another VC on the same Physical Channel. 732.1-B-2 and B-3: same rule (§4.1.6.2.1, Table 5-1). It applies only if Yamcs uses the option on USLP uplink frames (UNVERIFIED). 132.0-B-3: N/A (the Yamcs TM side has no per-VC option). | **N/A** (no stated reason). Same as R8. |
| R8-c | 131.0 randomizer: scope and status (book fact asked for R8) | Docs label 131.0-B-4. 131.0-B-3 was current at the commits. Current: 131.0-B-6. | Book fact, not a Yamcs verdict / no draft. 131.0-B-4: per Physical Channel, optional with a designer escape clause (§12.2.1, §12.3, Table 12-1 "Randomizer Present/Absent", §10.1, §4.2.1). 131.0-B-6: per Physical Channel, mandatory (§12.2.1; §5.2.1 "The pseudo-randomizer defined in section 10 shall be used."; Document Control "it makes the pseudo-randomizer as mandatory."). **Rule changed** (optional → mandatory). No 131.0 text sets the randomizer per VC. The change does not reach the R8-a option, which is 231.0 (D2). | **N/A** (book fact; no project reason). |
| R9 | LibreCube: link-layer docs "To be written..." | HEAD 8739a6ff (2026-09-15). No link-layer book or issue named. | N/A (no stated deviation). | **not checked** |
| R10 | OpenCCSDS: no docs, no license | HEAD 87cae72e (2022-03-30). No issue named (no docs). | N/A (no stated deviation). | **not checked** |
| R11 | Bifrost: blanket non-compliance note | Last commit 2024-04-14. No issue named. | N/A (no list of deviations). | **not checked** |
| R12 | PicSat: SPP modified (quote not verified) | not checked | not checked | **not checked** |

### Counts

Updated 2026-10-06 (CT) with D2.

Check 2, Table A (43 rows: map rows 1–41, X1, X2). No change from D2:
- Verdict changed: 2 rows. X1 (Allowed → Deviation). Row 39 (Allowed → Open).
- Rule changed, verdict the same: 1 row. Row 35 (Open in 131.0-B-5 and in 131.0-B-6).
- Verdict and rule the same: 40 rows.
- Draft: no draft for all 43 rows.

Check 2, Table B (16 rows: R1–R12 plus sub-rows R2-b, R8-a, R8-b, R8-c):
- Allowed / Allowed: 2 rows (R2, R7).
- Open / Open: 2 rows (R2-b, R8-a).
- Deviation / Deviation: 2 rows (R6, INFERENCE; R8-b, only when VCs on one Physical Channel differ).
- N/A: 7 rows (R1, R3, R4, R5, R9, R10, R11).
- Book fact with a rule change: 1 row (R8-c: 131.0 randomizer optional in B-4, mandatory in B-6).
- Parent row, see sub-rows: 1 row (R8).
- not checked: 1 row (R12).
- Verdict changed: 0 rows.
- Wording changed, verdict the same: R6 (ASM renamed CSM in 131.0-B-6), R7 (232.0-B-4 adds PICS TC-101), R2 (PICS USLP-123 → USLP-147).
- Draft: no draft for every Table B row that D2 checked.

Check 3 (Table A and Table B, 59 rows):
- Yes: 9 rows (1, 2, 3, 5, 6, 7, 8, 27, 28).
- No: 1 row (R4).
- Partly: 4 rows (R1, R2, R2-b, R6).
- Judgment call: 3 rows (X2, R3, R7).
- N/A (no stated reason, or a book fact): 32 rows (4, 9–13, 15, 16, 18, 20, 21, 23–26, 30–33, 35–41, X1, R5, R8, R8-a, R8-b, R8-c).
- not checked: 10 rows (14, 17, 19, 22, 29, 34, R9, R10, R11, R12).

### Rows whose status or reason changed

- X1, Yamcs CRC32 FECF: Allowed under 732.1-B-1. Deviation under B-2 and B-3. Yamcs 5.13.6 still cites B-2 and still offers CRC32.
- 39, SX-USP 255-bit randomizer: Allowed under 131.0-B-3. Open under 131.0-B-6 §10.4.2 (legacy clause).
- 35, SatNOGS-COMMS randomizer: the rule changed. The verdict is Open under 131.0-B-5 and 131.0-B-6.
- X2, AcubeSAT 5-octet HMAC: new row. Deviation under 355.0-B-1 and B-2 (INFERENCE). Still in the code. The reason is a judgment call.
- 30–31, AcubeSAT "no planned support for SDLS": the verdict is the same. The README note is out of date (SDLS HMAC code on branch Tc-Rx-Unit-Testing).
- R1, OreSat / UniClOGS radio reason: Partly. The AX5043 is discontinued (2022). OreSat1 moves to AT86RF215.
- R2, OreSat UPID / Yamcs reason: Partly. The packet service still rejects UPID ≠ 0. A VCA handler path exists. "1000 lines" is not verified.
- R4, OSDLP "superseded" reason: No. ccsds.org lists TM and TC as active.
- R6, SX-USP sync reason: Partly. It depends on the chip.
- R2-b, OreSat UPID: new sub-row. Open / Open. The docs say `0b000101`; the code sends UPID 0 (source-verified). COP-1 control frames also go out with UPID 0.
- R6, SX-USP sync word: Deviation / Deviation (INFERENCE). The ASM is now named CSM in 131.0-B-6. The rule is the same.
- R7, AcubeSAT MAP routing: Allowed / Allowed. Check 1 fixed: the 2021-05-17 design doc names 232.0-B-3.
- R8, Yamcs per-VC options: scope corrected to the TC uplink (231.0, 232.0). R8-a Open / Open. R8-b Deviation / Deviation when VCs on one Physical Channel differ.
- R8-c, 131.0 randomizer: rule changed. Per Physical Channel in both issues. Optional in 131.0-B-4, mandatory in 131.0-B-6.
