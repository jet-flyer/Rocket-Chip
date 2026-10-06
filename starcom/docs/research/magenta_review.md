# CCSDS Magenta Books review for Rocket Chip / Starcom

Prepared 2026-10-03 (CT). No repo was edited. Everything below comes from CCSDS documents or ccsds.org pages, with a clause or page for each claim.

Conventions in this file:
- **QUOTE** means verbatim text from the named document.
- **INFERENCE** is my own reading. It is kept in separate lines so you can discard it.
- **UNVERIFIED** means I could not confirm it from a primary source.
- Page numbers are the printed page numbers, with the PDF page in brackets where useful.

Files used (all on the box):
- Magenta PDFs and text, downloaded from the URLs in /workspace/magenta.txt: /workspace/mag/*.pdf and *.txt (39 files).
- CCSDS A02.1-Y-4 Cor. 2, "Organization and Processes for the CCSDS", from https://ccsds.org/Pubs/A02x1y4c2.pdf (/workspace/mag/A02x1y4c2.pdf).
- CCSDS 727.0-B-5, from https://ccsds.org/Pubs/727x0b5e1.pdf.
- CCSDS 355.0-B-2, from https://ccsds.org/Pubs/355x0b2.pdf.
- CCSDS 355.1-B-1, from https://ccsds.org/Pubs/355x1b1.pdf.
- Already on the box: 211.0-B-6, 133.0-B-2, 130.12-G-2, 131.31-O-1 Cor. 1.
- ccsds.org pages: https://ccsds.org/publications/magentabooks/ and the saved blue/green/orange pages in /workspace.

---

## 0. Short answers

**(1) Equal authority?** Partly.
- The one CCSDS document that defines the colors says a Magenta Book is a "full-fledged specification at a peer level with" a Blue Book (A02.1-Y-4 §6.1.4.3, p. 6-5).
- The same document says Magenta Books are *not directly implementable for interoperability* and are not required to be prototyped (§6.1.4.3, §6.1.4.4).
- Every Magenta Book's own Statement of Intent calls Recommended Practices "more descriptive in nature … general guidance". It says endorsement "does not imply a commitment … to implement its recommendations in a prescriptive sense". The Blue Book Statement of Intent has no such sentence.
- So the wording is not identical to the Blue Books' (see §A2 for the exact diff). Both readings are in the CCSDS text. I found no document that says "equal" in those words. The closest is "peer level".
- The 'shall' definition inside Magenta Books ("binding and verifiable specification") is word-for-word the same as in Blue Books.

**(2) Which are useful?** Only a few, and mostly as guidance or checklists, not as new requirements.

| Book | Project issue | Verdict |
|---|---|---|
| 354.0-M-1 Symmetric Key Management | SDLS key management | **Useful now** (design checklist, no OTAR needed) |
| 320.0-M-7 (+Cor. 1) SCID assignment | SCID use | **Useful now** (know the rules, document your position). It gives no hobbyist path. |
| 722.1-M-2 CFDP Unitdata Transfer Layers | CFDP offload | **Useful later** (when CFDP is built). The PDF is marked DRAFT. |
| 351.0-M-1 Security Architecture | security architecture | **Useful later** (documentation checklist) |
| 350.8-M-3 Security Glossary | security terms | **Useful later** (reference only) |
| 311.0-M-2 RASDS | reference architecture | **Useful later / optional** |
| 876.1-M-1 EDS Dictionary of Terms | electronic data sheets | **Not useful alone.** The SEDS Blue Book 876.0-B-1 is the real thing, and I have not read it. |
| 882.0-M-1 Low Data-Rate Wireless | onboard wireless | **Not useful** (prescribes 802.15.4 or ISA100.11a only) |
| 851.0-M-1 Subnetwork Packet Service | subnetwork services | **Not useful** (service definition only) |
| 901.1-M-1 SCCS-ARD (Prox-1 / cross support) | Prox-1 ops | **Not useful** (agency-scale) |
| All others | – | **Clearly irrelevant** (§B10) |

Nathan's Orange preference: none of the useful books is Orange. 354.0-M-1 and 320.0-M-7 involve no experimental book. The Blue Book SDLS 355.0-B-2 is Blue, not Orange (§B1).

---

## A. Authority: what the color means

### A1. What CCSDS says the colors are

**CCSDS A02.1-Y-4 (Yellow Book, April 2014, includes Cor. 2 of Jan 2016).** This is the current issue. A web search found no A02.1-Y-5. A02 is itself a Yellow Book (a "Record"), described in §6.1.6.

Color to type mapping, §5.5.2.2 b (p. 5-17) and Table 6-1 (p. 6-17):
> QUOTE: "Document type shall be designated by color, where – Blue = Recommended Standard; – Magenta = Recommended Practice; – Green = Informational Report; – Orange = Experimental Specification; – Yellow = CCSDS Record."

Table 6-1 also lists Red = "Draft Recommended Standard or Practice", Pink = "Draft revised Recommended Standard or Recommended Practice", White = "Proposed Draft Recommended Standard", Silver = "Document having Historical status".

Tracks, §6.1.1 and §6.1.4–6.1.6 (pp. 6-1, 6-5, 6-6):
- Normative Track: Blue (Recommended Standard) and Magenta (Recommended Practice). Pre-final stages are White, Red and Pink.
- Non-Normative Track: Orange (Experimental), Green (Informational), Silver (Historical).
- Administrative Track: Yellow.

Blue versus Magenta, §6.1.4.2–6.1.4.4 (p. 6-5):
> QUOTE (6.1.4.2): "Recommended Standards are precise, prescriptive and/or normative specifications that define interfaces, protocols, or other controlling standards at a sufficient level of technical detail that they can be directly implemented and used for space-mission interoperability and cross support."
>
> QUOTE (6.1.4.3): "Recommended Practices are normative and have prescriptive content but are typically not directly implementable for interoperability or cross support. They may be of several types: a) specifications that are 'foundational' for other specifications …; b) system descriptions … that capture 'best' or 'state-of-the-art' recommendations …; c) reference architectures …; d) operational practices that are associated with other CCSDS specifications; e) Application Programming Interfaces (APIs) …"
>
> QUOTE (6.1.4.3, continued): "Recommended Practices (Magenta Books) differ from 'informational' documents in that they do provide normative, controlling guidance rather than purely descriptive material. Because they provide normative guidance, they are indeed a full-fledged specification at a peer level with Recommended Standards (Blue Books). A Magenta Book is no less of a specification than a Blue Book. However, unlike Recommended Standards, Recommended Practices do not include the level of normative technical, directly implementable, specification that would allow the independent development of separate but interoperable systems."
>
> QUOTE (6.1.4.4): "As a result of this difference in prescriptive content between Blue and Magenta books, Blue Books are required to be prototyped before final approval and publication, but Magenta Books are not required to be prototyped."

Annex B, "Recommended Practices" (p. B-2):
> QUOTE: "Practices say, 'Here is how the community recommends that one should carry out or describe this particular kind of operation at present, or how the community recommends that it should be carried out in the future.'"
>
> QUOTE (NOTE): "a Recommended Practice typically cannot be directly implemented to develop interoperable system components."

"Normative" is defined in §6.1.1 (p. 6-2):
> QUOTE: "To say that a section of a standard is 'normative' is to say that it is both well specified, i.e., a norm, and that it must be adhered to in order for an implementation to be compliant."

Non-normative documents, Annex B §B2.1 (p. B-8):
> QUOTE: "Specifications that are on the Non-Normative Track are labeled with one of three 'off-track' levels, and documents bearing these labels are not CCSDS standards in any sense: – Experimental; – Informational; – Historical."

Orange, §B2.2 (p. B-8):
> QUOTE: "The 'experimental' designation typically denotes a specification that is part of some research or development effort. … Every draft issue must clearly state the experimental status of the specification and must indicate the risks associated with implementing it in its current state."

Green, §B2.3 (pp. B-9, B-10):
> QUOTE: "The 'informational' document designation is intended to provide for the timely publication of a very broad range of general information for the CCSDS community. … There is no requirement for a formal agency review prior to publishing a CCSDS informational document."

Pink, §B1.2 d (pp. B-5, B-6):
> QUOTE: "Draft revisions are designated 'Pink' rather than 'Red' (cf. table 6-1). Pink Books are issued when changes to an existing Blue Book are extensive enough to warrant review of the entire document; Pink Sheets (changed pages only) are issued otherwise."

ccsds.org publication pages say the same in shorter form:
- Magenta page https://ccsds.org/publications/magentabooks/ QUOTE: "CCSDS Recommended Practices (Magenta Books) are the consensus results of CCSDS community deliberations and provide a way to capture 'best' or 'state of the art' approaches for applying or using standards. … a Recommended Practice might specify some specific 'Application Profiles' of multiple CCSDS Standards that are recommended for use in particular mission support configurations."
- Blue page QUOTE: "Standards must say very clearly, 'this is how you must build something if you want it to be compliant'."
- Green page QUOTE: "intended to provide for the timely publication of a very broad range of general information for the CCSDS community."
- Orange page QUOTE: "Experimental Specification (Orange Books) indicates that it is part of a research or development effort based on prospective requirements, and as such it is not considered a Standards Track document."

Does any CCSDS document say Magenta = Blue? A02.1-Y-4 §6.1.4.3 says "peer level" and "no less of a specification". It does not use the word "equal". It also lists the differences above. I found no other document that addresses this.

### A2. Statement of Intent, side by side

Texts compared (printed page ii in each):
- Blue: 211.0-B-6, and 133.0-B-2 (identical to 211.0-B-6).
- Magenta: 354.0-M-1 and 320.0-M-7 Cor. 1. These two are word-for-word identical to each other, and so are the 36 other Magenta PDFs I checked except 651.0-M-1 (below).

The first two paragraphs are identical in Blue and Magenta up to "The Committee meets periodically … technical solutions to these problems". After that they differ.

**Blue (211.0-B-6 p. ii):**
> "Inasmuch as participation in the CCSDS is completely voluntary, the results of Committee actions are termed Recommended Standards and are not considered binding on any Agency.
>
> This Recommended Standard is issued by, and represents the consensus of, the CCSDS members. Endorsement of this Recommendation is entirely voluntary. Endorsement, however, indicates the following understandings:
> o Whenever a member establishes a CCSDS-related standard, this standard will be in accord with the relevant Recommended Standard. Establishing such a standard does not preclude other provisions which a member may develop.
> o Whenever a member establishes a CCSDS-related standard, that member will provide other CCSDS members with the following information: -- The standard itself. -- The anticipated date of initial operational capability. -- The anticipated duration of operational service.
> o Specific service arrangements shall be made via memoranda of agreement. Neither this Recommended Standard nor any ensuing standard is a substitute for a memorandum of agreement.
>
> No later than five years from its date of issuance, this Recommended Standard will be reviewed …"

**Magenta (354.0-M-1 p. ii; 320.0-M-7 identical):**
> "Inasmuch as participation in the CCSDS is completely voluntary, the results of Committee actions are termed Recommendations and are not in themselves considered binding on any Agency.
>
> CCSDS Recommendations take two forms: Recommended Standards that are prescriptive and are the formal vehicles by which CCSDS Agencies create the standards that specify how elements of their space mission support infrastructure shall operate and interoperate with others; and Recommended Practices that are more descriptive in nature and are intended to provide general guidance about how to approach a particular problem associated with space mission support. This Recommended Practice is issued by, and represents the consensus of, the CCSDS members. Endorsement of this Recommended Practice is entirely voluntary and does not imply a commitment by any Agency or organization to implement its recommendations in a prescriptive sense.
>
> No later than five years from its date of issuance, this Recommended Practice will be reviewed …"

Exact differences (from a sentence-level diff):
1. Blue: "termed Recommended Standards and are not considered binding". Magenta: "termed Recommendations and are not **in themselves** considered binding".
2. Magenta adds a whole paragraph, "CCSDS Recommendations take two forms …", that contrasts "Recommended Standards that are prescriptive" with "Recommended Practices that are more descriptive in nature and are intended to provide general guidance".
3. Blue has the "Endorsement, however, indicates the following understandings" list (members' standards "will be in accord with" the Recommended Standard, information-sharing, memoranda of agreement). Magenta has none of it. It says instead that endorsement "does not imply a commitment … to implement its recommendations in a prescriptive sense."
4. The five-year review and "new version issued" paragraphs differ only by "Standard"→"Practice" and "standards"→"Practices".

So the Statements of Intent are **not identical**. They share the opening and the closing review boilerplate. The Blue one has commitments the Magenta one lacks.

Exception: **651.0-M-1** (May 2004, p. ii) uses an older Blue-style text ("termed Recommendations", the "following understandings" list, "Whenever an Agency establishes a CCSDS-related standard …"). It does not use the "two forms" paragraph. It is not relevant to the project.

**Green (130.12-G-2, June 2023).** There is no Statement of Intent. Pages i and ii give "AUTHORITY" and "FOREWORD":
> QUOTE (p. i): "This document has been approved for publication by the Management Council … and reflects the consensus of technical panel experts from CCSDS Member Agencies. The procedure for review and authorization of CCSDS Reports is detailed in … A02.1-Y-4."
> QUOTE (Foreword, p. ii): "This document is a CCSDS Informational Report that contains background and explanatory material to support the CCSDS Recommended Standard …"

The Green book has zero 'shall' by my count (grep of the extracted text).

**Orange (131.31-O-1 Cor. 1, Sept 2021).** There is no Statement of Intent. Page i says only "approved for publication by the Consultative Committee for Space Data Systems (CCSDS)". Note it does not say "Management Council". Page iv has a PREFACE:
> QUOTE: "This document is a CCSDS Experimental Specification. Its Experimental status indicates that it is part of a research or development effort based on prospective requirements, and as such it is not considered a Standards Track document. Experimental Specifications are intended to demonstrate technical feasibility in anticipation of a 'hard' requirement that has not yet emerged. Experimental work may be rapidly transferred onto the Standards Track should a hard requirement emerge in the future."

The Orange book still uses the same 'shall'/'must' nomenclature (§1.6.1, p. 1-3) and has 40 'shall' lines. Its §1.4 says "Where mandatory capabilities are clearly indicated … it is mandatory to implement them when this document is used as a basis for cross support."

**Pink.** No Pink Sheet is on the box. On the ccsds.org review sidebar there is a "CCSDS 722.1-P-1.1" item (saved page of Sept 29). The Pink definition is the A02.1-Y-4 text above.

**The ladder as the books word it:**

| Color | Own wording | Standards Track? | 'shall' language |
|---|---|---|---|
| Blue | "not considered binding on any Agency", plus the "following understandings" (A02 §B1.1: "These are the technical properties of what the implementer must build") | Yes | Yes (binding and verifiable) |
| Magenta | "not in themselves considered binding"; "more descriptive in nature … general guidance"; endorsement "does not imply a commitment … to implement … in a prescriptive sense" | Yes (A02 §6.1.4.3: "peer level") | Yes (same definition) |
| Green | "background and explanatory material" | No | No |
| Orange | "not considered a Standards Track document"; for feasibility | No ("not CCSDS standards in any sense", A02 §B2.1) | Yes within its own text |
| Pink / Red | draft revision of Blue or Magenta (A02 Table 6-1) | pre-final | – |

### A3. 'shall' language and conformance annexes in the Magenta set

Every Magenta book I scanned that has 'shall' defines it the same way as Blue. For example 354.0-M-1 §1.6.1 (p. 1-3) and 320.0-M-7 §1.3.1 (p. 1-1):
> QUOTE: "the words 'shall' and 'must' imply a binding and verifiable specification; the word 'should' implies an optional, but desirable, specification; the word 'may' implies an optional specification; the words 'is', 'are', and 'will' imply statements of fact."

Counts of 'shall' / 'should' / 'must' (grep of the PDF text) and conformance annexes:

| Book | shall | should | must | Own conformance annex |
|---|---|---|---|---|
| 354.0-M-1 | 109 | 12 | 1 | none |
| 320.0-M-7 Cor.1 | 31 | 6 | 16 | none |
| 351.0-M-1 | 3 | 54 | 42 | none |
| 722.1-M-2 | 49 | 6 | 2 | none |
| 876.1-M-1 | 82 | 8 | 30 | **Annex A, ICS proforma (normative)** |
| 882.0-M-1 | 4 | 21 | 8 | none |
| 851.0-M-1 (and 852–855) | 45 (90, 37, 26, 24) | 6 | 1 | **§5 Service Conformance Statement proforma ("shall be completed")** |
| 311.0-M-2 | 1 | 24 | 30 | none (the word "PICS" only appears inside "topics") |
| 350.8-M-3 | 1 | 4 | 2 | none |
| 523.1/523.2 (Java/C++ API) | ~2160 / ~1990 | 25 | 6 | none |
| 914.0-M-2 | 792 | 45 | 309 | none |
| 921.2-M-1 | 329 | 23 | 27 | requires CSTS specs to carry an ICS (§4.14), no own annex |

For comparison, 211.0-B-6 (Blue) has a normative "Annex A, Protocol Implementation Conformance Statement (PICS) Proforma" (contents page, p. ix; Annex A p. A-1).

Inferences (mine):
- Where a Magenta book has 'shall', it is as "binding and verifiable" as in a Blue Book, inside that book.
- Only a few Magenta books (876.1, 851–855) ship a conformance proforma. Most have no PICS annex, so a claim of conformance to a Magenta book has no standard checklist.
- The Statement of Intent of Magenta says endorsement does not commit anyone to implement "in a prescriptive sense". My reading: the 'shall's bind a claim of conformance to that book, but the book does not oblige anyone to claim it.

### A4. Cross-references that treat Magenta books as normative

- 211.0-B-6 §1.6, p. 1-7, reference [3] is 320.0-M-7, a Magenta Book. The section says: "The following publications contain provisions which, through reference in this text, constitute provisions of this document." The SCID field 3.2.2.1 e) says "SCID (see reference [3]) (10 bits)" (pp. 3-2/3-3). So the Prox-1 Blue Book points to the Magenta SCID rules as part of its own provisions.
- 727.0-B-5 §1.4, p. 1-5, reference [7] is 722.1-M-1 (Magenta), under the same "contain provisions" sentence.
- 355.1-B-1 §1.8, p. 1-4, reference [2] is 354.0-R-1, then a Red Book draft of the key-management book.
- 320.0-M-7 changed type. Its document-control table (p. v) says: "changes the document type from Recommended Standard (Blue Book) to Recommended Practice (Magenta Book)." Its §1.2 (p. 1-1) still says "These procedures shall be followed by all organizations that require a spacecraft identifier to use CCSDS protocols for space communication and by the SANA".

---

## B. Relevance to the project

All page numbers are printed pages of the named book.

### B1. 354.0-M-1 Symmetric Key Management (Dec 2023) — SDLS key management

**What it covers.** QUOTE (§1.1, p. 1-1): "This document recommends standard practices for CCSDS symmetric cryptographic key management. … In particular, this document recommends types of cryptographic keys, a cryptographic key lifecycle, and abstract symmetric key management procedures for communication security in CCSDS-compliant space missions."
QUOTE (§1.2, p. 1-1): "The specification contained in this document is recommended for use on space missions with a requirement for symmetric key management."

**Relation to 355.0 (asked).**
- QUOTE (§1.1, p. 1-1): "All or a subset of these procedures can be instantiated into concrete procedures for specific security protocols, such as the SDLS Extended Procedures (reference [B10])."
- QUOTE (§1.1): "It does not specify any cryptographic operations for the protection of information or data (those are specified in (reference [B2]))."
- Reference list (Annex B, p. B-1): [B9] "Space Data Link Security Protocol. Issue 2. Recommendation for Space Data System Standards (**Blue Book**), CCSDS 355.0-B-2 … July 2022." [B10] "SDLS—Extended Procedures … (**Blue Book**), CCSDS 355.1-B-1 … February 2020."
- QUOTE (Annex A §A1.3–A1.4, p. A-1): "it is recommended that the confidentiality of the key management messages specified in this Recommended Practice be further protected by security protocols such as the Space Data Link Security (SDLS) Protocol (reference [B9]) …"
- So 355.0-B-2 is Blue, not Orange. The project's CONFORMANCE.md line 162 also lists "355.0-B-2 … August 2022"; 354 gives the date as July 2022 (UNVERIFIED which is right, since I did not check that file against the 355.0-B-2 cover. The 355.0-B-2 cover on the box says July 2022).
- 355.0-B-2 itself says key management is out of its scope. QUOTE (355.0-B-2 §4.2.2.1 Note 2, p. 4-4): "Specifying the successful implementation of cryptographic key management is beyond the scope of this document." Also QUOTE (Annex B, p. B-2): "The Security Protocol provides no cryptographic key management protocol." Its §2.2.1 (p. 2-2) says the key management procedures are in the Extended Procedures (355.1-B-1).
- 355.1-B-1 §2 (p. 2-2) QUOTE: "While OTAR is recommended to be implemented, a space mission could also fly with pre-loaded keys only. In this case, OTAR is not required." Same page: the SDLS Extended Procedures lifecycle is "a simplified implementation of the full key management lifecycle as specified in the CCSDS Symmetric Key Management Recommended Practice (reference [2])." It does not use the Suspended state.
- 355.0-B-2 §4.2.2.1 Note 1 (p. 4-4): "It is expected that some missions will choose to define SAs statically and preload/pre-activate them prior to the start of the mission."

**Concrete requirements that stand out.**
- Two key categories only: Master Keys and Session Keys (§3.1.1, p. 3-1).
- A specific key shall be used for only one purpose during its lifetime (§3.1.2.2 for Master, §3.1.3.3 for Session, pp. 3-1, 3-2).
- Lifecycle states shall include Pre-Activation, Active, Deactivated, Destroyed; Suspended is optional (§3.2.1.1, p. 3-2). Newly generated keys shall be in Pre-Activation (§3.2.2.1.1).
- Only Active keys may do cryptographic operations (§3.2.3.1.2–3). Deactivated keys shall be limited to processing already-protected data (§3.2.5.2.2, p. 3-6). A key shall end in Destroyed (§3.2.6.2).
- A compromised key is an attribute, not a state; compromised keys "should be limited to processing of already-protected information" (§3.2.7, p. 3-7).
- Master Keys in Pre-Activation shall be communicated "over a communication channel providing authenticated encryption or equal protection" (§3.2.2.1.3, p. 3-3), with a note allowing operational processes (SECOPS).
- The service list is abstract and "Not all of them need to be implemented by a mission" (§4.1, p. 4-1). Key Activation, Deactivation, Destruction, Verification (challenge/response, §4.3.4), OTAR (§4.3.5), Zeroize, Generation, Suspension.

**Limits stated by the book.**
- QUOTE (§1.2, p. 1-1): "Symmetric key management mechanisms assume the presence of a secure side channel that allows secure distribution of an initial shared secret. The manner in which this initial shared secret is distributed and managed is left for individual agencies or missions to decide."
- QUOTE (§2, p. 2-1): "Non-communication security related aspects such as secure data storage, cryptographic key generation, key escrow, key backup, key renewal, and key reuse are not covered by this recommended practice."
- Its own §1.1 text says key management is "the foundation for the secure generation, storage, distribution, use, and destruction of cryptographic keys", which reads against that exclusion. I note the tension and do not resolve it.
- §1.3 p. 1-2: "The need for and nature of security services … is determined by the outcome of a threat/risk analysis."

**Project issue touched.** SDLS 355.0 telecommand authentication needs key management.

**INFERENCE.**
- The lifecycle rules (one purpose per key, Pre-Activation→Active→Deactivated→Destroyed, key IDs) cost nothing to adopt without OTAR and give the project a defined vocabulary and states.
- 354 does not solve the project's real hobby problem (how the first key gets onto the rocket). It explicitly leaves it to the mission.
- For concrete message formats, the book to read is the Blue 355.1-B-1, not 354.

**UNVERIFIED.**
- I read only the references, overview and a few clauses of 355.1-B-1. I did not diff the final 354.0-M-1 against the Red Book 354.0-R-1 that 355.1-B-1 cites.
- 354 cites 350.8-M-2 ([B8]) for definitions; the current glossary is 350.8-M-3.

**Verdict: useful now** as a design checklist. It adds no new wire format and no Orange dependency.

### B2. 320.0-M-7 + Cor. 1 SCID assignment (Nov 2017, Cor. 1 July 2019) — SCID use

The main PDF at ccsds.org is already consolidated with Cor. 1 (marked "Cor. 1" on its pages). The separate corrigendum PDF (6 pages) lists the changes. Both are on the box.

**What it covers.** QUOTE (§1.1, p. 1-1): "This Recommended Practice establishes the procedures governing requesting, assigning, and relinquishing CCSDS Spacecraft Identifier (SCID) field codes … It specifies the organizations and personnel authorized to participate in the performance of those procedures, the requirements for configuration management, and the acceptable use of SCIDs."
QUOTE (§1.2, p. 1-1): "This Recommended Practice applies to users of the CCSDS protocols … These procedures shall be followed by all organizations that require a spacecraft identifier to use CCSDS protocols for space communication and by the SANA, which registers these identifiers."

**What it says about a small user getting an SCID.**
- Requests go through an Agency Representative (AR) of a CCSDS agency (§3.1.4, §3.3.1, pp. 3-2, 3-4). Definitions note (§1.4, p. 1-2): "Affiliate organizations (Associates or Liaisons) make requests via the AR for their country. If there is no CCSDS agency for their country they can petition the Secretariat to be assigned the responsibility for their country."
- QUOTE (§3.3.2, p. 3-4): "Organizations that are not affiliated with a CCSDS Agency shall contact the CCSDS Secretariat for assistance with Q-SCID assignments."
- QUOTE (§3.1.2, p. 3-1): the Secretariat shall "act as intermediary for SCID requests from organizations not affiliated with a CCSDS Agency by assigning an existing AR to handle the request".
- The SCID is "qualified" by frequency band and frame version. QUOTE (§1.4, p. 1-2): "Q-SCID = FB . TFVN . SCID". Each request must give the uplink and downlink frequency bands assigned by "the agency spectrum manager" (§2.1 p. 2-4; Annex A, p. A-2 item d). QUOTE (§2.1, p. 2-4): "It is expected that every spacecraft, early in its development process, will acquire a frequency assignment for uplink and downlink."
- SANA assigns the numbers: QUOTE (§3.4.2, p. 3-4): "Only in exceptional circumstances will user requests for specific numerical code assignments be honored."
- Requests by form at https://sanaregistry.org/scid/ (§3.3.3).

**Rules that matter to someone using the frame format without a CCSDS-assigned SCID.**
- QUOTE (§2.1, p. 2-2): "The SANA … will no longer assign SCIDs for simulation and testing, nor for ground-based simulators or assemblies. … Missions may wish to assign separate simulation and test SCIDs to manage their own internal datasets, but this must be done by the missions themselves …"
- QUOTE (§3.4.6 Note 2, p. 3-5): "any agency may self-assign SCIDs for simulators as long as these are never used for RF radiation."
- Purpose (§2.1, p. 2-1): "to eliminate the possibility that data from any given CCSDS-compatible vehicle will be falsely interpreted as being from another CCSDS-compatible vehicle during the periods of mission operations; and that commands sent to a CCSDS-compatible vehicle will be received and acted upon by application processes for which they were not intended."
- QUOTE (§2.1, p. 2-1): "Since the space link data structures … are common to many missions, misinterpretation of the identity of a space vehicle is possible unless procedures are developed and followed …"
- Multiple frame versions need separate IDs. QUOTE (§1.4 TFVN note, p. 1-3): "Any spacecraft that uses more than one protocol with a different TFVN may require that two (or more) completely separate and distinct SCIDs be assigned." Cor. 1 adds USLP (TFVN 1100, 16-bit SCID, range 0–65535) to Table 1-1.
- QUOTE (Annex A, p. A-2 item c): "Requestor may ask for only an OID assignment, not just SCID. This is of benefit to organizations that do not use CCSDS link layer protocols but still wish to have a unique, registered, designator."
- Lifetime: QUOTE (§3.2.2): "As quickly as practical after reception of telemetry data, the SCID should be replaced with the OID". SCIDs are reused after relinquishment; they go to the bottom of the stack (§3.5.4).
- Other rules: protoflight or simulator designations get OIDs, not SCIDs (§3.4.6 Note 1); reservation of a sequence for unspecified spacecraft is not accepted (§3.4.6).

**What the book does not say.** A text search of all 39 Magenta texts (and 355.0 / 355.1) for "amateur", "hobby" and "LoRa" found no hits. 320.0-M-7 has no hobbyist route and does not say whether a rocket counts as a "spacecraft". UNVERIFIED: whether it applies to a hobby rocket payload. Also UNVERIFIED: whether Version-3 (TFVN '10') SCIDs 1 and 2 in a UHF band are assigned. The SANA registry page I fetched (https://sanaregistry.org/r/spacecraftid/) lists a UHF-Band record updated 2026-09-30 but I could not enumerate Version-3 entries from it.

**Project issue touched.** The project uses SCIDs in frames. Its IVP.md says "Soak SCIDs are RC IDs (vehicle 1 / station 2), not a Starcom MIB default."

**INFERENCE.**
- The "never used for RF radiation" allowance covers simulators only. A radiating rocket link falls outside it, so if the project wants to say "these IDs are local and unregistered", that is a project decision the book does not bless.
- The only documented path is the Secretariat contact in §3.3.2. If the project ever radiates frames in a band and with a TFVN where those numbers are assigned, §2.1 names the confusion risk.
- Since the Q-SCID is scoped by band and version, a collision matters only to receivers decoding the same band and version.
- No code change is implied. A line in CONFORMANCE.md saying "SCIDs are local, not CCSDS/SANA-registered, per 320.0-M-7 §3.3.2" would be accurate.

**Verdict: useful now** for the rules and for what to document. It gives no easy route for a hobbyist.

### B3. 722.1-M-2 CFDP Unitdata Transfer Layers (July 2026) — CFDP offload

**Status flag.** ccsds.org lists it as a Magenta Book, July 2026 (magenta.txt). The PDF itself carries "DRAFT RECOMMENDED PRACTICE CONCERNING CFDP UNITDATA TRANSFER LAYERS" as its running header, and its document-control table (p. v) says "Current draft update". A "CCSDS 722.1-P-1.1" item also appears in the ccsds.org review sidebar (saved page of Sept 29). UNVERIFIED: whether the PDF is the final published text. The relationship between 722.1-M-2 and 722.1-P-1.1 is also UNVERIFIED.

**What it covers.** QUOTE (§1.1, p. 1-1): "The purpose of this document is to specify the operation of CFDP over the CCSDS Encapsulation Packet Protocol (EPP), CCSDS Space Packet Protocol (SPP), CCSDS Bundle Protocol (BP), and CCSDS Licklider Transmission Protocol (LTP) … In addition, it specifies CFDP UT Layers for User Datagram Protocol (UDP) and Transmission Control Protocol (TCP), which might be used in terrestrial testing …"
QUOTE (§1.2, p. 1-1): "This document applies to any mission or equipment claiming to provide a CCSDS-compliant CFDP capability between two CFDP entities."

**What it adds relative to 727.0-B-5.**
- Text of the book: p. v, "addresses the change from Encapsulation Service to Encapsulation Packet Protocol (CCSDS 133.1-B-3) and extends supported underlying transport layers." The previous issue 722.1-M-1 was titled "Operation of CFDP over Encapsulation Service" (p. v).
- 727.0-B-5 defines only an abstract UT layer. §3.3 (p. 3-6): UNITDATA.request / UNITDATA.indication (UT_SDU, UT Address); Note 1 gives Encapsulation or the DTN Bundle Protocol as examples; Note 5 (p. 3-7) gives the assumed minimum QoS: "with possible errors … incomplete, with some UT_SDUs missing; in sequence".
- 722.1-M-2 maps those primitives onto concrete protocols. For Space Packet (§3.3, pp. 3-2, 3-3):
  - §3.3.2.1 equivalences: CFDP UT_SDU = octet string; CFDP UT address = APID.
  - §3.3.2.2: "the octet string shall be a single, complete CFDP PDU".
  - §3.3.2.4: "The packet secondary header indicator shall be set to absent."
  - §3.3.2.5: "The packet sequence count shall always be used instead of a packet name."
  - §3.3.2.6: "The optional data loss indicator shall be ignored."
  - Note: "the sequence flags in the packet primary header will always be set to '11' (unsegmented user data)."
  - Note p. 3-4: "The APID and packet type to be used for space packets are configured as part of the CFDP Remote Entity Configuration Information."
- §3.1.1 (p. 3-1): "For CCSDS-compliant space links, CFDP shall operate over EPP, SPP, BP, or LTP". §3.1.3: "For terrestrial links, CFDP may also operate using TCP/IP or UDP/IP."
- Annex A1.2 (p. A-1): "EPP … and SPP … do not provide any security functions. Nevertheless, security functions … can be implemented at the data link layer using Space Data Link Security (SDLS) protocols (references [B5] and [B6])." That is 355.0-B-2 and 355.1-B-1.

**Using CFDP over unitdata links.** The book says nothing about the data link underneath (frame size, MTU, loss rates). The only link assumption comes from 727.0-B-5 Note 5 as quoted above. Annex A1.1 (p. A-1): "As these Recommended Practices do not define a new protocol but rather the use of CFDP with existing protocols, no specific security mechanisms are included."

**Drafting slips noticed (QUOTE, supporting the DRAFT label).** §3.3.1 says "The Space Packet Protocol (reference [2])" but [2] is the Encapsulation Packet Protocol and SPP is [3]. §3.7.2.1 says "the service provided by UDP/IP" in the TCP section.

**Project issue touched.** CFDP post-mission file offload. The project's CONFORMANCE.md line 45 already plans CFDP "in Space Packet user data".

**INFERENCE.** The Space Packet mapping rules in §3.3.2 match what the project already plans. They are small and checkable at implementation time. CFDP is deferred, so nothing changes now.

**Verdict: useful later.** Read it with 727.0-B-5 when CFDP is built, and re-check whether the final text changed.

### B4. 351.0-M-1 Security Architecture for Space Data Systems (Nov 2012) — security architecture

**What it covers.** QUOTE (§1.1.1, p. 1-1): "This document is intended as a high-level systems engineering reference to enable engineers to better understand the layered security concepts required to secure a space system."
QUOTE (§1.1.2, p. 1-1): "This document presents a security reference architecture for space data systems and is intended to provide a standardized approach for description of security within data system architectures and high-level designs, which individual working groups may use within CCSDS."

**Concrete content.**
- Few 'shall's (3 in the text). The one body 'shall' is §5.10, p. 5-2: "Security mechanisms shall be capable of recovery after a failure."
- §3.6.1 (p. 3-2) QUOTE: "Every space mission should develop the following security documents in the order listed: a) Security Policy; b) Security Interconnection Policy; c) Mission Security Risk Assessment; d) Mission Security Architecture; e) Security Operating Procedures."
- §4.3.1 (p. 4-4) QUOTE: "All security mechanisms add overhead, but in bandwidth-limited space environments, overhead must be reduced to the absolute minimum required for security. A security system that uses 90% of the available communications resources or a majority of the onboard CPU cycles will be rejected by the mission planners."
- §4.4.1 (p. 4-5): develop the security architecture alongside the functional design: "Such an approach will save money and time during the mission lifecycle."
- Its nomenclature (§1.4.1, p. 1-3) says "this Recommended Standard", a copy leftover in a Magenta Book.

**Project issue touched.** Security architecture and terms.

**INFERENCE.** The five-document list is a sensible, cheap checklist (some lines already exist in the project's docs if you have them). The overhead paragraph is a useful sanity argument when sizing SDLS headers on a slow LoRa link. Nothing else here changes what the project does.

**Verdict: useful later** (documentation and review checklist only).

### B5. 350.8-M-3 Information Security Glossary (Feb 2024) — security terms (skim)

QUOTE (§1.1, p. 1-1): "This document is issued to provide a central source of information security terms and their respective definitions. It is intended that this document will be included as a normative reference in all CCSDS security documents and any CCSDS documents referencing information security."
Defines for example "key management" and "nonce" (PDF pp. 25 and 27).

INFERENCE: use it to keep project terms (Master Key, Session Key, SA, nonce, replay) consistent with the security books. No requirements.

**Verdict: useful later**, reference only.

### B6. 311.0-M-2 RASDS (Dec 2024) — reference architecture (skim)

QUOTE (§1.1, p. 1-1): "The RASDS provides a standardized framework and methodology for modeling space system architectures and related high-level designs, which individual working groups may use within CCSDS, or in International Standards Organization (ISO) TC20/SC13 or ISO TC20/SC14, or for projects within the space agencies or other organizations adopting this Recommended Practice."
QUOTE (§1.1): "It uses a document-based representation and does not propose any specific formal modeling method or tool."
QUOTE (§1.3, p. 1-2): "It is important to keep in mind that not all viewpoints are needed for every task … In many instances, only the Functional and Connectivity Viewpoints may be needed."
Its single 'shall' count is 1 (30 'must', 24 'should' by grep). It has five RASDS viewpoints (Enterprise, Connectivity, Functional, Information, Communications) per 351.0-M-1 §2.3 (pp. 2-1, 2-2); 311 itself lists more in its contents (Physical, Structural derived viewpoints).

INFERENCE: it lets the project describe its system (SAD.md) in standard terms; 184 pages is large for what the project would use. Nothing in it changes behavior.

**Verdict: useful later / optional.**

### B7. 876.1-M-1 SOIS EDS Dictionary of Terms (Mar 2024) — electronic data sheets

**What it covers.** QUOTE (§1.1, p. 1-1): "This document defines the SOIS Specification for Dictionary of Terms (DoT) for Electronic Data Sheets (EDSes) for onboard components. The SOIS DoT provides the vocabulary for electronically defining the interfaces offered by flight components such as sensors, actuators, and software components. This document describes the basic format of the vocabulary, while a SANA registry contains the actual normative details of the vocabulary."
QUOTE (§1.2): "This document applies to any mission or equipment claiming to provide CCSDS SOIS EDS for onboard components."

**What it does not contain.** Figure 2-1 (p. 2-1) shows "SEDS Blue Book (876x0)" as the schema. Reference [1] is the Blue Book CCSDS 876.0-B-1 "XML Specification for Electronic Data Sheets" (April 2019). So this Magenta book is only the vocabulary layer. The text is built around SpaceWire examples (classes SpaceWireProtocol, SpaceWireRMAP etc., pp. 1-4 to 1-5). It carries a normative ICS proforma (Annex A). I did not read 876.0-B-1, and the "SANA DoT" details are in a SANA registry that I did not open.

**Project issue touched.** Electronic data sheets for devices.

**INFERENCE.** For a project of this size, 876.1 does not provide a data-sheet format by itself. If device descriptions are wanted, 876.0-B-1 plus the SANA DoT is the decision. That decision needs reading 876.0-B-1 first (UNVERIFIED whether it fits).

**Verdict: not useful alone.** Revisit only if the project adopts SOIS EDS.

### B8. 882.0-M-1 Low Data-Rate Wireless Communications for Spacecraft M&C (May 2013) — onboard wireless

**What it covers.** QUOTE (§1.2, p. 1-1): "This Recommended Practice is targeted towards monitoring and control systems, typically low data-rate and low-power wireless-based applications."
QUOTE (§1.1): applies "in support of spacecraft ground testing and flight monitoring and control applications."
Definitions (§1.6, p. 1-2): low data-rate "250 kbps or less"; low power "10 mW or less (typical)".

**The actual requirements** (§3.2, p. 3-1; four 'shall's in the whole book):
- Contention-based single-hop: "both the air interface PHY layer and the MAC sublayer shall comply with the IEEE 802.15.4-2011 specification (reference [1])"; such networks "should utilize the 2.4 GHz frequency band".
- Scheduled single-hop: "shall comply with the ISA100.11a-2011 PHY-layer and MAC-sublayer specifications (reference [2])".
- Scope limits (§2.3, p. 2-2): only PHY and MAC, star topology, single-hop. No network layer or above, no multi-hop, no end-to-end acknowledgement.
- Security §2.5 (p. 2-4) relies on keys provided by higher-layer processes, i.e. key management is left outside the book.

LoRa, SX1276 and amateur radio do not appear anywhere in the book (text search).

**Project issue touched.** Onboard low-data-rate wireless.

**INFERENCE.** It would matter only if the project put 2.4 GHz 802.15.4 nodes inside the vehicle. It offers nothing for a LoRa telemetry link or a ground link.

**Verdict: not useful.**

### B9. 851.0-M-1 SOIS Subnetwork Packet Service (Dec 2009) — subnetwork services

QUOTE (§1.1, p. 1-1): "The purpose of this document is to define services and service interfaces provided by the SOIS Subnetwork Packet Service. Its scope is to specify the service only and not to specify methods of providing the service over a variety of onboard data links."
It defines three primitives only: PACKET_SEND.request, PACKET_RECEIVE.indication, PACKET_FAILURE.indication (§3.2.1, p. 3-4) with service classes (best-effort mandatory), priority and channel. §4.1 (p. 4-1): "There is currently no Management Information Base (MIB) associated with this service." §5 (p. 5-2) carries a Service Conformance Statement proforma.
Siblings 852–855 (Memory Access, Synchronisation, Device Discovery, Test) have the same structure; I read their purpose paragraph only (p. 1-1 each).

INFERENCE: it is an abstract API vocabulary, not a protocol. It could name an internal packet-service interface over I2C/SPI, but nothing here requires it. No change.

**Verdict: not useful** (851–855 all).

### B10. Prox-1 link (211.x) cross-support and ops material

No Magenta book is about Proximity-1 operations. The closest are:
- **901.1-M-1 SCCS Architecture Requirements Document (May 2015).** QUOTE (§1.1, p. 1-1): "to define a set of requirements for CCSDS-recommended configurations for secure Space Communications Cross Support (SCCS) architectures. This architecture is to be used as a common framework when CCSDS Agencies 1) provide and use SCCS services, and 2) develop systems that provide interoperable SCCS services." Proximity-1 (211.0/211.1/211.2) appears only as one of the allowed link-layer stacks, for example §5.3.3.2.10 (PDF p. 80) and a protocol table (PDF p. 89). Nothing there changes 211.x use. **Not useful** for the project.
- 901.3-M-1 Functional Resource Model, 902.12-M-2 Service Management Common Data Entities, 902.13-M-1 Abstract Event Definition, 921.2-M-1 CSTS guidelines: agency ground-station and cross-support service modelling. **Not useful.**

### B11. The rest, one line each

| Book | Scope (from magenta.txt / the book) | Verdict |
|---|---|---|
| 141.1-M-1 Atmospheric Characterization for Optical Links | "atmospheric data … that affect space-to-ground free-space optical communications" | Clearly irrelevant |
| 506.0-M-2 Delta-DOR Operations; 506.3-M-1 Quasar Catalog | navigation technique for deep-space ranging | Clearly irrelevant |
| 520.1-M-1 MO Reference Model; 523.1-M-1 / 523.2-M-1 MO MAL Java / C++ API | Mission Operations service reference and APIs (§1.1) | Clearly irrelevant |
| 650.0-M-3 OAIS; 651.0-M-1 PAIMAS; 652.0-M-2 / 652.1-M-3 repository audit; 653.0-M-1 info preparation | archives and digital preservation | Clearly irrelevant |
| 852.0 / 853.0 / 854.0 / 855.0-M-1 SOIS Memory Access / Synchronisation / Device Discovery / Test | same family as 851 | Not useful |
| 881.0-M-1 RFID Inventory | RFID inventory on missions | Clearly irrelevant |
| 901.3-M-1, 902.12-M-2, 902.13-M-1 (+Cor. 1), 921.2-M-1 (+Cor. 1) | cross-support service modelling | Not useful |
| 914.0, 915.1, 915.2, 915.5, 916.1, 916.3-M-2 (+Cor. 1) SLE APIs | C++ APIs for Space Link Extension | Clearly irrelevant |

---

## C. Things I could not verify or did not do

- Whether 722.1-M-2 on ccsds.org is final or a draft with a mislabelled color (the PDF says DRAFT).
- Whether a hobby rocket payload counts as a "spacecraft" under 320.0-M-7, and whether Version-3 UHF SCIDs 1/2 are assigned in the SANA registry.
- 354.0-M-1 against 354.0-R-1 (the Red Book 355.1-B-1 cites). Not diffed.
- Content of 876.0-B-1 (SEDS) and most of 355.1-B-1.
- I excluded one NASA NTRS slide set that gave a "Field Guide to CCSDS Book Colors" because it is not a ccsds.org primary source.
- The 'shall' counts come from a text search of extracted PDF text and may include some in front matter or Statements of Intent.
- Printed page numbers I give for A02.1-Y-4 are derived from the extracted footers. Spot-check them if you quote them formally.
