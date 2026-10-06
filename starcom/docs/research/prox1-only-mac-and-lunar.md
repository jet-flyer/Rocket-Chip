# Prox-1 books only: security/MAC (Q1) and lunar draft changes (Q2)

Prepared on the box, 6 Oct 2026 (CT). Every statement below is a quote, a page reference, or a count of words in the text. I give no reading of what a clause means.

## 0. Method and file checks

- Books searched (Prox-1): 211.0-B-6 (July 2020), 211.1-B-4 (Dec 2013), 211.2-B-3 (Oct 2019), 211.0-P-6.2 (April 2026), 211.1-P-4.2 (April 2026), 211.2-P-3.2 (cover April 2026; Authority page says January 2026), 235.1-R-1 (May 2026), 210.0-G-2 (Green Book, Dec 2013, with EC1).
- Non-Prox books, used only where the task said: 355.0-B-2 (§2.1 only) and 350.0-G-3 (§5.3.5 only, plus §5.3.1, which has the same sentence). These are labeled "(non-Prox)".
- Text: fresh `pdftotext -layout` of each PDF. The supplied .txt copies are byte-identical to that output for all books except 350.0-G-3. For 350.0-G-3, I used my own layout extraction of the PDF.
- Page numbers: the printed footer label ("Page N"). The footer comes at the END of the page it labels.
- 211.1-P-4.2: the front-matter pages carry footers "Page 1-1" to "Page 1-7". The body also starts at "Page 1-1". Unless I say "front matter", a 211.1-P-4.2 page number below is a body page.
- Every quote below was checked by a script against the page text (whitespace collapsed). In 211.1-P-4.2, the "±" sign is a Symbol-font glyph that pdftotext gives as U+F0B1. I write it as "±". I checked page 4-9 in the PDF image, and it shows "± 5%".
- "[...]" marks words I left out of a quote, or text from another table column.
- Table rows are quoted in layout order. Status codes (M, O) can land in the middle of a wrapped cell.
- Counts are numbers of text LINES that contain the term (whole text, contents pages included).

## Q1. Security, authentication, MAC, SDLS, encryption, keys

### Q1.1 Term counts (lines that contain the term)

| Term | 211.0-B-6 | 211.0-P-6.2 | 211.1-B-4 | 211.1-P-4.2 | 211.2-B-3 | 211.2-P-3.2 | 235.1-R-1 | 210.0-G-2 |
|---|---|---|---|---|---|---|---|---|
| authenticat | 3 | 6 | 0 | 0 | 3 | 3 | 6 | 0 |
| MAC (case-sensitive, whole word) | 42 | 24 | 14 | 14 | 7 | 5 | 14 | 36 |
| security | 15 | 15 | 7 | 7 | 13 | 13 | 15 | 0 |
| secur | 16 | 15 | 7 | 7 | 14 | 14 | 15 | 0 |
| SDLS | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 355.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 350 (whole number) | 1 | 1 | 0 | 0 | 0 | 0 | 1 | 0 |
| encrypt | 0 | 0 | 0 | 0 | 4 | 4 | 0 | 0 |
| key (word start) | 6 | 5 | 2 | 6 | 0 | 0 | 7 | 7 |
| integrity | 5 | 5 | 0 | 0 | 1 | 1 | 5 | 5 |
| cryptograph | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| cipher | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| confidential | 4 | 6 | 0 | 0 | 0 | 0 | 6 | 0 |
| message authentication | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| Space Data Link Security (line-based; see note) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| spoof | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| replay | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Notes on the counts (facts about the text):
- "SDLS" appears 0 times and "355.0" appears 0 times in all eight Prox-1 books.
- "cryptograph" appears 0 times in all eight. "message authentication" appears 0 times in all eight.
- "350" as a whole number appears only in the reference to 350.0-G-3 (211.0-B-6 [H4], 211.0-P-6.2 [C4], 235.1-R-1 [J4]). 235.1-R-1 also has "350k" in a data-rate table on p. K-3. That is a separate string; the whole-number count does not include it.
- All "key" hits are figure legends ("Key:"), the words "Shift Keying" (FSK, QPSK, GMSK), or common English in 210.0-G-2 ("key drivers", "key component", "key idea", "Key: Hailing pair").
- "encrypt" appears only in 211.2-B-3 and 211.2-P-3.2, in Annex D (quoted below).
- "authenticat" appears only in the security annexes (quoted below).
- "Space Data Link Security" appears once in all eight books: 235.1-R-1 §2.1 (quoted below). The phrase breaks across two lines there, so the line count shows 0. A search with line breaks removed found it.

### Q1.2 How the Prox-1 books expand "MAC"

In all eight books, every hit for "MAC" is Medium (or Media) Access Control. No book uses "MAC" for a message authentication code. The acronym lists say:

> "MAC medium access control"
>
> — 211.0-B-6, Annex I (acronyms), p. I-1

> "MAC Medium Access Control"
>
> — 211.1-B-4, Annex D (acronyms), p. D-1

> "MAC Medium Access Control"
>
> — 211.2-B-3, Annex F (acronyms), p. F-1

> "MAC medium access control"
>
> — 211.0-P-6.2, Annex D (acronyms), p. D-1

> "MAC Media Access Control"
>
> — 211.1-P-4.2, Annex D (acronyms), p. D-1

> "MAC Medium Access Control"
>
> — 211.2-P-3.2, Annex F (acronyms), p. F-1

> "MAC media access control"
>
> — 235.1-R-1, Annex L (acronyms), p. L-2

> "MAC Medium Access Control"
>
> — 210.0-G-2, Annex A (acronyms), p. A-1

Example uses in the body text:

> "This Recommended Standard defines the DLL (Framing, Medium Access Control [MAC], and Input/Output [I/O] sublayers)."
>
> — 211.0-P-6.2, §1.2, p. 1-1

> "Media Access Control (MAC) Sublayer (reference [3])"
>
> — 211.1-P-4.2, §2.2, p. 2-1

> "Subsection 4.2 specifies the functions of the MAC sublayer, which: – accepts Proximity-1 directives both from the local vehicle controller and across the Proximity link to control its operations;"
>
> — 211.0-P-6.2, §2.2.2.4, p. 2-12


### Q1.3 Security annexes, quoted in full

All Prox-1 security text is in annexes marked "(INFORMATIVE)". The one exception is the 235.1-R-1 §2.1 phrase in Q1.4.

#### 211.0-B-6, Annex E "SECURITY, SANA, AND PATENT CONSIDERATIONS (INFORMATIVE)"

> "E1 SECURITY CONSIDERATIONS E1.1 SECURITY BACKGROUND It may be required that security services be applied to the payloads carried by IP datagrams over CCSDS space links. The specification of such security services is outside the scope of this document but is discussed in the following subsections. If there is a reason to believe that non-authorized entities might be able to view or obtain the payload data, and if there is a need to ensure that non-authorized entities not be able to view or obtain the data, then confidentiality needs to be applied. If there is a need to ensure that the payload data has not been modified in transit without such modification being recognized, then integrity needs to be applied. If the authenticity of the source of the payload data is required (e.g., the payload contains a command), then authentication needs to be applied. It is possible for a single datagram to require all three security services to ensure that the payload is not disclosed, not altered, and authentic. E1.2 SECURITY CONCERNS As stated in the previous subsection, various security services might need to be applied to the IP datagram depending on the threat, the mission security policy (or policies), and the desire of the mission planners. While these security concerns are valid, they are outside the scope of this document. This document assumes that either upper or lower layers of the OSI model will provide the security services. That is, if authenticity at the granularity of a specific user is required, it is best applied at the Application Layer. If less granularity is required, it could be applied at the Network or Data Link Layers. If integrity is required, it can be applied at either the Application, Network, or Data Link Layer. If confidentiality is required, it can be applied at either the Application Layer, the Network Layer, or the Data Link Layer. Reference [H4] provides more information regarding the choice of service and where it can be implemented. E1.3 POTENTIAL THREATS AND ATTACK SCENARIOS Without authentication, unauthorized commands or software might be uploaded to a spacecraft or data retrieved from a source masquerading as the spacecraft. Without integrity, corrupted commands or software might be uploaded to a spacecraft. Without integrity, corrupted telemetry might be retrieved from a spacecraft, and the result could be that an incorrect course of action is taken. Sensitive or private information might be disclosed to an eavesdropper if confidentiality is not applied to the data. E1.4 CONSEQUENCES OF NOT APPLYING SECURITY The security services are out of scope of this document and should be applied at layers above or below those specified in this document. However, should there be a requirement for authentication, and if it is not implemented, unauthorized commands or software might be loaded onto a spacecraft. If integrity is not implemented, erroneous commands or software might be loaded onto a spacecraft, potentially resulting in the loss of the mission. If confidentiality is not implemented, data flowing to or from a spacecraft might be visible to unauthorized entities, resulting in disclosure of sensitive or private information."
>
> — 211.0-B-6, Annex E, E1.1–E1.4, p. E-1–E-2

> "The Application of Security to CCSDS Protocols. Issue 3."
>
> — 211.0-B-6, Annex H, reference [H4], p. H-1

> "CCSDS 350.0-G-3"
>
> — 211.0-B-6, Annex H, reference [H4], p. H-1

#### 211.0-P-6.2, Annex B "SECURITY, SANA, AND PATENT CONSIDERATIONS (INFORMATIVE)"

Text fact: the heading of B1.2 prints as one run-on line: "B1.2 A SINGLE DATAGRAM MAY REQUIRE ALL THREE SECURITY SERVICES TO ENSURE THAT THE PAYLOAD IS CONFIDENTIAL, UNALTERED, AND AUTHENTIC.SECURITY CONCERNS".

> "B1 SECURITY CONSIDERATIONS B1.1 SECURITY BACKGROUND Security services may be required for IP datagram payloads over CCSDS space links based on threat assessment, mission security policies, and mission specifications. While security service specification is outside the scope of this document, a brief overview of the three available services is provided below: – Confidentiality: Protects payload data from unauthorized access. – Integrity: Protects payload data from undetected modification during transit. – Authentication: Verifies the source of payload data (e.g., for command data). B1.2 A SINGLE DATAGRAM MAY REQUIRE ALL THREE SECURITY SERVICES TO ENSURE THAT THE PAYLOAD IS CONFIDENTIAL, UNALTERED, AND AUTHENTIC.SECURITY CONCERNS As stated in the previous subsection, various security services might need to be applied to the IP datagram depending on the threat, mission security policies, and mission planner specifications. This document assumes that either upper or lower layers of the OSI model will provide the security services depending on required granularity: – Fine-grained user authentication: Application Layer; – General authentication: Network or DLLs; – Data integrity protection: Any layer (Application, Network, or Data Link); – Data confidentiality: Any layer (Application, Network, or Data Link). Reference [C4] provides more information regarding the choice of service and where it can be implemented. B1.3 POTENTIAL THREATS AND ATTACK SCENARIOS When authentication, integrity, and confidentiality protections are not implemented, spacecraft systems face the following threats: – Without authentication unauthorized commands or software could be uploaded to spacecraft from unverified origins or data may be retrieved from unverified sources masquerading as legitimate users. – Without integrity corrupted commands or software might reach the spacecraft, while corrupted telemetry from the spacecraft could lead to incorrect operational decisions. – Without confidentiality sensitive or private information may be exposed to eavesdroppers during transmission. B1.4 CONSEQUENCES OF NOT APPLYING SECURITY The security services are out of scope of this document and should be applied at layers above or below those specified in this document. However, when required security controls are not properly implemented, the following vulnerabilities arise: – If authentication is not implemented, unauthorized commands or software may be loaded onto the spacecraft. – If integrity is not implemented, erroneous commands or software could be uploaded, potentially resulting in mission loss. – If confidentiality is not implemented, data transmitted to or from the spacecraft becomes visible to unauthorized entities, leading to disclosure of sensitive information."
>
> — 211.0-P-6.2, Annex B, B1.1–B1.4, p. B-1–B-2

> "CCSDS 350.0-G-3"
>
> — 211.0-P-6.2, Annex C, reference [C4], p. C-1

#### 235.1-R-1, Annex I "SECURITY, SANA, AND PATENT CONSIDERATIONS (INFORMATIVE)"

> "I1 SECURITY CONSIDERATIONS I1.1 BACKGROUND Security services may be required for IP datagram payloads over CCSDS space links based on threat assessment, mission security policies, and mission specifications. While security service specification is outside the scope of this document (see reference [J4] for implementation guidance), a brief overview of the three available services is provided below: – Confidentiality: Protects payload data from unauthorized access. Without this service, sensitive or private information might be disclosed to eavesdroppers, and data flowing to or from a spacecraft might be visible to unauthorized entities. – Integrity: Protects payload data from undetected modification during transit. Without this service, corrupted/erroneous commands or software might be uploaded to a spacecraft, or corrupted telemetry might be retrieved from a spacecraft, potentially resulting in incorrect course of action or loss of the mission. – Authentication: Verifies the source of payload data (e.g., for command data). Without this service, unauthorized commands or software might be uploaded/loaded onto a spacecraft, or data retrieved from a source masquerading as the spacecraft. A single datagram may require all three security services to ensure that the payload is confidential, unaltered, and authentic. I1.2 SECURITY CONCERNS As stated in the previous subsection, various security services might need to be applied to the IP datagram depending on the threat, mission security policies, and mission planner specifications. This document assumes that either upper or lower layers of the OSI model will provide the security services depending on required granularity: – Fine-grained user authentication: Application Layer; – General authentication: Network or DLL; – Data integrity protection: Any layer (Application, Network, or Data Link); – Data confidentiality: Any layer (Application, Network, or Data Link). Reference [J4] provides more information regarding the choice of service and where it can be implemented. I1.3 POTENTIAL THREATS AND ATTACK SCENARIOS When authentication, integrity, and confidentiality protections are not implemented, spacecraft systems face the following threats: – Without authentication, unauthorized commands or software could be uploaded to spacecraft from unverified origins, or data may be retrieved from unverified sources masquerading as legitimate users. – Without integrity, corrupted commands or software might reach the spacecraft, while corrupted telemetry from the spacecraft could lead to incorrect operational decisions. – Without confidentiality, sensitive or private information may be exposed to eavesdroppers during transmission. I1.4 CONSEQUENCES OF NOT APPLYING SECURITY The security services are out of scope of this document and should be applied at layers above or below those specified in this document. However, when required security controls are not properly implemented, the following vulnerabilities arise: – If authentication is not implemented, unauthorized commands or software may be loaded onto the spacecraft. – If integrity is not implemented, erroneous commands or software could be uploaded, potentially resulting in mission loss. – If confidentiality is not implemented, data transmitted to or from the spacecraft becomes visible to unauthorized entities, leading to disclosure of sensitive information."
>
> — 235.1-R-1, Annex I, I1.1–I1.4, p. I-1–I-2

> "CCSDS 350.0-G-3"
>
> — 235.1-R-1, Annex J, reference [J4], p. J-1

#### 211.1-B-4, Annex B "SECURITY, SANA, AND PATENT CONSIDERATIONS (INFORMATIVE)"

> "B1 SECURITY CONSIDERATIONS B1.1 INTRODUCTION The security concern involves radio frequency jamming of the forward and/or return link signal. Jamming of the signal could lead to the total loss of data, and potential navigation errors if Doppler tracking is disrupted. B1.2 SECURITY CONCERNS WITH RESPECT TO THE CCSDS DOCUMENT The forward and return link signals are vulnerable to jamming, although there are several mitigating factors. The forward signal cannot be transmitted from Earth since the currently specified channels are reserved by ITU to other services on the Earth surface. A deliberate attempt at jamming the forward signal in violation of the ITU regulations would disrupt a very large number of terrestrial links in light of the difference in distances involved between a terrestrial user and a Proximity-1 user on Mars. Concerning the return signal (received by the orbiter in a deep space scenario), there is limited availability of equipment capable of generating enough uplink power to effectively jam the spacecraft receiver at interplanetary distances. B1.3 POTENTIAL THREATS AND ATTACK SCENARIOS Jamming of the signal could result in the loss of data or of Doppler measurements. During a critical maneuver (e.g., probe landing on Mars), jamming could cause uncertainty in the lander trajectory. B1.4 CONSEQUENCES OF NOT APPLYING SECURITY TO THE TECHNOLOGY While these security issues are of concern, they are out of scope with respect to this document. Jamming denies all communications, and protection must be accomplished by Physical-Layer techniques such as spread spectrum and/or frequency hopping. This problem is somewhat mitigated by the amount of power and the size of antennas needed to communicate with the spacecraft, or by the need of having a jamming source in Mars orbit."
>
> — 211.1-B-4, Annex B, B1.1–B1.4, p. B-1

#### 211.1-P-4.2, Annex B "SECURITY, SANA, AND PATENT CONSIDERATIONS (INFORMATIVE)"

The B1 heading is "B1 SECURITY CONSIDERATIONS FOR MARTIAN ENVIRONMENT" (211.1-B-4: "B1 SECURITY CONSIDERATIONS"). The text of B1.1–B1.4 is word-for-word the same as 211.1-B-4 (script comparison). Pages: B-1.

> "B1 SECURITY CONSIDERATIONS FOR MARTIAN ENVIRONMENT"
>
> — 211.1-P-4.2, Annex B, B1 heading, p. B-1

#### 211.2-B-3, Annex D "SECURITY, SANA, AND PATENT CONSIDERATIONS (INFORMATIVE)"

> "D1 SECURITY CONSIDERATIONS D1.1 BACKGROUND It is assumed that security is provided by encryption, authentication methods, and access control to be performed at higher layers (Application and/or Transport Layers). Mission and service providers are expected to select from recommended security methods, suitable to the specific application profile. Specification of these security methods and other security provisions is outside the scope of this Recommended Standard. The C&S Sublayer has the objective of delivering data with the minimum possible amount of residual errors. The Proximity-1 codes ensure a very low error probability, and the Frame Error Control Field is used to ensure that residual errors are detected and the frame flagged. There is an extremely low probability of additional undetected errors that may escape this scrutiny. These errors may affect the encryption process in unpredictable ways, possibly affecting the decryption stage and producing data loss, but will not compromise the security of the data. D1.2 SECURITY CONCERNS Security concerns in the areas of data privacy, authentication, access control, availability of resources, and auditing are to be addressed in higher layers and are not related to this Recommended Standard. The C&S Sublayer does not affect the proper functioning of methods used to achieve such protection at higher layers, except for undetected errors, as explained above. The physical integrity of data bits is protected from channel errors by the coding systems specified in this Recommended Standard. In case of congestion or disruption of the link, the C&S Sublayer provides methods for frame re-synchronization. D1.3 POTENTIAL THREATS AND ATTACK SCENARIOS An eavesdropper can receive and decode the codewords, but will not be able to get to the user data if proper encryption is performed at a higher layer. An interferer could affect the performance of the decoder by congesting it with unwanted data, but such data would be rejected by the authentication process. Such interference or jamming must be dealt with at the Physical Layer and through proper spectrum regulatory entities. D1.4 CONSEQUENCES OF NOT APPLYING SECURITY There are no specific security measures prescribed for the C&S Sublayer. Therefore consequences of not applying security are only imputable to the lack of proper security measures in other layers. Residual undetected errors may produce additional data loss when the link carries encrypted data."
>
> — 211.2-B-3, Annex D, D1.1–D1.4, p. D-1–D-2

#### 211.2-P-3.2, Annex D "SECURITY, SANA, AND PATENT CONSIDERATIONS (INFORMATIVE)"

The text of D1.1–D1.4 is the same as 211.2-B-3, with one difference (script comparison): D1.4 has "Therefore," where 211.2-B-3 has "Therefore". Pages: D-6 to D-7.

> "There are no specific security measures prescribed for the C&S Sublayer. Therefore, consequences of not applying security are only imputable to the lack of proper security measures in other layers."
>
> — 211.2-P-3.2, Annex D, D1.4, p. D-7

### Q1.4 Security text outside the annexes

> "Supervisor Protocol Data Unit (SPDU) exchange, which manages SPDUs (section 3) for directives, transceiver status, time, ranging, and COP-P/Space Data Link Security reporting."
>
> — 235.1-R-1, §2.1 c), p. 2-7

In the eight Prox-1 books, I found no other non-annex sentence with "secur", "authenticat", "encrypt", "SDLS", "355.0", "cryptograph", or "cipher". The other "security" lines are contents-page entries and reference titles. I also searched with line breaks removed. That search found only the phrase above.

### Q1.5 Does any clause CALL FOR authentication, or point to SDLS?

- "shall": no sentence in the eight books has both "shall" and "authenticat".
- No clause points to SDLS or to 355.0 (0 hits each). The security annexes point to 350.0-G-3 ([H4], [C4], [J4]).
- These are the security-annex sentences with "should", "need(s) to", or "must" (quoted above in full):

> "If the authenticity of the source of the payload data is required (e.g., the payload contains a command), then authentication needs to be applied."
>
> — 211.0-B-6, Annex E, E1.1, p. E-1

> "The security services are out of scope of this document and should be applied at layers above or below those specified in this document."
>
> — 211.0-B-6, Annex E, E1.4, p. E-2

> "The security services are out of scope of this document and should be applied at layers above or below those specified in this document."
>
> — 211.0-P-6.2, Annex B, B1.4, p. B-2

> "The security services are out of scope of this document and should be applied at layers above or below those specified in this document."
>
> — 235.1-R-1, Annex I, I1.4, p. I-2

> "Jamming denies all communications, and protection must be accomplished by Physical-Layer techniques such as spread spectrum and/or frequency hopping."
>
> — 211.1-B-4, Annex B, B1.4, p. B-1

> "Such interference or jamming must be dealt with at the Physical Layer and through proper spectrum regulatory entities."
>
> — 211.2-B-3, Annex D, D1.3, p. D-1

How 211.0-B-6 defines these words, and how it treats informative text:

> "a) the words ‘shall’ and ‘must’ imply a binding and verifiable specification; b) the word ‘should’ implies an optional, but desirable, specification; c) the word ‘may’ implies an optional specification;"
>
> — 211.0-B-6, §1.5.2.1, p. 1-5

> "NOTE – These conventions do not imply constraints on diction in text that is clearly informative in nature."
>
> — 211.0-B-6, §1.5.2.1 NOTE, p. 1-5

### Q1.6 210.0-G-2 (Green Book) on security

- "secur" appears 0 times (fresh layout text, and also raw pdftotext). "authenticat" 0, "encrypt" 0, "SDLS" 0, "355.0" 0, "cryptograph" 0.
- 210.0-G-2 has no security annex. Its annexes are A (acronyms), B (CRC), and C (lessons learned).
- "MAC" = Medium Access Control (quoted in Q1.2).
- The "integrity" hits are:

> "applies error detection on each PDU to ensure data integrity"
>
> — 210.0-G-2, §2.3.6.2, p. 2-23

> "high-integrity delivery of SDU(s)"
>
> — 210.0-G-2, §2.3.6.3, p. 2-24

> "a three-way handshake is necessary to ensure the integrity of the hailing process"
>
> — 210.0-G-2, §4.1.4, p. 4-20

> "frame integrity check field"
>
> — 210.0-G-2, Annex B, p. B-1

### Q1.7 Version-3 and Version-4 frame clauses

No Version-3 or Version-4 frame clause in the Prox-1 books contains any of the search terms. Under the rule in this file, I quote no Version-3/Version-4 clause as "bearing on security". I make no judgment about other clauses. These are the clauses that say where Version-4 is specified:

> "Version-4 Transfer Frame: A Unified Space Data Link Protocol (USLP) Transfer Frame. (See annex C of reference [7] and 3.3 of this document.)"
>
> — 211.0-B-6, §1.5.1.2, p. 1-5

> "Alternatively, the Version-4 Transfer Frame may be used in lieu of the Version-3 frame as the Transfer Frame protocol data unit over the Proximity-1 Coding Sublayer. In this case, the functions provided by the Proximity-1 Frame Sublayer (see 2.1.4.3) are replaced by the USLP Space Data Link Protocol."
>
> — 211.0-B-6, §3.3, p. 3-16

> "b) the Version-4 (USLP) Transfer Frame defined in reference [8]. The Version-4 Transfer Frame may be used in lieu of the Version-3 frame as the Transfer Frame PDU over the Proximity-1 C&S sublayer. In this case, the functions provided by the Proximity-1 Frame sublayer (see 2.2.2.3) are replaced by the USLP Space Data Link Protocol reference [8]."
>
> — 211.0-P-6.2, §3.1, p. 3-1

> "Only transfer frames of the same version number shall be contained in the same PLTU stream once the link has been established, to avoid mixing fixed and variable length frames in the same symbol stream."
>
> — 211.0-P-6.2, §3.1.1.1, p. 3-1

> "SPDUs are transmitted between transceivers in the data fields of Version-3 (Proximity-1) or Version-4 (USLP) transfer frames, which are called Protocol frames (P-frames)."
>
> — 235.1-R-1, §2.1.3, p. 2-9

Text facts: in 211.0-B-6, reference [7] is 732.1-B-1 (Oct 2018). In 211.0-P-6.2, reference [8] is 732.1-B-3 (June 2024).

> "CCSDS 732.1-B-1"
>
> — 211.0-B-6, §1.7 reference [7], p. 1-8

> "CCSDS 732.1-B-3"
>
> — 211.0-P-6.2, §1.7 reference [8], p. 1-8

### Q1.8 Non-Prox books (cross-reference only)

> "(The Security Protocol is not applicable for use with the Proximity-1 Space Data Link Protocol.)"
>
> — 355.0-B-2 (non-Prox), §2.1 (non-Prox), p. 2-1

> "SDLS is not applicable for use with the Proximity-1 Space Data Link Protocol. For Proximity-1, data link security services are best implemented above the I/O sublayer as shown in figure 5-4. The security services are applied to the User Data. All Proximity-1 protocol handling is carried out as it normally would be. This is analogous to Transport Layer Security (TLS)."
>
> — 350.0-G-3 (non-Prox), §5.3.5 (non-Prox), p. 5-7

### Q1 summary (facts only)

- No Prox-1 book has a "shall" clause that calls for authentication.
- No Prox-1 book names SDLS or 355.0.
- "MAC" in the Prox-1 books is always Medium/Media Access Control.
- The security text is in INFORMATIVE annexes, plus one phrase in 235.1-R-1 §2.1 ("COP-P/Space Data Link Security reporting").
- **The books do not say** anything about a message authentication code, key management, cryptographic algorithms, anti-replay, or SDLS for Proximity-1. Search terms: authentication, authenticat, MAC, message authentication, security, secur, SDLS, 355.0, 350, encrypt, key, integrity, cryptograph, cipher, confidential, spoof, replay, Space Data Link Security.

## Q2. What the lunar drafts add or change (other than S-band frequencies)

Drafts: 211.1-P-4.2, 211.0-P-6.2, 211.2-P-3.2, 235.1-R-1. Blue Books: 211.0-B-6, 211.1-B-4, 211.2-B-3.

Scope statements in the drafts:

Table cells are interleaved in the layout text. '[...]' below marks text from other table columns.

> "Adds extension to S-Band [...] for lunar [...] communications."
>
> — 211.1-P-4.2, Document Control (front matter), Status cell for 211.1-P-4.2, p. 1-5

> "NOTE – Changes from the current issue are too extensive to permit markup."
>
> — 211.1-P-4.2, Document Control (front matter), p. 1-5

> "Removed data service [...] sublayer and COP-P, [...] removed Annex C (Mars Odyssey), D (MRO). Transferred P1 state tables, diagrams, and SPDU formats."
>
> — 211.0-P-6.2, Document Control, Status cell for 211.0-P-6.2, p. vi

> "Type 5 SPDU format is envisioned for S-band Lunar operations but is not limited to them."
>
> — 235.1-R-1, §3.3.6 NOTE 2, p. 3-8

> "Type 4 SPDU directives shall be used for space link supervisory configuration and control of the transceiver and its operation at S-band."
>
> — 235.1-R-1, Annex D, D1.1, p. D-2

> "Type 5 SPDU directives shall be used for space link supervisory configuration as well as control of the transceiver and its operation at S-band."
>
> — 235.1-R-1, Annex E, E1.1, p. E-3

> "Uses of the S-Band Proximity-1 standard outside the lunar environment are not yet addressed by this specification. As such, the manufacturer will need to perform proper engineering to tailor the standard to scenarios not described in this document."
>
> — 211.1-P-4.2, §5.1 NOTE, p. 5-13

### Q2.1 Suppressed carrier

The four waveform families are kept apart below, one heading each.

#### Q2.1a Pink Book GMSK (211.1-P-4.2, S-band, "suppressed carrier option")

> "Two types of modulations can be used: Filtered PCM/PM/Bi-Phase-L (residual carrier option) and Gaussian Minimum Shift Keying (GMSK) (suppressed carrier option)."
>
> — 211.1-P-4.2, §5.1.6.1, p. 5-16

> "Suppressed carrier Gaussian Minimum Shift Keying (GMSK) modulation BTs=0.25 (where B refers to the one-sided 3-dB bandwidth of the filter) with pre-coding as shown in Figure 5-2."
>
> — 211.1-P-4.2, §5.1.6.3.1, p. 5-17

Text fact: the §5.1.6.3.1 sentence has no "shall", "should", "may", or "must".

> "R es is equal to 2* Rs when Bi-phase-L waveform is used, while Res is equal to Rs when GMSK waveform is used."
>
> — 211.1-P-4.2, §5.2.4 NOTE, p. 5-19

> "5 Modulation 5.1.6 M Filtered PCM/PM/Bi-Phase-L, GMSK"
>
> — 211.1-P-4.2, PICS A2.2.2 item 5 (S-band), p. A-6

235.1-R-1 (Type 5 and Type 4 LEC directives):

> "a) ‘0000’ = PCM/PM/Bi-phase-L (filtered); b) ‘0001’ = GMSK; c) ‘0010’ = OQPSK (filtered); d) ‘0011’ = BPSK (filtered); e) ‘0100’ = PCM/PSK/PM; f) ‘0101’ = PCM/PM/NRZ-L (filtered);"
>
> — 235.1-R-1, Annex E, E2.2.11, p. E-7–E-8

> "NOTE – Only options a) and b) are supported by Proximity-1 PL reference [5]."
>
> — 235.1-R-1, Annex E, E2.2.11 NOTE, p. E-8

> "The uncoded option with suppressed carrier modulation and without randomization cannot guarantee sufficient bit transitions resulting in an unreliable link."
>
> — 235.1-R-1, Annex E, E2.2.12 NOTE 2, p. E-9

> "NOTE – This field is ignored when using suppressed carrier modulation."
>
> — 235.1-R-1, Annex E, E2.2.14 NOTE, p. E-9

> "a) ‘0000’ = PCM/PM/Bi-phase-L (filtered); b) ‘0001’ = GMSK; c) ‘0010’ = RESERVED BY CCSDS;"
>
> — 235.1-R-1, Annex D, D2.2.12, p. D-8

Matching Blue Book text: "GMSK" appears 0 times in 211.0-B-6, 211.1-B-4, 211.2-B-3, and 210.0-G-2. "suppress" appears 0 times in 211.1-B-4. **The Blue Books do not say** anything about GMSK. Search terms: GMSK, Gaussian, suppress, Minimum Shift.

#### Q2.1b 211.0-B-6 suppressed-carrier PSK (SET PL EXTENSIONS "Mode Select"), and its 235.1-R-1 copy

Text fact: in 211.0-B-6, the suppressed carrier option appears only in Annex B (SET PL EXTENSIONS) and in informative Annex G (MRO). 211.1-B-4 has "suppress" 0 times. 211.0-P-6.2 has "suppress" 0 times. Its Document Control says it removed Annex D (MRO) and moved the SPDU formats (see above).

> "Bits 7-8 of the SET PL EXTENSIONS directive shall indicate the type of carrier suppression used: a) ‘00’ = Suppressed Carrier (Requires transmit side utilize Modulation Index of 90° and transmit/receive sides utilize Differential Mark Encoding/Decoding); b) ‘01’ = Residual Carrier; c) ‘10’ = Reserved; d) ‘11’ = Reserved. Option a) is not required for cross-support except for those missions required to interoperate with NASA MRO. (See annex G.)"
>
> — 211.0-B-6, Annex B, B1.7.7, p. B-17

> "Option a) is not required for cross-support except for those missions required to interoperate with NASA MRO. (See annex G.)"
>
> — 211.0-B-6, Annex B, B1.7.8, p. B-17

> "a) ‘00’ = No Modulation; b) ‘01’ = PSK; c) ‘10’ = FSK; d) ‘11’ = QPSK. Options c) and d) are not required for cross-support."
>
> — 211.0-B-6, Annex B, B1.7.9, p. B-17

> "For the suppressed carrier mode, the transmitted symbols are Non-Return to Zero-Level (NRZ-L) phase encoded, and all of the transmit power goes into the data modulation. The modulation index is 90 degrees. Since there is no residual carrier to establish a phase reference at the receive end, a Costas loop is used to reconstruct an equivalent phase reference from the data signal."
>
> — 211.0-B-6, Annex G (INFORMATIVE), G1, p. G-1

235.1-R-1 Annex B (Type 1 SPDU) has the same fields. Its notes are worded differently:

> "a) ‘00’ = Suppressed Carrier (requires Modulation Index of 90° on transmit side and Differential Mark Encoding/Decoding on transmit/receive sides); b) ‘01’ = Residual Carrier; c) ‘10’ = Reserved; d) ‘11’ = Reserved. NOTE – Option a) is required only for missions that interoperate with NASA MRO."
>
> — 235.1-R-1, Annex B, B7.7, p. B-14

> "NOTE – Differential coding must be enabled only for missions that interoperate with NASA Mars Reconnaissance Orbiter (MRO)."
>
> — 235.1-R-1, Annex B, B7.5 NOTE, p. B-14

> "NOTE – NRZ-L is required only for missions that interoperate with NASA MRO."
>
> — 235.1-R-1, Annex B, B7.8 NOTE, p. B-15

#### Q2.1c E2d FSK

The Table 3-1 text for E2d is the same in 211.1-B-4 and 211.1-P-4.2:

> "E2d: E2 elements with a descoped receiver capable of receiving an FSK modulated carrier. These elements transmit using PSK modulation. NOTE – E2d radio equipment is intended to be used in microprobes. This option is not required for cross support."
>
> — 211.1-B-4, §3.1, Table 3-1, p. 3-1

> "E2d: E2 elements with a descoped receiver capable of receiving an FSK modulated carrier. These elements transmit using PSK modulation. NOTE – E2d radio equipment is intended to be used in microprobes. This option is not required for cross support."
>
> — 211.1-P-4.2, §3.1, Table 3-1, p. 3-1

> "1.4 Radio equipment category E2d table 3-1 O.1"
>
> — 211.1-P-4.2, PICS A2.2.2 item 1.4 (S-band), p. A-5

> "c) ‘10’ = Frequency Shift Keying (FSK); d) ‘11’ = Quadrature Phase Shift Keying (QPSK). NOTE – FSK and QPSK are not required for cross-support."
>
> — 235.1-R-1, Annex B, B7.9, p. B-15

#### Q2.1d Residual-carrier PCM/PM

> "3.3.5.1 The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier. 3.3.5.2 Residual carrier shall be provided with modulation index of 60° ± 5%."
>
> — 211.1-B-4, §3.3.5.1–3.3.5.2 (UHF), p. 3-8

> "4.1.6.1 The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier. 4.1.6.2 Residual carrier shall be provided with modulation index of π/3 rad-pk ± 5%."
>
> — 211.1-P-4.2, §4.1.6.1–4.1.6.2 (UHF), p. 4-9

> "Residual carrier shall be provided with one of the following modulation index: – 0.0 rad-pk (No Modulation) – 0.4 rad-pk – 0.6 rad-pk – 0.8 rad-pk – π/3 rad-pk (60 degrees) – 1.15 rad-pk – 1.3 rad-pk – 1.4 rad-pk"
>
> — 211.1-P-4.2, §5.1.6.2.2 (S-band), p. 5-16

> "When using Filtered PCM/PM/Bi-Phase-L, the emitted spectrum for in-situ lunar link must be compliant with the emission mask in reference [7]."
>
> — 211.1-P-4.2, §5.1.6.2.5 (S-band), p. 5-17

> "it is recommended that Bi-Phase-L waveform must be filtered with a Butterworth filter of the 3rd order, with a cut-off frequency equal to 3.3 times the coded symbol rate."
>
> — 211.1-P-4.2, §5.1.6.2.6 (S-band), p. 5-17

> "In a similar trade-off, a higher modulation index provides greater carrier suppression but demands stricter Butterworth filtering."
>
> — 211.1-P-4.2, §5.1.6.2.6 NOTE, p. 5-17

> "For the case of filtered Bi-Phase L modulation, the spectral lines at the even multiples of the normalized symbol rate shall not be higher than ‐20 dBc assuming a resolution bandwidth of 4kHz (reference [8])."
>
> — 211.1-P-4.2, §5.2.4.1, p. 5-20

> "a) ‘000’ = 0 rad/pk (No Modulation); b) ‘001’ = 0.4 rad/pk; c) ‘010’ = 0.6 rad/pk; d) ‘011’ = 0.8 rad/pk; e) ‘100’ = π/3 rad/pk (60 degrees); f) ‘101’ = 1.15 rad/pk; g) ‘110’ = 1.3 rad/pk; h) ‘111’ = 1.4 rad/pk."
>
> — 235.1-R-1, Annex E, E2.2.14, p. E-9

### Q2.2 Two-way / full-duplex without a residual carrier

**The books do not say.** No sentence in any Prox-1 book, Blue or draft, puts full duplex (or two-way) together with GMSK, suppressed carrier, or "no residual carrier". Search terms: full duplex, full-duplex, duplex, two-way, suppress, GMSK, residual, Costas. I checked co-occurrence on the same page. The only page with both is the 235.1-R-1 acronym list (p. L-1).

Related text, quoted without comment:

> "DLL-100 Full duplex operations 6.4.2, M Tables 6-7, 6-8, 6-9"
>
> — 211.0-B-6, PICS DLL-100, p. A-9

> "39 Full duplex operations 5.4.2, Tables 5-6, 5-7, M 5-8"
>
> — 235.1-R-1, PICS item 39, p. A-6

> "b) ‘001’ = Full Duplex; c) ‘010’ = Half Duplex; d) ‘011’ = Simplex Transmit; e) ‘100’ = Simplex Receive;"
>
> — 235.1-R-1, Annex E, E2.2.7, p. E-7

Text fact: in 235.1-R-1, E2.2.7 (Duplex/Simplex) and E2.2.11 (Modulation, with GMSK as option b) are fields of the same LEC directive. The text has no rule that links the two fields.

### Q2.3 Coherent turnaround, or ranging, without a carrier

Coherency requirement (Blue vs draft):

> "Link elements in category E2c (table 3-1), for which range and range-rate measurements are needed, shall have transmit/receive frequency coherency capability. (See 3.4.5 for Doppler tracking and acquisition requirements.)"
>
> — 211.1-B-4, §3.1.2, p. 3-1

> "Link elements in category E2c (table 3-1), for which range and range-rate measurements are needed, shall have transmit/receive frequency coherency capability"
>
> — 211.1-P-4.2, §3.1.2, p. 3-1

> "Transmit/receive frequency coherency capability is mandatory for link elements in category E2c (table 3-1), for which GMSK and range-rate measurements are needed."
>
> — 211.1-P-4.2, PICS A2.2.2 NOTE 1 (S-band), p. A-7

> "Mandatory for link elements in category E2c (table 3-1), for which range and range-rate measurements are needed."
>
> — 211.1-P-4.2, PICS A2.2.1 NOTE 1 (UHF), p. A-5

Text fact: the S-band PICS Note 1 says "GMSK and range-rate". §3.1.2 and the UHF PICS Note 1 say "range and range-rate".

S-band turnaround and Doppler (211.1-P-4.2):

> "Forward and return link frequencies may be coherently related or non-coherent. For coherent mode, the turn-around ratio is 240/221."
>
> — 211.1-P-4.2, §5.1.3, p. 5-14

> "a) Doppler frequency range: ±80 kHz; b) Doppler frequency rate: 1) 600 Hz/s (non-coherent mode), 2) 1.2 kHz/s (coherent mode)."
>
> — 211.1-P-4.2, §5.2.5, p. 5-20

> "In the case of the coherent RF interface between E2c elements the effect of the coherent turnaround ratio of the responding element has to be considered."
>
> — 211.1-P-4.2, §5.2.5 NOTE 6, p. 5-20

Ranging: "ranging" appears 0 times in 211.0-B-6, 211.1-B-4, 211.2-B-3, 211.0-P-6.2, 211.1-P-4.2, 211.2-P-3.2, and 210.0-G-2. It appears on 86 lines of 235.1-R-1. 235.1-R-1 adds:

> "RANGING is a PL interface variable which shall control whether ranging is modulated onto the transmitted carrier. When RANGING=on, the Ranging Code is modulated on to the radiated carrier; when RANGING=off, the Ranging Code is not modulated on to the radiated carrier."
>
> — 235.1-R-1, §5.2.2.6, p. 5-8

> "Bit 14 of the LEC directive shall contain the transceiver coherent or non-coherent option as per below: a) ‘0’ = Coherent; b) ‘1’ = Non-coherent."
>
> — 235.1-R-1, Annex E, E2.2.9, p. E-7

> "– The PN RANGING directive should support one-way (pseudo-range) and two-way ranging. – Only a single PN ranging code sequence will be specified for Proximity-1 use. – The PN RANGING directive will follow the SPDU Type 5 format for second generation lunar use."
>
> — 235.1-R-1, Annex E, E2.8.1, p. E-18

> "These parameters include chip rate, PN code type, PN ranging mode (coherent, non-coherent, regenerative, non-regenerative), ranging mod index, and the epoch time-tag for the start of the PN sequence."
>
> — 235.1-R-1, Annex E, E2.8.1, p. E-18

> "a) ‘00’ = Ranging Off; b) ‘01’ = One-way Ranging (pseudo-range); c) ‘10’ = Two-way Non-Regenerative Ranging (turnaround ranging); d) ‘11’ = Two-way Regenerative Ranging."
>
> — 235.1-R-1, Annex E, E2.8.4, p. E-20

> "NOTE – Only option a) is supported by the Proximity-1 PL reference [5]. For options b) and c), reference [9] applies."
>
> — 235.1-R-1, Annex E, E2.8.5 NOTE, p. E-20

> "69 PN RANGING ANNEX E O"
>
> — 235.1-R-1, PICS item 69, p. A-7

Text facts: in 235.1-R-1, reference [5] is "211.1-B-5 ... forthcoming" and reference [9] is 414.1-B-3. "ranging" (any case) appears 0 times in 211.1-P-4.2. E2.8.2 lists "Directive Name (3 bits)". Figure E-9 and E2.8.3.1 use 4 bits ("Bits 0-3"). E2.8.3.2 gives '1100' for PN RANGING.

Old Blue Book text on coherent Doppler (informative Odyssey annex):

> "The Tone Beacon Mode can be used to perform Doppler measurements."
>
> — 211.0-B-6, Annex F (INFORMATIVE), p. F-1

**The books do not say** how coherent turnaround or ranging works with GMSK or with a suppressed carrier. The one place where GMSK and range-rate share a sentence is 211.1-P-4.2 PICS Note 1 (quoted above). 235.1-R-1 E2.8 (PN RANGING) has no sentence with GMSK, suppressed, or residual. Search terms: ranging, PN, turnaround, turn-around, coherent, GMSK, suppress, residual, carrier, Costas.

### Q2.4 Hailing: required modulation

Blue Books:

> "For interoperability at UHF, the default hailing channel shall be Channel 1"
>
> — 211.1-B-4, §3.3.2.3.1, p. 3-6

> "The hailing channel is enterprise specific. The default configuration of the Physical Layer parameters (established by the enterprise) defines the hailing channel frequencies that enable two transceivers to initially communicate (via a demand or negotiation process) so that they can establish a configuration for the data services portion of the session. Hailing channel assignments are defined in the Physical Layer."
>
> — 211.0-B-6, §6.2.4.15, p. 6-14

> "Hailing_Data_Rate shall represent the data rate assigned during the Hail activity. NOTE – Proximity data rates are defined in the Physical Layer."
>
> — 211.0-B-6, §6.2.4.16, p. 6-14

**The Blue Books do not say** which modulation, coding, or symbol rate to use for hailing. The string "Hailing shall be performed" appears 0 times in 211.1-B-4 and 211.0-B-6. Search terms: hailing + modulation, Hailing shall be performed, Hailing Symbol Rate, 8000 symbols, 8,000.

Drafts:

> "Hailing shall be performed using the PCM/PM/Bi-Phase-L Modulation with a modulation index equal to π/3 rad-pk, uncoded coding option and with symbol rate equal to 8000 symbols per second."
>
> — 211.1-P-4.2, §4.1.2.2 (UHF), p. 4-6

> "For interoperability at S-Band, the default hailing channel shall be Channel 0 with Channel 9 being an optional hailing channel."
>
> — 211.1-P-4.2, §5.1.2.1 (S-band), p. 5-13

> "In the Proximity link radio equipment, the hailing channel shall be distinct from the working channel."
>
> — 211.1-P-4.2, §5.1.2.2 (S-band), p. 5-14

> "Hailing should be performed using the PCM/PM/Bi-Phase-L Modulation with a modulation index equal to π/3 rad-pk, the LDPC 1/2 K=1024 coding option and symbol rate equal to 2000 symbol per seconds."
>
> — 211.1-P-4.2, §5.1.2.4 (S-band), p. 5-14

> "Filtered PCM/PM/Bi-Phase-L must be used to perform the hailing procedure with modulation index equal to π/3 rad-pk."
>
> — 211.1-P-4.2, §5.1.6.2.4 (S-band), p. 5-17

Text fact: §5.1.2.4 uses "should" and §5.1.6.2.4 uses "must" for the hailing modulation.

> "5.1.2.5 Hailing should be performed using Left Polarization when the hailing is performed on Channel 0. 5.1.2.6 Hailing should be performed using Right Polarization when the hailing is performed on Channel 9."
>
> — 211.1-P-4.2, §5.1.2.5–5.1.2.6, p. 5-14

> "During the Hailing Phase, if the hailing is performed on Channel 0, Left-Hand Circular Polarization (LHCP) shall be used for both forward and return links. If the hailing is performed on Channel 9 (optional hailing channel), Right Hand Circular Polarization (RHCP) shall be used for both forward and return links."
>
> — 211.1-P-4.2, §5.1.5, p. 5-15

> "Support for the coded symbol rate to be used for hailing (2000 sps) is mandatory."
>
> — 211.1-P-4.2, PICS A2.2.2 NOTE 3 (S-band), p. A-7

> "Hailing-channel parameters for UHF-Band (Martial environment) shall be: – Hailing Channel Number: Channel 1 forward and return link;1 – Hailing Symbol Rate: 8,000 symbols/second; – Coding: Uncoded; – Modulation: Bi-Phase-L; – Polarization: Right Hand Circular; – Transceiver Mode: Proximity-1;"
>
> — 235.1-R-1, Annex H (NORMATIVE), H2.1, p. H-1

> "Haling-channel parameters for S Band (lunar environment) shall be: [...] – Default Hailing Channel Number: 0;3 – Optional Hailing Channel Number: 9;3 – Hailing Coded Symbol Rate: 2000 symbols/second – Coding: LDPC (n=2048,k=1024) rate 1/2 code defined in reference [4]; – Modulation: Bi-phase-L; – Polarization: • Left Hand Circular for Default Hailing Channel 0; • Right Hand Circular for Optional Hailing Channel 9; – Transceiver Mode: USLP; – Physical Channel ID (PCID): 0;2 – Coherency: Non-coherent."
>
> — 235.1-R-1, Annex H (NORMATIVE), H2.2, p. H-1–H-2

> "The default hailing symbol rate in S-band is 2,000 symbols/s, which is LDPC (2048,1024) encoded."
>
> — 235.1-R-1, Annex E, E2.2.20 NOTE 2, p. E-11

> "Using the demand function to move to an initial working channel immediately after establishing a link on a hailing channel shall be mandatory for all link establishment and comm change directives."
>
> — 235.1-R-1, §5.1.1.1, p. 5-1

> "Hailing_Data_Rate shall represent the data rate assigned during the Hail activity. Similarly, the Hailing_Symbol_Rate shall represent the symbol rate assigned during the Hail activity."
>
> — 235.1-R-1, §5.2.3.15, p. 5-12

Text fact (235.1-R-1 Annex K, INFORMATIVE, Table K-2, p. K-2): the row "Hailing, no negotiation / Caller demands RTN link" lists Channel "ch0", Function "Demand", Modulation "GMSK", Coding "LDPC 1/2", Symbol Rate "128k", Freq "2265". The "Hailing with negotiation" rows list "GMSK" for "Caller sends RTN query" and for "Responder sends RTN NACK" ("Retry required"). Then "Caller sends revised RTN query" lists "SP-L/PM".

### Q2.5 Acquisition, idle, and carrier-only timing

MIB definitions, Blue vs draft:

> "6.2.4.3 Carrier_Only_Duration Carrier_Only_Duration represents the time that shall be used to radiate an unmodulated carrier at the beginning of a transmission. 6.2.4.4 Acquisition_Idle_Duration Acquisition_Idle_Duration represents the time that shall be used to radiate the idle sequence pattern after carrier only to enable the receiving transceiver to achieve symbol synchronization and decoder lock. 6.2.4.5 Tail_Idle_Duration Tail_Idle_Duration represents the time that shall be used to radiate the idle sequence pattern at the end of a transmission"
>
> — 211.0-B-6, §6.2.4.3–6.2.4.5, p. 6-12

> "5.2.3.3 Carrier_Only_Duration Carrier_Only_Duration represents the duration for radiating an unmodulated carrier at the beginning of a transmission. 5.2.3.4 Acquisition_Idle_Duration Acquisition_Idle_Duration represents the duration for radiating the idle sequence pattern after the carrier-only period, enabling the receiving transceiver to achieve symbol synchronization and decoder lock. 5.2.3.5 Tail_Idle_Duration Tail_Idle_Duration represents the duration for radiating the idle sequence pattern at the end of a transmission"
>
> — 235.1-R-1, §5.2.3.3–5.2.3.5, p. 5-10

Text fact: the 211.0-B-6 definitions contain "shall". The 235.1-R-1 definitions do not. "Carrier_Only_Duration" appears 0 times in 211.0-P-6.2.

Physical Layer, Table 3-2 Note 2. The only wording difference is "Data Link Layer" (211.1-B-4) vs "DLL" (211.1-P-4.2):

> "An MIB parameter, Carrier_Only_Duration, is used in the Data Link Layer to control the duration of the carrier-only transmission (TRANSMIT = on and MODULATION = false)."
>
> — 211.1-B-4, §3.2.2.1, Table 3-2 NOTE 2, p. 3-3

> "An MIB parameter, Carrier_Only_Duration, is used in the DLL to control the duration of the carrier-only transmission (TRANSMIT = on and MODULATION = false)."
>
> — 211.1-P-4.2, §3.2.2.1, Table 3-2 NOTE 2, p. 3-3

C&S Sublayer, LDPC acquisition alignment (Blue vs draft):

> "When LDPC coding is used, the Acquisition sequence shall start on the first bit of the PN sequence. NOTE – The requirement to start the Acquisition sequence on the first bit of the PN sequence applies only when LDPC coding is used."
>
> — 211.2-B-3, §3.3.2.3, p. 3-4

> "When LDPC coding is used, octet synchronization shall be maintained between the PLTUs and LDPC codewords as follows. a) The first LDPC message block shall start with the first bit of the Acquisition Sequence, and this shall be the first bit of the PN sequence defined in 3.3.2.2. b) The Acquisition Sequence shall be an integer number of octets in lengths. c) Idle data inserted between PLTUs shall start with the first bit of the PN sequence defined in 3.3.2.2 and shall be an integer number of octets in length. NOTE – These synchronization requirements apply only when LDPC coding is used."
>
> — 211.2-P-3.2, §3.3.2.3, p. 3-4

> "NOTE – In case of LDPC, the acquisition sequence must be composed of an integer number of octets, as specified in section 3.3.2.3b."
>
> — 211.2-P-3.2, §3.3.3.2.2 NOTE, p. 3-5

**The books do not say** anything about carrier-only, acquisition, or idle timing for GMSK or suppressed carrier. I found no page where Carrier_Only_Duration or Acquisition_Idle_Duration occurs with GMSK, suppress, or residual. Search terms: Carrier_Only_Duration, Acquisition_Idle_Duration, Tail_Idle_Duration, carrier only, carrier-only, acquisition, idle, GMSK, suppress.

### Q2.6 Partial implementation, tied options ("if X then Y shall"), PICS notes, conditional mandatory items

Warnings against partial implementation: **The books do not say** this in those words. "partial" occurs in 211.0-B-6 §8.3.3 (p. 8-2, "partial data units are not delivered to the end user"), 211.0-P-6.2 §6.3.3 ("partial data units are not delivered"), 235.1-R-1 Table K-2 ("Partial accept"), and 210.0-G-2 (p. 2-7 "partial functionality through remotely commanded changes"; p. 2-23 "partial SDUs" and "partial delivery of an image"). None of these says "partial implementation". Search terms: partial, subset, incomplete, full implementation, fully implement, must implement, shall implement, must support, shall support, must be implemented, interoperab.

Applicability sentence (present in every book; 211.1-P-4.2 shown):

> "must be implemented when this document is used as a basis for cross support."
>
> — 211.1-P-4.2, §1.3 (body), p. 1-1

PICS rules and conditional items, 211.1-P-4.2:

> "An implementation claiming conformance must satisfy the mandatory requirements referenced in the RL."
>
> — 211.1-P-4.2, PICS A1.1, p. A-1

> "O.1: Support for one of these categories must be indicated. C1: IF (Category = E2c) THEN M ELSE O. C2: IF (Radio equipment category NOT E1) THEN M ELSE N/A."
>
> — 211.1-P-4.2, PICS A2.2.2 (S-band), p. A-6

> "The working channel must be different from the hailing channel; therefore, radios must support at least two channels (the hailing and one of the data channels)."
>
> — 211.1-P-4.2, PICS A2.2.2 NOTE 2 (S-band), p. A-7

> "2 Channel 1 is recommended; Channel 0 is used by legacy systems; Channel N is to be used by radios with only one channel (for hailing and working) or if agreed to. 3 The working channel has to be the same as the hailing channel for radios with only one channel."
>
> — 211.1-P-4.2, PICS A2.2.1 NOTES 2–3 (UHF), p. A-5

> "6.1 Forward coded symbol rates 5.1.7.1 M From 1000 to (symbols/s) (note 4) 4096000 6.2 Return coded symbol rates 5.1.7.1 M From 1000 to (symbols/s) (note 4) 4000000"
>
> — 211.1-P-4.2, PICS A2.2.2 items 6.1–6.2 (S-band), p. A-6

> "The Proximity-1 link shall support coded symbol rate Rcs value in the range from 1000 to 4096000 symbols per second."
>
> — 211.1-P-4.2, §5.1.7.1, p. 5-17

Text facts (211.1-P-4.2 PICS): items 6.1 and 6.2 cite "(note 4)", but the S-band NOTES list stops at 3. Item 6.2 says "4000000" and §5.1.7.1 says "4096000". PICS A1.1 and A2 still name "CCSDS 211.1-B-4".

211.2-P-3.2 (C&S):

> "Depending on the selected Data Link Layer specific protocol, Type 1 or Type 5 of reference [3], some of the coding option listed above may be not available. In particular: – Coding option a) is only possible with Bi-Phase-L Modulation both for Type 1 and Type 5 directives. – Coding option b) is only possible for Type 1 directives. – Coding option c) is possible with both Type 1 and Type 5 directives. – Coding options d) and e) are only Type 5 directives."
>
> — 211.2-P-3.2, §3.4.2.2 NOTE 1, p. 3-7

> "Only transfer frames of the same version number shall be contained in the same PLTU stream."
>
> — 211.2-P-3.2, §3.2.4.3, p. 3-2

> "O.1 It is mandatory to support at least one of these items."
>
> — 211.2-P-3.2, PICS A2.2, p. A-4

Text fact: 211.2-P-3.2 reference [3] is "211.0-B-7 ... Forthcoming". The Type 5 directives are in 235.1-R-1 Annex E.

235.1-R-1:

> "An implementation claiming conformance must satisfy the mandatory requirements referenced in the RL."
>
> — 235.1-R-1, PICS A1.1, p. A-1

> "If a conditional requirement is inapplicable, N/A should be used."
>
> — 235.1-R-1, PICS A1.3, p. A-2

> "54 SET TRANSMITTER B2 M PARAMETERS 55 SET CONTROL PARAMETERS B3 M 56 SET RECEIVER PARAMETERS B4 M 57 SET V(R) B5 O 58 REPORT REQUEST B6 M 59 SET PL EXTENSIONS B7 O"
>
> — 235.1-R-1, PICS A2.2.7 items 54–59, p. A-6

> "62 LEC ANNEX D O SPDU Type 5 Directives 63 LEC ANNEX E M – Demand; O – M Query/Response 64 REPORT REQUEST ANNEX E M 65 SET V(R) ANNEX E O 66 REPORT SOURCE ANNEX E SPACECRAFT ID O 67 SERVICE REQUEST ANNEX E M 68 SET FIXED-LENGTH FRAME ANNEX E O 69 PN RANGING ANNEX E O"
>
> — 235.1-R-1, PICS A2.2.7 items 62–69, p. A-7

Text fact: the 235.1-R-1 PICS status list (A1.2) defines M, O, and O.<n> only. It has no "C" code.

> "This directive shall precede the LEC directive in the SPDU."
>
> — 235.1-R-1, Annex E, E2.7.1, p. E-16

> "Only options a, b, f, and k are supported by the Proximity-1 C&S sublayer, reference [4]. For options c, d, g, j, and l, see reference [2]."
>
> — 235.1-R-1, Annex E, E2.2.12 NOTE 1, p. E-9

The 211.0-B-6 text in Q2.1b ("not required for cross-support except for those missions required to interoperate with NASA MRO") and the 235.1-R-1 notes in Q2.1b ("required only for missions that interoperate with NASA MRO") are the other option-tying sentences I found.

### Q2.7 New modulation, data-rate, or coding options

I sort these by where the text sits. I do not judge whether an option is band-specific.

Outside an S-band section:

> "This document defines two channel codes for use on Proximity-1 links: an optional convolutional code and an optional LDPC code."
>
> — 211.2-B-3, §3.4.1, p. 3-5

> "This document defines four channel codes for use on Proximity-1 links: an optional convolutional code and three optional LDPC code."
>
> — 211.2-P-3.2, §3.4.1, p. 3-6

> "a) no coding; b) convolutional code (see 3.4.3); c) LDPC code (see 3.4.4) k=1024 and R=1/2; d) LDPC code (see 3.4.5) k=4096 and R=2/3. e) LDPC code (see 3.4.6) k=7136 and R=7/8."
>
> — 211.2-P-3.2, §3.4.2.2, p. 3-6

> "The current data rate is configured using the Link Establishment & Control directive defined as part of the Type 5 directives of reference [3], and it is selected in such a way that the corresponding Rcs value falls in the interval 1000 sps – 4096000 sps."
>
> — 211.2-P-3.2, §3.4.2.1 NOTE 3, p. 3-6

> "Each LDPC message block shall be encoded using the LDPC code (n=6144, k=4096) rate 2/3 code defined in reference [2]."
>
> — 211.2-P-3.2, §3.4.5.3, p. 3-9

> "Designers should note that this length-255-bit pseudo-randomizer may introduce spectral lines at 1/255 of the symbol rate, and these may be significant in some systems."
>
> — 211.2-P-3.2, §3.4.7.2.8 NOTE, p. 3-12

> "July 2025 Current issue: 211.2-B-4"
>
> — 211.2-P-3.2, Document Control, p. iv

Text facts (211.2-P-3.2): the Document Control table lists "211.2-B-4" (July 2025) as "Current issue" with "New LDPC coding options". PICS item 5 (Coding option: LDPC) cites "3.4.4, 3.4.5, 3.4.6".

> "a) ‘00000’ = Uncoded; b) ‘00001’ = LDPC(2048,1024); c) ‘00010’ = LDPC( 8192,4096); d) ‘00011’ = LDPC(32768, 16384); e) ‘00100’ = Reserved by CCSDS; f) ‘00101’= LDPC(6144,4096); g) ‘00110’= LDPC(24576, 16384);"
>
> — 235.1-R-1, Annex E, E2.2.12, p. E-8

> "j) ‘01001’= LDPC(20480, 16384); k) ‘01010’ = LDPC(8160,7136); l) ‘01011’ = Convolutional Code(7,1/2);"
>
> — 235.1-R-1, Annex E, E2.2.12, p. E-9

> "MODCOD is a placeholder for the ability to table drive the combined settings of both the Modulation and Coding fields."
>
> — 235.1-R-1, Annex E, E2.2.10, p. E-7

> "Symbol rate values in the range from 1,000 symbols/s to 4,096,000 symbols/s are represented with a precision of 0.1% from the defined Proximity-1 channel symbol rates as measured at the output of the transmitter."
>
> — 235.1-R-1, Annex E, E2.2.20 NOTE 3, p. E-11

The 235.1-R-1 modulation list (E2.2.11) and its NOTE are quoted in Q2.1a. 235.1-R-1 D1.1 and E1.1 tie Type 4 and Type 5 to "operation at S-band" (quoted at the top of Q2).

Inside the 211.1-P-4.2 S-band section (§5): the modulation index list (§5.1.6.2.2, Q2.1d), GMSK (§5.1.6.3.1, Q2.1a), and the continuous coded symbol rate range (§5.1.7.1, Q2.6). Blue 211.1-B-4 has 13 discrete rates:

> "The Proximity-1 link shall support one or more of the following 13 discrete forward and return values for the coded symbol rate Rcs shown in symbols per second: 1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000, 512000, 1024000, 2048000, 4096000."
>
> — 211.1-B-4, §3.3.6.1, p. 3-8

### Q2.8 Cross-reference text facts (no comment)


> "under the control of the directives defined in Annex B (UHF-Mars scenario) or Annex E (S band-Moon Scenario) of reference [3]."
>
> — 211.1-P-4.2, §3.2.1.2, p. 3-2

> "CCSDS 211.0-B-6"
>
> — 211.1-P-4.2, §1.7 reference [3], p. 1-5

> "Annex F of reference [3] defines these enterprise-specific parameters."
>
> — 211.1-P-4.2, §5.1.2 NOTE 4, p. 5-14

> "NOTE 2 – The symbol rates for the S-band Proximity-1 forward and return link are in Section 5.1.8"
>
> — 211.1-P-4.2, §4.1.7.1 NOTE 2, p. 4-9

- In 211.0-B-6, Annex E is "SECURITY, SANA, AND PATENT CONSIDERATIONS" and Annex F is "NASA MARS SURVEYOR PROJECT 2001 ODYSSEY ORBITER PROXIMITY SPACE LINK CAPABILITIES". In 235.1-R-1, Annex E is the Type 5 SPDU and Annex F is the MIB.
- 211.1-P-4.2 has no heading "5.1.8". The S-band rates heading is "5.1.7 PROXIMITY-1 RATES".
- 235.1-R-1 reference [6] is printed "211.0-P-6.0 ... forthcoming".

## Buzz quote check (211.1-P-4.2)

| Buzz cite | Result |
|---|---|
| §5.1.6.1, p. 5-16 | OK. Exact wording (quoted in Q2.1a). |
| §5.1.6.3.1, p. 5-17 | OK. Full sentence in Q2.1a. The sentence has no shall/should/may/must. |
| §5.1.6.2.4, p. 5-17 | OK. The sentence ends "...with modulation index equal to π/3 rad-pk." Note: §5.1.2.4 (p. 5-14) says "should" for the same hailing modulation. |
| UHF §4.1.6.2, p. 4-8 | CORRECTION: the page is 4-9 (checked in the PDF image). Wording: "Residual carrier shall be provided with modulation index of π/3 rad-pk ± 5%." The clause has one value. The word "only" is not in it. |
| PICS Note 1, p. A-7 | OK. Exact: "Transmit/receive frequency coherency capability is mandatory for link elements in category E2c (table 3-1), for which GMSK and range-rate measurements are needed." (§3.1.2 and the UHF PICS say "range and range-rate".) |
| §5.1.1, p. 5-13 | OK. Wording: "5.1.1.1 The forward frequency band (Lunar Orbit to Lunar Surface) shall be from 2025 to 2110 MHz. 5.1.1.2 The return frequency band (Lunar Surface to Lunar Orbit) shall be from 2200 to 2290 MHz." The text says "to", not a dash. |

> "Residual carrier shall be provided with modulation index of π/3 rad-pk ± 5%."
>
> — 211.1-P-4.2, §4.1.6.2 (Buzz check), p. 4-9

> "5.1.1.1 The forward frequency band (Lunar Orbit to Lunar Surface) shall be from 2025 to 2110 MHz. 5.1.1.2 The return frequency band (Lunar Surface to Lunar Orbit) shall be from 2200 to 2290 MHz."
>
> — 211.1-P-4.2, §5.1.1 (Buzz check), p. 5-13

## Where the books do not say

1. A message authentication code, key management, cryptography, anti-replay, or SDLS for Proximity-1. (Prox-1 books: SDLS 0, 355.0 0, cryptograph 0, "message authentication" 0, cipher 0, spoof 0, replay 0.)
2. A "shall" clause that calls for authentication. (None in the eight Prox-1 books.)
3. Security in 210.0-G-2. ("secur" 0, "authenticat" 0, "encrypt" 0.)
4. GMSK, in any Blue Book. ("GMSK" 0 in 211.0-B-6, 211.1-B-4, 211.2-B-3, 210.0-G-2.)
5. Full duplex or two-way operation without a residual carrier. (Search terms in Q2.2.)
6. Coherent turnaround or ranging with GMSK or a suppressed carrier, other than the 211.1-P-4.2 PICS Note 1 sentence. (Search terms in Q2.3.)
7. Hailing modulation, coding, or symbol rate, in the Blue Books.
8. Carrier-only, acquisition, or idle timing for GMSK or a suppressed carrier. (Search terms in Q2.5.)
9. A warning against partial implementation, in those words. (Search terms in Q2.6.)
