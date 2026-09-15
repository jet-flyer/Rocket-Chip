# Starcom Blue Book coverage

**Status:** Living. Clause walk, not a published PICS.

This file answers two questions `CONFORMANCE.md` does not:

1. **Walk** — which Blue Book sections have we implemented, left 0, or not opened yet.
2. **Neighbors** — when you sit in a clause, what else is load-bearing (the book’s own “see § / as specified in …”, plus cites already in the glossary, SAD, ICD, and CONFORMANCE).

The PDF is the authority. If this file disagrees with the book, the book wins; fix this file.

## SSOT

| File | Owns |
|------|------|
| the Blue Book PDF | the requirement |
| [`CONFORMANCE.md`](CONFORMANCE.md) | the **published tick** (PICS mark or explicit non-tick) |
| **this file** | **status + neighbors + test pointer** for the walk |
| [`SAD.md`](SAD.md) | working bit tables / field maps |
| [`GLOSSARY.md`](GLOSSARY.md) | term → section index |
| [`ICD.md`](ICD.md) | codec / engine handshake |

Do not copy SAD bit layouts or glossary definitions here. Do not retcon a CONFORMANCE Level to Full from a row here — CONFORMANCE’s “Full needs a test pointer” rule still applies.

## Status

| Status | Means |
|--------|--------|
| **unread** | ToC is listed; this sitting has not walked the clause. Not a silent 0. |
| **0** | Walked. Not implemented (or not in `starcom::ccsds`). |
| **partial** | Walked. Some SHALLs coded and tested; leftovers named in Notes. |
| **Full** | Walked. In-scope SHALLs coded **and** a test pointer exists. |
| **out of scope** | Walked. Starcom will not implement (named reason). |
| **forbidden** | The book forbids it. Do not build. |

`Best effort` is a CONFORMANCE PHY/bearer level, not a clause status here.

**Grain:** one row per ToC section by default. Explode to a PICS option or table-event only where CONFORMANCE already talks that way (211.2 O.1 encode vs decode; mixed-stream ban; V-3 DFC O.1; COP-1 Table 5-1 leftovers).

**Maintenance:** update the row in the **same commit** as the TU. An untouched matrix is a failed experiment. Deeper “see §” harvest happens when you sit in that clause, not as a PDF dump.

**How to look up:** find the ID, then walk **Neighbors**. That is the “don’t miss related guidance” mechanism. Graphify/RAG are query UIs over this file and the PDF; they do not write status.

## How agents use this

When the sitting touches a Blue Book clause (new codec, leftover option, CONFORMANCE row, SAD field map):

1. Find the row in this file (book + `§` / table / PICS item).
2. Walk **Neighbors** — those are the coupled constraints. Open the cited PDF for anything you will code or tick.
3. Code against the PDF. SAD/ICD are working copies.
4. In the **same commit** as the TU: set Status, Claim, Code/test, Neighbors. **Full** needs a test pointer. Do not retcon [`CONFORMANCE.md`](CONFORMANCE.md) from a row here.

Do not ingest `standards/starcom/ccsds/*.pdf` into graphify. This file is the map; the PDF is the authority.

All eight shelf books below are filled at section grain.

PDFs: [`../../standards/starcom/ccsds/`](../../standards/starcom/ccsds/).

---

## 211.2-B-3 — Coding and Synchronization (filled)

Issue 3, October 2019. `CCSDS-211.2-B-3.pdf`. CONFORMANCE claims that live here: PLTU envelope, hunt, coding O.1 (encode Full / decode 0), mixed V-3/V-4 on one stream (forbidden). Repeater is not a 211.2 product (see 133.0 §2.4).

PICS Annex A item 2 (Idle data) is **mandatory** in the book and **0** in the core. CONFORMANCE does not tick “whole C&S sublayer.” Do not read PLTU Full as Idle Full.

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | Fig 1-1 bit numbering (SAD field-map preamble); §1.6 conventions |
| §2 | Overview | unread | — | — | §2.4 C&S send/receive list; 211.1 PHY; 211.0 / 732.1 frame sublayers |
| §3.1 | C&S overview | unread | — | — | §3.2–§3.6 |
| §3.2 | PLTU | Full | PLTU | `pltu.hpp` / `test_pltu.cpp` | Fig 3-1; §3.2.3–§3.2.5; Annex C; §3.5.2 construct; §3.6 delimit |
| §3.2.3 | ASM `FAF320` | Full | PLTU | `kPltuAsm`; `test_reject_bad_asm`; `test_asm_in_crc` | §3.2.5.4 ASM **not** in CRC; §3.6.3 locate PLTU; hunt is exact (book *allows* bit errors; ICD does not) |
| §3.2.4 | Transfer Frame (V-3 xor V-4 per PLTU) | Full | PLTU; USLP in PLTU | `decode_pltu` / `encode_pltu`; `test_reject_tfvn_unknown` | 211.0 §3.2 V-3; 732.1 §4.1 USLP; §3.6.4 TFVN → length; **forbidden** mixed versions on one stream (CONFORMANCE) |
| Mixed V-3 and V-4 on one PLTU stream | (book: one version per stream) | forbidden | Mixed V-3/V-4 | — | §3.2.4; SAD D-4; `decode_pltu` TFVN branch is per candidate, not a mixer |
| §3.2.5 | CRC-32 field | Full | PLTU | `crc32()`; `test_crc_*`; `test_reject_bad_crc` | **Annex C** (procedure); §3.2.3 ASM excluded; §3.6.4 find CRC from TFVN; §3.6.5/§3.6.6 syndrome; 732.1 Annex B FECF is a **different** CRC-16 (all-ones init) |
| §3.3 | Idle data | 0 | — (PICS item 2 **M**, not a CONFORMANCE tick) | no generator in core | PN `352EF853` (§3.3.2.2); Acquisition / Idle / Tail (§3.3.3–§3.3.5) + MIB durations; receive skip: `test_hunt_idle_then_pltu`; 211.0 §6 MAC timers (`acquisition_idle_duration`) are session, not this PN; §3.5.3 insert idle; conv/LDPC encode the stream the **caller** feeds |
| §3.4 | Channel coding | partial | Coding | `conv.hpp` / `ldpc.hpp` / `test_coding.cpp` | PICS O.1 (at least one of uncoded / conv / LDPC); §3.4.2–§3.4.5 exploded below; 211.1 not this book |
| §3.4.2 uncoded (O.1) | Uncoded C&S bitstream | Full | PLTU (uncoded path) | `phy_uncoded_encode` (`adapters/phy.hpp`); `test_conv_pltu_and_uncoded_phy` | §3.2 envelope; `PhyTier::none` / `best_effort`; `compliant` not offered (211.1) |
| §3.4.3 conv (O.1) | Rate 1/2, K=7, non-punctured | partial | Coding: encode Full; decode 0 | `conv_encode`; `test_conv_*` | 131.0-B-5 **§3.3** (not punctured §3.4); G2 inverted; encodes ASM+PLTU+Idle as one stream; §3.4.3.3 soft decisions = decode (0, GCS/Pi) |
| §3.4.4 LDPC (O.1) | (2048,1024) + CSM | partial | Coding: encode Full; decode 0 | `ldpc_encode_*`; `kLdpcCsm`; `test_ldpc_*` | 131.0-B-5 **§7.4**; CSM `034776C7272895B0` not randomized; §3.3 Idle/fill is caller’s job (`ldpc.hpp`); Fig 3-3 |
| §3.4.5 LDPC randomizer | Codeword PN, not CSM | partial | Coding: encode Full; decode 0 | `ldpc_randomize`; `test_ldpc_zero_is_pn` | §3.4.4.4; 131.0-B-5 §8.3 / §10 related but TM-stream; init all-ones per codeword |
| §3.5 | Send-side C&S procedures | partial | PLTU + Coding | `encode_pltu`; conv/LDPC encode | §3.5.2 construct PLTU = Full; §3.5.3 insert idle = 0; §3.5.4 selected encoding = encode only; §3.5.5 deliver to PHY = 0 in core (sans-I/O); §3.5.6 time tag = 0 (PICS item 6 O) |
| §3.5.6 / §3.6.8 | Time tag support | 0 | — (PICS item 6 O) | — | 211.0 §5 Prox-1 timing (unread); not IVP 0–25 |
| §3.6 | Receive-side C&S procedures | partial | PLTU (hunt) | `hunt_pltu` / `decode_pltu`; `test_hunt_*` | §3.6.2 channel **decode** = 0; §3.6.3–§3.6.6 exploded; Annex B indication primitive is not the Starcom API |
| §3.6.3 | ASM hunt | Full | PLTU (IVP 8) | `hunt_pltu`; `test_hunt_*` | §3.2.3; exact `FAF320` (book allows bit errors — we do not); leftover is caller span, no library buffer (ICD) |
| §3.6.4 | TFVN → Frame Length | Full | PLTU | `decode_pltu`; `test_reject_tfvn_unknown`; `test_reject_v3_length_oob` | PICS 7 / 7.1 / 7.2 / 7.3; 211.0 §3.2.2.10 V-3 11-bit length; 732.1 §4.1.2.7 USLP 16-bit; 732.1 Table 5-3 / annex D truncated MIB; SAD “How C&S finds the CRC” |
| §3.6.5 | CRC-32 check | Full | PLTU | `decode_pltu` / `crc32`; `test_reject_bad_crc` | Annex C decode; §3.2.5 |
| §3.6.6 | Invalid received frames | Full | PLTU | `test_hunt_bad_crc_consumes_unit` | PICS item 8; mark invalid, search after; `tfvn_unknown` / length OOB: skip one octet (ICD) |
| Annex A | PICS proforma | — | CONFORMANCE | this file + CONFORMANCE | A2.2 items 1–8 below; completing the blank Support column is a published tick, not this walk |
| Annex B | Service primitives (normative) | 0 | — | sans-I/O `encode`/`decode`/`hunt` instead | ChannelAccess.request / .indication; Quality Indicator; not a Starcom type |
| Annex C | CRC-32 procedures (normative) | Full | PLTU | `crc.hpp` `crc32()`; `test_crc_v3_header_only` golden `BCC004E7` | §3.2.5; G(X)=X³²+X²³+X²¹+X¹¹+X²+1 (`0x00A00805`); init **all-zero**; ASM not covered; **not** ISO-HDLC/Ethernet CRC-32; **not** 732.1 FECF CRC-16 |
| Annex D–F | Security / informative refs / acronyms | unread | — | — | — |

### 211.2 PICS A2.2 (exploded)

O.1 = support at least one of items 3–5.

| Item | Feature | Book | Status here | CONFORMANCE |
|------|---------|------|-------------|-------------|
| 1 | PLTU structure ASM + frame + CRC | §3.2 M | Full | PLTU Full |
| 2 | Idle data | §3.3 M | **0** (core does not generate) | no tick — gap vs a whole-C&S PICS |
| 3 | Uncoded | §3.4 O.1 | Full (encode/decode envelope) | PLTU uncoded path |
| 4 | Convolutional | §3.4.3 O.1 | encode Full; decode 0 | Coding |
| 5 | LDPC | §3.4.4–§3.4.5 O.1 | encode Full; decode 0 | Coding |
| 6 | Time tag | §3.5.6, §3.6.8 O | 0 | no tick |
| 7 | TFVN length | §3.6.4 M | Full | PLTU |
| 7.1 | V-3 Frame Length | 211.0 §3.2.2.10 M | Full | Version-3 |
| 7.2 | USLP Frame Length | 732.1 §4.1.2.7 M | Full | USLP in PLTU |
| 7.3 | USLP truncated length | 732.1 Table 5-3 M | Full (MIB supplied) | USLP remainder IVP 9 |
| 8 | Invalid received frames | §3.6.6 M | Full | PLTU hunt |

---

## 211.0-B-6 — Data Link Layer (filled)

Issue 6, July 2020. `CCSDS-211.0-B-6.pdf`. CONFORMANCE ticks: V-3 Full; DFC `11` Full; PLCW codec Full; COP-P Full; §6 MAC Full. Annex F Odyssey bitstream is **not** the library default.

Do not read those ticks as a whole-book PICS. Mandatory items still **0** in the core are listed after the walk (timing, segment reassembly, most Type-1 SPDUs).

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | Fig 1-1 bit numbering (same family as 211.2 §1.6) |
| §2 | Overview | unread | — | — | §2.2.2.3 User Defined Data (service); layered model → 211.1 / 211.2 |
| §3.1 | PDU overview | unread | — | — | §3.2 V-3; §3.3 V-4 |
| §3.2 | Version-3 Transfer Frame | Full | Version-3 | `v3.hpp` / `test_v3.cpp` | Fig 3-2 / 3-3; 211.2 §3.6.4 uses §3.2.2.10 length; SAD field map (working copy) |
| §3.2.2 | V-3 header (10 fields) | Full | Version-3 | `decode_v3` / `encode_v3`; `test_roundtrip_populated` | TFVN `10`; QoS; PDU Type; DFC; SCID; PCID; Port ID; Src/Dst; Frame Length; FSN. PICS DLL-1–DLL-12 |
| §3.2.3.2 DFC `00` | PACKETS in a U-frame | Full | Space Packet as SDU | `copp_submit_sdu`; `test_v3_one_sp_n` | 133.0 §4.1; PICS DLL-14 O.1 (at least one DFC) |
| §3.2.3.3 DFC `01` | SEGMENT DATA UNITS | 0 | — (PICS DLL-15 O.1) | — | §8.3 / DLL-42 assemble segments also 0; no reassembly |
| §3.2.3.4 DFC `10` | Reserved | 0 | — (PICS DLL-16 O.1) | `test_reserved_dfc_not_a_service` | Table 3-1 reserved; not a service |
| §3.2.3.5 DFC `11` | USER-DEFINED DATA | Full | User Defined Data | `encode_v3_user_defined` / `copp_submit_user_defined`; `test_user_defined.cpp` | §2.2.2.3 opaque, no reassembly; **not** Annex F bitstream; USLP has no DFC (ICD) |
| §3.2.3.6 | P-frame data field | Full | PLCW + SET V(R) | PLCW on Expedited P-frame; SET V(R) Type-1 SPDU | §3.2.4; Annex B1.5; Port ID / DFC forced `00` on P-frame (`encode_v3`) |
| §3.2.4 | SPDU | partial | PLCW | `plcw.hpp` / `test_ocf.cpp` | Fixed-length PLCW Full; variable-length Type-1 only SET V(R) Full; Type 2/3 0 |
| §3.2.4.3.2 | PLCW 16-bit | Full | PLCW | `encode_plcw` / `decode_plcw` | Fig 3-5; **not** CLCW; **not** an OCF (Prox-1 has no OCF); Expedited supervisory |
| §3.3 | Version-4 Transfer Frame | Full | USLP in PLTU | `uslp.hpp` / `test_uslp.cpp` | 732.1 is the V-4 spec; 211.0 §3.3 points there; never nest V-4 in V-3 data (D-4 / 211.2 §3.2.4) |
| §4.1 | Frame sublayer | partial | Version-3 / USLP / COP-P | codecs + `CoppEndpoint` | DLL-25/26; output selection is MAC table 6-14 (`mac_fifo_source`) |
| §4.2 | MAC sublayer (control) | Full | Prox-1 session / MAC | `mac.hpp` / `test_mac.cpp` | §6 is the state tables; 4.2 is the mechanisms; persistence DLL-27 |
| §4.3 | Data services sublayer | partial | COP-P | `copp_submit_sdu` Seq vs Exp | Two QoS (DLL-36) yes; order/queues are host-loop caps not MIB |
| §4.4 | I/O sublayer | 0 | — | no I/O sublayer object | Sans-I/O: caller is the I/O. DLL-42/43 packet reassembly **0**. `copp_take_sdu` is 7.3.3, not §8 |
| §5 | Timing services | 0 | — (PICS DLL-52–64 many **M**) | — | 211.2 §3.5.6 / §3.6.8 time tag also 0; Annex B2 Time Distribution SPDU 0; not IVP 0–25 |
| §6 | Data services operations (session / MAC / hailing) | Full | Prox-1 session / MAC | `MacSession`; `test_full_hail_tables`; `test_half_and_simplex` | Tables 6-2–6-13; table 6-14 FIFO; 6.5 PHY *view* (`mac_phy`) is flags, not 211.1; SET V(R) 7.2.3.2 lives here |
| §6.4.2 | Full duplex | Full | Prox-1 session / MAC | `test_full_hail_tables` | PICS DLL-100 **M** |
| §6.4.3 | Half duplex | Full | Prox-1 session / MAC | `test_half_and_simplex` | PICS DLL-101 **O**; CONFORMANCE ticked the full module |
| §6.4.4 | Simplex | Full | Prox-1 session / MAC | `test_half_and_simplex`; `test_simplex_no_hail` | PICS DLL-102 **O**; no hailing on simplex (211.2 / glossary) |
| §6.5 | Interface to PHY (via C&S) | 0 as 211.1 | — | `mac_phy()` returns TRANSMIT/MODULATION bits | 211.1 Full not offered; caller drives radio from the bits |
| §6.7 | Receiving operations / frame validation | partial | Prox-1 session / MAC | `mac_on_valid_frame`; COP-P `copp_receive_bytes` | Test_Source / SCID (Annex C); 211.2 §3.6.6 invalid C&S is separate |
| §7 | COP-P | Full | COP-P | `copp.hpp` / `test_copp.cpp` | FARM-P RE0–RE6 + RE7 report; FOP-P SE0–SE4/SE7; modulo-256 `seq_lt`; 232.1 uses the same compare |
| §7.2.3.2 | SET V(R) persistent | Full | Prox-1 session / MAC | `mac_drive_set_vr`; `test_set_vr_persistent` | Annex B1.5 codec; inbound RE2; not a FOP-P table row — MAC activity |
| §7.3.3 | Interface to I/O (`take_sdu`) | Full | COP-P | `copp_take_sdu` | Named I/O in the book; Starcom verb is sans-I/O |
| §8 | I/O sublayer operations | 0 | — | — | §4.4; segment reassembly 0; queues in `CoppEndpoint` are host-loop caps (`kCoppHold`) |
| Annex A | PICS proforma | — | CONFORMANCE | this file | Completing Support is a published tick. M-not-in-core listed below |
| Annex B | Variable-length SPDU formats | partial | SET V(R) | `encode_set_vr` / `decode_set_vr`; `test_set_vr_codec` | **B1.5 SET V(R) Full.** B1.2–B1.4 / B1.7 SET TRANSMITTER/RECEIVER/CONTROL/PL_EXTENSIONS **0** (PICS M). B1.6 / B1.8 O = 0. Type 2 time / Type 3 status = 0 |
| Annex C | MIB (normative) | partial | — | `CoppMib` / `MacMib` / `UslpMib` caller-owned | Names only; **no library default milliseconds**. 211.2 Idle durations are C&S PN, not these timers |
| Annex D | Notifications to vehicle controller | partial | Prox-1 session / MAC | `MacNotify` | Mapped subset (`hail_fail`, `end_session`, …). Not a complete Annex D PICS |
| Annex E, H, I, J | informative | unread | — | — | — |
| Annex F | Odyssey 2001 (informative) | out of scope | — (explicit non-default) | `test_user_defined` “not Annex F” | Do not copy Retransmit Flag always 0 / unused Expedited counter |
| Annex G | MRO 2005 (informative) | unread | — | — | Mission profile, not a codec |

### 211.0 PICS M not in the core (gaps vs a whole-book tick)

| Item | Feature | Why 0 |
|------|---------|--------|
| DLL-15 / 42 / 43 | Segment DFC + assemble/verify packets | No reassembly (DFC 11 is opaque) |
| DLL-21 except B1.5 | Variable-length Type-1 other than SET V(R) | No SET TRANSMITTER/RECEIVER/CONTROL/PL_EXTENSIONS codec |
| DLL-31 / 32 / 52–64 | Time buffers + §5 timing | Not IVP 0–25; 211.2 time tag also 0 |
| DLL-37–51, 141–142 | I/O sublayer as an object | Sans-I/O; caller owns queues |

---

## 232.1-B-2 — COP-1 (filled)

Issue 2, Sep 2010 + TC1 Apr 2019. `CCSDS-232.1-B-2.pdf`. No PICS annex — the state tables *are* the requirement.

CONFORMANCE: **Full (implemented options)**. FARM-1 Table 6-1 Full. FOP-1 E23 + S4/S5 (E24/E25/E27) + E29 Full. **Do not collapse to “§5 Full.”** Wire is **USLP + CLCW-in-OCF**, not 232.0 TC frames (ICD).

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | — |
| §2 | Overview | unread | — | — | AD / BD / BC service types; 232.0 CLCW; 732.1 OCF |
| §3 | Inter-procedure interfaces | partial | COP-1 | `Cop1Endpoint` verbs | Directives §4; LLIF Accept/Reject are E41–E46 (**0**) |
| §4 | Detailed service definition | partial | COP-1 | initiate / terminate APIs | E23–E29 subset; Resume E30–E34 **0**; setup E35–E40 **0** |
| §5 | FOP-1 | partial | COP-1 | `fop_1_*` / `test_fop_1_*` | Table 5-1 exploded below. States S1–S6 exist in `Fop1State` |
| §5.1 / §5.2 | FOP-1 variables / actions | partial | COP-1 | `Fop1` / `Cop1Mib` | K, T1_Initial, Transmission_Limit, Timeout_Type; sent-queue cap `kFop1SentCap` is host-loop, not MIB |
| §6 | FARM-1 | Full | COP-1 | `farm_1_*`; `test_farm_1_table` | Table 6-1 E1–E11. States Open/Wait/Lockout |
| §6.1 / §6.2 | FARM-1 variables / actions | Full | COP-1 | `Farm1` / `Farm1Mib` | W even, 2–254; PW = NW = W/2 |
| §7 | Managed parameters | partial | COP-1 | `Cop1Mib` / `Farm1Mib` | Table 7-1 Timeout_Type 0/1 present; suspend-related params unused while E30–E34 are 0 |
| Annex A–D | informative | unread | — | — | Annex C is table-format notes |

### 232.1 Table 5-1 / 6-1 (exploded)

| ID | Feature | Status | Code / test | Neighbors |
|----|---------|--------|-------------|-----------|
| Table 6-1 E1–E11 | FARM-1 | Full | `test_farm_1_table` | CLCW flags Lockout/Wait/Retransmit; `farm_1_report` |
| Table 5-1 E1–E22, E101–E104 | FOP-1 CLCW / timeout / wait-queue | partial | `fop_1_on_clcw`; `fop_1_tick`; `test_fop_1_retransmit_flag` | Not listed as a CONFORMANCE leftover; not claimed as a complete main-protocol tick |
| E23 | Initiate AD (no CLCW check) | Full | `cop1_initiate_ad`; `test_fop_1_initiate_and_ack` | → S1 |
| E24 | Initiate AD with CLCW check | Full | `cop1_initiate_ad_with_clcw_check`; `test_fop_1_s4_clcw_check` | → S4 |
| E25 | Initiate AD + Unlock BC | Full | `cop1_initiate_ad_unlock`; `test_fop_1_s5_unlock_and_terminate` | → S5; BC `00` (232.0 §4.1.3.3) |
| E27 | Initiate AD + Set V(R) BC | Full | `cop1_initiate_ad_set_vr`; `test_fop_1_s5_set_vr` | → S5; not the Prox-1 Annex B1.5 SPDU |
| E29 | Terminate AD | Full | `cop1_terminate_ad` | → S6 |
| E26, E28 | other Initiate variants | 0 | — | Table 5-1 still has these rows |
| E30–E34 | Resume AD (by Suspend_State) | 0 | — | CONFORMANCE leftover; SS in the book ≠ MAC `ss` |
| E35–E40 | FOP-1 setup directives | 0 | — | Window/timer knobs exist on `Cop1Mib` as fields, not as E35–E40 |
| E41–E46 | LLIF AD/BC/BD Accept/Reject | 0 | — | CONFORMANCE leftover; Alert [LLIF] |
| Host loop | USLP + OCF CLCW | Full as Starcom wire | `test_host_loop`; `test_host_loop_unlock` | 732.1 OCF; 232.0 §4.2.1 CLCW; **not** 232.0 TC frame |

---

## 232.0-B-4 — TC Space Data Link Protocol (filled)

Issue 4 + TC1. `CCSDS-232.0-B-4.pdf`. CONFORMANCE: CLCW codec Full. COP-1 *procedures* are 232.1. Starcom COP-1 **does not** implement the TC Transfer Frame (ICD: USLP in a PLTU).

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | — |
| §2 | Overview | unread | — | — | COP-1 assumed; SDLS optional ch. 6 |
| §3 | Service definition | 0 | — | — | MAP/VC/MC services are TC-frame services; Starcom SDU is Space Packet in USLP/V-3 |
| §4.1 | TC Transfer Frame | 0 | — | no TC-frame codec | 232.1 rides this *or* USLP; we picked USLP. Unlock/Set V(R) *bytes* `00` / `82 00` still from §4.1.3.3 |
| §4.1.3.3 | COP-1 control commands (Unlock / Set V(R)) | Full as bytes | COP-1 | `kCop1Unlock` / `kCop1SetVr*`; `test_fop_1_s5_*` | Distinct from Prox-1 Annex B1.5 SET V(R) SPDU |
| §4.2.1 | CLCW | Full | CLCW | `clcw.hpp` / `test_clcw_cop1` | Fig 4-6; COP in Effect `01`; lives in USLP OCF later; **not** PLCW |
| §4.3 / §4.4 | TC send/receive procedures | 0 | — | — | Framing procedures for the TC frame we do not codec |
| §5 | Managed parameters (no SDLS) | 0 | — | — | VC/MAP/MC; COP-1 MIB is 232.1 §7 |
| §6 | Protocol with SDLS | out of scope | — | — | 355.0; not CFDP |
| Annex A | PICS | — | CONFORMANCE | CLCW tick only | Do not tick TC frame |
| Annex B–C | informative | unread | — | — | — |

---

## 732.1-B-3 — USLP (filled)

Issue 3, June 2024. `CCSDS-732.1-B-3.pdf`. CONFORMANCE: Version-4 in the same PLTU Full (non-truncated + truncated annex D + Insert Zone + FECF Annex B). Never nested in the V-3 data field.

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | — |
| §2 | Overview | unread | — | — | 211.0 §3.3; 211.2 PLTU wrapper; COP-1 per VC |
| §3 | Service definition | 0 as named services | — | PDU codec only | MAP/VC/MC/Insert *services* are not Starcom APIs; Insert *Zone field* is §4/§5 |
| §4.1 | USLP Transfer Frame (no SDLS) | Full | USLP in PLTU | `encode_uslp` / `decode_uslp`; `test_uslp.cpp` | Fig 4-1 / 4-2; TFVN `1100`; 211.2 §3.6.4 length |
| §4.1.5 | OCF | Full as field | CLCW (when present) | `ocf_present`; `test_pointer_rule_and_ocf`; COP-1 host loop | 4 octets; carries CLCW for COP-1; Prox-1 V-3 has **no** OCF |
| §4.1.6 / Annex B | FECF CRC-16 | Full | USLP remainder | `crc16_fecf`; `test_fecf_roundtrip_and_bad` | Init **all-ones**; **not** 211.2 Annex C CRC-32; truncated frame forbids FECF |
| §4.2 / §4.3 | Send/receive procedures | 0 | — | codec only | COP-1/COP-P engines are separate books |
| §5 | Managed parameters (no SDLS) | partial | USLP | `UslpMib` | truncated length; insert zone length; FECF present. No invented depths |
| §6 | Protocol with SDLS | out of scope | — | — | 355.0 |
| Annex A | PICS | — | CONFORMANCE | USLP-in-PLTU tick | — |
| Annex B | FECF coding (normative) | Full | USLP remainder | `crc.hpp` `crc16_fecf` | G(X)=X¹⁶+X¹²+X⁵+1 |
| Annex C | V-3 vs V-4 relationship (normative) | Full as rule | D-4 / mixed-stream forbidden | `test_pltu_composition`; 211.2 §3.2.4 | Same PLTU, never mixed, never nested |
| Annex D | Truncated Transfer Frame | Full | USLP remainder | `test_truncated_roundtrip_and_pltu` | 6–32 octets; MIB length; no insert/FECF (`test_truncated_rejects_insert_or_fecf`) |
| Insert Zone | §4 / §5 / §3.11 | Full as field | USLP remainder | `test_insert_zone` | Length 0 = absent |
| OID VCID 63 | §4.1.2 | 0 as generator | — | `kUslpOidVcid` constant only | Annex H informative |
| Annex E–H | informative | unread | — | — | — |

---

## 133.0-B-2 — Space Packet Protocol (filled)

Issue 2, June 2020 (e2). `CCSDS-133.0-B-2.pdf`. CONFORMANCE: Space Packet as SDU Full. Not a Starcom product name.

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | — |
| §2 | Overview | unread | — | — | Packet vs Octet String |
| §2.4 | Services assumed from lower layers | — | PLTU repeater (related) | `repeat_pltu` / `PltuRepeatQ` | Store-and-forward **assumed**, not an SPP procedure. Starcom repeater is not a 133.0 product |
| §3.3 | Packet service | Full as SDU codec | Space Packet as SDU | `encode_space_packet` / `decode_space_packet` | APID; seq; length-minus-one. cFS/PUS/F´ *contents* stay out of `starcom::ccsds` |
| §3.4 | Octet String service | 0 | — | seq flags default `11` is unsegmented Packet, not this service | Table 5-1 |
| §4.1 | Space Packet PDU | Full | Space Packet as SDU | `test_roundtrip`; `test_sp_idle`; `test_reject_sp_pvn` | Fig 4-1 / 4-2; idle APID `0x7FF`; PVN `000` |
| §4.2 / §4.3 | Send/receive procedures | 0 | — | codec only | No SAP / routing |
| §5 | Managed parameters | 0 as a table | — | `Maximum_Packet_Size` named in SAD MIB working copy; caller supplies | Table 5-1 |
| Annex A | PICS | — | CONFORMANCE | Space Packet tick | — |
| Annex B–D | informative | unread | — | — | — |

---

## 211.1-B-4 — Physical Layer (filled)

Issue 4, Dec 2013. `CCSDS-211.1-B-4.pdf`. CONFORMANCE: blanket Full **not offered**. `PhyTier::compliant` is not implemented. Best-effort bearers (LoRa, PIO, FSK continuous) are **not this book**.

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | — |
| §2 | Overview | unread | — | — | C&S is 211.2; MAC is 211.0 §6.5 |
| §3.1 | Radio equipment | 0 | 211.1 = 0 | — | — |
| §3.2 | Physical layer functions | 0 | 211.1 = 0 | `mac_phy()` is 211.0 flags | Carrier / symbol-inlock are caller inputs to MAC |
| §3.3 | Controlled channel properties | out of scope as Full | 211.1 = 0 | — | Residual-carrier Bi-Phase-L PM, 60°±5% — not RFM95 FSK/LoRa |
| §3.4 | Performance requirements | 0 | 211.1 = 0 | — | ppm / residual AM |
| Adapter tiers | (Starcom, not a 211.1 clause) | Best effort / 0 | PHY tiers | `PhyDecl`; `test_tiers`; `test_compliant_not_offered` | `none` / `best_effort` uncoded PLTU; `compliant` rejected |
| Annex A | PICS | — | CONFORMANCE | no tick | Do not complete Support as Y |
| Annex B–D | informative | unread | — | — | — |

---

## 131.0-B-5 — TM Synchronization and Channel Coding (filled)

Issue 5, Sep 2023. `CCSDS-131.0-B-5.pdf`. CONFORMANCE: long-haul TM C&S **0** (different sublayer than PLTU). 211.2 **cites** two clauses; those are walked as 211.2 coding, not as a 131.0 product.

| ID | Title | Status | Claim | Code / test | Neighbors |
|----|-------|--------|-------|-------------|-----------|
| §1 | Introduction | unread | — | — | — |
| §2 | Overview | unread | — | — | TM ASM / FECF path ≠ Prox-1 PLTU |
| §3.3 | Basic convolutional code | Full as 211.2 cite | Coding encode | `conv_encode`; `test_conv_*` | 211.2 §3.4.3 → **this** clause. Punctured §3.4 = **0** |
| §3.4 | Punctured convolutional | 0 | — | — | Not 211.2 O.1 |
| §4 | Reed-Solomon | 0 | long-haul TM = 0 | — | — |
| §5 | Concatenated | 0 | long-haul TM = 0 | — | — |
| §6 | Turbo | 0 | long-haul TM = 0 | — | — |
| §7.3 | LDPC rate 223/255 | 0 | — | — | Not the Prox-1 (2048,1024) |
| §7.4 | LDPC rates 1/2, 2/3, 4/5 | Full as 211.2 cite (1/2 only) | Coding encode | `ldpc_encode_*`; `test_ldpc_*` | 211.2 §3.4.4 → (n=2048, k=1024). 2/3 and 4/5 = **0**. Decode = **0** |
| §8 | LDPC of SMTF stream | 0 | long-haul TM = 0 | — | 211.2 has its own CSM + randomizer |
| §9 | TM frame synchronization (ASM) | 0 | long-haul TM = 0 | — | **Not** Prox-1 `FAF320` (211.2 §3.2.3) |
| §10 | TM pseudo-randomizer | 0 | long-haul TM = 0 | — | 211.2 §3.4.5 is the Prox LDPC randomizer |
| §11 | Transfer frame lengths | 0 | long-haul TM = 0 | — | Prox length is 211.0 / 732.1 / 211.2 §3.6.4 |
| §12 | Managed parameters | 0 | — | — | — |
| §13 | Ground-to-space / space-to-space | unread | — | — | — |
| Annex A–G | | unread | — | — | Annex A service is TM, not PLTU |

---

## Not on the shelf (named elsewhere)

| Book | Why it appears | Status |
|------|----------------|--------|
| 727.0 CFDP | CONFORMANCE deferred; wanted post-mission offload | out of scope for IVP 0–25 |
| 355.0 SDLS | Named to keep it distinct from CFDP and from 232.0/732.1 ch. 6 | out of scope |
| ECSS-E-ST-50-12C SpaceWire | URL-only; gated | not in this walk |
