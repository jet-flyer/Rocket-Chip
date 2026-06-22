# Starcom — Canonical Design Record (Condensed)

**Condensed 2026-06-22 on feat/condense-starcom-ccsds-prelim-20260622.**  
This is the single canonical record consolidating:
- `research/ccsds_domain_claude.md` (Claude CCSDS domain research)
- `research/ccsds_domain_grok.md` (Grok CCSDS domain research)
- `research/library_craft_claude.md` (Claude library-craft research)
- `research/library_craft_grok.md` (Grok library-craft research)
- `comparison.md` (Claude Entry 1 + Grok Entry 2 + Grok Council Review)
- `design_record_claude.md` (Claude scope + council rounds + standing decisions)

**No substantive data loss rule:** Every load-bearing fact, table, enumerated list, decision (D-1..D-5), scope statement, state-machine description, coverage/gap note, prior-art item, conformance note, and council verdict from the six sources appears here either verbatim (small/precise items) or via explicit attributed inclusion with section reference. Historical research docs remain untouched as primary sources.

**Governing scope:** `design_record_claude.md` §0 (lifted below) + refinements in Round 3.

**Sources remain historical.** Append-only spirit preserved by leaving `comparison.md`/`design_record_claude.md` as-is; this DESIGN synthesizes without rewriting them.

---

## §0 Canonical Scope & Audience (verbatim from design_record_claude.md §0, governing)

**Starcom is a standalone, universal CCSDS data-link library.** It is built for *any* consumer that needs CCSDS-conformant (or CCSDS-derived) telemetry and command links — cubesats, ground stations, high-altitude balloons, research platforms, drones, and high-power rocketry. **Rocket-Chip (RC) is the first consumer and the integration driver — it is not the owner, and it is not the boundary of the design.**

**What Starcom is, and why now (settled 2026-06-19):** *What* it is never changes — a **universal, CCSDS-compliant telemetry/command library.** The *why* and *when* are RC-specific: RC's **radio telemetry is currently broken** (improper half-duplex handling; MAVLink only works over direct serial), which is a **flight blocker**, and prior ad-hoc band-aids (a STOP-GAP retry/ACK layer + RadioScheduler TX-window hacks) have cost more time than a principled implementation would have. The universal goal and the RC blocker are *not* in tension: the blocker is *why we build it now*; the universal library is *what we build*. And doing it right is not a luxury — because band-aids mask the failure chain, a clean principled foundation is the **diagnostic instrument** that lets the real bug (and the next one behind it) finally surface.

**Two distinct goalposts — keep them separate:**
- **Library end goal = the CCSDS Blue Books.** Full standards compliance. This destination is *precisely defined* by the standards and reached *incrementally* — it is not fuzzy.
- **MVP goalpost = empirical.** "Minimum functionality — the telemetry link genuinely works, verified." Its exact location is *unknown until crossed*, because masked cascading issues only surface once the layer beneath them is genuinely fixed. Do not declare victory before crossing; the MVP "fires" when the real link works on the bench, not when a predefined feature checklist is ticked.

Consequences that bind every decision:
1. RC is one user among many. ...
2. The strictest plausible adopter sets the floor. ... core is exceptionless, no-RTTI, no-heap-after-init **by construction**.
3. The ceiling is full CCSDS compliance.
4. Honesty about conformance is a hard requirement... declared **per component**.
5. RC appears as an example integration only.
6. Pedagogical intent is foundational.

(See full §0 in source `design_record_claude.md` for complete numbered consequences and MISRA C++:2023 compatibility note: stay *compatible-with*, do not adopt now.)

---

## 1. Agreement / Conflict / Gap Analysis Table

**Source baseline:** comparison.md Entries 1.A/1.B/1.C/1.D/1.F/2.A/2.B + design_record_claude.md §2.4/§3 + research docs.

This table categorizes content. **Attribution** per original (Claude comparison 2026-06-01, Grok verification, Grok council 2026-06).

| Topic / Item | Agreement (both / all) | Direct Conflict (pre-adjudication) | Major Gap — Claude only (or stronger) | Major Gap — Grok only (or stronger) | Resolution / Notes (from comparison + councils) |
|--------------|------------------------|------------------------------------|---------------------------------------|-------------------------------------|-------------------------------------------------|
| **Blue Book ownership splits** (211.1/211.2/211.0 + 232/732) | Identical: PHY 211.1-B-4, C&S 211.2-B-3, Data Link/COP-P 211.0-B-6; COP-1 232.1-B-2 + 232.0-B-4 for CLCW format; USLP 732.1-B-3 | None substantive | Full feature-ownership map + clause refs (e.g. PLCW in 211.0-B-6 §3.2.4.3.2.1.1) | N/A (Grok corrected to match) | Settled per comparison 1.A + 1.E. Use Claude map. |
| **PHY waveform (211.1-B-4)**: residual-carrier Bi-Phase-L/PM 60°±5%, etc. | Both: exact requirements listed; SX1276 cannot natively emit | None (Grok §2.3 optimism superseded internally by §2.5) | Detailed COTS/SDR/FPGA survey + prices (Pluto ~$186-230, ComBlock ~$2500); JPL Electra as proof | RP2350 PIO concrete prior-art examples (pico manchester_encoding/differential_manchester, 433MHz OOK decoders, ADS-B dual-PIO preamble+Manchester as ASM analog, 100BASE-TX) + 6 PIO opportunities enum | Adjudicated: NO full 211.1 claim. Best-effort only on current HW. D-1 settled → Claude doc. |
| **SX1276 / AX5043 feasibility** | Both converge: no native compliant PHY waveform | Grok initially called AX5043 "closer" (category error) | Full radio survey + viability table | N/A (corrected) | AX5043 is suppressed-carrier BPSK, not residual PM. Fixed in post-5b33e39. Do not roadmap as "closer". |
| **COP state machines (FOP-1 / FARM-1 / FOP-P / FARM-P)** | Both: FOP-1 6 flat states, FARM-1 3 states; COP-P simpler; table-driven impl; prior art (OSDLP, cFS cop1.c, yamcs) use no framework | None | Full enumerated tables (FOP-1 S1–S6 with 46 events per 232.1-B-2 Table 5-1; FARM-1 Open/Wait/Lockout Table 6-1); FARM-P "stateless/data-driven" | N/A (Grok conceptual only) | Design: plain passive table-driven C++ (switch / array of transitions). No QP in core. See design_record §2.7 table. |
| **PLCW vs CLCW / OCF** | Both recognize distinct return-link reporting | Grok imprecise on OCF/PLCW | **Exact 16-bit 7-field SPDU layout** (211.0-B-6 A3.2.4.3.2.1.1): Report Value (8b), Expedited Frame Ctr (3b), Reserved (1b), PCID (1b), Retransmit (1b), SPDU Type (1b `0`), SPDU Format (1b `1`). **Proximity-1 has NO OCF.** PLCW sent Expedited on supervisory channel, not in frame trailer. | N/A | Model Clcw32 + Plcw16 as **distinct types** + separate FARM-1 / FARM-P paths. No generic OCF hosting both. Claude §6.3/6.4 governs. |
| **USLP vs V-3/PLTU framing** | Both: USLP forward-looking unifier carrying COP-1+COP-P per VC; PLTU = ASM(0xFAF320)+V3+CRC-32; V-3 permanent for native Prox-1 | D-4 fork: Grok early "V-3 primary"; Claude "USLP spine" | Full USLP Primary Header bit table + multiplexing hierarchy + FECF(CRC-16) vs PLTU(CRC-32) decision + supersession note (do not cite B-2) | N/A | USLP strategic unifying spine (Round 3). V-3/PLTU **permanent peer codec** + likely MVP-first (smallest vertical). "USLP-primary (arch)" + "V-3-first (MVP seq)" coexist. |
| **sans-I/O core** | Both: bytes/timeouts in, events/bytes out; passive; high testability | Grok placed IPhysicalLayer seam *in/under* core | **sans-I/O as THE foundation** (receive_bytes/bytes_to_send/poll_event/handle_timeout/submit_sdu); cites sans-io.readthedocs + h11 etc. | State in caller-owned static structs; high-level semantic events (CommandAccepted etc) | D-2 settled → Claude: core strictly I/O-free. Adapters outside. Policy/templates ok; no virtual ABC in core (P10 + no-RTTI). |
| **Library form (header-only vs static)** | Both ship both eventually; MCU constraint binding | Grok favored header-mostly (LTO claim later retracted) | Static default for state-machine shape + measurement spike recommended | LTO is -flto flag (not packaging); CCSDSPack static-on-MCU precedent | D-3: static primary; header INTERFACE opt-in. Run compile-both spike Phase 0. Lean header-mostly per aerospace norm unless data says otherwise. |
| **Error handling** | `std::expected` / `tl::expected` (or backport); no exceptions in core; small typed errors | None | `tl::expected` default + `STARCOM_USE_STD_EXPECTED` knob for zero-dep Result | Error objects trivially copyable, ≤16-32 bytes, noexcept | Adopt Grok size bound + Claude knob. |
| **CI / packaging / no-heap** | Host-heavy, ASan/UBSan, fuzz (esp FARM-1), arm cross, GoogleTest | None (Grok additive) | 7-phase plan; GoogleTest+FuzzTest; install/export + FetchContent | Malloc-after-init **hard gate** shim (positive control); flash/RAM/stack report per config (delta vs baseline); vcpkg triplet checked-in; F' non-viable | D-5 superset: hard no-heap gate + published size report (Phase 0). vcpkg portfile now but registry post-1.0. |
| **Conformance / honesty** | Both: declare honestly; no blanket PHY claim | None | **Tested per-component subsystem**: descriptors → published matrix + capability query; over-claim fails CI. "Best-effort spectrum" explicit per-service | N/A | Elevated to first-class tested subsystem (Round 2). Defer full impl to post-MVP unblock but principle from day 1. |
| **MIB / managed params** | Configurable (T1, windows, etc) | None | MIB surface is **versioned public API** (SemVer-bound) | N/A | Elevate from internal to interface design task. |
| **<50 km / LunaNet / in-field impact** | N/A (Claude high-level only) | N/A | N/A | **Detailed <50 km assessment** (link margins >> Mars; limitation felt in post-landing wake-up, edge-of-coverage descent, future ranging). LunaNet v5 + SSTL concrete numbers (as profile, not normative). | Include as profile-tier annotation only (two-tier source: CCSDS normative, LunaNet profile). |
| **Prior art & concrete enablement** | OSDLP, cFS cop1.c, CCSDSPack, yamcs, RadioLib PhysicalLayer precedent | N/A | Framework-agnostic seam guidance; Hourglass ABI appendix (deferred) | Concrete fetched CCSDSPack CMake, CMakePresets, FILE_SET; broader prior-art (OpenCyphal/CETL, EmbeddedSDLP, Glaze); RP2350 PIO examples | Merge both. Grok shipping/ops + prior-art; Claude arch + pedagogy. |
| **AO / framework** | AO wrapper = thin optional consumer adapter only; core no QP dep | Grok had I/O seam that pulled framework concerns in | **Core = portable plain table-driven HSM object** (standalone usable); AO = optional adapter like PHY tiers. Blue Books prescribe tables, not code structure | N/A | design_record §2.7 + Round 2/3. "Architecturally complete + feature-incremental". |
| **Standards currency / other** | All other Blue Books current | Grok had stale 131.0-B-3 / Rev.4 SX1276 / 232.1 for CLCW bits | Full FCC Part 15.247/15.249 + Part 97 amateur encryption prohibition detail | SX1276 Rev.7 (May 2020) correction; 6 PIO opportunities | Fix stale refs before code (131.0-B-5 check vs 211.2 normative ref). Use Claude FCC/Part97. |
| **Pedagogy + governance** | Docs first-class; explain why | N/A | Pedagogical 5-part shape; 4-persona council; Keep-a-Changelog + SemVer models; CONTRIBUTING + DCO | RC `CODING_STANDARDS.md` line-item alignment checklist | Adopt both. Starcom identity standalone. |

**Summary from comparison (1.A/2.A settled; 1.B/2.B adjudicated):**
- Load-bearing CCSDS facts: converged or corrected; Claude doc = stronger spine.
- Library arch: Claude spine for sans-I/O / core purity; Grok additive for shipping mechanics + prior art.
- 5 decisions (D-1..D-5) logged in comparison §3; refined in design_record §2.4/§3.

**Coverage gaps explicitly preserved (no loss):** See comparison 1.F and 2.B coverage sections + design_record §2.6 audit of docs against §0. All absorbed into this record or noted as historical.

---

## 2. Standing Decisions & Open Items (consolidated from design_record §2.4 + §3 + comparison D-1..D-5, Grok council)

**Build philosophy:** architecturally complete + feature-incremental. Universal foundation from day one; features sequenced blocker-first. MVP first cut ≈ V-3/PLTU + COP-P + minimal half-duplex turnaround (replace band-aids). Half-duplex turnaround verified independently.

**Core:** sans-I/O, passive (submit/on_clcw/tick(now)), exceptionless/no-RTTI/no-heap-after-init. Plain table-driven C++ state machines (FOP-1 6 states per 232.1-B-2 Table 5-1 etc.). No framework in core. AO optional adapter only.

**PHY:** three first-class (none / best-effort 1–99% spectrum / compliant) behind one profile-parameterized seam. Neutral naming (IRadio/ILink per councils; Grok suggests ILink). Per-adapter conformance declarations.

**Framing:** USLP strategic spine (carries COP-1+COP-P). V-3/PLTU permanent peer + MVP-first sequencing.

**Four axes:** USLP framing · COP-1/COP-P ARQ (per VC) · Prox-1 §6 session/MAC (minimal turnaround pulled forward; full deferred) · PHY tier.

**Form/CI:** Static + header-only (default via spike). tl::expected + knob. Hard no-heap shim gate + published size report. Apache-2.0. SemVer.

**Other:** MIB versioned public API. Standalone identity (RC = example). Pedagogical. Compatible-with MISRA C++:2023 etc. Update 131.0 references.

**Open (pre Phase 0):**
- Precise MVP cut + turnaround ownership (RadioScheduler vs new minimal MAC).
- CCSDS source-version pinning (exact issues).
- Header-vs-static default (data from spike).
- Whether core includes full ~30-state Prox §6 MAC (or deferred).
- Generator test telemetry as Phase-0 deliverable.

**Grok council (2026-06, appended to comparison):** Reinforces universal scope, neutral ILink seam, three PHY tiers as optional labeled adapters outside "Starcom Core". Naming: "Starcom Core" + ILink preferred in Grok panel.

---

## 3. Unique Data by Source (explicit preservation of gaps)

**Claude-only / stronger (ccsds_domain_claude + library_craft_claude + design_record):**
- Detailed PLCW 16-bit 7-field layout + "NO OCF" correction + verification notes.
- USLP Primary Header full bit table + history.
- Full feature-to-Blue-Book ownership map.
- FCC Part 15/97 encryption prohibition + channel tables (UHF 435-450 etc.).
- 4-persona council outputs (Round 2/3) + "architecturally complete + feature-incremental".
- Sans-I/O as named foundation with API surface; 7-phase plan outline.
- "function-generated CCSDS test telemetry" idea.
- Exact state machine table (FOP-1 6 / FARM-1 3) + prior-art survey proving no-framework implementations.

**Grok-only / stronger (ccsds_domain_grok + library_craft_grok):**
- RP2350 PIO prior-art catalog + 6 specific opportunities (manchester examples, ADS-B dual-PIO as ASM analog, etc.).
- <50 km practical impact assessment (post-landing emphasis).
- LunaNet v5 + SSTL profile numbers (as secondary).
- Concrete fetched CMake patterns, CCSDSPack excerpts, CMakePresets, vcpkg triplet sketch.
- F' ruling (not viable as plugin).
- Broader prior-art roster + RC standards alignment checklist.
- SX1276 Rev.7 + PIO continuous-mode enablement details.

**Cross-verified (comparison):** All factual corrections recorded; no data dropped. D-1..D-5 + Grok council section preserved.

---

## 4. Migration / Usage Notes

- Research + comparison + design_record files = historical (append only; do not edit).
- This DESIGN.md is now the condensation target and prep for dev plan (per goal: "in preparation for the dev plan for the library itself (not building that yet)").
- Next (per STATUS): Phase 0 CMake skeleton, standards ref update (131.0-B-5), precise MVP scoping.

**Verification of no loss (to be run):** Grep for 10+ unique tokens from each source (e.g. "16-bit fixed-length SPDU", "ADS-B (1090 MHz)", "FOP-1.*6.*states", "architecturally complete + feature-incremental", "D-5", "tl::expected", "no-heap-after-init hard gate", "LunaNet", "Part 97", "Table 5-1").

All modifications only on this branch; main untouched for starcom/.

---

*Condensation complete per plan. Sources: starcom/docs/* (moved 2026-06-18 from docs/research/).*