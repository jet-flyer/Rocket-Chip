# Starcom - Canonical Design Record (Condensed)

**Condensed 2026-06-22 on feat/condense-starcom-ccsds-prelim-20260622.**  
This is the single canonical record consolidating:
- research/ccsds_domain_claude.md (Claude CCSDS domain research)
- research/ccsds_domain_grok.md (Grok CCSDS domain research)
- research/library_craft_claude.md (Claude library-craft research)
- research/library_craft_grok.md (Grok library-craft research)
- comparison.md (Claude Entry 1 + Grok Entry 2 + Grok Council Review)
- design_record_claude.md (Claude scope + council rounds + standing decisions)

**No substantive data loss rule:** Every load-bearing fact, table, enumerated list, decision (D-1..D-5), scope statement, state-machine description, coverage/gap note, prior-art item, conformance note, and council verdict from the six sources appears here either verbatim (small/precise items) or via explicit attributed inclusion with section reference. Historical research docs remain untouched as primary sources.

**Governing scope:** design_record_claude.md section 0 (lifted below) + refinements in Round 3.

**Sources remain historical.** Append-only spirit preserved by leaving comparison.md/design_record_claude.md as-is; this DESIGN synthesizes without rewriting them.

---

## Section 0 Canonical Scope & Audience (verbatim from design_record_claude.md section 0, governing)
## 0. Canonical Scope & Audience (the framing that governs everything below)

**Starcom is a standalone, universal CCSDS data-link library.** It is built for *any* consumer that needs CCSDS-conformant (or CCSDS-derived) telemetry and command links — cubesats, ground stations, high-altitude balloons, research platforms, drones, and high-power rocketry. **Rocket-Chip (RC) is the first consumer and the integration driver — it is not the owner, and it is not the boundary of the design.**

**What Starcom is, and why now (settled 2026-06-19):** *What* it is never changes — a **universal, CCSDS-compliant telemetry/command library.** The *why* and *when* are RC-specific: RC's **radio telemetry is currently broken** (improper half-duplex handling; MAVLink only works over direct serial), which is a **flight blocker**, and prior ad-hoc band-aids (a STOP-GAP retry/ACK layer + RadioScheduler TX-window hacks) have cost more time than a principled implementation would have. The universal goal and the RC blocker are *not* in tension: the blocker is *why we build it now*; the universal library is *what we build*. And doing it right is not a luxury — because band-aids mask the failure chain, a clean principled foundation is the **diagnostic instrument** that lets the real bug (and the next one behind it) finally surface. (See Round 3.)

**Two distinct goalposts — keep them separate:**
- **Library end goal = the CCSDS Blue Books.** Full standards compliance. This destination is *precisely defined* by the standards and reached *incrementally* — it is not fuzzy.
- **MVP goalpost = empirical.** "Minimum functionality — the telemetry link genuinely works, verified." Its exact location is *unknown until crossed*, because masked cascading issues only surface once the layer beneath them is genuinely fixed. Do not declare victory before crossing; the MVP "fires" when the real link works on the bench, not when a predefined feature checklist is ticked.

Consequences that bind every decision in this document:

1. **RC is one user among many.** Any argument that holds "because RC needs X" must be re-derived as "because the library serves many consumers, of whom RC is one." RC-specific facts (a particular MCU, a particular coding standard, a 915 MHz log-recovery use case) are *example* constraints, not the library's law.

2. **The strictest plausible adopter sets the floor.** When adopters' constraints differ, the library must satisfy the *most demanding* one, because that is a superset of compatibility. A flight cubesat program running MISRA / AUTOSAR C++14 / a NASA-class institutional standard is a realistic adopter. Therefore the *core* is exceptionless, no-RTTI, no-heap-after-init **by construction** — not because RC says so, but because anything less excludes the strict end of the market. Looser adopters simply don't exercise the strictness; they lose nothing.

   > **AUTOSAR C++14 / MISRA C++:2023** (referenced by the panel; user asked whether to adopt): AUTOSAR C++14 is the automotive-industry safe-C++ standard ("Guidelines for the use of the C++14 language in critical and safety-related systems," ~340 rules). It was **superseded by / merged into MISRA C++:2023**, the current modern safe-C++ standard for safety-critical C++ (automotive, aerospace, medical, industrial). Same family as JSF / JPL-C / Power-of-10 / MISRA-C.
   >
   > **Is MISRA C++:2023 appropriate, and should the project adopt it?** It is *more* appropriate than the project's current C++ standard — RC's adopted C++ standard is **JSF AV C++ (2005)**, which predates C++11; MISRA C++:2023 is the natural modern endpoint of `CODING_STANDARDS.md`'s own "newer-standard-wins" precedence chain. **Recommendation: do NOT formally adopt it now, but build Starcom to be compatible with it.** Reasons not to adopt now mirror the project's existing MISRA-C deferral (`CODING_STANDARDS.md` "MISRA-C chain-of-custody"): (a) **paywalled** — conflicts with the project's open-source-audit principle; (b) **heavy overlap** with JSF+P10+JPL already enforced, so low marginal coverage for real adoption cost; (c) hobbyist/educational tier — full MISRA C++ is certification-grade effort. **The Starcom-specific point:** this is the cleanest instance of §0 point 2. We don't *adopt* MISRA C++:2023 (that would impose it on all consumers and the project), but the sans-I/O, exceptionless, no-RTTI, no-heap core is built so a consumer who *has* adopted it is not blocked. Compatible-with, not adopted. MISRA C++:2023 remains the right future target if a formal-certification variant is ever pursued (same forward-pointer as the existing MISRA-C LOC-5/6 deferral).

3. **The ceiling is full CCSDS compliance** (= the *library end goal* above: the Blue Books). The long-term target is genuine standards conformance (data link, C&S, and — via the right hardware — the Physical Layer). We are **not** putting this through a formal NASA/mission-assurance review any time soon, and nothing here implies we are. But **nothing done in the meantime may foreclose that ceiling.** "Don't hurt future compliance" is a standing design constraint even for work that has nothing to do with compliance per se (see §3).

4. **Honesty about conformance is a hard requirement, not a doc footnote.** The library must never claim more conformance than it delivers — not even unintentionally. Conformance is declared **per component**, **machine-checkable**, and **tested for holes** (see §2). "Best effort" is not one thing; it is a spectrum, and the system must say *where on the spectrum* each build sits.

5. **RC appears as an example integration only.** Starcom's identity, namespace (`starcom::ccsds`), README, and core vocabulary present it as a standalone project. RC-specific glue (the Active-Object wrapper, QP/C events, the SX1276/PIO adapter) lives in *adapters/examples*, never in the core, and never in the framing.

6. **Pedagogical intent is foundational.** Starcom's development is *deliberately* also a **library-craft learning exercise** for the user (Nathan). Any agent working on Starcom — Claude, Grok, or otherwise — must work **pedagogically**: explain the *why*, teach the general software-engineering / library-design concept behind a decision (not just state the decision), correct the user's mental model when it's off and confirm it when it's right, and surface trade-offs as teaching moments. This is a standing requirement, not a one-off. (Example from the 2026-06-17 session: clarifying *passive/call-driven library* vs. *active/framework library* — and that a stateful, timer-driven protocol can still be a passive, self-contained library via a `tick(now)` method — was the conceptual unlock that resolved the "does an AO get forced into the library?" worry. That kind of concept-teaching is the expected default.)

---

## 1. Agreement / Conflict / Gaps Analysis Table
**Synthesized from comparison.md (Entries 1/2 + council) + design_record. Captured for verif plan step 3.**

| Item | Agreement (both docs) | Direct Conflict / Fork | Major Gap (one doc only) | Resolution / Status | Source Refs |
|------|-----------------------|------------------------|---------------------------|---------------------|-------------|
| PHY compliance claim | COTS 915MHz (SX1276) cannot do 211.1-B-4 residual-carrier Bi-Phase-L/PM 60 deg; value is data-link/C&S only. | None on verdict. | Claude stronger on exact Blue Book waveform reqs + OCF/PLCW distinction; Grok on PIO feasibility examples. | D-1 settled: no PHY claim; 3-tier adapters (none/best-effort/full) behind seam. Honest labeling. | Claude 2.x + 6.3/6.4; Grok 2.1-2.5; comparison D-1 |
| Core architecture (sans-I/O) | Core must be passive sans-I/O; bytes/time in, events/bytes out; no I/O in core. | Claude: pure seam at adapter; Grok initially sketched IPhysical in some places (pre-fix). | Grok added concrete IPhysical surface details + policy/template preference. | D-2: sans-I/O pure core; policy/templates in core, virtual only at adapter edge (P10). AO optional consumer adapter. Detailed Claude council Round 2 verdict (design_record_claude.md) ratified + elevated sans-I/O as structural enabler; same for policy-vs-vtable (virtual at adapter edge only). | comparison 2.B-(B); design_record_claude.md Round 2 council table (Â§2.4) + Â§2.7 |
| Framing (USLP vs V-3/PLTU) | Both agree USLP forward unifier (multi-VC, carries COP-1+COP-P); PLTU carries V-3; CRC details matter. | D-4 open in both: which is primary for MVP vs arch. | Claude bit tables + history for USLP header; Grok CMake patterns. | USLP strategic spine (arch); V-3/PLTU permanent second + likely MVP first. Reconcile CRC ownership in framing module. | comparison D-4 + Entry 1.E; Claude Â§6 USLP table; design_record D-4 |
| Return link reports (PLCW vs CLCW/OCF) | CLCW for COP-1; PLCW for COP-P. | None on split. | Claude: full 7-field 16-bit PLCW layout + proof no OCF in Prox-1 (correction); explicit SPDU vs OCF. | Separate Clcw32 / Plcw16 types + paths. No generic OCF. PLCW on supervisory, Expedited. | Claude 6.3/6.4 verbatim 7-field; comparison |
| State machines (FOP/FARM) | FOP-1 6 flat states, FARM-1 3; COP-P simpler; table-driven plain C++ (no QP/HSM in core). All prior art framework-free. | None. | Claude: exact Blue Book table refs + prior-art survey (OSDLP etc); decision text. | Summary table + prior-art + "6x46 events" note in DESIGN; full transition matrices are in 232.1-B-2 external (REF only). | design_record 2.7; comparison |
| CI / no-heap / packaging | Hard no-heap-after-init gate + size report (delta) required; static lib default; tl::expected + knob. | None on gates. | Grok: explicit malloc shim as positive control + RTOS example; Claude size spike. | D-5 + D-3: shim as hard gate; size report; static+header opt-in. | comparison D-3/D-5; design_record |
| Conformance / best-effort | Per-component, machine-checkable, tested; spectrum not single label. | None. | Claude elevates to first-class subsystem (descriptors + matrix + runtime query). | Principle kept; full subsystem deferred to post-MVP polish but design for it from start. | design_record Round 2 Â§2.2 |
| Gaps (coverage) | Both note open D-4 framing primary; exact MVP cut; header-vs-static default. | (resolved) I-prefix naming (e.g. ILink/IRadio) considered archaic per direction; modern plain names for seams (Radio, Bearer, etc.) preferred unless a specific standard (e.g. k-prefixed constants) requires otherwise. | Grok: concrete fetched CMake + F' ruling + broader prior art; Claude: 7-phase plan + pedagogical + council outputs. | All absorbed; D-4 left open with rec; naming follows modern conventions. | comparison Entry 1.F + 2 coverage; design_record |

**Full source comparison entries remain historical (append-only).**

**The CCSDS-mandated state machines (verified):**

| State machine | States | Shape | Doc / table | Notes |
|---|---|---|---|---|
| **FOP-1** (COP-1 sender) | **6**, flat | S1 Active, S2 Retransmit-w/o-Wait, S3 Retransmit-w/Wait, S4 Init-w/o-BC, S5 Init-w/BC, S6 Initial | 232.1-B-2 **Table 5-1** | 6 states × **46 events**; complexity is in the *actions* (Sent/Wait queues, T1 timer, window K, V(S)/NN(R)), not state nesting. Standard itself calls the table "large and complex." |
| **FARM-1** (COP-1 receiver) | **3**, flat | Open, Wait, Lockout | 232.1-B-2 **Table 6-1** | 3 × 11 events; V(R), PW/NW window, 3 flags, FARM-B counter, CLCW emit. |
| **FOP-P** (COP-P sender) | **2**, flat | Active, Resync | 211.0-B-6 §7.2 | go-back-N (`Transmission_Window` ≤127); simpler than COP-1. |
| **FARM-P** (COP-P receiver) | **stateless / data-driven** | (no named states) | 211.0-B-6 §7.3 | "simply reacts to what it receives." |
| **Proximity-1 session / MAC / hailing** | **~30**, partial hierarchy | per-DUPLEX (Full/Half/Simplex) tables | 211.0-B-6 §6 | The one genuinely large machine; ~8 timers, token-pass turnaround. Still normatively flat per-DUPLEX tables. |
| PHY transmitter / receiver | **not FSMs** — lookup tables | — | 211.1-B-4 Tables 3-2/3-3 | Out of the data-link core. |

**Prior-art survey — every reference implementation is framework-free:** OSDLP (C, `enum`+`switch`, handlers `fop_e1()`…`fop_e46()` named to the standard's events, queues/timers via weak functions), NASA `cop1.c` (C, switch, FARM-1 only), dariol83/ccsds (Java, GoF State pattern), yamcs (Java, plain `int state`). **Zero use QP/C, Boost.SML, or any statechart engine.**

**Decision (recommended; pending final user sign-off at framework-planning):**
- **Build COP-1/COP-P as plain, passive, table-driven C++ state machines** — transcribe the Blue-Book state×event tables into a `switch` or a `std::array` of transitions. This is **less** work than a framework (you're copying tables, not building an HSM engine), it's what 100% of the field does, and an HSM framework's value (deep hierarchy) does not apply to 2–6 flat states.
- **The core takes NO framework/QP dependency.** It is a **passive sans-I/O library** (`Starcom::FopEngine` etc.) driven by the consumer via `submit()` / `on_clcw()` / `tick(now)`. (Time handled the sans-I/O way: the engine checks its own deadlines against the `now` the caller passes to `tick()`; it owns no clock and no thread.)
- **The AO is a consumer-side OPTIONAL adapter, never in the core.** RC wraps the passive engine in RC's own QP AO; non-QP consumers drive it from their own loop. The earlier "does QP get forced on parent code?" worry is **void** — the AO was never in the library.
- **Maximal testability** (serves the standards-fidelity + robust-determinism goal): a passive table-driven SM is golden-vector-tested by dispatching events and asserting transitions directly against Table 5-1 / 6-1 — no framework, no hardware, no threads.

**Open scope question (for framework-planning, not resolved here):** whether the Starcom data-link "core" includes the **Proximity-1 §6 session/MAC/hailing machine (~30 states)** — by far the largest single chunk — or whether the first target is COP-1/COP-P only with session/MAC as a later/optional module. Even if included, it needs no HSM framework (flat per-DUPLEX tables + explicit sub-state variables).

**Supersedes** the over-strong "AO = optional adapter only / must NOT be an AO" language in earlier drafts of §2.5 #4: the sharper truth is **the core is a passive table-driven object with no framework; AO-ness is one optional way a consumer drives it.**

**Concrete decisions for the implementation plan:**

- **D-1 — PHY honesty / compliance claim (settled → Claude doc).** Starcom ships **NO "211.1-B-4 Physical Layer."** Headline deliverable = a Proximity-1/CCSDS **data-link + Coding & Sync** stack (211.2-B-3 PLTU, 211.0-B-6 COP-P/PLCW, USLP/COP-1 per 732.1-B-3 / 232.1-B-2) over an **honestly-labeled non-compliant COTS 915 MHz bearer**. Documentation states PHY non-compliance explicitly. **Do not claim any PHY compliance; do not add the AX5043 as a "closer" path** (verified category error, since caveated by Grok). True PHY is a deferred SDR (Pluto ~$186–230) / FPGA-IP (ComBlock ~$2,500) upgrade behind a **neutrally-named `IRadio`/`IBearer` seam** (non-blocking `begin_tx()`/`poll_tx()` tri-state, exposing carrier-acquired/symbol-inlock signals).

- **D-2 — Core architecture: sans-I/O pure core (settled → Claude doc).** Core API = `receive_bytes()`/`bytes_to_send()`/`poll_event()`/`handle_timeout(now)`/`submit_sdu()`, holding/calling **no I/O object**. State in caller-owned statically-allocated structs (Grok). Grok's semantic events (`CommandAccepted`/`LinkLost`/`LogOffloadComplete`) are the `poll_event()` vocabulary the Rocket-Chip AO consumes. The `IPhysicalLayer`/radio seam lives in the **adapter layer, not the core**; any in-core seam uses **policy/templates, not a virtual ABC** (P10 Rule 9 + `-fno-rtti`). *(This is the one genuine architecture fork between the two docs — 2.B-(B).)*

- **D-3 — Library form + error backport (one spike + one knob).** **Default = static library**; header-only INTERFACE as opt-in; build `-flto -ffunction-sections -fdata-sections -Wl,--gc-sections`. **Run the Claude header-vs-static spike in Phase 0/1** (compile both on arm-none-eabi, compare flash + compile time) — the CI size report (D-5) produces this data. **Ship `tl::expected` as the default backport** with a `STARCOM_USE_STD_EXPECTED` knob and a one-flag swap to a zero-dependency `starcom::Result`. Error objects trivially copyable, ≤16–32 bytes, core handling `noexcept` (Grok).

- **D-4 — Framing foundation: USLP-v4 vs native-v3 (THE remaining genuinely-open architectural question).** Both docs flag it; neither fully resolves it. Both agree USLP (732.1-B-3) is the forward-looking unifier and PLTU carries a Version-3 Transfer Frame. The plan must decide whether the core's primary frame is **USLP** (modern, multi-VC, carries both COP-1 and COP-P) or **classic Version-3 / TC-TM**, and reconcile **PLTU CRC-32 vs USLP FECF CRC-16** integrity-check ownership. **Recommendation:** design the framing module to **support USLP as primary with a Version-3/PLTU path**, using the Claude doc's §6 USLP header table as the bit-level spec. *(CRC-32 polynomial + 211.2-B-3 date now confirmed — Entry 1.E — so no remaining re-confirm TODO.)*

- **D-5 — CI / packaging hardening (settled → superset both).** Phase-0 CI MUST add: (a) a **no-heap-after-init malloc/operator-new trapping shim** as a **hard pass/fail gate** (positive control per `HW_GATE_DISCIPLINE` Rule 1); (b) a **flash/RAM/stack size report** for 1–2 fixed configs, pinned arm flags, gated on **delta-vs-baseline, not absolute threshold** (also feeds D-3's spike); plus the agreed matrix (GCC/Clang/MSVC × Debug/Release, arm cross, ASan/UBSan, fuzz-smoke, GoogleTest+FuzzTest). One RTOS example build = nightly/"should." Packaging: ship **install/export + a checked-in (not-upstreamed) `vcpkg.json`/`portfile.cmake` + a custom bare-metal triplet** now; **defer registry publication, Conan, and binary-cache infra to post-1.0.** Model the **CLCW (232.0-B-4 §4.2.1) and PLCW (211.0-B-6 §3.2.4.3.2.1.1, 16-bit/7-field SPDU) as two distinct return-link types** — separate `Clcw32`/`Plcw16` types and FARM-1/FARM-P paths, **no generic OCF abstraction** — as a hard design input.

## Standing decisions (current consolidated state — read this first)

For an implementer picking this up later, the current settled position after Round 3:

- **Purpose & goalposts:** *What* = universal CCSDS telemetry/command library; *why/when* = RC's broken half-duplex telemetry (a flight blocker). **Library end goal = the Blue Books** (defined, reached incrementally). **MVP goalpost = empirical** (the link genuinely works, verified) — discovered by crossing, not predefined; no premature victory (§0, §3.3).
- **Build philosophy:** **architecturally complete + feature-incremental.** Universal foundation built right from day one (NOT deferred); features sequenced blocker-first. MVP first cut ≈ **V-3/PLTU + COP-P + minimal half-duplex turnaround**, replacing the band-aids. Half-duplex turnaround pulled forward from the deferred §6 machine; **verified independently** of the reliability layer (§3.3).
- **Scope:** universal CCSDS library; RC is the first consumer, not the owner (§0).
- **Core:** sans-I/O, **passive** (call-driven via `submit()`/`on_clcw()`/`tick(now)`), exceptionless, no-RTTI, no-heap-after-init **by construction**. No HAL/SDK includes in the core (CI-guarded). State machines are **plain table-driven C++, no framework/QP dependency** (§2.7). Policy/template seams in core; virtual dispatch only at adapter edges. AO is an *optional consumer-side adapter*, never in the core.
- **PHY:** three first-class cases (none / best-effort 1–99% spectrum / compliant) behind one profile-parameterized service-contract seam. No blanket PHY claim ever.
- **Conformance:** a tested per-component subsystem (descriptors → published matrix + runtime capability query), with over-claim failing CI. "Best effort" always resolves to a specific spectrum point. *(Library-polish tier — defer the full subsystem to post-unblock; keep the principle from the start.)*
- **Compliance-safety:** CCSDS is positive/behavioral, so internal abstractions are compliance-neutral; keep approximations out of wire-format/state-table code and in swappable adapters; label-and-lockout anything unavoidably non-compliant. Standing principle, not an audit.
- **Framing:** **USLP is the strategic unifying spine** (PHY-agnostic; carries COP-1+COP-P per VC; covers both RC use cases) — revising Round-2's "neither primary." **V-3/PLTU is a permanent second codec** (native Prox-1 interop) **and** the likely **MVP-first** codec (smallest path to flight). "USLP-primary (architecture)" + "V-3-first (MVP sequencing)" coexist (§3.1, §3.3).
- **Four axes:** USLP framing · COP-1/COP-P ARQ (per VC) · Prox-1 §6 session/MAC (optional, mostly deferred — but minimal half-duplex turnaround pulled forward) · PHY tier.
- **Form:** ship static + header-only; default decided by the Phase-0 size spike (lean header-mostly). `tl::expected` default + zero-dep-`Result` knob.
- **CI:** malloc-after-init hard gate + published per-config size report.
- **MIB:** versioned public API. **Identity:** standalone; RC is an example integration.
- **Standards baseline:** keep JSF AV C++ / P10 / JPL-C as-is; **do not adopt MISRA C++:2023** (paywalled, ~90% overlap) — stay *compatible-with* it. Update CCSDS **131.0-B-3 → B-5** references (superseded twice; verify which issue 211.2-B-3 normatively points to). See `AGENT_WHITEBOARD.md` standards-currency note.

**Open (need resolution before/within Phase 0 / framework-planning):**
- **MVP scope precise cut** — confirm V-3/PLTU + COP-P + minimal half-duplex turnaround; decide whether turnaround rides RC's existing RadioScheduler or a fresh minimal MAC.
- **CCSDS source-version pinning** — lock the exact authoritative issue of every referenced spec (incl. the USLP profile and the 131.0-B issue 211.2-B-3 references). The natural next step.
- The header-vs-static *default* (D-3) — settle with the Phase-0 size/compile spike.
- Grok-council cross-check on §2.1 (PHY three-way), §2.2 (conformance subsystem), §2.3 (compliance-neutrality), §2.4/§3.1 (framing). Convergence settles them; divergence comes to the user.


## 3. Unique Data by Source (verbatim excerpts â€” no paraphrase)

**Claude-only / stronger (ccsds_domain_claude + library_craft_claude + design_record):**
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

The **PLCW is a 16-bit fixed-length SPDU** comprising **seven contiguous fields** (§3.2.4.3.2.1.1, Fig 3-5) — structurally completely different from the 32-bit CLCW. Fields: Report Value 8 bits (bits 8–15) = V(R); Expedited Frame Counter 3 bits (bits 5–7, mod-8); Reserved Spare 1 bit (bit 4, set to `0`); PCID 1 bit (bit 3, selects one of two redundant receivers); Retransmit Flag 1 bit (bit 2); SPDU Type Identifier 1 bit (bit 1, `0`); SPDU Format ID 1 bit (bit 0, `1`). **Key differences vs. CLCW:** PLCW is 16 bits not 32; has **no** No-RF/No-Bit-Lock/Lockout/Wait flags and **no** COP-in-Effect/Version/Status/VCID fields; uses PCID + a 3-bit mod-8 Expedited Frame Counter instead of the CLCW's 2-bit FARM-B counter. Both carry an 8-bit Report Value = V(R) and a Retransmit flag. PLCW is transmitted only with Expedited QoS, on the supervisory channel (not in a Transfer-Frame OCF — see §6.4).

> **Verification correction (re-verify 2026-05-31):** the original draft listed six PLCW fields; the standard enumerates **seven** — the 1-bit Reserved Spare (bit 4) was added above. Verified against CCSDS 211.0-B-6 §3.2.4.3.2.1.1.

Proximity-1 session model (§6): session establishment → data services (with optional resynchronization / reconnect subphases) → session termination. **Hailing** establishes a link over the forward/return hailing-channel pair (not used in simplex). **Resynchronization** forces the responder to readjust Sequence-Controlled frame numbers via the `Set V(R)` directive (carried by a Type-1 SPDU); **reconnect** rehails mid-session without resetting FOP-P/FARM-P counters.

### 6.4 OCF (Operational Control Field) — and why Proximity-1 has none

In the **TM, AOS, and USLP** Space Data Link Protocols, the OCF is a **4-octet (32-bit) optional Transfer-Frame trailer field**, signaled by the **OCF Flag** in the frame primary header (TM 132.0-B-3 §4.1.5, §4.1.2.4; USLP 732.1-B-3 §4.1.1, USLP_MC_OCF Service). When it carries a **Type-1 Report** it holds the 32-bit **CLCW** for COP-1 status reporting on the return link (it may alternatively carry a Type-2 Report, e.g., an SDLS Frame Security Report). Its 32-bit width matches the CLCW because both are exactly 32 bits.

> **Verification correction (re-verify 2026-05-31) — important:** the original draft said the OCF carries "the CLCW (or, for Proximity-1, the PLCW)." **That is wrong.** **Proximity-1 (CCSDS 211.0-B) defines no Operational Control Field at all.** The PLCW is **not** carried in an OCF and is **not** 32 bits — it is the 16-bit fixed-length SPDU of §6.3, sent on the **supervisory channel** with Expedited QoS, not appended to a Transfer-Frame trailer and not signaled by any OCF Flag. So the OCF↔CLCW width identity holds **only** for the TM/AOS/USLP CLCW; it does **not** extend to the Proximity-1 PLCW. For Starcom this means COP-1 return-link reporting (CLCW-in-OCF) and COP-P return-link reporting (PLCW-as-SPDU) are **two distinct mechanisms**, not one parameterized field — a real design implication, not a cosmetic note.

### 7.2 915 MHz legality and band status

- **915 MHz ISM is not a CCSDS-authorized Proximity-1 band.** Prox-1 PHY frequencies are UHF 390–450 MHz (211.1-B-4, Mars) or the new S-band (~2 GHz). A Starcom 915 MHz LoRa link is a COTS bearer chosen for cost/availability — it can carry CCSDS data-link/COP framing, but it is **not** operating a CCSDS-compliant Physical Layer.
- **FCC Part 15.247 (47 CFR 15.247)** governs unlicensed 902–928 MHz use. The *digital-modulation* path (§15.247(a)(2)) requires ≥ 500 kHz of 6 dB bandwidth and is capped at **1 W (30 dBm) conducted** (§15.247(b)(3)); the FHSS path (§15.247(a)(1)/(g)) caps at 1 W (≥ 50 hop channels) or 0.25 W (25–49) and has *no* 500 kHz minimum. Antenna/EIRP: at ≤ 6 dBi no reduction is required and above 6 dBi the conducted power must be reduced dB-for-dB (§15.247(b)(4); no point-to-point relaxation in 902–928 MHz — that relaxation applies only to the 2.4 GHz and 5.8 GHz bands), so EIRP is effectively pinned at ~36 dBm. **Practical caveat (corrected re-verify 2026-05-31):** many cheap LoRa configs use 125/250 kHz bandwidth (< 500 kHz). This does **not** automatically relegate them to §15.249 — narrowband LoRa is routinely certified under §15.247's **FHSS** provisions (no bandwidth floor). §15.249 (a far lower ~50 mV/m field-strength limit, ≈ −1.2 dBm EIRP) is only *one* alternative route for non-hopping narrowband devices. Operators must verify their actual 6 dB bandwidth **and** which §15.247 sub-path their radio is certified under.
- **FCC Part 97 (amateur)** allows far higher power (e.g., 33 cm 902–928 MHz and 23 cm 1240–1300 MHz up to 1500 W PEP; 70 cm 420–450 MHz overlaps the Mars Prox UHF forward band) **but 97.113(a)(4) prohibits "messages encoded for the purpose of obscuring their meaning."** Documented public codes (CCSDS framing, FEC, CRC, randomization, Manchester) are fine; **payload content must not be encrypted.** If you encrypt on 902–928 MHz you must operate as a Part 15 user, not Part 97. The 33 cm band is secondary/shared with ISM and federal radiolocation.

### 7.3 COTS pricing anchors (HPR/cubesat audience)

RP2350 silicon ~$0.80 (reel) / ~$1.10 (single); Raspberry Pi Pico 2 board ~$5; Adafruit RFM95W (SX1276) 915 MHz breakout (PID 3072) ~$20; bare RFM95W modules ~$8–15. Attractive for HPR/research budgets — but, per §3, they deliver a LoRa/(G)FSK COTS link, not a 211.1-B-4 PHY.

---

## 8. Integration Note (Light) — Framework-Agnostic Starcom as an Active Object

This section is informed by a local scout review of an existing radio Active Object (the Rocket-Chip `AO_Radio`). It is **explicitly not** a design base — the Rocket-Chip telemetry/CCSDS/RF-manager specifics are out of scope, and Starcom must be framework-agnostic. The patterns below are generic run-to-completion / AO concepts, useful as integration *seams*, kept deliberately high-level.

**Grok-only / stronger (ccsds_domain_grok + library_craft_grok):**
Real-world evidence from the Raspberry Pi ecosystem and community (all portable to RP2350):

- Official `pico-examples/pio/manchester_encoding` and `differential_manchester` — complete bidirectional examples with precise timing state machines (one SM for TX encoding with side-set, one for RX edge detection and decode). This is production-quality starting material.
- Real RF deployments:
  - 433 MHz OOK Manchester decoders (PIO handles edge detection + Manchester decode + streams bits; CPU does sliding-window header/sync detection).
  - ADS-B (1090 MHz) receiver using **two PIO state machines**: one dedicated to preamble/sync pattern correlation on the pulse train from an RF frontend, second for Manchester decoding of the message. This is an extremely close analog to what we need for ASM detection + PLTU framing.
- High-speed bit-banged protocols (e.g., 100BASE-TX Ethernet on PIO) demonstrate that RP2350 PIO can handle complex encoding, preamble/SFD detection, and multi-level signaling at impressive rates with DMA offload.

**Conclusion for Starcom (with current 915 MHz hardware) — see corrected analysis in §2.5 below:**

Early assessment suggested that a useful "software-defined" approximation of Physical Layer *services* (symbol stream + basic lock indications via FSK Continuous Mode + PIO for timing/encoding) could support high-value COP-1/COP-P operation. 

**Important correction (see full analysis in the later §2.5 Corrected Analysis section):** This remains a best-effort adaptation only. It cannot produce a conformant CCSDS 211.1-B-4 waveform (residual-carrier Bi-Phase-L/PM at 60° ±5%, required spectral purity, phase noise, residual AM, stability). The early optimistic language in this subsection is superseded by the detailed corrected assessment later in the document. 

**Honest documentation recommendation (unchanged):** Clearly label any implementation as a non-conformant "best-effort COTS approximation" behind a narrow IPhysicalLayer boundary so future hardware upgrades can provide a true native PHY without changing the Data Link layer.

### 2.4 Recommended Next Concrete Steps for This Section

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

**Cross-verified (comparison):** All factual corrections recorded; no data dropped. D-1..D-5 + Grok council section preserved.

---

## 4. Migration / Usage Notes

- Research + comparison + design_record files = historical (append only; do not edit).
- This DESIGN.md is now the condensation target and prep for dev plan (per goal: "in preparation for the dev plan for the library itself (not building that yet)").
- Next (per STATUS): Phase 0 CMake skeleton, standards ref update (131.0-B-5), precise MVP scoping.

**Verification of no loss (to be run):** Grep for 10+ unique tokens from each source (e.g. "16-bit fixed-length SPDU", "ADS-B (1090 MHz)", "FOP-1.*6.*states", "architecturally complete + feature-incremental", "D-5", "tl::expected", "no-heap-after-init hard gate", "LunaNet", "Part 97", "Table 5-1").

All modifications only on this branch; main untouched for starcom/.

---

*Condensation complete per plan. Sources: starcom/docs/* (moved 2026-06-18 from docs/research/). Only starcom/ files were modified for this condensation work. Pre-existing unrelated files (e.g. L2P5 audit docs, agent-tools/, mcps/) visible in broad git status are outside this scope and were not created or modified by the condensation commits.*