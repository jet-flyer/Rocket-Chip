# Starcom — Claude Design Record (Scope, Decisions & Council Verdicts)

**This is the Claude-side consolidated design record for Starcom.** It began as a council-verdict document and has grown into the living record of the Claude-side scope, architecture decisions, council rounds, focused research passes, and design notes. (Filename retains `..._COUNCIL_VERDICT` for continuity; the content is broader than councils.) A separate Grok-run council reached its own verdict (recorded in `STARCOM_RESEARCH_COMPARISON.md`, "Council Review — Universal CCSDS Scope" section). The two are independent panels on the same material — convergence is strong signal; divergences are flagged inline. A future **condensation session** (see flag below) will merge everything into a single canonical record.

**Status:** Forward-going source of truth for Starcom's scope, audience, and architecture decisions *as seen by the Claude council*. Where this document and the earlier research artifacts (`*_STARCOM_*_RESEARCH.md`) differ in *framing*, **this document governs** (pending the condensation merge). The research docs remain valid as the underlying domain/library-craft research; this verdict reframes their conclusions for the library's actual audience.

> ### ⚑ CONDENSATION SESSION — REQUIRED FOLLOW-UP
> The four research docs + the comparison doc + this Claude verdict + the Grok council verdict are **not yet condensed into one canonical record.** A dedicated condensation session must: (1) merge the Claude and Grok council verdicts (reconciling naming — see the naming note in §2.1/Standing Decisions), (2) re-frame the four research docs' RC-centric language to the §0 universal scope (per the §2.6 audit — research docs were deliberately **not** edited in this session), and (3) produce the single source-of-truth design-scope doc. Until then, no research doc's framing has been corrected in place; this verdict's §0 is the governing framing by reference only.

**Append-only, attributed.** This is a shared multi-agent record. Council rounds are appended as `## Round N` (with running agent + date); focused research passes, design notes, and direct user↔Claude conclusions are appended as dated sections/notes. Do not rewrite prior entries; append.

---

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

## Round 2 — Claude council (2026-06-17): universal-library reframe

**Run by:** Claude (Opus 4.8, Code) — 2026-06-17. Personas per `COUNCIL_PROCESS.md`: NASA/JPL Avionics Lead, ArduPilot Core Contributor, Embedded Systems Professor, Cubesat Startup Engineer (auxiliary). This is the **Claude** panel, run in parallel to a Grok-run council on the same material. Round 1 (Claude, 2026-06-02, conversation-only — not separately documented) reasoned with RC too central; the user corrected the frame and this round re-runs under §0.

> **Cross-check with the Grok council (now complete):** The Grok-run council finished and recorded its verdict in `STARCOM_RESEARCH_COMPARISON.md` ("Council Review — Universal CCSDS Scope"). **Substantive convergence:** both panels independently land the universal-CCSDS scope, the three PHY tiers (none / best-effort / full-compliant) as **optional adapters outside the core**, and a transport abstraction. **Divergence (naming, for the condensation session to reconcile):** Grok proposed the core be called **"Starcom Core"** and the transport seam **`ILink`**; the Claude panel proposed a neutral PHY/transport seam name `IRadio`/`IBearer`. These are compatible in intent (a neutrally-named, non-PHY-committal transport seam) — the exact token is a naming decision, not an architectural one. The convergence on *substance* across two independent panels is the strong signal; the naming is a condensation-session pick.

### 2.1 Physical Layer — a first-class three-way design axis (the biggest reframe)

Round 1 collapsed the PHY into "you can't emit 211.1-B-4 on the SX1276, so claim nothing." That is true **for RC's radio** and was never a statement about the *library*. Under §0 the PHY is a deliberate **three-population design axis**, all first-class:

- **(a) No PHY** — the data-link/C&S-only consumer. They own emission entirely (vendor radio, SDR, a wire, a UDP socket). The library hands them bytes and never touches a radio. This is the **default** and the majority case (ground stations, cubesats with a vendor transponder).
- **(b) Best-effort PHY** — the PIO/COTS consumer (RC's SX1276 is the exemplar; also HAB, hobby cubesats on LoRa/FSK). The library *helps* with symbol-stream services (Bi-Phase-L/Manchester shaping, ASM correlation, lock heuristics) while declaring it is **not** 211.1-B-4. Critically, "best-effort" is a **spectrum from ~1% to ~99%** of the PHY service surface — not a single implementation.
- **(c) Fully-compliant PHY** — the consumer with an SDR (ADALM-PLUTO) or FPGA IP (ComBlock COM-1852SOFT) that *can* synthesize the residual-carrier Bi-Phase-L/PM waveform and wants the library's PHY abstraction to drive it. Includes the future **S-band Proximity-1** profile (lunar; the JPL User Terminal class of mission).

**Design mandate (unanimous):** **one PHY abstraction = the 211.x PHY *service contract*, with three (or more) implementors.** Key properties the panel insisted on:

- **The seam carries the CCSDS PHY service vocabulary** — coded-symbol-stream in/out, `CARRIER_ACQUIRED`, `SYMBOL_INLOCK_STATUS` — because the MAC/data-link layers above legitimately depend on those for hailing/acquisition. (JPL, Cubesat)
- **"No PHY" must be expressible as absence** — the sans-I/O core (§2.2) doesn't call the PHY, so case (a) is the natural default, not a special case to engineer around. (JPL, Professor)
- **A capability query on the interface** — an adapter must *honestly report what it supports* ("symbol stream + soft lock, no coherent ranging"), so upper layers degrade gracefully and never assume a capability an approximation lacks. (ArduPilot)
- **Compliant-waveform needs stay in the adapter, never leak to the core** — modulation index, coherent turnaround/ranging, the 13-rate symbol table are PHY-adapter concerns. If the data-link core ever sees a "modulation index," the layering is wrong. (ArduPilot)
- **The seam is parameterized by *profile*** — UHF Mars (211.1-B-4), S-band lunar, COTS best-effort — not hardcoded to the one published UHF waveform. Profile specifics stay out of the core (per the comparison doc's two-tier source rule: don't hardcode S-band params). (JPL)

**This resolves rather than contradicts the old D-1.** "Don't claim 211.1-B-4" becomes: **the library makes no blanket PHY claim; each adapter declares its own conformance** (§2). The no-PHY build claims nothing (correct); the best-effort adapter declares "approximation, non-conformant, here is exactly what it does and doesn't do"; the compliant adapter declares "211.1-B-4 conformant on this hardware." Conformance is a **per-adapter property**, not a property of "Starcom."

*Grok cross-check:* does the Grok panel also land the PHY as a three-way axis behind one profile-parameterized service contract?

### 2.2 The per-component Conformance System (the user's explicit ask — made a first-class subsystem)

The user's requirement, verbatim intent: *"the library makes no blanket PHY claim; each adapter declares its own conformance. There should be a system for this in the library that is tested for robustness and holes. Best-effort is not a single implementation — it's a spectrum from 1%–99% compliance."*

The council elevates this from "write an honest paragraph" to a **tested library subsystem**:

- **Conformance is declared as structured data, per component**, not prose. Each framing codec (V-3/PLTU, USLP), each COP engine (COP-1, COP-P), and each PHY adapter (none / best-effort-N / compliant / S-band) carries a machine-readable **conformance descriptor**: which Blue Book + issue it implements, which features are *conformant*, which are *approximated* (with a stated fidelity bucket), and which are *absent*.
- **A conformance matrix is a build-time artifact**, generated from those descriptors — the published "what this configuration actually is" table an adopter reads before trusting it. (Professor)
- **It is tested for holes.** The robustness tests must catch: (i) a component that *claims* a feature it doesn't implement (over-claim — the dangerous direction); (ii) a code path that emits/accepts wire output inconsistent with its declared conformance level; (iii) a "best-effort" adapter silently presenting itself as more capable than it is via the capability query. Property-based + golden-vector tests assert *claim ⇔ behavior* equivalence. An over-claim must fail CI. (JPL, Professor — this is the positive-control discipline: a conformance claim with no test proving it is a structurally-soft gate.)
- **The capability query (§2.1) is the runtime face of the same descriptor** — so upper layers can refuse to assume an absent capability, and an integrator can `static_assert` on a required conformance level at compile time.
- **"Best-effort spectrum" is explicit.** A best-effort PHY adapter states its fidelity per service (e.g. "Bi-Phase-L symbol shaping: present; residual-carrier PM: absent; ASM correlation: present; coherent ranging: absent"), so "best effort" always resolves to *a specific point on the spectrum*, never a vague label.

*Grok cross-check:* does the Grok panel also call for a tested conformance-declaration subsystem (vs. documentation-only)?

### 2.3 "Does anything we plan break future full compliance?" — explicit analysis (user's standing question)

**Finding: almost certainly no, and the reason is structural.** Recorded here as a standing design principle, not a one-time check.

- **CCSDS Blue Books are positive-conformance / interoperability specs.** They constrain *wire output and observable protocol behavior* ("the bits shall be X; the state machine shall transition Y"). They do **not** constrain internal software architecture. This is the opposite shape from JSF/MISRA/P10, which are *prohibitive* ("do not use feature W"). (Professor, JPL — both independently)
- **Therefore internal abstractions cannot violate a CCSDS rule.** A sans-I/O core, an `IRadio`/PHY seam, a policy/template dispatch, a conformance descriptor — all invisible to a conformance test, which only ever inspects wire bytes and protocol behavior. There is no CCSDS rule an abstraction layer can break. (Unanimous)
- **The only two real ways to foreclose future compliance:**
  1. **Bake a non-compliant choice into the wire format or the state table** such that a later compliant build cannot reproduce the correct bits/behavior. *(Mitigation: the framing codecs and COP engines are written to the Blue Book exactly; approximations never live in the wire-format or state-table code — they live in the PHY adapter, which is swappable.)*
  2. **Hardcode an approximation where the standard mandates a specific value, with no path to the correct one.** *(Mitigation: managed parameters and profile values are configuration, never literals in the core; the conformance descriptor flags any approximation so it cannot silently become load-bearing.)*
- **Backstop (the user's "last resort"):** anything that genuinely cannot be made compliant at the partial/zero level, and is otherwise unavoidable, is **explicitly labeled in its conformance descriptor and able to be locked out** (compile-time flag / capability refusal). Nothing non-compliant is ever silent or unremovable.

**Bottom line:** this is a design *principle* ("never let an approximation become load-bearing in the wire format; keep approximations in swappable adapters; declare and gate anything non-compliant"), not an open-ended audit. Low rabbit-hole risk. The conformance system (§2.2) is the mechanism that enforces it.

*Grok cross-check:* does the Grok panel agree CCSDS conformance is positive/behavioral (so internal abstractions are compliance-neutral), or did it surface a specific wire-format/state-table trap to avoid?

### 2.4 Re-examined decision points (from the comparison doc's D-1…D-5)

The reframe re-derives several Round-1 verdicts. RC-colored reasoning is replaced with universal reasoning; conclusions that change are flagged.

| Item | Old (RC-colored) verdict | Round-2 (universal) verdict |
|---|---|---|
| **PHY / D-1** | "No 211.1-B-4 claim; neutral seam." | **Three first-class cases (none / best-effort-spectrum / compliant) behind one profile-parameterized PHY *service-contract* seam; conformance declared per-adapter and tested (§2.1–2.2).** Major reframe, not a contradiction. |
| **Core / D-2 (sans-I/O)** | Ratified for testability. | **Ratified + elevated** — sans-I/O is the *structural enabler* of the three-way PHY story (it makes "no PHY" the default). Add a CI guard failing the build on any `hardware/`/`pico/`/HAL `#include` under the core — reframed from "RC hygiene" to "the contract that lets unknown consumers drop the core onto hardware we never imagined." |
| **D-2 / policy-vs-vtable** | "No virtual ABC because P10 (RC's standard)." | **Same conclusion, stronger logic:** avoid virtual dispatch *in the core* because it would *exclude* every P10/MISRA/AUTOSAR adopter — satisfying the strictest is a superset of compatibility. **Virtual dispatch is allowed at the adapter edge** (integrator-selected, not in the no-RTTI hot path) where the adapter's own consumer permits it. Don't over-apply the rule. |
| **Framing / D-4** | "V-3/PLTU **primary**, USLP deferred." | **Corrected: neither is "primary."** For a universal library, V-3/PLTU and USLP V-4 are **co-equal peer codecs** behind one codec shape. Implement **V-3/PLTU first** on *sequencing* grounds (smaller codec, shortest tested vertical slice, carries the COP-P/PLCW path), but USLP is a **peer, not an afterthought** — modern missions consolidating TC/TM/AOS want USLP specifically. CRC-32-vs-CRC-16 ownership becomes a per-codec property. Drop the word "primary." |
| **Form / D-3 (static vs header-only)** | "Static is the answer (RC = one fixed toolchain)." | **Reopened as a genuine trade.** Header-mostly is the dominant aerospace-library distribution form (ETL, libcyphal, MAVLink, COMMS) *because* it ports to unknown toolchains with zero per-target build — which matters a lot more for a universal library than for RC. **Ship both forms** (static target + header-only INTERFACE); settle the *default* with the Phase-0 size/compile spike, leaning toward the **header-mostly market norm** unless the data contradicts. `tl::expected` + zero-dep-`Result` config knob unchanged (it already is the satisfy-strict-and-loose pattern). |
| **CI / D-5** | Ratified as discipline. | **Strongly ratified + elevated** — the malloc-after-init hard gate and the per-config flash/RAM/stack report are **adoption/assurance enablers** for the broad market (a cubesat reviewer asks "prove no heap after init; show me size for my config"). **Publish the size report as an artifact**, not just CI-internal. |

### 2.5 New items the universal frame surfaced (beyond D-1…D-5)

1. **Conformance matrix as a deliverable** (§2.2) — per-codec/per-COP/per-PHY-adapter, machine-generated, published. The RC frame only demanded one honest sentence; the universal frame demands a tested system.
2. **The MIB / managed-parameters surface is a versioned public API.** Multiple independent consumers depend on the *set, names, and units* of exposed managed parameters, so it is SemVer-bound interface design, not an internal struct. Elevates "expose the MIB" from implementation note to interface-design task.
3. **Standalone identity** — `starcom::ccsds` namespace, README, and core vocabulary present Starcom as its own project; RC is an *example integration*, absent from core naming and framing. (First-impression matters: a cubesat evaluator can't open the repo and read "some rocket project's telemetry code.")

4. **⚑ DESIGN FLAG — implement the Blue Book state machines as a portable HSM in the core; an Active-Object wrapper is an OPTIONAL adapter, not the core.** (User-raised, 2026-06-17; design-planning input, not resolved here.) The Blue Books **prescribe exact state-transition tables**: COP-1's **FOP-1 (states S1–S6)** and **FARM-1 (Open/Wait/Lockout)** per 232.1-B-2 Tables 5-1/6-1, and COP-P's FOP-P/FARM-P. An implementation "shall transition" exactly as tabulated.

   **Can we use Active Objects to implement the Blue Book rules? Yes — and AOs are NOT ruled out.** Two points, kept distinct because they have different sources of authority:

   - **(CCSDS says nothing about this.)** Verified against the CCSDS docs: **the Blue Books specify the FSMs as *behavior* (transition tables), not as *code structure*.** CCSDS gives the authoritative tables and explicitly leaves *implementation technique* to the implementer — there is **no CCSDS guideline** prescribing or forbidding AO / HSM / switch-table / coroutine / anything. A CCSDS state table maps almost 1:1 onto a hierarchical state machine (states→states, rows→transitions, the "actions" column→transition/entry actions), so an HSM is a natural, fully-compliant vehicle. **So "prefer HSM, make AO optional" below is a *library-universality* judgment — NOT a CCSDS rule.** Nothing in the standard constrains this choice.

   - **(Universality says: core = portable HSM; AO = optional adapter.)** The §0 universal-library goal — not CCSDS — is what drives the structure. The single hard line: **AO-ness must wrap the core, never be baked into it.** Concretely:
     - ✅ **Core: the protocol FSM is a pure, portable Hierarchical State Machine *object*** — driven, no thread, no queue, no framework dependency, **usable standalone**. (QP/C models exactly this split: `QHsm`/`QMsm` is the pure SM *class*, `QActive` is the threaded AO *on top of it*. Starcom's FOP/FARM is the `QHsm`-equivalent but written in **plain portable C++ with no QP dependency**.) This part is mandatory — it's what makes the library universal and table-test-able with no hardware/framework.
     - ✅ **Adapter (optional): an AO wrapper is a perfectly legitimate, offered adapter** — e.g. `starcom-ao-qp` — that *hosts and drives* the core HSM for consumers who want the protocol delivered as an Active Object (RC is exactly such a consumer). This is the **direct parallel to the PHY tiers**: just as best-effort-PHY is an optional adapter, an AO wrapper is an optional integration adapter.
     - ❌ **The only thing actually forbidden:** AO-ness reaching *into* the core such that the FSM can **only** be used by instantiating it as a framework Active Object. The test (user's phrasing): **an AO adapter is fine as long as the core HSM is still usable standalone without it.** If the protocol logic can't be exercised except *as* a QActive, universality is broken.

   Net: **don't rule AOs out — offer one as an optional adapter; keep the core a standalone-usable portable HSM.** Compliance-neutral structurally (§2.3); load-bearing for universality + testability. Cross-refs: §2.1/§2.2 (adapter-outside-core pattern, sans-I/O), §2.4 D-2 (no in-core virtual dispatch), the comparison doc's "thin AO wrapper only; core takes no QP dependency."

### 2.6 Documentation rework finding (answers "what docs need re-working")

Audit of the five existing Starcom docs against §0:

| Doc | Current framing | Universal-frame issue |
|---|---|---|
| `CLAUDE_STARCOM_LIBRARY_DEVELOPMENT_RESEARCH.md` | "taught **embedded-first** (because that is what binds Starcom)" | **Most RC-colored.** Reads as if the library *is* the embedded/RC use. Embedded is *a* target, not the binding center. |
| `STARCOM_LIBRARY_DEVELOPMENT_RESEARCH_GROK.md` | "standalone, reusable... **MCU-first** while remaining generally usable" | "MCU-first" subtly centers the embedded case; "generally usable" is vague about the actual audience. |
| `STARCOM_CCSDS_LIBRARY_RESEARCH.md` (Grok) | "standalone... cleanly integrable as a **dedicated Active Object**" | Says standalone, but the only concrete consumer named is RC-as-an-AO. Doesn't name cubesats / ground / the broad market. |
| `CLAUDE_STARCOM_CCSDS_LIBRARY_RESEARCH.md` | "Independent External Research" | No universal-scope statement at all (it's framed as a research method, not a product scope). |
| `STARCOM_RESEARCH_COMPARISON.md` | comparison record | Reasoning inherits the RC-colored frame in places (esp. the D-4 "primary" call and the P10 justification). |

**Disposition (user direction 2026-06-17): verdict-doc-only.** The research docs are **not edited** — they remain historical research. **This document's §0 is the canonical scope statement and supersedes their framing going forward.** Any future agent (Grok included) may align its *own* research doc's header to §0 if it chooses, but no agent rewrites another's. The comparison doc's D-4 "primary" wording is **corrected here** (§2.4), not edited there.

### 2.7 State-machine / framework decision — RESOLVED toward plain table-driven C++ (2026-06-17)

Run by Claude as a focused research pass (5 agents, primary-source-verified against the Blue Book PDFs + open-source implementation survey; HIGH confidence). This **resolves the "AO vs HSM vs QP" contention for the protocol core.**

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

### 2.8 Design note — function-generated CCSDS test telemetry (deterministic testing before real data)

*(User-parked idea, 2026-06-17. Side note, not tied to a council round; recorded here for permanence.)*

**The idea:** CCSDS test telemetry/command streams can be **produced by a generator function** — synthetic, fully-specified, deterministic — rather than requiring real sensors, real RF, or recorded captures. Because every CCSDS artifact (PLTU, USLP/V-3 frames, CLCW/PLCW, idle/acquisition sequences, coded symbol streams) is exactly defined by the Blue Books, a generator can emit **known-good** streams (valid frames with chosen content, sequence numbers, VC assignments) and **known-bad** streams (truncation, CRC corruption, sequence gaps, lockout-inducing patterns) on demand, with the *expected library response known in advance.*

**Why it matters:**
- **Decouples library testing from hardware entirely.** The whole stack (framing, coding, COP-1/COP-P state machines) is validated against generated input long before any IMU/baro/GPS/radio exists in the loop — and the validation is **reproducible bit-for-bit**, unlike sensor/RF data.
- **Composes naturally with the passive sans-I/O core (§2.7).** The core takes bytes/events in; a generator just *is* a source of those bytes/events. No mocking, no hardware shims.
- **Layers cleanly onto the existing test strategy:** static golden vectors (one known frame) → *generated* vectors (parametric families of frames) → property-based testing (generate-then-assert-invariants) → coverage-guided fuzzing (generate malformed input, assert no crash) → soak/stress (generate long streams). The generator is the common engine under several of these.
- **Reaches edge cases real sensors can't easily produce** — sequence-number wraparound, window-full, lockout, retransmission storms, idle-pattern boundaries.

**Status:** a *test-infrastructure deliverable* for the dev plan (a `starcom-testgen` / golden-vector generator), parallel to the code-to-state-table legend (§2.7). Not a core-library feature; a development/verification asset. Carry into the testing section of the implementation plan.

---

## Round 3 — Claude council (2026-06-18/19): USLP-as-spine + adversarial re-grounding on the flight blocker

**Run by:** Claude (Opus 4.8, Code) — 2026-06-18/19, one session, two linked council passes plus the conclusions that stuck. Personas per `COUNCIL_PROCESS.md`: JPL Avionics Lead, ArduPilot Core Contributor, Embedded Systems Professor, Cubesat Startup Engineer.

### 3.1 USLP is the unifying spine (refines Round-2 D-4)

Clarified this session: **USLP (732.1-B) defines no physical layer of its own — it is PHY-agnostic by design — and normatively carries BOTH COP-1 and COP-P** on different Virtual Channels simultaneously (per-frame Sequence-Controlled vs Expedited via the Bypass/Sequence-Control flag). Two consequences:
- It maps directly onto RC's two real cases *at once*: an **always-on reliable channel** (Sequence-Controlled / COP-1-style — drone↔GCS command+telemetry) and an **intermittent re-acquisition channel** (Expedited / COP-P-style — spinning rocket tracked from a drone). One stack, both at once.
- Because USLP has no PHY of its own, choosing it **dissolves the "are we claiming PHY compliance?" awkwardness** — there is no "USLP PHY" to be non-compliant with; the PHY tier is simply a separate, honestly-labeled adapter (§2.1).

**What "Starcom core" was circling *is* USLP** — USLP is the CCSDS committee's engineered answer to "unify these link types in one data-link." (Starcom is not *only* USLP: it's USLP framing **+** the COP engines USLP references but doesn't implement **+** codecs **+** optional session/MAC **+** optional PHY adapters. USLP is the spine, not the whole skeleton.)

**Layer correction (recorded because the layering is the point):** COP-1/COP-P are optional **ARQ procedures** (reliability) at the *data-link* layer — they are **NOT** physical layers. The optional *physical* layer is the PHY-tier adapter (none / best-effort 1–99% / full). Both are optional and pluggable, but at different layers; conflating them is exactly the delineation error the layered model prevents. The four clean axes: **USLP framing · COP-1/COP-P ARQ (per VC) · Prox-1 §6 session/MAC · PHY tier.**

**Decision — USLP is the strategic primary frame; V-3/PLTU is a permanent second codec** (this **revises** Round-2 D-4's "implement V-3/PLTU first / neither is primary"). Architecturally the design orients around USLP as the unifying spine. *But see §3.3 — the MVP feature-sequencing may still build a V-3/PLTU codec first because it is the smaller path to a working link; "USLP-primary (architecture)" and "V-3-first (MVP sequencing)" are not in conflict.* V-3/PLTU stays permanent for native Proximity-1 interop (real Prox-1 assets speak V-3, not USLP).

### 3.2 Proximity-1 is a sibling protocol family, not a layer on top of CCSDS

Recorded to settle a recurring question. Proximity-1 (211.x) is a **distinct, complete CCSDS protocol family** purpose-built for *short-range* proximity links (orbiter↔surface-asset) — its own PHY (211.1), coding/sync (211.2), and data-link (211.0). It is **not** a layer on top of, or a variant of, TM/TC/AOS; it is a peer. ("Proximity" = the short hop, *not* deep-space long-haul.) It fits RC because of the **link character** — short-range, intermittent, often half-duplex, acquire-after-silence — the direct analog of orbiter↔lander, *not* because of distance scale. Only the **PHY** is band-locked (UHF Mars / S-band; 915 MHz is not a Prox-1 band); the **data-link and coding layers are band- and PHY-agnostic**, so no pivot is needed at the layers we build.

### 3.3 Adversarial re-grounding on the flight blocker (the round that matters)

A first adversarial pass was run on a **false premise** — "RC already has working MAVLink telemetry" — and was **discarded.** The correct premise: **radio telemetry is broken (half-duplex), it is a flight blocker, and band-aids have cost more than a principled fix would.** Re-run under that premise:

- **The "why build it / it's a distraction" challenge is void** — Starcom is *on RC's critical path*, and the band-aid alternative has been empirically tried and lost. (ArduPilot withdrew it.)
- **CCSDS/Prox-1 is unusually well-matched, not overkill** — the protocol's entire reason for existing is the half-duplex, intermittent, controlled-turnaround link that is RC's exact failure mode. (Professor.)
- **"Doing it right" is the diagnostic instrument, not luxury rigor** — band-aids mask the cascade, so the real failure chain is *unknowable* until a clean foundation genuinely resolves issue N and exposes issue N+1. You cannot prove CCSDS does or doesn't work — or even find the real bug — without doing it right. (User's point, council-endorsed.)
- **Architecturally complete, feature-incremental** — the surviving adversarial finding. The *foundation/architecture* (layering, sans-I/O, the four axes, the seams) is **universal-capable and built right from day one** — NOT deferred; compromising it causes rework *and* defeats the diagnostic purpose. The *feature set* is **sequenced blocker-first.** Universality is a property of the *foundation* (kept from the start), not end-stage polish. This is "architecturally complete, feature-incremental" / a walking skeleton — the **opposite** of "minimal monolith, generalize later" (which incurs rework). *(This corrects an earlier loose "defer universality" framing — only the **features** sequence; the **foundation** does not.)*
- **MVP first cut (likely):** minimal frame (likely **V-3/PLTU** — smallest path to flight) + **COP-P** (expedited/intermittent ARQ — 2 states + stateless FARM-P) + **minimal half-duplex turnaround.** Replace the two band-aids — the retry/ACK STOP-GAP (≈ ad-hoc COP) and the RadioScheduler TX-window hacks (≈ ad-hoc MAC turnaround) — with their principled CCSDS equivalents, and *nothing else* in the first cut.
- **Pull the half-duplex turnaround FORWARD** from the deferred §6 pile (revises §2.7's open question): the *broken thing* is half-duplex turnaround, so its minimal turnaround discipline is **MVP-critical**, even though the full ~30-state session lifecycle (hailing / establish / resync / reconnect / terminate) stays deferred. The turnaround may be driven by RC's *existing* RadioScheduler rather than a fresh MAC machine — decide at planning.
- **Verify the turnaround fix INDEPENDENTLY of the reliability layer** (ArduPilot's hard check): if the underlying turnaround timing is still wrong, COP just retransmits into a broken window forever — looks like progress, fixes nothing (LL-36 shape). The half-duplex turnaround needs its **own positive-control bench signal**, separate from "COP delivered the frame."
- **Rigor proportionate** (JPL's point, user-endorsed): the *blocker-fix* gets flight-grade verification; *library polish* (full USLP surface, conformance subsystem, universality) is proportionate to its hobby/learning tier — don't cargo-cult flight-program V&V *machinery* without flight-program V&V *organization* (the LL-36 "rigor without organization" risk).
- **Defer to post-unblock:** full USLP surface, COP-1, the conformance subsystem, universality polish, V-3↔USLP dual-codec breadth, the full §6 session machine.

**Net:** right architecture (the council could not break it — meaningful confirmation, not flattery). The only real risk is *scope discipline*: build the **blocker-fix MVP on the universal foundation**, sequence features by the blocker, verify the half-duplex root cause independently, and don't celebrate before the link genuinely works.

---

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

---

*Rounds 2–3 logged by Claude (Opus 4.8, Code), 2026-06-18/19. Parallel to a Grok-run council on the same material. Future entries: append `## Round N` (councils) or dated sections/notes (research passes, design notes, conclusions); do not rewrite prior entries. §0 (Canonical Scope) is the governing framing — amend it only with explicit user direction.*
