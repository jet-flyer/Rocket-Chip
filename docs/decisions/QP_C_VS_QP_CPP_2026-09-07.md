# QP/C vs QP/C++ — Stay on QP/C

**Status:** Accepted  
**Date:** 2026-09-07  
**Author:** Grok 4.6 (Build CLI)  
**Closes:** `AGENT_WHITEBOARD.md` medium row “QP/C vs QP/C++ framework evaluation” (opened 2026-06-23); L2-P5 **WN-052** framework-block  

**Applies to:** vendored QP/C 8.1.3 (`lib/qep/`), all Core-0 Active Objects, `include/rocketchip/ao_signals.h` (`evt_cast<E>` / CAST-2), Flight Director QHsm.

**Does not change:** firmware behavior, vendor tree, AO APIs, event catalog numbering.

---

## Decision

Stay on **QP/C** (C11 framework, called from C++ TUs). Do **not** migrate to QP/C++.

The 2026-03-10 council adopted UML statecharts + QP/C (QEP, then QF/QV). That review compared QP/C vs STARS vs hand-rolled QEP, not C vs C++. The C edition was vendored as 8.1.3 without a recorded sub-choice. User direction 2026-06-23: an undocumented decision of that size is a red flag — weigh both editions and back-fill a decision either way.

This document is that record. The implicit C choice was correct.

---

## What the 2026-06-23 eval thought C++ uniquely fixed

The whiteboard claim, made while locking JSF-182 / CAST-2:

> QP/C++ inheritance downcasts (`static_cast`, JSF-178-compliant) eliminate the JSF-182 residual outright.

That was the only standards-level argument for switching. It does not hold against QP/C++ 8.x or against JSF-178 as this project triages it.

The other eval axes (event ergonomics, `nullptr`, footprint, FD callbacks, catalog redesign) are already handled on QP/C, or are the same problem in both editions.

---

## Why QP/C++ does not retire CAST-2

Quantum Leaps’ own C++ edition still downcasts `QEvt const*` after a signal check. QP/C++ 8.1.5 `include/qp.hpp`:

```cpp
#define Q_EVT_CAST(subclass_) (static_cast<subclass_ const *>(e))
#define Q_STATE_CAST(handler_) (reinterpret_cast<QP::QStateHandler>(handler_))
```

| Claim (2026-06-23) | Primary source | Actual |
|---|---|---|
| C++ downcast is JSF-178-compliant | JSF-178 as triaged in `docs/audits/RULE_VERIFIABILITY_TRIAGE.md`: “downcast only via **virtual/visitor**”; static-cast downcast is the *shell* the rule flags | `Q_EVT_CAST` *is* that shell |
| C++ retires JSF-182 | JSF-182 bans casting to/from pointers; two exceptions (`void*`→`T*` in memory management; literal HW address). Neither is `QEvt*`→derived | `static_cast<Derived const*>(e)` is still a pointer cast |
| Inheritance makes it type-safe without a cast | [QP::QEvt](https://www.state-machine.com/qpcpp/class_q_p_1_1_q_evt.html): subclasses must be standard-layout, trivially copyable, **no virtuals, no vptr** | `dynamic_cast` / visitor are framework-forbidden |
| RTTI could police the downcast | Pico SDK `pico_cxx_options`: **`-fno-rtti -fno-exceptions`** (already on this tree) | RTTI downcasts are off on target |

Samek’s 2019-01-16 forum answer: check `e->sig`, then downcast; `QEvt` is not a virtual base and must not grow a vtable (doubles every event). That is exactly `evt_cast<E>` after the AO `switch (e->sig)` — already centralized, with `static_assert(std::is_standard_layout<E>::value)` and `static_assert(offsetof(E, super) == 0)`.

Switching editions changes the spelling (`reinterpret_cast` of first-member composition → `static_cast` of inheritance). It does **not** remove the deviation. Council 2026-06-23 already rejected `memcpy` as a CAST-2 alternative (256-byte `RadioRxEvt` copy, zero functional gain).

**CAST-2 stays Accepted.** Containment is `evt_cast<E>`. There is no remaining remediation path.

---

## Why C++ is not a better HSM/AO shape here

- **State handlers stay C-shaped in QP/C++ 4.x+.** Quantum Leaps dropped true member-function handlers because pointers-to-member were non-portable and slow ([AN_QP_Inheriting](https://www.state-machine.com/doc/AN_QP_Inheriting.pdf)). Handlers are `static` methods with an explicit `me` pointer — the same calling convention we already use.
- **FD / `action_executor` P10-9 rider is not an edition issue.** `fd_effect_*` are already direct calls. Remaining C-ness is QEP `QStateHandler`, which exists in both editions.
- **C++ would add virtuals this tree refuses.** `QP::QAsm` / `QHsm` expose virtual `init` / `dispatch` / `isIn`. Application AOs inherit that vtable. `CMakeLists.txt` currently states authored code has **no virtual functions (QP/C, not OO)** — the reason `-Wnon-virtual-dtor` is clean. A C++ migration would put vtables on every AO and fight P10-9 (vtables are function pointers).
- **Feature tables are a wash** for what we run: 9 AOs, QV cooperative, QHsm, pub-sub, time events, static events (`QF_MAX_EPOOL` wired, pool not allocated). Official [QP family comparison](https://www.state-machine.com/products/qp): C11 vs C++17, same AO/HSM/kernel/QM surface. We already compile **C++20** around the C core.
- **JPL shape matches C.** Perseverance used QEP in C, not QP/C++. That is what the 2026-03-10 council actually adopted.

Migration blast radius if we switched anyway: 9 AOs + Flight Director `QHsm`, every event in `ao_signals.h`, `lib/qep` + Pico `bsp_qv.c` / `qp_port.h`, CMake, host tests, full vehicle+station HW re-verify. Flight-critical churn for a cosmetic downcast spelling.

---

## How the perceived issues are satisfied on QP/C

| Eval axis (whiteboard 2026-06-23) | On QP/C (this tree) |
|---|---|
| Undocumented C vs C++ sub-choice | This decision. |
| Event ergonomics / type-safety | `RcSignal` enum + typed `{ QEvt super; ... }` payloads + `evt_cast<E>` (one audited site). |
| `(void *)0` vs `nullptr` | Already remediating (CHANGELOG `2026-06-24-001`). |
| JSF-182 CAST-2 | Contained in `evt_cast`. C++ does not retire it. **No remediation path.** |
| JSF-178 | C++ `static_cast` downcast is the pattern the rule flags; QEvt forbids the virtual/visitor the rule wants. Not a C++ win. |
| Framework footprint / RTTI / exceptions | C QEP/QF/QV; Pico already `-fno-rtti -fno-exceptions`. C++ edition would add QHsm vtables. |
| FD C-HSM callbacks | Direct `fd_effect_*`; not blocked on edition. |
| WN-052 “wait for QP/QF / QP/C++” | Framework-block **lifted**. Catalog/comment polish (`WN-053`) is ordinary later cleanup, not a medium architecture item and not this sitting. |
| Migration cost | Avoided. |

Nothing on that list requires a framework swap. The issues that looked C++-only are either already contained in C or unfixable in both editions.

---

## Revisit only if

- We adopt QM autocoder **C++** output as the Flight Director SSOT, or
- We buy a **SafeQP/C++** certification kit for a real SIL/ASIL claim.

Neither is planned. STARS/QM were already deferred (statechart too small; STARS commercial). SafeQP is commercial-only.

---

## Sources

- Quantum Leaps QP family / feature comparison: https://www.state-machine.com/products/qp  
- QP/C++ 8.1.5 `QEvt` (no virtuals; standard-layout subclasses): https://www.state-machine.com/qpcpp/class_q_p_1_1_q_evt.html  
- QP/C++ `Q_EVT_CAST` / `Q_STATE_CAST`: `QuantumLeaps/qpcpp` `include/qp.hpp`  
- Samek, 2019-01-16, Q_EVT_CAST vs MISRA C++ `dynamic_cast`: https://sourceforge.net/p/qpc/discussion/668726/thread/8e6de830cb/  
- QP/C++ 4.x static handlers (`me` pointer): https://www.state-machine.com/doc/AN_QP_Inheriting.pdf  
- Pico SDK `-fno-rtti -fno-exceptions`: `pico-sdk/src/rp2_common/pico_cxx_options/CMakeLists.txt`  
- Council 2026-03-10: `docs/decisions/flight_director/council_state_machine_formalism.md`  
- CAST-2 (updated with this decision): `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`  
- JSF-178 triage: `docs/audits/RULE_VERIFIABILITY_TRIAGE.md`  
- Containment helper: `include/rocketchip/ao_signals.h` (`evt_cast`)

---

## Follow-through (this sitting)

Whiteboard medium row erased (IRL rule). CAST-2 itself is unchanged: it was already Accepted with `evt_cast<E>` as containment. This decision does not add, retire, or rewrite a standards deviation.
