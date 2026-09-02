# Working in `starcom/` — Dos and Don'ts

**Audience:** Nathan and any agent touching this folder.
**Read this first** before adding code, docs, or CMake here.

---

## What this folder is

`starcom/` is a **self-contained comms stack** incubating inside the Rocket-Chip monorepo. It will eventually become its own repository. The **core** is a portable library (`starcom::ccsds`). First-party **ports** (host bearers, reference radio adapters against generic I/O) live here too. Rocket-Chip is the **first consumer and integration driver** — not the owner, not the design boundary.

Identity: a CCSDS data-link *stack* whose “universal” means **matching PICS**, not every radio. Usable by cubesats, ground stations, HABs, and RC alike when the peer ticks the same book/option. See `DESIGN.md` notes 2026-08-21 and 2026-08-29.


## Vocabulary

Blue Book names. Picture: `SAD.md`. Full list with section cites: [`GLOSSARY.md`](GLOSSARY.md). Short table for agents:

| Term | Means |
|------|--------|
| **PLTU** | Proximity Link Transmission Unit (211.2). The coding-and-sync wrapper: ASM + one transfer frame + CRC-32. |
| **Repeater** | Same-link regenerative forward of a **PLTU** (check envelope, same octets out, no payload decode). Wanted early on RC (RP2350 + LoRa). Not a Prox-1 session and not a second long-haul link. Bent-pipe is IVP 7; buffered (133.0 §2.4 store-and-forward, caller-owned queue) is IVP 12. |
| **USLP** | Unified Space Data Link Protocol (732.1). Its on-the-wire frame is Version-4. On Prox-1 it sits *in* a PLTU, in lieu of Version-3, never inside the V-3 data field. |
| **Space Packet** | CCSDS 133.0-B-2. The usual SDU inside a transfer frame: 6-octet header + user data. Not a Starcom product name. Starcom codecs the header; the user field is the app. |
| **PUS** | ECSS-E-ST-70-41C Packet Utilization Standard (**ESA**, not a CCSDS Blue Book). Service type/subtype **inside** Space Packet user data (e.g. ST[20] get/set onboard parameters). Optional later stack module; not in IVP 0–25. |
| **CFDP** | CCSDS 727.0 File Delivery Protocol. Post-mission **data offload**: files with a checksum, in Space Packet user data. Owner-wanted; not IVP 0–25. Not SDLS. |
| **cFS / F´** | NASA-world **frameworks**, not CCSDS books. They also ride Space Packets: cFS (GSFC) puts cFE command codes in the user field; F´ (JPL) uses typed PRM_SET/SAVE. US missions often leave packet *contents* to the FSW; ESA standardized those contents as PUS. Neither belongs in `starcom::ccsds` as RC settings. |


---

## Dos

### Boundary and dependencies

- **Do** keep the names straight: **Starcom** = stack (this tree), **core** = sans-I/O library (`src/ccsds/`), **port** = first-party adapter (may do I/O), **integration** = Rocket-Chip. “No hardware / sans-I/O” applies to the **core**, not the whole stack. Dual-radio, half-duplex, LoRa, ELRS boards live in ports or in RC — never as assumptions inside codecs.
- **Do** keep dependency direction one-way: **Rocket-Chip → Starcom**. Starcom must never `#include` Rocket-Chip headers, AO/QP types, board pins, mission profiles, or firmware drivers.
- **Do** put portable protocol logic in `include/starcom/` and `src/ccsds/`.
- **Do** put platform-specific glue in `adapters/` (host UDP, generic radio port, optional AO wrapper) — never in the core.
- **Do** structure CMake so Starcom can build and test **independently** of the Pico firmware target (own `ctest` suite, host-first).
- **Do** use `starcom::` / `starcom::ccsds::` namespaces. RC integration code lives outside this tree.

### Architecture (settled direction — see `design_record_claude.md` §0)

- **Do** adopt **sans-I/O** for the core: bytes and timeouts in, events and bytes out. No sockets, no SPI, no GPIO in `src/ccsds/`.
- **Do** implement protocol state machines as **plain portable C++** (table-testable, no framework dependency). An optional AO adapter may wrap them; the core must work without it.
- **Do** declare conformance **per component**, honestly — never claim more CCSDS compliance than the code delivers.
- **Do** use three claim levels: **0** (not implemented / out of scope), **Best effort** (non-conformant approximation — still not a PICS tick), **Full** (PICS-claimable for that book/option). Code `PhyTier` maps `none` / `best_effort` / `compliant` onto those. Full 211.1 (`PhyTier::compliant`) is not offered.
- **Do** treat PHY as **optional adapters** outside the core (0 / Best effort / Full). Best effort is still non-compliant.
- **Do** pursue Best effort only when it advances product performance or features, especially if the work later transfers to Full. Do not add it just to have it. Radio Best-effort on Rocket-Chip often stays RC-specific (pins, AO); keep those out of `starcom::ccsds`.

### Documentation

- **Do** treat the CCSDS Blue Books (and other named primary sources) as the authority. SAD / ICD / DESIGN field tables are working copies. Open the cited book first; if they disagree, the book wins.
- **Do** write public-facing Starcom docs as what the system **is**. Corrections for a hung-up mistake belong here or on [`../AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md), not as a “what this isn’t” banner on README / SAD / ICD.
- **Do** treat files in `docs/research/`, `comparison.md`, and `design_record_claude.md` as **historical** — written before `starcom/` existed. They were relocated **without content edits**; internal links still cite `docs/research/STARCOM_*`. Use `docs/README.md` mapping; do not rewrite cross-references in those files.
- **Do** read in this order when onboarding:
  1. this file (agents); `USER_GUIDE.md` (consumers)
  2. `DESIGN.md` (locks)
  3. `SAD.md` (map + codec field maps), `ICD.md` (handshake), `CONFORMANCE.md` (claims)
  4. `STATUS.md` (phase), `IVP.md` (order of proof; Closed log when gates pass), `TESTING.md` (how we write and run host tests)
  5. [`../AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md) — Starcom-only open flags
  6. `comparison.md` / research pair as needed (historical)
- **Do** append to research/comparison/design-record docs — **do not silently rewrite** another agent's entries (`CROSS_AGENT_REVIEW.md`).

### Code quality (core targets strictest plausible adopter)

In the core: `std::span`, `expected`/`Result`, `enum class`, and `constexpr` are in. Exceptions, RTTI, and heap-after-init are out (`-fno-exceptions -fno-rtti`; no `new` on codec paths). Tests of the core may use exceptions. Rocket-Chip [`standards/CODING_STANDARDS.md`](../../standards/CODING_STANDARDS.md) **applies** to Starcom the same as firmware. The language bar above is additional, not a substitute.

**Identifiers follow the house scheme. No Starcom accepted-deviation row.** NASA F´ ([style guidelines](https://github.com/nasa/fprime/wiki/F%C2%B4-Style-Guidelines)) uses lower camel case for functions and locals, PascalCase for types; cFS prefers CamelCase for terms. Neither keeps Blue Book prose as C++ snake_case — book names stay in comments / CONFORMANCE. House already supersedes JSF 45/51/52 with camelBack / `k`CamelCase / CamelCase types. Public verbs are camelBack (`decodePltu`). Remaining: `Error` enumerators (`uslp_truncated` → `uslpTruncated`) and JSF 151 named field masks. `#pragma once` is the existing project-wide exception, not a Starcom row.

- **Do** write host-side unit tests **before** hardware adapters. Procedure: [`TESTING.md`](TESTING.md). Golden vectors and table-driven state-machine tests are the first wins.
- **Do** keep MCU automatic storage tiny. Pico Core 0 stack is 4 KiB. Do not value-init `CoppEndpoint` / `Cop1Endpoint` (`memset` in place). Sent copies are `kFopPSentCap` / `kFop1SentCap`, not a 256-FSN table. Do not put `kTransferFrameMax` (2048) arrays on the stack — caller span or file-scope scratch. GNU `-Wstack-usage=1024` is on the library (`cmake/CompilerWarnings.cmake`).

### Pedagogy (standing requirement per `design_record_claude.md` §0.6)

- **Do** explain *why* behind decisions, not just *what*.
- **Do** teach the general library-design concept (sans-I/O, adapter pattern, conformance honesty) when making structural choices.
- **Do** flag trade-offs explicitly so Nathan can learn from them.

### Extraction readiness

- **Do** keep this tree buildable as if it were already standalone (own README, LICENSE, CMake, tests, docs).
- **Do** avoid RC-only assumptions in filenames, comments, or public API naming.

---

## Don'ts

### Boundary violations (most common failure mode)

- **Don't** add Rocket-Chip concepts to the core: `rocketchip::`, `AO_*`, `RadioScheduler`, `Mission Profile`, `QF_*`, board headers, GPIO pin constants.
- **Don't** move or refactor RC's pre-Starcom `telemetry_encoder` into the core as-is — research explicitly says it is **not** the design base. Replace it at IVP increment 22 via adapter + COP. Do not mint a Starcom stop-gap or temporary retry layer.
- **Don't** let Rocket-Chip's root `CMakeLists.txt` become the only way to build or test Starcom.
- **Don't** create reverse dependencies (Starcom linking against firmware targets).

### Architecture mistakes

- **Don't** put I/O (virtual `IPhysicalLayer` with real hardware calls) **inside** the sans-I/O core. Adapters implement transport; core consumes/produces bytes.
- **Don't** make the protocol FSM **only** usable as a QP Active Object. AO wrapper = optional adapter.
- **Don't** hard-code COP-1 managed parameters (T1, window sizes) — they must be configurable.
- **Don't** claim 211.1-B-4 PHY compliance on SX1276/LoRa paths. Best-effort PHY must say so loudly.
- **Don't** treat last night’s “repeater is increment 0+1 codec lock” as current. Bent-pipe is IVP 7 (`repeatPltu`). Buffered / dedup is IVP 12.
- **Don't** describe a future repeater as an orbiter/gateway (Prox-1 hop + a different Earth link). If we build one, it is range-extend of a PLTU, not a second link. Don't run COP on that path and don't decode the Space Packet just to forward.

### Documentation mistakes

- **Don't** scatter new Starcom library docs under `docs/research/` at repo root — they belong in `starcom/docs/`.
- **Don't** edit `DESIGN.md` substantively until the condensation session merges the six research artifacts into one canonical record.
- **Don't** delete historical comparison entries when facts change — append `Status:` lines per `comparison.md` convention.
- **Don't** mix RC stage plans (board IVP, Stage T, AO architecture) into `starcom/docs/` — those stay in repo-root `docs/`. Starcom's IVP is `docs/IVP.md` in this tree. RC-specific migration notes may eventually live in `starcom/docs/integration/` if needed.
- **Don't** mint a repo-root `CHANGELOG.md` entry for work that stayed entirely under `starcom/`. Starcom [`CHANGELOG.md`](../CHANGELOG.md) is **major pushes only** (first tagged cut, public extract, supported-API change) — not every codec sitting. Root changelog is for firmware/integration sittings.

### Process mistakes

- **Don't** implement large features against stale `comparison.md` D-1…D-5 text. Living locks are SAD / ICD / CONFORMANCE / STATUS. `comparison.md` is historical; append Status lines, do not rewrite entries.
- **Don't** lock Prox-1 C&S codes against the wrong 131.0-B issue. 211.2-B-3 [2] is 131.0-B-3; 131.0-B-5 is current TM-only. See `DESIGN.md` pin.

---

## Tracking documents — what goes in `starcom/` vs. repo root

Starcom gets its **own** tracking files so it can extract to a standalone repo without archaeology. A **major push** of Starcom-only work gets one entry in [`CHANGELOG.md`](../CHANGELOG.md) here, **not** a second row on repo-root `CHANGELOG.md`. Ordinary sittings skip the library changelog. Root wrap/push rules in `docs/agents/SESSION_CHECKLIST.md` still apply when the sitting also touched firmware or RC docs.

### In `starcom/` (library-owned)

| File | Purpose | Status |
|---|---|---|
| [`CHANGELOG.md`](../CHANGELOG.md) | Library-scoped changes. **Major pushes only** (see its scope note and `TESTING.md`). | Live |
| [`VERSIONING.md`](../VERSIONING.md) | SemVer + `STARCOM_VERSION` SSOT. Generated `version.hpp`. | Live 2026-08-27 |
| [`CONTRIBUTING.md`](../CONTRIBUTING.md) | How to build/test, coding standard for core, DCO, PR expectations. | Interim rules are this file + SAD/ICD until Phase 0 |
| [`LICENSE`](../LICENSE) | Library license (research leans Apache-2.0). | Placeholder until extraction/release |
| [`docs/comparison.md`](comparison.md) | Historical cross-agent comparison log (D-1…D-5). Append Status lines; do not rewrite. Living locks are SAD / STATUS / CONFORMANCE. | Historical |
| [`docs/design_record_claude.md`](design_record_claude.md) | Scope, council rounds, standing architecture decisions. | Historical — DESIGN.md is the freeze |
| [`docs/DESIGN.md`](DESIGN.md) | Future **single** condensed design record (condensation session). | DONE 2026-06-22 [x] - canonical on branch; manifests+SCRATCH prove no loss; historical untouched. |
| [`docs/SAD.md`](SAD.md) | Architecture map (views + on-the-wire figure + increment 0+1 field maps). | Draft 2026-08-25; field maps 2026-08-27 |
| [`docs/ICD.md`](ICD.md) | Core handshake: principles, named verbs. Signatures land with the first codec. | Draft 2026-08-25 |
| [`docs/CONFORMANCE.md`](CONFORMANCE.md) | In-scope / deferred / out-of-scope claim table. | Draft 2026-08-25 |
| [`docs/IVP.md`](IVP.md) | Integration/verification plan (IEEE 1012 + ECSS methods). Closed log IDs when gates pass. | Living 2026-08-25 |
| [`docs/GLOSSARY.md`](GLOSSARY.md) | Terms and Blue Book section cites. README holds the short list. | Living 2026-08-27 |
| [`STATUS.md`](../STATUS.md) | Starcom phase, blockers, next step. Lighter than RC `PROJECT_STATUS.md`. | Live sketch 2026-08-25 |
| [`AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md) | Starcom-only active flags (graphify, shelf holes, sittings). Not a second RC board. | Live 2026-08-27 |

### At Rocket-Chip repo root (firmware-owned — do not copy into `starcom/`)

| File | Starcom relationship |
|---|---|
| Root [`CHANGELOG.md`](../../CHANGELOG.md) | Rocket-Chip firmware and integration history. Not for Starcom-only sittings. |
| [`AGENT_WHITEBOARD.md`](../../AGENT_WHITEBOARD.md) | RC firmware flags. Starcom-only items belong on [`starcom/AGENT_WHITEBOARD.md`](../AGENT_WHITEBOARD.md); the root board keeps a **pointer row**, not a copy. |
| [`docs/PROJECT_STATUS.md`](../../docs/PROJECT_STATUS.md) | RC phase/blockers. Starcom progress does not belong here except "RC blocked on Starcom MVP". |
| [`docs/IVP.md`](../../docs/IVP.md) | RC verification plan (board bring-up checklist). Starcom’s plan is [`docs/IVP.md`](IVP.md), not a clone. |
| [`docs/decisions/*`](../../docs/decisions/) | RC architectural decisions (STOP-GAP retry map, Stage T, etc.). Not Starcom library decisions. |

### Not needed yet (add at Phase 0 / extraction)

- **`CODEOWNERS`** — when Starcom has its own GitHub repo or monorepo path owners
- **Issue/PR templates** — when external contributors are expected; comparison doc suggests frame hexdumps in PR template
- **Separate Starcom `PROJECT_STATUS.md`** — overkill until implementation; `STATUS.md` is enough

The old “no `starcom/AGENT_WHITEBOARD.md`” rule buried Starcom flags on the RC board (firmware leftover sittings). One coordination surface **per concern**: Starcom flags here, RC flags on the root board. `STATUS.md` is still phase, not a whiteboard.

---

## What lives where

| Location | Belongs here |
|---|---|
| `starcom/include/`, `starcom/src/ccsds/` | Portable CCSDS protocol core |
| `starcom/adapters/host/` | Desktop transports (UDP, file replay, SDR bridge) |
| `starcom/adapters/rp2350/` | Generic SPI/GPIO radio port. Board pins and AO stay in Rocket-Chip. |
| `starcom/tests/` | Host-side unit, property, fuzz tests |
| `starcom/docs/` | Library design, research, comparison |
| `starcom/docs/integration/` | What RC and other consumers can call today vs later |
| Rocket-Chip `src/telemetry/telemetry_encoder.*` | Pre-Starcom RC firmware — stays until IVP 22 replaces it with COP |
| Rocket-Chip `src/active_objects/ao_telemetry.*` | RC integration — not Starcom core |
| Rocket-Chip `docs/decisions/CURRENT_COMMAND_RETRY_ACK_*` | RC pre-Starcom retry map — input to IVP 22, not library docs |

---

## Quick checklist (agents)

Before opening a PR that touches `starcom/`:

- [ ] Core changes compile and test on **host** without Pico SDK?
- [ ] No Rocket-Chip headers included from `starcom/src/` or `starcom/include/`?
- [ ] Public API uses `starcom::` namespace?
- [ ] Conformance claims match what was actually implemented?
- [ ] Another agent's doc entries appended, not silently rewritten?

---

## When in doubt

1. Re-read `design_record_claude.md` §0.
2. Ask: *"Would a cubesat developer understand this without knowing Rocket-Chip?"*
3. If no → move it to `adapters/` or back to Rocket-Chip `src/`.