# Working in `starcom/` — Dos and Don'ts

**Audience:** Nathan and any agent touching this folder.
**Read this first** before adding code, docs, or CMake here.

---

## What this folder is

`starcom/` is a **self-contained library project** incubating inside the Rocket-Chip monorepo. It will eventually become its own repository. Rocket-Chip is the **first consumer and integration driver** — not the owner, not the design boundary.

The library's identity: **universal CCSDS data-link** (`starcom::ccsds`), usable by cubesats, ground stations, HABs, and RC alike.

---

## Dos

### Boundary and dependencies

- **Do** keep dependency direction one-way: **Rocket-Chip → Starcom**. Starcom must never `#include` Rocket-Chip headers, AO/QP types, board pins, mission profiles, or firmware drivers.
- **Do** put portable protocol logic in `include/starcom/` and `src/ccsds/`.
- **Do** put platform-specific glue in `adapters/` (host UDP, RP2350/SX1276, optional AO wrapper) — never in the core.
- **Do** structure CMake so Starcom can build and test **independently** of the Pico firmware target (own `ctest` suite, host-first).
- **Do** use `starcom::` / `starcom::ccsds::` namespaces. RC integration code lives outside this tree.

### Architecture (settled direction — see `design_record_claude.md` §0)

- **Do** adopt **sans-I/O** for the core: bytes and timeouts in, events and bytes out. No sockets, no SPI, no GPIO in `src/ccsds/`.
- **Do** implement protocol state machines as **plain portable C++** (table-testable, no framework dependency). An optional AO adapter may wrap them; the core must work without it.
- **Do** declare conformance **per component**, honestly — never claim more CCSDS compliance than the code delivers.
- **Do** treat PHY as **optional adapters** outside the core (none / best-effort / full-compliant tiers).

### Documentation

- **Do** treat files in `docs/research/`, `comparison.md`, and `design_record_claude.md` as **historical** — written before `starcom/` existed. They were relocated **without content edits**; internal links still cite `docs/research/STARCOM_*`. Use `docs/README.md` mapping; do not rewrite cross-references in those files.
- **Do** read in this order when onboarding:
  1. `design_record_claude.md` §0 (canonical scope)
  2. `comparison.md` (settled facts + open forks D-1…D-5)
  3. Research pair as needed (`research/ccsds_domain_*.md`, `research/library_craft_*.md`)
  4. `DESIGN.md` once the condensation session produces it
- **Do** append to research/comparison/design-record docs — **do not silently rewrite** another agent's entries (`CROSS_AGENT_REVIEW.md`).

### Code quality (core targets strictest plausible adopter)

- **Do** write the core **exceptionless, no-RTTI, no-heap-after-init** — compatible with MISRA/JPL/JSF rigor even though RC's root `CODING_STANDARDS.md` governs firmware separately.
- **Do** use `Result` / `expected`-style returns for fallible operations in the core API.
- **Do** use `span` for frame boundaries — caller-owned buffers, zero-copy where possible.
- **Do** write host-side unit tests **before** hardware adapters. Golden vectors and table-driven state-machine tests are the first wins (see `library_craft_claude.md` §7 phased plan).

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
- **Don't** move or refactor RC's STOP-GAP `telemetry_encoder` into the core as-is — research explicitly says it is **not** the design base. Replace it later via adapter + new core APIs.
- **Don't** let Rocket-Chip's root `CMakeLists.txt` become the only way to build or test Starcom.
- **Don't** create reverse dependencies (Starcom linking against firmware targets).

### Architecture mistakes

- **Don't** put I/O (virtual `IPhysicalLayer` with real hardware calls) **inside** the sans-I/O core. Adapters implement transport; core consumes/produces bytes.
- **Don't** make the protocol FSM **only** usable as a QP Active Object. AO wrapper = optional adapter.
- **Don't** hard-code COP-1 managed parameters (T1, window sizes) — they must be configurable.
- **Don't** claim 211.1-B-4 PHY compliance on SX1276/LoRa paths. Best-effort PHY must say so loudly.

### Documentation mistakes

- **Don't** scatter new Starcom library docs under `docs/research/` at repo root — they belong in `starcom/docs/`.
- **Don't** edit `DESIGN.md` substantively until the condensation session merges the six research artifacts into one canonical record.
- **Don't** delete historical comparison entries when facts change — append `Status:` lines per `comparison.md` convention.
- **Don't** mix RC stage plans (IVP, Stage T, AO architecture) into `starcom/docs/` — those stay in repo-root `docs/`. RC-specific migration notes may eventually live in `starcom/docs/integration/` if needed.

### Process mistakes

- **Don't** implement large features without checking `comparison.md` open decisions (D-1…D-5) — some forks are not settled.
- **Don't** lock coding definitions against stale CCSDS standard issues — verify Blue Book currency before code locks types (see `AGENT_WHITEBOARD.md` 131.0-B audit).

---

## Tracking documents — what goes in `starcom/` vs. repo root

Starcom gets its **own** tracking files so it can extract to a standalone repo without archaeology. Changelog discipline follows repo-root agent guidance (`CHANGELOG.md` header, `docs/agents/SESSION_CHECKLIST.md`).

### In `starcom/` (library-owned)

| File | Purpose | Status |
|---|---|---|
| [`CHANGELOG.md`](../CHANGELOG.md) | Library-scoped changes only (see its scope note). | Live |
| [`VERSIONING.md`](../VERSIONING.md) | SemVer rules, supported API surface (`include/starcom/` only), breaking-change policy. | Placeholder until first tagged release |
| [`CONTRIBUTING.md`](../CONTRIBUTING.md) | How to build/test, coding standard for core, DCO, PR expectations. | Placeholder — interim rules are this file |
| [`LICENSE`](../LICENSE) | Library license (research leans Apache-2.0). | Placeholder until extraction/release |
| [`docs/comparison.md`](comparison.md) | **Open architectural forks** (D-1…D-5), cross-agent comparison log. Append-only. | Live — do not duplicate elsewhere |
| [`docs/design_record_claude.md`](design_record_claude.md) | Scope, council rounds, standing architecture decisions. | Live — governs §0 until `DESIGN.md` exists |
| [`docs/DESIGN.md`](DESIGN.md) | Future **single** condensed design record (condensation session). | DONE 2026-06-22 [x] - canonical on branch; manifests+SCRATCH prove no loss; historical untouched. |
| [`STATUS.md`](../STATUS.md) | Starcom phase, blockers, next step (e.g. "Phase 0 CMake", "131.0-B-5 ref update"). Lighter than RC `PROJECT_STATUS.md`. | Placeholder — flesh out when implementation starts |

### At Rocket-Chip repo root (firmware-owned — do not copy into `starcom/`)

| File | Starcom relationship |
|---|---|
| Root [`CHANGELOG.md`](../../CHANGELOG.md) | Rocket-Chip firmware and integration history. |
| [`AGENT_WHITEBOARD.md`](../../AGENT_WHITEBOARD.md) | Cross-session RC flags. Starcom-specific notes may appear here **briefly** with a link to `starcom/` — not a second whiteboard. |
| [`docs/PROJECT_STATUS.md`](../../docs/PROJECT_STATUS.md) | RC phase/blockers. Starcom progress does not belong here except "RC blocked on Starcom MVP". |
| [`docs/IVP.md`](../../docs/IVP.md) | RC verification plan. Starcom has its own phased plan in `library_craft_claude.md` §7. |
| [`docs/decisions/*`](../../docs/decisions/) | RC architectural decisions (STOP-GAP retry map, Stage T, etc.). Not Starcom library decisions. |

### Not needed yet (add at Phase 0 / extraction)

- **`CODEOWNERS`** — when Starcom has its own GitHub repo or monorepo path owners
- **Issue/PR templates** — when external contributors are expected; comparison doc suggests frame hexdumps in PR template
- **`starcom/AGENT_WHITEBOARD.md`** — avoid; use root whiteboard + `STATUS.md` + `comparison.md` append-only instead (one coordination surface per concern)
- **Separate Starcom `PROJECT_STATUS.md`** — overkill until implementation; `STATUS.md` is enough

---

## What lives where

| Location | Belongs here |
|---|---|
| `starcom/include/`, `starcom/src/ccsds/` | Portable CCSDS protocol core |
| `starcom/adapters/host/` | Desktop transports (UDP, file replay, SDR bridge) |
| `starcom/adapters/rp2350/` | SX1276/PIO adapter (or keep thin wrapper in RC `src/` — TBD) |
| `starcom/tests/` | Host-side unit, property, fuzz tests |
| `starcom/docs/` | Library design, research, comparison, future `DESIGN.md` |
| Rocket-Chip `src/telemetry/telemetry_encoder.*` | STOP-GAP — stays until replaced by Starcom + RC adapter |
| Rocket-Chip `src/active_objects/ao_telemetry.*` | RC integration — not Starcom core |
| Rocket-Chip `docs/decisions/CURRENT_COMMAND_RETRY_ACK_*` | RC STOP-GAP baseline — input to future TC-layer work, not library docs |

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