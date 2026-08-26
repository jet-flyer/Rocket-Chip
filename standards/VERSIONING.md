# Rocket-Chip firmware identification

**Purpose:** One identification scheme so banners, CLI, and flight-log headers
cannot rot independently. This is the procedure SSOT for Rocket-Chip firmware.

Starcom has its own VERSIONING stub under `starcom/`. This document does not
cover that library.

NASA layers (SWEHB Ver D, not a Class claim):

- **SWE-081** identifies each configuration-item version. That is the git hash / `kBuildIdentity` on every build.
- **SWE-084** is why the banner must print that ID: an audit of the DUT cannot treat `v0.16.0` as proof of which tree is on the chip.
- **SWE-063** is the tagged cut: a version description (VDD) at each *release*, not each commit. SWE-116's a-i list is Topic 7.18 in SWEHB Ver D; we fill the Small Projects bar (unique ID, rebuild path, checksum, what changed, known issues), not a Class A/B VDD.

This is identification practice inspired by NASA NPR 7150.2 SWE-063/SWE-081/SWE-084,
SWEHB VDD, ECSS-M-ST-40C, ECSS-E-ST-40 software release, DO-178C configuration
management identification, Zephyr VERSION + generated header, PX4
`build_git_version.h`, and NASA cFS `cfe_version.h` + VDD. **It is not a NASA
Class, DO-178C, or ECSS certification claim.**

History: `docs/audits/VERSION_STRING_AUDIT.md` (IVP-127b, 2026-04-15) is the
frozen audit that first centralized literals. This procedure supersedes it.

---

## Two layers

| Layer | What it is | Who edits it | When it changes |
|---|---|---|---|
| **Product version** | Human-facing `MAJOR.MINOR.PATCH[-EXTRA]` | Humans, in repo-root `RC_VERSION` only | IVP numbered stage/sub-stage close, or a tagged cut |
| **Build identity** | `git describe --abbrev=12 --always --dirty` plus short hash | Nobody. CMake captures it | Every configure/build whose git HEAD or index moved |

The human tuple is the IVP stage, not a leftover advertising string.

- `MAJOR` stays 0 until a first-flight / first tagged operational cut (`1.0.0`).
- `MINOR` = last completed numbered Stage in `docs/IVP.md` (as of 2026-08-26: Stage 16).
- `PATCH` = last closed numbered sub-stage of that Stage (`16A`=1, `16B`=2, `16C`=3). Product line as of 2026-08-26: **`0.16.3-dev`**.
- Letter stages (L, T) and remediates (L2-P5) do not tick the tuple.
- Do not tick `MINOR` to 17 until a Stage 17 IVP actually closes. A board-only health check is not that.

`EXTRAVERSION = dev` until a tagged cut. Filename is `RC_VERSION`, never `VERSION`: a repo-root `VERSION` file is the C++20 header `<version>` on Windows (case-insensitive).

Between releases the unique discriminant is the git identity, not a hand-edited
build number. cFE's committed `CFE_BUILD_NUMBER` is the rot this scheme avoids:
CMake generates a CFE-style `kBuildNumber` as `git rev-list --count` since the
last `v*` tag if any, else since the first commit.

## Configuration items (SWE-081)

Each configuration item has its own product line. Do not hang RCOS or Starcom as extra fields on `0.16.3`.

| CI | Human epoch | File | In the banner (SWE-084) |
|---|---|---|---|
| **Rocket-Chip flight firmware** | IVP numbered Stage.sub-stage (as of 2026-08-26: `0.16.3-dev`) | repo-root `RC_VERSION` | always, plus git identity |
| **RCOS** | Own epoch, **not** Stage 16. Source still open (not `docs/IVP.md`). | to be named when the epoch is picked; never a second literal on `RC_VERSION` | not until that file exists (no fake `0.5.0`) |
| **Starcom library** | Own epoch. Not an IVP stage. | `starcom/` VERSIONING stub (library SSOT) | only when `Starcom::starcom` is linked; otherwise the banner says stop-gap/absent, not a fake Starcom version |

The old `kRcOsVersion = "0.5.0"` failed because it was a second RC tuple that never ticked. Separate CI, separate tick, or it will rot again.

---

## What humans may edit

**Firmware CI:** `RC_VERSION` only, when an IVP numbered stage/sub-stage closes or at a tagged cut. RCOS and Starcom are not this file. Zephyr-shaped:

```
VERSION_MAJOR = 0
VERSION_MINOR = 16
PATCHLEVEL = 3
VERSION_TWEAK = 0
EXTRAVERSION = dev
```

Never hand-edit:

- `${CMAKE_BINARY_DIR}/generated/rocketchip/version.h` (generated)
- A committed numeric `include/rocketchip/version.h` (there is none; the
  template is `include/rocketchip/version.h.in`)

CMake writes the generated header from the template. Both the host
(`BUILD_TESTS=ON`) and target builds include `cmake/rc_version.cmake` so there
is one mechanism, so the old `GIT_HASH` compile-definition is gone.

The generated header rebuilds (via CMake reconfigure) when `RC_VERSION`, git
`HEAD`, or git `index` change. If git is missing, `kGitHash` / `kBuildIdentity`
fall back to `"unknown"`.

---

## Public C++ names

Callers include `"rocketchip/version.h"` (the generated file; add
`${CMAKE_BINARY_DIR}/generated` to the include path).

| Symbol | Value |
|---|---|
| `kVersionMajor` / `kVersionMinor` / `kVersionPatch` | From `RC_VERSION` |
| `kFirmwareVersion` | `"MAJOR.MINOR.PATCH"` (PCM only) |
| `kVersionString` | `"MAJOR.MINOR.PATCH[-EXTRA]"` (e.g. `0.16.3-dev`) |
| `kGitHash` | `git rev-parse --short HEAD` |
| `kBuildIdentity` | `git describe --abbrev=12 --always --dirty` |
| `kBuildConfig` | CMake, not a version. `"flight"` (role/tier). Unchanged meaning post-R-26 |
| `kBuildNumber` | Generated count; do not hand-edit |

Dropped as RC-firmware fields: `kRcOsVersion` (RCOS is its own CI, not a second RC string) and `kBuildIterationTag`
(`16B-init`, frozen).

---

## SSOT in code (UX)

Firmware and host scripts never invent a version string.

- **C++:** `#include "rocketchip/version.h"` (generated). Print the symbols in
  the table above. No product-version literals in `src/` or `include/` except
  `RC_VERSION` and `include/rocketchip/version.h.in`.
- **Banner grammar** (SWE-084, one line):
  `RocketChip <kVersionString> <kBuildConfig>-<kGitHash>`
  Example as of 2026-08-26: `RocketChip 0.16.3-dev flight-c5a0d5b`
  Hardware-status may also print `kBuildIdentity` (`git describe`).
- **Host parsers** (`scripts/_rc_test_common.py`) consume that grammar. They may
  also accept the pre-2026-08-26 form (`RocketChip vX.Y.Z RCOS vA.B.C flight-<sha>`)
  so old logs still classify. They do not emit it.
- **PCM** `firmware_version[8]`: `kFirmwareVersion` only (`0.16.3` fits 7 chars +
  NUL). `static_assert` on the generated header rejects a product string that
  cannot fit. Full identity is CLI/banner only.
- **RCOS / Starcom:** not a second tuple on this line until that CI has its own
  epoch file. Linked CIs print their own grammar, never a hand-typed string.

### Audit 2026-08-26 (this sitting)

Live paths: `src/cli/rc_os_commands.cpp`, `src/active_objects/ao_rcos.cpp`,
`src/diag/diag_stats.cpp`, `src/logging/pcm_frame.cpp`,
`scripts/_rc_test_common.py`, `scripts/test__rc_test_common.py`.

Left as history (do not rewrite): `logs/stage_t/*`, `docs/baselines/*`,
`docs/audits/*`, the example banner in `standards/HW_GATE_DISCIPLINE.md`,
and `docs/USER_GUIDE.md` `Version: Stage 16A` (doc revision, not firmware).

Closes walk leftover **R-9** / **WN-010** / **WN-067**: process exists, numbers
are generated, host UX parser matches firmware. The walk log stays a
historical record; this document is the close.

## How to cut a release

1. Edit `RC_VERSION`: set `MAJOR.MINOR.PATCH`, set `EXTRAVERSION` empty.
2. Tag `vMAJOR.MINOR.PATCH` on the release commit (annotated).
3. Write a unique-path Version Description Document:
   `docs/releases/VDD-vX.Y.Z.md`.
4. Wrap `CHANGELOG.md` for that sitting (checklist wrap, not every commit).
5. Record integrity hashes (sha256 of the released UF2 and ELF).
6. Flash once, read the banner, paste the exact string into the VDD.

Do **not** bump product numbers on ordinary commits. Git identity already
changes.

### VDD template (releases only)

A VDD is NASA SWE-063/SWE-116 practice for **releases**, not every commit.
SWE-063: the project manager shall provide a software version description for
each software release. SWE-116 lists SVD content: identification, executable,
lifecycle data, archive, build instructions, integrity checks, product files,
open CRs, CRs implemented.

Subset we fill (do not fake compliance):

| Field | Content |
|---|---|
| Full ID | Product name, `X.Y.Z`, git tag `vX.Y.Z`, full commit hash |
| How to recreate | CMake preset, pointer to `docs/FLASHING.md` |
| Integrity | sha256 of the released UF2 and ELF |
| Changes since last release | Pointer to `CHANGELOG.md` |
| Known problems / open PRs | Pointer to `docs/PROBLEM_REPORTS.md` / open items |
| Banner string | Exact string the board prints |

---

## First release sitting (later)

This sitting installs the scheme. It does **not** mint `v0.16.0` or write a
VDD. When that sitting happens:

- [ ] Confirm advertised product is still the intended `MAJOR.MINOR.PATCH`
- [ ] Set `EXTRAVERSION` empty in `RC_VERSION`
- [ ] Tag `vMAJOR.MINOR.PATCH`
- [ ] Write `docs/releases/VDD-vX.Y.Z.md` from the table above
- [ ] Changelog wrap
- [ ] sha256 the UF2/ELF
- [ ] Confirm banner matches the VDD

---

## Why the old system failed

IVP-127b (2026-04-15, `docs/audits/VERSION_STRING_AUDIT.md`) centralized
literals into `include/rocketchip/version.h`, but the numbers stayed manual.
`kBuildIterationTag` froze at `16B-init`. Sitting 7 leftover **R-9**
(WN-010 / WN-067) left the bump process uninvented. The CMake-injected git
hash was the only live field.

This sitting addresses WN-010, WN-067, and rem WB R-9: product numbers live in
`RC_VERSION` and move when an IVP numbered stage/sub-stage closes, or at a tagged cut; every other build is identified by git.

---

## Sources (by name; not a certification claim)

- NASA NPR 7150.2 SWE-063 (VDD at each release)
- NASA NPR 7150.2 SWE-081 (identify each CI version)
- NASA NPR 7150.2 SWE-084 (mark the product so the DUT can be audited)
- NASA SWEHB Topic 7.18 / Ver D Small Projects (VDD content bar for this project)
- NASA NPR 7150.2 SWE-116 (historical SVD list; superseded as a numbered SWE)
- NASA SWEHB Version Description Document
- ECSS-M-ST-40C (configuration / information management; mark the item)
- ECSS-E-ST-40 (software engineering, software release)
- DO-178C configuration-management identification
- Zephyr Application version management (`VERSION` file + generated header)
- PX4 `build_git_version.h`
- NASA cFS cFE `cfe_version.h` (and why we do **not** commit a build number)
