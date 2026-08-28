# Starcom versioning

**Purpose:** One identification scheme so CMake, headers, and tests cannot rot independently. This is the library SSOT. Rocket-Chip firmware uses repo-root `RC_VERSION` and `standards/VERSIONING.md` — that document does **not** cover Starcom.

Same shape as the 2026-08-26 Rocket-Chip close (Zephyr `VERSION` + generated header; git as the live discriminant). Product numbers are human; every other build is identified by git.

[Semantic Versioning](https://semver.org): MAJOR for incompatible changes to the supported API, MINOR for backward-compatible additions, PATCH for compatible fixes. While MAJOR is 0 the API may still move. Owner pick for the first tagged cut (IVP 25): **`0.2.N`** where **N is the IVP increment number**.

---

## Two layers

| Layer | What it is | Who edits it | When it changes |
|---|---|---|---|
| **Product version** | `MAJOR.MINOR.PATCH[-EXTRA]` | Humans, in `STARCOM_VERSION` only | Starcom IVP increment close, or a tagged cut |
| **Build identity** | `git describe --abbrev=12 --always --dirty` plus short hash | Nobody. CMake captures it | Every configure whose git HEAD or index moved |

- `MAJOR` stays 0 until an extract / operational `1.0.0`.
- `MINOR` = **2** for this incubation line (owner pick, IVP 25). Not the IVP number.
- `PATCH` = last **completed** Starcom IVP increment (`docs/IVP.md`). Increment 0+1 counts as 1. First tagged cut is increment 25 → **`0.2.25`**.
- `EXTRAVERSION` empty on an annotated tag; `dev` on untagged work after that.

The old IVP phrase “first `0.1.0`” and the pre-tag `0.N.0-dev` (MINOR = increment) are **not** a second product line. The first tag is `starcom-v0.2.25`.

Filename is `STARCOM_VERSION`, never `VERSION`: on Windows a repo-root `VERSION` file is the C++20 header `<version>` (same reason Rocket-Chip uses `RC_VERSION`).

While incubating in the monorepo, git tags for this library are `starcom-v*`. After extract they can be `v*`. `kBuildNumber` is `git rev-list --count` since the last `starcom-v*` tag if any, else commits that touch this tree (`rev-list --count HEAD -- .` from `starcom/`). Do not commit a hand-edited build number (cFE `CFE_BUILD_NUMBER` rot).

---

## Supported API

`include/starcom/` only (generated `version.hpp` included). Not `src/`, not `detail/`, not test helpers. Breaking a public header or `Starcom::starcom` / `Starcom::adapters_host` CMake alias is a MAJOR bump (or, while 0.x, a MINOR tick plus a migration note in `CHANGELOG.md`).

Adapters (`starcom::adapters`) are first-party ports, still under SemVer for this tree.

---

## What humans may edit

`STARCOM_VERSION` only, when an increment closes or at a tagged cut:

```
VERSION_MAJOR = 0
VERSION_MINOR = 2
PATCHLEVEL = 25
VERSION_TWEAK = 0
EXTRAVERSION =
```

Never hand-edit:

- `${CMAKE_BINARY_DIR}/generated/starcom/version.hpp` (generated)
- A committed numeric `include/starcom/version.hpp` (there is none; the template is `version.hpp.in`)

CMake (`cmake/starcom_version.cmake`) writes the generated header. Reconfigure when `STARCOM_VERSION`, git `HEAD`, or git `index` change. If git is missing, `kGitHash` / `kBuildIdentity` fall back to `"unknown"`.

Do **not** bump product numbers on ordinary codec commits. Git identity already changes.

---

## Public C++ names

`#include "starcom/version.hpp"` (generated). Add the build `generated/` dir to the include path (`Starcom::starcom` does this).

| Symbol | Value |
|--------|--------|
| `starcom::kVersionMajor` / `Minor` / `Patch` | From `STARCOM_VERSION` |
| `starcom::kLibraryVersion` | `"MAJOR.MINOR.PATCH"` |
| `starcom::kVersionString` | `"MAJOR.MINOR.PATCH[-EXTRA]"` (e.g. `0.2.25`) |
| `starcom::kGitHash` | `git rev-parse --short HEAD` |
| `starcom::kBuildIdentity` | `git describe --abbrev=12 --always --dirty` |
| `starcom::kBuildNumber` | Generated count; do not hand-edit |

No product-version literals in `src/` or `include/` except `STARCOM_VERSION` and `version.hpp.in`.

---

## How to cut a release

1. Edit `STARCOM_VERSION`: intended `MAJOR.MINOR.PATCH`, `EXTRAVERSION` empty.
2. Tag `starcom-vMAJOR.MINOR.PATCH` (annotated) on that commit. After extract: `vMAJOR.MINOR.PATCH`.
3. Wrap `CHANGELOG.md` for that sitting (major push).
4. Record sha256 of the released static lib / install tree if one is published.

### First tagged cut (IVP 25)

Owner pick: **`0.2.25`** (`PATCH` = increment 25). FPGA PHY/decode still held.

- [x] Advertised product `0.2.25`
- [x] `EXTRAVERSION` empty
- [x] Tag `starcom-v0.2.25`
- [x] Starcom `CHANGELOG.md` wrap for the whole close (not versioning-only)

---

## Why this matches Rocket-Chip

IVP-127b centralized RC literals, then they froze (`16B-init`). 2026-08-26 closed that: humans edit one file; CMake generates the header; git is the discriminant. Starcom copies that mechanism, with its **own** epoch (IVP increments, not firmware Stage 16). RC `standards/VERSIONING.md` already said Starcom is a separate configuration item.

Sources (practice, not a certification claim): SemVer 2.0.0; Zephyr Application version management; PX4 `build_git_version.h`; NASA cFS `cfe_version.h` (and why we do not commit a build number); Rocket-Chip `standards/VERSIONING.md` (2026-08-26).
