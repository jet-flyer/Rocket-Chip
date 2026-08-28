# Starcom versioning

**Purpose:** One identification scheme so CMake, headers, and tests cannot rot independently. This is the library SSOT. Rocket-Chip firmware uses repo-root `RC_VERSION` and `standards/VERSIONING.md` — that document does **not** cover Starcom.

Same shape as the 2026-08-26 Rocket-Chip close (Zephyr `VERSION` + generated header; git as the live discriminant). Product numbers are human; every other build is identified by git.

[Semantic Versioning](https://semver.org): MAJOR for incompatible changes to the supported API, MINOR for backward-compatible additions, PATCH for compatible fixes. While MAJOR is 0 the API may still move; MINOR still ticks with completed increments so the tuple is not advertising.

---

## Two layers

| Layer | What it is | Who edits it | When it changes |
|---|---|---|---|
| **Product version** | `MAJOR.MINOR.PATCH[-EXTRA]` | Humans, in `STARCOM_VERSION` only | Starcom IVP increment close, or a tagged cut |
| **Build identity** | `git describe --abbrev=12 --always --dirty` plus short hash | Nobody. CMake captures it | Every configure whose git HEAD or index moved |

- `MAJOR` stays 0 until a first tagged operational/extract cut (`1.0.0`).
- `MINOR` = last **completed** Starcom IVP increment (`docs/IVP.md`). Increment 0+1 counts as 1. After increment 19 (conv / LDPC encode): **`0.19.0-dev`**.
- `PATCH` = remediates inside that increment (usually 0).
- `EXTRAVERSION = dev` until an annotated tag with empty EXTRA.

The old IVP phrase “first `0.1.0`” is **not** a second product line. The first tag is `starcom-vMAJOR.MINOR.PATCH` matching this file with EXTRA empty (increment 25; not a marketing `0.1.0` that disagrees with MINOR).

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
VERSION_MINOR = 15
PATCHLEVEL = 0
VERSION_TWEAK = 0
EXTRAVERSION = dev
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
| `starcom::kVersionString` | `"MAJOR.MINOR.PATCH[-EXTRA]"` (e.g. `0.15.0-dev`) |
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

### First tagged cut (later)

This sitting installs the scheme. It does **not** mint `starcom-v0.5.0` or empty EXTRA.

- [ ] Confirm advertised product is the intended `MAJOR.MINOR.PATCH`
- [ ] Set `EXTRAVERSION` empty
- [ ] Tag `starcom-vMAJOR.MINOR.PATCH`
- [ ] Changelog wrap

---

## Why this matches Rocket-Chip

IVP-127b centralized RC literals, then they froze (`16B-init`). 2026-08-26 closed that: humans edit one file; CMake generates the header; git is the discriminant. Starcom copies that mechanism, with its **own** epoch (IVP increments, not firmware Stage 16). RC `standards/VERSIONING.md` already said Starcom is a separate configuration item.

Sources (practice, not a certification claim): SemVer 2.0.0; Zephyr Application version management; PX4 `build_git_version.h`; NASA cFS `cfe_version.h` (and why we do not commit a build number); Rocket-Chip `standards/VERSIONING.md` (2026-08-26).
