// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Canonical home for firmware version *constants* (semver + RC_OS string).
// Git hash is CMake-injected. Board/job identity is board:: / job::, not here.
// There is no version_string() helper — use kFirmwareVersion / kGitHash.
// Numbers are manual; kGitHash is the live discriminant. No bump process
// is wired (WN-010/067; rem WB R-9).
// See docs/audits/VERSION_STRING_AUDIT.md.
#ifndef ROCKETCHIP_VERSION_H
#define ROCKETCHIP_VERSION_H

#include <stdint.h>

// One trio. String is the stringify of these — do not edit the quoted form.
#define RC_VERSION_MAJOR 0
#define RC_VERSION_MINOR 16
#define RC_VERSION_PATCH 0
#define RC_VERSION_STR_X(a, b, c) #a "." #b "." #c
#define RC_VERSION_STR(a, b, c) RC_VERSION_STR_X(a, b, c)

constexpr uint8_t     kVersionMajor  = RC_VERSION_MAJOR;
constexpr uint8_t     kVersionMinor  = RC_VERSION_MINOR;
constexpr uint8_t     kVersionPatch  = RC_VERSION_PATCH;
constexpr const char* kFirmwareVersion =
    RC_VERSION_STR(RC_VERSION_MAJOR, RC_VERSION_MINOR, RC_VERSION_PATCH);

constexpr const char* kRcOsVersion = "0.5.0";

// Build identity. Always "flight" — single flight binary per role per
// R-25-exec (2026-05-13). Board/job strings live in board:: / job:: (WN-068).
constexpr const char* kBuildConfig = "flight";

// Git hash injected by CMake (-DGIT_HASH="..."). Falls back to "unknown".
#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif
constexpr const char* kGitHash = GIT_HASH;

// Build iteration tag — increment on every rebuild during debug sessions.
// See LESSONS_LEARNED.md Entry 2.
constexpr const char* kBuildIterationTag = "16B-init";

#endif // ROCKETCHIP_VERSION_H
