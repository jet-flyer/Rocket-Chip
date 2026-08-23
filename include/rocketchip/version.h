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

constexpr uint8_t     kVersionMajor  = 0;
constexpr uint8_t     kVersionMinor  = 16;
constexpr uint8_t     kVersionPatch  = 0;
constexpr const char* kFirmwareVersion = "0.16.0";

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
