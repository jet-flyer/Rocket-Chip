// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Single source of truth for firmware version strings.
// All print sites must use version_string() or the constants below.
// See docs/audits/VERSION_STRING_AUDIT.md for the full audit.
#ifndef ROCKETCHIP_VERSION_H
#define ROCKETCHIP_VERSION_H

#include <stdint.h>

constexpr uint8_t     kVersionMajor  = 0;
constexpr uint8_t     kVersionMinor  = 16;
constexpr uint8_t     kVersionPatch  = 0;
constexpr const char* kFirmwareVersion = "0.16.0";

constexpr const char* kRcOsVersion = "0.5.0";

#ifndef BUILD_FOR_FLIGHT
constexpr const char* kBuildConfig = "bench";
#else
constexpr const char* kBuildConfig = "flight";
#endif

// Git hash injected by CMake (-DGIT_HASH="..."). Falls back to "unknown".
#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif
constexpr const char* kGitHash = GIT_HASH;

// Build iteration tag — increment on every rebuild during debug sessions.
// See LESSONS_LEARNED.md Entry 2.
constexpr const char* kBuildIterationTag = "16B-init";

#endif // ROCKETCHIP_VERSION_H
