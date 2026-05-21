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

// Build identity. Always "flight" — single flight binary per role per
// R-25-exec (2026-05-13).
constexpr const char* kBuildConfig = "flight";

// Job role identity — for T=0 soak preconditions (IVP-132a.4 re-eval).
// Must match ROCKETCHIP_JOB_STATION / kRole. Readable by GDB.
#ifdef ROCKETCHIP_JOB_STATION
constexpr const char* kJobRole = "station";
#elif defined(ROCKETCHIP_JOB_RELAY)
constexpr const char* kJobRole = "relay";
#else
constexpr const char* kJobRole = "vehicle";
#endif

// Board identity — set by PICO_BOARD at CMake config time, exposed as
// a C string so GDB can read it at T=0 to catch Frankenstein builds.
// PICO_BOARD is a C preprocessor string (e.g., "adafruit_fruit_jam").
#ifndef PICO_BOARD
#define PICO_BOARD "unknown"
#endif
constexpr const char* kBoardName = PICO_BOARD;

// Git hash injected by CMake (-DGIT_HASH="..."). Falls back to "unknown".
#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif
constexpr const char* kGitHash = GIT_HASH;

// Build iteration tag — increment on every rebuild during debug sessions.
// See LESSONS_LEARNED.md Entry 2.
constexpr const char* kBuildIterationTag = "16B-init";

#endif // ROCKETCHIP_VERSION_H
