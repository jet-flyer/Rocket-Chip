// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Boot hardware check for PSRAM (plan section 5): a board with no PSRAM
// (Tiny 2350, Pico 2) does not count a PSRAM pass; a board that fits it
// passes only when the logger could use it.

#include <gtest/gtest.h>
#include <stdint.h>

#include "rocketchip/psram_check_policy.h"

using rc::PsramCheck;
using rc::psram_check;

namespace {
constexpr uint32_t k8Mb = 8U * 1024U * 1024U;
}

TEST(PsramCheckPolicy, NotFittedIsNotCountedWhateverTheFlags) {
    EXPECT_EQ(psram_check(false, 0U, false, false), PsramCheck::kNotCounted);
    EXPECT_EQ(psram_check(false, k8Mb, true, true), PsramCheck::kNotCounted);
}

TEST(PsramCheckPolicy, FittedAndUsablePasses) {
    EXPECT_EQ(psram_check(true, k8Mb, true, true), PsramCheck::kPass);
}

TEST(PsramCheckPolicy, FittedButNotDetectedFails) {
    EXPECT_EQ(psram_check(true, 0U, false, false), PsramCheck::kFail);
}

TEST(PsramCheckPolicy, FittedSelfTestOrFlashSafeFailFails) {
    EXPECT_EQ(psram_check(true, k8Mb, false, false), PsramCheck::kFail);
    EXPECT_EQ(psram_check(true, k8Mb, true, false), PsramCheck::kFail);
}

TEST(PsramCheckPolicy, PacksMatchTheirHardware) {
    // Compile-time: the predicate is constexpr, so the boot count can
    // branch on it with if constexpr for the not-fitted case.
    static_assert(psram_check(false, 0U, false, false) == PsramCheck::kNotCounted,
                  "no PSRAM: not counted");
    SUCCEED();
}
