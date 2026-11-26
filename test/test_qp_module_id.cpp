// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// QP assert module name to crash-record index and back.
#include <gtest/gtest.h>

#include "safety/qp_module_id.h"

TEST(QpModuleId, KnownNamesRoundTrip) {
    for (uint32_t i = 0U; i < rc::kQpModuleCount; ++i) {
        EXPECT_EQ(rc::qp_module_index(rc::kQpModuleNames[i]), i);
        EXPECT_STREQ(rc::qp_module_name(i), rc::kQpModuleNames[i]);
    }
}

// Q_onError gets the module's own copy of the string, not our pointer.
TEST(QpModuleId, ComparesTextNotPointer) {
    char buf[16] = "qf_actq";  // Own copy, not the table pointer
    EXPECT_EQ(rc::qp_module_index(buf), 2U);
    EXPECT_STREQ(rc::qp_module_name(rc::qp_module_index(buf)), "qf_actq");
}

TEST(QpModuleId, PrefixAndLongerNamesDoNotMatch) {
    EXPECT_EQ(rc::qp_module_index("qf_act"), 1U);
    EXPECT_EQ(rc::qp_module_index("qf_ac"), rc::kQpModuleUnknown);
    EXPECT_EQ(rc::qp_module_index("qf_actqx"), rc::kQpModuleUnknown);
    EXPECT_EQ(rc::qp_module_index(""), rc::kQpModuleUnknown);
}

TEST(QpModuleId, UnknownAndNull) {
    EXPECT_EQ(rc::qp_module_index(nullptr), rc::kQpModuleUnknown);
    EXPECT_EQ(rc::qp_module_index("some/path/file.c"), rc::kQpModuleUnknown);
    EXPECT_STREQ(rc::qp_module_name(rc::kQpModuleUnknown), "unknown");
    EXPECT_STREQ(rc::qp_module_name(rc::kQpModuleCount), "unknown");
}
