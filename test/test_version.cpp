// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Host test: generated rocketchip/version.h matches repo-root RC_VERSION,
// PCM field fits 8 bytes, and git identity is live in a git tree.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <string>

#include "rocketchip/version.h"

#ifndef RC_REPO_ROOT
#error "RC_REPO_ROOT must be defined to the repository root"
#endif

static std::map<std::string, std::string> ParseVersionFile() {
    const std::string path = std::string(RC_REPO_ROOT) + "/RC_VERSION";
    std::ifstream in(path);
    EXPECT_TRUE(in.good()) << "failed to open " << path;
    std::map<std::string, std::string> kv;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        auto trim = [](std::string& s) {
            const auto first = s.find_first_not_of(" \t");
            const auto last  = s.find_last_not_of(" \t");
            if (first == std::string::npos) {
                s.clear();
                return;
            }
            s = s.substr(first, last - first + 1);
        };
        trim(key);
        trim(val);
        kv[key] = val;
    }
    return kv;
}

TEST(Version, GeneratedHeaderMatchesVersionFileAndGit) {
    const auto kv = ParseVersionFile();
    ASSERT_NE(kv.find("VERSION_MAJOR"), kv.end());
    ASSERT_NE(kv.find("VERSION_MINOR"), kv.end());
    ASSERT_NE(kv.find("PATCHLEVEL"), kv.end());

    EXPECT_EQ(kVersionMajor,
              static_cast<uint8_t>(std::stoi(kv.at("VERSION_MAJOR"))));
    EXPECT_EQ(kVersionMinor,
              static_cast<uint8_t>(std::stoi(kv.at("VERSION_MINOR"))));
    EXPECT_EQ(kVersionPatch,
              static_cast<uint8_t>(std::stoi(kv.at("PATCHLEVEL"))));

    const std::string expected_fw =
        kv.at("VERSION_MAJOR") + "." + kv.at("VERSION_MINOR") + "." +
        kv.at("PATCHLEVEL");
    EXPECT_STREQ(kFirmwareVersion, expected_fw.c_str());
    EXPECT_LT(std::strlen(kFirmwareVersion), 8u);

    auto extra_it = kv.find("EXTRAVERSION");
    std::string expected_string = expected_fw;
    if (extra_it != kv.end() && !extra_it->second.empty()) {
        expected_string += "-";
        expected_string += extra_it->second;
    }
    EXPECT_STREQ(kVersionString, expected_string.c_str());

    EXPECT_STRNE(kGitHash, "unknown");
    EXPECT_STRNE(kBuildIdentity, "unknown");
    EXPECT_GT(std::strlen(kGitHash), 0u);
    EXPECT_GT(kBuildNumber, 0u);
}
