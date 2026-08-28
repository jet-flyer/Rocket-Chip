// Generated starcom/version.hpp matches STARCOM_VERSION; git identity is live.

#include "starcom/version.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>

#ifndef STARCOM_ROOT
#error "STARCOM_ROOT must be defined to the starcom/ tree"
#endif

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

std::map<std::string, std::string> parseVersionFile() {
  const std::string path = std::string(STARCOM_ROOT) + "/STARCOM_VERSION";
  std::ifstream in(path);
  CHECK(in.good());
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
      const auto last = s.find_last_not_of(" \t");
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

void test_header_matches_file_and_git() {
  const auto kv = parseVersionFile();
  CHECK(kv.find("VERSION_MAJOR") != kv.end());
  CHECK(kv.find("VERSION_MINOR") != kv.end());
  CHECK(kv.find("PATCHLEVEL") != kv.end());

  CHECK(starcom::kVersionMajor ==
        static_cast<std::uint8_t>(std::stoi(kv.at("VERSION_MAJOR"))));
  CHECK(starcom::kVersionMinor ==
        static_cast<std::uint8_t>(std::stoi(kv.at("VERSION_MINOR"))));
  CHECK(starcom::kVersionPatch ==
        static_cast<std::uint8_t>(std::stoi(kv.at("PATCHLEVEL"))));

  const std::string expected_lib = kv.at("VERSION_MAJOR") + "." +
                                   kv.at("VERSION_MINOR") + "." +
                                   kv.at("PATCHLEVEL");
  CHECK(std::strcmp(starcom::kLibraryVersion, expected_lib.c_str()) == 0);

  std::string expected_string = expected_lib;
  const auto extra = kv.find("EXTRAVERSION");
  if (extra != kv.end() && !extra->second.empty()) {
    expected_string += "-";
    expected_string += extra->second;
  }
  CHECK(std::strcmp(starcom::kVersionString, expected_string.c_str()) == 0);

  CHECK(std::strcmp(starcom::kGitHash, "unknown") != 0);
  CHECK(std::strcmp(starcom::kBuildIdentity, "unknown") != 0);
  CHECK(std::strlen(starcom::kGitHash) > 0);
  CHECK(std::strstr(starcom::kVersionString, "-dev") == nullptr);
}

}  // namespace

int run_version_tests() {
  test_header_matches_file_and_git();
  return g_fails;
}
