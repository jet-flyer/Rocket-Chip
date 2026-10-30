// tidy_probe.cpp: Tier 1.1 known-bad fixture for starcom/.clang-tidy.
// Each marked line holds one planted defect. It is never built or linked.
// starcom/scripts/tidy_selftest.sh runs clang-tidy on it and requires each
// check named in an EXPECT tag to fire, and no hit in third_party/.
#include <cassert>
#include <stdint.h>  // EXPECT modernize-deprecated-headers
#include <utility>

#include "probe_hit.hpp"     // header hit (PR-1b) is the tag in probe_hit.hpp
#include "probe_vendor.hpp"  // EXPECT-NOT probe_vendor.hpp (third_party/include: matches the include rule, so the exclude rule must drop it, G3)

int _probe_reserved = 0;  // EXPECT bugprone-reserved-identifier

enum class ProbeEnum : int { a, b };  // EXPECT performance-enum-size

void probeConstParam(const int v);  // EXPECT readability-avoid-const-params-in-decls

int probeVarargDef(int n, ...) { return n; }  // EXPECT cert-dcl50-cpp

int probeGoto(int x) {
  if (x > 0) {
    goto done;  // EXPECT cppcoreguidelines-avoid-goto
  }
  x = 2;
done:
  return x;
}

int probeAnalyzerNull() {
  int* p = nullptr;
  return *p;  // EXPECT clang-analyzer-core.NullDereference
}

int probeAnalyzerDiv(int x) {
  const int z = 0;
  return x / z;  // EXPECT clang-analyzer-core.DivideZero
}

int probeMisc(int x, int* buf) {
  int unchanged = x;  // EXPECT misc-const-correctness
  assert(++x > 0);    // EXPECT bugprone-assert-side-effect
  const int s = x & 3;  // EXPECT hicpp-signed-bitwise
  auto* raw = reinterpret_cast<char*>(buf);  // EXPECT cppcoreguidelines-pro-type-reinterpret-cast
  (void)raw;
  const auto pr = std::make_pair<int, int>(1, 2);  // EXPECT google-build-explicit-make-pair
  if (s > 1)
    x = 1;
    x = 2;  // EXPECT readability-misleading-indentation
  return probeVarargDef(1, unchanged, pr.first);  // EXPECT cppcoreguidelines-pro-type-vararg
}
