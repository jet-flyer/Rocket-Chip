// IVP increment 24 — size report (D-3/D-5 library-craft). Host sizeof
// of the types that blew Pico Core 0 (4 KiB, PICO_STACK_SIZE=0x1000).
// Numbers are printed so the increment record is measured, not invented.

#include "starcom/ccsds/cop1.hpp"
#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/ldpc.hpp"
#include "starcom/ccsds/types.hpp"

#include <array>
#include <cstddef>
#include <cstdio>

using starcom::ccsds::Cop1Endpoint;
using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::FarmP;
using starcom::ccsds::FopP;
using starcom::ccsds::kLdpcCodewordOctets;
using starcom::ccsds::kLdpcMessageOctets;
using starcom::ccsds::kTransferFrameMax;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

void test_not_stack_safe_on_pico_core0() {
  // These must live in BSS/static. Value-init on a 4 KiB stack reset-loops.
  CHECK(sizeof(CoppEndpoint) > 4096u);
  CHECK(sizeof(Cop1Endpoint) > 4096u);
  CHECK(sizeof(std::array<std::byte, kTransferFrameMax>) ==
        kTransferFrameMax);
  // ldpcEncodeBlock keeps u[128]+p[128] automatic; must stay under the
  // GNU -Wstack-usage=1024 gate (CompilerWarnings.cmake).
  CHECK(kLdpcMessageOctets + kLdpcMessageOctets < 1024u);
  CHECK(kLdpcCodewordOctets == 256u);
}

void test_print_ivp24_sizes() {
  std::printf(
      "IVP24 sizeof CoppEndpoint=%zu Cop1Endpoint=%zu FopP=%zu FarmP=%zu "
      "kTransferFrameMax=%zu kLdpcMessageOctets=%zu\n",
      sizeof(CoppEndpoint), sizeof(Cop1Endpoint), sizeof(FopP), sizeof(FarmP),
      kTransferFrameMax, kLdpcMessageOctets);
}

}  // namespace

int run_size_tests() {
  test_not_stack_safe_on_pico_core0();
  test_print_ivp24_sizes();
  return g_fails;
}
