#include "starcom/ccsds/clcw.hpp"
#include "starcom/ccsds/cop1.hpp"
#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/crc.hpp"
#include "starcom/ccsds/mac.hpp"
#include "starcom/ccsds/plcw.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/space_packet.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/ccsds/uslp.hpp"
#include "starcom/ccsds/v3.hpp"

namespace starcom::ccsds {

static_assert(kOctetMsb == 7U);
static_assert(kSeqHalfRange == 127U);
static_assert(kCrc16Init == 0xFFFFU);
static_assert(kClcwStatusMask == 0x07U);
static_assert(kClcwVcidMask == 0x3FU);
static_assert(kPlcwExpeditedMask == 0x07U);
static_assert(kFarm1WindowMax == 254U);
static_assert(kBe32HiShift == 24U);
static_assert(kSpVersionShift == 5U);
static_assert(kSpApidHighMask == 0x07U);
static_assert(kSpApidMask == 0x7FFU);
static_assert(kSpSeqFlagsShift == 6U);
static_assert(kSpSeqCountHighMask == 0x3FU);
static_assert(kV3TfvnShift == 6U);
static_assert(kV3PcidShift == 7U);
static_assert(kV3PortMask == 0x07U);
static_assert(kV3LengthHighMask == 0x07U);
static_assert(kUslpScidNibbleMask == 0x0FU);
static_assert(kUslpScidHiShift == 12U);
static_assert(kUslpVcidPartMask == 0x07U);
static_assert(kUslpVcidLoShift == 5U);
static_assert(kUslpMapMask == 0x0FU);
static_assert(kUslpUpidMask == 0x1FU);
static_assert(kUslpRuleShift == 5U);
static_assert(kUslpRuleMask == 0x07U);
static_assert(kUslpVcfLenMask == 0x07U);
static_assert(kUslpVcfLenMax == 7U);
static_assert(kUslpLengthHeaderOctets == 6U);
static_assert(kRev8NibbleHi == 0xF0U);
static_assert(kRev8NibbleLo == 0x0FU);
static_assert(kRev8PairHi == 0xCCU);
static_assert(kRev8PairLo == 0x33U);
static_assert(kRev8BitHi == 0xAAU);
static_assert(kRev8BitLo == 0x55U);
static_assert(kMacSs0 == 0U);
static_assert(kMacSs1 == 1U);
static_assert(kMacSs2 == 2U);
static_assert(kMacSs3 == 3U);
static_assert(kMacSs4 == 4U);
static_assert(kMacSs5 == 5U);
static_assert(kMacSs6 == 6U);
static_assert(kMacSs7 == 7U);
static_assert(kMacXNoMoreData == 5U);
static_assert(kMacYActOnRccd == 5U);
static_assert(kSpduTypeMask == 0x07U);
static_assert(kSetVrSpareMask == 0xF8U);
static_assert(kDirTypeLsb == 15U);
static_assert(kDirTypeMid == 14U);
static_assert(kDirTypeMsb == 13U);
static_assert(kPhyModeMask == 0x07U);
static_assert(kPhyRateMask == 0x0FU);
static_assert(kPhyModBit == 7U);
static_assert(kPhyFreqMask == 0x07U);
static_assert(kPhyFreqShift == 10U);
static_assert(kCtrlTimeMask == 0x3FU);
static_assert(kCtrlDuplexMask == 0x07U);
static_assert(kCtrlDuplexShift == 6U);
static_assert(kCtrlRnmdBit == 11U);
static_assert(kCtrlPassBit == 12U);
static_assert(kPlExtDataShift == 5U);
static_assert(kPlExtModeShift == 7U);
static_assert(kPlExtScramShift == 9U);
static_assert(kPlExtDiffBit == 11U);
static_assert(kPlExtRsBit == 12U);
static_assert(kFopPSentCap == 127U);

}  // namespace starcom::ccsds

int run_constant_tests() {
  return 0;
}
