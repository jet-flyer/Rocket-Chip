// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "starcom_adapt/nav_sdu.h"
#include <string.h>

namespace rc {

uint8_t pack_nav_sdu_user(uint8_t* out, uint8_t out_len,
                          const TelemetryState& telem) {
    if ((out == nullptr) || (out_len < kNavSduUserBytes)) {
        return 0;
    }
    memcpy(out, &telem, kNavSduUserBytes);
    return kNavSduUserBytes;
}

bool unpack_nav_sdu_user(const uint8_t* in, uint8_t in_len,
                         TelemetryState* telem) {
    if ((in == nullptr) || (telem == nullptr) ||
        (in_len != kNavSduUserBytes)) {
        return false;
    }
    memcpy(telem, in, kNavSduUserBytes);
    return true;
}

}  // namespace rc
