// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// QP assert module name to a small index, for the crash record. Q_onError
// gets a pointer to the module name string (Q_DEFINE_THIS_MODULE). The
// record keeps the index, not the pointer (no pointer-to-integer cast,
// JSF AV 182). Safe in fault context: no stdio, no heap, bounded loops.
// Pure code, so the host tests can use it.
#ifndef ROCKETCHIP_SAFETY_QP_MODULE_ID_H
#define ROCKETCHIP_SAFETY_QP_MODULE_ID_H

#include <stddef.h>
#include <stdint.h>

namespace rc {

// Q_DEFINE_THIS_MODULE names in lib/qep and src. Add new names at the end,
// so the index of a name does not change between images.
inline constexpr const char* kQpModuleNames[] = {
    "qep_hsm",  "qf_act",  "qf_actq", "qf_defer", "qf_dyn",  "qf_mem",
    "qf_ps",    "qf_qact", "qf_qeq",  "qf_time",  "qv",      "flight_director",
};
inline constexpr uint32_t kQpModuleCount =
    static_cast<uint32_t>(sizeof(kQpModuleNames) / sizeof(kQpModuleNames[0]));
inline constexpr uint32_t kQpModuleUnknown = 0xFFU;
inline constexpr size_t kQpModuleNameMax = 32U;  // Compare limit

// True if a and b are the same string in the first kQpModuleNameMax chars.
inline bool qp_module_name_equal(const char* a, const char* b) {
    for (size_t i = 0U; i < kQpModuleNameMax; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
        if (a[i] == '\0') {
            return true;
        }
    }
    return true;
}

// Index in kQpModuleNames, or kQpModuleUnknown (also for nullptr).
inline uint32_t qp_module_index(const char* name) {
    if (name == nullptr) {
        return kQpModuleUnknown;
    }
    for (uint32_t i = 0U; i < kQpModuleCount; ++i) {
        if (qp_module_name_equal(name, kQpModuleNames[i])) {
            return i;
        }
    }
    return kQpModuleUnknown;
}

// Name for an index from the crash record, or "unknown".
inline const char* qp_module_name(uint32_t index) {
    return (index < kQpModuleCount) ? kQpModuleNames[index] : "unknown";
}

} // namespace rc

#endif // ROCKETCHIP_SAFETY_QP_MODULE_ID_H
