// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// F´-shaped parameter catalog (sitting 5 v0). Pico/QP-free.
// set≠save: radio preset apply is runtime only (not .cfg persist).
// DEV_MODE / USB_ARM_INH are never SET over radio.
#ifndef ROCKETCHIP_CLI_CATALOG_H
#define ROCKETCHIP_CLI_CATALOG_H

#include <stddef.h>
#include <stdint.h>

namespace rc {
namespace cli {

enum class WriteClass : uint8_t {
    kRead = 0,
    kBasic,
    kAdvanced,
    kLocked,
};

enum class CatalogId : uint16_t {
    kIdentity   = 1,
    kNavPreset  = 2,
    kTxPower    = 3,
    kUsbArmInh  = 4,
    kUsbCfgEn   = 5,
    kStationOut = 6,
};

struct CatalogRow {
    CatalogId     id;
    const char*   name;
    WriteClass    wr;
    bool          vehicle;
    bool          station;
    const char*   starcom;  // "" or "radio"
    bool          persist;
};

inline constexpr CatalogRow kCatalog[] = {
    {CatalogId::kIdentity,   "IDENTITY",    WriteClass::kRead,   true,  true,  "",      false},
    {CatalogId::kNavPreset,  "NAV_PRESET",  WriteClass::kBasic,  true,  true,  "radio", false},
    {CatalogId::kTxPower,    "TX_POWER",    WriteClass::kLocked, true,  true,  "radio", false},
    {CatalogId::kUsbArmInh,  "USB_ARM_INH", WriteClass::kLocked, true,  false, "",      true},
    {CatalogId::kUsbCfgEn,   "USB_CFG_EN",  WriteClass::kLocked, true,  false, "",      true},
    {CatalogId::kStationOut, "STN_OUTPUT",  WriteClass::kBasic,  false, true,  "",      false},
};

inline constexpr size_t kCatalogCount = sizeof(kCatalog) / sizeof(kCatalog[0]);

inline constexpr const char* write_class_name(WriteClass w) {
    switch (w) {
        case WriteClass::kRead:     return "read";
        case WriteClass::kBasic:    return "basic";
        case WriteClass::kAdvanced: return "advanced";
        case WriteClass::kLocked:   return "locked";
    }
    return "?";
}

}  // namespace cli
}  // namespace rc

#endif
