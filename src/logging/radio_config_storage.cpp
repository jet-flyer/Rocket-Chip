// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// radio_config_storage.cpp — dual-sector flash persistence for RadioConfig.
//
// Flash layout (anchored from end of flash via flash_layout.h):
//   Sector A: kFlashRadioCfgSectorA (4 KB)
//   Sector B: kFlashRadioCfgSectorB (4 KB)
//
// Writes alternate between sectors; sector with higher sequence number is
// the authoritative copy on read. Pattern cloned from calibration_storage.
//============================================================================

#include "radio_config_storage.h"
#include "rocketchip/flash_layout.h"
#include "rocketchip/radio_config_table.h"  // whitelist validation on read
#include "crc16_ccitt.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

static constexpr uint32_t kSectorA = rc::kFlashRadioCfgSectorA;
static constexpr uint32_t kSectorB = rc::kFlashRadioCfgSectorB;

// Sector header (16 bytes — aligns cleanly with flash page)
struct SectorHeader {
    uint32_t state;         // kStateInUse or 0xFFFFFFFF (erased)
    uint32_t sequence;      // write sequence; higher = newer
    uint32_t reserved[2];   // padding to 16 B
};
static_assert(sizeof(SectorHeader) == 16, "SectorHeader size");

static constexpr uint32_t kStateErased = 0xFFFFFFFFU;
static constexpr uint32_t kStateInUse  = 0x52435253U;  // "RCRS" = RocketChip Radio Storage

// Entry payload: magic + RadioConfig + CRC16
struct Entry {
    uint32_t         magic;   // kEntryMagic
    rc::RadioConfig  cfg;
    uint16_t         crc;     // crc16_ccitt over magic + cfg
    uint16_t         pad;
};
static constexpr uint32_t kEntryMagic = 0x52434352U;  // "RCCR" = RocketChip Cfg Radio

static constexpr uint32_t kFlashTimeoutMs = 1000;

static bool     g_initialized = false;
static uint32_t g_active_sector = 0;
static uint32_t g_write_sequence = 0;
static rc::RadioConfig g_cached = {};
static bool     g_cached_valid = false;

// ============================================================================
// Flash helpers (mirrored from calibration_storage for consistency)
// ============================================================================

struct WriteParams {
    uint32_t offset;
    const uint8_t* data;
    size_t len;
};

struct EraseParams {
    uint32_t offset;
    size_t len;
};

static void do_flash_write(void* param) {
    auto* p = static_cast<WriteParams*>(param);
    flash_range_program(p->offset, p->data, p->len);
}

static void do_flash_erase(void* param) {
    auto* p = static_cast<EraseParams*>(param);
    flash_range_erase(p->offset, p->len);
}

static bool safe_write(uint32_t offset, const uint8_t* data, size_t len) {
    if ((offset % FLASH_PAGE_SIZE) != 0) { return false; }
    if ((len % FLASH_PAGE_SIZE) != 0) { return false; }
    WriteParams p{offset, data, len};
    return flash_safe_execute(do_flash_write, &p, kFlashTimeoutMs) == PICO_OK;
}

static bool safe_erase(uint32_t offset, size_t len) {
    if ((offset % FLASH_SECTOR_SIZE) != 0) { return false; }
    if ((len % FLASH_SECTOR_SIZE) != 0) { return false; }
    EraseParams p{offset, len};
    return flash_safe_execute(do_flash_erase, &p, kFlashTimeoutMs) == PICO_OK;
}

static void flash_read(uint32_t offset, void* dest, size_t len) {
    const auto* src = reinterpret_cast<const uint8_t*>(XIP_BASE) + offset;
    memcpy(dest, src, len);
}

// ============================================================================
// CRC + validation
// ============================================================================

static uint16_t compute_crc(const Entry& e) {
    // CRC over magic + cfg bytes (exclude the stored crc field itself)
    uint8_t buf[sizeof(uint32_t) + sizeof(rc::RadioConfig)];
    memcpy(buf, &e.magic, sizeof(uint32_t));
    memcpy(buf + sizeof(uint32_t), &e.cfg, sizeof(rc::RadioConfig));
    return rc::crc16_ccitt(buf, sizeof(buf));
}

static bool validate_entry(const Entry& e) {
    if (e.magic != kEntryMagic) { return false; }
    if (e.crc != compute_crc(e)) { return false; }
    // Extra defence: reject configs that aren't SX1276-hardware-legal.
    // Protects against flash corruption that happens to match CRC. Uses the
    // broader validator (not preset-match) per 2026-04-21 user direction —
    // persisted configs may be user-set advanced values, not just presets.
    if (!rc::radio_config_sx1276_legal(
            e.cfg.bandwidth_khz, e.cfg.nav_rate_hz,
            e.cfg.spreading_factor, e.cfg.coding_rate,
            e.cfg.power_dbm)) {
        return false;
    }
    return true;
}

// ============================================================================
// Sector management
// ============================================================================

static bool read_valid_entry(uint32_t sector, Entry* out_entry, uint32_t* out_seq) {
    SectorHeader hdr;
    flash_read(sector, &hdr, sizeof(hdr));
    if (hdr.state != kStateInUse) { return false; }

    Entry entry;
    flash_read(sector + sizeof(SectorHeader), &entry, sizeof(entry));
    if (!validate_entry(entry)) { return false; }

    *out_entry = entry;
    *out_seq = hdr.sequence;
    return true;
}

static void find_active_sector() {
    Entry entry_a, entry_b;
    uint32_t seq_a = 0, seq_b = 0;
    bool valid_a = read_valid_entry(kSectorA, &entry_a, &seq_a);
    bool valid_b = read_valid_entry(kSectorB, &entry_b, &seq_b);

    if (!valid_a && !valid_b) {
        g_active_sector = kSectorA;
        g_write_sequence = 1;
        g_cached_valid = false;
        return;
    }
    if (valid_a && !valid_b) {
        g_active_sector = kSectorA;
        g_write_sequence = seq_a + 1;
        g_cached = entry_a.cfg;
        g_cached_valid = true;
        return;
    }
    if (!valid_a && valid_b) {
        g_active_sector = kSectorB;
        g_write_sequence = seq_b + 1;
        g_cached = entry_b.cfg;
        g_cached_valid = true;
        return;
    }
    // Both valid — newer sequence wins.
    if (seq_a >= seq_b) {
        g_active_sector = kSectorA;
        g_write_sequence = seq_a + 1;
        g_cached = entry_a.cfg;
    } else {
        g_active_sector = kSectorB;
        g_write_sequence = seq_b + 1;
        g_cached = entry_b.cfg;
    }
    g_cached_valid = true;
}

static uint32_t alternate_of(uint32_t sector) {
    return (sector == kSectorA) ? kSectorB : kSectorA;
}

static bool write_to_sector(uint32_t sector, const rc::RadioConfig* cfg, uint32_t seq) {
    // Page-aligned buffer (single page is enough — header+entry is <256B)
    static uint8_t g_page[FLASH_PAGE_SIZE] __attribute__((aligned(4)));
    memset(g_page, 0xFF, sizeof(g_page));

    auto* hdr = static_cast<SectorHeader*>(static_cast<void*>(g_page));
    hdr->state = kStateInUse;
    hdr->sequence = seq;

    auto* entry = static_cast<Entry*>(static_cast<void*>(g_page + sizeof(SectorHeader)));
    entry->magic = kEntryMagic;
    entry->cfg = *cfg;
    entry->pad = 0;
    entry->crc = compute_crc(*entry);

    if (!safe_erase(sector, FLASH_SECTOR_SIZE)) { return false; }
    return safe_write(sector, g_page, FLASH_PAGE_SIZE);
}

// ============================================================================
// Public API
// ============================================================================

bool radio_config_storage_init() {
    if (g_initialized) { return true; }
    find_active_sector();
    g_initialized = true;
    return true;
}

bool radio_config_storage_read(rc::RadioConfig* cfg) {
    if (cfg == nullptr) { return false; }
    if (!g_initialized) { radio_config_storage_init(); }
    if (!g_cached_valid) { return false; }
    *cfg = g_cached;
    return true;
}

bool radio_config_storage_write(const rc::RadioConfig* cfg) {
    if (cfg == nullptr) { return false; }
    if (!g_initialized) { radio_config_storage_init(); }

    // If the cached copy matches, skip the write — avoids flash wear from
    // no-op writes (e.g., revert that lands on the same config as last saved).
    if (g_cached_valid && memcmp(&g_cached, cfg, sizeof(*cfg)) == 0) {
        return true;
    }

    uint32_t target = alternate_of(g_active_sector);
    if (!write_to_sector(target, cfg, g_write_sequence)) {
        return false;
    }
    g_active_sector = target;
    g_write_sequence++;
    g_cached = *cfg;
    g_cached_valid = true;
    return true;
}

bool radio_config_storage_erase() {
    if (!g_initialized) { radio_config_storage_init(); }
    if (!safe_erase(kSectorA, FLASH_SECTOR_SIZE)) { return false; }
    if (!safe_erase(kSectorB, FLASH_SECTOR_SIZE)) { return false; }
    g_active_sector = kSectorA;
    g_write_sequence = 1;
    g_cached_valid = false;
    return true;
}
