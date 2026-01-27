/**
 * @file AP_ROMFS.h
 * @brief Stub AP_ROMFS for RocketChip
 *
 * ROMFS (embedded filesystem) is disabled - provides stub interface.
 */
#pragma once

#include <cstdint>
#include <cstddef>

class AP_ROMFS {
public:
    // Find a file in ROMFS - returns nullptr when disabled
    static const uint8_t *find_decompress(const char *name, uint32_t &size) {
        size = 0;
        return nullptr;
    }

    // Free decompressed data - no-op when disabled
    static void free(const uint8_t *data) {
        (void)data;
    }

    // Check if file exists - always false when disabled
    static bool exists(const char *name) {
        (void)name;
        return false;
    }

    // Get directory listing - returns nullptr when disabled
    static const char *dir_list(const char *path, uint16_t &num_files) {
        (void)path;
        num_files = 0;
        return nullptr;
    }
};
