/**
 * @file AP_Filesystem.h
 * @brief Stub AP_Filesystem for RocketChip
 *
 * Filesystem is disabled - provides stub interface.
 */
#pragma once

#include "AP_Filesystem_config.h"
#include <cstdint>
#include <cstddef>

#if !AP_FILESYSTEM_ENABLED

class AP_Filesystem {
public:
    static AP_Filesystem* get_singleton() { return nullptr; }

    // File operations - all fail when disabled
    int open(const char* path, int flags) { return -1; }
    int close(int fd) { return -1; }
    int read(int fd, void* buf, size_t count) { return -1; }
    int write(int fd, const void* buf, size_t count) { return -1; }
    int fsync(int fd) { return -1; }
    int32_t lseek(int fd, int32_t offset, int whence) { return -1; }
    int stat(const char* path, struct stat* stbuf) { return -1; }
    int unlink(const char* path) { return -1; }
    int mkdir(const char* path) { return -1; }

    // Directory operations
    void* opendir(const char* path) { return nullptr; }
    int closedir(void* dirp) { return -1; }

    // Disk info
    bool disk_free(const char* path, uint64_t* free_bytes) { return false; }
    bool disk_total(const char* path, uint64_t* total_bytes) { return false; }

    // Format
    bool format(void) { return false; }
};

namespace AP {
    AP_Filesystem& FS();
}

#endif // !AP_FILESYSTEM_ENABLED
