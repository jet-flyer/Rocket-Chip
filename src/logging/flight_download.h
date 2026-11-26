// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// CLI 'd' flight download job. cli_do_download_flight calls
// download_job_begin. AO_RCOS then calls download_job_step one time each
// 20 Hz tick until it returns true. One step has a fixed time bound and a
// fixed byte bound, so the QV loop and all AOs run between the steps in
// every flight phase. The pure plan logic is in logging/flash_job_plan.h
// (download plan) and has host tests.
#ifndef ROCKETCHIP_LOGGING_FLIGHT_DOWNLOAD_H
#define ROCKETCHIP_LOGGING_FLIGHT_DOWNLOAD_H

#include <stdint.h>

#include "logging/flash_job_plan.h"
#include "logging/flight_table.h"

namespace rc {

struct DownloadProgress {
    uint32_t bytes_sent;   // Payload bytes sent
    uint32_t total_bytes;  // Payload bytes in the flight
};

// Start the job for one flight table entry (the caller checks its layout).
// kBusy if a download, flush, or erase job is in progress.
DownloadBegin download_job_begin(const FlightLogEntry& entry,
                                 uint32_t flight_num);

// One bounded step. Returns true when the job ended (RCEND or RCERR).
bool download_job_step();

bool download_job_active();

// Stop the job now. Sends RCERR:ABORT:<frames> through rc_log.
void download_job_abort();

DownloadProgress download_job_progress();

} // namespace rc

#endif // ROCKETCHIP_LOGGING_FLIGHT_DOWNLOAD_H
