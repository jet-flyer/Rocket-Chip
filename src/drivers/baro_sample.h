// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Rocket Chip Project
// Barometer poll result and new-sample watch, independent of the sensor
// part. Pure (no SDK) so it is host-tested (test/test_baro_poll.cpp).
//
// A driver poll reports one of three results. "No new sample" means the
// part has not finished a conversion since the last one was read. It is
// normal (the poll and the conversion clock are not locked together) and
// is not a fault: the caller publishes nothing, so consumers keyed on a new
// sample (the ESKF baro update counts baro_read_count) never fuse a re-read
// of the previous result. Only a gap longer than the part's own maximum
// between new samples means conversions stopped, and that is a fault.

#ifndef ROCKETCHIP_BARO_SAMPLE_H
#define ROCKETCHIP_BARO_SAMPLE_H

#include <stdint.h>

enum class BaroSampleStatus : uint8_t {
    kNewSample,    // a conversion finished since the last read; data filled
    kNoNewSample,  // bus fine, no conversion finished yet; data untouched
    kError,        // bus or driver failure; data not valid
};

// What the sensor loop does with one poll.
enum class BaroSampleAction : uint8_t {
    kPublish,  // new sample: store it, bump the sample count
    kSkip,     // no new sample yet: publish nothing, count nothing
    kFault,    // bus error, or no new sample for longer than the max gap
};

// Tracks time since the last new sample across polls.
class BaroSampleWatch {
public:
    explicit constexpr BaroSampleWatch(uint32_t max_gap_us)
        : maxGapUs_(max_gap_us) {}

    // Conversions were (re)started; give the part a full gap to deliver.
    constexpr void restart(uint32_t now_us) {
        lastNewUs_ = now_us;
        lastPollUs_ = now_us;
        started_ = true;
    }

    constexpr BaroSampleAction on_poll(BaroSampleStatus status, uint32_t now_us) {
        // First poll, or polling was paused (e.g. Core 1 held off the bus
        // for a flash operation) for longer than a gap: the time without
        // polls says nothing about the part, so restart the gap clock.
        if (!started_ || (now_us - lastPollUs_) > maxGapUs_) {
            restart(now_us);
        }
        lastPollUs_ = now_us;
        switch (status) {
            case BaroSampleStatus::kNewSample:
                lastNewUs_ = now_us;
                return BaroSampleAction::kPublish;
            case BaroSampleStatus::kNoNewSample:
                return ((now_us - lastNewUs_) > maxGapUs_)
                    ? BaroSampleAction::kFault
                    : BaroSampleAction::kSkip;
            case BaroSampleStatus::kError:
            default:
                return BaroSampleAction::kFault;
        }
    }

private:
    uint32_t maxGapUs_;
    uint32_t lastNewUs_ = 0;
    uint32_t lastPollUs_ = 0;
    bool started_ = false;
};

#endif // ROCKETCHIP_BARO_SAMPLE_H
