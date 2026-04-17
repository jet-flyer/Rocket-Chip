// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Diagnostic statistics dump for soak testing (IVP-132).

#ifndef BUILD_FOR_FLIGHT

#include "dev/diag_stats.h"
#include "active_objects/ao_radio.h"
#include "active_objects/ao_flight_director.h"
#include "active_objects/ao_health_monitor.h"
#include "active_objects/ao_led_engine.h"
#include "active_objects/ao_logger.h"
#include "active_objects/ao_notify.h"
#include "active_objects/ao_rcos.h"
#include "active_objects/ao_telemetry.h"
#include "safety/health_monitor.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/job.h"
#include "pico/time.h"
#include "hardware/structs/scb.h"
#include <stdio.h>

#ifdef ROCKETCHIP_JOB_STATION
#include "core1/sensor_core1.h"
#endif

extern "C" {
#include "qp_port.h"
}

// ARM CMSIS __get_MSP intrinsic — SDK provides via hardware/sync.h
static inline uint32_t msp_read() {
    uint32_t result;
    asm volatile("MRS %0, msp" : "=r"(result));
    return result;
}

extern sensor_seqlock_t g_sensorSeqlock;

// MSP tracking: track minimum-seen MSP value = deepest stack usage.
// Initialized to UINT32_MAX on first tick so any read seeds the baseline.
static uint32_t s_mspMin = 0xFFFFFFFFU;
static uint32_t s_mspInitial = 0;

void diag_stats_msp_tick() {
    uint32_t msp = msp_read();
    if (s_mspInitial == 0) { s_mspInitial = msp; }
    if (msp < s_mspMin) { s_mspMin = msp; }
}

static void dump_ao_queue(const char* name, QActive* ao) {
    if (ao == nullptr) {
        printf("  %-20s (null)\n", name);
        return;
    }
    const QEQueue* q = &ao->eQueue;
    uint16_t free_slots = QEQueue_getFree(q);
    uint16_t min_free = QEQueue_getMin(q);
    uint16_t use_now = QEQueue_getUse(q);
    uint16_t depth = free_slots + use_now;  // total slots
    uint16_t high_water = depth - min_free;  // max used ever
    printf("  %-20s depth=%u use=%u high=%u\n",
           name, depth, use_now, high_water);
}

extern "C" __attribute__((used))
void diag_stats_dump() {
    printf("\n=== Diagnostic Stats ===\n");

    printf("[MSP] initial=0x%08lx min=0x%08lx depth=%lu bytes\n",
           (unsigned long)s_mspInitial,
           (unsigned long)s_mspMin,
           (unsigned long)(s_mspInitial - s_mspMin));

    printf("[AO Queues]\n");
    dump_ao_queue("AO_Radio", AO_Radio);
    dump_ao_queue("AO_FlightDirector", AO_FlightDirector);
    dump_ao_queue("AO_HealthMonitor", AO_HealthMonitor);
    dump_ao_queue("AO_Notify", AO_Notify);
    dump_ao_queue("AO_Logger", AO_Logger);
    dump_ao_queue("AO_Telemetry", AO_Telemetry);
    dump_ao_queue("AO_LedEngine", AO_LedEngine);
    dump_ao_queue("AO_RCOS", AO_RCOS);

    printf("[Radio]\n");
    const RadioAoState* radio = AO_Radio_get_state();
    if (radio != nullptr) {
        printf("  tx=%lu rx=%lu rx_crc_err=%lu tx_consec_fail=%u relay=%lu\n",
               (unsigned long)radio->tx_count,
               (unsigned long)radio->rx_count,
               (unsigned long)radio->rx_crc_errors,
               (unsigned)radio->tx_consec_fail,
               (unsigned long)radio->relay_count);
        if constexpr (job::kRole == job::DeviceRole::kStation) {
            printf("  last_rssi=%d dBm last_snr=%d dB\n",
                   (int)radio->last_rx_rssi,
                   (int)radio->last_rx_snr);
        }
    }

#ifdef ROCKETCHIP_JOB_STATION
    // IVP-132a: station-specific diagnostics
    printf("[Station]\n");
    const RxTelemSnapshot* rx = AO_Telemetry_get_rx_state();
    if (rx != nullptr) {
        printf("  rx_valid=%s last_seq=%u met_ms=%lu\n",
               rx->valid ? "YES" : "NO",
               (unsigned)rx->seq,
               (unsigned long)rx->met_ms);
    }
    printf("  cmd_pending=%s\n",
           AO_Telemetry_is_cmd_pending() ? "YES" : "NO");
    printf("  station_gps: valid=%s sats=%u fix_type=%u\n",
           g_bestGpsValid.load() ? "YES" : "NO",
           (unsigned)g_bestGpsFix.satellites,
           (unsigned)g_bestGpsFix.fix_type);
#endif

    printf("[Health]\n");
    const rc::HealthState* h = rc::health_monitor_get_state();
    if (h != nullptr) {
        printf("  primary=0x%02x secondary=0x%02x go_nogo=%s\n",
               (unsigned)h->primary,
               (unsigned)h->secondary,
               h->go_nogo_ready ? "READY" : "NOT_READY");
    }

    printf("[Sensors]\n");
    shared_sensor_data_t snap;
    seqlock_read(&g_sensorSeqlock, &snap);
    printf("  IMU temp=%.2fC baro temp=%.2fC mcu temp=%.2fC\n",
           (double)snap.imu_temperature_c,
           (double)snap.baro_temperature_c,
           (double)snap.mcu_temperature_c);
    printf("  IMU reads=%lu errs=%lu  baro reads=%lu errs=%lu  gps reads=%lu errs=%lu\n",
           (unsigned long)snap.imu_read_count,
           (unsigned long)snap.imu_error_count,
           (unsigned long)snap.baro_read_count,
           (unsigned long)snap.baro_error_count,
           (unsigned long)snap.gps_read_count,
           (unsigned long)snap.gps_error_count);
    printf("  core1 loops=%lu\n", (unsigned long)snap.core1_loop_count);

    printf("[Uptime] %lu ms\n",
           (unsigned long)to_ms_since_boot(get_absolute_time()));
    printf("========================\n\n");
}

#endif // BUILD_FOR_FLIGHT
