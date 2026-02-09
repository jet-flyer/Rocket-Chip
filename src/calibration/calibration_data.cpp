/**
 * @file calibration_data.c
 * @brief Calibration data handling implementation
 */

#include "calibration_data.h"
#include <string.h>
#include <stddef.h>

// ============================================================================
// CRC16 (CCITT polynomial 0x1021)
// ============================================================================

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Public Functions
// ============================================================================

void calibration_init_defaults(calibration_store_t* cal) {
    if (cal == nullptr) return;

    memset(cal, 0, sizeof(calibration_store_t));

    cal->magic = kCalibrationMagic;
    cal->version = kCalibrationVersion;

    // Accel defaults: no offset, unity scale, no cross-coupling
    cal->accel.offset = cal_vec3_t{0.0F, 0.0F, 0.0F};
    cal->accel.scale = cal_vec3_t{1.0F, 1.0F, 1.0F};
    cal->accel.offdiag = cal_vec3_t{0.0F, 0.0F, 0.0F};
    cal->accel.status = CAL_STATUS_NONE;

    // Gyro defaults: no bias
    cal->gyro.bias = cal_vec3_t{0.0F, 0.0F, 0.0F};
    cal->gyro.status = CAL_STATUS_NONE;

    // Baro defaults: standard atmosphere
    cal->baro.ground_pressure_pa = 101325.0F;
    cal->baro.ground_temperature_c = 20.0F;
    cal->baro.status = CAL_STATUS_NONE;

    cal->cal_flags = 0;

    // Board orientation: identity matrix (no rotation)
    cal->board_rotation.m[0] = 1.0F; cal->board_rotation.m[1] = 0.0F; cal->board_rotation.m[2] = 0.0F;
    cal->board_rotation.m[3] = 0.0F; cal->board_rotation.m[4] = 1.0F; cal->board_rotation.m[5] = 0.0F;
    cal->board_rotation.m[6] = 0.0F; cal->board_rotation.m[7] = 0.0F; cal->board_rotation.m[8] = 1.0F;

    calibration_update_crc(cal);
}

bool calibration_validate(const calibration_store_t* cal) {
    if (cal == nullptr) return false;

    // Check magic
    if (cal->magic != kCalibrationMagic) return false;

    // Check version
    if (cal->version > kCalibrationVersion) return false;

    // Check CRC (computed over everything after crc16 field)
    const uint8_t* data_start = (const uint8_t*)&cal->accel;
    size_t data_len = sizeof(calibration_store_t) - offsetof(calibration_store_t, accel);
    uint16_t computed_crc = crc16_ccitt(data_start, data_len);

    return (cal->crc16 == computed_crc);
}

void calibration_update_crc(calibration_store_t* cal) {
    if (cal == nullptr) return;

    // CRC computed over everything after crc16 field
    const uint8_t* data_start = (const uint8_t*)&cal->accel;
    size_t data_len = sizeof(calibration_store_t) - offsetof(calibration_store_t, accel);
    cal->crc16 = crc16_ccitt(data_start, data_len);
}

bool calibration_has(const calibration_store_t* cal, cal_status_flags_t flag) {
    if (cal == nullptr) return false;
    return (cal->cal_flags & flag) != 0;
}
