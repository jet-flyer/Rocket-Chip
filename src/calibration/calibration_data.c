/**
 * @file calibration_data.c
 * @brief Calibration data handling implementation
 */

#include "calibration_data.h"
#include <string.h>

// ============================================================================
// CRC16 (CCITT polynomial)
// ============================================================================

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
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
    if (cal == NULL) return;

    memset(cal, 0, sizeof(calibration_store_t));

    cal->magic = CALIBRATION_MAGIC;
    cal->version = CALIBRATION_VERSION;

    // Accel defaults: no offset, unity scale
    cal->accel.offset = (cal_vec3_t){0.0f, 0.0f, 0.0f};
    cal->accel.scale = (cal_vec3_t){1.0f, 1.0f, 1.0f};
    cal->accel.status = CAL_STATUS_NONE;

    // Gyro defaults: no bias
    cal->gyro.bias = (cal_vec3_t){0.0f, 0.0f, 0.0f};
    cal->gyro.status = CAL_STATUS_NONE;

    // Mag defaults: no hard iron, identity soft iron
    cal->mag.hard_iron = (cal_vec3_t){0.0f, 0.0f, 0.0f};
    // Identity matrix for soft iron
    cal->mag.soft_iron[0] = 1.0f; cal->mag.soft_iron[1] = 0.0f; cal->mag.soft_iron[2] = 0.0f;
    cal->mag.soft_iron[3] = 0.0f; cal->mag.soft_iron[4] = 1.0f; cal->mag.soft_iron[5] = 0.0f;
    cal->mag.soft_iron[6] = 0.0f; cal->mag.soft_iron[7] = 0.0f; cal->mag.soft_iron[8] = 1.0f;
    cal->mag.declination_deg = 0.0f;
    cal->mag.status = CAL_STATUS_NONE;

    // Baro defaults: standard atmosphere
    cal->baro.ground_pressure_pa = 101325.0f;
    cal->baro.ground_temperature_c = 20.0f;
    cal->baro.altitude_offset_m = 0.0f;
    cal->baro.status = CAL_STATUS_NONE;

    cal->cal_flags = 0;
    cal->cal_timestamp = 0;

    calibration_update_crc(cal);
}

bool calibration_validate(const calibration_store_t* cal) {
    if (cal == NULL) return false;

    // Check magic
    if (cal->magic != CALIBRATION_MAGIC) return false;

    // Check version (allow current or older versions we can migrate)
    if (cal->version > CALIBRATION_VERSION) return false;

    // Check CRC
    // CRC is computed over everything after the crc16 field
    const uint8_t* data_start = (const uint8_t*)&cal->accel;
    size_t data_len = sizeof(calibration_store_t) - offsetof(calibration_store_t, accel);
    uint16_t computed_crc = crc16_ccitt(data_start, data_len);

    return (cal->crc16 == computed_crc);
}

void calibration_update_crc(calibration_store_t* cal) {
    if (cal == NULL) return;

    // CRC is computed over everything after the crc16 field
    const uint8_t* data_start = (const uint8_t*)&cal->accel;
    size_t data_len = sizeof(calibration_store_t) - offsetof(calibration_store_t, accel);
    cal->crc16 = crc16_ccitt(data_start, data_len);
}

bool calibration_has(const calibration_store_t* cal, cal_status_flags_t flag) {
    if (cal == NULL) return false;
    return (cal->cal_flags & flag) != 0;
}
