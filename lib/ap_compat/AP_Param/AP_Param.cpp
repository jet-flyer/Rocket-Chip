/**
 * @file AP_Param.cpp
 * @brief ArduPilot-compatible parameter system implementation
 *
 * Implements persistent parameter storage using AP_HAL::Storage.
 */

// Include HAL_RP2350 FIRST to set macros before AP_HAL.h fallbacks
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include "AP_Param.h"
#include <cstring>
#include <cmath>

// ============================================================================
// AP_Param Static Members
// ============================================================================

bool AP_Param::_initialized = false;
bool AP_Param::_erased_on_boot = false;
uint16_t AP_Param::_cal_flags = 0;

// ============================================================================
// AP_Param Implementation
// ============================================================================

void AP_Param::setup()
{
    if (_initialized) {
        return;
    }

    // Initialize storage subsystem if not already done
    // (normally done via hal.init(), but be safe)
    if (!hal.storage.healthy()) {
        hal.storage.init();
    }

    // Validate or initialize header
    if (!validate_header()) {
        // Storage is empty or corrupt - initialize fresh
        _erased_on_boot = true;
        erase_all();
    }

    // Load calibration flags
    uint16_t flags_temp;
    hal.storage.read_block(&flags_temp, RocketChip::kHeaderFlagsOffset, sizeof(flags_temp));
    _cal_flags = flags_temp;
    printf("[AP_Param] Loaded cal_flags = 0x%04X\n", _cal_flags);

    _initialized = true;
}

bool AP_Param::validate_header()
{
    uint32_t magic;
    uint16_t version;

    hal.storage.read_block(&magic, RocketChip::kHeaderMagicOffset, sizeof(magic));
    hal.storage.read_block(&version, RocketChip::kHeaderVersionOffset, sizeof(version));

    return (magic == RocketChip::kParamMagic) && (version == RocketChip::kParamVersion);
}

void AP_Param::write_header()
{
    uint32_t magic = RocketChip::kParamMagic;
    uint16_t version = RocketChip::kParamVersion;

    hal.storage.write_block(RocketChip::kHeaderMagicOffset, &magic, sizeof(magic));
    hal.storage.write_block(RocketChip::kHeaderVersionOffset, &version, sizeof(version));
    hal.storage.write_block(RocketChip::kHeaderFlagsOffset, &_cal_flags, sizeof(_cal_flags));
}

void AP_Param::erase_all()
{
    // Zero out entire parameter region
    uint8_t zeros[32];
    memset(zeros, 0, sizeof(zeros));

    // Erase in 32-byte chunks up to mission base
    for (uint16_t offset = 0; offset < RocketChip::kConfigEnd; offset += sizeof(zeros)) {
        hal.storage.write_block(offset, zeros, sizeof(zeros));
    }

    // Reset flags
    _cal_flags = 0;

    // Write fresh header
    write_header();

    // Flush to flash
    hal.storage._timer_tick();
}

uint16_t AP_Param::get_cal_flags()
{
    return _cal_flags;
}

void AP_Param::set_cal_flag(uint16_t flag)
{
    _cal_flags |= flag;

    // Persist to storage
    hal.storage.write_block(RocketChip::kHeaderFlagsOffset, &_cal_flags, sizeof(_cal_flags));
}

void AP_Param::clear_cal_flag(uint16_t flag)
{
    _cal_flags &= ~flag;

    // Persist to storage
    hal.storage.write_block(RocketChip::kHeaderFlagsOffset, &_cal_flags, sizeof(_cal_flags));
}

void AP_Param::flush()
{
    hal.storage._timer_tick();
}

// ============================================================================
// AP_Float Implementation
// ============================================================================

AP_Float::AP_Float(uint16_t storage_offset)
    : _offset(storage_offset), _value(0.0f)
{
}

AP_Float::AP_Float(uint16_t storage_offset, float default_value)
    : _offset(storage_offset), _value(default_value)
{
}

void AP_Float::save()
{
    if (_offset == 0) {
        return;  // No storage assigned
    }
    hal.storage.write_block(_offset, &_value, sizeof(_value));
}

void AP_Float::load()
{
    if (_offset == 0) {
        return;  // No storage assigned
    }
    hal.storage.read_block(&_value, _offset, sizeof(_value));

    // Check for NaN/Inf and reset to 0 if invalid
    if (!std::isfinite(_value)) {
        _value = 0.0f;
    }
}

void AP_Float::set_and_save(float v)
{
    _value = v;
    save();
}

void AP_Float::set_and_save_ifchanged(float v)
{
    // Check if value actually changed (within epsilon)
    constexpr float kEpsilon = 1e-6f;
    if (std::fabs(_value - v) > kEpsilon) {
        set_and_save(v);
    }
}

// ============================================================================
// AP_Int8 Implementation
// ============================================================================

AP_Int8::AP_Int8(uint16_t storage_offset)
    : _offset(storage_offset), _value(0)
{
}

AP_Int8::AP_Int8(uint16_t storage_offset, int8_t default_value)
    : _offset(storage_offset), _value(default_value)
{
}

void AP_Int8::save()
{
    if (_offset == 0) {
        return;
    }
    hal.storage.write_block(_offset, &_value, sizeof(_value));
}

void AP_Int8::load()
{
    if (_offset == 0) {
        return;
    }
    hal.storage.read_block(&_value, _offset, sizeof(_value));
}

void AP_Int8::set_and_save(int8_t v)
{
    _value = v;
    save();
}

// ============================================================================
// AP_Int16 Implementation
// ============================================================================

AP_Int16::AP_Int16(uint16_t storage_offset)
    : _offset(storage_offset), _value(0)
{
}

AP_Int16::AP_Int16(uint16_t storage_offset, int16_t default_value)
    : _offset(storage_offset), _value(default_value)
{
}

void AP_Int16::save()
{
    if (_offset == 0) {
        return;
    }
    hal.storage.write_block(_offset, &_value, sizeof(_value));
}

void AP_Int16::load()
{
    if (_offset == 0) {
        return;
    }
    hal.storage.read_block(&_value, _offset, sizeof(_value));
}

void AP_Int16::set_and_save(int16_t v)
{
    _value = v;
    save();
}

// ============================================================================
// AP_Int32 Implementation
// ============================================================================

AP_Int32::AP_Int32(uint16_t storage_offset)
    : _offset(storage_offset), _value(0)
{
}

AP_Int32::AP_Int32(uint16_t storage_offset, int32_t default_value)
    : _offset(storage_offset), _value(default_value)
{
}

void AP_Int32::save()
{
    if (_offset == 0) {
        return;
    }
    hal.storage.write_block(_offset, &_value, sizeof(_value));
}

void AP_Int32::load()
{
    if (_offset == 0) {
        return;
    }
    hal.storage.read_block(&_value, _offset, sizeof(_value));
}

void AP_Int32::set_and_save(int32_t v)
{
    _value = v;
    save();
}

// ============================================================================
// AP_Vector3f Implementation
// ============================================================================

AP_Vector3f::AP_Vector3f(uint16_t storage_offset)
    : _offset(storage_offset), _value()
{
}

AP_Vector3f::AP_Vector3f(uint16_t storage_offset, const Vector3f& default_value)
    : _offset(storage_offset), _value(default_value)
{
}

void AP_Vector3f::save()
{
    if (_offset == 0) {
        return;  // No storage assigned
    }

    // Write each component (3 floats = 12 bytes)
    hal.storage.write_block(_offset, &_value.x, sizeof(float));
    hal.storage.write_block(_offset + sizeof(float), &_value.y, sizeof(float));
    hal.storage.write_block(_offset + 2 * sizeof(float), &_value.z, sizeof(float));
}

void AP_Vector3f::load()
{
    if (_offset == 0) {
        return;  // No storage assigned
    }

    // Read each component
    hal.storage.read_block(&_value.x, _offset, sizeof(float));
    hal.storage.read_block(&_value.y, _offset + sizeof(float), sizeof(float));
    hal.storage.read_block(&_value.z, _offset + 2 * sizeof(float), sizeof(float));

    // Check for NaN/Inf and reset to 0 if invalid
    if (!std::isfinite(_value.x)) _value.x = 0.0f;
    if (!std::isfinite(_value.y)) _value.y = 0.0f;
    if (!std::isfinite(_value.z)) _value.z = 0.0f;
}

void AP_Vector3f::set_and_save(const Vector3f& v)
{
    _value = v;
    save();
}

void AP_Vector3f::set_and_save_ifchanged(const Vector3f& v)
{
    // Check if any component changed
    constexpr float kEpsilon = 1e-6f;
    if (std::fabs(_value.x - v.x) > kEpsilon ||
        std::fabs(_value.y - v.y) > kEpsilon ||
        std::fabs(_value.z - v.z) > kEpsilon) {
        set_and_save(v);
    }
}
