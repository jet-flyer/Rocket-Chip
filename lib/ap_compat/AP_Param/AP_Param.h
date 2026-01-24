/**
 * @file AP_Param.h
 * @brief ArduPilot-compatible parameter system for RocketChip
 *
 * Provides persistent parameter storage via AP_HAL::Storage.
 * Parameters are stored at fixed offsets defined in ParamStorageLayout.h.
 *
 * Key types:
 * - AP_Float: Single float parameter
 * - AP_Int8, AP_Int16, AP_Int32: Integer parameters
 * - AP_Vector3f: 3-component vector (12 bytes)
 *
 * Usage:
 * @code
 * // Define parameter with storage offset
 * AP_Vector3f accel_offset(RocketChip::kAccelOffset0);
 *
 * // Load from flash
 * accel_offset.load();
 *
 * // Set and save
 * accel_offset.set_and_save(Vector3f(0.1f, -0.2f, 0.05f));
 * @endcode
 */

#pragma once

#include <cstdint>
#include <AP_Math/vector3.h>
#include "ParamStorageLayout.h"

// Forward declaration
namespace AP_HAL {
    class Storage;
}

// ============================================================================
// Parameter Types Enum
// ============================================================================

enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8    = 1,
    AP_PARAM_INT16   = 2,
    AP_PARAM_INT32   = 3,
    AP_PARAM_FLOAT   = 4,
    AP_PARAM_VECTOR3F = 5,
    AP_PARAM_VECTOR6F = 6,
    AP_PARAM_MATRIX3F = 7,
    AP_PARAM_GROUP   = 8
};

// ============================================================================
// AP_Param - Static Parameter Management
// ============================================================================

/**
 * @brief Static parameter management class
 *
 * Handles initialization and bulk operations on parameter storage.
 */
class AP_Param {
public:
    /**
     * @brief Initialize parameter storage
     *
     * Validates storage header, initializes if empty.
     * Must be called before any parameter load/save operations.
     */
    static void setup();

    /**
     * @brief Check if parameter system is initialized
     */
    static bool initialized() { return _initialized; }

    /**
     * @brief Erase all parameter storage
     *
     * Resets storage to default state. Use with caution.
     */
    static void erase_all();

    /**
     * @brief Get calibration validity flags
     */
    static uint16_t get_cal_flags();

    /**
     * @brief Set calibration validity flag
     * @param flag Flag to set (from kCalFlag* constants)
     */
    static void set_cal_flag(uint16_t flag);

    /**
     * @brief Clear calibration validity flag
     */
    static void clear_cal_flag(uint16_t flag);

    /**
     * @brief Force flush pending writes to flash
     */
    static void flush();

    /**
     * @brief Check if storage was erased on boot (header was invalid)
     */
    static bool erased_on_boot() { return _erased_on_boot; }

    // Legacy compatibility (no-op)
    static void load_all() {}
    static void save() { flush(); }

private:
    static bool _initialized;
    static bool _erased_on_boot;
    static uint16_t _cal_flags;

    static void write_header();
    static bool validate_header();
};

// ============================================================================
// AP_Float - Single Float Parameter
// ============================================================================

/**
 * @brief Persistent float parameter
 *
 * Stores a single float value at a fixed storage offset.
 */
class AP_Float {
public:
    /**
     * @brief Construct with storage offset
     * @param storage_offset Byte offset in parameter storage
     */
    explicit AP_Float(uint16_t storage_offset);

    /**
     * @brief Construct with storage offset and default value
     */
    AP_Float(uint16_t storage_offset, float default_value);

    /**
     * @brief Default constructor (for array initialization)
     */
    AP_Float() : _offset(0), _value(0.0f) {}

    /**
     * @brief Construct with just default value (legacy compatibility)
     */
    AP_Float(float v) : _offset(0), _value(v) {}

    /**
     * @brief Set storage offset (for deferred initialization)
     */
    void set_offset(uint16_t offset) { _offset = offset; }

    /**
     * @brief Set value in RAM (does not save to flash)
     */
    void set(float v) { _value = v; }

    /**
     * @brief Get current value
     */
    float get() const { return _value; }

    /**
     * @brief Save current value to flash
     */
    void save();

    /**
     * @brief Load value from flash
     */
    void load();

    /**
     * @brief Set value and save to flash
     */
    void set_and_save(float v);

    /**
     * @brief Set and save only if value changed
     */
    void set_and_save_ifchanged(float v);

    /**
     * @brief Implicit conversion to float
     */
    operator float() const { return _value; }

    /**
     * @brief Assignment operator
     */
    AP_Float& operator=(float v) { set(v); return *this; }

private:
    uint16_t _offset;
    float _value;
};

// ============================================================================
// AP_Int8 - 8-bit Integer Parameter
// ============================================================================

class AP_Int8 {
public:
    explicit AP_Int8(uint16_t storage_offset);
    AP_Int8(uint16_t storage_offset, int8_t default_value);
    AP_Int8() : _offset(0), _value(0) {}

    void set_offset(uint16_t offset) { _offset = offset; }
    void set(int8_t v) { _value = v; }
    int8_t get() const { return _value; }
    void save();
    void load();
    void set_and_save(int8_t v);

    operator int8_t() const { return _value; }
    AP_Int8& operator=(int8_t v) { set(v); return *this; }

private:
    uint16_t _offset;
    int8_t _value;
};

// ============================================================================
// AP_Int16 - 16-bit Integer Parameter
// ============================================================================

class AP_Int16 {
public:
    explicit AP_Int16(uint16_t storage_offset);
    AP_Int16(uint16_t storage_offset, int16_t default_value);
    AP_Int16() : _offset(0), _value(0) {}

    void set_offset(uint16_t offset) { _offset = offset; }
    void set(int16_t v) { _value = v; }
    int16_t get() const { return _value; }
    void save();
    void load();
    void set_and_save(int16_t v);

    operator int16_t() const { return _value; }
    AP_Int16& operator=(int16_t v) { set(v); return *this; }

private:
    uint16_t _offset;
    int16_t _value;
};

// ============================================================================
// AP_Int32 - 32-bit Integer Parameter
// ============================================================================

class AP_Int32 {
public:
    explicit AP_Int32(uint16_t storage_offset);
    AP_Int32(uint16_t storage_offset, int32_t default_value);
    AP_Int32() : _offset(0), _value(0) {}

    void set_offset(uint16_t offset) { _offset = offset; }
    void set(int32_t v) { _value = v; }
    int32_t get() const { return _value; }
    void save();
    void load();
    void set_and_save(int32_t v);

    operator int32_t() const { return _value; }
    AP_Int32& operator=(int32_t v) { set(v); return *this; }

private:
    uint16_t _offset;
    int32_t _value;
};

// ============================================================================
// AP_Vector3f - 3D Vector Parameter
// ============================================================================

/**
 * @brief Persistent 3D vector parameter
 *
 * Stores a Vector3f (12 bytes) at a fixed storage offset.
 * Used for calibration offsets, scales, etc.
 */
class AP_Vector3f {
public:
    /**
     * @brief Construct with storage offset
     * @param storage_offset Byte offset in parameter storage
     */
    explicit AP_Vector3f(uint16_t storage_offset);

    /**
     * @brief Construct with storage offset and default value
     */
    AP_Vector3f(uint16_t storage_offset, const Vector3f& default_value);

    /**
     * @brief Default constructor (for array initialization)
     */
    AP_Vector3f() : _offset(0), _value() {}

    /**
     * @brief Set storage offset (for deferred initialization)
     */
    void set_offset(uint16_t offset) { _offset = offset; }

    /**
     * @brief Set value in RAM (does not save to flash)
     */
    void set(const Vector3f& v) { _value = v; }

    /**
     * @brief Get current value
     */
    const Vector3f& get() const { return _value; }

    /**
     * @brief Save current value to flash
     */
    void save();

    /**
     * @brief Load value from flash
     */
    void load();

    /**
     * @brief Set value and save to flash
     */
    void set_and_save(const Vector3f& v);

    /**
     * @brief Set and save only if value changed
     */
    void set_and_save_ifchanged(const Vector3f& v);

    /**
     * @brief Implicit conversion to Vector3f
     */
    operator Vector3f() const { return _value; }

    /**
     * @brief Const reference access
     */
    operator const Vector3f&() const { return _value; }

    /**
     * @brief Component access
     */
    float& x() { return _value.x; }
    float& y() { return _value.y; }
    float& z() { return _value.z; }
    float x() const { return _value.x; }
    float y() const { return _value.y; }
    float z() const { return _value.z; }

private:
    uint16_t _offset;
    Vector3f _value;
};

// ============================================================================
// Legacy Compatibility Macros
// ============================================================================

// These macros are used by ArduPilot code but we don't need full implementation
#define AP_PARAMDEFV(vtype, ctype, ptype)
#define AP_GROUPINFO(name, idx, clss, var, def)
#define AP_SUBGROUPINFO(var, name, idx, clss, type)
