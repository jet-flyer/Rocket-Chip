/**
 * @file AP_Param.h
 * @brief Stub AP_Param for RocketChip when AP_PARAM_ENABLED=0
 *
 * Provides minimal types and macros to satisfy ArduPilot library compilation.
 * All parameter functionality is stubbed out.
 */
#pragma once

#include <cstdint>

// Ensure AP_PARAM_ENABLED is defined
#ifndef AP_PARAM_ENABLED
#define AP_PARAM_ENABLED 0
#endif

// Parameter type enum (used by some ArduPilot code)
enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8    = 1,
    AP_PARAM_INT16   = 2,
    AP_PARAM_INT32   = 3,
    AP_PARAM_FLOAT   = 4,
    AP_PARAM_VECTOR3F = 5,
    AP_PARAM_GROUP   = 6,
};

#if !AP_PARAM_ENABLED

// Stub parameter group info
struct AP_Param_Info {};
struct GroupInfo {};

// Base parameter class - all operations are no-ops
class AP_Param {
public:
    // Parameter table loading (stub)
    static void setup_object_defaults(const void *object_pointer, const struct GroupInfo *group_info) {
        (void)object_pointer;
        (void)group_info;
    }

    static void setup_sketch_defaults() {}

    // Load/save (stubs)
    static void load_all() {}
    static void invalidate_count() {}

    // Count parameters
    static uint16_t count_parameters() { return 0; }

    // Find parameter by name (stub - returns nullptr)
    static AP_Param* find(const char* name, enum ap_var_type* ptype = nullptr) {
        (void)name;
        if (ptype) *ptype = AP_PARAM_NONE;
        return nullptr;
    }

    // Set value (stub)
    void set(float value) { (void)value; }
    void save() {}
    void load() {}

    // Conversion stubs
    virtual float cast_to_float() const { return 0.0f; }
};

// Parameter template class
template<typename T>
class AP_ParamT : public AP_Param {
public:
    AP_ParamT() : _value() {}
    AP_ParamT(T default_value) : _value(default_value) {}

    T get() const { return _value; }
    void set(T value) { _value = value; }
    void set_and_save(T value) { _value = value; }
    void set_default(T value) { _value = value; }

    operator T() const { return _value; }
    AP_ParamT<T>& operator=(const T& value) { _value = value; return *this; }

    float cast_to_float() const override { return static_cast<float>(_value); }

private:
    T _value;
};

// Concrete parameter types
typedef AP_ParamT<float> AP_Float;
typedef AP_ParamT<int8_t> AP_Int8;
typedef AP_ParamT<int16_t> AP_Int16;
typedef AP_ParamT<int32_t> AP_Int32;

// Parameter group macros (expand to nothing when disabled)
#define AP_GROUPINFO(name, idx, class_name, member, default_val)
#define AP_GROUPINFO_FLAGS(name, idx, class_name, member, default_val, flags)
#define AP_GROUPEND
#define AP_VAREND

#define AP_PARAM_TABLE_NAME(name) static const struct GroupInfo name[]
#define AP_PARAM_DEFAULTS_ONLY 0
#define AP_PARAM_FLAG_ENABLE 0
#define AP_PARAM_NO_SHIFT 0

// Parameter defaults (stub)
class ParametersG2 {};
class Parameters {};

#endif // !AP_PARAM_ENABLED
