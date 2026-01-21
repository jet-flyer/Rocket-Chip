/**
 * @file AP_Param.h
 * @brief Minimal AP_Param stub for RocketChip
 *
 * AP_Param is ArduPilot's parameter system. We don't need it for calibration,
 * but some headers include it. This provides minimal stubs.
 */

#pragma once

#include <cstdint>

// Parameter types (minimal set)
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

// Stub class for AP_Float used in some template checks
class AP_Float {
public:
    AP_Float() : _value(0.0f) {}
    AP_Float(float v) : _value(v) {}
    operator float() const { return _value; }
    AP_Float& operator=(float v) { _value = v; return *this; }
private:
    float _value;
};

// Macro to define parameter type info (no-op for us)
#define AP_PARAMDEFV(vtype, ctype, ptype)

// Stub for AP_Param class
class AP_Param {
public:
    static void setup() {}
    static void load_all() {}
    static void save() {}
};
