/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (c) 2025-2026 Rocket Chip Project
   Calls the musl sinf/cosf/atanf/atan2f/asinf/acosf in this directory.
   Compiled with -fno-builtin so these are real calls to those
   definitions, not a compiler builtin that would reach libc. */
float sinf(float);
float cosf(float);
float atanf(float);
float atan2f(float, float);
float asinf(float);
float acosf(float);

float rc_host_sinf(float x) { return sinf(x); }
float rc_host_cosf(float x) { return cosf(x); }
float rc_host_atanf(float x) { return atanf(x); }
float rc_host_atan2f(float y, float x) { return atan2f(y, x); }
float rc_host_asinf(float x) { return asinf(x); }
float rc_host_acosf(float x) { return acosf(x); }
