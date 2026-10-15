/* Host-only names for the musl float trig in this directory.
   Include this after <cmath>. It renames the C99 float functions
   the ESKF replay path calls. Firmware does not include it, so
   the flight image keeps the toolchain libm. */
#ifndef RC_HOST_TRIG_H
#define RC_HOST_TRIG_H

#ifdef __cplusplus
extern "C" {
#endif

float rc_host_sinf(float x);
float rc_host_cosf(float x);
float rc_host_atanf(float x);
float rc_host_atan2f(float y, float x);
float rc_host_asinf(float x);
float rc_host_acosf(float x);

#ifdef __cplusplus
}
#endif

#define sinf rc_host_sinf
#define cosf rc_host_cosf
#define atanf rc_host_atanf
#define atan2f rc_host_atan2f
#define asinf rc_host_asinf
#define acosf rc_host_acosf

#endif
