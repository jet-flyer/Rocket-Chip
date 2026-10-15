/* Glue for the musl 1.2.5 float trig sources in this directory.
   The bit casts match musl src/internal/libm.h. They use the native
   IEEE-754 representation, so little-endian and big-endian hosts
   both see the same exponent and sign masks. */
#ifndef RC_HOST_LIBM_H
#define RC_HOST_LIBM_H

#include <float.h>
#include <math.h>
#include <stdint.h>

/* pi/2 with the bits the musl sources name in the comment
   (0x3FF921FB, 0x54442D18). A decimal M_PI_2 can round differently. */
#ifndef M_PI_2
#define M_PI_2 0x1.921fb54442d18p+0
#endif

int __rem_pio2f(float x, double *y);
int __rem_pio2_large(double *x, double *y, int e0, int nx, int prec);
float __sindf(double x);
float __cosdf(double x);

#define predict_true(x) (x)
#define predict_false(x) (x)

static inline void fp_force_evalf(float x)
{
    volatile float y = x;
    (void)y;
}

static inline void fp_force_eval(double x)
{
    volatile double y = x;
    (void)y;
}

#define FORCE_EVAL(x)                                   \
    do {                                                \
        if (sizeof(x) == sizeof(float))                 \
            fp_force_evalf(x);                          \
        else                                            \
            fp_force_eval(x);                           \
    } while (0)

#define asuint(f) ((union { float _f; uint32_t _i; }){f})._i
#define asfloat(i) ((union { uint32_t _i; float _f; }){i})._f

#define GET_FLOAT_WORD(w, d) \
    do {                     \
        (w) = asuint(d);     \
    } while (0)

#define SET_FLOAT_WORD(d, w) \
    do {                     \
        (d) = asfloat(w);    \
    } while (0)

#endif
