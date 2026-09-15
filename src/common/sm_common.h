#ifndef SM_COMMON_H
#define SM_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#define SM_PI       3.14159265358979323846
#define SM_TWO_PI   6.28318530717958647692

#ifndef MIN
#define MIN(a, b)   ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)   ((a) > (b) ? (a) : (b))
#endif
#ifndef CLAMP
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

#define ARRAY_LEN(a)  (sizeof(a)/sizeof((a)[0]))

#define SM_SAMPLE_RATE  8000

typedef struct {
    float re;
    float im;
} sm_complexf_t;

static inline sm_complexf_t sm_cf(float re, float im)              { sm_complexf_t c = {re, im}; return c; }
static inline sm_complexf_t sm_cf_add(sm_complexf_t a, sm_complexf_t b) { sm_complexf_t c = {a.re+b.re, a.im+b.im}; return c; }
static inline sm_complexf_t sm_cf_sub(sm_complexf_t a, sm_complexf_t b) { sm_complexf_t c = {a.re-b.re, a.im-b.im}; return c; }
static inline sm_complexf_t sm_cf_mul(sm_complexf_t a, sm_complexf_t b) { sm_complexf_t c = {a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re}; return c; }
static inline sm_complexf_t sm_cf_scale(sm_complexf_t a, float s)  { sm_complexf_t c = {a.re*s, a.im*s}; return c; }
static inline float sm_cf_abs(sm_complexf_t a)                    { return sqrtf(a.re*a.re + a.im*a.im); }
static inline float sm_cf_abs_sq(sm_complexf_t a)                 { return a.re*a.re + a.im*a.im; }
static inline sm_complexf_t sm_cf_conj(sm_complexf_t a)           { sm_complexf_t c = {a.re, -a.im}; return c; }
static inline sm_complexf_t sm_cf_exp(float phase)                { sm_complexf_t c = {cosf(phase), sinf(phase)}; return c; }

static inline int16_t sm_sat16(float x)
{
    if (x >= 32767.0f)  return 32767;
    if (x <= -32768.0f) return -32768;
    return (int16_t) lrintf(x);
}

#endif