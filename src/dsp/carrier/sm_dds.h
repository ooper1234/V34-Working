#ifndef SM_DDS_H
#define SM_DDS_H

#include "sm_common.h"

/* Numerically controlled oscillator. phase_range = 2^32. Phase rate in
   units of phase per sample: rate = freq / sample_rate * 2^32. */
typedef struct {
    uint32_t phase;
    int32_t rate;
} sm_dds_t;

static inline void sm_dds_init(sm_dds_t *d, float freq_hz)
{
    d->phase = 0;
    d->rate = (int32_t) (freq_hz / (float) SM_SAMPLE_RATE * 4294967296.0f);
}

static inline void sm_dds_set_freq(sm_dds_t *d, float freq_hz)
{
    d->rate = (int32_t) (freq_hz / (float) SM_SAMPLE_RATE * 4294967296.0f);
}

static inline void sm_dds_advance(sm_dds_t *d)
{
    d->phase += (uint32_t) d->rate;
}

static inline float sm_dds_freq(const sm_dds_t *d)
{
    return (float) ((int32_t) d->rate) / 4294967296.0f * (float) SM_SAMPLE_RATE;
}

static inline sm_complexf_t sm_dds_complex(const sm_dds_t *d)
{
    float ph = (float) d->phase * (SM_TWO_PI / 4294967296.0f);
    return sm_cf(cosf(ph), sinf(ph));
}

static inline void sm_dds_rotate(sm_dds_t *d, int32_t step)
{
    d->phase += (uint32_t) step;
}

#endif