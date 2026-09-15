#ifndef SM_POWER_METER_H
#define SM_POWER_METER_H

#include "sm_common.h"

/* Exponential-style single-pole power meter with a 'current' signal level.
   Modeled on the classic PSTN power meter: power responds with an attack/decay
   time constant, and can be read in dBm0 when scaled correctly. Here we keep
   the raw mean-square in 'power' and provide conversion helpers. */
typedef struct {
    double power;
    double decay;
} sm_power_meter_t;

static inline void sm_power_meter_init(sm_power_meter_t *m, double time_constant_s)
{
    m->power = 0.0;
    m->decay = exp(-1.0 / (time_constant_s * SM_SAMPLE_RATE));
}

/* Feed one sample; returns the current signal power (mean square). */
static inline double sm_power_meter_update(sm_power_meter_t *m, int16_t sample)
{
    double s = (double) sample;
    m->power = m->power * m->decay + (1.0 - m->decay) * (s * s);
    return m->power;
}

static inline double sm_power_meter_current(const sm_power_meter_t *m)
{
    return m->power;
}

/* Convert to full-scale-16bit dB relative: dB = 10*log10(power/32768^2) */
static inline double sm_power_meter_db(const sm_power_meter_t *m)
{
    return 10.0 * log10(m->power / (32768.0 * 32768.0));
}

#endif