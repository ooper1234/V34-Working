#include "v22bis.h"
#include "../../dsp/carrier/sm_dds.h"
#include "../../dsp/detectors/sm_power_meter.h"
#include "../../dsp/filters/v22bis_filters.h"
#include "../../common/sm_common.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static int rx_trace_enabled(void)
{
    static int enabled = -1;
    if (enabled < 0)
        enabled = (getenv("SM_RX_TRACE") != NULL);
    return enabled;
}

/* V.22bis receiver. Reference architecture: SpanDSP v22bis_rx.c. All training
   timings and thresholds below come from that verified reference. */

static const int phase_steps[4] = {1, 0, 2, 3};

static const sm_complexf_t v22bis_constellation[16] = {
    {1.0f,  1.0f},   {3.0f,  1.0f},   {1.0f,  3.0f},   {3.0f,  3.0f},
    {-1.0f, 1.0f},   {-1.0f, 3.0f},   {-3.0f, 1.0f},   {-3.0f, 3.0f},
    {-1.0f, -1.0f},  {-3.0f, -1.0f},  {-1.0f, -3.0f},  {-3.0f, -3.0f},
    {1.0f,  -1.0f},  {1.0f,  -3.0f},  {3.0f,  -1.0f},  {3.0f,  -3.0f}
};

static const sm_complexf_t rot45 = {0.894427f, 0.44721f};   /* cos(26.565 deg) */

/* 16-QAM slicing table: indices of closest constellation point for each
   integer cell in the [-1..5]x[-1..5] grid. Same scheme as the reference's
   space_map_v22bis[][]. */
static const uint8_t space_map_v22bis[6][6] = {
    {11,  9,  9,  6,  6,  7},
    {10,  8,  8,  4,  4,  5},
    {10,  8,  8,  4,  4,  5},
    {13, 12, 12,  0,  0,  2},
    {13, 12, 12,  0,  0,  2},
    {15, 14, 14,  1,  1,  3}
};

static void report_status(v22bis_state_t *s, v22bis_status_t status)
{
    if (s->status_handler)
        s->status_handler(s->status_user_data, status);
}

/* Descrambler: self-synchronizing, same taps as TX scrambler. */
static int descramble(v22bis_state_t *s, int bit)
{
    int out_bit;

    bit &= 1;
    out_bit = (bit ^ (s->rx.scramble_reg >> 13) ^ (s->rx.scramble_reg >> 16)) & 1;
    s->rx.scramble_reg = (s->rx.scramble_reg << 1) | bit;

    if (s->rx.scrambler_pattern_count >= 64)
    {
        out_bit ^= 1;
        s->rx.scrambler_pattern_count = 0;
    }
    if (bit)
        s->rx.scrambler_pattern_count++;
    else
        s->rx.scrambler_pattern_count = 0;
    return out_bit;
}

static void put_bit(v22bis_state_t *s, int bit)
{
    if (s->put_bit)
        s->put_bit(s->put_bit_user_data, descramble(s, bit));
}

static void equalizer_reset(v22bis_state_t *s)
{
    int i;

    for (i = 0; i < V22BIS_EQUALIZER_LEN; i++)
    {
        s->rx.eq_coeff[i].re = 0.0f;
        s->rx.eq_coeff[i].im = 0.0f;
        s->rx.eq_buf[i].re = 0.0f;
        s->rx.eq_buf[i].im = 0.0f;
    }
    /* Start with an equalizer based on everything being perfect: a 3.0+0.0j
       centre tap at the symbol point (matches the reference). */
    s->rx.eq_coeff[V22BIS_EQUALIZER_PRE_LEN].re = 3.0f;
    s->rx.eq_delta = 0.25f / V22BIS_EQUALIZER_LEN;
    s->rx.eq_put_step = 20 - 1;
    s->rx.eq_step = 0;
}

static sm_complexf_t equalizer_get(v22bis_state_t *s)
{
    sm_complexf_t z = {0.0f, 0.0f};
    int i;
    int j;

    j = s->rx.eq_step;
    for (i = 0; i < V22BIS_EQUALIZER_LEN; i++)
    {
        z.re += s->rx.eq_buf[j].re * s->rx.eq_coeff[i].re
              - s->rx.eq_buf[j].im * s->rx.eq_coeff[i].im;
        z.im += s->rx.eq_buf[j].re * s->rx.eq_coeff[i].im
              + s->rx.eq_buf[j].im * s->rx.eq_coeff[i].re;
        if (++j >= V22BIS_EQUALIZER_LEN)
            j = 0;
    }
    return z;
}

static void tune_equalizer(v22bis_state_t *s, const sm_complexf_t *z,
                           const sm_complexf_t *target)
{
    sm_complexf_t err;
    int i;
    int j;

    err.re = (target->re - z->re) * s->rx.eq_delta;
    err.im = (target->im - z->im) * s->rx.eq_delta;

    /* LMS update with circular buffer. */
    j = s->rx.eq_step;
    for (i = 0; i < V22BIS_EQUALIZER_LEN; i++)
    {
        s->rx.eq_coeff[i].re += s->rx.eq_buf[j].re * err.re
                              + s->rx.eq_buf[j].im * err.im;
        s->rx.eq_coeff[i].im += s->rx.eq_buf[j].re * err.im
                              - s->rx.eq_buf[j].im * err.re;
        if (++j >= V22BIS_EQUALIZER_LEN)
            j = 0;
    }
}

static void track_carrier(v22bis_state_t *s, const sm_complexf_t *z,
                          const sm_complexf_t *target)
{
    float error;

    /* Cross-product phase error: im(z*conj(target)) proportional to phase error. */
    error = z->im * target->re - z->re * target->im;
    s->rx.carrier_phase_rate += (int32_t) (s->rx.carrier_track_i * error);
    s->rx.carrier_phase += (int32_t) (s->rx.carrier_track_p * error);
}

static void decode_baud(v22bis_state_t *s, int nearest)
{
    int raw_bits;

    raw_bits = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];
    s->rx.constellation_state = nearest;
    /* First two bits are the quadrant code. */
    put_bit(s, raw_bits >> 1);
    put_bit(s, raw_bits & 1);
    if (s->rx.sixteen_way_decisions)
    {
        /* Remaining two bits are the position within the quadrant. */
        put_bit(s, nearest >> 1);
        put_bit(s, nearest & 1);
    }
}

static int decode_baudx(v22bis_state_t *s, int nearest)
{
    int raw_bits;
    int out_bits;

    raw_bits = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];
    s->rx.constellation_state = nearest;
    out_bits = descramble(s, raw_bits >> 1);
    out_bits = (out_bits << 1) | descramble(s, raw_bits & 1);
    if (s->rx.sixteen_way_decisions)
    {
        out_bits = (out_bits << 1) | descramble(s, nearest >> 1);
        out_bits = (out_bits << 1) | descramble(s, nearest & 1);
    }
    return out_bits;
}

/* Gardner symbol timing recovery. */
static void symbol_sync(v22bis_state_t *s)
{
    float p;
    float q;
    sm_complexf_t a;
    sm_complexf_t b;
    sm_complexf_t c;
    int aa[3];
    int i;
    int j;

    for (i = 0, j = s->rx.eq_step; i < 3; i++)
    {
        if (--j < 0)
            j = V22BIS_EQUALIZER_LEN - 1;
        aa[i] = j;
    }
    if (s->rx.sixteen_way_decisions)
    {
        p = s->rx.eq_buf[aa[2]].re - s->rx.eq_buf[aa[0]].re;
        p *= s->rx.eq_buf[aa[1]].re;
        q = s->rx.eq_buf[aa[2]].im - s->rx.eq_buf[aa[0]].im;
        q *= s->rx.eq_buf[aa[1]].im;
    }
    else
    {
        /* Rotate to 45-degree points to make Gardner more effective. */
        a = sm_cf_mul(s->rx.eq_buf[aa[2]], rot45);
        b = sm_cf_mul(s->rx.eq_buf[aa[1]], rot45);
        c = sm_cf_mul(s->rx.eq_buf[aa[0]], rot45);
        p = (a.re - c.re) * b.re;
        q = (a.im - c.im) * b.im;
    }
    s->rx.gardner_integrate += (p + q > 0) ? s->rx.gardner_step : -s->rx.gardner_step;

    if (abs(s->rx.gardner_integrate) >= 16)
    {
        /* Integrate-and-dump with hysteresis avoids jitter. */
        s->rx.eq_put_step += (s->rx.gardner_integrate / 16);
        s->rx.total_baud_timing_correction += (s->rx.gardner_integrate / 16);
        s->rx.gardner_integrate = 0;
    }
}

static void process_half_baud(v22bis_state_t *s, const sm_complexf_t *sample)
{
    sm_complexf_t z;
    sm_complexf_t zz;
    const sm_complexf_t *target;
    int re;
    int im;
    int nearest;
    int bitstream;
    int raw_bits;

    z = *sample;

    s->rx.eq_buf[s->rx.eq_step] = z;
    if (++s->rx.eq_step >= V22BIS_EQUALIZER_LEN)
        s->rx.eq_step = 0;

    /* On alternate insertions a whole baud is present. */
    if ((s->rx.baud_phase ^= 1))
        return;

    symbol_sync(s);
    z = equalizer_get(s);

    raw_bits = 0;
    if (s->rx.sixteen_way_decisions)
    {
        re = (int) (z.re + 3.0f);
        im = (int) (z.im + 3.0f);
        if (re > 5) re = 5;
        else if (re < 0) re = 0;
        if (im > 5) im = 5;
        else if (im < 0) im = 0;
        nearest = space_map_v22bis[re][im];
    }
    else
    {
        /* Rotate to 45 degrees to make slicing trivial for 4PSK. */
        zz = sm_cf_mul(z, rot45);
        nearest = 0x01;
        if (zz.re < 0)
            nearest |= 0x04;
        if (zz.im < 0)
        {
            nearest ^= 0x04;
            nearest |= 0x08;
        }
    }

    switch (s->rx.training)
    {
    case V22BIS_RX_TRAINING_NORMAL_OPERATION:
        target = &v22bis_constellation[nearest];
        track_carrier(s, &z, target);
        tune_equalizer(s, &z, target);
        raw_bits = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];

        /* Search for S1 signal requesting a retrain. */
        if (((s->rx.last_raw_bits ^ raw_bits) == 0x3))
            s->rx.pattern_repeats++;
        else
        {
            if (s->rx.pattern_repeats >= 50
                && (s->rx.last_raw_bits == 0x3 || s->rx.last_raw_bits == 0x0))
            {
                sm_log_message(&s->log, SM_LOG_FLOW,
                               "+++ S1 retrain detected (%d long)", s->rx.pattern_repeats);
                s->rx.pattern_repeats = 0;
                s->rx.training_count = 0;
                s->rx.training = V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200;
                s->tx.training_count = 0;
                s->tx.training = V22BIS_TX_TRAINING_U0011;
                equalizer_reset(s);
                report_status(s, V22BIS_STATUS_RETRAIN_OCCURRED);
            }
            s->rx.pattern_repeats = 0;
        }
        decode_baud(s, nearest);
        break;

    case V22BIS_RX_TRAINING_SYMBOL_ACQUISITION:
        target = &z;
        if (++s->rx.training_count >= 40)
        {
            if (rx_trace_enabled())
                fprintf(stderr, "TRACE acq_done bit_rate=%d\n", s->bit_rate);
            s->rx.gardner_step = 4;
            s->rx.pattern_repeats = 0;
            s->rx.training = s->calling_party
                                 ? V22BIS_RX_TRAINING_UNSCRAMBLED_ONES
                                 : V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200;
            s->negotiated_bit_rate = 1200;
        }
        else if (s->rx.training_count == 30)
        {
            s->rx.gardner_step = 32;
        }
        break;

    case V22BIS_RX_TRAINING_UNSCRAMBLED_ONES:
        /* Calling side only: receive unscrambled ones (or zeros) from answerer. */
        target = &v22bis_constellation[nearest];
        track_carrier(s, &z, target);
        raw_bits = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];
        s->rx.constellation_state = nearest;
        if (raw_bits != s->rx.last_raw_bits)
            s->rx.pattern_repeats = 0;
        else
            s->rx.pattern_repeats++;
        if (++s->rx.training_count == ms_to_symbols(155 + 456))
        {
            if (raw_bits == s->rx.last_raw_bits
                && (raw_bits == 0x3 || raw_bits == 0x0)
                && s->rx.pattern_repeats >= ms_to_symbols(456))
            {
                if (s->bit_rate == 2400)
                {
                    s->tx.training = V22BIS_TX_TRAINING_U0011;
                    s->tx.training_count = 0;
                }
                else
                {
                    s->tx.training = V22BIS_TX_TRAINING_S11;
                    s->tx.training_count = 0;
                }
            }
            s->rx.pattern_repeats = 0;
            s->rx.training_count = 0;
            s->rx.training = V22BIS_RX_TRAINING_UNSCRAMBLED_ONES_SUSTAINING;
        }
        break;

    case V22BIS_RX_TRAINING_UNSCRAMBLED_ONES_SUSTAINING:
        target = &v22bis_constellation[nearest];
        track_carrier(s, &z, target);
        raw_bits = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];
        s->rx.constellation_state = nearest;
        if (raw_bits != s->rx.last_raw_bits)
        {
            /* Unscrambled ones have ended. */
            s->tx.training_count = 0;
            s->tx.training = V22BIS_TX_TRAINING_TIMED_S11;
            s->rx.training_count = 0;
            s->rx.training = V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200;
            s->rx.pattern_repeats = 0;
        }
        break;

    case V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200:
        target = &v22bis_constellation[nearest];
        track_carrier(s, &z, target);
        tune_equalizer(s, &z, target);
        raw_bits = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];
        bitstream = decode_baudx(s, nearest);
        (void) bitstream;
        s->rx.training_count++;
        if (rx_trace_enabled())
            fprintf(stderr, "TRACE scr1200 n=%d raw=%d repeats=%d neg=%d\n",
                    s->rx.training_count, raw_bits, s->rx.pattern_repeats,
                    s->negotiated_bit_rate);

        if (s->negotiated_bit_rate == 1200)
        {
            /* Search for S1 (00/11 alternating) requesting 2400. The pattern
               is detected over a sliding window instead of requiring an
               uninterrupted run of clean symbols: real calling modems are
               often preceded by a plain carrier segment, so the equalizer is
               still settling when S1 arrives and a few symbols demodulate
               wrongly. Alternation in random data is 25%, so requiring 75%
               over 32 symbols cannot false-trigger. */
            s->rx.raw_history = (s->rx.raw_history << 2) | (uint64_t) raw_bits;
            s->rx.raw_hist_count++;
            if (s->rx.raw_hist_count >= 16)
            {
                int k;
                int alt = 0;
                for (k = 0; k < 31; k++)
                {
                    int a = (int) ((s->rx.raw_history >> (2 * k)) & 3u);
                    int b = (int) ((s->rx.raw_history >> (2 * (k + 1))) & 3u);
                    if ((a ^ b) == 3)
                        alt++;
                }
                if (alt >= 23
                    && (s->rx.last_raw_bits == 0x3 || s->rx.last_raw_bits == 0x0))
                    s->rx.pattern_repeats = alt;
                else
                    s->rx.pattern_repeats = 0;
            }
            if (s->rx.pattern_repeats >= 23
                && (s->rx.last_raw_bits == 0x3 || s->rx.last_raw_bits == 0x0))
            {
                sm_log_message(&s->log, SM_LOG_FLOW,
                               "+++ S1 detected (%d long)", s->rx.pattern_repeats);
                if (s->bit_rate == 2400)
                {
                    if (!s->calling_party)
                    {
                        s->tx.training = V22BIS_TX_TRAINING_U0011;
                        s->tx.training_count = 0;
                    }
                    s->negotiated_bit_rate = 2400;
                }
                s->rx.pattern_repeats = 0;
                s->rx.raw_hist_count = 0;
                s->rx.training_count = 0;
                break;
            }
            if (s->rx.training_count >= ms_to_symbols(400))
            {
                /* No S1 seen in time: commit to 1200 b/s. */
                if (s->calling_party)
                {
                    s->tx.training_count = 0;
                    s->tx.training = V22BIS_TX_TRAINING_TIMED_S11;
                    s->rx.training = V22BIS_RX_TRAINING_NORMAL_OPERATION;
                    s->rx.carrier_track_i = 8000.0f;
                }
                else
                {
                    s->tx.training_count = 0;
                    s->tx.training = V22BIS_TX_TRAINING_TIMED_S11;
                    s->rx.training = V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200_SUSTAINING;
                }
            }
        }
        else
        {
            if (s->calling_party)
            {
                if (s->rx.training_count >= ms_to_symbols(100 + 450))
                {
                    s->rx.sixteen_way_decisions = true;
                    s->rx.training = V22BIS_RX_TRAINING_WAIT_FOR_SCRAMBLED_ONES_AT_2400;
                    s->rx.pattern_repeats = 0;
                    s->rx.carrier_track_i = 8000.0f;
                }
            }
            else
            {
                if (s->rx.training_count >= ms_to_symbols(450))
                {
                    s->rx.sixteen_way_decisions = true;
                    s->rx.training = V22BIS_RX_TRAINING_WAIT_FOR_SCRAMBLED_ONES_AT_2400;
                    s->rx.pattern_repeats = 0;
                }
            }
        }
        break;

    case V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200_SUSTAINING:
        target = &v22bis_constellation[nearest];
        track_carrier(s, &z, target);
        tune_equalizer(s, &z, target);
        (void) decode_baudx(s, nearest);
        if (++s->rx.training_count > ms_to_symbols(270 + 765))
        {
            s->rx.training = V22BIS_RX_TRAINING_NORMAL_OPERATION;
        }
        break;

    case V22BIS_RX_TRAINING_WAIT_FOR_SCRAMBLED_ONES_AT_2400:
        target = &v22bis_constellation[nearest];
        track_carrier(s, &z, target);
        tune_equalizer(s, &z, target);
        bitstream = decode_baudx(s, nearest);
        /* Need 32 sustained 1s to enter 2400 data mode. */
        if (bitstream == 0xF)
        {
            if (++s->rx.pattern_repeats >= 9)
            {
                sm_log_message(&s->log, SM_LOG_FLOW, "+++ Rx normal operation (2400)");
                s->rx.training = V22BIS_RX_TRAINING_NORMAL_OPERATION;
            }
        }
        else
        {
            s->rx.pattern_repeats = 0;
        }
        break;

    case V22BIS_RX_TRAINING_PARKED:
    default:
        target = &z;
        break;
    }
    s->rx.last_raw_bits = raw_bits;
}

int v22bis_rx(v22bis_state_t *s, const int16_t amp[], int len)
{
    int i;
    int step;
    sm_complexf_t z;
    sm_complexf_t zz;
    sm_complexf_t sample;
    float ii;
    float qq;
    float root_power;
    double power;

    for (i = 0; i < len; i++)
    {
        /* Complex bandpass filter the signal with the RRC pair, using the
           carrier-specific coefficient set. */
        s->rx.rrc_filter[s->rx.rrc_filter_step] = (float) amp[i];
        if (++s->rx.rrc_filter_step >= V22BIS_RX_FILTER_STEPS)
            s->rx.rrc_filter_step = 0;

        /* Reference-phase I filter, to measure signal power at the carrier. */
        {
            int j;
            const float *cf;
            if (s->calling_party)
                cf = sm_rx_pulseshaper2400_re[6];
            else
                cf = sm_rx_pulseshaper1200_re[6];
            ii = 0.0f;
            {
                int k = s->rx.rrc_filter_step;
                for (j = 0; j < V22BIS_RX_FILTER_STEPS; j++)
                {
                    ii += s->rx.rrc_filter[k] * cf[j];
                    if (++k >= V22BIS_RX_FILTER_STEPS)
                        k = 0;
                }
            }
        }

        power = sm_power_meter_update(&s->rx.rx_power, (int16_t) ii);
        if (s->rx.signal_present)
        {
            if (power < s->rx.carrier_off_power)
            {
                /* Debounce carrier loss: adjacent-channel energy (e.g. a
                   V.32/V.34 calling modem's 1800 Hz signal or the tail of
                   the answer tone) can dip the meter below the threshold
                   for a few frames. Only a sustained loss (100 ms) means
                   the far end is gone. Importantly, do NOT reset the TX
                   side: an answerer must keep sending its training signal
                   until the calling modem responds. */
                if (++s->rx.carrier_down_count >= 800)
                {
                    v22bis_rx_restart(s);
                    report_status(s, V22BIS_STATUS_CARRIER_DOWN);
                    continue;
                }
            }
            else
            {
                s->rx.carrier_down_count = 0;
            }
        }
        else
        {
            if (power < s->rx.carrier_on_power)
                continue;
            s->rx.signal_present = 1;
            s->rx.carrier_down_count = 0;
            sm_log_message(&s->log, SM_LOG_FLOW, "Carrier up (%.1f Hz)", v22bis_rx_carrier_frequency(s));
            report_status(s, V22BIS_STATUS_CARRIER_UP);
        }

        if (s->rx.training == V22BIS_RX_TRAINING_PARKED)
            continue;

        /* T/2-rate insertion into the equalizer buffer, with fractional stepping
           under Gardner control -- NOT a fixed samples/symbol count. */
        if ((s->rx.eq_put_step -= V22BIS_RX_COEFF_SETS) <= 0)
        {
            const float *cf_re;
            const float *cf_im;
            int j;

            if (s->rx.training == V22BIS_RX_TRAINING_SYMBOL_ACQUISITION)
            {
                root_power = sqrtf((float) power);
                if (root_power < 1.0f)
                    root_power = 1.0f;
                s->rx.agc_scaling = 0.18f * 3.60f / root_power;
            }

            /* Select the fractional-phase coefficient set, pulse shaping at the
               carrier frequency with the quadrature RRC pair. */
            step = -s->rx.eq_put_step;
            if (step > V22BIS_RX_COEFF_SETS - 1)
                step = V22BIS_RX_COEFF_SETS - 1;
            if (s->calling_party)
            {
                cf_re = sm_rx_pulseshaper2400_re[step];
                cf_im = sm_rx_pulseshaper2400_im[step];
            }
            else
            {
                cf_re = sm_rx_pulseshaper1200_re[step];
                cf_im = sm_rx_pulseshaper1200_im[step];
            }
            ii = 0.0f;
            qq = 0.0f;
            {
                int k = s->rx.rrc_filter_step;
                for (j = 0; j < V22BIS_RX_FILTER_STEPS; j++)
                {
                    ii += s->rx.rrc_filter[k] * cf_re[j];
                    qq += s->rx.rrc_filter[k] * cf_im[j];
                    if (++k >= V22BIS_RX_FILTER_STEPS)
                        k = 0;
                }
            }
            /* AGC scaling, then direct complex mixing to baseband. */
            sample.re = ii * s->rx.agc_scaling;
            sample.im = qq * s->rx.agc_scaling;
            sm_dds_t dds;

            dds.phase = s->rx.carrier_phase;
            dds.rate = s->rx.carrier_phase_rate;
            z = sm_dds_complex(&dds);
            zz.re = sample.re * z.re - sample.im * z.im;
            zz.im = -sample.re * z.im - sample.im * z.re;

            s->rx.eq_put_step += V22BIS_RX_COEFF_SETS * 40 / (3 * 2);
            process_half_baud(s, &zz);
        }
        /* Advance the carrier DDS. */
        s->rx.carrier_phase += (uint32_t) s->rx.carrier_phase_rate;
    }
    return 0;
}

float v22bis_rx_carrier_frequency(const v22bis_state_t *s)
{
    return (float) s->rx.carrier_phase_rate / 4294967296.0f * (float) SM_SAMPLE_RATE;
}

/* Level in raw int16 power meter units for a dBm0 signal level. Mirrors the
   reference power_meter_level_dbm0() (DBM0_MAX_POWER = 6.16). */
static double power_meter_level_dbm0(float level)
{
    level -= 6.16f;
    if (level > 0.0f)
        level = 0.0f;
    return pow(10.0, level / 10.0) * (32767.0 * 32767.0);
}

void v22bis_rx_set_signal_cutoff(v22bis_state_t *s, float cutoff)
{
    s->rx.carrier_on_power = power_meter_level_dbm0(cutoff + 2.5f) * 0.232f;
    s->rx.carrier_off_power = power_meter_level_dbm0(cutoff - 2.5f) * 0.232f;
}

void v22bis_rx_restart(v22bis_state_t *s)
{
    int i;

    for (i = 0; i < V22BIS_RX_FILTER_STEPS; i++)
        s->rx.rrc_filter[i] = 0.0f;
    s->rx.rrc_filter_step = 0;
    s->rx.scramble_reg = 0;
    s->rx.scrambler_pattern_count = 0;
    s->rx.training_error = 0.0f;
    s->rx.training = V22BIS_RX_TRAINING_SYMBOL_ACQUISITION;
    s->rx.training_count = 0;
    s->rx.signal_present = false;

    /* RX carrier: caller listens on 2400 Hz, answerer on 1200 Hz. */
    s->rx.carrier_phase_rate = (int32_t) ((s->calling_party
                                               ? V22BIS_CARRIER_2400
                                               : V22BIS_CARRIER_1200)
                                          / (float) SM_SAMPLE_RATE * 4294967296.0f);
    s->rx.carrier_phase = 0;
    /* Reference uses an IIR power meter with shift 5 (per-sample dispersal
       1/32, ~4 ms time constant). */
    sm_power_meter_init(&s->rx.rx_power, 0.004);
    v22bis_rx_set_signal_cutoff(s, -45.5f);
    s->rx.agc_scaling = 0.0005f * 0.025f;

    s->rx.constellation_state = 0;
    s->rx.sixteen_way_decisions = false;
    s->rx.carrier_down_count = 0;

    equalizer_reset(s);

    s->rx.pattern_repeats = 0;
    s->rx.last_raw_bits = 0;
    s->rx.raw_history = 0;
    s->rx.raw_hist_count = 0;
    s->rx.gardner_integrate = 0;
    s->rx.gardner_step = 256;
    s->rx.baud_phase = 0;
    s->rx.total_baud_timing_correction = 0;
    /* The answerer pulls the carrier in faster, as it has very little time
       to adapt before unscrambled ones arrive. */
    s->rx.carrier_track_i = s->calling_party ? 8000.0f : 40000.0f;
    s->rx.carrier_track_p = 8000000.0f;

    s->negotiated_bit_rate = 1200;
}

void v22bis_rx_fillin(v22bis_state_t *s, int len)
{
    int i;

    if (!s->rx.signal_present)
        return;
    for (i = 0; i < len; i++)
        s->rx.carrier_phase += (uint32_t) s->rx.carrier_phase_rate;
}

int v22bis_equalizer_state(v22bis_state_t *s, sm_complexf_t **coeffs)
{
    *coeffs = s->rx.eq_coeff;
    return V22BIS_EQUALIZER_LEN;
}