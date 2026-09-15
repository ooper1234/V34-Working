#include "v22bis.h"
#include "../../dsp/carrier/sm_dds.h"
#include "../../dsp/detectors/sm_power_meter.h"
#include "../../dsp/filters/v22bis_filters.h"
#include "../../common/sm_common.h"

#include <string.h>

/* V.22bis - quadrant code to phase-advance mapping.
   Reference: SpanDSP v22bis_tx.c phase_steps[] = {1, 0, 2, 3}. */
static const int phase_steps[4] = {1, 0, 2, 3};

/* 16-QAM constellation, differential quadrant coding.
   Reference: SpanDSP v22bis_constellation[]. */
static const sm_complexf_t v22bis_constellation[16] = {
    {1.0f,  1.0f},   {3.0f,  1.0f},   {1.0f,  3.0f},   {3.0f,  3.0f},
    {-1.0f, 1.0f},   {-1.0f, 3.0f},   {-3.0f, 1.0f},   {-3.0f, 3.0f},
    {-1.0f, -1.0f},  {-3.0f, -1.0f},  {-1.0f, -3.0f},  {-3.0f, -3.0f},
    {1.0f,  -1.0f},  {1.0f,  -3.0f},  {3.0f,  -1.0f},  {3.0f,  -3.0f}
};

static const sm_complexf_t zero = {0.0f, 0.0f};

static void report_status(v22bis_state_t *s, v22bis_status_t status)
{
    if (s->status_handler)
        s->status_handler(s->status_user_data, status);
}

static int fake_get_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

/* Self-synchronizing scrambler: 1 + x^-14 + x^-17.
   Reference: SpanDSP v22bis_tx.c scramble(). */
static int scramble(v22bis_state_t *s, int bit)
{
    int out_bit;

    if (s->tx.scrambler_pattern_count >= 64)
    {
        bit ^= 1;
        s->tx.scrambler_pattern_count = 0;
    }
    out_bit = (bit ^ (s->tx.scramble_reg >> 13) ^ (s->tx.scramble_reg >> 16)) & 1;
    s->tx.scramble_reg = (s->tx.scramble_reg << 1) | out_bit;

    if (out_bit == 1)
        s->tx.scrambler_pattern_count++;
    else
        s->tx.scrambler_pattern_count = 0;
    return out_bit;
}

static int get_scrambled_bit(v22bis_state_t *s)
{
    int bit;

    if ((bit = s->tx.current_get_bit(s->get_bit_user_data)) == -1)
    {
        /* End of data: pad with ones and then begin shutdown. */
        s->tx.current_get_bit = fake_get_bit;
        s->tx.shutdown = 1;
        bit = 1;
    }
    return scramble(s, bit);
}

static sm_complexf_t training_get(v22bis_state_t *s)
{
    int bits;

    switch (s->tx.training)
    {
    case V22BIS_TX_TRAINING_INITIAL_TIMED_SILENCE:
        /* Answerer waits 75 ms before sending unscrambled 1s. */
        if (++s->tx.training_count >= ms_to_symbols(75))
        {
            s->tx.training_count = 0;
            sm_log_message(&s->log, SM_LOG_FLOW, "+++ starting U11 1200");
            s->tx.training = V22BIS_TX_TRAINING_U11;
        }
        s->tx.constellation_state = 0;
        return zero;
    case V22BIS_TX_TRAINING_INITIAL_SILENCE:
        s->tx.constellation_state = 0;
        return zero;
    case V22BIS_TX_TRAINING_U11:
        /* Unscrambled ones at 1200 b/s -> +270 deg phase advance each symbol. */
        s->tx.constellation_state = (s->tx.constellation_state + phase_steps[3]) & 3;
        return v22bis_constellation[(s->tx.constellation_state << 2) | 0x01];
    case V22BIS_TX_TRAINING_U0011:
        /* S1 segment: unscrambled double-dibit 00 11 at 1200 b/s, 100 ms,
           requesting or accepting 2400 b/s. */
        s->tx.constellation_state =
            (s->tx.constellation_state + phase_steps[3 * (s->tx.training_count & 1)]) & 3;
        if (++s->tx.training_count >= ms_to_symbols(100))
        {
            if (s->calling_party)
            {
                s->tx.training_count = 0;
                s->tx.training = V22BIS_TX_TRAINING_S11;
            }
            else
            {
                /* Answering side: start the timed scrambled-1 run part way through. */
                s->tx.training_count = ms_to_symbols(756 - (600 - 100));
                s->tx.training = V22BIS_TX_TRAINING_TIMED_S11;
            }
        }
        return v22bis_constellation[(s->tx.constellation_state << 2) | 0x01];
    case V22BIS_TX_TRAINING_TIMED_S11:
        if (++s->tx.training_count >= ms_to_symbols(756))
        {
            if (s->negotiated_bit_rate == 2400)
            {
                s->tx.training_count = 0;
                s->tx.training = V22BIS_TX_TRAINING_S1111;
            }
            else
            {
                s->tx.training_count = 0;
                s->tx.training = V22BIS_TX_TRAINING_NORMAL_OPERATION;
                report_status(s, V22BIS_STATUS_TRAINING_SUCCEEDED);
                s->tx.current_get_bit = s->get_bit;
            }
        }
        /* fall through */
    case V22BIS_TX_TRAINING_S11:
        /* Scrambled ones at 1200 b/s, 4PSK. */
        bits = scramble(s, 1);
        bits = (bits << 1) | scramble(s, 1);
        s->tx.constellation_state = (s->tx.constellation_state + phase_steps[bits]) & 3;
        return v22bis_constellation[(s->tx.constellation_state << 2) | 0x01];
    case V22BIS_TX_TRAINING_S1111:
        /* Scrambled ones at 2400 b/s, 200 ms burst, then data mode 2400. */
        bits = scramble(s, 1);
        bits = (bits << 1) | scramble(s, 1);
        s->tx.constellation_state = (s->tx.constellation_state + phase_steps[bits]) & 3;
        bits = scramble(s, 1);
        bits = (bits << 1) | scramble(s, 1);
        if (++s->tx.training_count >= ms_to_symbols(200))
        {
            s->tx.training_count = 0;
            s->tx.training = V22BIS_TX_TRAINING_NORMAL_OPERATION;
            report_status(s, V22BIS_STATUS_TRAINING_SUCCEEDED);
            s->tx.current_get_bit = s->get_bit;
        }
        return v22bis_constellation[(s->tx.constellation_state << 2) | bits];
    default:
        break;
    }
    return zero;
}

static sm_complexf_t getbaud(v22bis_state_t *s)
{
    int bits;

    if (s->tx.training)
        return training_get(s);

    if (s->tx.shutdown)
    {
        if (++s->tx.shutdown > 10)
            return zero;
    }
    /* First two bits define the quadrant advance. */
    bits = get_scrambled_bit(s);
    bits = (bits << 1) | get_scrambled_bit(s);
    s->tx.constellation_state = (s->tx.constellation_state + phase_steps[bits]) & 3;
    if (s->negotiated_bit_rate == 1200)
    {
        bits = 0x01;
    }
    else
    {
        /* The following two bits define the position within the quadrant. */
        bits = get_scrambled_bit(s);
        bits = (bits << 1) | get_scrambled_bit(s);
    }
    return v22bis_constellation[(s->tx.constellation_state << 2) | bits];
}

int v22bis_tx(v22bis_state_t *s, int16_t amp[], int len)
{
    sm_complexf_t v;
    sm_complexf_t z;
    float famp;
    int sample;

    if (s->tx.shutdown > 10)
        return 0;

    for (sample = 0; sample < len; sample++)
    {
        /* 40 phase slots per symbol, advance 3 per sample -> 13.333 samples/symbol
           at 8 kHz = 600 symbols/s. */
        if ((s->tx.baud_phase += V22BIS_TX_PHASE_STEP) >= V22BIS_TX_PHASES)
        {
            s->tx.baud_phase -= V22BIS_TX_PHASES;
            v = getbaud(s);
            s->tx.rrc_filter_re[s->tx.rrc_filter_step] = v.re;
            s->tx.rrc_filter_im[s->tx.rrc_filter_step] = v.im;
            if (++s->tx.rrc_filter_step >= V22BIS_TX_FILTER_STEPS)
                s->tx.rrc_filter_step = 0;
        }
        /* Root raised cosine pulse shaping at baseband, then carrier modulation. */
        {
            int i;
            const float *coeffs =
                sm_tx_pulseshaperTX[SM_TX_PULSESHAPERTX_SETS - 1 - s->tx.baud_phase];
            float x_re = 0.0f, x_im = 0.0f;
            int step = s->tx.rrc_filter_step;
            for (i = 0; i < V22BIS_TX_FILTER_STEPS; i++)
            {
                x_re += s->tx.rrc_filter_re[step] * coeffs[i];
                x_im += s->tx.rrc_filter_im[step] * coeffs[i];
                if (++step >= V22BIS_TX_FILTER_STEPS)
                    step = 0;
            }
            sm_dds_t dds;

            dds.phase = s->tx.carrier_phase;
            dds.rate = s->tx.carrier_phase_rate;
            z = sm_dds_complex(&dds);
            famp = (x_re * z.re - x_im * z.im) * s->tx.gain;
            if (s->tx.guard_tone_phase_rate
                && (s->tx.rrc_filter_re[s->tx.rrc_filter_step] != 0.0f
                    || s->tx.rrc_filter_im[s->tx.rrc_filter_step] != 0.0f))
            {
                /* Add the guard tone while symbols are present. */
                float gph = (float) s->tx.guard_tone_phase * (SM_TWO_PI / 4294967296.0f);
                famp += s->tx.guard_tone_gain * cosf(gph);
                s->tx.guard_tone_phase += (uint32_t) s->tx.guard_tone_phase_rate;
            }
            amp[sample] = sm_sat16(famp);
        }
        s->tx.carrier_phase += (uint32_t) s->tx.carrier_phase_rate;
    }
    return sample;
}

void v22bis_tx_power(v22bis_state_t *s, float power)
{
    float sig_power;
    float sig_gain;

    /* If there is a guard tone we need to scale down the signal power a bit, so the
       aggregate of the signal and guard tone power is the specified power. */
    if (s->tx.guard_tone_phase_rate == (int32_t) (550.0f / (float) SM_SAMPLE_RATE * 4294967296.0f))
    {
        sig_power = power - 1.0f;
        s->tx.guard_tone_gain = powf(10.0f, (sig_power - 3.0f - 3.14f) / 20.0f) * 32768.0f;
    }
    else if (s->tx.guard_tone_phase_rate == (int32_t) (1800.0f / (float) SM_SAMPLE_RATE * 4294967296.0f))
    {
        sig_power = power - 0.55f;
        s->tx.guard_tone_gain = powf(10.0f, (sig_power - 6.0f - 3.14f) / 20.0f) * 32768.0f;
    }
    else
    {
        sig_power = power;
        s->tx.guard_tone_gain = 0.0f;
    }
    /* TX_PULSESHAPER_GAIN is 1.0 in floating point builds (confirmed against the
       reference generator output). DBM0_MAX_SINE_POWER = 3.14. */
    sig_gain = 0.4490f * powf(10.0f, (sig_power - 3.14f) / 20.0f) * 32768.0f;
    s->tx.gain = sig_gain;
}

void v22bis_restart(v22bis_state_t *s)
{
    int i;

    for (i = 0; i < V22BIS_TX_FILTER_STEPS; i++)
    {
        s->tx.rrc_filter_re[i] = 0.0f;
        s->tx.rrc_filter_im[i] = 0.0f;
    }
    s->tx.rrc_filter_step = 0;
    s->tx.scramble_reg = 0;
    s->tx.scrambler_pattern_count = 0;
    s->tx.training = s->calling_party
                         ? V22BIS_TX_TRAINING_INITIAL_SILENCE
                         : V22BIS_TX_TRAINING_INITIAL_TIMED_SILENCE;
    s->tx.training_count = 0;
    s->tx.carrier_phase = 0;
    s->tx.guard_tone_phase = 0;
    s->tx.baud_phase = 0;
    s->tx.constellation_state = 0;
    s->tx.current_get_bit = fake_get_bit;
    s->tx.shutdown = 0;

    /* TX carrier: caller 1200 Hz, answerer 2400 Hz. */
    s->tx.carrier_phase_rate = (int32_t) ((s->calling_party
                                               ? V22BIS_CARRIER_1200
                                               : V22BIS_CARRIER_2400)
                                          / (float) SM_SAMPLE_RATE * 4294967296.0f);
    s->tx.carrier_phase = 0;
    s->tx.guard_tone_gain = 0.0f;
    v22bis_tx_power(s, -14.0f);

    s->negotiated_bit_rate = 1200;
    v22bis_rx_restart(s);
}

void v22bis_init(v22bis_state_t *s, bool calling_party, int bit_rate,
                 v22bis_get_bit_fn get_bit, void *get_bit_user_data,
                 v22bis_put_bit_fn put_bit, void *put_bit_user_data,
                 v22bis_status_fn status_handler, void *status_user_data)
{
    memset(s, 0, sizeof(*s));
    sm_log_init(&s->log, SM_LOG_DEBUG, "V22BIS", -1);
    s->calling_party = calling_party;
    s->bit_rate = bit_rate;
    s->negotiated_bit_rate = 1200;
    s->get_bit = get_bit;
    s->get_bit_user_data = get_bit_user_data;
    s->put_bit = put_bit;
    s->put_bit_user_data = put_bit_user_data;
    s->status_handler = status_handler;
    s->status_user_data = status_user_data;

    /* The answerer may add a guard tone (550 Hz or 1800 Hz). */
    if (!calling_party)
    {
        switch (s->options & 0xFF)
        {
        case 1:  /* V22BIS_GUARD_TONE_550HZ */
            s->tx.guard_tone_phase_rate =
                (int32_t) (550.0f / (float) SM_SAMPLE_RATE * 4294967296.0f);
            break;
        case 2:  /* V22BIS_GUARD_TONE_1800HZ */
            s->tx.guard_tone_phase_rate =
                (int32_t) (1800.0f / (float) SM_SAMPLE_RATE * 4294967296.0f);
            break;
        default:
            s->tx.guard_tone_phase_rate = 0;
            break;
        }
    }
    v22bis_restart(s);
}

void v22bis_set_bit_rate(v22bis_state_t *s, int bit_rate)
{
    s->bit_rate = bit_rate;
    v22bis_restart(s);
}