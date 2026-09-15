#ifndef V22BS_CHANNEL_H
#define V22BS_CHANNEL_H

#include <stdint.h>
#include <math.h>
#include <string.h>

#define CHAN_CHUNK 160

typedef struct {
    /* Biquad state: a1,a2,b0,b1,b2, x1,x2,y1,y2 */
    float b0,b1,b2,a1,a2;
    float x1,x2,y1,y2;
} biquad_t;

typedef struct {
    /* Hilbert FIR (Type III, length 65) for analytic signal construction.
       delay_line is a ring buffer of 65 int16 samples; write_idx is next write position. */
    float h[65];
    int16_t delay_line[65];
    int write_idx;
} hilbert_t;

typedef struct {
    double   t;              /* current sample number (double to avoid precision loss) */
    double   freq_off_hz;    /* carrier frequency offset in Hz */
    double   amp_variation;  /* sinusoidal amplitude ripple peak (0 = off) */
    double   amp_freq;       /* frequency of ripple, Hz */
    double   attenuation;    /* linear attenuation (<1.0 = attenuation) */
    double   sigma;          /* AWGN standard deviation (int16 units) */
    double   skew_ppm;       /* clock skew ppm (rx runs at 8000*(1+skew*1e-6) Hz) */
    uint64_t noise_state;    /* xorshift64 state for PRNG */
    float    noise_cache;    /* cached Box-Muller sample */
    int      noise_ready;    /* 1 if cache valid */

    biquad_t hp_bq;          /* high-pass 300 Hz */
    biquad_t lp_bq;          /* low-pass 3400 Hz */
    hilbert_t hilb;          /* for carrier freq offset */

    /* Clock skew: maintains fractional read pointer into a 4-sample resample buffer
       of the filtered input, so that we can produce at (1+skew) * 8000 samples/second. */
    float    rsamp_buf[2];   /* last two filtered input samples before resampling */
    float    rsamp_frac;     /* fractional read position in [0,1) between rsamp_buf[0] and [1] */

    float    hilb_dly[33];  /* 33-sample delay line for bp_out, to align with Hilbert group delay (32) */
    int      hilb_dly_idx;  /* write position in hilb_dly ring buffer */
} v22bs_channel_t;

static void v22bs_channel_init(v22bs_channel_t *ch, uint32_t seed)
{
    memset(ch, 0, sizeof(*ch));
    ch->noise_state = seed ? seed : 0x9e3779b97f4a7c15ULL;

    /* 2nd-order Butterworth biquads (bilinear transform, Fs=8000).
       Computed: HP 300 Hz -> b={0.71901,-1.43802,0.71901} a_std={-1.35744,0.51860}
                 LP 3400 Hz -> b={0.84447,1.68895,0.84447} a_std={1.66461,0.71328}
       Our biquad_step computes y = b0x+b1x1+b2x2 + a1*y1 + a2*y2, i.e. it ADDS the
       stored a-coefficients, so we store the negation of standard a1,a2. */
    ch->hp_bq.b0 =  0.71901f;
    ch->hp_bq.b1 = -1.43802f;
    ch->hp_bq.b2 =  0.71901f;
    ch->hp_bq.a1 =  1.35744f;
    ch->hp_bq.a2 = -0.51860f;

    ch->lp_bq.b0 =  0.84447f;
    ch->lp_bq.b1 =  1.68895f;
    ch->lp_bq.b2 =  0.84447f;
    ch->lp_bq.a1 = -1.66461f;
    ch->lp_bq.a2 = -0.71328f;

    /* Hilbert FIR Type III, length 65.
       h[k] = (2/(pi*n)) * Hamming, for odd n != 0; 0 for even n and center.
       Normalize so the passband magnitude is unity (not peak coefficient),
       which is what the analytic-signal rotation requires. */
    {
        int i;
        float gain = 0.0f;
        float w = 2.0f * 3.14159265f * 1000.0f / 8000.0f;
        for (i = 0; i < 65; i++)
        {
            int n = i - 32;
            if (n == 0 || (n & 1) == 0)
                ch->hilb.h[i] = 0.0f;
            else
            {
                float hamming = 0.54f - 0.46f * cosf(2.0f * 3.14159265f * (float)i / 64.0f);
                ch->hilb.h[i] = (2.0f / (3.14159265f * (float)n)) * hamming;
            }
        }
        /* Compute passband gain magnitude (the Hilbert's DTFT is purely imaginary,
       so a plain cos-sum would be near zero). */
    {
        int i;
        double re = 0.0, im = 0.0;
        float w = 2.0f * 3.14159265f * 1000.0f / 8000.0f;
        for (i = 0; i < 65; i++)
        {
            int m = i - 32;
            re += ch->hilb.h[i] * cosf(w * (float)m);
            im -= ch->hilb.h[i] * sinf(w * (float)m);
        }
        gain = (float)hypot(re, im);
        if (fabsf(gain) < 1e-9f)
            gain = 1.0f;
        for (i = 0; i < 65; i++)
            ch->hilb.h[i] /= gain;
    }
    }
    ch->rsamp_buf[0] = ch->rsamp_buf[1] = 0.0f;
    ch->rsamp_frac = 0.0f;
    memset(ch->hilb_dly, 0, sizeof(ch->hilb_dly));
    ch->hilb_dly_idx = 0;
}

static float biquad_step(biquad_t *b, float x)
{
    float y = b->b0*x + b->b1*b->x1 + b->b2*b->x2
            + b->a1*b->y1 + b->a2*b->y2;
    b->x2 = b->x1; b->x1 = x;
    b->y2 = b->y1; b->y1 = y;
    return y;
}

static float hilbert_step(hilbert_t *h, int16_t x)
{
    float acc = 0.0f;
    int idx = h->write_idx;
    int i;

    h->delay_line[idx] = x;
    for (i = 0; i < 65; i++)
    {
        acc += h->h[i] * (float)h->delay_line[idx];
        if (--idx < 0)
            idx = 65 - 1;
    }
    if (++h->write_idx >= 65)
        h->write_idx = 0;
    return acc;
}

static float v22bs_randf(v22bs_channel_t *ch)
{
    uint64_t x = ch->noise_state;

    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    ch->noise_state = x;
    return (float)((int32_t)(x & 0xFFFFFFFF)) / (float)0x7FFFFFFF;
}

static float v22bs_gauss(v22bs_channel_t *ch)
{
    float u1, u2;

    if (ch->noise_ready)
    {
        ch->noise_ready = 0;
        return ch->noise_cache;
    }
    /* Box-Muller with two independent uniforms in (0,1): standard normal.
       (The previous polar form computed sqrt(-2 ln r / r) as the output
       magnitude without the sqrt(r)*direction factor, giving a divergent
       tail, e.g. sigma*30 -> RMS ~250 instead of 30.) */
    do {
        u1 = v22bs_randf(ch) * 0.5f + 0.5f;  /* (0,1) */
        u2 = v22bs_randf(ch) * 0.5f + 0.5f;
    } while (u1 <= 0.0f || u1 >= 1.0f || u2 <= 0.0f || u2 >= 1.0f);
    ch->noise_cache = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
    ch->noise_ready = 1;
    return sqrtf(-2.0f * logf(u1)) * sinf(2.0f * 3.14159265f * u2);
}

/* Process one sample through the channel. Returns the filtered/processed sample. */
static inline int16_t v22bs_channel_step(v22bs_channel_t *ch, int16_t xin)
{
    float x;
    float bp_out;
    float hilb_val;
    float analy_re, analy_im;
    float phase, cos_ph, sin_ph;
    float rotated_re;
    float out;
    int16_t result;

    x = (float)xin;

    /* Bandpass: HP 300 Hz then LP 3400 Hz */
    bp_out = biquad_step(&ch->lp_bq, biquad_step(&ch->hp_bq, x));

    /* Hilbert transform for carrier frequency offset */
    hilb_val = hilbert_step(&ch->hilb, (int16_t)bp_out);
    /* Align the in-phase branch with the Hilbert branch: the 65-tap
       antisymmetric FIR has group delay (65-1)/2 = 32 samples. A 33-tap
       ring buffer exposes sample n-32 as the aligned in-phase value. */
    analy_re = ch->hilb_dly[(ch->hilb_dly_idx + 1) % 33];
    ch->hilb_dly[ch->hilb_dly_idx] = bp_out;
    ch->hilb_dly_idx = (ch->hilb_dly_idx + 1) % 33;
    analy_im = hilb_val;

    /* Rotate analytic signal by carrier frequency offset */
    phase = (float)(ch->t * ch->freq_off_hz * 2.0 * 3.14159265358979 / 8000.0);
    cos_ph = cosf(phase);
    sin_ph = sinf(phase);
    /* Re{analytic * exp(j*phase)} = analy_re*cos - analy_im*sin */
    rotated_re = analy_re * cos_ph - analy_im * sin_ph;

    /* Attenuation */
    out = rotated_re * (float)ch->attenuation;

    /* Level variation */
    if (ch->amp_variation != 0.0)
    {
        float amp_mod = 1.0f + (float)ch->amp_variation
                      * sinf((float)(ch->t * ch->amp_freq * 2.0 * 3.14159265 / 8000.0));
        out *= amp_mod;
    }

    /* AWGN */
    if (ch->sigma > 0.0)
        out += (float)ch->sigma * v22bs_gauss(ch);

    ch->t += 1.0;

    /* Quantize to int16 with saturation */
    if (out > 32767.0f)  out = 32767.0f;
    else if (out < -32768.0f) out = -32768.0f;
    result = (int16_t)out;
    return result;
}

/* Apply channel to an entire buffer (160 samples). If skew_ppm != 0, produces a
   slightly different number of output samples via simple linear interpolation.
   Writes output to `out` (max `max_out` samples). Returns actual output length. */
static inline int v22bs_channel_apply(v22bs_channel_t *ch,
                                      const int16_t *in, int in_len,
                                      int16_t *out, int max_out)
{
    int i;
    int out_n = 0;

    if (ch->skew_ppm == 0.0)
    {
        for (i = 0; i < in_len && out_n < max_out; i++)
            out[out_n++] = v22bs_channel_step(ch, in[i]);
    }
    else
    {
        double advance = 1.0 / (1.0 + ch->skew_ppm * 1e-6);
        float frac = ch->rsamp_frac;
        float prev = ch->rsamp_buf[0];
        float curr = ch->rsamp_buf[1];

        for (i = 0; i < in_len; i++)
        {
            curr = (float)in[i];
            while (frac < 1.0f && out_n < max_out)
            {
                float interp = prev + frac * (curr - prev);
                int16_t proc = v22bs_channel_step(ch, (int16_t)interp);
                out[out_n++] = proc;
                frac += (float)advance;
            }
            frac -= 1.0f;
            prev = curr;
        }
        ch->rsamp_buf[0] = prev;
        ch->rsamp_buf[1] = curr;
        ch->rsamp_frac = frac;
    }
    return out_n;
}

#endif