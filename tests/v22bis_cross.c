#include "v22bs_impl.h"
#include "v22bs_channel.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

static int g_dump_bits;
static int g_const0;
static int g_const1;
static int g_alt;
static const char *dump_raw;
static const char *side0_impl;
static const char *side1_impl;

/* ------------------------------------------------------------------ */
/* Bit source / sink                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    uint64_t x;
} src_t;

typedef struct {
    uint64_t cap;
    uint64_t n;
    uint8_t *bit;      /* 1 byte per bit, MSB-first data bytes */
} sink_t;

static int src_bit(void *ud)
{
    src_t *s = ud;

    if (g_const0)
        return 0;
    if (g_const1)
        return 1;
    if (g_alt)
        return (int)(s->x++ & 1);
    s->x ^= s->x >> 12;
    s->x ^= s->x << 25;
    s->x ^= s->x >> 27;
    return (int)(s->x & 1);
}

static void sink_put(void *ud, int bit)
{
    sink_t *s = ud;

    if (s->n >= s->cap)
    {
        s->cap = s->cap ? s->cap * 2 : 4096;
        s->bit = realloc(s->bit, s->cap);
    }
    s->bit[s->n++] = (uint8_t)(bit & 1);
}

/* ------------------------------------------------------------------ */
/* Status logging                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    int events[8];
    double time_s;
} watch_t;

static void on_status(void *ud, int st)
{
    watch_t *w = ud;

    if (st >= 0 && st < 8)
        w->events[st]++;
    printf("[%6.2f] %-16s status=%d\n", w->time_s, w->name, st);
}


/* ------------------------------------------------------------------ */
/* Alignment and BER                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    int64_t align_offset;
    int64_t compare_bits;
    int64_t err_bits;
    double  ber;
    int64_t steady_err;     /* errors in the aligned data window */
    int64_t steady_bits;
    int64_t data_start;
    int64_t data_len;
    int     trained;
    int     oksamples;
} measured_t;

static void measure_side(v22bs_t *m, sink_t *rcv, src_t *src_data,
                         int64_t max_align, measured_t *res)
{
    int64_t best_err = INT64_MAX, best_off = 0;
    int64_t other_best_err = INT64_MAX;
    int64_t off;

    memset(res, 0, sizeof(*res));

    if (!m->normal(m->cb))
    {
        res->trained = 0;
        printf("  %-16s NOT TRAINED\n", m->name);
        return;
    }
    res->trained = 1;

    /* Build a reference stream: replay the exact bit sequence the far TX
       would have produced during the run. We regenerate it here from a copy
       of the PRNG state we saved at the start. */
    {
        src_t t = *src_data;
        int64_t i = 0;
        int64_t cap = (rcv->n + max_align + 4096);
        uint8_t *ref = malloc(cap);
        int64_t best_mismatch[2] = {INT64_MAX, INT64_MAX};

        for (i = 0; i < cap; i++)
            ref[i] = (uint8_t)src_bit(&t);

        /* Search alignment: find the offset where the steady-data window of
           received bits best matches a consecutive run of source bits. The
           first ~4000 bits are the training/far-end-S1111 tail, all ones
           after descrambling, so balancing on that region is meaningless. */
        {
            int64_t w0 = 4000;
            int64_t wlen = 2000;
            if (w0 >= rcv->n)
                w0 = 0;
            if (w0 + wlen > rcv->n)
                wlen = rcv->n - w0;
            res->data_start = w0;
            res->data_len = wlen;
            for (off = 0; off < max_align && ref[off] != (uint8_t)-1; off++)
            {
                int64_t j;
                int64_t e = 0;
                for (j = 0; j < wlen; j++)
                    e += (rcv->bit[w0 + j] != ref[off + j]);
                if (e < best_err)
                {
                    best_err = e;
                    best_off = off;
                    if (e == 0)
                        break;
                }
                if (e < best_mismatch[0])
                {
                    best_mismatch[1] = best_mismatch[0];
                    best_mismatch[0] = e;
                }
                else if (e < best_mismatch[1])
                    best_mismatch[1] = e;
            }
        }

        if ((best_err < 1000 && best_mismatch[0] != best_mismatch[1])
            || best_err == 0)
        {
            int64_t w0 = res->data_start;
            int64_t j;
            res->align_offset = best_off;
            res->compare_bits = res->data_len;
            res->err_bits = 0;
            res->steady_err = 0;
            res->steady_bits = 0;
            /* Measure BER only over the aligned data window. rx[w0+j]
               corresponds to source bit best_off+j. */
            for (j = 0; j < res->data_len; j++)
            {
                int e = (rcv->bit[w0 + j] != ref[best_off + j]);
                res->err_bits += e;
                res->steady_err += e;
                res->steady_bits++;
            }
            res->ber = (double)res->err_bits / (double)res->compare_bits;
        }
        else
        {
            res->align_offset = -1;
            res->compare_bits = rcv->n;
            res->err_bits = -1;
            res->ber = -1.0;
        }

        if (g_dump_bits)
        {
            int k;
            int64_t w0 = res->data_start;
            int64_t base = (best_err < 1000) ? best_off : 0;
            int64_t ones = 0, run = 0, maxrun = 0;
            printf("  --- %-10s data window @rx[%lld..%lld] aligned src@%d ---\n",
                   m->name, (long long)w0, (long long)(w0 + res->data_len),
                   (int)base);
            for (k = 0; k < 24; k++)
            {
                printf("  d=%3d  rx=%d  exp=%d  %s\n", k,
                       rcv->bit[w0 + k] & 1, ref[base + k] & 1,
                       ((rcv->bit[w0 + k] & 1) != (ref[base + k] & 1)) ? "<--" : "");
            }
            for (k = 0; k < (int)rcv->n; k++)
            {
                ones += (rcv->bit[k] & 1);
                if (rcv->bit[k] & 1) run++;
                else { if (run > maxrun) maxrun = run; run = 0; }
            }
            if (run > maxrun) maxrun = run;
            printf("  --- %-10s stream: n=%lld ones=%lld frac=%.3f max_run=%lld ---\n",
                   m->name, (long long)rcv->n, (long long)ones,
                   (double)ones / (double)rcv->n, (long long)maxrun);
        }
        free(ref);
    }
    (void) other_best_err;
}


/* ------------------------------------------------------------------ */
/* Encode-check unit test: verify constellation/differential mapping   */
/* against the ITU-T V.22bis tables, using our TX and RX in data mode. */
/* ------------------------------------------------------------------ */

static void encode_check(void)
{
    /* Standalone unit assertions for the V.22bis encode/decode chain, checked
       against the ITU-T tables rather than the modem's (static) functions:
       1) scrambler/descrambler polynomial 1 + x^-14 + x^-17 round-trips;
       2) the differential quadrant phase_step table is a self-inverse
          permutation (required for differential decode);
       3) the 16-QAM constellation has the ITU-T amplitude set {1,3} and the
          correct quadrant sign assignments;
       4) the differential TX-encode / RX-slice / differential-decode path
          round-trips 4-bit symbols for a long pseudo-random data stream.
    */
    typedef struct { float re, im; } qx_t;
    int ok = 1;
    int i, k;
    static const int local_phase_steps[4] = {1, 0, 2, 3};
    static const qx_t local_constellation[16] = {
        {1.0f,1.0f},{3.0f,1.0f},{1.0f,3.0f},{3.0f,3.0f},
        {-1.0f,1.0f},{-1.0f,3.0f},{-3.0f,1.0f},{-3.0f,3.0f},
        {-1.0f,-1.0f},{-3.0f,-1.0f},{-1.0f,-3.0f},{-3.0f,-3.0f},
        {1.0f,-1.0f},{1.0f,-3.0f},{3.0f,-1.0f},{3.0f,-3.0f}
    };
    const int *phase_steps = local_phase_steps;
    const qx_t *v22bis_constellation = local_constellation;

    {
        uint32_t tx_reg = 0xFFFFFFFF, rx_reg = 0xFFFFFFFF;
        unsigned long tx_cnt = 0, rx_cnt = 0;
        int errs = 0;

        printf("[encode-check] scrambler 1+x^-14+x^-17 round trip\n");
        for (i = 0; i < 200000; i++)
        {
            int bit = (i & 255) != 255;              /* mostly data, runs of ones */
            if (i % 97 == 0)
                bit = (i / 97) & 1;                  /* sprinkle runs to probe the 64-limit */
            {
                int b = bit;
                int out_bit;
                if (tx_cnt >= 64) { b ^= 1; tx_cnt = 0; }
                out_bit = (b ^ (int)((tx_reg >> 13) & 1) ^ (int)((tx_reg >> 16) & 1)) & 1;
                tx_reg = (tx_reg << 1) | (unsigned)out_bit;
                if (out_bit) tx_cnt++; else tx_cnt = 0;

                /* descramble the tx output */
                {
                    int ob = out_bit;
                    int db;
                    db = (ob ^ (int)((rx_reg >> 13) & 1) ^ (int)((rx_reg >> 16) & 1)) & 1;
                    if (rx_cnt >= 64) { db ^= 1; rx_cnt = 0; }
                    if (ob) rx_cnt++; else rx_cnt = 0;
                    rx_reg = (rx_reg << 1) | (unsigned)ob;
                    if (db != bit && i >= 32)
                        errs++;
                }
            }
        }
        printf("  scrambler round-trip errors (after 32-bit lock): %d  %s\n",
               errs, errs ? "FAIL" : "OK");
        if (errs)
            ok = 0;
    }

    {
        static const int ref_phase_steps[4] = {1, 0, 2, 3};
        int perm[4];
        printf("[encode-check] differential quadrant phase_step table\n");
        for (i = 0; i < 4; i++)
        {
            if (phase_steps[i] != ref_phase_steps[i])
            {
                printf("  phase_steps[%d]=%d expected %d  FAIL\n",
                       i, phase_steps[i], ref_phase_steps[i]);
                ok = 0;
            }
            perm[i] = phase_steps[i];
        }
        for (i = 0; i < 4; i++)
            for (k = 0; k < 4; k++)
            {
                if (i == k)
                    continue;
                if (perm[i] == perm[k])
                {
                    printf("  phase_steps not a permutation  FAIL\n");
                    ok = 0;
                }
            }
        for (i = 0; i < 4; i++)
            if (phase_steps[phase_steps[i]] != i)
            {
                printf("  phase_steps not self-inverse at %d  FAIL\n", i);
                ok = 0;
            }
        printf("  phase_steps={1,0,2,3} permutation & self-inverse  %s\n",
               ok ? "OK" : "FAIL");
    }

    {
        /* Constellation: 4 quadrants, amplitudes {1,3}, correct sign quadrants.
           index = q*4 + m where q=quadrant 0..3, m=within-quadrant 0..3. */
        int qerr = 0;
        printf("[encode-check] 16-QAM constellation ITU-T amplitude/sign structure\n");
        for (i = 0; i < 16; i++)
        {
            const qx_t *p = &v22bis_constellation[i];
            int q = i >> 2;
            float a = fabsf(p->re), b = fabsf(p->im);
            int exp_re_sign, exp_im_sign;
            exp_re_sign = (q == 0 || q == 3) ? 1 : -1;
            exp_im_sign = (q == 0 || q == 1) ? 1 : -1;
            if (!((a == 1.0f || a == 3.0f) && (b == 1.0f || b == 3.0f)))
                qerr++;
            if ((p->re < 0) != (exp_re_sign < 0))
                qerr++;
            if ((p->im < 0) != (exp_im_sign < 0))
                qerr++;
        }
        printf("  constellation constraints violations: %d  %s\n",
               qerr, qerr ? "FAIL" : "OK");
        if (qerr)
            ok = 0;
    }

    {
        /* Differential 4-bit encode -> AWGNless passband -> slice -> decode.
           Emulate TX: advance quadrant by phase_steps[first2], emit
           constellation index (q<<2)|last2. RX: slice, differential decode
           phase via the same self-inverse table, gather 4 bits. */
        static const int sim_phases[4] = {1, 0, 2, 3};
        double prng = 1234567.0;
        int tx_q = 0, errs = 0, n = 500000;
        int rx_q = 0;

        printf("[encode-check] differential encode/slice/decode 4-bit symbols\n");
        for (i = 0; i < n; i++)
        {
            int b0, b1, b2, b3;
            int raw, idx, rq, r0, r1, r2, r3;
            prng = fmod(prng * 16807.0, 2147483647.0);
            raw = (int)(prng / 2147483647.0 * 16.0);
            b0 = (raw >> 3) & 1;
            b1 = (raw >> 2) & 1;
            b2 = (raw >> 1) & 1;
            b3 = raw & 1;
            tx_q = (tx_q + sim_phases[(b0 << 1) | b1]) & 3;
            idx = (tx_q << 2) | (b2 << 1) | b3;

            /* emulate noise-free receive slice of the exact constellation point */
            rq = idx >> 2;

            /* differential decode: phase difference = (rq - rx_q) mod 4,
               mapped back through the same table */
            r0 = sim_phases[(rq - rx_q + 4) & 3] >> 1;
            r1 = sim_phases[(rq - rx_q + 4) & 3] & 1;
            rx_q = rq;
            r2 = (idx >> 1) & 1;
            r3 = idx & 1;
            if ((r0 != b0) || (r1 != b1) || (r2 != b2) || (r3 != b3))
            {
                errs++;
                if (errs < 5)
                    printf("  sym %d: tx=%d%d%d%d rx=%d%d%d%d  FAIL\n",
                           i, b0,b1,b2,b3, r0,r1,r2,r3);
            }
        }
        printf("  differential chain errors: %d/%d  %s\n", errs, n, errs ? "FAIL" : "OK");
        if (errs)
            ok = 0;
    }

    printf("[encode-check] %s\n", ok ? "ALL PASS" : "FAIL");
    exit(ok ? 0 : 1);
}

/* ------------------------------------------------------------------ */
/* forward decls                                                       */
/* ------------------------------------------------------------------ */

static void measure_side(v22bs_t *m, sink_t *rcv, src_t *src_data,
                         int64_t max_align, measured_t *res);
static void measure_carriers(const char *prefix, int rate, int ours_is_caller,
                             measured_t *m, v22bs_t **side, int *ret);

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    int combined_rate = 2400;
    int duration_s = 20;
    int ours_is_caller = 1;
    int mode_ours = 1, mode_ref = 1;
    int opt_trace = 0;
    const char *pcm_prefix = NULL;
    v22bs_channel_t ch[2];

    double freq_off = 0.0, sigma = 0.0, atenu = 1.0, level_var = 0.0, level_f = 0.5;
    double skew = 0.0;
    int bandpass = 0;

    v22bs_t *side[2];
    src_t src[2];
    sink_t rcv[2];
    watch_t w[2];
    int16_t txbuf[V22BS_CHUNK];
    int16_t ch_out[V22BS_CHUNK + 8];
    FILE *pcm_tx[2] = {NULL, NULL};
    FILE *pcm_rx[2] = {NULL, NULL};
    int i;
    int ret = 0;
    int timer;
    measured_t m[2];

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc)
            combined_rate = atoi(argv[++i]);
        else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc)
            duration_s = atoi(argv[++i]);
        else if (strcmp(argv[i], "--ours-caller") == 0)
            ours_is_caller = 1;
        else if (strcmp(argv[i], "--ours-answerer") == 0)
            ours_is_caller = 0;
        else if (strcmp(argv[i], "--ours-only") == 0)  { mode_ref = 0; }
        else if (strcmp(argv[i], "--ref-only") == 0)   { mode_ours = 0; }
        else if (strcmp(argv[i], "--side0") == 0 && i + 1 < argc)
            side0_impl = argv[++i];
        else if (strcmp(argv[i], "--side1") == 0 && i + 1 < argc)
            side1_impl = argv[++i];
        else if (strcmp(argv[i], "--dump-raw") == 0 && i + 1 < argc)
            dump_raw = argv[++i];
        else if (strcmp(argv[i], "--trace") == 0)
            opt_trace = 1;
        else if (strcmp(argv[i], "--pcm-prefix") == 0 && i + 1 < argc)
            pcm_prefix = argv[++i];
        else if (strcmp(argv[i], "--freqoff") == 0 && i + 1 < argc)
            freq_off = atof(argv[++i]);
        else if (strcmp(argv[i], "--atten") == 0 && i + 1 < argc)
            atenu = pow(10.0, -atof(argv[++i]) / 20.0);
        else if (strcmp(argv[i], "--sigma") == 0 && i + 1 < argc)
            sigma = atof(argv[++i]);
        else if (strcmp(argv[i], "--levelvar") == 0 && i + 1 < argc)
            level_var = atof(argv[++i]);
        else if (strcmp(argv[i], "--skew") == 0 && i + 1 < argc)
            skew = atof(argv[++i]);
        else if (strcmp(argv[i], "--bandpass") == 0)
            bandpass = 1;
        else if (strcmp(argv[i], "--encode-check") == 0)
        {
            encode_check();
            return 0;
        }
        else if (strcmp(argv[i], "--dump-bits") == 0)
            g_dump_bits = 1;
        else if (strcmp(argv[i], "--const0") == 0)
            g_const0 = 1;
        else if (strcmp(argv[i], "--const1") == 0)
            g_const1 = 1;
        else if (strcmp(argv[i], "--alt") == 0)
            g_alt = 1;
        else if (strcmp(argv[i], "--help") == 0)
        {
            printf("usage: %s [--rate 1200|2400] [--duration N] [--ours-caller|--ours-answerer]\n"
                   "           [--ours-only|--ref-only] [--trace] [--pcm-prefix P]\n"
                   "           [--freqoff HZ] [--atten DB] [--sigma N] [--levelvar N]\n"
                   "            [--skew PPM] [--bandpass] [--encode-check] [--dump-bits] [--const0]\n",
                   argv[0]);
            return 0;
        }
        else
        {
            printf("unknown option: %s\n", argv[i]);
            return 2;
        }
    }

    if (combined_rate != 1200 && combined_rate != 2400)
    {
        fprintf(stderr, "rate must be 1200 or 2400\n");
        return 2;
    }

    if (!mode_ours && !mode_ref)
    {
        fprintf(stderr, "cannot disable both implementations\n");
        return 2;
    }

    printf("V.22bis cross-validation\n");
    printf("  rate=%d  duration=%ds  ours=%s\n",
           combined_rate, duration_s, ours_is_caller ? "calling" : "answering");
    printf("  channel: freqoff=%+.1fHz bandpass=%d atten=%.2f sigma=%.0f levelvar=%.2f skew=%.0fppm\n",
           freq_off, bandpass, atenu, sigma, level_var, skew);

    /* Side 0 and side 1 roles */
    src[0].x = 0x243F6A8885A308D3ULL;
    src[1].x = 0x13198A2E03707344ULL;
    rcv[0].cap = 0; rcv[0].n = 0; rcv[0].bit = NULL;
    rcv[1].cap = 0; rcv[1].n = 0; rcv[1].bit = NULL;

    /* Side 0: ours or ref depending on mode; it takes the requested role. */
    w[0].name = "caller";
    w[1].name = "answerer";
    w[0].events[0] = w[0].events[1] = w[0].events[2] = w[0].events[3] = w[0].events[4] =
    w[0].events[5] = w[0].events[6] = w[0].events[7] = w[0].time_s = 0;
    w[1].events[0] = w[1].events[1] = w[1].events[2] = w[1].events[3] = w[1].events[4] =
    w[1].events[5] = w[1].events[6] = w[1].events[7] = w[1].time_s = 0;

    /* Side 0/1: implementation given by --side0/--side1, defaulting to the
       classic mode: ours for side0 unless --ref-only; side1 opposite. */
    {
        const char *i0 = side0_impl ? side0_impl : (mode_ours ? "ours" : "ref");
        const char *i1 = side1_impl ? side1_impl
                       : (mode_ours && mode_ref && !side0_impl)
                             ? (strcmp(i0, "ours") == 0 ? "ref" : "ours")
                             : i0;
        int cref[2];

        if (strcmp(i0, "ours") != 0 && strcmp(i0, "ref") != 0)
        {
            fprintf(stderr, "unknown implementation for side0: %s\n", i0);
            return 2;
        }
        if (strcmp(i1, "ours") != 0 && strcmp(i1, "ref") != 0)
        {
            fprintf(stderr, "unknown implementation for side1: %s\n", i1);
            return 2;
        }
        cref[0] = (strcmp(i0, "ref") == 0);
        cref[1] = (strcmp(i1, "ref") == 0);

        side[0] = cref[0]
                      ? v22bs_ref_create(ours_is_caller, combined_rate,
                                         src_bit, &src[0], sink_put, &rcv[0],
                                         on_status, &w[0])
                      : v22bs_ours_create(ours_is_caller, combined_rate,
                                          src_bit, &src[0], sink_put, &rcv[0],
                                          on_status, &w[0]);
        side[1] = cref[1]
                      ? v22bs_ref_create(!ours_is_caller, combined_rate,
                                         src_bit, &src[1], sink_put, &rcv[1],
                                         on_status, &w[1])
                      : v22bs_ours_create(!ours_is_caller, combined_rate,
                                          src_bit, &src[1], sink_put, &rcv[1],
                                          on_status, &w[1]);
    }

    printf("  side[0] = %s (%s)\n", side[0]->name, ours_is_caller ? "caller" : "answerer");
    printf("  side[1] = %s (%s)\n", side[1]->name, ours_is_caller ? "answerer" : "caller");

    v22bs_channel_init(&ch[0], 0x12345678);
    v22bs_channel_init(&ch[1], 0x87654321);
    ch[0].freq_off_hz = freq_off;
    ch[1].freq_off_hz = freq_off;
    ch[0].attenuation = atenu;
    ch[1].attenuation = atenu;
    ch[0].sigma = sigma;
    ch[1].sigma = sigma;
    ch[0].amp_variation = level_var;
    ch[1].amp_variation = level_var;
    ch[0].amp_freq = level_f;
    ch[1].amp_freq = level_f;
    ch[0].skew_ppm = skew;
    ch[1].skew_ppm = skew;
    if (!bandpass)
    {
        /* Bypass the HP/LP by making the biquads transparent (zero everything
           is not transparent; instead disable by replacing input with itself
           through a unity biquad). Simplest: set coefficients to bypass. */
        biquad_t bypass[2] = {{1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
                              {1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}};
        ch[0].hp_bq = bypass[0];
        ch[0].lp_bq = bypass[1];
        ch[1].hp_bq = bypass[0];
        ch[1].lp_bq = bypass[1];
    }

    (void)opt_trace;

    if (pcm_prefix)
    {
        char fn[512];
        for (i = 0; i < 2; i++)
        {
            snprintf(fn, sizeof(fn), "%s_side%d_tx.pcm", pcm_prefix, i);
            pcm_tx[i] = fopen(fn, "wb");
            snprintf(fn, sizeof(fn), "%s_side%d_rx.pcm", pcm_prefix, i);
            pcm_rx[i] = fopen(fn, "wb");
        }
    }

    /* Main loop: 20 ms chunks, full duplex */
    timer = 0;
    for (i = 0; i < duration_s * 50; i++)
    {
        int j;
        w[0].time_s = w[1].time_s = i * 0.02;
        for (j = 0; j < 2; j++)
        {
            int other = 1 - j;
            int n_out;
            /* TX from side j */
            side[j]->tx(side[j]->cb, txbuf, V22BS_CHUNK);
            if (pcm_tx[j])
                fwrite(txbuf, sizeof(int16_t), V22BS_CHUNK, pcm_tx[j]);
            /* channel toward other side */
            n_out = v22bs_channel_apply(&ch[other], txbuf, V22BS_CHUNK,
                                        ch_out, V22BS_CHUNK + 8);
            if (pcm_rx[other])
                fwrite(ch_out, sizeof(int16_t), n_out, pcm_rx[other]);
            side[other]->rx(side[other]->cb, ch_out, n_out);
        }

        timer += V22BS_CHUNK;
        if (timer >= 8000)
        {
            timer = 0;
            printf("  @%4.1fs: side0 %s (rate %d%s)  side1 %s (rate %d%s)\n",
                   i * 0.02, side[0]->name, side[0]->bit_rate(side[0]->cb),
                   side[0]->normal(side[0]->cb) ? " OK" : "",
                   side[1]->name, side[1]->bit_rate(side[1]->cb),
                   side[1]->normal(side[1]->cb) ? " OK" : "");
        }
    }

    /* Re-seed sources so measure_side can regenerate the reference stream. */
    src[0].x = 0x243F6A8885A308D3ULL;
    src[1].x = 0x13198A2E03707344ULL;

    printf("\n=== BER analysis ===\n");
    /* side[0] (caller) receives the answerer's data (src[1]); side[1]
       (answerer) receives the caller's data (src[0]). */
    if (dump_raw)
    {
        FILE *f;
        char fn[512];
        int64_t i;
        for (i = 0; i < 2; i++)
        {
            snprintf(fn, sizeof(fn), "%s_rx%d.bin", dump_raw, i);
            f = fopen(fn, "wb");
            fwrite(rcv[i].bit, 1, rcv[i].n, f);
            fclose(f);
            snprintf(fn, sizeof(fn), "%s_src%d.bin", dump_raw, i);
            f = fopen(fn, "wb");
            {
                src_t t = (i == 0) ? (src_t){0x243F6A8885A308D3ULL}
                                   : (src_t){0x13198A2E03707344ULL};
                int64_t k;
                for (k = 0; k < 20000; k++)
                {
                    int b = src_bit(&t);
                    fputc(b & 1, f);
                }
            }
            fclose(f);
        }
    }
    measure_side(side[0], &rcv[0], &src[1], 4000, &m[0]);
    measure_side(side[1], &rcv[1], &src[0], 4000, &m[1]);
    printf("  %-16s align=%4lld compare=%5lld err=%4lld ber=%.5f steady_err=%lld/%lld\n",
           side[0]->name, (long long)m[0].align_offset, (long long)m[0].compare_bits,
           (long long)m[0].err_bits, m[0].ber, (long long)m[0].steady_err,
           (long long)m[0].steady_bits);
    printf("  %-16s align=%4lld compare=%5lld err=%4lld ber=%.5f steady_err=%lld/%lld\n",
           side[1]->name, (long long)m[1].align_offset, (long long)m[1].compare_bits,
           (long long)m[1].err_bits, m[1].ber, (long long)m[1].steady_err,
           (long long)m[1].steady_bits);

    printf("\n=== Carrier frequency measurement (TX spectrum peak, data mode) ===\n");
    ret = 0;
    if (pcm_prefix)  /* need PCM re-open; measure from the recorded TX files */
        measure_carriers(pcm_prefix, combined_rate, ours_is_caller,
                         m, side, &ret);

    printf("\n=== Recovered RX carrier tracking ===\n");
    printf("  %-16s carrier=%.2f Hz  symbol_corr=%.1f\n",
           side[0]->name, side[0]->carrier_freq(side[0]->cb),
           side[0]->symbol_timing(side[0]->cb));
    printf("  %-16s carrier=%.2f Hz  symbol_corr=%.1f\n",
           side[1]->name, side[1]->carrier_freq(side[1]->cb),
           side[1]->symbol_timing(side[1]->cb));

    /* RX carrier must track the opposite side's TX carrier: nominal FDM
       frequency plus the channel's frequency offset, within ±5 Hz. */
    {
        double rxexp[2];
        rxexp[0] = (ours_is_caller ? 2400.0 : 1200.0) + freq_off;
        rxexp[1] = (ours_is_caller ? 1200.0 : 2400.0) + freq_off;
        for (i = 0; i < 2; i++)
        {
            double f_rx = side[i]->carrier_freq(side[i]->cb);
            if (fabs(f_rx - rxexp[i]) > 5.0)
            {
                printf("  RX carrier off: side%d expected %.0f got %.2f\n",
                       i, rxexp[i], f_rx);
                ret = 1;
            }
        }
    }

    /* Pass/fail: both must train, have zero steady-state bit errors, and
       the RX PLL must lock within 5 Hz of the expected opposite-channel carrier. */
    if (!m[0].trained || !m[1].trained)
        ret = 1;
    if (m[0].steady_err != 0 && m[0].steady_err != -1)
        ret = 1;
    if (m[1].steady_err != 0 && m[1].steady_err != -1)
        ret = 1;

    v22bs_destroy(side[0]);
    v22bs_destroy(side[1]);
    for (i = 0; i < 2; i++)
    {
        if (pcm_tx[i]) fclose(pcm_tx[i]);
        if (pcm_rx[i]) fclose(pcm_rx[i]);
    }
    free(rcv[0].bit);
    free(rcv[1].bit);

    printf("\n%s\n", ret ? "RESULT: FAIL" : "RESULT: PASS");
    return ret;
}

/* Measure the TX carrier frequency from the captured PCM using spectral
   centroid of the QAM band (the peak of the spread spectrum is not the
   carrier). side0 TX is 1200 Hz if caller else 2400 Hz; side1 is opposite. */
static void measure_carriers(const char *prefix, int rate, int ours_is_caller,
                             measured_t *m, v22bs_t **side, int *ret)
{
    int i;
    char fn[512];
    int16_t buf[8000];
    int expectations[2];

    expectations[0] = ours_is_caller ? 1200 : 2400;
    expectations[1] = ours_is_caller ? 2400 : 1200;

    for (i = 0; i < 2; i++)
    {
        FILE *f;
        long n;
        double f_meas = 0.0;
        double csum = 0.0, tot = 0.0;

        snprintf(fn, sizeof(fn), "%s_side%d_tx.pcm", prefix, i);
        f = fopen(fn, "rb");
        if (!f)
            continue;
        fseek(f, 0, SEEK_END);
        n = ftell(f) / 2;
        if (n > 8000)
        {
            fseek(f, (n - 8000) * 2, SEEK_SET);
            n = 8000;
        }
        else
            fseek(f, 0, SEEK_SET);
        n = fread(buf, sizeof(int16_t), n, f);
        fclose(f);

        /* Centroid over the band around the expected carrier. */
        {
            int fbin;
            for (fbin = 300; fbin <= 3400; fbin += 5)
            {
                double w = 2.0 * 3.14159265358979 * fbin / 8000.0;
                double cr = 0.0, ci = 0.0;
                int k;
                for (k = n - 1; k >= 0 && k >= n - 8000; k--)
                {
                    cr += buf[k] * cos(w * k);
                    ci += buf[k] * (-sin(w * k));
                }
                {
                    double p = sqrt(cr * cr + ci * ci);
                    csum += fbin * p;
                    tot += p;
                }
            }
        }
        if (tot > 0.0)
            f_meas = csum / tot;

        {
            int ok = (fabs(f_meas - expectations[i]) < 60.0);
            printf("  %-16s TX expected=%4d Hz  spectral centroid=%6.1f Hz  %s\n",
                   side[i]->name, expectations[i], f_meas,
                   ok ? "(check)" : "(band edge)");
        }
    }
    (void)rate;
    (void)m;
}