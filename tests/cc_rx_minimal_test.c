/* cc_rx_minimal_test.c - Minimal test for CC RX chain with MATCHED filters.
 *
 * Uses the SAME tx_pulseshaper for TX and rx_pulseshaper for RX,
 * matching the real V.34 CC TX/RX code paths.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* spandsp arctan2 — exact copy */
static inline int32_t arctan2(float y, float x)
{
    float abs_y, angle;
    if (y == 0.0f) { if (x < 0.0f) return (int32_t)0x80000000; return 0; }
    if (x == 0.0f) { if (y < 0.0f) return (int32_t)0xc0000000; return (int32_t)0x40000000; }
    abs_y = fabsf(y);
    if (x < 0.0f) angle = 3.0f - (x + abs_y)/(abs_y - x);
    else           angle = 1.0f - (x - abs_y)/(abs_y + x);
    angle *= 536870912.0f;
    if (y < 0.0f) angle = -angle;
    return (int32_t)angle;
}

#define DDS_PHASE(x) ((uint32_t)((x) * (4294967296.0 / 360.0)))

/* TX RRC filter (same as V.34 CC TX uses) */
#include "v22bis_tx_rrc.h"

/* RX complex bandpass filters */
#include "v22bis_rx_2400_rrc.h"
#include "v22bis_rx_1200_rrc.h"

#define RX_FILTER_COEFF_SETS  RX_PULSESHAPER_2400_COEFF_SETS  /* 12 */
#define RX_FILTER_TAPS        27
#define RX_FILTER_STEPS       27

#define TX_FILTER_COEFF_SETS  TX_PULSESHAPER_COEFF_SETS  /* 40 */
#define TX_FILTER_TAPS        9
#define TX_FILTER_STEPS       9

static float carrier_cos(uint32_t phase) { return cosf((float)phase * 6.2831853f / 4294967296.0f); }
static float carrier_sin(uint32_t phase) { return sinf((float)phase * 6.2831853f / 4294967296.0f); }
static inline void dds_advancef(uint32_t *phase, uint32_t rate) { *phase += rate; }

/* spandsp vec_circular_dot_prodf */
static float vec_circular_dot_prodf(const float *coeff, const float *buf, int len, int pos)
{
    float sum = 0;
    for (int k = 0; k < len; k++)
        sum += coeff[k] * buf[(pos + k) % len];
    return sum;
}

/* QPSK constellation — same as spandsp training_constellation_4 */
typedef struct { float re; float im; } complex_sig_t;
static const complex_sig_t qpsk_const[4] =
{
    {-0.7071068f, -0.7071068f},
    {-0.7071068f,  0.7071068f},
    { 0.7071068f,  0.7071068f},
    { 0.7071068f, -0.7071068f}
};

/* TX: generate QPSK passband signal with RRC pulse shaping + carrier modulation
 * Matches the real V.34 CC TX path in v34tx.c tx_cc_modulation() */
static void generate_qpsk_passband(float *out, int n_samples,
    float carrier_freq, int total_bauds, int *dibits)
{
    uint32_t tx_carrier_rate = (uint32_t)(carrier_freq * 4294967296.0f / 8000.0f);
    uint32_t tx_carrier = 0;

    /* TX RRC filter buffers (complex baseband) */
    float tx_buf_re[TX_FILTER_STEPS];
    float tx_buf_im[TX_FILTER_STEPS];
    memset(tx_buf_re, 0, sizeof(tx_buf_re));
    memset(tx_buf_im, 0, sizeof(tx_buf_im));
    int tx_step = 0;

    int baud_phase = 0;
    int prev_diff = 0;
    int baud_idx = 0;

    for (int s = 0; s < n_samples; s++)
    {
        float sym_re = 0, sym_im = 0;

        /* Baud clock: 600 baud = 40/3 samples per baud */
        baud_phase += 3;
        if (baud_phase >= 40)
        {
            baud_phase -= 40;
            if (baud_idx < total_bauds)
            {
                int diff = (prev_diff + dibits[baud_idx]) & 3;
                sym_re = qpsk_const[diff].re;
                sym_im = qpsk_const[diff].im;
                prev_diff = diff;
                baud_idx++;

                /* Load into TX RRC filter buffer */
                tx_buf_re[tx_step] = sym_re;
                tx_buf_im[tx_step] = sym_im;
                tx_step = (tx_step + 1) % TX_FILTER_STEPS;
            }
        }

        /* TX RRC pulse shaping (same as spandsp tx_cc_modulation) */
        int filter_idx = TX_FILTER_COEFF_SETS - 1 - baud_phase;
        float x_re = vec_circular_dot_prodf(tx_buf_re, tx_pulseshaper[filter_idx], TX_FILTER_TAPS, tx_step);
        float x_im = vec_circular_dot_prodf(tx_buf_im, tx_pulseshaper[filter_idx], TX_FILTER_TAPS, tx_step);

        /* Carrier modulation: passband = Re((x_re + j*x_im) * e^(j*2pi*fc*t)) */
        out[s] = x_re * carrier_cos(tx_carrier) - x_im * carrier_sin(tx_carrier);
        tx_carrier += tx_carrier_rate;
    }
}

/* RX: complex bandpass filter + carrier DDS + differential decode
 * Matches the real V.34 CC RX path in v34rx.c cc_rx() + process_cc_half_baud() */
static int rx_decode(const float *passband, int n_samples,
    float rx_carrier_freq, int total_bauds, int *recovered)
{
    const float (*filter_re)[RX_FILTER_TAPS];
    const float (*filter_im)[RX_FILTER_TAPS];
    int coeff_sets;

    if (rx_carrier_freq > 1800.0f)
    {
        filter_re = rx_pulseshaper_2400_re;
        filter_im = rx_pulseshaper_2400_im;
        coeff_sets = RX_PULSESHAPER_2400_COEFF_SETS;
    }
    else
    {
        filter_re = rx_pulseshaper_1200_re;
        filter_im = rx_pulseshaper_1200_im;
        coeff_sets = RX_PULSESHAPER_1200_COEFF_SETS;
    }

    float rrc_buf[RX_FILTER_STEPS];
    memset(rrc_buf, 0, sizeof(rrc_buf));
    int rrc_step = 0;

    int eq_put_step = coeff_sets * 40 / 3 - 1;
    uint32_t rx_carrier = 0;
    uint32_t rx_carrier_rate = (uint32_t)(rx_carrier_freq * 4294967296.0f / 8000.0f);

    float last_re = 0, last_im = 0;
    int rx_baud_count = 0;
    int baud_half = 0;
    int verbose = (getenv("V34Verbose") != NULL);

    for (int s = 0; s < n_samples && rx_baud_count < total_bauds; s++)
    {
        rrc_buf[rrc_step] = passband[s];
        rrc_step++;
        if (rrc_step >= RX_FILTER_STEPS) rrc_step = 0;

        eq_put_step -= coeff_sets;
        int step = -eq_put_step;
        if (step > coeff_sets - 1) step = coeff_sets - 1;
        while (step < 0) step += coeff_sets;

        /* Compute ii at every sample (matches spandsp cc_rx) */
        float ii = vec_circular_dot_prodf(rrc_buf, filter_re[step], RX_FILTER_TAPS, rrc_step);

        if (eq_put_step <= 0)
        {
            eq_put_step += coeff_sets * 40 / 3;

            /* Compute qq at baud boundaries */
            float qq = vec_circular_dot_prodf(rrc_buf, filter_im[step], RX_FILTER_TAPS, rrc_step);

            /* Carrier DDS shift: conj(sample * z) */
            float z_re = carrier_cos(rx_carrier);
            float z_im = carrier_sin(rx_carrier);
            float out_re = ii * z_re - qq * z_im;
            float out_im = -(ii * z_im + qq * z_re);

            /* process_cc_half_baud: T/2 rate, decode on alternate halves */
            baud_half ^= 1;
            if (baud_half)
            {
                last_re = out_re;
                last_im = out_im;
                dds_advancef(&rx_carrier, rx_carrier_rate);
                continue;
            }

            /* Differential decode: phase difference + 45° offset */
            int32_t ang1 = arctan2(out_re, out_im);
            int32_t ang2 = arctan2(last_re, last_im);
            uint32_t ang3 = (uint32_t)(ang1 - ang2 + (int32_t)DDS_PHASE(45.0f));
            int raw_dibit = (int)(ang3 >> 30);

            if (rx_baud_count < total_bauds)
                recovered[rx_baud_count] = raw_dibit;

            if (verbose && rx_baud_count < 25)
            {
                float mag = sqrtf(out_re*out_re + out_im*out_im);
                printf("  RX %3d: re=%8.4f im=%8.4f mag=%.3f raw=%d\n",
                       rx_baud_count, out_re, out_im, mag, raw_dibit);
            }

            last_re = out_re;
            last_im = out_im;
            rx_baud_count++;
        }

        dds_advancef(&rx_carrier, rx_carrier_rate);
    }

    return rx_baud_count;
}

static int run_test(const char *label, float tx_carrier, float rx_carrier,
    int total_bauds, int *dibits)
{
    /* 8000 sps, 600 baud = 13.33 samples/baud */
    int total_samples = total_bauds * 14 + 100;
    float *passband = malloc(total_samples * sizeof(float));
    int *recovered = malloc(total_bauds * sizeof(int));

    printf("=== %s === (TX=%.0f Hz, RX=%.0f Hz, %d bauds)\n",
           label, tx_carrier, rx_carrier, total_bauds);

    generate_qpsk_passband(passband, total_samples, tx_carrier, total_bauds, dibits);

    int rx_count = rx_decode(passband, total_samples, rx_carrier, total_bauds, recovered);

    /* Differential decode output: recovered[i] should equal dibits[i+1]
     * because the decode computes the differential between symbol i and symbol i+1 */
    int errors = 0;
    for (int i = 0; i < rx_count && i + 1 < total_bauds; i++)
    {
        if (recovered[i] != dibits[i + 1])
        {
            if (errors < 5)
                printf("  MISMATCH at %d: expected %d got %d\n",
                       i, dibits[i + 1], recovered[i]);
            errors++;
        }
    }

    printf("  %d/%d correct → %s\n\n",
           rx_count - errors, rx_count, errors ? "FAIL" : "PASS");

    free(passband);
    free(recovered);
    return errors;
}

int main(void)
{
    int total_errors = 0;
    int total_bauds = 50;
    int *dibits = malloc(total_bauds * sizeof(int));

    printf("CC RX Minimal Test (Matched TX/RX)\n");
    printf("===================================\n\n");

    /* Test pattern: repeating 0,1,2,3 */
    for (int i = 0; i < total_bauds; i++)
        dibits[i] = i % 4;

    /* Test A: TX 2400 Hz → RX 2400 Hz (caller receiving answerer CC) */
    total_errors += run_test("A: 2400→2400 (caller RX)", 2400.0f, 2400.0f, total_bauds, dibits);

    /* Test B: TX 1200 Hz → RX 1200 Hz (answerer receiving caller CC) */
    total_errors += run_test("B: 1200→1200 (answerer RX)", 1200.0f, 1200.0f, total_bauds, dibits);

    /* Test C: TX 2400 Hz → RX 1200 Hz (mismatch — should fail) */
    total_errors += run_test("C: 2400→1200 (mismatch)", 2400.0f, 1200.0f, total_bauds, dibits);

    free(dibits);

    printf("=== SUMMARY ===\n");
    printf("Total errors: %d\n", total_errors);
    return total_errors ? 1 : 0;
}
