/* v34_cc_passband_test.c - Layer 2: passband TX→RX with RRC filters.
 *
 * KEY FINDING from previous run:
 *   Test A (no carrier DDS): first 4 bauds correct, then errors
 *   Test B (with carrier DDS): worse
 *   Filter output magnitudes are ~0.3 (expected ~0.7)
 *
 * HYPOTHESIS: The TX uses rectangular pulses (constant per baud) but the
 * RX filter is a matched RRC filter. The TX should also use RRC pulse shaping
 * to properly match the RX filter.
 *
 * Also testing: with vs without carrier DDS on the RX side.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ===== spandsp arctan2 — exact copy ===== */
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

/* ===== TX constellation ===== */
typedef struct { float re; float im; } complex_sig_t;
static const complex_sig_t training_constellation_4[4] =
{
    {-0.7071068f, -0.7071068f},   /* 225 degrees */
    {-0.7071068f,  0.7071068f},   /* 135 degrees */
    { 0.7071068f,  0.7071068f},   /*  45 degrees */
    { 0.7071068f, -0.7071068f}    /* 315 degrees */
};

/* ===== RX RRC filter ===== */
#include "v22bis_rx_2400_rrc.h"

#define RX_FILTER_COEFF_SETS  RX_PULSESHAPER_2400_COEFF_SETS  /* 12 */
#define RX_FILTER_TAPS        27
#define RX_FILTER_STEPS       27

/* ===== TX RRC filter (same structure as RX) ===== */
/* The TX uses v22bis_tx_rrc.h. Let me check if it exists. */
/* For now, use a simple raised-cosine pulse at baseband. */
#define TX_FILTER_LEN  27

static float carrier_cos(uint32_t phase) { return cosf((float)phase * 6.2831853f / 4294967296.0f); }
static float carrier_sin(uint32_t phase) { return sinf((float)phase * 6.2831853f / 4294967296.0f); }
static inline void dds_advancef(uint32_t *phase, uint32_t rate) { *phase += rate; }

/* Simple raised cosine pulse at baseband for TX.
 * This is a simplified version — the real TX uses tx_pulseshaper. */
static float tx_rrc_pulse[TX_FILTER_LEN];

static void tx_rrc_init(void)
{
    /* Generate a simple raised cosine pulse */
    float sum = 0;
    for (int i = 0; i < TX_FILTER_LEN; i++)
    {
        float t = (float)(i - TX_FILTER_LEN/2) / 600.0f;  /* normalized to baud rate */
        /* Raised cosine with rolloff 0.75 */
        float alpha = 0.75f;
        float val;
        if (fabsf(t) < 1e-6f)
            val = 1.0f;
        else if (fabsf(fabsf(t) - 1.0f/(2.0f*alpha)) < 1e-6f)
            val = alpha/(2.0f) * 3.14159265f/4.0f; /* sin(pi/(2*alpha)) / (pi*t) simplified */
        else
            val = sinf(3.14159265f * t * (1.0f - alpha)) / (3.14159265f * t) *
                  cosf(3.14159265f * alpha * t) / (1.0f - 4.0f*alpha*alpha*t*t);
        tx_rrc_pulse[i] = val;
        sum += val;
    }
    /* Normalize */
    for (int i = 0; i < TX_FILTER_LEN; i++)
        tx_rrc_pulse[i] /= sum;
}

/* Run one test. Returns number of mismatches. */
static int run_test(const char *label, int use_carrier_dds, int use_tx_rrc)
{
    int errors = 0;
    int verbose = (getenv("V34Verbose") != NULL);

    printf("=== %s ===\n", label);

    uint32_t carrier_rate = (uint32_t)(2400.0f * 4294967296.0f / 8000.0f);
    int total_bauds = 40;
    int samples_per_baud = 13;  /* floor(8000/600) */
    int total_samples = total_bauds * samples_per_baud + 60;

    int *dibits = malloc(total_bauds * sizeof(int));
    int *recovered = malloc(total_bauds * sizeof(int));
    for (int i = 0; i < total_bauds; i++)
        dibits[i] = i % 4;

    /* ===== TX: baseband QPSK + optional RRC + carrier modulation ===== */
    float *passband = malloc(total_samples * sizeof(float));

    /* TX baseband state */
    float tx_baseband_re[TX_FILTER_LEN] = {0};
    float tx_baseband_im[TX_FILTER_LEN] = {0};
    int tx_bb_step = 0;
    int tx_baud_phase = 0;
    int tx_prev_diff = 0;

    uint32_t tx_carrier = 0;

    for (int s = 0; s < total_samples; s++)
    {
        float sym_re = 0, sym_im = 0;

        /* Baud clock */
        tx_baud_phase += 3;
        if (tx_baud_phase >= 40)
        {
            tx_baud_phase -= 40;
            int dibit_idx = s / samples_per_baud;
            if (dibit_idx < total_bauds)
            {
                int diff = (tx_prev_diff + dibits[dibit_idx]) & 3;
                sym_re = training_constellation_4[diff].re;
                sym_im = training_constellation_4[diff].im;
                tx_prev_diff = diff;

                /* Feed into TX RRC filter */
                tx_baseband_re[tx_bb_step] = sym_re;
                tx_baseband_im[tx_bb_step] = sym_im;
                tx_bb_step = (tx_bb_step + 1) % TX_FILTER_LEN;
            }
        }

        float bb_re, bb_im;
        if (use_tx_rrc)
        {
            /* Apply TX RRC pulse shaping */
            bb_re = 0; bb_im = 0;
            for (int k = 0; k < TX_FILTER_LEN; k++)
            {
                int idx = (tx_bb_step + k) % TX_FILTER_LEN;
                bb_re += tx_baseband_re[idx] * tx_rrc_pulse[k];
                bb_im += tx_baseband_im[idx] * tx_rrc_pulse[k];
            }
        }
        else
        {
            /* Rectangular pulse: use most recent symbol */
            int prev = (tx_bb_step - 1 + TX_FILTER_LEN) % TX_FILTER_LEN;
            bb_re = tx_baseband_re[prev];
            bb_im = tx_baseband_im[prev];
        }

        /* Carrier modulation: s = Re(bb * carrier) = bb_re*cos - bb_im*sin */
        passband[s] = bb_re * carrier_cos(tx_carrier) - bb_im * carrier_sin(tx_carrier);
        tx_carrier += carrier_rate;
    }

    /* ===== RX: RRC filter + optional carrier DDS ===== */
    float rrc_buf[RX_FILTER_STEPS];
    memset(rrc_buf, 0, sizeof(rrc_buf));
    int rrc_step = 0;
    int eq_put_step = RX_FILTER_COEFF_SETS * 40 / 3 - 1;
    uint32_t rx_carrier = 0;
    float last_re = 0, last_im = 0;
    int rx_baud_count = 0;
    int rx_baud_half = 0;

    for (int s = 0; s < total_samples && rx_baud_count < total_bauds; s++)
    {
        rrc_buf[rrc_step] = passband[s];
        rrc_step++;
        if (rrc_step >= RX_FILTER_STEPS) rrc_step = 0;

        eq_put_step -= RX_FILTER_COEFF_SETS;
        int step = -eq_put_step;
        if (step > RX_FILTER_COEFF_SETS - 1) step = RX_FILTER_COEFF_SETS - 1;
        while (step < 0) step += RX_FILTER_COEFF_SETS;

        /* _re filter at every sample */
        float ii = 0;
        for (int k = 0; k < RX_FILTER_TAPS; k++)
        {
            int idx = (rrc_step + k) % RX_FILTER_STEPS;
            ii += rrc_buf[idx] * rx_pulseshaper_2400_re[step][k];
        }

        if (eq_put_step <= 0)
        {
            eq_put_step += RX_FILTER_COEFF_SETS * 40 / 3;

            /* _im filter */
            float qq = 0;
            for (int k = 0; k < RX_FILTER_TAPS; k++)
            {
                int idx = (rrc_step + k) % RX_FILTER_STEPS;
                qq += rrc_buf[idx] * rx_pulseshaper_2400_im[step][k];
            }

            float out_re, out_im;
            if (use_carrier_dds)
            {
                float z_re = carrier_cos(rx_carrier);
                float z_im = carrier_sin(rx_carrier);
                out_re = ii * z_re - qq * z_im;
                out_im = -ii * z_im - qq * z_re;
            }
            else
            {
                out_re = ii;
                out_im = qq;
            }

            rx_baud_half ^= 1;
            if (rx_baud_half)
            {
                last_re = out_re;
                last_im = out_im;
                if (use_carrier_dds) dds_advancef(&rx_carrier, carrier_rate);
                continue;
            }

            int32_t ang1 = arctan2(out_re, out_im);
            int32_t ang2 = arctan2(last_re, last_im);
            uint32_t ang3 = (uint32_t)(ang1 - ang2 + (int32_t)DDS_PHASE(45.0f));
            int raw_dibit = (int)(ang3 >> 30);

            if (rx_baud_count < total_bauds - 1)
                recovered[rx_baud_count] = raw_dibit;

            if (verbose && rx_baud_count < 15)
            {
                float mag = sqrtf(out_re*out_re + out_im*out_im);
                printf("  RX %3d: step=%2d |ii|=%.3f |qq|=%.3f mag=%.3f "
                       "raw=%d exp=%d%s\n",
                       rx_baud_count, step, fabsf(ii), fabsf(qq), mag,
                       raw_dibit, dibits[rx_baud_count + 1],
                       raw_dibit == dibits[rx_baud_count + 1] ? "" : " ***");
            }

            last_re = out_re;
            last_im = out_im;
            rx_baud_count++;
        }

        if (use_carrier_dds) dds_advancef(&rx_carrier, carrier_rate);
    }

    for (int i = 0; i < rx_baud_count && i + 1 < total_bauds; i++)
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
           rx_baud_count - errors, rx_baud_count, errors ? "FAIL" : "PASS");

    free(dibits);
    free(recovered);
    free(passband);
    return errors;
}

int main(void)
{
    int total = 0;

    printf("V.34 CC Passband Layer Tests\n");
    printf("============================\n\n");

    tx_rrc_init();

    total += run_test("A: Rect TX, NO carrier DDS", 0, 0);
    total += run_test("B: Rect TX, WITH carrier DDS", 1, 0);
    total += run_test("C: RRC TX, NO carrier DDS", 0, 1);
    total += run_test("D: RRC TX, WITH carrier DDS", 1, 1);

    printf("=== SUMMARY ===\n");
    printf("Total errors: %d\n", total);
    return total ? 1 : 0;
}
