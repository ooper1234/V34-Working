/*
 * cc_rx_diag_test.c — Diagnostic test for CC RX chain.
 *
 * Generates a clean QPSK signal at 1200 Hz (answerer's carrier for
 * receiving caller's CC), passes it through cc_rx(), and checks whether
 * the descrambled bits form the expected MP sync pattern.
 *
 * Build:  gcc -I../src/modem/v34/v34build/include -I../third_party/spandsp/src
 *           -I../src/modem/v34/v34build -o cc_rx_diag_test cc_rx_diag_test.c
 *           -L../build -lsmv34 -lspan -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "v34.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Generate a QPSK symbol: phase = 0/90/180/270 degrees */
static void qpsk_mod(complexf_t *sym, int dibit)
{
    /* dibit 0..3 → phase 45, 135, 225, 315 deg (before differential encoding) */
    double phase = (dibit * 90 + 45) * M_PI / 180.0;
    sym->re = cos(phase);
    sym->im = sin(phase);
}

/* Build the TX scrambler output for MP sync (17 ones, type=0) */
static int mp_tx_scramble(int *scrambler_reg, int tap, int bit)
{
    int out = (bit ^ (*scrambler_reg >> tap) ^ (*scrambler_reg >> 22)) & 1;
    *scrambler_reg = (*scrambler_reg << 1) | bit;
    return out;
}

int main(void)
{
    v34_state_t v34;
    v34_rx_state_t *rx;
    int calling_party = 0; /* We are the answerer */
    int carrier_freq = 1200; /* Answerer receives caller's CC at 1200 Hz */
    int sample_rate = 8000;
    double carrier_phase = 0.0;
    double carrier_phase_rate;
    int baud_rate_code = 5; /* CC baud rate */
    int filter_steps = 240; /* V34_RX_FILTER_STEPS */

    memset(&v34, 0, sizeof(v34));
    rx = &v34.rx;

    /* Initialize as answerer */
    rx->calling_party = calling_party;
    rx->baud_rate = baud_rate_code;
    rx->high_carrier = 0; /* Caller transmits at low carrier (1200 Hz) */
    rx->scrambler_tap = 17; /* Answerer RX uses GPC (caller's scrambler) */

    /* Set up carrier phase rate for the CC carrier */
    carrier_phase_rate = 2.0 * M_PI * carrier_freq / sample_rate;

    /* Initialize DDS phase */
    rx->carrier_phase = 0;

    /* Set up filter step */
    rx->rrc_filter_step = 0;
    memset(rx->rrc_filter, 0, sizeof(rx->rrc_filter));

    /* Initialize eq_put_step for CC */
    rx->eq_put_step = 12 * 40 / (3*2) - 1; /* 79 */

    /* Initialize AGC */
    rx->agc_scaling = 1.0;

    /* Initialize cc_symbol_sync state */
    memset(&rx->cc_ted, 0, sizeof(rx->cc_ted));
    rx->cc_ted.low_band_edge_coeff[0] = 2 * cos(2 * M_PI * (carrier_freq - 600) / sample_rate);
    rx->cc_ted.high_band_edge_coeff[0] = 2 * cos(2 * M_PI * (carrier_freq + 600) / sample_rate);

    /* Initialize state for process_cc_half_baud */
    rx->baud_half = 0;
    rx->scramble_reg = 0;
    rx->bitstream = 0;
    rx->mp_seen = -1;
    rx->mp_count = -1;
    rx->last_sample.re = 0;
    rx->last_sample.im = 0;

    printf("=== CC RX Diagnostic Test ===\n");
    printf("Carrier: %d Hz, Phase rate: %.6f rad/sample\n", carrier_freq, carrier_phase_rate);
    printf("Scrambler tap: %d, eq_put_step init: %d\n", rx->scrambler_tap, rx->eq_put_step);

    /* ---- Generate TX: MP sync = 17 ones (type=0) ---- */
    /* MP input bits: 17 ones + 0 (type=0 for MPh) + ... */
    int tx_scram_reg = 0;
    int tx_tap = 17; /* Caller's GPC */
    int mp_input[20]; /* First 10 bauds of MP = 20 bits */
    int mp_scrambled[20];
    int mp_diff_encoded[20];

    for (int i = 0; i < 17; i++) {
        mp_input[i] = 1;
    }
    mp_input[17] = 0; /* type bit = 0 for MPh */
    mp_input[18] = 0; /* padding */
    mp_input[19] = 0;

    /* Scramble */
    for (int i = 0; i < 20; i++) {
        mp_scrambled[i] = mp_tx_scramble(&tx_scram_reg, tx_tap, mp_input[i]);
    }

    printf("\nTX input bits: ");
    for (int i = 0; i < 20; i++) printf("%d", mp_input[i]);
    printf("\nTX scrambled:   ");
    for (int i = 0; i < 20; i++) printf("%d", mp_scrambled[i]);
    printf("\n");

    /* Differential encoding: dibit[n] = (dibit[n-1] + scrambled_pair[n]) mod 4 */
    int diff_state = 0;
    for (int b = 0; b < 10; b++) {
        int pair = (mp_scrambled[2*b+1] << 1) | mp_scrambled[2*b];
        mp_diff_encoded[b] = (diff_state + pair) & 3;
        diff_state = mp_diff_encoded[b];
    }

    printf("TX diff encoded: ");
    for (int i = 0; i < 10; i++) printf("%d", mp_diff_encoded[i]);
    printf("\n");

    /* ---- Generate passband signal ---- */
    /* At 1200 Hz carrier, 8000 samples/sec, 600 baud → 8000/600 ≈ 13.33 samples/baud */
    /* We generate the raw samples at the correct baud timing */
    int samples_per_baud = 8000 / 600; /* ~13 */
    int total_samples = samples_per_baud * 15; /* 15 bauds */

    int16_t *passband = malloc(total_samples * sizeof(int16_t));
    if (!passband) { fprintf(stderr, "malloc failed\n"); return 1; }

    carrier_phase = 0;
    int sample_idx = 0;
    for (int b = 0; b < 15; b++) {
        complexf_t sym;
        qpsk_mod(&sym, mp_diff_encoded[b % 10]);

        for (int s = 0; s < samples_per_baud; s++) {
            /* Passband = real(sym)*cos(carrier) - imag(sym)*sin(carrier) */
            double val = sym.re * cos(carrier_phase) - sym.im * sin(carrier_phase);
            passband[sample_idx++] = (int16_t)(val * 8000);
        }
        /* Advance carrier phase for one baud */
        carrier_phase += carrier_phase_rate * samples_per_baud;
        carrier_phase = fmod(carrier_phase, 2 * M_PI);
    }

    printf("\nGenerated %d passband samples for %d bauds\n", sample_idx, 15);

    /* ---- Feed through cc_rx ---- */
    printf("\nCalling cc_rx()...\n");
    int ret = cc_rx(rx, passband, sample_idx);
    printf("cc_rx returned %d\n", ret);

    printf("\nFinal scramble_reg: 0x%06x\n", rx->scramble_reg & 0xFFFFFF);
    printf("Final bitstream:    0x%05x\n", rx->bitstream & 0xFFFFF);
    printf("mp_seen: %d, mp_count: %d\n", rx->mp_seen, rx->mp_count);

    free(passband);

    return 0;
}
