/* cc_path_test.c - Isolated unit tests for the V.34 CC data path.
 *
 * Tests:
 *  1. Scrambler/descrambler round-trip (self-synchronizing pair)
 *  2. QPSK dibit mapper/demapper round-trip
 *  3. Combined: data -> scramble -> QPSK -> demap -> descramble -> data
 *
 * All tests use the SAME algorithms as v34tx.c and v34rx.c.
 * If this test passes, the CC path logic is correct in isolation. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ---------- Scrambler (from v34tx.c) ---------- */
static int tx_scramble(uint32_t *reg, int tap, int in_bit)
{
    int out_bit = (*reg >> tap) ^ (*reg >> 22);
    out_bit = (out_bit ^ in_bit) & 1;
    *reg = (*reg << 1) | out_bit;
    return out_bit;
}

/* ---------- Descrambler (from v34rx.c) ---------- */
static int rx_descramble(uint32_t *reg, int tap, int in_bit)
{
    int out_bit = (*reg >> tap) ^ (*reg >> 22);
    out_bit = (out_bit ^ in_bit) & 1;
    *reg = (*reg << 1) | in_bit;
    return out_bit;
}

/* ---------- QPSK constellation (from v34tx.c, original spandsp) ---------- */
/* training_constellation_4[diff] = (re, im) at baseband.
 * diff=0: 225°, diff=1: 135°, diff=2: 45°, diff=3: 315°
 * The non-standard mapping compensates for arctan2(re,im) transposing I/Q. */
static const float qpsk_re[4] = { -0.7071068f, -0.7071068f,  0.7071068f,  0.7071068f };
static const float qpsk_im[4] = { -0.7071068f,  0.7071068f,  0.7071068f, -0.7071068f };

/* DDS_PHASE(x) = x * 2^32 / 360 */
#define DDS_PHASE(x) ((uint32_t)((x) * (4294967296.0 / 360.0)))

/* ---------- Test 1: scrambler/descrambler round-trip ---------- */
static int test_scrambler_descrambler(void)
{
    int tap = 17;   /* caller */
    uint32_t tx_reg = 0, rx_reg = 0;
    int errors = 0;
    int N = 2000;

    /* Generate known data bits */
    int *orig = malloc(N * sizeof(int));
    int *scrambled = malloc(N * sizeof(int));
    int *recovered = malloc(N * sizeof(int));

    for (int i = 0; i < N; i++)
        orig[i] = (i * 7 + 3) & 1;  /* deterministic pseudo-random */

    /* Scramble */
    for (int i = 0; i < N; i++)
        scrambled[i] = tx_scramble(&tx_reg, tap, orig[i]);

    /* Descramble - should recover original bits after sync */
    for (int i = 0; i < N; i++)
        recovered[i] = rx_descramble(&rx_reg, tap, scrambled[i]);

    /* After 23 bits of sync, all bits should match */
    for (int i = 23; i < N; i++)
    {
        if (recovered[i] != orig[i])
        {
            if (errors < 10)
                printf("  SCRAM ERR at bit %d: expected %d got %d\n", i, orig[i], recovered[i]);
            errors++;
        }
    }

    free(orig);
    free(scrambled);
    free(recovered);

    printf("Test 1 (scrambler/descrambler, tap=%d): %s (%d errors in %d bits after sync)\n",
           tap, errors ? "FAIL" : "PASS", errors, N - 23);
    return errors;
}

/* ---------- Test 2: QPSK dibit mapper/demapper ---------- */
static int test_qpsk_roundtrip(void)
{
    int errors = 0;
    int prev_diff = 0;

    printf("Test 2 (QPSK dibit round-trip):\n");

    for (int dibit = 0; dibit < 4; dibit++)
    {
        /* TX: diff = (prev_diff + dibit) & 3; map to constellation point */
        int diff = (prev_diff + dibit) & 3;
        float tx_re = qpsk_re[diff];
        float tx_im = qpsk_im[diff];

        /* RX: compute phase difference with 45° offset */
        float prev_re = qpsk_re[prev_diff];
        float prev_im = qpsk_im[prev_diff];

        /* ang1 = atan2(tx_re, tx_im), ang2 = atan2(prev_re, prev_im) */
        float ang1 = atan2f(tx_re, tx_im);
        float ang2 = atan2f(prev_re, prev_im);
        float ang3 = ang1 - ang2 + (45.0f * (float)M_PI / 180.0f);

        /* Normalize to [0, 2*PI) */
        while (ang3 < 0) ang3 += 2.0f * (float)M_PI;
        while (ang3 >= 2.0f * (float)M_PI) ang3 -= 2.0f * (float)M_PI;

        /* Convert to 32-bit phase and extract top 2 bits */
        uint32_t ang3_32 = (uint32_t)(ang3 * (4294967296.0 / (2.0 * M_PI)));
        int rx_dibit = ang3_32 >> 30;

        if (rx_dibit != dibit)
        {
            printf("  QPSK ERR: dibit=%d -> diff=%d -> ang3=%.1f° -> rx_dibit=%d\n",
                   dibit, diff, ang3 * 180.0f / (float)M_PI, rx_dibit);
            errors++;
        }
        else
        {
            printf("  dibit=%d -> diff=%d -> ang3=%.1f° -> rx_dibit=%d  OK\n",
                   dibit, diff, ang3 * 180.0f / (float)M_PI, rx_dibit);
        }

        prev_diff = diff;
    }

    printf("Test 2: %s (%d errors)\n", errors ? "FAIL" : "PASS", errors);
    return errors;
}

/* ---------- Test 3: Full path: data -> scramble -> QPSK -> demap -> descramble ---------- */
static int test_full_path(void)
{
    int tap = 17;
    uint32_t tx_scram_reg = 0, rx_descram_reg = 0;
    int prev_diff = 0;
    int errors = 0;
    int N = 500;

    int *orig = malloc(N * sizeof(int));
    int *recovered = malloc(N * sizeof(int));

    /* Generate known data */
    for (int i = 0; i < N; i++)
        orig[i] = (i * 13 + 7) & 1;

    /* TX: scramble 2 bits at a time, QPSK modulate */
    float *tx_re = malloc(N * sizeof(float));
    float *tx_im = malloc(N * sizeof(float));

    for (int i = 0; i < N; i += 2)
    {
        int b0 = tx_scramble(&tx_scram_reg, tap, orig[i]);
        int b1 = tx_scramble(&tx_scram_reg, tap, orig[i + 1]);
        int dibit = (b1 << 1) | b0;
        int diff = (prev_diff + dibit) & 3;
        tx_re[i / 2] = qpsk_re[diff];
        tx_im[i / 2] = qpsk_im[diff];
        prev_diff = diff;
    }

    /* RX: demap QPSK, descramble */
    prev_diff = 0;
    for (int i = 0; i < N; i += 2)
    {
        /* Ideal demap: find closest constellation point */
        float best_re = tx_re[i / 2];
        float best_im = tx_im[i / 2];

        /* Recover phase difference */
        float ang1 = atan2f(best_re, best_im);
        float ang2;
        if (i == 0)
        {
            /* First symbol: use the known previous (diff=0 -> 225°) */
            ang2 = atan2f(qpsk_re[0], qpsk_im[0]);
        }
        else
        {
            ang2 = atan2f(tx_re[i / 2 - 1], tx_im[i / 2 - 1]);
        }

        float ang3 = ang1 - ang2 + (45.0f * (float)M_PI / 180.0f);
        while (ang3 < 0) ang3 += 2.0f * (float)M_PI;
        while (ang3 >= 2.0f * (float)M_PI) ang3 -= 2.0f * (float)M_PI;

        uint32_t ang3_32 = (uint32_t)(ang3 * (4294967296.0 / (2.0 * M_PI)));
        int rx_dibit = ang3_32 >> 30;

        int rx_b0 = rx_descramble(&rx_descram_reg, tap, rx_dibit & 1);
        int rx_b1 = rx_descramble(&rx_descram_reg, tap, (rx_dibit >> 1) & 1);

        recovered[i] = rx_b0;
        recovered[i + 1] = rx_b1;
    }

    /* Check after sync */
    for (int i = 23; i < N; i++)
    {
        if (recovered[i] != orig[i])
        {
            if (errors < 10)
                printf("  FULL ERR at bit %d: expected %d got %d\n", i, orig[i], recovered[i]);
            errors++;
        }
    }

    free(orig);
    free(recovered);
    free(tx_re);
    free(tx_im);

    printf("Test 3 (full path): %s (%d errors in %d bits after sync)\n",
           errors ? "FAIL" : "PASS", errors, N - 23);
    return errors;
}

int main(void)
{
    int total = 0;

    printf("=== CC Path Unit Tests ===\n\n");
    total += test_scrambler_descrambler();
    printf("\n");
    total += test_qpsk_roundtrip();
    printf("\n");
    total += test_full_path();
    printf("\n");

    if (total == 0)
        printf("ALL TESTS PASSED\n");
    else
        printf("TESTS FAILED (%d total errors)\n", total);

    return total ? 1 : 0;
}
