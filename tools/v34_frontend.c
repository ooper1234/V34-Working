/* Offline replica of v34rx primary_channel_rx front end, to validate the RX
   RRC tables and the timing grid against the captured TX waveform. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "v34_rx_2400_high_carrier_rrc.h"

#define FILTER_STEPS 27
#define COEFF_SETS   192

static int16_t ulaw_decode(uint8_t u)
{
    int t;
    u = (uint8_t) ~u;
    t = ((u & 0x0F) << 3) + 0x84;
    t <<= (u & 0x70) >> 4;
    return (int16_t) ((u & 0x80) ? (0x84 - t) : (t - 0x84));
}

int main(int argc, char **argv)
{
    FILE *f = fopen(argv[1], "rb");
    double t0 = atof(argv[2]);
    long n;
    uint8_t *buf;
    float rrc[FILTER_STEPS];
    int rrc_step = 0;
    long i;
    int eq_put_step;
    uint32_t phase = 0;
    int32_t rate;
    int emits = 0;
    long emit_min = (argc > 3) ? atol(argv[3]) : 2000;
    long emit_max = (argc > 4) ? atol(argv[4]) : 2020;
    int init_phase = (argc > 5) ? atoi(argv[5]) : 0;

    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc((size_t) n);
    n = (long) fread(buf, 1, (size_t) n, f);
    fclose(f);

    memset(rrc, 0, sizeof(rrc));
    eq_put_step = -init_phase;   /* negative -> first emission after init_phase samples */
    rate = (int32_t) (1800.0/8000.0*4294967296.0);

    for (i = (long) (t0*8000.0); i < n; i++)
    {
        float ii = 0.0f, qq = 0.0f;
        int j, k, step;

        rrc[rrc_step] = (float) ulaw_decode(buf[i]);
        if (++rrc_step >= FILTER_STEPS)
            rrc_step = 0;

        eq_put_step -= COEFF_SETS;
        step = -eq_put_step;
        if (step > COEFF_SETS - 1)
            step = COEFF_SETS - 1;
        while (step < 0)
            step += COEFF_SETS;

        k = rrc_step;
        for (j = 0; j < FILTER_STEPS; j++)
        {
            ii += rrc[k]*rx_pulseshaper_2400_high_carrier_re[step][j];
            qq += rrc[k]*rx_pulseshaper_2400_high_carrier_im[step][j];
            if (++k >= FILTER_STEPS)
                k = 0;
        }
        if (eq_put_step <= 0)
        {
            double zr = cos(2.0*M_PI*(double) phase/4294967296.0);
            double zi = sin(2.0*M_PI*(double) phase/4294967296.0);
            double sr = ii*0.0017;
            double si = qq*0.0017;
            double re = sr*zr - si*zi;
            double im = -sr*zi - si*zr;

            if (emits >= emit_min && emits < emit_max)
                printf("emit %ld %.3f %.3f\n", emits, re, im);
            emits++;
            eq_put_step += COEFF_SETS*8000/(2400*2);
        }
        phase += (uint32_t) rate;
    }
    fprintf(stderr, "total emissions %d\n", emits);
    free(buf);
    return 0;
}
