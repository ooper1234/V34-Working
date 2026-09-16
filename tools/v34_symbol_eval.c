/* Offline: run the RX front-end replica over the ideal waveform and, for
   every timing phase, measure the recovered symbol EVM against the known
   transmitted symbols. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "v34_rx_2400_high_carrier_rrc.h"
#define FSTEPS 27
#define CSETS 192
#define TS 9

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
    FILE *sf = fopen(argv[2], "r");
    long n;
    uint8_t ub[200000];
    int16_t pcm[200000];
    float ts_re[20000], ts_im[20000];
    long nt = 0;
    float rrc[FSTEPS];
    int best_phase = -1;
    double best_err = 1e30;

    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    n = (long) fread(ub, 1, (size_t) n, f);
    fclose(f);
    {
        long i;
        for (i = 0; i < n; i++)
            pcm[i] = ulaw_decode(ub[i]);
    }
    while (nt < 20000 && fscanf(sf, "%f %f", &ts_re[nt], &ts_im[nt]) == 2)
        nt++;
    fclose(sf);

    {
        int P;
        for (P = 0; P < 640; P += 8)
        {
            long i;
            int eq_put_step = P;
            int rrc_step = 0;
            uint32_t ph = 0;
            int32_t rate = (int32_t) (1800.0/8000.0*4294967296.0);
            float rxr[4000], rxi[4000];
            long nrx = 0, e;
            long k;
            double num_re = 0, num_im = 0, den = 0, err = 0;
            long cnt = 0;
            long best_d = 0;
            double best_fit = 1e30;
            long d;
            int o;

            memset(rrc, 0, sizeof(rrc));
            for (i = 0; i < n; i++)
            {
                float ii = 0, qq = 0;
                int j, m, step;

                rrc[rrc_step] = (float) pcm[i];
                if (++rrc_step >= FSTEPS)
                    rrc_step = 0;
                eq_put_step -= CSETS;
                step = -eq_put_step;
                if (step > CSETS - 1)
                    step = CSETS - 1;
                while (step < 0)
                    step += CSETS;
                m = rrc_step;
                for (j = 0; j < FSTEPS; j++)
                {
                    ii += rrc[m]*rx_pulseshaper_2400_high_carrier_re[step][j];
                    qq += rrc[m]*rx_pulseshaper_2400_high_carrier_im[step][j];
                    if (++m >= FSTEPS)
                        m = 0;
                }
                if (eq_put_step <= 0)
                {
                    double zr = cos(2.0*M_PI*(double) ph/4294967296.0);
                    double zi = sin(2.0*M_PI*(double) ph/4294967296.0);
                    double sr = ii*0.0017, si = qq*0.0017;
                    double re = sr*zr - si*zi;
                    double im = -sr*zi - si*zr;
                    if (nrx < 4000)
                    {
                        rxr[nrx] = re;
                        rxi[nrx] = im;
                    }
                    nrx++;
                    eq_put_step += CSETS*8000/(2400*2);
                }
                ph += (uint32_t) rate;
            }
            /* The emission stream has two samples per symbol. Try both
               parities and a small symbol delay, and fit the complex gain. */
            for (o = 0; o < 2; o++)
            for (d = 0; d < 3; d++)
            {
                double nre = 0, nim = 0, dn = 0, er = 0;
                long c = 0;
                long kmax = nt - d;
                if (kmax > nrx/2 - d)
                    kmax = nrx/2 - d;
                for (k = 0; k < kmax; k++)
                {
                    double xr = rxr[2*(k + d) + o];
                    double xi = rxi[2*(k + d) + o];
                    nre += ts_re[k + d]*xr + ts_im[k + d]*xi;
                    nim += ts_im[k + d]*xr - ts_re[k + d]*xi;
                    dn += xr*xr + xi*xi;
                    c++;
                }
                if (dn > 1e-9 && c > 100)
                {
                    double kr = nre/dn, ki = nim/dn;
                    er = 0;
                    for (k = 0; k < kmax; k++)
                    {
                        double xr = rxr[2*(k + d) + o];
                        double xi = rxi[2*(k + d) + o];
                        double tr = ts_re[k + d];
                        double ti = ts_im[k + d];
                        double er_r = tr - (kr*xr - ki*xi);
                        double er_i = ti - (kr*xi + ki*xr);
                        er += er_r*er_r + er_i*er_i;
                    }
                    er = sqrt(er/c);
                    if (er < best_fit)
                    {
                        best_fit = er;
                        best_d = d;
                    }
                }
            }
            (void) e;
            (void) num_re; (void) num_im; (void) den; (void) err; (void) cnt; (void) best_d;
            if (best_fit < best_err)
            {
                best_err = best_fit;
                best_phase = P;
            }
        }
        printf("best timing phase %d, residual RMS %.2f (tx symbol scale 128=1 unit)\n",
               best_phase, best_err);
        if (getenv("SYM_EVAL_PAIRS"))
        {
            /* Re-run the best phase and print fitted pairs. */
            int P = best_phase;
            long i;
            int eq_put_step = P;
            int rrc_step = 0;
            uint32_t ph = 0;
            int32_t rate = (int32_t) (1800.0/8000.0*4294967296.0);
            float rxr[4000], rxi[4000];
            long nrx = 0, k;
            int o, d;
            double nre = 0, nim = 0, dn = 0;
            double kr = 0, ki = 0;

            memset(rrc, 0, sizeof(rrc));
            for (i = 0; i < n; i++)
            {
                float ii = 0, qq = 0;
                int j, m, step;

                rrc[rrc_step] = (float) pcm[i];
                if (++rrc_step >= FSTEPS)
                    rrc_step = 0;
                eq_put_step -= CSETS;
                step = -eq_put_step;
                if (step > CSETS - 1)
                    step = CSETS - 1;
                while (step < 0)
                    step += CSETS;
                m = rrc_step;
                for (j = 0; j < FSTEPS; j++)
                {
                    ii += rrc[m]*rx_pulseshaper_2400_high_carrier_re[step][j];
                    qq += rrc[m]*rx_pulseshaper_2400_high_carrier_im[step][j];
                    if (++m >= FSTEPS)
                        m = 0;
                }
                if (eq_put_step <= 0)
                {
                    double zr = cos(2.0*M_PI*(double) ph/4294967296.0);
                    double zi = sin(2.0*M_PI*(double) ph/4294967296.0);
                    double sr = ii*0.0017, si = qq*0.0017;
                    if (nrx < 4000)
                    {
                        rxr[nrx] = sr*zr - si*zi;
                        rxi[nrx] = -sr*zi - si*zr;
                    }
                    nrx++;
                    eq_put_step += CSETS*8000/(2400*2);
                }
                ph += (uint32_t) rate;
            }
            /* Refit for the same parity/delay choices and print pairs. */
            for (o = 0; o < 2; o++)
            for (d = 0; d < 3; d++)
            {
                long kmax = nt - d;
                nre = nim = dn = 0;
                if (kmax > nrx/2 - d)
                    kmax = nrx/2 - d;
                if (kmax < 100)
                    continue;
                for (k = 0; k < kmax; k++)
                {
                    double xr = rxr[2*(k + d) + o];
                    double xi = rxi[2*(k + d) + o];
                    nre += ts_re[k + d]*xr + ts_im[k + d]*xi;
                    nim += ts_im[k + d]*xr - ts_re[k + d]*xi;
                    dn += xr*xr + xi*xi;
                }
                kr = nre/dn;
                ki = nim/dn;
                printf("parity %d delay %ld gain |k|=%.3f ang=%.1f: ",
                       o, d, hypot(kr, ki), atan2(ki, kr)*180.0/M_PI);
                for (k = 0; k < 10; k++)
                {
                    double xr = rxr[2*(k + d) + o];
                    double xi = rxi[2*(k + d) + o];
                    printf("(%d,%d)->(%.0f,%.0f) ", (int) ts_re[k + d], (int) ts_im[k + d],
                           kr*xr - ki*xi, kr*xi + ki*xr);
                }
                printf("\n");
            }
        }
    }
    return 0;
}
