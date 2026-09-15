/* Crude V.22 receiver for captured audio: 1200 Hz carrier, 600 baud QPSK.
   Prints the differential raw bits so S1 (alternating 3/0) can be spotted. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

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
    FILE *f;
    long n, i;
    uint8_t *buf;
    int16_t *pcm;
    double phase = 0.0;
    double samples_per_symbol = 8000.0 / 600.0;
    double sps;
    double t_start = (argc > 2) ? atof(argv[2]) : 0.0;
    double t_end = (argc > 3) ? atof(argv[3]) : 1e9;
    int count = 0;

    f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc((size_t) n);
    pcm = malloc((size_t) n * sizeof(int16_t));
    n = (long) fread(buf, 1, (size_t) n, f);
    fclose(f);
    for (i = 0; i < n; i++)
        pcm[i] = ulaw_decode(buf[i]);

    /* Mix down with 1200 Hz, integrate over one symbol, sample phase. */
    {
        double w = 2.0 * M_PI * 1200.0 / 8000.0;
        double acc_re, acc_im;
        int acc_n = 0;
        double sps_frac = 0.0;
        double phase_at = -1;
        double prev_phase = -1;
        long start = (long) (t_start * 8000.0);

        for (i = start; i < n && (double) i / 8000.0 < t_end; i++)
        {
            double x = pcm[i];
            acc_re += x * cos(w * i);
            acc_im += x * sin(w * i);
            acc_n++;
            sps_frac += 1.0;
            if (sps_frac >= samples_per_symbol)
            {
                sps_frac -= samples_per_symbol;
                phase = atan2(acc_im, acc_re);
                acc_re = acc_im = 0.0;
                acc_n = 0;
                if (phase_at < 0)
                {
                    phase_at = phase;
                }
                else
                {
                    double d = phase - phase_at;
                    while (d > M_PI) d -= 2 * M_PI;
                    while (d < -M_PI) d += 2 * M_PI;
                    {
                        int q = (int) floor(d / (M_PI / 2) + 0.5) & 3;
                        int qq = (q + 4) & 3;
                        printf("sym %5d  t=%.3f  dphi=%7.1f  raw=%d\n",
                               count, (double) i / 8000.0, d * 180.0 / M_PI, qq);
                        count++;
                    }
                    phase_at = phase;
                }
                (void) prev_phase;
            }
        }
    }
    free(buf);
    free(pcm);
    return 0;
}
