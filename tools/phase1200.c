/* Track the phase of the 1200 Hz component of a ulaw capture over time to
   detect the V.34 phase-1 "A" signal (1200 Hz tone with 180-degree phase
   reversals at ~600 ms intervals). */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

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
    double t1 = atof(argv[3]);
    long off = (long) (t0 * 8000.0);
    long n = (long) ((t1 - t0) * 8000.0);
    uint8_t *buf = malloc((size_t) n);
    long i;
    double last_phase = 0.0;
    int have = 0;

    fseek(f, off, SEEK_SET);
    n = (long) fread(buf, 1, (size_t) n, f);
    fclose(f);

    for (i = 0; i + 800 <= n; i += 800)
    {
        double accr = 0.0, acci = 0.0;
        int j;
        for (j = 0; j < 800; j++)
        {
            double w = 2.0 * M_PI * 1200.0 * j / 8000.0;
            double x = ulaw_decode(buf[i + j]);
            accr += x * cos(w);
            acci -= x * sin(w);
        }
        {
            double ph = atan2(acci, accr);
            double rms = sqrt(accr * accr + acci * acci) / 400.0;
            double d = ph - last_phase;
            while (d > M_PI) d -= 2 * M_PI;
            while (d < -M_PI) d += 2 * M_PI;
            printf("t=%6.3f  rms=%8.1f  phase=%7.1f deg  dphase=%7.1f\n",
                   t0 + (double) i / 8000.0, rms, ph * 180.0 / M_PI, d * 180.0 / M_PI);
            last_phase = ph;
            have = 1;
        }
    }
    (void) have;
    free(buf);
    return 0;
}
