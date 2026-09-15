/* High-resolution FFT of a window of ulaw audio to check for modulation sidebands. */
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

#define N 4096
static double re[N], im[N];

int main(int argc, char **argv)
{
    FILE *f = fopen(argv[1], "rb");
    uint8_t buf[N];
    double t0 = atof(argv[2]);
    long off = (long) (t0 * 8000.0);
    int i, k;
    fseek(f, off, SEEK_SET);
    fread(buf, 1, N, f);
    fclose(f);

    for (i = 0; i < N; i++)
    {
        double w = 0.5 - 0.5 * cos(2.0 * M_PI * i / (N - 1)); /* Hann */
        re[i] = ulaw_decode(buf[i]) * w;
        im[i] = 0.0;
    }
    /* DFT only in 500..3500 Hz region, coarse: just brute force bins of 1 Hz over a range of 1600 bins */
    for (k = 500; k <= 3500; k += 1)
    {
        double accr = 0.0, acci = 0.0;
        double w = 2.0 * M_PI * k / 8000.0;
        for (i = 0; i < N; i++)
        {
            accr += re[i] * cos(w * i);
            acci -= re[i] * sin(w * i);
        }
        double mag = sqrt(accr * accr + acci * acci) / N;
        if (mag > 30.0)
            printf("%5d Hz  %8.1f\n", k, mag);
    }
    return 0;
}
