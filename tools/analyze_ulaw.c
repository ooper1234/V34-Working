/* Analyze a mu-law capture: RMS + dominant frequency per 100 ms window. */
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
    long n;
    uint8_t *buf;
    int16_t *pcm;
    long i;
    int win = 800;              /* samples per window */
    int show = (argc > 2) ? atoi(argv[2]) : 1;

    if (argc < 2)
    {
        fprintf(stderr, "usage: %s file.ulaw [show_all]\n", argv[0]);
        return 2;
    }
    f = fopen(argv[1], "rb");
    if (!f)
    {
        perror(argv[1]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc((size_t) n);
    pcm = malloc((size_t) n * sizeof(int16_t));
    n = (long) fread(buf, 1, (size_t) n, f);
    fclose(f);
    for (i = 0; i < n; i++)
        pcm[i] = ulaw_decode(buf[i]);

    for (i = 0; i + win <= n; i += win)
    {
        double rms = 0;
        int fbin, best_bin = 0;
        double best_p = 0;
        int k;

        for (k = 0; k < win; k++)
            rms += (double) pcm[i + k] * pcm[i + k];
        rms = sqrt(rms / win);

        /* spectral peak scan 300..3400 Hz in 50 Hz steps */
        for (fbin = 300; fbin <= 3400; fbin += 50)
        {
            double w = 2.0 * M_PI * fbin / 8000.0;
            double cr = 0, ci = 0;
            for (k = 0; k < win; k++)
            {
                cr += pcm[i + k] * cos(w * k);
                ci += pcm[i + k] * -sin(w * k);
            }
            {
                double p = cr * cr + ci * ci;
                if (p > best_p)
                {
                    best_p = p;
                    best_bin = fbin;
                }
            }
        }
        if (show || rms > 300)
            printf("t=%7.3f rms=%7.1f peak=%4d Hz\n", (double) i / 8000.0, rms, best_bin);
    }
    free(buf);
    free(pcm);
    return 0;
}
