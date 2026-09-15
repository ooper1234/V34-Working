/* Replay a ulaw capture through the same spanDSP-based V.8 answerer the
   daemon uses, printing every status event with its sample offset. */
#include "sm_v8.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

static long g_sample_off;

static void result_cb(void *ud, int status, int mods, int cf, int proto)
{
    printf("t=%.3f  status=%d mods=0x%x cf=%d proto=%d\n",
           (double) g_sample_off / 8000.0, status, mods, cf, proto);
    fflush(stdout);
    (void) ud;
}

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
    uint8_t *buf;
    int16_t *pcm;
    long n, i;
    double t0 = (argc > 2) ? atof(argv[2]) : 0.0;
    double t1 = (argc > 3) ? atof(argv[3]) : 1e9;
    sm_v8_t *v;
    int16_t out[160];

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

    v = sm_v8_create(false, 0x04, result_cb, NULL, 0);
    if (!v) { fprintf(stderr, "sm_v8_create failed\n"); return 1; }

    for (i = (long) (t0 * 8000.0); i < n && (double) i / 8000.0 < t1; i += 160)
    {
        int len = (int) ((n - i < 160) ? (n - i) : 160);
        g_sample_off = i;
        sm_v8_rx(v, pcm + i, len);
        (void) out;
    }
    sm_v8_destroy(v);
    free(buf);
    free(pcm);
    return 0;
}
