/* V.21 channel-1 (980/1180 Hz, 300 baud) FSK demodulator for ulaw captures.
   Decodes bytes (start bit + 8 data bits, 10 bits/frame) and prints them with
   timestamps, so V.8 CM/CI transmissions can be inspected offline. */
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
    long capn;
    uint8_t *buf;
    int16_t *pcm;
    long i;
    double t0 = (argc > 2) ? atof(argv[2]) : 0.0;
    double t1 = (argc > 3) ? atof(argv[3]) : 1e9;
    double spb = 8000.0 / 300.0;   /* samples per bit */
    long start = (long) (t0 * 8000.0);
    long end;
    double phase = 0.0;
    double prev_phase = 0.0;
    int have_prev = 0;
    int last_bit = -1;
    int bit_count = 0;
    int byte_val = 0;
    int in_frame = 0;
    double frame_start = 0;
    int level_energy = 0;

    fseek(f, 0, SEEK_END);
    capn = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc((size_t) capn);
    pcm = malloc((size_t) capn * sizeof(int16_t));
    capn = (long) fread(buf, 1, (size_t) capn, f);
    fclose(f);
    for (i = 0; i < capn; i++)
        pcm[i] = ulaw_decode(buf[i]);
    end = (long) (t1 * 8000.0);
    if (end > capn)
        end = capn;

    for (i = start; i < end; i++)
    {
        double dphase;
        int bit;
        /* Correlate against mark and space over one bit period. */
        {
            double mr = 0.0, mi = 0.0, sr = 0.0, si = 0.0;
            long j;
            long n = (long) spb;
            if (i + n > capn)
                break;
            for (j = 0; j < n; j++)
            {
                double wm = 2.0 * M_PI * 1180.0 * j / 8000.0;
                double ws = 2.0 * M_PI * 980.0 * j / 8000.0;
                double x = pcm[i + j];
                mr += x * cos(wm);
                mi -= x * sin(wm);
                sr += x * cos(ws);
                si -= x * sin(ws);
            }
            dphase = 0; /* unused */
            bit = (mr * mr + mi * mi) > (sr * sr + si * si) ? 1 : 0;
            level_energy = (int) (sqrt((mr * mr + mi * mi + sr * sr + si * si) / 2.0) / n);

            i += (long) spb - 1;  /* advance one bit */
        }
        if (level_energy < 300)
        {
            if (in_frame)
            {
                if (bit_count != 0)
                    printf("t=%.3f  [frame aborted after %d bits]\n",
                           (double) i / 8000.0, bit_count);
                in_frame = 0;
            }
            last_bit = -1;
            bit_count = 0;
            continue;
        }
        if (!in_frame)
        {
            /* Hunt for start bit (mark->space transition). */
            if (last_bit == 1 && bit == 0)
            {
                in_frame = 1;
                bit_count = 0;
                byte_val = 0;
                frame_start = (double) i / 8000.0;
            }
        }
        else
        {
            /* Sample each bit; data bits are LSB first after the start bit. */
            if (bit_count >= 1 && bit_count <= 8)
                byte_val |= (bit << (bit_count - 1));
            bit_count++;
            if (bit_count == 10)
            {
                int stop_ok = bit;
                printf("t=%.3f  byte=0x%02x (%c) stop=%d\n",
                       frame_start, byte_val & 0xFF,
                       (byte_val >= 32 && byte_val < 127) ? byte_val : '.',
                       stop_ok);
                in_frame = 0;
                bit_count = 0;
            }
        }
        last_bit = bit;
        (void) phase; (void) prev_phase; (void) have_prev;
    }
    free(buf);
    free(pcm);
    return 0;
}
