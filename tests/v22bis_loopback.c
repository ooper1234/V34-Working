#include "modem/v22bis/v22bis.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHUNK 160
#define MAX_BITS 48000

typedef struct {
    uint32_t lfsr;
} bitsrc_t;

static int src_bit(void *user_data)
{
    bitsrc_t *b = user_data;
    uint32_t v;

    b->lfsr ^= b->lfsr >> 7;
    v = b->lfsr & 1;
    b->lfsr >>= 1;
    return (int) v;
}

typedef struct {
    int rx_bits[MAX_BITS];
    int n;
} bitsink_t;

static void sink_bit(void *user_data, int bit)
{
    bitsink_t *s = user_data;
    if (s->n < MAX_BITS)
        s->rx_bits[s->n++] = bit & 1;
}

static void status_cb(void *user_data, v22bis_status_t status)
{
    (void) user_data;
    const char *name = user_data ? NULL : NULL;
    (void) name;
    const char *names[] = {"TRAINED-OK", "TRAIN-FAIL", "CARRIER-UP", "CARRIER-DOWN", "RETRAIN"};
    printf("    [status] %s\n", names[status]);
}

int main(int argc, char *argv[])
{
    v22bis_state_t caller_modem;
    v22bis_state_t answer_modem;
    int rate = (argc > 1) ? atoi(argv[1]) : 2400;
    int16_t caller_tx[CHUNK];
    int16_t answer_tx[CHUNK];
    int16_t silence[CHUNK];
    bitsrc_t caller_src;
    bitsrc_t answer_src;
    bitsink_t caller_rx;
    bitsink_t answer_rx;
    long total = 0;
    long stats_at = 0;
    int i;

    memset(&silence, 0, sizeof(silence));

    caller_src.lfsr = 0x13579bdfu;
    answer_src.lfsr = 0x2468ace0u;
    memset(&caller_rx, 0, sizeof(caller_rx));
    memset(&answer_rx, 0, sizeof(answer_rx));

    v22bis_init(&caller_modem, true, rate,
                src_bit, &caller_src,
                sink_bit, &caller_rx,
                status_cb, NULL);
    v22bis_init(&answer_modem, false, rate,
                src_bit, &answer_src,
                sink_bit, &answer_rx,
                status_cb, NULL);

    while (total < 8000 * 20)
    {
        v22bis_tx(&caller_modem, caller_tx, CHUNK);
        v22bis_tx(&answer_modem, answer_tx, CHUNK);
        v22bis_rx(&caller_modem, answer_tx, CHUNK);
        v22bis_rx(&answer_modem, caller_tx, CHUNK);

        total += CHUNK;
        if (total >= stats_at + 8000 / 50)
        {
            stats_at = total;
            if (total < 8000 * 4 || total / 8000 == (total - 1) / 8000)
            {
                printf("t=%.3fs C:(tx=%d rx=%d neg=%d,%d) A:(tx=%d rx=%d neg=%d,%d)\n",
                       (double) total / 8000.0,
                       caller_modem.tx.training, caller_modem.rx.training,
                       caller_modem.negotiated_bit_rate, caller_rx.n,
                       answer_modem.tx.training, answer_modem.rx.training,
                       answer_modem.negotiated_bit_rate, answer_rx.n);
            }
        }
    }

    printf("\n--- finished at %.3f s ---\n", (double) total / 8000.0);
    printf("caller rx: %d bits, answerer rx: %d bits\n",
           caller_rx.n, answer_rx.n);
    printf("caller TX stage %d, RX stage %d, neg %d\n",
           caller_modem.tx.training, caller_modem.rx.training,
           caller_modem.negotiated_bit_rate);
    printf("answer TX stage %d, RX stage %d, neg %d\n",
           answer_modem.tx.training, answer_modem.rx.training,
           answer_modem.negotiated_bit_rate);

    /* Regenerate the two source PRBS streams and compare against what each
       receiver decoded. The caller RX decodes the answerer's TX stream and
       vice versa. Training + the far-end's leftover S1111 come first, so
       align on the steady-data window rx[4000..6000], then measure BER over
       the whole captured region. */
    {
        int c_off = -1, a_off = -1;
        int c_best = -1, a_best = -1;
        int c_err = 0, a_err = 0;
        int off;
        int i2;

        for (off = 0; off < 2400 * 10; off++)
        {
            int err = 0;
            bitsrc_t a = answer_src;
            for (i = 0; i < off; i++)
                (void) src_bit(&a);
            for (i2 = 4000; i2 < 6000 && i2 < caller_rx.n; i2++)
                if (src_bit(&a) != caller_rx.rx_bits[i2])
                    err++;
            if (err < c_best || c_best < 0)
            {
                c_best = err;
                c_off = off;
            }
        }
        {
            bitsrc_t a = answer_src;
            for (i = 0; i < c_off; i++)
                (void) src_bit(&a);
            for (i = 0; i < caller_rx.n; i++)
                if (src_bit(&a) != caller_rx.rx_bits[i])
                    c_err++;
        }

        for (off = 0; off < 2400 * 10; off++)
        {
            int err = 0;
            bitsrc_t c = caller_src;
            for (i = 0; i < off; i++)
                (void) src_bit(&c);
            for (i2 = 4000; i2 < 6000 && i2 < answer_rx.n; i2++)
                if (src_bit(&c) != answer_rx.rx_bits[i2])
                    err++;
            if (err < a_best || a_best < 0)
            {
                a_best = err;
                a_off = off;
            }
        }
        {
            bitsrc_t c = caller_src;
            for (i = 0; i < a_off; i++)
                (void) src_bit(&c);
            for (i = 0; i < answer_rx.n; i++)
                if (src_bit(&c) != answer_rx.rx_bits[i])
                    a_err++;
        }

        printf("caller rx: %d bits @ src offset %d -> %d errors (%.5f%%)\n",
               caller_rx.n, c_off, c_err,
               caller_rx.n ? 100.0 * c_err / caller_rx.n : 0.0);
        printf("answerer rx: %d bits @ src offset %d -> %d errors (%.5f%%)\n",
               answer_rx.n, a_off, a_err,
               answer_rx.n ? 100.0 * a_err / answer_rx.n : 0.0);
        printf("caller rx steady (post-bit %d): ", 1000);
        {
            bitsrc_t a = answer_src;
            int e = 0;
            int t = 0;
            for (i = 0; i < c_off; i++)
                (void) src_bit(&a);
            for (i = 1000; i < caller_rx.n; i++)
            {
                if (src_bit(&a) != caller_rx.rx_bits[i])
                    e++;
                t++;
            }
            printf("%d errors in %d bits\n", e, t);
        }
        printf("answerer rx steady (post-bit %d): ", 1000);
        {
            bitsrc_t c = caller_src;
            int e = 0;
            int t = 0;
            for (i = 0; i < a_off; i++)
                (void) src_bit(&c);
            for (i = 1000; i < answer_rx.n; i++)
            {
                if (src_bit(&c) != answer_rx.rx_bits[i])
                    e++;
                t++;
            }
            printf("%d errors in %d bits\n", e, t);
        }
    }

    return 0;
}