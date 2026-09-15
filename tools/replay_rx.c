/* Replay a mu-law capture through the V.22bis answerer receiver offline,
   printing the training progression. */
#include "modem/v22bis/v22bis.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int dummy_bit(void *ud) { (void) ud; return 1; }
static void dummy_put(void *ud, int bit) { (void) ud; (void) bit; }

static int16_t ulaw_decode(uint8_t u)
{
    int t;
    u = (uint8_t) ~u;
    t = ((u & 0x0F) << 3) + 0x84;
    t <<= (u & 0x70) >> 4;
    return (int16_t) ((u & 0x80) ? (0x84 - t) : (t - 0x84));
}

static const char *rx_names[] = {
    "NORMAL", "SYM_ACQ", "UNSCR_ON", "UNSCR_ON_SUST", "SCR_ON_1200", "SCR_ON_1200_SUST",
    "WAIT_SCR_2400", "PARKED"
};

static void status_cb(void *ud, v22bis_status_t st)
{
    v22bis_state_t *m = ud;
    printf("[status] %d  rx_state=%s tx_state=%d neg=%d\n", st,
           rx_names[m->rx.training], m->tx.training, m->negotiated_bit_rate);
}

int main(int argc, char **argv)
{
    FILE *f;
    uint8_t *buf;
    int16_t *pcm;
    long n, i;
    v22bis_state_t modem;
    long t0 = (argc > 2) ? atol(argv[2]) : 0;   /* start sample */
    long t1 = (argc > 3) ? atol(argv[3]) : 0;   /* end sample (0=all) */

    if (argc < 2)
    {
        fprintf(stderr, "usage: %s file.ulaw [start_sample] [end_sample]\n", argv[0]);
        return 2;
    }
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
    if (t1 == 0)
        t1 = n;

    v22bis_init(&modem, false, 2400, dummy_bit, NULL, dummy_put, NULL, status_cb, &modem);
    modem.log.call_id = 0;
    modem.log.level = SM_LOG_FLOW;

    for (i = t0; i < t1; i += 160)
    {
        int len = (int) ((t1 - i < 160) ? (t1 - i) : 160);
        v22bis_rx(&modem, pcm + i, len);
    }
    printf("final: rx=%s tx=%d neg=%d\n", rx_names[modem.rx.training],
           modem.tx.training, modem.negotiated_bit_rate);
    free(buf);
    free(pcm);
    return 0;
}
