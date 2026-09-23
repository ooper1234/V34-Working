/* Replay a raw s16le capture (.in.pcm) through the V.22bis answerer
   receiver offline, reproducing the live call structure:

     - no RX during the V.8 / answer-tone phases (samples just advance)
     - rx_guard (500 ms) at handshake start: RX sees silence
     - then real audio, 160-sample chunks

   usage: replay_pcm file.in.pcm [handshake_start_s] [guard_ms]
   defaults: handshake_start=5.33s, guard=500ms
   Env SM_RX_TRACE=1 enables the same TRACE lines as live. */
#include "modem/v22bis/v22bis.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int dummy_bit(void *ud) { (void) ud; return 1; }
static void dummy_put(void *ud, int bit) { (void) ud; (void) bit; }

static const char *rx_names[] = {
    "NORMAL", "SYM_ACQ", "UNSCR_ON", "UNSCR_ON_SUST", "SCR_ON_1200", "SCR_ON_1200_SUST",
    "WAIT_SCR_2400", "PARKED"
};

static long g_pos = 0;   /* content sample position currently fed */

static void status_cb(void *ud, v22bis_status_t st)
{
    v22bis_state_t *m = ud;
    printf("[status] %d  rx_state=%s tx_state=%d neg=%d  content_t=%.3fs\n", st,
           rx_names[m->rx.training], m->tx.training, m->negotiated_bit_rate,
           (double) g_pos / 8000.0);
}

int main(int argc, char **argv)
{
    FILE *f;
    int16_t *pcm;
    long n, i;
    v22bis_state_t modem;
    double hs = (argc > 2) ? atof(argv[2]) : 5.33;
    int guard_ms = (argc > 3) ? atoi(argv[3]) : 500;
    double phase_turns = (argc > 4) ? atof(argv[4]) : 0.0;  /* initial carrier phase, in turns */
    long guard, start;

    if (argc < 2)
    {
        fprintf(stderr, "usage: %s file.in.pcm [handshake_start_s] [guard_ms]\n", argv[0]);
        return 2;
    }
    f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    n = ftell(f) / (long) sizeof(int16_t);
    fseek(f, 0, SEEK_SET);
    pcm = malloc((size_t) n * sizeof(int16_t));
    n = (long) fread(pcm, sizeof(int16_t), (size_t) n, f);
    fclose(f);

    v22bis_init(&modem, false, 2400, dummy_bit, NULL, dummy_put, NULL, status_cb, &modem);
    modem.log.call_id = 0;
    modem.log.level = SM_LOG_FLOW;
    modem.rx.carrier_phase = (uint32_t) (phase_turns * 4294967296.0);

    start = (long) (hs * 8000.0);
    guard = 8000L * guard_ms / 1000L;
    if (start >= n) { fprintf(stderr, "handshake start beyond file\n"); return 1; }

    /* guard: RX fed silence */
    {
        static int16_t zeros[160];
        long g = guard;
        while (g > 0)
        {
            int len = (int) (g < 160 ? g : 160);
            g_pos = start + (guard - g);
            v22bis_rx(&modem, zeros, len);
            g -= len;
        }
    }
    /* real audio: during the guard those file samples were discarded live */
    for (i = start + guard; i < n; i += 160)
    {
        int len = (int) ((n - i < 160) ? (n - i) : 160);
        g_pos = i;
        v22bis_rx(&modem, pcm + i, len);
    }
    printf("final: rx=%s tx=%d neg=%d\n", rx_names[modem.rx.training],
           modem.tx.training, modem.negotiated_bit_rate);
    free(pcm);
    return 0;
}
