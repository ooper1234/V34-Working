/* Minimal test: drive a v34_rx with the audio output of a v34_tx in
   loopback and compare the descrambled CC bits.  This isolates the
   analog-layer decode from the V.8 handshake. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "spandsp/v34.h"
#include "spandsp/v8.h"

#define RATE 9600
#define CHUNK 320

static v34_state_t caller_v34, answerer_v34;
static v8_state_t caller_v8, answerer_v8;
static int caller_phase, answerer_phase;

/* Captured TX audio and rx'd raw_dibits from the CC phase */
static int16_t cap_caller[400000], cap_answerer[400000];
static int cap_w;

static int rx_raw_dibs[8000];
static int rx_desc[8000];
static int rx_count;

static void put_bit(void *ud, int bit)
{
    (void) ud;
    /* not used for this test */
}

static int get_bit(void *ud)
{
    (void) ud;
    return 1;
}

static void put_aux_bit(void *ud, int bit)
{
    (void) ud;
    (void) bit;
}

static int get_aux_bit(void *ud)
{
    (void) ud;
    return 1;
}

int main(void)
{
    int sample = 0;
    int i;

    v34_rx_init(&caller_v34.rx, &caller_v34, 0,
                put_bit, NULL, get_bit, NULL,
                put_aux_bit, NULL, get_aux_bit, NULL);
    v34_tx_init(&caller_v34.tx, &caller_v34, 0);

    v34_rx_init(&answerer_v34.rx, &answerer_v34, 1,
                put_bit, NULL, get_bit, NULL,
                put_aux_bit, NULL, get_aux_bit, NULL);
    v34_tx_init(&answerer_v34.tx, &answerer_v34, 1);

    v8_init(&caller_v8, 0);
    v8_init(&answerer_v8, 1);

    caller_phase = 0;
    answerer_phase = 0;

    int last_stage_caller = -1, last_stage_answerer = -1;
    int saved_count = -1;
    int got_both_mp = 0;

    while (sample < RATE * 30)
    {
        int16_t caller_tx[CHUNK], answerer_tx[CHUNK];
        int n;

        /* Caller TX */
        n = 0;
        if (caller_phase == 0)
        {
            n = v8_tx(&caller_v8, caller_tx, CHUNK);
            if (n < CHUNK) caller_phase = 1;
        }
        if (n < CHUNK)
        {
            int got = v34_tx(&caller_v34, caller_tx + n, CHUNK - n);
            if (got < 0) got = 0;
            n += got;
        }
        if (n < CHUNK)
            memset(caller_tx + n, 0, (CHUNK - n) * sizeof(int16_t));

        /* Answerer TX */
        n = 0;
        if (answerer_phase == 0)
        {
            n = v8_tx(&answerer_v8, answerer_tx, CHUNK);
            if (n < CHUNK) answerer_phase = 1;
        }
        if (n < CHUNK)
        {
            int got = v34_tx(&answerer_v34, answerer_tx + n, CHUNK - n);
            if (got < 0) got = 0;
            n += got;
        }
        if (n < CHUNK)
            memset(answerer_tx + n, 0, (CHUNK - n) * sizeof(int16_t));

        /* Capture */
        memcpy(cap_caller + sample, caller_tx, CHUNK * sizeof(int16_t));
        memcpy(cap_answerer + sample, answerer_tx, CHUNK * sizeof(int16_t));

        /* Cross-connect */
        if (answerer_phase == 0)
            v8_rx(&answerer_v8, caller_tx, CHUNK);
        else
            v34_rx(&answerer_v34, caller_tx, CHUNK);
        if (caller_phase == 0)
            v8_rx(&caller_v8, answerer_tx, CHUNK);
        else
            v34_rx(&caller_v34, answerer_tx, CHUNK);

        sample += CHUNK;

        /* Report stages */
        if (getenv("V34_TRACE_MP"))
        {
            if (caller_v34.tx.stage != last_stage_caller ||
                answerer_v34.tx.stage != last_stage_answerer)
            {
                fprintf(stderr, "t=%.3f caller tx=%d rx=%d | answerer tx=%d rx=%d\n",
                        (double)sample / RATE,
                        caller_v34.tx.stage, caller_v34.rx.stage,
                        answerer_v34.tx.stage, answerer_v34.rx.stage);
                last_stage_caller = caller_v34.tx.stage;
                last_stage_answerer = answerer_v34.tx.stage;
            }
        }

        /* Enable CC trace */
        if (!getenv("V34_TRACE_MP"))
            setenv("V34_TRACE_MP", "1", 0);

        /* Check if both are in MP stage */
        if (caller_v34.tx.stage >= 10 && answerer_v34.tx.stage >= 10 && !got_both_mp)
        {
            fprintf(stderr, "Both in MP stage at t=%.3f\n", (double)sample / RATE);
            got_both_mp = 1;
        }
    }

    fprintf(stderr, "Done. caller RX cc_baud_count=%ld, answerer RX cc_baud_count=%ld\n",
            caller_v34.rx.cc_baud_count, answerer_v34.rx.cc_baud_count);
    return 0;
}
