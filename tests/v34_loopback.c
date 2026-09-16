/* V.34 loopback harness: a calling and an answering modem connected directly,
   starting with V.8 negotiation then the spanDSP V.34 engine. Prints every
   V.8/V.34 state transition and a data bit error count.

   spanDSP's V.34 is work in progress: this test currently proves the V.8
   startup and V.34 phases 1-2 (INFO0, A/B tones, INFO1a/INFO1c exchange and
   L1/L2 line probing). It then stalls at phase 3 because the S/!S signal
   detection is not implemented in the receiver (V34_EVENT_S is never
   generated). See docs/v34.md for the full status.

   Usage: v34_loopback [baud rate] [bit rate] [seconds]
   e.g.   v34_loopback 2400 4800 60                                        */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define SPANDSP_EXPOSE_INTERNAL_STRUCTURES 1
#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/complex.h"
#include "spandsp/async.h"
#include "spandsp/dds.h"
#include "spandsp/power_meter.h"
#include "spandsp/fsk.h"
#include "spandsp/queue.h"
#include "spandsp/tone_generate.h"
#include "spandsp/super_tone_rx.h"
#include "spandsp/modem_connect_tones.h"
#include "spandsp/v8.h"
#include "spandsp/v29rx.h"
#include "spandsp/v34.h"
#include "spandsp/bitstream.h"
#include "spandsp/modem_echo.h"
#include "spandsp/private/bitstream.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/v34.h"

#define CHUNK 160
#define RATE 8000


static FILE *cap_caller;
static FILE *cap_answerer;

/* Inline G.711 mu-law encoder, so captures can be analysed with tools/ */
static uint8_t lin2ulaw(int16_t pcm)
{
    int sign;
    int exponent;
    int mantissa;
    int mask;

    sign = (pcm >> 8) & 0x80;
    if (sign)
        pcm = -pcm;
    if (pcm > 32635)
        pcm = 32635;
    pcm += 0x84;
    exponent = 7;
    for (mask = 0x4000;  (pcm & mask) == 0  &&  exponent > 0;  exponent--, mask >>= 1)
        ;
    mantissa = (pcm >> (exponent + 3)) & 0x0F;
    return (uint8_t) ~(sign | (exponent << 4) | mantissa);
}

static void capture_block(FILE *f, const int16_t *amp, int len)
{
    uint8_t buf[CHUNK];
    int i;

    if (!f)
        return;
    for (i = 0; i < len; i++)
        buf[i] = lin2ulaw(amp[i]);
    fwrite(buf, 1, (size_t) len, f);
}

static int g_baud = 2400;
static int g_bps = 4800;
static int g_seconds = 40;

static v34_state_t *v34_caller;
static v34_state_t *v34_answerer;
static int caller_phase;      /* 0 = V.8 TX, 1 = V.34 TX */
static int answerer_phase;

static uint8_t tx_buf[400000];
static int tx_ptr;
static int rx_ptr;
static int rx_bits;
static int rx_bad;

/* Per-direction PRBS data: the transmitter pulls bits from the sequence and
   the receiver compares against the same sequence. Constant data is
   pathological for the V.34 scrambler (all-ones is its fixed point), so a
   pseudo-random source is required for a meaningful data test. */
typedef struct
{
    uint32_t state;
    uint8_t buf[400000];
    int wr;
    int rd;
    long bits;
} data_dir_t;

static data_dir_t dir_a;   /* caller transmits, answerer receives */
static data_dir_t dir_b;   /* answerer transmits, caller receives */

static int dir_next_bit(data_dir_t *d)
{
    uint32_t bit = (d->state ^ (d->state >> 1)) & 1;
    d->state = (d->state << 1) | bit;
    return (int) (d->state & 1);
}

static int get_bit_a(void *user_data)
{
    data_dir_t *d = user_data;
    int bit = dir_next_bit(d);
    if (d->wr < (int) sizeof(d->buf))
        d->buf[d->wr++] = (uint8_t) bit;
    d->bits++;
    return bit;
}

static void put_bit_a(void *user_data, int bit)
{
    data_dir_t *d = user_data;
    if (bit < 0)
    {
        printf("  [v34] signal status %d\n", bit);
        return;
    }
    if (d->rd < d->wr && bit != d->buf[d->rd])
        rx_bad++;
    d->rd++;
    rx_bits++;
}

static data_dir_t *g_dir;   /* not used; kept for clarity */

static int get_aux_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void put_aux_bit(void *user_data, int bit)
{
    (void) user_data;
    printf("  [aux] rx bit %d\n", bit);
}


static const char *tx_stage_name(int st)
{
    switch (st)
    {
    case V34_TX_STAGE_INITIAL_PREAMBLE: return "PREAMBLE";
    case V34_TX_STAGE_INFO0: return "INFO0";
    case V34_TX_STAGE_INITIAL_A: return "INIT_A";
    case V34_TX_STAGE_FIRST_A: return "FIRST_A";
    case V34_TX_STAGE_FIRST_NOT_A: return "FIRST_NOT_A";
    case V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN: return "NOT_A_REV";
    case V34_TX_STAGE_SECOND_A: return "SECOND_A";
    case V34_TX_STAGE_L1: return "L1";
    case V34_TX_STAGE_L2: return "L2";
    case V34_TX_STAGE_POST_L2_A: return "POST_L2_A";
    case V34_TX_STAGE_POST_L2_NOT_A: return "POST_L2_NOT_A";
    case V34_TX_STAGE_A_SILENCE: return "A_SILENCE";
    case V34_TX_STAGE_PRE_INFO1_A: return "PRE_INFO1_A";
    case V34_TX_STAGE_INFO1: return "INFO1";
    case V34_TX_STAGE_FIRST_B: return "FIRST_B";
    case V34_TX_STAGE_FIRST_B_INFO_SEEN: return "B_INFO_SEEN";
    case V34_TX_STAGE_FIRST_NOT_B_WAIT: return "NOT_B_WAIT";
    case V34_TX_STAGE_FIRST_NOT_B: return "FIRST_NOT_B";
    case V34_TX_STAGE_FIRST_B_SILENCE: return "B_SILENCE";
    case V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE: return "B_POST_REV_SIL";
    case V34_TX_STAGE_SECOND_B: return "SECOND_B";
    case V34_TX_STAGE_SECOND_B_WAIT: return "SECOND_B_WAIT";
    case V34_TX_STAGE_SECOND_NOT_B: return "SECOND_NOT_B";
    case V34_TX_STAGE_INFO0_RETRY: return "INFO0_RETRY";
    case V34_TX_STAGE_FIRST_S: return "FIRST_S";
    case V34_TX_STAGE_FIRST_NOT_S: return "FIRST_NOT_S";
    case V34_TX_STAGE_MD: return "MD";
    case V34_TX_STAGE_SECOND_S: return "SECOND_S";
    case V34_TX_STAGE_SECOND_NOT_S: return "SECOND_NOT_S";
    case V34_TX_STAGE_TRN: return "TRN";
    case V34_TX_STAGE_J: return "J";
    case V34_TX_STAGE_J_DASHED: return "J_DASHED";
    case V34_TX_STAGE_MP: return "MP";
    case V34_TX_STAGE_DATA: return "DATA";
    default: return "?";
    }
}

static const char *rx_stage_name(int st)
{
    switch (st)
    {
    case V34_RX_STAGE_INFO0: return "INFO0";
    case V34_RX_STAGE_INFOH: return "INFOH";
    case V34_RX_STAGE_INFO1C: return "INFO1C";
    case V34_RX_STAGE_INFO1A: return "INFO1A";
    case V34_RX_STAGE_TONE_A: return "TONE_A";
    case V34_RX_STAGE_TONE_B: return "TONE_B";
    case V34_RX_STAGE_L1_L2: return "L1_L2";
    case V34_RX_STAGE_CC: return "CC";
    case V34_RX_STAGE_PRIMARY_CHANNEL: return "PRIMARY";
    default: return "?";
    }
}

static int last_cts = -1, last_crs = -1, last_ats = -1, last_ars = -1;
static long g_sample;
static int last_tx_frame;
static int last_rx_sym;
static int phase_aligned_caller;
static int phase_aligned_answerer;

static void report_stages(v34_state_t *caller, v34_state_t *answerer)
{
    if (caller->tx.stage != last_cts || caller->rx.stage != last_crs
        || answerer->tx.stage != last_ats || answerer->rx.stage != last_ars)
    {
        last_cts = caller->tx.stage;
        last_crs = caller->rx.stage;
        last_ats = answerer->tx.stage;
        last_ars = answerer->rx.stage;
        printf("t=%.3f STAGES caller tx=%s rx=%s | answerer tx=%s rx=%s\n",
               (double) g_sample / RATE,
               tx_stage_name(last_cts), rx_stage_name(last_crs),
               tx_stage_name(last_ats), rx_stage_name(last_ars));
        fflush(stdout);
    }
}

static void v8_handler(void *user_data, v8_parms_t *result)
{
    const char *who = (const char *) user_data;

    printf("[V8 %s] status=%d mods=0x%x cf=%d proto=%d\n", who, result->status,
           (unsigned) result->jm_cm.modulations, (int) result->jm_cm.call_function,
           (int) result->jm_cm.protocols);
    fflush(stdout);
    if (result->status == V8_STATUS_V8_CALL)
    {
        if (strcmp(who, "caller") == 0)
            v34_restart(v34_caller, g_baud, g_bps, true);
        else
            v34_restart(v34_answerer, g_baud, g_bps, true);
    }
}

int main(int argc, char **argv)
{
    v8_state_t *v8_caller;
    v8_state_t *v8_answerer;
    v8_parms_t parms;
    int16_t caller_tx[CHUNK];
    int16_t answerer_tx[CHUNK];
    long sample;
    long total = (long) g_seconds * RATE;
    logging_state_t *log;

    if (argc > 1)
        g_baud = atoi(argv[1]);
    if (argc > 2)
        g_bps = atoi(argv[2]);
    if (argc > 3)
        g_seconds = atoi(argv[3]);

    cap_caller = fopen("v34_caller_tx.ulaw", "wb");
    cap_answerer = fopen("v34_answerer_tx.ulaw", "wb");
    dir_a.state = 0x1234;
    dir_b.state = 0x5678;
    (void) g_dir;
    v34_caller = v34_init(NULL, g_baud, g_bps, true, true, get_bit_a, &dir_a, put_bit_a, &dir_b);
    v34_answerer = v34_init(NULL, g_baud, g_bps, false, true, get_bit_a, &dir_b, put_bit_a, &dir_a);
    if (!v34_caller || !v34_answerer)
    {
        fprintf(stderr, "v34_init failed\n");
        return 1;
    }
    v34_set_get_aux_bit(v34_caller, get_aux_bit, NULL);
    v34_set_put_aux_bit(v34_caller, put_aux_bit, NULL);
    v34_set_get_aux_bit(v34_answerer, get_aux_bit, NULL);
    v34_set_put_aux_bit(v34_answerer, put_aux_bit, NULL);
    v34_tx_power(v34_caller, -13.0f);
    v34_tx_power(v34_answerer, -13.0f);
    log = v34_get_logging_state(v34_caller);
    span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
    span_log_set_tag(log, "caller  ");
    log = v34_get_logging_state(v34_answerer);
    span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
    span_log_set_tag(log, "answerer");

    memset(&parms, 0, sizeof(parms));
    parms.modem_connect_tone = MODEM_CONNECT_TONES_ANSAM;
    parms.gateway_mode = false;
    parms.send_ci = true;
    parms.v92 = -1;
    parms.jm_cm.call_function = V8_CALL_V_SERIES;
    parms.jm_cm.modulations = V8_MOD_V34;
    parms.jm_cm.protocols = V8_PROTOCOL_LAPM_V42;
    parms.jm_cm.pstn_access = 0;
    parms.jm_cm.nsf = -1;
    parms.jm_cm.t66 = -1;
    v8_answerer = v8_init(NULL, false, &parms, v8_handler, (void *) "answerer");
    parms.modem_connect_tone = MODEM_CONNECT_TONES_NONE;
    v8_caller = v8_init(NULL, true, &parms, v8_handler, (void *) "caller");
    if (!v8_caller || !v8_answerer)
    {
        fprintf(stderr, "v8_init failed\n");
        return 1;
    }
    log = v8_get_logging_state(v8_caller);
    span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
    span_log_set_tag(log, "v8-callr");
    log = v8_get_logging_state(v8_answerer);
    span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
    span_log_set_tag(log, "v8-answr");

    for (sample = 0; sample < total; sample += CHUNK)
    {
        int n;

        /* Caller TX */
        n = 0;
        if (caller_phase == 0)
        {
            n = v8_tx(v8_caller, caller_tx, CHUNK);
            if (n < CHUNK)
                caller_phase = 1;
        }
        if (n < CHUNK)
        {
            int got = v34_tx(v34_caller, caller_tx + n, CHUNK - n);
            if (got < 0)
                got = 0;
            n += got;
        }
        if (n < CHUNK)
            memset(&caller_tx[n], 0, (CHUNK - n) * sizeof(int16_t));

        /* Answerer TX */
        n = 0;
        if (answerer_phase == 0)
        {
            n = v8_tx(v8_answerer, answerer_tx, CHUNK);
            if (n < CHUNK)
                answerer_phase = 1;
        }
        if (n < CHUNK)
        {
            int got = v34_tx(v34_answerer, answerer_tx + n, CHUNK - n);
            if (got < 0)
                got = 0;
            n += got;
        }
        if (n < CHUNK)
            memset(&answerer_tx[n], 0, (CHUNK - n) * sizeof(int16_t));

        capture_block(cap_caller, caller_tx, CHUNK);
        capture_block(cap_answerer, answerer_tx, CHUNK);

        /* Cross-connect RX (ideal channel) */
        if (answerer_phase == 0)
            v8_rx(v8_answerer, caller_tx, CHUNK);
        else
            v34_rx(v34_answerer, caller_tx, CHUNK);
        if (caller_phase == 0)
            v8_rx(v8_caller, answerer_tx, CHUNK);
        else
            v34_rx(v34_caller, answerer_tx, CHUNK);

        g_sample = sample;
        report_stages(v34_caller, v34_answerer);

        /* Diagnostic: align the receiver's carrier phase accumulator with the
           transmitter's when data mode begins. A QAM slicer cannot tolerate an
           arbitrary constant phase offset, and the two phase accumulators
           have unrelated histories from the handshake. */
        if (getenv("V34_PHASE_ALIGN"))
        {
            /* One-shot: when a receiver first enters data reception, line its
               carrier phase up with the far transmitter's. Without carrier
               recovery this arbitrary offset makes QAM slicing impossible.
               (Diagnostic for the loopback; a real receiver tracks phase.) */
            if (v34_caller->rx.data_rx_active && !phase_aligned_caller)
            {
                v34_caller->rx.carrier_phase = v34_answerer->tx.carrier_phase;
                phase_aligned_caller = 1;
            }
            if (v34_answerer->rx.data_rx_active && !phase_aligned_answerer)
            {
                v34_answerer->rx.carrier_phase = v34_caller->tx.carrier_phase;
                phase_aligned_answerer = 1;
            }
        }

        /* Diagnostic: record TX frames (answerer -> caller direction) and the
           caller's received symbols so the two can be compared directly. */
        if (v34_answerer->tx.data_frame != last_tx_frame
            && v34_answerer->tx.data_frame > 0)
        {
            int j;

            last_tx_frame = v34_answerer->tx.data_frame;
            if (last_tx_frame >= 200 && last_tx_frame < 260)
            {
                printf("SYMTX %d", last_tx_frame);
                for (j = 0; j < 16; j++)
                    printf(" %d", v34_answerer->tx.data_bits[j]);
                printf("\n");
            }
        }
        if (v34_caller->rx.data_rx_count == 0
            && v34_caller->rx.data_rx_symbol_count != last_rx_sym
            && v34_caller->rx.data_rx_symbol_count > 0)
        {
            int j;

            last_rx_sym = v34_caller->rx.data_rx_symbol_count;
            if (last_rx_sym >= 1600 && last_rx_sym < 2200)
            {
                printf("SYMRX %d", last_rx_sym);
                for (j = 0; j < 8; j++)
                    printf(" %d,%d", v34_caller->rx.data_rx_symbols[2*j],
                           v34_caller->rx.data_rx_symbols[2*j + 1]);
                printf("\n");
            }
        }
        if ((sample % (RATE * 5)) == 0)
        {
            printf("  t=%lds rx_bits=%d bad=%d\n", sample / RATE, rx_bits, rx_bad);
            fflush(stdout);
        }
    }

    printf("final: rx_bits=%d rx_bad=%d\n", rx_bits, rx_bad);
    if (rx_bits > 0)
        printf("NOTE: the primary-channel receiver front end (timing lock, AGC,\n"
               "      level calibration) is not complete; received bits are not\n"
               "      yet valid data (expect ~50%% errors). See docs/v34.md.\n");
    if (cap_caller)
        fclose(cap_caller);
    if (cap_answerer)
        fclose(cap_answerer);
    v34_free(v34_caller);
    v34_free(v34_answerer);
    v8_free(v8_caller);
    v8_free(v8_answerer);
    return rx_bits > 0 ? 0 : 2;
}
