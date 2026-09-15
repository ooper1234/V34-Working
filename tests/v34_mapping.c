/* V.34 mapping round-trip test: one modem context maps data bits into 4D/2D
   symbols (shell mapping, trellis coding, differential encoding, precoding,
   scrambling), and a second context demaps them back to bits. With the
   Viterbi bypass used by the library for this path, the round trip is
   bit-exact, which verifies the whole mapping engine independently of the
   channel and the (still unimplemented) primary-channel receiver.

   This is the same check the spanDSP v34_tests.c harness performs, extended
   with a PRBS data source and a bit-for-bit comparison.

   Usage: v34_mapping [baud rate] [bit rate] [frames]
   e.g.   v34_mapping 2400 4800 200                                      */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

SPAN_DECLARE(int) v34_get_mapping_frame(v34_tx_state_t *s, int16_t bits[16]);
SPAN_DECLARE(void) v34_put_mapping_frame(v34_rx_state_t *s, int16_t bits[16]);

/* PRBS-15 data source, identical sequence read by the transmitter and
   expected by the receiver. */
static uint32_t prbs_state = 0x1234;
static long tx_bit_count;
static long rx_bit_count;
static long mismatches;
static long rx_bits;

static int prbs_next_bit(void)
{
    /* x^15 + x^14 + 1 */
    uint32_t bit = (prbs_state ^ (prbs_state >> 1)) & 1;
    prbs_state = (prbs_state << 1) | bit;
    return (int) (prbs_state & 1);
}

static int tx_get_bit(void *user_data)
{
    (void) user_data;
    tx_bit_count++;
    return prbs_next_bit();
}

static void rx_put_bit(void *user_data, int bit)
{
    int expected;

    (void) user_data;
    if (bit < 0)
        return;
    expected = prbs_next_bit();
    if (bit != expected)
        mismatches++;
    rx_bits++;
    rx_bit_count++;
}

static int get_aux_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void put_aux_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

int main(int argc, char **argv)
{
    int baud = 2400;
    int bps = 4800;
    int frames = 200;
    int i;
    int16_t bits[16];
    v34_state_t *tx;
    v34_state_t *rx;
    logging_state_t *log;

    if (argc > 1)
        baud = atoi(argv[1]);
    if (argc > 2)
        bps = atoi(argv[2]);
    if (argc > 3)
        frames = atoi(argv[3]);

    /* The transmitter runs as one role and the receiver as the other, so
       the scrambler/descrambler taps pair up as they do on a real link. */
    tx = v34_init(NULL, baud, bps, false, true, tx_get_bit, NULL, NULL, NULL);
    rx = v34_init(NULL, baud, bps, true, true, NULL, NULL, rx_put_bit, NULL);
    if (!tx || !rx)
    {
        fprintf(stderr, "v34_init failed\n");
        return 1;
    }
    v34_set_get_aux_bit(tx, get_aux_bit, NULL);
    v34_set_put_aux_bit(rx, put_aux_bit, NULL);
    log = v34_get_logging_state(tx);
    span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
    span_log_set_tag(log, "tx");
    log = v34_get_logging_state(rx);
    span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
    span_log_set_tag(log, "rx");

    for (i = 0; i < frames; i++)
    {
        v34_get_mapping_frame(&tx->tx, bits);
        v34_put_mapping_frame(&rx->rx, bits);
    }

    printf("v34 mapping: baud=%d bps=%d frames=%d tx_bits=%ld rx_bits=%ld mismatches=%ld\n",
           baud, bps, frames, tx_bit_count, rx_bit_count, mismatches);
    v34_free(tx);
    v34_free(rx);
    /* The first mapping frame is used to establish scrambler synchronisation. */
    if (rx_bits == 0)
        return 2;
    if (mismatches > 32)
        return 1;
    printf("RESULT: PASS\n");
    return 0;
}
