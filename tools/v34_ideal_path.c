/* Deterministic V.34 ideal-path test.

   Generates a known symbol sequence with the transmit mapping engine,
   synthesizes the exact data-mode transmit waveform (same polyphase RRC and
   carrier as tx_v34_modulation), feeds it into the real receiver front end
   (v34_rx in primary-channel mode) and fits the received symbols against the
   transmitted ones. Reports the best delay, complex gain and residual EVM.

   This localises whether symbol corruption happens in the transmit waveform
   or in the receive filtering/timing, independently of the handshake.

   Usage: v34_ideal_path [baud rate] [bit rate] [frames]                      */
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

#include "v34_tx_2400_rrc.h"

SPAN_DECLARE(int) v34_get_mapping_frame(v34_tx_state_t *s, int16_t bits[16]);

#define MAX_SYMS 20000

static uint32_t prbs_state = 0x1234;
static long tx_bit_count;
static long mismatches;
static long rx_bits;

static int prbs_next_bit(void)
{
    uint32_t bit = (prbs_state ^ (prbs_state >> 1)) & 1;
    prbs_state = (prbs_state << 1) | bit;
    return (int) (prbs_state & 1);
}

static int use_ones_source;

static int tx_get_bit(void *user_data)
{
    (void) user_data;
    tx_bit_count++;
    /* V34_IDEAL_ONES=1 makes the transmitter send constant 1 data bits,
       matching the loopback harness, so waveforms can be compared. */
    return use_ones_source ? 1 : prbs_next_bit();
}

static void rx_put_bit(void *user_data, int bit)
{
    (void) user_data;
    if (bit < 0)
        return;
    if (bit != (use_ones_source ? 1 : prbs_next_bit()))
        mismatches++;
    rx_bits++;
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
    int frames = 2000;
    int i, j, n;
    v34_state_t *tx;
    v34_state_t *rx;
    static int16_t tx_syms[2*MAX_SYMS];
    static int16_t rx_syms[2*MAX_SYMS];
    long n_tx = 0;
    long n_rx = 0;
    static int16_t bits[16];
    static int16_t wave[2000000];
    long n_wave = 0;
    float rrc_re[V34_TX_FILTER_STEPS];
    float rrc_im[V34_TX_FILTER_STEPS];
    int rrc_step = 0;
    int baud_phase = 0;
    int num, den;
    uint32_t carrier_phase = 0;
    float gain;
    long last_symbol_count;
    int best_d;
    double best_err = 1e30;
    double k_re = 0.0, k_im = 0.0;

    {
        const char *o = getenv("V34_IDEAL_ONES");
        use_ones_source = o ? atoi(o) : 0;
    }
    if (argc > 1  &&  (argv[1][0] < '0'  ||  argv[1][0] > '9'))
    {
        /* Capture mode: decode a recorded waveform with a fresh receiver.
           The loopback transmitter sends constant 1 data bits, so a correct
           decode yields all ones. */
        FILE *f = fopen(argv[1], "rb");
        double t0 = (argc > 2) ? atof(argv[2]) : 0.0;
        double secs = (argc > 3) ? atof(argv[3]) : 10.0;
        long n, i;
        static uint8_t ubuf[4000000];
        static int16_t pcm[4000000];
        long start;
        long ones = 0;

        if (!f)
        {
            perror(argv[1]);
            return 1;
        }
        fseek(f, 0, SEEK_END);
        n = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (n > (long) sizeof(ubuf))
            n = (long) sizeof(ubuf);
        n = (long) fread(ubuf, 1, (size_t) n, f);
        fclose(f);
        for (i = 0; i < n; i++)
        {
            unsigned u = (uint8_t) ~ubuf[i];
            int t = (int) (((u & 0x0F) << 3) + 0x84);
            t <<= (u & 0x70) >> 4;
            pcm[i] = (int16_t) ((u & 0x80) ? (0x84 - t) : (t - 0x84));
        }
        rx = v34_init(NULL, baud, bps, true, true, NULL, NULL, rx_put_bit, NULL);
        v34_restart(rx, baud, bps, true);
        rx->rx.current_demodulator = V34_MODULATION_V34;
        rx->rx.stage = V34_RX_STAGE_PRIMARY_CHANNEL;
        rx->rx.data_rx_active = true;
        rx->rx.data_rx_count = 0;
        rx->rx.data_rx_symbol_count = 0;
        rx->rx.data_rx_offset = 0;
        rx->rx.data_rx_scale = 1.0f;
        rx->rx.data_rx_rot_re = 1.0f;
        rx->rx.data_rx_rot_im = 0.0f;
        start = (long) (t0*8000.0);
        {
            int ph;
            int tph;
            double best_ones = -1.0;
            long best_bits = 0;
            long best_bad = 0;
            int best_ph = 0;
            int best_tph = 0;

            for (tph = 0; tph < 6; tph++)
            for (ph = 0; ph < 4; ph++)
            {
                /* The transmitter's carrier phase at this capture point is
                   arbitrary; a fresh receiver starts at 0. Sweep the initial
                   phase so a rotation mismatch does not mask the test. */
                rx_bits = 0;
                mismatches = 0;
                v34_restart(rx, baud, bps, true);
                rx->rx.current_demodulator = V34_MODULATION_V34;
                rx->rx.stage = V34_RX_STAGE_PRIMARY_CHANNEL;
                rx->rx.data_rx_active = true;
                rx->rx.data_rx_count = 0;
                rx->rx.data_rx_symbol_count = 0;
                rx->rx.data_rx_offset = 0;
                rx->rx.data_rx_scale = 1.0f;
                rx->rx.data_rx_rot_re = 1.0f;
                rx->rx.data_rx_rot_im = 0.0f;
                rx->rx.carrier_phase = (uint32_t) ((double) ph/4.0*4294967296.0);
                rx->rx.eq_put_step = tph*32;
                for (i = start; i < n  &&  (double) (i - start)/8000.0 < secs; i += 8)
                    v34_rx(rx, &pcm[i], (int) ((n - i < 8) ? (n - i) : 8));
                {
                    double ones = rx_bits > 0 ? (double) (rx_bits - mismatches)/rx_bits : 0.0;
                    if (ones > best_ones)
                    {
                        best_ones = ones;
                        best_bits = rx_bits;
                        best_bad = mismatches;
                        best_ph = ph;
                        best_tph = tph;
                    }
                }
            }
            printf("capture decode: best ones %.3f (carrier phase %d/4, timing phase %d/6, bits %ld, bad %ld)\n",
                   best_ones, best_ph, best_tph, best_bits, best_bad);
        }
        v34_free(rx);
        return 0;
    }
    if (argc > 1)
        baud = atoi(argv[1]);
    if (argc > 2)
        bps = atoi(argv[2]);
    if (argc > 3)
        frames = atoi(argv[3]);
    if (frames*8 > MAX_SYMS)
        frames = MAX_SYMS/8;

    tx = v34_init(NULL, baud, bps, false, true, tx_get_bit, NULL, NULL, NULL);
    rx = v34_init(NULL, baud, bps, true, true, NULL, NULL, rx_put_bit, NULL);
    if (!tx || !rx)
    {
        fprintf(stderr, "v34_init failed\n");
        return 1;
    }
    v34_set_get_aux_bit(tx, get_aux_bit, NULL);
    v34_set_put_aux_bit(rx, put_aux_bit, NULL);
    v34_restart(tx, baud, bps, true);
    v34_restart(rx, baud, bps, true);

    /* Known transmitted symbols: eight 2D symbols per mapping frame, in Q9.7. */
    for (i = 0; i < frames; i++)
    {
        v34_get_mapping_frame(&tx->tx, bits);
        for (j = 0; j < 16; j++)
            tx_syms[n_tx++] = bits[j];
    }

    /* Synthesize the data-mode waveform with the same polyphase RRC, carrier
       and gain as tx_v34_modulation (circular filter read, as fixed). */
    num = tx->tx.parms.samples_per_symbol_numerator;
    den = tx->tx.parms.samples_per_symbol_denominator;
    gain = tx->tx.gain;
    memset(rrc_re, 0, sizeof(rrc_re));
    memset(rrc_im, 0, sizeof(rrc_im));
    {
        long sym_i = 0;

        while (sym_i < frames*8)
        {
            float xr = 0.0f, xi = 0.0f;
            int k;
            double zr, zi;

            if ((baud_phase += den) >= num)
            {
                baud_phase -= num;
                rrc_re[rrc_step] = tx_syms[2*sym_i]/128.0f;
                rrc_im[rrc_step] = tx_syms[2*sym_i + 1]/128.0f;
                sym_i++;
                if (++rrc_step >= V34_TX_FILTER_STEPS)
                    rrc_step = 0;
            }
            k = rrc_step;
            for (j = 0; j < V34_TX_FILTER_STEPS; j++)
            {
                xr += tx_pulseshaper_2400[num - 1 - baud_phase][j]*rrc_re[k];
                xi += tx_pulseshaper_2400[num - 1 - baud_phase][j]*rrc_im[k];
                if (++k >= V34_TX_FILTER_STEPS)
                    k = 0;
            }
            zr = cos(2.0*M_PI*(double) carrier_phase/4294967296.0);
            zi = sin(2.0*M_PI*(double) carrier_phase/4294967296.0);
            if (n_wave < (long) (sizeof(wave)/sizeof(wave[0])))
                wave[n_wave] = (int16_t) ((xr*zr - xi*zi)*gain);
            n_wave++;
            carrier_phase += (uint32_t) tx->tx.v34_carrier_phase_rate;
        }
    }

    /* Also write the synthesized waveform as G.711 mu-law, so it can be run
       through exactly the same capture-decode path as a recorded waveform. */
    {
        FILE *cf = fopen("v34_ideal_tx.ulaw", "wb");
        if (cf)
        {
            for (i = 0; i < n_wave; i++)
            {
                int pcm = wave[i];
                int mask;
                int seg;
                uint8_t u;
                int sign = (pcm >> 8) & 0x80;
                if (sign)
                    pcm = -pcm;
                if (pcm > 32635)
                    pcm = 32635;
                pcm += 0x84;
                seg = 7;
                for (mask = 0x4000; (pcm & mask) == 0 && seg > 0; seg--, mask >>= 1)
                    ;
                u = (uint8_t) ~(sign | (seg << 4) | ((pcm >> (seg + 3)) & 0x0F));
                fputc(u, cf);
            }
            fclose(cf);
        }
    }

    /* Put the receiver into primary-channel data reception. */
    rx->rx.current_demodulator = V34_MODULATION_V34;
    rx->rx.stage = V34_RX_STAGE_PRIMARY_CHANNEL;
    rx->rx.data_rx_active = true;
    rx->rx.data_rx_count = 0;
    rx->rx.data_rx_symbol_count = 0;
    rx->rx.data_rx_offset = 0;
    rx->rx.data_rx_scale = 1.0f;
    rx->rx.data_rx_rot_re = 1.0f;
    rx->rx.data_rx_rot_im = 0.0f;
    last_symbol_count = 0;

    /* Feed in small chunks so no collected frame is missed. */
    for (i = 0; i < n_wave; i += 8)
    {
        n = (n_wave - i < 8) ? (int) (n_wave - i) : 8;
        v34_rx(rx, &wave[i], n);
        if (rx->rx.data_rx_count == 0
            && rx->rx.data_rx_symbol_count != last_symbol_count
            && rx->rx.data_rx_symbol_count > 0)
        {
            last_symbol_count = rx->rx.data_rx_symbol_count;
            if (n_rx + 16 <= 2*MAX_SYMS)
            {
                for (j = 0; j < 16; j++)
                    rx_syms[n_rx++] = rx->rx.data_rx_symbols[j];
            }
        }
    }

    printf("tx symbols %ld, rx symbols %ld, wave samples %ld\n", n_tx/2, n_rx/2, n_wave);
    printf("demapped bits %ld, mismatches %ld (BER %.4f)\n",
           rx_bits, mismatches, rx_bits > 0 ? (double) mismatches/rx_bits : 0.0);

    /* Fit rx against tx: complex gain and delay that minimise the error. */
    for (best_d = 0; best_d < 64; best_d++)
    {
        double num_re = 0.0, num_im = 0.0, den2 = 0.0, err = 0.0;
        long cnt = 0;

        for (i = 0; i < (n_tx/2) - best_d && i < (n_rx/2); i++)
        {
            double rxr = rx_syms[2*i];
            double rxi = rx_syms[2*i + 1];
            double txr = tx_syms[2*(i + best_d)];
            double txi = tx_syms[2*(i + best_d) + 1];

            num_re += txr*rxr + txi*rxi;
            num_im += txi*rxr - txr*rxi;
            den2 += rxr*rxr + rxi*rxi;
            cnt++;
        }
        if (den2 > 1.0)
        {
            k_re = num_re/den2;
            k_im = num_im/den2;
            for (i = 0; i < (n_tx/2) - best_d && i < (n_rx/2); i++)
            {
                double rxr = rx_syms[2*i];
                double rxi = rx_syms[2*i + 1];
                double txr = tx_syms[2*(i + best_d)];
                double txi = tx_syms[2*(i + best_d) + 1];
                double er = txr - (k_re*rxr - k_im*rxi);
                double ei = txi - (k_re*rxi + k_im*rxr);
                err += er*er + ei*ei;
            }
            err = sqrt(err/cnt);
            if (err < best_err)
            {
                best_err = err;
                printf("delay %2d: |k|=%.4f angle=%.1f deg residual-RMS=%.1f  (fit over %ld syms)\n",
                       best_d, hypot(k_re, k_im), atan2(k_im, k_re)*180.0/M_PI, err, cnt);
            }
        }
    }
    printf("best: delay %d, |k| %.4f, angle %.1f deg, residual RMS %.1f\n",
           best_d, hypot(k_re, k_im), atan2(k_im, k_re)*180.0/M_PI, best_err);
    printf("(transmitted symbol units are Q9.7: 128 = one constellation unit)\n");

    v34_free(tx);
    v34_free(rx);
    return mismatches == 0 ? 0 : 1;
}
