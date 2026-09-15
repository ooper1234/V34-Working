/* V.22bis PTY byte-transport integration test.
 *
 * Proves the full data path: PTY bytes -> async framing (start bit + 8 data
 * bits, LSB first) -> V.22bis TX -> audio channel -> V.22bis RX -> async
 * deframing -> PTY bytes, for arbitrary binary data, in BOTH directions
 * simultaneously (full duplex).
 *
 * Child process runs both modems (caller and answerer, ours) back-to-back.
 * Parent feeds a pseudo-random pattern into slave A, reads it back from
 * slave B, and the mirrored pattern in the reverse direction, verifying
 * byte-exact transport. This is exactly the async-to-sync conversion that
 * V.42 sits above on a real modem.
 */

#include "v22bs_impl.h"
#include "v22bs_channel.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <time.h>
#include <pty.h>
#include <termios.h>
#include <sys/wait.h>

#define MAX_FBIT 65536
#define MAX_RBIT 65536
#define OBUF_MAX 64

typedef struct {
    int pty_fd;                 /* master fd for this side */
    v22bs_t *modem;
    v22bs_channel_t ch;

    /* framed TX bits awaiting modem pull */
    uint8_t fbits[MAX_FBIT];
    int fn;

    /* bits received from modem RX awaiting deframing */
    uint8_t rbits[MAX_RBIT];
    int rn;

    /* async deframer state: 0=idle scan 1=after start 2..9=collecting data */
    int deframe_state;
    int deframe_byte;
    int deframe_nbits;
    int deframe_prev;           /* previous bit for 1->0 edge detection */
    int framing_errors;

    /* bytes ready to write out to the peer PTY */
    uint8_t obuf[OBUF_MAX];
    int on;
    long bits_consumed;
    long bits_idle;
    long bits_rx;
    long bytes_read;
    long bits_deframed;
    long bytes_deframed;
} pty_side_t;

/* Async TX framing: start bit (0) + 8 data bits LSB-first + stop bit (1).
   Idle = continuous 1s. */
static void pty_async_encode(pty_side_t *p, const uint8_t *buf, int n)
{
    int i, b;

    for (i = 0; i < n; i++)
    {
        uint8_t byte = buf[i];
        if (p->fn + 10 > MAX_FBIT)
            break;
        p->fbits[p->fn++] = 0;
        for (b = 0; b < 8; b++)
            p->fbits[p->fn++] = (byte >> b) & 1;
        p->fbits[p->fn++] = 1;  /* stop bit */
    }
}

/* Async deframer with 1->0 edge detection and stop-bit validation. */
static void pty_async_decode(pty_side_t *p, int bit)
{
    switch (p->deframe_state)
    {
    case 0:
        /* Scanning for start bit via 1->0 edge detection.
           During training / settling, the descrambler may output random bits
           including spurious 0s. A start bit is only recognized if the
           PREVIOUS bit was 1 (idle/stop) and current bit is 0. */
        if (p->deframe_prev == 1 && bit == 0)
        {
            p->deframe_state = 1;
            p->deframe_nbits = 0;
            p->deframe_byte = 0;
        }
        p->deframe_prev = bit;
        break;
    default:
        /* Collecting 8 data bits LSB-first */
        p->deframe_byte |= bit << p->deframe_nbits;
        p->deframe_nbits++;
        if (p->deframe_nbits >= 8)
        {
            p->deframe_state = 2;           /* await stop bit */
        }
        break;
    case 2:
        /* Stop bit: must be 1. If not, framing error — resync. */
        p->deframe_prev = bit;
        if (bit == 1)
        {
            if (p->on < OBUF_MAX)
                p->obuf[p->on++] = (uint8_t)p->deframe_byte;
            p->bytes_deframed++;
        }
        else
            p->framing_errors++;
        p->deframe_state = 0;
        break;
    }
}

/* Pacing/source callback: the modem pulls a bit whenever its TX needs one
   (in data mode). Empty queue -> idle '1'. */
static int pty_get_bit(void *ud)
{
    pty_side_t *p = ud;
    int bit = 1;

    if (p->fn > 0)
    {
        bit = p->fbits[0];
        memmove(p->fbits, p->fbits + 1, p->fn - 1);
        p->fn--;
        p->bits_consumed++;
    }
    else
        p->bits_idle++;
    return bit;
}

static void pty_put_bit(void *ud, int bit)
{
    pty_side_t *p = ud;

    p->bits_rx++;
    if (p->rn < MAX_RBIT)
        p->rbits[p->rn++] = bit & 1;
}

int main(int argc, char *argv[])
{
    int rate = 2400;
    int duration_s = 20;
    int pat_len = 2048;
    int master_a, slave_a, master_b, slave_b;
    pid_t pid;
    int status;

    if (argc > 1)
        rate = atoi(argv[1]);
    if (argc > 2)
        duration_s = atoi(argv[2]);
    if (argc > 3)
        pat_len = atoi(argv[3]);
    if (rate != 1200 && rate != 2400)
    {
        fprintf(stderr, "rate must be 1200 or 2400\n");
        return 2;
    }

    printf("V.22bis PTY byte-transport test: rate=%d, %ds, pat=%dB\n",
           rate, duration_s, pat_len);

    if (openpty(&master_a, &slave_a, NULL, NULL, NULL) < 0 ||
        openpty(&master_b, &slave_b, NULL, NULL, NULL) < 0)
    {
        perror("openpty");
        return 1;
    }

    /* Put both PTYs in raw mode: no echo, no line-buffering, no flow control.
       This is required both for the test (prevent echo feedback) and for any
       real pppd integration. */
    {
        struct termios tio;
        cfmakeraw(&tio);
        tcsetattr(slave_a, TCSANOW, &tio);
        tcsetattr(slave_b, TCSANOW, &tio);
    }

    pid = fork();
    if (pid < 0)
    {
        perror("fork");
        return 1;
    }

    if (pid == 0)
    {
        pty_side_t sA, sB;
        int elapsed_frames = 0;
        int total_frames = duration_s * 50;

        close(slave_a);
        close(slave_b);
        fprintf(stderr, "[child] started: total_frames=%d rate=%d\n", total_frames, rate);

        memset(&sA, 0, sizeof(sA));
        memset(&sB, 0, sizeof(sB));
        sA.pty_fd = master_a;
        sB.pty_fd = master_b;

        v22bs_channel_init(&sA.ch, 0x1111);
        v22bs_channel_init(&sB.ch, 0x2222);
        sA.ch.attenuation = 1.0;
        sB.ch.attenuation = 1.0;

        /* caller = side A (TX low channel), answerer = side B (TX high channel) */
        sA.modem = v22bs_ours_create(1, rate, pty_get_bit, &sA,
                                     pty_put_bit, &sA,
                                     NULL, NULL);
        sB.modem = v22bs_ours_create(0, rate, pty_get_bit, &sB,
                                     pty_put_bit, &sB,
                                     NULL, NULL);
        if (!sA.modem || !sB.modem)
        {
            fprintf(stderr, "modem create failed\n");
            _exit(1);
        }

        fcntl(sA.pty_fd, F_SETFL, O_NONBLOCK);
        fcntl(sB.pty_fd, F_SETFL, O_NONBLOCK);

        while (elapsed_frames < total_frames)
        {
            struct pollfd pfds[2];
            int nfound, k;

            pfds[0].fd = sA.pty_fd;  pfds[0].events = POLLIN;  pfds[0].revents = 0;
            pfds[1].fd = sB.pty_fd;  pfds[1].events = POLLIN;  pfds[1].revents = 0;
            nfound = poll(pfds, 2, 1);
            for (k = 0; k < 2 && nfound > 0; k++)
            {
                pty_side_t *p = (k == 0) ? &sA : &sB;
                if ((pfds[k].revents & POLLIN) != 0)
                {
                    uint8_t tmp[64];
                    int nr = read(p->pty_fd, tmp, sizeof(tmp));
                    if (nr > 0)
                    {
                        pty_async_encode(p, tmp, nr);
                        p->bytes_read += nr;
                    }
                }
            }

/* audio frame: caller TX (160 smp) -> ch A -> answerer RX,
               answerer TX (160 smp) -> ch B -> caller RX  */
            {
                int16_t oa[160], ob[160];
                int16_t ba[160], bb[160];
                int na, nb;

                na = sA.modem->tx(sA.modem->cb, oa, 160);
                nb = sB.modem->tx(sB.modem->cb, ob, 160);

                na = v22bs_channel_apply(&sA.ch, oa, na, ba, 160);
                nb = v22bs_channel_apply(&sB.ch, ob, nb, bb, 160);
                if (na > 0)
                    sB.modem->rx(sB.modem->cb, ba, na);
                if (nb > 0)
                    sA.modem->rx(sA.modem->cb, bb, nb);
            }

            /* deframe a bounded number of RX bits per frame per direction */
            for (k = 0; k < 2; k++)
            {
                pty_side_t *p = (k == 0) ? &sA : &sB;
                int c = 0;
                while (p->rn > 0 && c < 64)
                {
                    pty_async_decode(p, p->rbits[0]);
                    memmove(p->rbits, p->rbits + 1, p->rn - 1);
                    p->rn--;
                    c++;
                    p->bits_deframed++;
                }
            }

            /* flush de-framed bytes to the peer PTY master */
            if (sA.on > 0)
            {
                int nw = write(sB.pty_fd, sA.obuf, sA.on);
                if (nw > 0)
                {
                    memmove(sA.obuf, sA.obuf + nw, sA.on - nw);
                    sA.on -= nw;
                }
            }
            if (sB.on > 0)
            {
                int nw = write(sA.pty_fd, sB.obuf, sB.on);
                if (nw > 0)
                {
                    memmove(sB.obuf, sB.obuf + nw, sB.on - nw);
                    sB.on -= nw;
                }
            }

            elapsed_frames++;
        }

        fprintf(stderr, "[child] done: %d frames A{rd=%ld cons=%ld idle=%ld rx=%ld defr=%ld by=%ld fe=%d} B{rd=%ld cons=%ld idle=%ld rx=%ld defr=%ld by=%ld fe=%d}\n",
                elapsed_frames,
                sA.bytes_read, sA.bits_consumed, sA.bits_idle, sA.bits_rx, sA.bits_deframed, sA.bytes_deframed, sA.framing_errors,
                sB.bytes_read, sB.bits_consumed, sB.bits_idle, sB.bits_rx, sB.bits_deframed, sB.bytes_deframed, sB.framing_errors);
        v22bs_destroy(sA.modem);
        v22bs_destroy(sB.modem);
        close(master_a);
        close(master_b);
        _exit(0);
    }

    /* Parent: slave A -> write pattern; expect identical bytes at slave B.
       Simultaneously the mirrored pattern flows B->A (both in one pass). */
    {
        unsigned char *pat = malloc(pat_len);
        unsigned char *mir = malloc(pat_len);
        int errs_a = 0, errs_b = 0, i;
        int got_a = 0, got_b = 0;
        unsigned rng = 0x1234ABCD;
        struct pollfd pfd[2];

        for (i = 0; i < pat_len; i++)
        {
            rng = rng * 1664525u + 1013904223u;
            pat[i] = (unsigned char)((rng >> 16) & 0xFF);
            mir[i] = (unsigned char)(~(pat[pat_len - 1 - i] & 0xFF));
        }

        if (write(slave_a, pat, pat_len) < 0)
            perror("write slave_a");
        if (write(slave_b, mir, pat_len) < 0)
            perror("write slave_b");

        /* read from both slaves until we have pat_len each */
        {
            unsigned char *racc = malloc(pat_len);
            unsigned char *rbcc = malloc(pat_len);
            int k, polls;

            if (fcntl(slave_a, F_SETFL, O_NONBLOCK) < 0 ||
                fcntl(slave_b, F_SETFL, O_NONBLOCK) < 0)
                perror("fcntl");

            /* bound the wait: training ~1-2 s + full-duplex transfer at 2400
               bps with 10 bits/byte = ~8.5 s for 2048 B/side; give slack */
            polls = 0;
            while ((got_a < pat_len || got_b < pat_len) && polls < 6000)
            {
                pfd[0].fd = slave_a; pfd[0].events = POLLIN; pfd[0].revents = 0;
                pfd[1].fd = slave_b; pfd[1].events = POLLIN; pfd[1].revents = 0;
                if (poll(pfd, 2, 100) <= 0)
                {
                    polls++;
                    continue;
                }
                for (k = 0; k < 2; k++)
                {
                    int fd = (k == 0) ? slave_a : slave_b;
                    int cap = pat_len;
                    unsigned char *dst = (k == 0) ? racc : rbcc;
                    int *got = (k == 0) ? &got_a : &got_b;
                    if ((pfd[k].revents & (POLLIN | POLLHUP | POLLERR)) != 0)
                    {
                        int nr = read(fd, dst + *got, cap - *got);
                        if (nr > 0)
                            *got += nr;
                    }
                }
                polls++;
            }

            errs_a = 0;
            {
                int first_bad = -1, max_run = 0, run = 0;
                for (i = 0; i < pat_len; i++)
                {
                    if (racc[i] != pat[i])
                    {
                        errs_a++;
                        if (first_bad < 0)
                            first_bad = i;
                        run++;
                        if (run > max_run)
                            max_run = run;
                    }
                    else
                        run = 0;
                }
                printf("[pty] A->B: %d bytes received, %d mismatches (first@%d maxrun=%d)  %s\n",
                       got_a, errs_a, first_bad, max_run, errs_a ? "FAIL" : "OK");
            }
            errs_b = 0;
            {
                int first_bad = -1, max_run = 0, run = 0;
                for (i = 0; i < pat_len; i++)
                {
                    if (rbcc[i] != mir[i])
                    {
                        errs_b++;
                        if (first_bad < 0)
                            first_bad = i;
                        run++;
                        if (run > max_run)
                            max_run = run;
                    }
                    else
                        run = 0;
                }
                printf("[pty] B->A: %d bytes received, %d mismatches (first@%d maxrun=%d)  %s\n",
                       got_b, errs_b, first_bad, max_run, errs_b ? "FAIL" : "OK");
            }

            free(racc);
            free(rbcc);
        }

        free(pat);
        free(mir);
    }

    close(slave_a);
    close(slave_b);
    waitpid(pid, &status, 0);
    printf("[pty] child exit=%d\n", status);
    return 0;
}