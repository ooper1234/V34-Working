/* BinModem caller <-> answerer through the C FFI, over a delayed line at
   8 kHz: V.8 to V.34 to data mode, then async-framed bytes each way, which
   is exactly what sm_call.c does with the engine on a real call. Exit 0
   when both directions carry their bytes intact. */

#include "bm_answerer.h"
#include "serial/sm_async.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FS 8000
#define CHUNK 160
#define DELAY_MS 30
#define DELAY (FS * DELAY_MS / 1000)
#define SIM_SECONDS 60
#define RING (1 << 16)              /* power of two, > DELAY */
#define RING_MASK (RING - 1)
#define N_BYTES 128

#define TRACE_BITS 65536

typedef struct {
    sm_bitq_t q;
    sm_deframer_t d;
    int data_mode;                   /* mirrors sm_call.c's phase == DATA */
    long gate_at;                    /* sample count when the gate opened */
    int sent[TRACE_BITS];            /* bits popped after the gate opened */
    int sent_n;
    int got[TRACE_BITS];             /* bits delivered after the gate opened */
    int got_n;
    long pops;                       /* total get_bit calls */
} end_t;

static int get_bit(void *ud)
{
    end_t *e = ud;
    int b = sm_bitq_pop(&e->q);
    int r = (b < 0) ? 1 : b;

    e->pops++;
    if (e->data_mode && e->sent_n < TRACE_BITS)
        e->sent[e->sent_n++] = r;
    return r;
}

static void put_bit(void *ud, int bit)
{
    end_t *e = ud;

    if (!e->data_mode)
        return;
    if (e->got_n < TRACE_BITS)
        e->got[e->got_n++] = bit & 1;
    sm_deframer_bit(&e->d, bit);
}

static const char *st_name(int st)
{
    switch (st)
    {
    case BM_RUNNING:      return "RUNNING";
    case BM_CONNECTED:    return "CONNECTED";
    case BM_FAILED:       return "FAILED";
    case BM_AGREED_V22:   return "AGREED_V22";
    case BM_AGREED_OTHER: return "AGREED_OTHER";
    case BM_RETRAINING:   return "RETRAINING";
    default:              return "?";
    }
}

int main(void)
{
    static end_t call_e, ans_e;
    static int16_t to_answer[RING], to_call[RING];
    bm_answerer *caller, *answerer;
    long w_a = DELAY, w_c = DELAY;   /* ring write positions */
    long limit = (long) FS * SIM_SECONDS;
    long chunk;
    uint8_t from_call[N_BYTES], from_answer[N_BYTES];
    /* What arrived: caller->answerer lands in ans_e, answer->call in call_e. */
    uint8_t rc_buf[256], ra_buf[256];
    int rc_n = 0, ra_n = 0;
    int rc_done = 0, ra_done = 0;
    int connected_at = -1;
    int r = 1;

    memset(&call_e, 0, sizeof(call_e));
    memset(&ans_e, 0, sizeof(ans_e));
    sm_bitq_init(&call_e.q);
    sm_bitq_init(&ans_e.q);
    sm_deframer_init(&call_e.d);
    sm_deframer_init(&ans_e.d);
    /* Rings pre-zeroed by static storage: the DELAY samples before the
       first write read as silence. */

    caller = bm_create(0 /* call */, 1 /* want V.34 */,
                       get_bit, &call_e, put_bit, &call_e);
    answerer = bm_create(1 /* answer */, 1 /* want V.34 */,
                         get_bit, &ans_e, put_bit, &ans_e);
    if (!caller || !answerer)
    {
        fprintf(stderr, "bm_create failed\n");
        return 1;
    }

    for (chunk = 0; chunk < limit / CHUNK; chunk++)
    {
        int sc = bm_status(caller);
        int sa = bm_status(answerer);
        int k;
        static int prev_sc = -99, prev_sa = -99;
        static int trace_left = 0;
        static int prev_found_c = -1, prev_found_a = -1;
        static int prev_slips_c = -1, prev_slips_a = -1;
        int pc0 = bm_pending(caller), pa0 = bm_pending(answerer);
        int found_c = bm_found_again(caller), found_a = bm_found_again(answerer);
        int slips_c = bm_slips(caller), slips_a = bm_slips(answerer);
        long pc_pop0 = call_e.pops, pa_pop0 = ans_e.pops;

        if (sc != prev_sc || sa != prev_sa)
        {
            printf("[t=%ld] caller=%s answerer=%s\n", w_c - DELAY,
                   st_name(sc), st_name(sa));
            fflush(stdout);
            prev_sc = sc;
            prev_sa = sa;
        }

        if (sc == BM_FAILED || sa == BM_FAILED)
        {
            fprintf(stderr, "failed: caller=%s (%s) answerer=%s (%s) at %.1fs\n",
                    st_name(sc), bm_failure(caller),
                    st_name(sa), bm_failure(answerer),
                    (double) (w_c - DELAY) / FS);
            goto out;
        }
        if (sc == BM_AGREED_V22 || sa == BM_AGREED_V22
            || sc == BM_AGREED_OTHER || sa == BM_AGREED_OTHER)
        {
            fprintf(stderr, "unexpected hand-back: caller=%s answerer=%s\n",
                    st_name(sc), st_name(sa));
            goto out;
        }

        if (connected_at < 0 && sc == BM_CONNECTED && sa == BM_CONNECTED)
        {
            connected_at = (int) ((double) (w_c - DELAY) / FS * 1000.0);
            printf("connected at %d ms: caller rx=%d tx=%d, answerer rx=%d tx=%d\n",
                   connected_at,
                   bm_rate_rx(caller), bm_rate_tx(caller),
                   bm_rate_rx(answerer), bm_rate_tx(answerer));
            printf("  phases: %s | %s\n", bm_phase(caller), bm_phase(answerer));
            printf("  error control: caller=%d answerer=%d, compression=%d/%d\n",
                   bm_error_control(caller), bm_error_control(answerer),
                   bm_compression(caller), bm_compression(answerer));
            if (!bm_error_control(caller) || !bm_error_control(answerer)
                || bm_compression(caller) != 1 || bm_compression(answerer) != 1)
            {
                fprintf(stderr, "V.42/compression negotiation did not complete\n");
                goto out;
            }
            printf("  audio so far: underruns c/a %llu/%llu, clips c/a %llu/%llu, peak c/a %.3f/%.3f\n",
                   bm_underruns(caller), bm_underruns(answerer),
                   bm_clips(caller), bm_clips(answerer),
                   bm_peak(caller), bm_peak(answerer));
            trace_left = 40;
            for (k = 0; k < N_BYTES; k++)
            {
                from_call[k] = (uint8_t) (k * 7 + 3);
                from_answer[k] = (uint8_t) (k * 5 + 11);
                sm_bitq_push_byte(&call_e.q, from_call[k]);
                sm_bitq_push_byte(&ans_e.q, from_answer[k]);
            }
        }

        bm_service(caller);
        bm_service(answerer);

        /* As sm_call.c does at first BM_CONNECTED: drop what the receiver
           made of the handshake, then open the byte path. */
        if (bm_status(caller) == BM_CONNECTED && !call_e.data_mode)
        {
            bm_flush_rx(caller);
            call_e.data_mode = 1;
            call_e.gate_at = w_c;
            printf("[t=%ld] caller gate open\n", w_c - DELAY);
        }
        if (bm_status(answerer) == BM_CONNECTED && !ans_e.data_mode)
        {
            bm_flush_rx(answerer);
            ans_e.data_mode = 1;
            ans_e.gate_at = w_a;
            printf("[t=%ld] answerer gate open\n", w_a - DELAY);
        }

        for (k = 0; k < CHUNK; k++)
        {
            int16_t in_c = to_call[(w_c - DELAY) & RING_MASK];
            int16_t in_a = to_answer[(w_a - DELAY) & RING_MASK];
            int16_t out_c = bm_step(caller, in_c);
            int16_t out_a = bm_step(answerer, in_a);

            to_answer[w_a & RING_MASK] = out_c;
            w_a++;
            to_call[w_c & RING_MASK] = out_a;
            w_c++;
        }

        if (trace_left > 0 || found_c != prev_found_c || found_a != prev_found_a
            || slips_c != prev_slips_c || slips_a != prev_slips_a)
        {
            printf("  tr pend c/a %d/%d fill c/a %ld/%ld rx_total c/a %llu/%llu "
                   "carr c/a %d/%d found c/a %d/%d slips c/a %d/%d "
                   "sent_n=%d got_n=%d q=%zu/%zu ph=%s|%s\n",
                   pc0, pa0,
                   call_e.pops - pc_pop0, ans_e.pops - pa_pop0,
                   bm_rx_total(caller), bm_rx_total(answerer),
                   bm_carrier(caller), bm_carrier(answerer),
                   found_c, found_a, slips_c, slips_a,
                   call_e.sent_n, ans_e.got_n,
                   sm_bitq_count(&call_e.q), sm_bitq_count(&ans_e.q),
                   bm_phase(caller), bm_phase(answerer));
            prev_found_c = found_c;
            prev_found_a = found_a;
            prev_slips_c = slips_c;
            prev_slips_a = slips_a;
            if (trace_left > 0)
                trace_left--;
        }

        if (connected_at >= 0)
        {
            uint8_t tmp[256];
            int n;

            n = sm_deframer_take(&call_e.d, tmp, sizeof(tmp));
            if (getenv("BM_TRACE_TAKE"))
            {
                static int shown = 0;
                if (shown++ < 20 || n > 0)
                    fprintf(stderr, "  [chunk %ld] take call n=%d ra_n=%d bytes_out=%llu\n",
                            chunk, n, ra_n,
                            (unsigned long long) call_e.d.bytes_out);
            }
            if (n > 0 && ra_n + n <= (int) sizeof(ra_buf))
            {
                memcpy(ra_buf + ra_n, tmp, (size_t) n);
                ra_n += n;
            }
            else if (n > 0)
            {
                fprintf(stderr, "  DROPPED %d bytes (ra_n=%d)\n", n, ra_n);
            }
            n = sm_deframer_take(&ans_e.d, tmp, sizeof(tmp));
            if (n > 0 && rc_n + n <= (int) sizeof(rc_buf))
            {
                memcpy(rc_buf + rc_n, tmp, (size_t) n);
                rc_n += n;
            }
            n = sm_deframer_take(&call_e.d, tmp, sizeof(tmp));
            if (n > 0 && ra_n + n <= (int) sizeof(ra_buf))
            {
                memcpy(ra_buf + ra_n, tmp, (size_t) n);
                ra_n += n;
            }
            if (rc_n >= N_BYTES && !rc_done)
            {
                int i;

                rc_done = 1;
                for (i = 0; i < N_BYTES; i++)
                {
                    if (rc_buf[i] != from_call[i])
                    {
                        fprintf(stderr,
                                "byte mismatch call->answerer at %d: got %02x want %02x "
                                "(framing_errors ans=%llu call=%llu)\n",
                                i, rc_buf[i], from_call[i],
                                (unsigned long long) ans_e.d.framing_errors,
                                (unsigned long long) call_e.d.framing_errors);
                        fprintf(stderr, "  got :");
                        {
                            int j;
                            for (j = (i > 8 ? i - 8 : 0); j < i + 8 && j < N_BYTES; j++)
                                fprintf(stderr, " %02x", rc_buf[j]);
                        }
                        fprintf(stderr, "\n  want:");
                        {
                            int j;
                            for (j = (i > 8 ? i - 8 : 0); j < i + 8 && j < N_BYTES; j++)
                                fprintf(stderr, " %02x", from_call[j]);
                        }
                        fprintf(stderr, "\n");
                        {
                            /* Bit level: what the caller's engine took after
                               its gate vs what the answerer's engine handed
                               up after its gate, and where they part. */
                            int d = 0, n, off = 0, i2, j;
                            n = call_e.sent_n < ans_e.got_n ? call_e.sent_n : ans_e.got_n;
                            while (d < n && call_e.sent[d] == ans_e.got[d])
                                d++;
                            for (i2 = 1; i2 < 400 && i2 < n; i2++)
                            {
                                int k, ok = 1;
                                for (k = 0; k + i2 < n && k < 400; k++)
                                    if (ans_e.got[k] != call_e.sent[k + i2])
                                    {
                                        ok = 0;
                                        break;
                                    }
                                if (ok)
                                {
                                    off = i2;
                                    break;
                                }
                            }
                            fprintf(stderr,
                                    "  bits: caller sent %d (gate t=%ld), answerer got %d (gate t=%ld), "
                                    "diverge at %d, best shift %d\n",
                                    call_e.sent_n, call_e.gate_at - DELAY,
                                    ans_e.got_n, ans_e.gate_at - DELAY, d, off);
                            {
                                /* The first zero the answerer saw after the
                                   gate, with context: this is what fed the
                                   deframer. */
                                int z = 0;
                                while (z < ans_e.got_n && ans_e.got[z] != 0)
                                    z++;
                                fprintf(stderr, "  first 0 in got at bit %d (of %d):\n  ctx:",
                                        z, ans_e.got_n);
                                for (j = (z > 16 ? z - 16 : 0); j < z + 96 && j < ans_e.got_n; j++)
                                    fprintf(stderr, "%d", ans_e.got[j]);
                                fprintf(stderr, "\n");
                            }
                            fprintf(stderr, "  sent:");
                            for (j = 0; j < 96 && j < call_e.sent_n; j++)
                                fprintf(stderr, "%d", call_e.sent[j]);
                            fprintf(stderr, "\n  got :");
                            for (j = 0; j < 96 && j < ans_e.got_n; j++)
                                fprintf(stderr, "%d", ans_e.got[j]);
                            fprintf(stderr, "\n");
                        }
                        goto out;
                    }
                }
                printf("call->answerer bytes intact (%d)\n", N_BYTES);
            }
            if (ra_n >= N_BYTES && !ra_done)
            {
                int i;

                ra_done = 1;
                for (i = 0; i < N_BYTES; i++)
                {
                    if (ra_buf[i] != from_answer[i])
                    {
                        fprintf(stderr, "byte mismatch answerer->call at %d: got %02x want %02x "
                                "(framing_errors call=%llu ans=%llu)\n",
                                i, ra_buf[i], from_answer[i],
                                (unsigned long long) call_e.d.framing_errors,
                                (unsigned long long) ans_e.d.framing_errors);
                        goto out;
                    }
                }
                printf("answerer->call bytes intact (%d)\n", N_BYTES);
            }
            if (rc_done && ra_done)
            {
                printf("bytes intact both ways (%d each)\n", N_BYTES);
                r = 0;
                goto out;
            }
        }
    }

    fprintf(stderr, "timeout at %.0fs: caller=%s (%s) answerer=%s (%s), connected_at=%d, bytes %d/%d\n",
            (double) (w_c - DELAY) / FS,
            st_name(bm_status(caller)), bm_phase(caller),
            st_name(bm_status(answerer)), bm_phase(answerer),
            connected_at, rc_n, ra_n);
    fprintf(stderr, "  rx_total c/a %llu/%llu, slips c/a %d/%d, found c/a %d/%d, "
            "carrier c/a %d/%d\n",
            bm_rx_total(caller), bm_rx_total(answerer),
            bm_slips(caller), bm_slips(answerer),
            bm_found_again(caller), bm_found_again(answerer),
            bm_carrier(caller), bm_carrier(answerer));
    fprintf(stderr, "  deframer: ans bytes=%llu errors=%llu, call bytes=%llu errors=%llu\n",
            (unsigned long long) ans_e.d.bytes_out,
            (unsigned long long) ans_e.d.framing_errors,
            (unsigned long long) call_e.d.bytes_out,
            (unsigned long long) call_e.d.framing_errors);
    fprintf(stderr, "  got_n=%d sent_n(caller)=%d q=%zu/%zu\n",
            ans_e.got_n, call_e.sent_n,
            sm_bitq_count(&call_e.q), sm_bitq_count(&ans_e.q));

out:
    {
        unsigned char lc = 0, la = 0;
        printf("  audio: underruns c/a %llu/%llu, up_odd c/a %llu/%llu (last %u/%u), "
               "clips c/a %llu/%llu, peak c/a %.3f/%.3f\n",
               bm_underruns(caller), bm_underruns(answerer),
               bm_up_odd(caller, &lc), bm_up_odd(answerer, &la), lc, la,
               bm_clips(caller), bm_clips(answerer),
               bm_peak(caller), bm_peak(answerer));
    }
    bm_destroy(caller);
    bm_destroy(answerer);
    return r;
}
