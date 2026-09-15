#include "sm_call.h"
#include "ppp/sm_pppd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <sys/wait.h>
#include <sys/stat.h>

/* ------------------------------------------------------------------ */
/* Modem callbacks                                                     */
/* ------------------------------------------------------------------ */

static int call_get_bit(void *ud)
{
    sm_call_t *c = ud;
    int b = sm_bitq_pop(&c->txbits);

    /* Idle line is all ones when there is nothing to send. */
    return (b < 0) ? 1 : b;
}

static void call_put_bit(void *ud, int bit)
{
    sm_call_t *c = ud;

    if (c->phase != SM_CALL_DATA)
        return;
    sm_deframer_bit(&c->deframer, bit);
}

static const char *status_name(v22bis_status_t st)
{
    switch (st)
    {
    case V22BIS_STATUS_TRAINING_SUCCEEDED: return "TRAINING_SUCCEEDED";
    case V22BIS_STATUS_TRAINING_FAILED:    return "TRAINING_FAILED";
    case V22BIS_STATUS_CARRIER_UP:         return "CARRIER_UP";
    case V22BIS_STATUS_CARRIER_DOWN:       return "CARRIER_DOWN";
    case V22BIS_STATUS_RETRAIN_OCCURRED:   return "RETRAIN";
    default:                               return "?";
    }
}

static void call_status(void *ud, v22bis_status_t st)
{
    sm_call_t *c = ud;

    sm_log_message(&c->log, SM_LOG_FLOW, "modem status: %s", status_name(st));
    if (st == V22BIS_STATUS_RETRAIN_OCCURRED)
    {
        /* A retrain resets the link; drop partial async state. */
        sm_deframer_init(&c->deframer);
        c->phase = SM_CALL_HANDSHAKE;
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

void sm_call_init(sm_call_t *c, const sm_call_config_t *cfg, int call_id)
{
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    c->call_id = call_id;
    c->ppp_fd = -1;
    c->pppd_pid = -1;
    sm_log_init(&c->log, cfg->log_level, "CALL", call_id);
    v22bis_init(&c->modem, false /* answerer */, cfg->rate,
                call_get_bit, c, call_put_bit, c, call_status, c);
    sm_bitq_init(&c->txbits);
    sm_deframer_init(&c->deframer);

    c->tone_phase = 0.0;
    c->tone_phase_inc = SM_TWO_PI * 2100.0 / (double) SM_SAMPLE_RATE;
    c->tone_amplitude = 6000.0;

    if (cfg->answer_tone_ms > 0 || cfg->pre_tone_silence_ms > 0)
    {
        c->phase = SM_CALL_ANSWER_TONE;
        c->silence_samples_left = cfg->pre_tone_silence_ms * SM_SAMPLE_RATE / 1000;
        c->tone_samples_left = cfg->answer_tone_ms * SM_SAMPLE_RATE / 1000;
    }
    else
    {
        c->phase = SM_CALL_HANDSHAKE;
    }
}

/* ------------------------------------------------------------------ */
/* PPP plumbing                                                        */
/* ------------------------------------------------------------------ */

static void ppp_flush(sm_call_t *c)
{
    while (c->ppy_out_len > 0 && c->ppp_fd >= 0)
    {
        ssize_t w = write(c->ppp_fd, c->ppy_out, (size_t) c->ppy_out_len);
        if (w < 0)
        {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return;
            sm_log_message(&c->log, SM_LOG_WARNING, "ppp write failed: %s", strerror(errno));
            return;
        }
        memmove(c->ppy_out, c->ppy_out + w, (size_t) (c->ppy_out_len - w));
        c->ppy_out_len -= (int) w;
    }
}

static void pump_ppp(sm_call_t *c)
{
    if (c->ppp_fd < 0 && !c->cfg.echo_data)
        return;

    /* Modem RX -> pppd (or echo). */
    if (c->phase == SM_CALL_DATA)
    {
        uint8_t tmp[256];
        int n = sm_deframer_take(&c->deframer, tmp, (int) sizeof(tmp));
        int off = 0;

        if (n > 0 && c->cfg.echo_data)
        {
            int i;
            for (i = 0; i < n; i++)
                sm_bitq_push_byte(&c->txbits, tmp[i]);
            c->data_bytes_tx += n;
            c->data_bytes_rx += n;
            return;
        }

        while (off < n)
        {
            if (c->ppy_out_len >= (int) sizeof(c->ppy_out))
            {
                sm_log_message(&c->log, SM_LOG_WARNING,
                               "ppp rx buffer full, dropped %d bytes", n - off);
                break;
            }
            {
                int space = (int) sizeof(c->ppy_out) - c->ppy_out_len;
                int chunk = (n - off < space) ? n - off : space;
                memcpy(c->ppy_out + c->ppy_out_len, tmp + off, (size_t) chunk);
                c->ppy_out_len += chunk;
                off += chunk;
            }
        }
        c->data_bytes_rx += n;
        ppp_flush(c);
    }

    /* pppd -> modem TX. Read regardless of phase so pppd never blocks; the
       bits wait in the queue until the modem reaches data mode. */
    {
        uint8_t tmp[256];
        ssize_t r = read(c->ppp_fd, tmp, sizeof(tmp));
        if (r > 0)
        {
            int i;
            for (i = 0; i < r; i++)
            {
                if (sm_bitq_push_byte(&c->txbits, tmp[i]) < 0)
                {
                    sm_log_message(&c->log, SM_LOG_WARNING, "modem tx queue full");
                    break;
                }
            }
            c->data_bytes_tx += i;
        }
        else if (r == 0)
        {
            sm_log_message(&c->log, SM_LOG_INFO, "pppd closed the pty");
            c->ppp_fd = -1;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Audio processing                                                    */
/* ------------------------------------------------------------------ */

static void process_audio(sm_call_t *c, const int16_t *in, int n)
{
    int i = 0;

    while (i < n)
    {
        if (c->phase == SM_CALL_ANSWER_TONE)
        {
            /* Tone phase: one sample at a time so the switch to modem TX is
               sample-exact. */
            if (c->silence_samples_left > 0)
            {
                c->txbuf[i] = 0;
                c->silence_samples_left--;
            }
            else if (c->tone_samples_left > 0)
            {
                c->txbuf[i] = sm_sat16((float) (c->tone_amplitude * sin(c->tone_phase)));
                c->tone_phase += c->tone_phase_inc;
                c->tone_samples_left--;
            }
            if (c->silence_samples_left <= 0 && c->tone_samples_left <= 0)
            {
                c->phase = SM_CALL_HANDSHAKE;
                sm_log_message(&c->log, SM_LOG_FLOW,
                               "answer sequence done -> V.22bis handshake (rate %d)", c->cfg.rate);
            }
            i++;
        }
        else
        {
            int m = n - i;
            v22bis_tx(&c->modem, c->txbuf + i, m);
            v22bis_rx(&c->modem, in + i, m);
            i = n;
        }
    }

    /* Detect data mode: both directions in NORMAL_OPERATION. */
    if (c->phase == SM_CALL_HANDSHAKE
        && c->modem.tx.training == V22BIS_TX_TRAINING_NORMAL_OPERATION
        && c->modem.rx.training == V22BIS_RX_TRAINING_NORMAL_OPERATION)
    {
        c->phase = SM_CALL_DATA;
        c->negotiated_rate = c->modem.negotiated_bit_rate;
        sm_deframer_init(&c->deframer);
        sm_log_message(&c->log, SM_LOG_FLOW,
                       "==> DATA MODE at %d bps (%s)",
                       c->negotiated_rate,
                       c->modem.rx.sixteen_way_decisions ? "16-way" : "4-way");
    }

    c->samples_in += n;
    c->samples_out += n;
}

/* ------------------------------------------------------------------ */
/* pppd management                                                     */
/* ------------------------------------------------------------------ */

static int start_pppd(sm_call_t *c)
{
    sm_pppd_config_t pc;
    char log_path[512];

    memset(&pc, 0, sizeof(pc));
    pc.pppd_path = c->cfg.pppd_path;
    pc.shim_exe = c->cfg.shim_exe;
    pc.local_ip = c->cfg.local_ip;
    pc.peer_ip = c->cfg.peer_ip;
    pc.dns1 = c->cfg.dns1;
    pc.dns2 = c->cfg.dns2;
    pc.auth = c->cfg.auth;
    pc.ip_up_script = c->cfg.ip_up_script;
    pc.ip_down_script = c->cfg.ip_down_script;
    if (c->cfg.log_dir && c->cfg.log_dir[0])
        snprintf(log_path, sizeof(log_path), "%s/pppd-call%d.log", c->cfg.log_dir, c->call_id);
    else
        log_path[0] = 0;
    pc.log_path = log_path[0] ? log_path : NULL;

    c->pppd_pid = sm_pppd_spawn(&pc, &c->ppp_fd);
    if (c->pppd_pid < 0)
    {
        sm_log_message(&c->log, SM_LOG_ERROR, "failed to spawn pppd: %s", strerror(errno));
        return -1;
    }
    fcntl(c->ppp_fd, F_SETFL, O_NONBLOCK);
    c->ppp_started = 1;
    sm_log_message(&c->log, SM_LOG_FLOW, "pppd started pid=%d %s:%s (log %s)",
                   (int) c->pppd_pid, c->cfg.local_ip, c->cfg.peer_ip,
                   log_path[0] ? log_path : "-");
    return 0;
}

static void stop_pppd(sm_call_t *c)
{
    if (c->ppp_fd >= 0)
    {
        close(c->ppp_fd);
        c->ppp_fd = -1;
    }
    if (c->pppd_pid > 0)
    {
        int st;
        kill(c->pppd_pid, SIGTERM);
        /* Give it a moment, then be firm. */
        if (waitpid(c->pppd_pid, &st, WNOHANG) == 0)
        {
            usleep(200000);
            kill(c->pppd_pid, SIGKILL);
            waitpid(c->pppd_pid, &st, 0);
        }
        c->pppd_pid = -1;
    }
}

/* ------------------------------------------------------------------ */
/* Main call loop                                                      */
/* ------------------------------------------------------------------ */

int sm_call_run(sm_call_t *c, int socket_fd)
{
    uint8_t payload[SM_AS_MAX_PAYLOAD];
    uint8_t kind;
    size_t plen;
    int r;
    int timeout_count = 0;
    int rc = 0;

    sm_ast_init(&c->as, socket_fd);

    /* First message from Asterisk carries the 16-byte call UUID. */
    for (;;)
    {
        r = sm_ast_read(&c->as, &kind, payload, sizeof(payload), &plen, 5000);
        if (r < 0)
        {
            sm_log_message(&c->log, SM_LOG_WARNING, "socket closed before UUID");
            return -1;
        }
        if (r == 0)
        {
            sm_log_message(&c->log, SM_LOG_WARNING, "timeout waiting for UUID");
            return -1;
        }
        if (kind == SM_AS_KIND_UUID && plen >= 16)
        {
            char u[37];
            sm_ast_uuid_string(payload, u);
            sm_log_message(&c->log, SM_LOG_FLOW, "call start, UUID %s", u);
            break;
        }
        if (kind == SM_AS_KIND_AUDIO)
        {
            sm_log_message(&c->log, SM_LOG_FLOW, "audio before UUID, starting anyway");
            process_audio(c, (const int16_t *) payload, (int) plen / 2);
            break;
        }
        sm_log_message(&c->log, SM_LOG_DEBUG, "pre-call msg kind=0x%02x len=%d", kind, (int) plen);
    }

    if (c->cfg.enable_ppp && !c->cfg.echo_data)
    {
        if (start_pppd(c) < 0)
        {
            stop_pppd(c);
            return -1;
        }
    }

    while (c->phase != SM_CALL_HANGUP)
    {
        r = sm_ast_read(&c->as, &kind, payload, sizeof(payload), &plen, 2000);
        if (r < 0)
        {
            sm_log_message(&c->log, SM_LOG_FLOW, "call ended: socket closed");
            break;
        }
        if (r == 0)
        {
            if (++timeout_count >= 5)
            {
                sm_log_message(&c->log, SM_LOG_FLOW, "call ended: no audio for 10s");
                break;
            }
            pump_ppp(c);
            continue;
        }
        timeout_count = 0;

        switch (kind)
        {
        case SM_AS_KIND_AUDIO:
            if (plen & 1)
            {
                sm_log_message(&c->log, SM_LOG_WARNING, "odd audio payload %d", (int) plen);
                plen--;
            }
            if (plen > (size_t) (sizeof(c->txbuf) / sizeof(c->txbuf[0])) * 2)
                plen = (size_t) (sizeof(c->txbuf) / sizeof(c->txbuf[0])) * 2;
            process_audio(c, (const int16_t *) payload, (int) plen / 2);
            if (sm_ast_write_audio(&c->as, c->txbuf, plen / 2) < 0)
            {
                sm_log_message(&c->log, SM_LOG_WARNING, "audio write failed");
                goto out;
            }
            break;

        case SM_AS_KIND_HANGUP:
            sm_log_message(&c->log, SM_LOG_FLOW, "call ended: hangup from Asterisk");
            goto out;

        case SM_AS_KIND_DTMF:
            if (plen >= 1)
                sm_log_message(&c->log, SM_LOG_FLOW, "DTMF digit '%c'", payload[0]);
            break;

        case SM_AS_KIND_ERROR:
            sm_log_message(&c->log, SM_LOG_WARNING, "AudioSocket error frame");
            goto out;

        default:
            sm_log_message(&c->log, SM_LOG_DEBUG, "ignoring msg kind=0x%02x len=%d", kind, (int) plen);
            break;
        }

        pump_ppp(c);

        /* If pppd died, the data path is gone: end the call. */
        if (c->ppp_started && c->pppd_pid > 0)
        {
            int st;
            if (waitpid(c->pppd_pid, &st, WNOHANG) == c->pppd_pid)
            {
                sm_log_message(&c->log, SM_LOG_FLOW, "pppd exited (status %d)", st);
                c->pppd_pid = -1;
                goto out;
            }
        }
    }

out:
    sm_log_message(&c->log, SM_LOG_FLOW,
                   "call summary: samples_in=%lld samples_out=%lld tx_bytes=%lld rx_bytes=%lld rate=%d state=%s",
                   c->samples_in, c->samples_out, c->data_bytes_tx, c->data_bytes_rx,
                   c->modem.negotiated_bit_rate,
                   c->phase == SM_CALL_DATA ? "DATA" : "HANDSHAKE");
    stop_pppd(c);
    close(socket_fd);
    return rc;
}
