#include "sm_sip.h"
#include "ast_socket/sm_ast_socket.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ------------------------------------------------------------------ */
/* G.711 mu-law                                                        */
/* ------------------------------------------------------------------ */

int16_t sm_ulaw_decode(uint8_t u)
{
    int t;

    u = (uint8_t) ~u;
    t = ((u & 0x0F) << 3) + 0x84;
    t <<= (u & 0x70) >> 4;
    return (int16_t) ((u & 0x80) ? (0x84 - t) : (t - 0x84));
}

uint8_t sm_ulaw_encode(int16_t pcm)
{
    const int BIAS = 0x84;
    const int CLIP = 32635;
    int sign = (pcm >> 8) & 0x80;
    int exponent = 7;
    int mantissa;
    int sample = pcm;

    if (sign)
        sample = -sample;
    if (sample > CLIP)
        sample = CLIP;
    sample += BIAS;
    while (exponent > 0 && (sample & 0x4000) == 0)
    {
        sample <<= 1;
        exponent--;
    }
    mantissa = (sample >> 10) & 0x0F;
    return (uint8_t) ~(sign | (exponent << 4) | mantissa);
}

/* ------------------------------------------------------------------ */
/* SIP message parsing                                                 */
/* ------------------------------------------------------------------ */

#define SM_SIP_MAX_VIAS 4

typedef struct {
    int is_response;
    int status;
    char method[16];
    char uri[256];
    char via[SM_SIP_MAX_VIAS][320];
    int n_via;
    char from[320];
    char to[320];
    char call_id[256];
    char cseq[64];
    int cseq_num;
    char cseq_method[16];
    char contact[320];
    char content_type[64];
    char user_agent[128];
    int content_length;
    uint8_t body[SM_SIP_MAX_MSG];
    int body_len;
    char from_tag[64];
    char to_tag[64];
} sip_msg_t;

static const char *find_eol(const char *s, const char *end)
{
    while (s < end)
    {
        if (*s == '\n')
            return s;
        s++;
    }
    return end;
}

static void copy_line_value(const char *start, const char *end, char *out, size_t outlen)
{
    const char *e = end;
    size_t n;

    while (e > start && (e[-1] == '\r' || e[-1] == ' ' || e[-1] == '\t'))
        e--;
    while (start < e && (*start == ' ' || *start == '\t'))
        start++;
    n = (size_t) (e - start);
    if (n >= outlen)
        n = outlen - 1;
    memcpy(out, start, n);
    out[n] = 0;
}

static void extract_tag(const char *hdr, char *out, size_t outlen)
{
    const char *p = strstr(hdr, "tag=");

    out[0] = 0;
    if (!p)
        return;
    p += 4;
    {
        size_t i = 0;
        while (p[i] && p[i] != ';' && p[i] != ' ' && p[i] != '\r' && p[i] != '\n'
               && i < outlen - 1)
        {
            out[i] = p[i];
            i++;
        }
        out[i] = 0;
    }
}

static int parse_sip(uint8_t *buf, int len, sip_msg_t *m)
{
    char *s = (char *) buf;
    char *end = s + len;
    char *p;
    char *body = NULL;

    memset(m, 0, sizeof(*m));

    /* Find the header/body separator. */
    for (p = s; p + 1 < end; p++)
    {
        if (p[0] == '\r' && p[1] == '\n' && p + 3 < end && p[2] == '\r' && p[3] == '\n')
        {
            body = p + 4;
            break;
        }
        if (p[0] == '\n' && p + 1 < end && p[1] == '\n')
        {
            body = p + 2;
            break;
        }
    }
    if (!body)
        body = end;

    /* First line. */
    {
        char *eol = (char *) find_eol(s, body);
        char line[512];
        size_t n = (size_t) (eol - s);

        if (n >= sizeof(line))
            n = sizeof(line) - 1;
        memcpy(line, s, n);
        line[n] = 0;
        if (strncmp(line, "SIP/2.0", 7) == 0)
        {
            m->is_response = 1;
            m->status = atoi(line + 8);
        }
        else
        {
            char *sp1 = strchr(line, ' ');
            if (sp1)
            {
                size_t mn = (size_t) (sp1 - line);
                if (mn >= sizeof(m->method))
                    mn = sizeof(m->method) - 1;
                memcpy(m->method, line, mn);
                m->method[mn] = 0;
                sp1++;
                {
                    char *sp2 = strchr(sp1, ' ');
                    size_t un = sp2 ? (size_t) (sp2 - sp1) : strlen(sp1);
                    if (un >= sizeof(m->uri))
                        un = sizeof(m->uri) - 1;
                    memcpy(m->uri, sp1, un);
                    m->uri[un] = 0;
                }
            }
        }
        s = eol < body ? eol + 1 : body;
    }

    /* Headers. */
    while (s < body)
    {
        char *eol = (char *) find_eol(s, body);
        const char *v;

        if (eol > s && eol[-1] == '\r')
        {
            /* normal */
        }
        if (s == eol || (*s == '\r' && eol - s == 1))
        {
            s = eol + 1;
            continue;
        }

        if (strncasecmp(s, "Via:", 4) == 0 || strncasecmp(s, "v:", 2) == 0)
        {
            v = s + (strncasecmp(s, "Via:", 4) == 0 ? 4 : 2);
            if (m->n_via < SM_SIP_MAX_VIAS)
                copy_line_value(v, eol, m->via[m->n_via++], 320);
        }
        else if (strncasecmp(s, "From:", 5) == 0 || strncasecmp(s, "f:", 2) == 0)
        {
            v = s + (strncasecmp(s, "From:", 5) == 0 ? 5 : 2);
            copy_line_value(v, eol, m->from, sizeof(m->from));
            extract_tag(m->from, m->from_tag, sizeof(m->from_tag));
        }
        else if (strncasecmp(s, "To:", 3) == 0 || strncasecmp(s, "t:", 2) == 0)
        {
            v = s + (strncasecmp(s, "To:", 3) == 0 ? 3 : 2);
            copy_line_value(v, eol, m->to, sizeof(m->to));
            extract_tag(m->to, m->to_tag, sizeof(m->to_tag));
        }
        else if (strncasecmp(s, "Call-ID:", 8) == 0 || strncasecmp(s, "i:", 2) == 0)
        {
            v = s + (strncasecmp(s, "Call-ID:", 8) == 0 ? 8 : 2);
            copy_line_value(v, eol, m->call_id, sizeof(m->call_id));
        }
        else if (strncasecmp(s, "CSeq:", 5) == 0)
        {
            v = s + 5;
            copy_line_value(v, eol, m->cseq, sizeof(m->cseq));
            m->cseq_num = atoi(m->cseq);
            {
                char *sp = strchr(m->cseq, ' ');
                if (sp)
                    snprintf(m->cseq_method, sizeof(m->cseq_method), "%s", sp + 1);
            }
        }
        else if (strncasecmp(s, "Contact:", 8) == 0 || strncasecmp(s, "m:", 2) == 0)
        {
            v = s + (strncasecmp(s, "Contact:", 8) == 0 ? 8 : 2);
            copy_line_value(v, eol, m->contact, sizeof(m->contact));
        }
        else if (strncasecmp(s, "Content-Type:", 13) == 0)
        {
            v = s + 13;
            copy_line_value(v, eol, m->content_type, sizeof(m->content_type));
        }
        else if (strncasecmp(s, "User-Agent:", 11) == 0)
        {
            v = s + 11;
            copy_line_value(v, eol, m->user_agent, sizeof(m->user_agent));
        }
        else if (strncasecmp(s, "Content-Length:", 15) == 0)
        {
            m->content_length = atoi(s + 15);
        }

        s = eol + 1;
    }

    /* Body. */
    if (body < end)
    {
        int blen = (int) (end - body);
        if (m->content_length > 0 && m->content_length < blen)
            blen = m->content_length;
        if (blen > (int) sizeof(m->body))
            blen = (int) sizeof(m->body);
        memcpy(m->body, body, (size_t) blen);
        m->body_len = blen;
    }
    return 0;
}

static void extract_sdp(const sip_msg_t *m, char *ip, size_t iplen, int *port,
                        int *pt_out)
{
    const char *s = (const char *) m->body;
    const char *end = s + m->body_len;

    ip[0] = 0;
    *port = 0;
    *pt_out = 0;
    while (s < end)
    {
        const char *eol = find_eol(s, end);
        if (strncmp(s, "c=IN IP4 ", 9) == 0)
        {
            copy_line_value(s + 9, eol, ip, iplen);
        }
        else if (strncmp(s, "m=audio ", 8) == 0)
        {
            /* m=audio PORT RTP/AVP 0 101 */
            const char *q = s + 8;
            *port = atoi(q);
            {
                const char *r = strstr(q, "RTP/AVP");
                if (r)
                {
                    int pt;
                    r += 7;
                    while (r < eol && (*r == ' ' || *r == '\t'))
                        r++;
                    pt = atoi(r);
                    *pt_out = pt;
                }
            }
        }
        s = eol < end ? eol + 1 : end;
    }
}

/* ------------------------------------------------------------------ */
/* Server                                                              */
/* ------------------------------------------------------------------ */

struct sm_sip {
    sm_sip_config_t cfg;
    sm_log_t log;
    int sip_fd;

    /* Active call state. */
    int call_active;
    int media_active;
    char call_id[256];
    char to_hdr[320];
    char from_hdr[320];
    char contact_hdr[320];
    int cseq;
    uint32_t local_ssrc;
    uint16_t rtp_seq;
    uint32_t rtp_ts;
    struct sockaddr_in rtp_peer;
    int rtp_peer_known;
    int rtp_fd;
    int daemon_fd;
    sm_ast_socket_t daemon_as;

    /* Last 200 OK for INVITE retransmission. */
    uint8_t last_ok[SM_SIP_MAX_MSG];
    int last_ok_len;

    int calls_done;
};

static void make_tag(char *out, size_t outlen)
{
    snprintf(out, outlen, "%08x%08x", (unsigned) rand(), (unsigned) time(NULL));
}

/* Build a SIP response for request m. */
static int build_response(const sip_msg_t *m, int status, const char *reason,
                          const char *extra_headers, const char *body,
                          uint8_t *out, size_t outlen)
{
    char to_tag[64];
    char buf[SM_SIP_MAX_MSG];
    int n = 0;
    int i;
    size_t blen = body ? strlen(body) : 0;

    make_tag(to_tag, sizeof(to_tag));

    n += snprintf(buf + n, sizeof(buf) - n, "SIP/2.0 %d %s\r\n", status, reason);
    for (i = 0; i < m->n_via; i++)
        n += snprintf(buf + n, sizeof(buf) - n, "Via: %s\r\n", m->via[i]);
    n += snprintf(buf + n, sizeof(buf) - n, "From: %s\r\n", m->from);
    /* Add a To tag if the request's To has none. */
    if (m->to_tag[0])
        n += snprintf(buf + n, sizeof(buf) - n, "To: %s\r\n", m->to);
    else
        n += snprintf(buf + n, sizeof(buf) - n, "To: %s;tag=%s\r\n", m->to, to_tag);
    n += snprintf(buf + n, sizeof(buf) - n, "Call-ID: %s\r\n", m->call_id);
    n += snprintf(buf + n, sizeof(buf) - n, "CSeq: %s\r\n", m->cseq);
    if (extra_headers && extra_headers[0])
        n += snprintf(buf + n, sizeof(buf) - n, "%s", extra_headers);
    n += snprintf(buf + n, sizeof(buf) - n, "Server: SoftmodemSIP/1.0\r\n");
    if (blen > 0)
        n += snprintf(buf + n, sizeof(buf) - n, "Content-Type: application/sdp\r\n");
    n += snprintf(buf + n, sizeof(buf) - n, "Content-Length: %d\r\n\r\n", (int) blen);
    if (blen > 0)
    {
        memcpy(buf + n, body, blen);
        n += (int) blen;
    }
    if ((size_t) n > outlen)
        n = (int) outlen;
    memcpy(out, buf, (size_t) n);
    return n;
}

static void send_raw(sm_sip_t *s, const struct sockaddr_in *to,
                     const uint8_t *data, int len)
{
    sendto(s->sip_fd, data, (size_t) len, 0, (const struct sockaddr *) to, sizeof(*to));
}

static void send_response(sm_sip_t *s, const sip_msg_t *m,
                          const struct sockaddr_in *src, int status,
                          const char *reason, const char *extra,
                          const char *body)
{
    uint8_t buf[SM_SIP_MAX_MSG];
    int n = build_response(m, status, reason, extra, body, buf, sizeof(buf));

    if (n > 0)
        send_raw(s, src, buf, n);
}

/* ------------------------------------------------------------------ */
/* Media bridge                                                        */
/* ------------------------------------------------------------------ */

static int daemon_connect(sm_sip_t *s)
{
    int fd = sm_ast_connect(s->cfg.daemon_host, s->cfg.daemon_port);

    if (fd < 0)
        return -1;
    {
        sm_ast_socket_t as;
        uint8_t uuid[16];
        int i;
        unsigned int a, b, c, d;

        if (sscanf(s->cfg.uuid, "%8x-%4x-%4x-%4x-%8x",
                   &a, &b, &c, &d, (unsigned *) &i) == 5)
        {
            /* Construct from the string form for readability. */
        }
        /* Use a fixed call UUID derived from the call id: simple and unique
           enough for the daemon's logging. */
        for (i = 0; i < 16; i++)
            uuid[i] = (uint8_t) (i + 1);
        sm_ast_init(&as, fd);
        sm_ast_write(&as, SM_AS_KIND_UUID, uuid, 16);
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    sm_ast_init(&s->daemon_as, fd);
    return fd;
}

static void end_call(sm_sip_t *s)
{
    if (s->daemon_fd >= 0)
    {
        close(s->daemon_fd);
        s->daemon_fd = -1;
    }
    if (s->rtp_fd >= 0)
    {
        close(s->rtp_fd);
        s->rtp_fd = -1;
    }
    s->call_active = 0;
    s->media_active = 0;
    s->rtp_peer_known = 0;
    s->call_id[0] = 0;
    s->last_ok_len = 0;
    s->calls_done++;
}

static void send_bye(sm_sip_t *s)
{
    char buf[1024];
    char tag[64];
    uint32_t ssrc = s->local_ssrc;
    int n;

    if (!s->call_active || !s->rtp_peer_known)
        return;
    make_tag(tag, sizeof(tag));
    n = snprintf(buf, sizeof(buf),
                 "BYE sip:modem@%s SIP/2.0\r\n"
                 "Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%08x\r\n"
                 "From: <sip:modem@%s>;tag=%s\r\n"
                 "To: %s\r\n"
                 "Call-ID: %s\r\n"
                 "CSeq: %d BYE\r\n"
                 "Max-Forwards: 70\r\n"
                 "Content-Length: 0\r\n\r\n",
                 s->cfg.advertise_ip, s->cfg.advertise_ip, s->cfg.bind_port,
                 ssrc, s->cfg.advertise_ip, tag, s->to_hdr, s->call_id,
                 s->cseq + 1);
    if (n > 0)
        sendto(s->sip_fd, buf, (size_t) n, 0,
               (struct sockaddr *) &s->rtp_peer, sizeof(s->rtp_peer));
    s->cseq++;
}

/* Process one RTP packet (PCMU). Returns 0 on success. */
static int process_rtp(sm_sip_t *s, const uint8_t *pkt, int len)
{
    int version, cc, has_ext, has_pad, pt;
    int hdr_len;
    const uint8_t *payload;
    int payload_len;
    int16_t pcm[SM_SIP_RTP_MAXPAY];
    uint8_t ulaw_out[SM_SIP_RTP_MAXPAY];
    int nsamples;
    int i;

    if (len < 12)
        return -1;
    version = pkt[0] >> 6;
    has_pad = (pkt[0] >> 5) & 1;
    has_ext = (pkt[0] >> 4) & 1;
    cc = pkt[0] & 0x0F;
    pt = pkt[1] & 0x7F;
    if (version != 2)
        return -1;
    hdr_len = 12 + cc * 4;
    if (has_ext)
    {
        if (len < hdr_len + 4)
            return -1;
        hdr_len += 4 + (pkt[hdr_len + 2] << 8 | pkt[hdr_len + 3]) * 4;
    }
    if (len < hdr_len)
        return -1;
    payload = pkt + hdr_len;
    payload_len = len - hdr_len;
    if (has_pad && payload_len > 0)
        payload_len -= payload[payload_len - 1];
    if (payload_len <= 0)
        return -1;
    if (pt != 0)
    {
        sm_log_message(&s->log, SM_LOG_DEBUG, "RTP payload type %d ignored", pt);
        return 0;
    }
    if (payload_len > SM_SIP_RTP_MAXPAY)
        payload_len = SM_SIP_RTP_MAXPAY;

    nsamples = payload_len;
    for (i = 0; i < nsamples; i++)
        pcm[i] = sm_ulaw_decode(payload[i]);

    /* Feed the daemon. */
    if (sm_ast_write_audio(&s->daemon_as, pcm, (size_t) nsamples) < 0)
    {
        sm_log_message(&s->log, SM_LOG_WARNING, "daemon write failed");
        if (s->rtp_peer_known)
            send_bye(s);
        end_call(s);
        return -1;
    }

    /* Read the daemon's reply (exactly one frame; the daemon emits one audio
       frame for each one it receives). */
    {
        uint8_t kind;
        uint8_t buf[SM_AS_MAX_PAYLOAD];
        size_t plen;
        int r;

        for (;;)
        {
            r = sm_ast_read(&s->daemon_as, &kind, buf, sizeof(buf), &plen, 200);
            if (r <= 0)
                break;
            if (kind == SM_AS_KIND_AUDIO && plen >= 2)
            {
                int ns = (int) plen / 2;
                int out_len = ns < SM_SIP_RTP_MAXPAY ? ns : SM_SIP_RTP_MAXPAY;
                uint8_t rtpbuf[12 + SM_SIP_RTP_MAXPAY];
                int j;

                for (j = 0; j < out_len; j++)
                    ulaw_out[j] = sm_ulaw_encode(((int16_t *) buf)[j]);
                rtpbuf[0] = 0x80;
                rtpbuf[1] = 0;              /* PT 0 PCMU */
                rtpbuf[2] = (uint8_t) (s->rtp_seq >> 8);
                rtpbuf[3] = (uint8_t) (s->rtp_seq & 0xFF);
                rtpbuf[4] = (uint8_t) (s->rtp_ts >> 24);
                rtpbuf[5] = (uint8_t) (s->rtp_ts >> 16);
                rtpbuf[6] = (uint8_t) (s->rtp_ts >> 8);
                rtpbuf[7] = (uint8_t) (s->rtp_ts & 0xFF);
                rtpbuf[8] = (uint8_t) (s->local_ssrc >> 24);
                rtpbuf[9] = (uint8_t) (s->local_ssrc >> 16);
                rtpbuf[10] = (uint8_t) (s->local_ssrc >> 8);
                rtpbuf[11] = (uint8_t) (s->local_ssrc & 0xFF);
                memcpy(rtpbuf + 12, ulaw_out, (size_t) out_len);
                sendto(s->rtp_fd, rtpbuf, (size_t) (12 + out_len), 0,
                       (struct sockaddr *) &s->rtp_peer, sizeof(s->rtp_peer));
                s->rtp_seq++;
                s->rtp_ts += (uint32_t) out_len;
                break;              /* one audio frame per received frame */
            }
            else if (kind == SM_AS_KIND_HANGUP)
            {
                sm_log_message(&s->log, SM_LOG_FLOW, "daemon asked to hang up");
                if (s->rtp_peer_known)
                    send_bye(s);
                end_call(s);
                return -1;
            }
        }
        if (r < 0)
        {
            sm_log_message(&s->log, SM_LOG_FLOW, "daemon closed connection");
            if (s->rtp_peer_known)
                send_bye(s);
            end_call(s);
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* SIP request handling                                                */
/* ------------------------------------------------------------------ */

static void handle_register(sm_sip_t *s, const sip_msg_t *m,
                            const struct sockaddr_in *src)
{
    char extra[512];

    sm_log_message(&s->log, SM_LOG_FLOW, "REGISTER from %s:%d user-agent='%s'",
                   inet_ntoa(src->sin_addr), ntohs(src->sin_port), m->user_agent);
    snprintf(extra, sizeof(extra),
             "Contact: %s;expires=3600\r\n"
             "Date: %s\r\n",
             m->contact[0] ? m->contact : "<sip:modem@unknown>", "Tue, 15 Sep 2026 00:00:00 GMT");
    send_response(s, m, src, 200, "OK", extra, NULL);
}

static void handle_options(sm_sip_t *s, const sip_msg_t *m,
                           const struct sockaddr_in *src)
{
    send_response(s, m, src, 200, "OK",
                  "Allow: INVITE, ACK, CANCEL, BYE, OPTIONS, REGISTER\r\n", NULL);
}

static void handle_invite(sm_sip_t *s, const sip_msg_t *m,
                          const struct sockaddr_in *src)
{
    char sdp_ip[64] = "";
    int sdp_port = 0;
    int sdp_pt = 0;
    char sdp[512];
    char extra[512];
    char body_tag[64];
    struct sockaddr_in rtp_addr;
    socklen_t rtp_addr_len = sizeof(rtp_addr);
    int rtp_port;
    uint8_t ok[SM_SIP_MAX_MSG];
    int ok_len;

    sm_log_message(&s->log, SM_LOG_FLOW, "INVITE %s from %s:%d", m->uri,
                   inet_ntoa(src->sin_addr), ntohs(src->sin_port));

    /* Retransmission of the current INVITE? Resend the 200 OK. */
    if (s->call_active && strcmp(s->call_id, m->call_id) == 0 && s->last_ok_len > 0)
    {
        send_raw(s, src, s->last_ok, s->last_ok_len);
        return;
    }

    if (s->call_active)
    {
        send_response(s, m, src, 486, "Busy Here", NULL, NULL);
        return;
    }

    extract_sdp(m, sdp_ip, sizeof(sdp_ip), &sdp_port, &sdp_pt);
    if (sdp_port <= 0)
    {
        send_response(s, m, src, 488, "Not Acceptable Here", NULL, NULL);
        return;
    }

    send_response(s, m, src, 100, "Trying", NULL, NULL);

    /* Create RTP socket. */
    s->rtp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s->rtp_fd < 0)
    {
        send_response(s, m, src, 500, "Server Internal Error", NULL, NULL);
        end_call(s);
        return;
    }
    memset(&rtp_addr, 0, sizeof(rtp_addr));
    rtp_addr.sin_family = AF_INET;
    rtp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rtp_addr.sin_port = 0;
    if (bind(s->rtp_fd, (struct sockaddr *) &rtp_addr, sizeof(rtp_addr)) < 0
        || getsockname(s->rtp_fd, (struct sockaddr *) &rtp_addr, &rtp_addr_len) < 0)
    {
        send_response(s, m, src, 500, "Server Internal Error", NULL, NULL);
        end_call(s);
        return;
    }
    rtp_port = ntohs(rtp_addr.sin_port);

    /* Connect to the softmodem daemon. */
    s->daemon_fd = daemon_connect(s);
    if (s->daemon_fd < 0)
    {
        sm_log_message(&s->log, SM_LOG_ERROR, "cannot connect to daemon %s:%d",
                       s->cfg.daemon_host, s->cfg.daemon_port);
        send_response(s, m, src, 503, "Service Unavailable", NULL, NULL);
        end_call(s);
        return;
    }

    /* Record call state. */
    snprintf(s->call_id, sizeof(s->call_id), "%s", m->call_id);
    snprintf(s->to_hdr, sizeof(s->to_hdr), "%s", m->to);
    snprintf(s->from_hdr, sizeof(s->from_hdr), "%s", m->from);
    s->cseq = m->cseq_num;
    s->local_ssrc = (uint32_t) rand() ^ ((uint32_t) rand() << 16);
    s->rtp_seq = (uint16_t) rand();
    s->rtp_ts = (uint32_t) rand();
    s->rtp_peer_known = 0;
    s->media_active = 0;
    s->call_active = 1;

    /* SDP answer. */
    snprintf(sdp, sizeof(sdp),
             "v=0\r\n"
             "o=- %u %u IN IP4 %s\r\n"
             "s=softmodem\r\n"
             "c=IN IP4 %s\r\n"
             "t=0 0\r\n"
             "m=audio %d RTP/AVP 0\r\n"
             "a=rtpmap:0 PCMU/8000\r\n"
             "a=ptime:20\r\n"
             "a=sendrecv\r\n",
             (unsigned) time(NULL), (unsigned) time(NULL),
             s->cfg.advertise_ip, s->cfg.advertise_ip, rtp_port);

    make_tag(body_tag, sizeof(body_tag));
    snprintf(extra, sizeof(extra),
             "Contact: <sip:modem@%s:%d>\r\n",
             s->cfg.advertise_ip, s->cfg.bind_port);
    ok_len = build_response(m, 200, "OK", extra, sdp, ok, sizeof(ok));
    if (ok_len > 0)
    {
        if (ok_len > (int) sizeof(s->last_ok))
            ok_len = (int) sizeof(s->last_ok);
        memcpy(s->last_ok, ok, (size_t) ok_len);
        s->last_ok_len = ok_len;
        send_raw(s, src, ok, ok_len);
    }

    sm_log_message(&s->log, SM_LOG_FLOW,
                   "call %s established: RTP peer %s:%d local port %d",
                   m->call_id, sdp_ip[0] ? sdp_ip : "?", sdp_port, rtp_port);
}

static void handle_ack(sm_sip_t *s, const sip_msg_t *m)
{
    if (s->call_active && strcmp(s->call_id, m->call_id) == 0)
    {
        s->media_active = 1;
        sm_log_message(&s->log, SM_LOG_FLOW, "ACK: media active");
    }
}

static void handle_bye(sm_sip_t *s, const sip_msg_t *m,
                       const struct sockaddr_in *src)
{
    sm_log_message(&s->log, SM_LOG_FLOW, "BYE for call %s", m->call_id);
    send_response(s, m, src, 200, "OK", NULL, NULL);
    if (s->call_active && strcmp(s->call_id, m->call_id) == 0)
        end_call(s);
}

static void handle_cancel(sm_sip_t *s, const sip_msg_t *m,
                          const struct sockaddr_in *src)
{
    send_response(s, m, src, 200, "OK", NULL, NULL);
    if (s->call_active && strcmp(s->call_id, m->call_id) == 0)
    {
        /* Send 487 for the INVITE too. */
        sip_msg_t inv = *m;
        snprintf(inv.cseq_method, sizeof(inv.cseq_method), "%s", "INVITE");
        send_response(s, &inv, src, 487, "Request Terminated", NULL, NULL);
        end_call(s);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

sm_sip_t *sm_sip_create(const sm_sip_config_t *cfg)
{
    sm_sip_t *s = calloc(1, sizeof(*s));
    struct sockaddr_in addr;
    int one = 1;

    if (!s)
        return NULL;
    s->cfg = *cfg;
    s->sip_fd = -1;
    s->rtp_fd = -1;
    s->daemon_fd = -1;
    sm_log_init(&s->log, cfg->log_level, "SIP", -1);

    s->sip_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s->sip_fd < 0)
    {
        free(s);
        return NULL;
    }
    setsockopt(s->sip_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t) cfg->bind_port);
    if (cfg->bind_addr[0] && strcmp(cfg->bind_addr, "*") != 0)
        inet_pton(AF_INET, cfg->bind_addr, &addr.sin_addr);
    else
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s->sip_fd, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        close(s->sip_fd);
        free(s);
        return NULL;
    }
    return s;
}

void sm_sip_destroy(sm_sip_t *s)
{
    if (!s)
        return;
    end_call(s);
    if (s->sip_fd >= 0)
        close(s->sip_fd);
    free(s);
}

int sm_sip_run(sm_sip_t *s, int max_calls)
{
    uint8_t buf[SM_SIP_MAX_MSG + 64];

    sm_log_message(&s->log, SM_LOG_FLOW, "listening on %s:%d (advertise %s), daemon %s:%d",
                   s->cfg.bind_addr, s->cfg.bind_port, s->cfg.advertise_ip,
                   s->cfg.daemon_host, s->cfg.daemon_port);

    while (max_calls < 0 || s->calls_done < max_calls)
    {
        struct pollfd pfds[2];
        int nfds = 0;
        int sip_idx = -1, rtp_idx = -1;
        int r;

        pfds[nfds].fd = s->sip_fd;
        pfds[nfds].events = POLLIN;
        pfds[nfds].revents = 0;
        sip_idx = nfds++;
        if (s->rtp_fd >= 0)
        {
            pfds[nfds].fd = s->rtp_fd;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            rtp_idx = nfds++;
        }
        (void) sip_idx;

        r = poll(pfds, (nfds_t) nfds, 500);
        if (r < 0)
        {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (r == 0)
            continue;

        if (s->rtp_fd >= 0 && (pfds[rtp_idx].revents & POLLIN))
        {
            uint8_t pkt[2048];
            struct sockaddr_in from;
            socklen_t fromlen = sizeof(from);
            int n = (int) recvfrom(s->rtp_fd, pkt, sizeof(pkt), 0,
                                   (struct sockaddr *) &from, &fromlen);
            if (n > 0 && s->call_active)
            {
                if (!s->rtp_peer_known)
                {
                    s->rtp_peer = from;
                    s->rtp_peer_known = 1;
                    sm_log_message(&s->log, SM_LOG_FLOW,
                                   "RTP peer is %s:%d",
                                   inet_ntoa(from.sin_addr), ntohs(from.sin_port));
                }
                process_rtp(s, pkt, n);
            }
        }
        else if (pfds[0].revents & POLLIN)
        {
            struct sockaddr_in src;
            socklen_t srclen = sizeof(src);
            int n = (int) recvfrom(s->sip_fd, buf, sizeof(buf), 0,
                                   (struct sockaddr *) &src, &srclen);
            if (n > 0)
            {
                sip_msg_t m;

                parse_sip(buf, n, &m);
                if (m.is_response)
                {
                    /* We only act as UAS; responses to our BYE are ignored. */
                    continue;
                }
                if (strcmp(m.method, "REGISTER") == 0)
                    handle_register(s, &m, &src);
                else if (strcmp(m.method, "INVITE") == 0)
                    handle_invite(s, &m, &src);
                else if (strcmp(m.method, "ACK") == 0)
                    handle_ack(s, &m);
                else if (strcmp(m.method, "BYE") == 0)
                    handle_bye(s, &m, &src);
                else if (strcmp(m.method, "CANCEL") == 0)
                    handle_cancel(s, &m, &src);
                else if (strcmp(m.method, "OPTIONS") == 0)
                    handle_options(s, &m, &src);
                else
                    send_response(s, &m, &src, 405, "Method Not Allowed", NULL, NULL);
            }
        }
    }
    return 0;
}
