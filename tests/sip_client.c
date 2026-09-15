/* SIP/RTP test caller: substitutes for the physical modem + PAP2T ATA.
 *
 *   [client pppd] <-> pty-shim <-> [software V.22bis CALLER modem]
 *                                        |
 *                        SIP/RTP (UDP) to sm_sip (port 5060)
 *                                        |
 *                                    [sm_sip]
 *                                        |
 *                              AudioSocket -> [sm_daemon] -> [server pppd]
 *
 * Proves: SIP signalling, RTP framing, mu-law coding, modem handshake and
 * real PPP carriage through the entire hardware-substitute chain.
 */
#include "v22bs_impl.h"
#include "ppp/sm_pppd.h"
#include "sip/sm_sip.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>

#define MAX_FBIT 65536
#define MAX_RBIT 65536
#define OBUF_MAX 4096

typedef struct {
    int woken;
    int sip_fd;
    int rtp_fd;
    v22bs_t *modem;
    int ppp_fd;

    struct sockaddr_in sip_server;
    struct sockaddr_in rtp_peer;
    int rtp_peer_known;

    char call_id[128];
    char tag[32];
    int cseq;
    char local_ip[64];
    int local_port;
    int rtp_local_port;
    uint32_t ssrc;
    uint16_t seq;
    uint32_t ts;

    uint8_t fbits[MAX_FBIT];
    int fn;
    uint8_t rbits[MAX_RBIT];
    int rn;

    int deframe_state, deframe_byte, deframe_nbits, deframe_prev;
    int framing_errors;
    uint8_t obuf[OBUF_MAX];
    int on;

    long long bits_consumed, bits_idle, bits_rx, bytes_read, bytes_deframed;
    long long rtp_sent, rtp_recv;
} sc_t;

static int sc_get_bit(void *ud)
{
    sc_t *c = ud;
    int bit = 1;

    if (c->fn > 0)
    {
        bit = c->fbits[0];
        memmove(c->fbits, c->fbits + 1, (size_t) (c->fn - 1));
        c->fn--;
        c->bits_consumed++;
    }
    else
        c->bits_idle++;
    return bit;
}

static void sc_put_bit(void *ud, int bit)
{
    sc_t *c = ud;

    c->bits_rx++;
    if (c->rn < MAX_RBIT)
        c->rbits[c->rn++] = (uint8_t) (bit & 1);
}

static void sc_async_decode(sc_t *c, int bit)
{
    switch (c->deframe_state)
    {
    case 0:
        if (c->deframe_prev == 1 && bit == 0)
        {
            c->deframe_state = 1;
            c->deframe_nbits = 0;
            c->deframe_byte = 0;
        }
        c->deframe_prev = bit;
        break;
    default:
        c->deframe_byte |= bit << c->deframe_nbits;
        c->deframe_nbits++;
        if (c->deframe_nbits >= 8)
            c->deframe_state = 2;
        break;
    case 2:
        c->deframe_prev = bit;
        if (bit == 1)
        {
            if (c->on < OBUF_MAX)
                c->obuf[c->on++] = (uint8_t) c->deframe_byte;
            c->bytes_deframed++;
        }
        else
            c->framing_errors++;
        c->deframe_state = 0;
        break;
    }
}

static unsigned my_rand(void)
{
    static unsigned x = 0x12345678;
    x = x * 1664525u + 1013904223u;
    return x;
}

static void sip_send(sc_t *c, const char *msg)
{
    sendto(c->sip_fd, msg, strlen(msg), 0,
           (struct sockaddr *) &c->sip_server, sizeof(c->sip_server));
}

static int build_sdp(char *buf, size_t len, const char *ip, int port)
{
    return snprintf(buf, len,
                    "v=0\r\n"
                    "o=- 1 1 IN IP4 %s\r\n"
                    "s=test\r\n"
                    "c=IN IP4 %s\r\n"
                    "t=0 0\r\n"
                    "m=audio %d RTP/AVP 0\r\n"
                    "a=rtpmap:0 PCMU/8000\r\n"
                    "a=ptime:20\r\n"
                    "a=sendrecv\r\n",
                    ip, ip, port);
}

static int do_register(sc_t *c)
{
    char msg[1024];
    char resp[4096];
    struct timeval tv = {5, 0};

    snprintf(msg, sizeof(msg),
             "REGISTER sip:modem@%s:%d SIP/2.0\r\n"
             "Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%08x\r\n"
             "From: <sip:modem@%s:%d>;tag=%s\r\n"
             "To: <sip:modem@%s:%d>\r\n"
             "Call-ID: %s\r\n"
             "CSeq: %d REGISTER\r\n"
             "Contact: <sip:modem@%s:%d>\r\n"
             "Max-Forwards: 70\r\n"
             "Expires: 3600\r\n"
             "Content-Length: 0\r\n\r\n",
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->local_ip, c->local_port, my_rand(),
             c->local_ip, c->local_port, c->tag,
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->call_id, c->cseq,
             c->local_ip, c->local_port);
    sip_send(c, msg);
    setsockopt(c->sip_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    {
        int n = (int) recv(c->sip_fd, resp, sizeof(resp) - 1, 0);
        if (n > 0)
        {
            resp[n] = 0;
            if (strstr(resp, "SIP/2.0 200"))
                return 0;
        }
    }
    return -1;
}

static int do_invite(sc_t *c)
{
    char body[512];
    char msg[2048];
    char resp[8192];
    struct timeval tv = {5, 0};
    int blen = build_sdp(body, sizeof(body), c->local_ip, c->rtp_local_port);
    int n;
    int got_200 = 0;

    snprintf(msg, sizeof(msg),
             "INVITE sip:modem@%s:%d SIP/2.0\r\n"
             "Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%08x\r\n"
             "From: <sip:caller@%s:%d>;tag=%s\r\n"
             "To: <sip:modem@%s:%d>\r\n"
             "Call-ID: %s\r\n"
             "CSeq: %d INVITE\r\n"
             "Contact: <sip:caller@%s:%d>\r\n"
             "Max-Forwards: 70\r\n"
             "Content-Type: application/sdp\r\n"
             "Content-Length: %d\r\n\r\n%s",
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->local_ip, c->local_port, my_rand(),
             c->local_ip, c->local_port, c->tag,
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->call_id, c->cseq,
             c->local_ip, c->local_port, blen, body);
    sip_send(c, msg);

    setsockopt(c->sip_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    while ((n = (int) recv(c->sip_fd, resp, sizeof(resp) - 1, 0)) > 0)
    {
        resp[n] = 0;
        if (strstr(resp, "SIP/2.0 100") || strstr(resp, "SIP/2.0 180"))
            continue;
        if (strstr(resp, "SIP/2.0 200"))
        {
            /* Parse the remote RTP endpoint from the SDP answer. */
            char *m = strstr(resp, "c=IN IP4 ");
            char *p = strstr(resp, "m=audio ");
            if (m && p)
            {
                char ip[64];
                int port;
                sscanf(m + 9, "%63s", ip);
                port = atoi(p + 8);
                memset(&c->rtp_peer, 0, sizeof(c->rtp_peer));
                c->rtp_peer.sin_family = AF_INET;
                c->rtp_peer.sin_port = htons((uint16_t) port);
                inet_pton(AF_INET, ip, &c->rtp_peer.sin_addr);
                c->rtp_peer_known = 1;
                printf("[sip] answer: RTP peer %s:%d\n", ip, port);
            }
            got_200 = 1;
            break;
        }
        if (strstr(resp, "SIP/2.0 4") || strstr(resp, "SIP/2.0 5"))
            return -1;
    }
    if (!got_200)
        return -1;

    /* ACK */
    c->cseq++;
    snprintf(msg, sizeof(msg),
             "ACK sip:modem@%s:%d SIP/2.0\r\n"
             "Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%08x\r\n"
             "From: <sip:caller@%s:%d>;tag=%s\r\n"
             "To: <sip:modem@%s:%d>\r\n"
             "Call-ID: %s\r\n"
             "CSeq: %d ACK\r\n"
             "Max-Forwards: 70\r\n"
             "Content-Length: 0\r\n\r\n",
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->local_ip, c->local_port, my_rand(),
             c->local_ip, c->local_port, c->tag,
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->call_id, c->cseq);
    sip_send(c, msg);
    return 0;
}

static void do_bye(sc_t *c)
{
    char msg[1024];

    c->cseq++;
    snprintf(msg, sizeof(msg),
             "BYE sip:modem@%s:%d SIP/2.0\r\n"
             "Via: SIP/2.0/UDP %s:%d;branch=z9hG4bK%08x\r\n"
             "From: <sip:caller@%s:%d>;tag=%s\r\n"
             "To: <sip:modem@%s:%d>\r\n"
             "Call-ID: %s\r\n"
             "CSeq: %d BYE\r\n"
             "Max-Forwards: 70\r\n"
             "Content-Length: 0\r\n\r\n",
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->local_ip, c->local_port, my_rand(),
             c->local_ip, c->local_port, c->tag,
             inet_ntoa(c->sip_server.sin_addr), ntohs(c->sip_server.sin_port),
             c->call_id, c->cseq);
    sip_send(c, msg);
}

static void send_rtp(sc_t *c, const int16_t *pcm, int nsamples)
{
    uint8_t pkt[12 + 160];
    int i;

    if (!c->rtp_peer_known || nsamples != 160)
        return;
    pkt[0] = 0x80;
    pkt[1] = 0;                     /* PCMU */
    pkt[2] = (uint8_t) (c->seq >> 8);
    pkt[3] = (uint8_t) (c->seq & 0xFF);
    pkt[4] = (uint8_t) (c->ts >> 24);
    pkt[5] = (uint8_t) (c->ts >> 16);
    pkt[6] = (uint8_t) (c->ts >> 8);
    pkt[7] = (uint8_t) (c->ts & 0xFF);
    pkt[8] = (uint8_t) (c->ssrc >> 24);
    pkt[9] = (uint8_t) (c->ssrc >> 16);
    pkt[10] = (uint8_t) (c->ssrc >> 8);
    pkt[11] = (uint8_t) (c->ssrc & 0xFF);
    for (i = 0; i < 160; i++)
        pkt[12 + i] = sm_ulaw_encode(pcm[i]);
    sendto(c->rtp_fd, pkt, 12 + 160, 0,
           (struct sockaddr *) &c->rtp_peer, sizeof(c->rtp_peer));
    c->seq++;
    c->ts += 160;
    c->rtp_sent++;
}

static int recv_rtp(sc_t *c, int16_t *pcm)
{
    uint8_t pkt[2048];
    int n;
    int i;

    n = (int) recv(c->rtp_fd, pkt, sizeof(pkt), 0);
    if (n < 12)
        return 0;
    if ((pkt[1] & 0x7F) != 0)
        return 0;
    {
        int cc = pkt[0] & 0x0F;
        int hdr = 12 + cc * 4;
        int pay = n - hdr;
        if (pay <= 0)
            return 0;
        if (pay > 160)
            pay = 160;
        for (i = 0; i < pay; i++)
            pcm[i] = sm_ulaw_decode(pkt[hdr + i]);
        c->rtp_recv++;
        return pay;
    }
}

static void usage(const char *p)
{
    fprintf(stderr, "usage: %s --sip HOST:PORT [--rate 2400|1200] [--local-ip IP]\n"
                    "          [--local-port N] [--duration S] [--peer-ip IP] [--no-ppp]\n",
            p);
}

int main(int argc, char **argv)
{
    sc_t cl;
    const char *sip = "127.0.0.1:5060";
    const char *local_ip = "127.0.0.1";
    const char *ppp_local_ip = "10.67.0.2";
    const char *peer_ip = "10.67.0.1";
    int rate = 2400;
    int duration_s = 40;
    int local_port = 5072;
    int no_ppp = 0;
    int rc = 1;
    int i;
    char *shim_exe = NULL;
    static char exe[512];
    {
        ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
        if (n > 0)
        {
            exe[n] = 0;
            shim_exe = exe;
        }
    }

    if (argc >= 3 && strcmp(argv[1], "--shim") == 0)
        return sm_pppd_shim_main(atoi(argv[2]));

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--sip") == 0 && i + 1 < argc)
            sip = argv[++i];
        else if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc)
            rate = atoi(argv[++i]);
        else if (strcmp(argv[i], "--local-ip") == 0 && i + 1 < argc)
            local_ip = argv[++i];
        else if (strcmp(argv[i], "--ppp-local") == 0 && i + 1 < argc)
            ppp_local_ip = argv[++i];
        else if (strcmp(argv[i], "--local-port") == 0 && i + 1 < argc)
            local_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc)
            duration_s = atoi(argv[++i]);
        else if (strcmp(argv[i], "--peer-ip") == 0 && i + 1 < argc)
            peer_ip = argv[++i];
        else if (strcmp(argv[i], "--no-ppp") == 0)
            no_ppp = 1;
        else
        {
            usage(argv[0]);
            return 2;
        }
    }

    memset(&cl, 0, sizeof(cl));
    cl.ppp_fd = -1;
    snprintf(cl.local_ip, sizeof(cl.local_ip), "%s", local_ip);
    cl.local_port = local_port;
    snprintf(cl.call_id, sizeof(cl.call_id), "%08x@%s", my_rand(), local_ip);
    snprintf(cl.tag, sizeof(cl.tag), "%08x", my_rand());

    {
        char host[64];
        int port;
        if (sscanf(sip, "%63[^:]:%d", host, &port) == 2)
        {
            memset(&cl.sip_server, 0, sizeof(cl.sip_server));
            cl.sip_server.sin_family = AF_INET;
            cl.sip_server.sin_port = htons((uint16_t) port);
            inet_pton(AF_INET, host, &cl.sip_server.sin_addr);
        }
    }
    if (cl.sip_server.sin_port == 0)
    {
        fprintf(stderr, "bad --sip address %s\n", sip);
        return 2;
    }

    cl.sip_fd = socket(AF_INET, SOCK_DGRAM, 0);
    cl.rtp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (cl.sip_fd < 0 || cl.rtp_fd < 0)
    {
        perror("socket");
        return 1;
    }
    {
        struct sockaddr_in a;
        memset(&a, 0, sizeof(a));
        a.sin_family = AF_INET;
        a.sin_port = htons((uint16_t) cl.local_port);
        inet_pton(AF_INET, cl.local_ip, &a.sin_addr);
        if (bind(cl.sip_fd, (struct sockaddr *) &a, sizeof(a)) < 0)
        {
            perror("bind sip");
            return 1;
        }
        memset(&a, 0, sizeof(a));
        a.sin_family = AF_INET;
        a.sin_port = 0;
        a.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(cl.rtp_fd, (struct sockaddr *) &a, sizeof(a)) < 0)
        {
            perror("bind rtp");
            return 1;
        }
        {
            socklen_t alen = sizeof(a);
            getsockname(cl.rtp_fd, (struct sockaddr *) &a, &alen);
            cl.rtp_local_port = ntohs(a.sin_port);
        }
    }

    cl.ssrc = my_rand();
    cl.seq = (uint16_t) my_rand();
    cl.ts = my_rand();
    cl.cseq = 1;

    if (do_register(&cl) == 0)
        printf("[sip] registered with %s\n", sip);
    else
        printf("[sip] register failed (continuing)\n");

    if (do_invite(&cl) < 0)
    {
        fprintf(stderr, "[sip] INVITE failed\n");
        return 1;
    }
    printf("[sip] call established\n");

    if (!no_ppp && shim_exe)
    {
        sm_pppd_config_t pc;
        pid_t pid;
        memset(&pc, 0, sizeof(pc));
        pc.pppd_path = "/usr/sbin/pppd";
        pc.shim_exe = shim_exe;
        pc.local_ip = ppp_local_ip;
        pc.peer_ip = peer_ip;
        pc.log_path = "/tmp/softmodem/pppd-sipclient.log";
        pid = sm_pppd_spawn(&pc, &cl.ppp_fd);
        if (pid > 0)
        {
            fcntl(cl.ppp_fd, F_SETFL, O_NONBLOCK);
            printf("[sip] client pppd pid=%d %s:%s\n", (int) pid, ppp_local_ip, peer_ip);
        }
    }

    cl.modem = v22bs_ours_create(1, rate, sc_get_bit, &cl, sc_put_bit, &cl, NULL, NULL);
    if (!cl.modem)
        return 1;

    fcntl(cl.rtp_fd, F_SETFL, O_NONBLOCK);
    fcntl(cl.sip_fd, F_SETFL, O_NONBLOCK);

    {
        time_t start = time(NULL);
        double next_tx = 0.0;

        while ((int) (time(NULL) - start) < duration_s)
        {
            /* pppd -> bits */
            if (cl.ppp_fd >= 0)
            {
                uint8_t tmp[64];
                ssize_t r = read(cl.ppp_fd, tmp, sizeof(tmp));
                if (r > 0)
                {
                    int k;
                    for (k = 0; k < r; k++)
                    {
                        uint8_t byte = tmp[k];
                        int b;
                        if (cl.fn + 10 > MAX_FBIT)
                            break;
                        cl.fbits[cl.fn++] = 0;
                        for (b = 0; b < 8; b++)
                            cl.fbits[cl.fn++] = (byte >> b) & 1;
                        cl.fbits[cl.fn++] = 1;
                        cl.bytes_read++;
                    }
                }
            }

            /* bits -> deframer -> pppd */
            {
                int guard = 0;
                while (cl.rn > 0 && guard < 1024)
                {
                    sc_async_decode(&cl, cl.rbits[0]);
                    memmove(cl.rbits, cl.rbits + 1, (size_t) (cl.rn - 1));
                    cl.rn--;
                    guard++;
                }
            }
            if (cl.on > 0 && cl.ppp_fd >= 0)
            {
                ssize_t w = write(cl.ppp_fd, cl.obuf, (size_t) cl.on);
                if (w > 0)
                {
                    memmove(cl.obuf, cl.obuf + w, (size_t) (cl.on - w));
                    cl.on -= (int) w;
                }
            }

            /* 20 ms cadence: TX one audio frame, drain RX. */
            {
                struct timespec now;
                double t;
                int16_t pcm[160];
                int n;

                clock_gettime(CLOCK_MONOTONIC, &now);
                t = (double) now.tv_sec + (double) now.tv_nsec / 1e9;
                if (t >= next_tx)
                {
                    n = cl.modem->tx(cl.modem->cb, pcm, 160);
                    if (n == 160)
                        send_rtp(&cl, pcm, n);
                    next_tx += 0.02;
                    if (next_tx < t)
                        next_tx = t + 0.02;
                }

                for (;;)
                {
                    int ns = recv_rtp(&cl, pcm);
                    if (ns <= 0)
                        break;
                    cl.modem->rx(cl.modem->cb, pcm, ns);
                }

                {
                    struct pollfd pfd = { cl.sip_fd, POLLIN, 0 };
                    if (poll(&pfd, 1, 0) > 0)
                    {
                        char buf[4096];
                        int r = (int) recv(cl.sip_fd, buf, sizeof(buf) - 1, 0);
                        if (r > 0)
                        {
                            buf[r] = 0;
                            if (strncmp(buf, "BYE", 3) == 0)
                            {
                                printf("[sip] remote BYE\n");
                                break;
                            }
                        }
                    }
                }

                /* Sleep until the next 1ms tick. */
                usleep(1000);
            }
        }
    }

    {
        int normal = cl.modem->normal(cl.modem->cb);
        int baud = cl.modem->bit_rate(cl.modem->cb);

        printf("[sip] done: modem=%s rate=%d rtp sent/recv=%lld/%lld"
               " bits rx=%lld consumed=%lld idle=%lld deframed=%lld framing_err=%d\n",
               normal ? "NORMAL" : "not-trained", baud,
               cl.rtp_sent, cl.rtp_recv,
               cl.bits_rx, cl.bits_consumed, cl.bits_idle,
               cl.bytes_deframed, cl.framing_errors);
        rc = normal ? 0 : 1;
    }

    do_bye(&cl);
    v22bs_destroy(cl.modem);
    if (cl.ppp_fd >= 0)
        close(cl.ppp_fd);
    close(cl.sip_fd);
    close(cl.rtp_fd);
    return rc;
}
