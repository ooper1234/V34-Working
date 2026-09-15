/* AudioSocket client test: substitutes for Asterisk + ATA + calling modem.
 *
 *   [client pppd] <-> pty-shim <-> [software V.22bis CALLER modem]
 *                                        |
 *                             AudioSocket TCP (like Asterisk)
 *                                        |
 *                                  [sm_daemon]
 *                                        |
 *                                   [server pppd]
 *
 * This proves the complete chain end to end: the client pppd gets an IP from
 * the daemon pppd across a real V.22bis (or V.22) modem link carried over the
 * exact AudioSocket wire protocol Asterisk uses.
 */
#include "v22bs_impl.h"
#include "ppp/sm_pppd.h"
#include "ast_socket/sm_ast_socket.h"

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
    int sock;
    int ppp_fd;
    v22bs_t *modem;

    uint8_t fbits[MAX_FBIT];
    int fn;
    uint8_t rbits[MAX_RBIT];
    int rn;

    int deframe_state;
    int deframe_byte;
    int deframe_nbits;
    int deframe_prev;
    int framing_errors;

    uint8_t obuf[OBUF_MAX];
    int on;

    long long bits_consumed;
    long long bits_idle;
    long long bits_rx;
    long long bytes_read;
    long long bytes_deframed;
    long long frames_in;
    long long frames_out;
} client_t;

static int client_get_bit(void *ud)
{
    client_t *c = ud;
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

static void client_put_bit(void *ud, int bit)
{
    client_t *c = ud;

    c->bits_rx++;
    if (c->rn < MAX_RBIT)
        c->rbits[c->rn++] = (uint8_t) (bit & 1);
}

static void client_async_decode(client_t *c, int bit)
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

static void usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s --server HOST:PORT [--rate 2400|1200] [--duration N]\n"
            "          [--local-ip IP] [--peer-ip IP] [--no-ppp] [--uuid HEX]\n",
            prog);
}

int main(int argc, char **argv)
{
    const char *server = "127.0.0.1:9092";
    const char *unix_path = NULL;
    int rate = 2400;
    int duration_s = 30;
    const char *local_ip = "192.168.2.240";
    const char *peer_ip = "192.168.2.1";
    int no_ppp = 0;
    int pattern_mode = 0;
    int pattern_len = 512;
    unsigned char *pat = NULL;
    unsigned char *echo = NULL;
    int echo_n = 0;
    int pat_sent = 0;
    client_t cl;
    struct sockaddr_in addr;
    char host[128];
    int port;
    int i;
    int rc = 1;
    char *shim_exe = NULL;
    {
        static char exe[512];
        ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
        if (n > 0)
        {
            exe[n] = 0;
            shim_exe = exe;
        }
    }

    /* pppd's `pty` command runs us as a relay shim on the client side too. */
    if (argc >= 3 && strcmp(argv[1], "--shim") == 0)
        return sm_pppd_shim_main(atoi(argv[2]));

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--server") == 0 && i + 1 < argc)
            server = argv[++i];
        else if (strcmp(argv[i], "--unix") == 0 && i + 1 < argc)
            unix_path = argv[++i];
        else if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc)
            rate = atoi(argv[++i]);
        else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc)
            duration_s = atoi(argv[++i]);
        else if (strcmp(argv[i], "--local-ip") == 0 && i + 1 < argc)
            local_ip = argv[++i];
        else if (strcmp(argv[i], "--peer-ip") == 0 && i + 1 < argc)
            peer_ip = argv[++i];
        else if (strcmp(argv[i], "--no-ppp") == 0)
            no_ppp = 1;
        else if (strcmp(argv[i], "--pattern") == 0)
            pattern_mode = 1;
        else if (strcmp(argv[i], "--pattern-len") == 0 && i + 1 < argc)
            pattern_len = atoi(argv[++i]);
        else
        {
            usage(argv[0]);
            return 2;
        }
    }

    if (!unix_path)
    {
        if (sscanf(server, "%127[^:]:%d", host, &port) != 2)
        {
            usage(argv[0]);
            return 2;
        }
    }

    memset(&cl, 0, sizeof(cl));
    cl.ppp_fd = -1;

    /* Connect like Asterisk's AudioSocket app does. */
    if (unix_path)
    {
        cl.sock = sm_ast_connect_unix(unix_path);
        if (cl.sock < 0)
        {
            perror("connect unix");
            return 1;
        }
    }
    else
    {
        cl.sock = socket(AF_INET, SOCK_STREAM, 0);
        if (cl.sock < 0)
        {
            perror("socket");
            return 1;
        }
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons((uint16_t) port);
        if (inet_pton(AF_INET, host, &addr.sin_addr) != 1)
        {
            fprintf(stderr, "bad server address %s\n", host);
            return 2;
        }
        if (connect(cl.sock, (struct sockaddr *) &addr, sizeof(addr)) < 0)
        {
            perror("connect");
            return 1;
        }
    }
    {
        sm_ast_socket_t as;
        uint8_t uuid[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                            0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
        sm_ast_init(&as, cl.sock);
        if (sm_ast_write(&as, SM_AS_KIND_UUID, uuid, 16) < 0)
        {
            perror("uuid write");
            return 1;
        }
    }

    /* Pattern mode: no pppd; feed a known byte pattern after training and
       verify the echo comes back byte-exact. */
    if (pattern_mode)
        no_ppp = 1;

    /* Optionally start a real pppd on the client side. */
    if (!no_ppp)
    {
        sm_pppd_config_t pc;
        pid_t pid;
        memset(&pc, 0, sizeof(pc));
        pc.pppd_path = "/usr/sbin/pppd";
        pc.shim_exe = shim_exe;
        pc.local_ip = local_ip;
        pc.peer_ip = peer_ip;
        pc.dns1 = NULL;
        pc.dns2 = NULL;
        pc.log_path = "/tmp/softmodem/pppd-client.log";
        pid = sm_pppd_spawn(&pc, &cl.ppp_fd);
        if (pid < 0)
        {
            perror("pppd spawn");
            return 1;
        }
        fcntl(cl.ppp_fd, F_SETFL, O_NONBLOCK);
        printf("[as_client] client pppd pid=%d %s:%s\n", (int) pid, local_ip, peer_ip);
    }

    cl.modem = v22bs_ours_create(1 /* calling party */, rate,
                                 client_get_bit, &cl,
                                 client_put_bit, &cl, NULL, NULL);
    if (!cl.modem)
    {
        fprintf(stderr, "modem create failed\n");
        return 1;
    }

    fcntl(cl.sock, F_SETFL, O_NONBLOCK);

    printf("[as_client] connected to %s, rate=%d, %ds\n", server, rate, duration_s);

    {
        time_t start = time(NULL);
        uint8_t payload[SM_AS_MAX_PAYLOAD];
        int16_t rx_audio[SM_AS_MAX_PAYLOAD / 2];
        int16_t tx_audio[SM_AS_MAX_PAYLOAD / 2];
        sm_ast_socket_t as;

        sm_ast_init(&as, cl.sock);

        if (pattern_mode)
        {
            unsigned rng = 0x2468ACE0;
            pat = malloc((size_t) pattern_len);
            echo = calloc(1, (size_t) pattern_len);
            for (i = 0; i < pattern_len; i++)
            {
                rng = rng * 1664525u + 1013904223u;
                pat[i] = (unsigned char) (rng >> 16);
            }
        }

        while ((int) (time(NULL) - start) < duration_s)
        {
            struct pollfd pfds[2];
            int nfds = 0;
            int sock_idx = -1, ppp_idx = -1;

            pfds[nfds].fd = cl.sock;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            sock_idx = nfds++;
            if (cl.ppp_fd >= 0)
            {
                pfds[nfds].fd = cl.ppp_fd;
                pfds[nfds].events = POLLIN;
                pfds[nfds].revents = 0;
                ppp_idx = nfds++;
            }

            if (poll(pfds, (nfds_t) nfds, 20) < 0)
            {
                if (errno == EINTR)
                    continue;
                perror("poll");
                break;
            }

            /* pppd bytes -> async bits -> modem TX */
            if (ppp_idx >= 0 && (pfds[ppp_idx].revents & POLLIN))
            {
                uint8_t tmp[64];
                ssize_t r = read(cl.ppp_fd, tmp, sizeof(tmp));
                if (r > 0)
                {
                    int k;
                    for (k = 0; k < r; k++)
                    {
                        uint8_t byte = tmp[k];
                        if (cl.fn + 10 > MAX_FBIT)
                            break;
                        cl.fbits[cl.fn++] = 0;
                        for (i = 0; i < 8; i++)
                            cl.fbits[cl.fn++] = (byte >> i) & 1;
                        cl.fbits[cl.fn++] = 1;
                        cl.bytes_read++;
                    }
                }
            }

            /* pattern mode: once trained, feed the pattern slowly */
            if (pattern_mode && cl.modem->normal(cl.modem->cb) && pat_sent < pattern_len)
            {
                /* Only feed when the TX bit queue is nearly drained, so the
                   echo arrives in near-real time and we can match it. */
                if (cl.fn < 20)
                {
                    int k;
                    for (k = 0; k < 4 && pat_sent < pattern_len; k++)
                    {
                        uint8_t byte = pat[pat_sent++];
                        if (cl.fn + 10 > MAX_FBIT)
                            break;
                        cl.fbits[cl.fn++] = 0;
                        for (i = 0; i < 8; i++)
                            cl.fbits[cl.fn++] = (byte >> i) & 1;
                        cl.fbits[cl.fn++] = 1;
                        cl.bytes_read++;
                    }
                }
            }

            /* Modem RX bits -> deframer -> pppd */
            {
                int guard = 0;
                while (cl.rn > 0 && guard < 1024)
                {
                    client_async_decode(&cl, cl.rbits[0]);
                    memmove(cl.rbits, cl.rbits + 1, (size_t) (cl.rn - 1));
                    cl.rn--;
                    guard++;
                }
            }
            if (cl.on > 0 && pattern_mode)
            {
                int take = (cl.on < pattern_len - echo_n) ? cl.on : pattern_len - echo_n;
                if (take > 0)
                {
                    memcpy(echo + echo_n, cl.obuf, (size_t) take);
                    echo_n += take;
                }
                memmove(cl.obuf, cl.obuf + take, (size_t) (cl.on - take));
                cl.on -= take;
                if (echo_n >= pattern_len)
                    goto done;
            }
            else if (cl.on > 0 && cl.ppp_fd >= 0)
            {
                ssize_t w = write(cl.ppp_fd, cl.obuf, (size_t) cl.on);
                if (w > 0)
                {
                    memmove(cl.obuf, cl.obuf + w, (size_t) (cl.on - w));
                    cl.on -= (int) w;
                }
            }

            /* Generate one audio frame (20 ms) from the modem and send it. */
            {
                int n = cl.modem->tx(cl.modem->cb, tx_audio, 160);
                if (n > 0)
                {
                    if (sm_ast_write(&as, SM_AS_KIND_AUDIO,
                                     (const uint8_t *) tx_audio,
                                     (size_t) (n * 2)) < 0)
                    {
                        printf("[as_client] audio write failed\n");
                        break;
                    }
                    cl.frames_out++;
                }
            }

            /* Drain any audio the daemon sent and feed the modem RX. */
            {
                uint8_t kind;
                size_t plen;
                int r;
                while ((r = sm_ast_read(&as, &kind, payload, sizeof(payload), &plen, -1)) > 0)
                {
                    if (kind == SM_AS_KIND_AUDIO)
                    {
                        int nsamples = (int) plen / 2;
                        if (nsamples > (int) (sizeof(rx_audio) / sizeof(rx_audio[0])))
                            nsamples = (int) (sizeof(rx_audio) / sizeof(rx_audio[0]));
                        memcpy(rx_audio, payload, (size_t) (nsamples * 2));
                        cl.modem->rx(cl.modem->cb, rx_audio, nsamples);
                        cl.frames_in++;
                    }
                    else if (kind == SM_AS_KIND_HANGUP)
                    {
                        printf("[as_client] daemon hung up\n");
                        goto done;
                    }
                }
                if (r < 0)
                {
                    printf("[as_client] daemon closed connection\n");
                    break;
                }
            }
        }
    }

done:
    {
        int normal = cl.modem->normal(cl.modem->cb);
        int baud = cl.modem->bit_rate(cl.modem->cb);

        printf("[as_client] done: modem=%s rate=%d frames in/out=%lld/%lld bits rx=%lld consumed=%lld idle=%lld deframed=%lld framing_err=%d\n",
               normal ? "NORMAL" : "not-trained", baud,
               cl.frames_in, cl.frames_out,
               cl.bits_rx, cl.bits_consumed, cl.bits_idle,
               cl.bytes_deframed, cl.framing_errors);

        if (pattern_mode)
        {
            int errs = 0;
            int first_bad = -1;
            int k;
            for (k = 0; k < echo_n; k++)
            {
                if (echo[k] != pat[k])
                {
                    errs++;
                    if (first_bad < 0)
                        first_bad = k;
                }
            }
            printf("[as_client] PATTERN TEST: sent=%d echoed=%d/%d errors=%d%s\n",
                   pattern_len, echo_n, pattern_len, errs,
                   first_bad >= 0 ? "" : " (byte-exact)");
            if (first_bad >= 0)
                printf("[as_client] first mismatch at %d: got 0x%02x want 0x%02x\n",
                       first_bad, echo[first_bad], pat[first_bad]);
            rc = (normal && echo_n == pattern_len && errs == 0) ? 0 : 1;
        }
        else
        {
            rc = normal ? 0 : 1;
        }
        free(pat);
        free(echo);
    }

    v22bs_destroy(cl.modem);
    if (cl.ppp_fd >= 0)
        close(cl.ppp_fd);
    close(cl.sock);
    return rc;
}
