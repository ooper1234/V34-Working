/* PPP over V.22bis bridge.
 *
 * Runs TWO pppd instances back-to-back through the software modem pair:
 *
 *   pppd_A  <-> pty_A (pppd creates it, "pty" option) <-> shim_A <-> sockA
 *                                                               |
 *                                                        bridge
 *            (modemA <-> channel <-> modemB)                  |
 *                                                        sockB
 *   pppd_B  <-> pty_B (pppd creates it, "pty" option) <-> shim_B <-'
 *
 * pppd is run with the `pty 'cmd'` option: pppd (running setuid-root) creates
 * a fresh pty master/slave pair and chowns the slave, then runs `cmd` with
 * the pty master on fd 0/1. The slave path is used directly and never
 * reopened by pppd, so the "EACCES on /dev/pts/N direct open" problem from
 * passing external ptys is avoided entirely.
 *
 * The `cmd` here is this same binary invoked as `pppbr --shim N`, where N is
 * the fd of one end of a socketpair. The shim relays bytes between the pty
 * master (pppd side) and the socket (bridge side). The bridge runs both
 * V.22bis modems and the telephone channel between the two socket pairs,
 * exactly the byte-transport logic proven in v22bis_pty.c.
 *
 * Data path (per direction): PPP async framing (done by pppd) -> pty ->
 * shim -> socket -> bridge async TX framing (start/stop bits) -> modem TX ->
 * channel -> modem RX -> async deframing -> socket -> shim -> pty -> pppd.
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
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/stat.h>

#define MAX_FBIT 65536
#define MAX_RBIT 65536
#define OBUF_MAX 64

#define SHIM_FD 45
#define IP_A "10.66.0.1"
#define IP_B "10.66.0.2"
#define PPPD "/usr/sbin/pppd"
#define LOGDIR "/tmp/pppbrlogs"

typedef struct {
    int sock_fd;                /* socket to shim for this side */
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

    /* bytes ready to write out to the peer shim */
    uint8_t obuf[OBUF_MAX];
    int on;
    long bits_consumed;
    long bits_idle;
    long bits_rx;
    long bytes_read;
    long bits_deframed;
    long bytes_deframed;
} ppp_side_t;

/* Async TX framing: start bit (0) + 8 data bits LSB-first + stop bit (1).
   Idle = continuous 1s. */
static void ppp_async_encode(ppp_side_t *p, const uint8_t *buf, int n)
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
static void ppp_async_decode(ppp_side_t *p, int bit)
{
    switch (p->deframe_state)
    {
    case 0:
        if (p->deframe_prev == 1 && bit == 0)
        {
            p->deframe_state = 1;
            p->deframe_nbits = 0;
            p->deframe_byte = 0;
        }
        p->deframe_prev = bit;
        break;
    default:
        p->deframe_byte |= bit << p->deframe_nbits;
        p->deframe_nbits++;
        if (p->deframe_nbits >= 8)
            p->deframe_state = 2;
        break;
    case 2:
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
static int ppp_get_bit(void *ud)
{
    ppp_side_t *p = ud;
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

static void ppp_put_bit(void *ud, int bit)
{
    ppp_side_t *p = ud;

    p->bits_rx++;
    if (p->rn < MAX_RBIT)
        p->rbits[p->rn++] = bit & 1;
}

static void write_all(int fd, const uint8_t *buf, int n)
{
    int off = 0;
    while (off < n)
    {
        int w = write(fd, buf + off, n - off);
        if (w < 0)
        {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN)
                break;
            return;
        }
        off += w;
    }
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "usage:\n"
            "  %s [--rate 2400|1200] [--duration SECS] [--ips LOCAL:REMOTE] [--raw]\n"
            "  %s --shim FD\n",
            prog, prog);
}

/* pppd 'pty' command: relay bytes between fd0/1 (pty master) and the socket. */
static int shim_main(int fd)
{
    struct pollfd pfds[2];
    char buf[4096];

    for (;;)
    {
        int n, i;
        pfds[0].fd = 0;  pfds[0].events = POLLIN;  pfds[0].revents = 0;
        pfds[1].fd = fd; pfds[1].events = POLLIN;  pfds[1].revents = 0;
        n = poll(pfds, 2, -1);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            return 1;
        }
        for (i = 0; i < 2; i++)
        {
            int out = (i == 0) ? fd : 1;
            if (pfds[i].revents & (POLLIN | POLLHUP))
            {
                int r = read(pfds[i].fd, buf, sizeof(buf));
                if (r <= 0)
                {
                    close(fd);
                    return 0;
                }
                r = write(out, buf, r);
                if (r < 0)
                {
                    close(fd);
                    return 1;
                }
            }
        }
    }
}

static void mkdirs(void)
{
    mkdir(LOGDIR, 0755);
}

static pid_t spawn_pppd(const char *shim_cmd, const char *ip_spec,
                        const char *log_name, int sockfd)
{
    pid_t pid;
    char logpath[256];
    int lfd;

    pid = fork();
    if (pid < 0)
    {
        perror("fork");
        return -1;
    }
    if (pid != 0)
        return pid;

    /* child */
    if (dup2(sockfd, SHIM_FD) < 0)
        _exit(127);
    if (fcntl(SHIM_FD, F_SETFD, 0) < 0)
        _exit(127);

    snprintf(logpath, sizeof(logpath), "%s/%s", LOGDIR, log_name);
    lfd = open(logpath, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (lfd >= 0)
    {
        dup2(lfd, 1);
        dup2(lfd, 2);
        close(lfd);
    }

    execl(PPPD, "pppd",
          "pty", shim_cmd,
          "noauth",
          "local",
          "nocrtscts",
          "nodetach",
          "noipdefault",
          "debug",
          ip_spec,
          NULL);
    perror("execl pppd");
    _exit(127);
}

/* Monitor for a (local,peer) pair becoming reachable; run ping tests. */
static int ping_monitor(const char *local_ip, const char *peer_ip, int timeout_s,
                        const char *outfile)
{
    char cmd[1024];
    int start, ok = 0;
    FILE *fp;

    start = (int)time(NULL);
    while ((int)time(NULL) - start < timeout_s)
    {
        /* check the peer is assigned and reachable */
        snprintf(cmd, sizeof(cmd),
                 "ip -4 addr show | grep -q '%s/'", peer_ip);
        if (system(cmd) == 0)
        {
            snprintf(cmd, sizeof(cmd),
                     "ping -c 1 -W 2 -I %s %s >/dev/null 2>&1",
                     local_ip, peer_ip);
            if (system(cmd) == 0)
            {
                snprintf(cmd, sizeof(cmd),
                         "ping -c 3 -W 2 -I %s %s", local_ip, peer_ip);
                fp = fopen(outfile, "w");
                if (fp)
                {
                    fclose(fp);
                    ok = system(cmd);
                }
                ok = (ok == 0);
                goto done;
            }
        }
        usleep(500000);
    }
done:
    if (ok)
        printf("[pppbr] PING OK %s -> %s\n", local_ip, peer_ip);
    else
        printf("[pppbr] PING FAILED %s -> %s\n", local_ip, peer_ip);
    return ok ? 0 : 1;
}

int main(int argc, char *argv[])
{
    int rate = 2400;
    int duration_s = 90;
    int sA[2], sB[2];
    pid_t pA = -1, pB = -1, pmon = -1;
    char exe[512], shimA[600], shimB[600];
    char ipA[160], ipB[160];
    char local[160], peer[160];
    int n_exe, ret = 0;
    int raw = 0;

    /* --shim mode */
    if (argc >= 3 && strcmp(argv[1], "--shim") == 0)
        return shim_main(atoi(argv[2]));

    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "--rate") && i + 1 < argc)
            rate = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--duration") && i + 1 < argc)
            duration_s = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ips") && i + 1 < argc)
        {
            snprintf(ipA, sizeof(ipA), "%s", argv[++i]);
        }
        else if (!strcmp(argv[i], "--raw"))
            raw = 1;
        else
        {
            usage(argv[0]);
            return 2;
        }
    }

    if (rate != 1200 && rate != 2400)
    {
        fprintf(stderr, "rate must be 1200 or 2400\n");
        return 2;
    }
    {
        char *colon;
        if (!ipA[0])
            snprintf(ipA, sizeof(ipA), IP_A ":" IP_B);
        /* ipA stays "LOCAL:REMOTE" for pppd A; ipB becomes "REMOTE:LOCAL" for pppd B */
        colon = strchr(ipA, ':');
        if (!colon)
        {
            fprintf(stderr, "bad --ips (want LOCAL:REMOTE)\n");
            return 2;
        }
        snprintf(local, sizeof(local), "%.*s", (int)(colon - ipA), ipA);
        snprintf(peer, sizeof(peer), "%s", colon + 1);
        snprintf(ipB, sizeof(ipB), "%s:%s", peer, local);
    }

    n_exe = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n_exe < 0)
    {
        perror("readlink");
        return 1;
    }
    exe[n_exe] = 0;

    snprintf(shimA, sizeof(shimA), "%s --shim %d", exe, SHIM_FD);
    snprintf(shimB, sizeof(shimB), "%s --shim %d", exe, SHIM_FD);

    printf("PPP-over-V22bis bridge: rate=%d, %ds, A=%s B=%s\n",
           rate, duration_s, ipA, ipB);

    mkdirs();
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sA) < 0 ||
        socketpair(AF_UNIX, SOCK_STREAM, 0, sB) < 0)
    {
        perror("socketpair");
        return 1;
    }

    pA = spawn_pppd(shimA, ipA, "pppd_A.log", sA[0]);
    pB = spawn_pppd(shimB, ipB, "pppd_B.log", sB[0]);
    if (pA < 0 || pB < 0)
        return 1;

    pmon = fork();
    if (pmon == 0)
    {
        /* monitor child: won't touch the sockets */
        char outfile[256];
        snprintf(outfile, sizeof(outfile), "%s/ping.txt", LOGDIR);
        _exit(ping_monitor(local, peer, duration_s - 5, outfile));
    }

    /* bridge parent */
    {
        ppp_side_t sA_m, sB_m;
        int elapsed_frames = 0;
        int total_frames = duration_s * 50;
        int a_open = 1, b_open = 1;

        fcntl(sA[1], F_SETFL, O_NONBLOCK);
        fcntl(sB[1], F_SETFL, O_NONBLOCK);
        close(sA[0]);
        close(sB[0]);

        if (raw)
        {
            /* bypass the modems: pure socket-to-socket relay, for isolation */
            elapsed_frames = 0;
            while (elapsed_frames < total_frames && (a_open || b_open))
            {
                struct pollfd pfds[2];
                int nfound, k;
                pfds[0].fd = sA[1]; pfds[0].events = POLLIN; pfds[0].revents = 0;
                pfds[1].fd = sB[1]; pfds[1].events = POLLIN; pfds[1].revents = 0;
                nfound = poll(pfds, 2, 100);
                for (k = 0; k < 2 && nfound > 0; k++)
                {
                    int src = (k == 0) ? sA[1] : sB[1];
                    int dst = (k == 0) ? sB[1] : sA[1];
                    int *open = (k == 0) ? &a_open : &b_open;
                    if (pfds[k].revents & (POLLIN | POLLHUP | POLLERR))
                    {
                        uint8_t tmp[512];
                        int r;
                        while ((r = read(src, tmp, sizeof(tmp))) > 0)
                            write_all(dst, tmp, r);
                        if (r == 0)
                            *open = 0;
                    }
                }
                elapsed_frames++;
            }
            fprintf(stderr, "[pppbr] raw relay done (%d polls)\n", elapsed_frames);
            goto out_cleanup;
        }

        memset(&sA_m, 0, sizeof(sA_m));
        memset(&sB_m, 0, sizeof(sB_m));
        sA_m.sock_fd = sA[1];
        sB_m.sock_fd = sB[1];

        v22bs_channel_init(&sA_m.ch, 0x1111);
        v22bs_channel_init(&sB_m.ch, 0x2222);
        sA_m.ch.attenuation = 1.0;
        sB_m.ch.attenuation = 1.0;

        sA_m.modem = v22bs_ours_create(1, rate, ppp_get_bit, &sA_m,
                                       ppp_put_bit, &sA_m, NULL, NULL);
        sB_m.modem = v22bs_ours_create(0, rate, ppp_get_bit, &sB_m,
                                       ppp_put_bit, &sB_m, NULL, NULL);
        if (!sA_m.modem || !sB_m.modem)
        {
            fprintf(stderr, "modem create failed\n");
            return 1;
        }

        while (elapsed_frames < total_frames && (a_open || b_open))
        {
            struct pollfd pfds[2];
            int nfound, k;

            pfds[0].fd = sA_m.sock_fd; pfds[0].events = POLLIN; pfds[0].revents = 0;
            pfds[1].fd = sB_m.sock_fd; pfds[1].events = POLLIN; pfds[1].revents = 0;
            nfound = poll(pfds, 2, 20);
            for (k = 0; k < 2 && nfound > 0; k++)
            {
                ppp_side_t *p = (k == 0) ? &sA_m : &sB_m;
                if (pfds[k].revents & (POLLIN | POLLHUP | POLLERR))
                {
                    uint8_t tmp[64];
                    int nr;
                    for (;;)
                    {
                        nr = read(p->sock_fd, tmp, sizeof(tmp));
                        if (nr > 0)
                        {
                            ppp_async_encode(p, tmp, nr);
                            p->bytes_read += nr;
                        }
                        else
                        {
                            if (nr == 0)
                            {
                                if (k == 0)
                                    a_open = 0;
                                else
                                    b_open = 0;
                            }
                            break;
                        }
                    }
                }
            }

/* audio frame: caller TX (160 smp) -> ch A -> answerer RX,
                answerer TX (160 smp) -> ch B -> caller RX */
            {
                int16_t oa[160], ob[160];
                int16_t ba[160], bb[160];
                int na, nb;

                na = sA_m.modem->tx(sA_m.modem->cb, oa, 160);
                nb = sB_m.modem->tx(sB_m.modem->cb, ob, 160);

                na = v22bs_channel_apply(&sA_m.ch, oa, na, ba, 160);
                nb = v22bs_channel_apply(&sB_m.ch, ob, nb, bb, 160);
                if (na > 0)
                    sB_m.modem->rx(sB_m.modem->cb, ba, na);
                if (nb > 0)
                    sA_m.modem->rx(sA_m.modem->cb, bb, nb);
            }

            /* deframe a bounded number of RX bits per frame per direction */
            for (k = 0; k < 2; k++)
            {
                ppp_side_t *p = (k == 0) ? &sA_m : &sB_m;
                int c = 0;
                while (p->rn > 0 && c < 64)
                {
                    ppp_async_decode(p, p->rbits[0]);
                    memmove(p->rbits, p->rbits + 1, p->rn - 1);
                    p->rn--;
                    c++;
                    p->bits_deframed++;
                }
            }

            /* flush de-framed bytes to the shim of the modem that received them:
               sA's RX (peer B's data) -> pppd A, sB's RX -> pppd B */
            if (sA_m.on > 0 && a_open)
            {
                int nw = write(sA_m.sock_fd, sA_m.obuf, sA_m.on);
                if (nw > 0)
                {
                    memmove(sA_m.obuf, sA_m.obuf + nw, sA_m.on - nw);
                    sA_m.on -= nw;
                }
            }
            if (sB_m.on > 0 && b_open)
            {
                int nw = write(sB_m.sock_fd, sB_m.obuf, sB_m.on);
                if (nw > 0)
                {
                    memmove(sB_m.obuf, sB_m.obuf + nw, sB_m.on - nw);
                    sB_m.on -= nw;
                }
            }

            elapsed_frames++;
        }

        fprintf(stderr, "[pppbr] done: %d frames A{rd=%ld cons=%ld idle=%ld rx=%ld defr=%ld by=%ld fe=%d} B{rd=%ld cons=%ld idle=%ld rx=%ld defr=%ld by=%ld fe=%d}\n",
                elapsed_frames,
                sA_m.bytes_read, sA_m.bits_consumed, sA_m.bits_idle, sA_m.bits_rx, sA_m.bits_deframed, sA_m.bytes_deframed, sA_m.framing_errors,
                sB_m.bytes_read, sB_m.bits_consumed, sB_m.bits_idle, sB_m.bits_rx, sB_m.bits_deframed, sB_m.bytes_deframed, sB_m.framing_errors);

        if (sA_m.modem->normal(sA_m.modem->cb))
            fprintf(stderr, "[pppbr] A modem: NORMAL_OPERATION\n");
        if (sB_m.modem->normal(sB_m.modem->cb))
            fprintf(stderr, "[pppbr] B modem: NORMAL_OPERATION\n");

        v22bs_destroy(sA_m.modem);
        v22bs_destroy(sB_m.modem);
    }

out_cleanup:
    /* wait for monitor result then stop pppd */
    if (pmon > 0)
    {
        int st;
        waitpid(pmon, &st, 0);
        ret = WIFEXITED(st) ? WEXITSTATUS(st) : 1;
    }
    if (pA > 0)
        kill(pA, SIGTERM);
    if (pB > 0)
        kill(pB, SIGTERM);
    if (pA > 0)
        waitpid(pA, NULL, 0);
    if (pB > 0)
        waitpid(pB, NULL, 0);

    printf("[pppbr] exit=%d\n", ret);
    return ret;
}