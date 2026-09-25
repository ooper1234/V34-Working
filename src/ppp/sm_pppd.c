#include "sm_pppd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>

#define SHIM_FD 45

/* Bytes either way the shim will hold while the far end is busy. */
#define SHIM_BUF 8192


pid_t sm_pppd_spawn(const sm_pppd_config_t *cfg, int *out_fd)
{
    int sv[2];
    pid_t pid;
    char shim_cmd[1024];
    char ip_spec[128];
    char *argv[32];
    int argc = 0;
    int lfd = -1;

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0)
        return -1;

    pid = fork();
    if (pid < 0)
    {
        close(sv[0]);
        close(sv[1]);
        return -1;
    }

    if (pid == 0)
    {
        /* Child: become pppd, with the socket end on a known fd. */
        close(sv[1]);
        if (dup2(sv[0], SHIM_FD) < 0)
            _exit(127);
        close(sv[0]);
        fcntl(SHIM_FD, F_SETFD, 0);

        if (cfg->log_path)
        {
            lfd = open(cfg->log_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
            if (lfd >= 0)
            {
                dup2(lfd, 1);
                dup2(lfd, 2);
                close(lfd);
            }
        }

        snprintf(shim_cmd, sizeof(shim_cmd), "%s --shim %d", cfg->shim_exe, SHIM_FD);
        snprintf(ip_spec, sizeof(ip_spec), "%s:%s", cfg->local_ip, cfg->peer_ip);

        argv[argc++] = (char *) cfg->pppd_path;
        argv[argc++] = (char *) "pty";
        argv[argc++] = shim_cmd;
        argv[argc++] = (char *) "noauth";
        argv[argc++] = (char *) "local";
        argv[argc++] = (char *) "nocrtscts";
        argv[argc++] = (char *) "nodetach";
        argv[argc++] = (char *) "noipdefault";
        argv[argc++] = (char *) "debug";
        /* pppd's default is 30 s of Configure-Requests, and on this line that
         * is not long enough: the far modem's data mode takes tens of seconds
         * to come up, and until it does the frames are damaged rather than
         * absent -- the 2026-09-26 00:30 call logged V.42 "frame RX BAD" for
         * every ConfReq it was sent and timed out with nothing received. The
         * client pppd is given 120 s for the same reason; this end should not
         * be the impatient one. */
        argv[argc++] = (char *) "lcp-max-configure";
        argv[argc++] = (char *) "120";
        /* No compression, either direction. It was negotiated and both ends
         * then went quiet: a deflate or VJ stream that arrives even slightly
         * wrong is discarded by the far pppd without a word, which hides the
         * damage instead of showing it. The 2026-09-26 calls put 192 CCP frames
         * and 13 VJ frames on the line and not one reply came back, while
         * uncompressed control frames crossed reliably. Plain IP frames also
         * make the transmit and receive dumps readable -- compressed ones hide
         * the payload behind 0xfd. */
        argv[argc++] = (char *) "noccp";
        argv[argc++] = (char *) "novj";
        if (cfg->auth)
        {
            /* "noauth" above is overridden; require the peer to authenticate. */
            argv[argc++] = (char *) "auth";
            argv[argc++] = (char *) "+pap";
            argv[argc++] = (char *) "-chap";
        }
        if (cfg->dns1 && cfg->dns1[0])
        {
            argv[argc++] = (char *) "ms-dns";
            argv[argc++] = (char *) cfg->dns1;
        }
        if (cfg->dns2 && cfg->dns2[0])
        {
            argv[argc++] = (char *) "ms-dns";
            argv[argc++] = (char *) cfg->dns2;
        }
        if (cfg->ip_up_script && cfg->ip_up_script[0])
        {
            argv[argc++] = (char *) "ip-up-script";
            argv[argc++] = (char *) cfg->ip_up_script;
        }
        if (cfg->ip_down_script && cfg->ip_down_script[0])
        {
            argv[argc++] = (char *) "ip-down-script";
            argv[argc++] = (char *) cfg->ip_down_script;
        }
        argv[argc++] = ip_spec;
        argv[argc++] = NULL;

        execv(cfg->pppd_path, argv);
        fprintf(stderr, "sm_pppd: execv %s failed: %s\n", cfg->pppd_path, strerror(errno));
        _exit(127);
    }

    /* Parent. */
    close(sv[0]);
    *out_fd = sv[1];
    return pid;
}

int sm_pppd_shim_main(int fd)
{
    struct pollfd pfds[4];
    /* One buffer each way. The relay used to read a chunk and write it out
     * blocking, which is a deadlock waiting for a busy line: pppd blocks
     * writing when the pty's input is full, this shim blocks writing when the
     * pty's output is full, and neither ever reads again. The modem's end then
     * sees a socket nobody is draining and drops the bytes it has correctly
     * decoded -- 22545 "ppp rx buffer full" messages on the 2026-09-25 V.34
     * call, with every one of them a frame pppd was waiting for. Buffering
     * each way and asking only for what can be taken turns that into
     * back-pressure the modem already handles.
     *
     * Static, not on the stack: this runs in a forked child of the daemon.
     */
    static struct
    {
        unsigned char data[SHIM_BUF];
        size_t len;
        size_t off;
    } to_pppd, to_modem;
    static unsigned char scratch[4096];

    unsigned long long to_pppd_bytes = 0, to_modem_bytes = 0, stalled = 0;
    unsigned long long to_pppd_shown = 0, to_modem_shown = 0;

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    fcntl(STDOUT_FILENO, F_SETFL, O_NONBLOCK);
    fcntl(fd, F_SETFL, O_NONBLOCK);

    for (;;)
    {
        int n;
        ssize_t r;

        /* Ask for more only where there is room, and for room only where
         * there is something to move. */
        pfds[0].fd = STDIN_FILENO;
        pfds[0].events = to_modem.len ? 0 : POLLIN;
        pfds[0].revents = 0;
        pfds[1].fd = fd;
        pfds[1].events = to_pppd.len ? 0 : POLLIN;
        pfds[1].revents = 0;
        pfds[2].fd = STDOUT_FILENO;
        pfds[2].events = to_pppd.len ? POLLOUT : 0;
        pfds[2].revents = 0;
        pfds[3].fd = fd;
        pfds[3].events = to_modem.len ? POLLOUT : 0;
        pfds[3].revents = 0;

        n = poll(pfds, 4, -1);
        if (n == 0 && ++stalled % 64 == 0)
            fprintf(stderr, "shim: stalled, to_pppd %zu, to_modem %zu\n",
                    to_pppd.len, to_modem.len);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            return 1;
        }

        /* pppd's output, on its way to the modem. */
        if (pfds[0].revents & (POLLIN | POLLHUP | POLLERR))
        {
            r = read(STDIN_FILENO, scratch, sizeof(scratch));
            if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
            {
                fprintf(stderr, "shim: pppd closed, to_pppd %llu, to_modem %llu\n",
                        to_pppd_bytes, to_modem_bytes);
                close(fd);
                return 0;
            }
            if (r > 0)
            {
                to_modem_bytes += (unsigned long long)r;
                if (to_modem.len + (size_t)r <= sizeof(to_modem.data))
                    memcpy(to_modem.data + to_modem.len, scratch, (size_t)r);
                to_modem.len += (size_t)r;
            }
        }
        if (to_modem.len && (pfds[3].revents & (POLLOUT | POLLERR | POLLHUP)))
        {
            r = write(fd, to_modem.data + to_modem.off, to_modem.len - to_modem.off);
            if (r > 0)
            {
                to_modem.off += (size_t)r;
                if (to_modem_bytes > (to_modem_shown + 4096))
                {
                    to_modem_shown = to_modem_bytes;
                    fprintf(stderr, "shim: pppd->modem %llu, modem->pppd %llu\n",
                            to_modem_bytes, to_pppd_bytes);
                }
            }
            if (to_modem.off == to_modem.len)
                to_modem.off = to_modem.len = 0;
        }

        /* The modem's output, on its way to pppd. */
        if (pfds[1].revents & (POLLIN | POLLHUP | POLLERR))
        {
            r = read(fd, scratch, sizeof(scratch));
            if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
            {
                fprintf(stderr, "shim: pppd closed, to_pppd %llu, to_modem %llu\n",
                        to_pppd_bytes, to_modem_bytes);
                close(fd);
                return 0;
            }
            if (r > 0)
            {
                to_pppd_bytes += (unsigned long long)r;
                if (to_pppd.len + (size_t)r <= sizeof(to_pppd.data))
                    memcpy(to_pppd.data + to_pppd.len, scratch, (size_t)r);
                to_pppd.len += (size_t)r;
            }
        }
        if (to_pppd.len && (pfds[2].revents & (POLLOUT | POLLERR | POLLHUP)))
        {
            r = write(STDOUT_FILENO, to_pppd.data + to_pppd.off, to_pppd.len - to_pppd.off);
            if (r > 0)
            {
                to_pppd.off += (size_t)r;
                if (to_pppd_bytes > (to_pppd_shown + 4096))
                {
                    to_pppd_shown = to_pppd_bytes;
                    fprintf(stderr, "shim: pppd->modem %llu, modem->pppd %llu\n",
                            to_modem_bytes, to_pppd_bytes);
                }
            }
            if (to_pppd.off == to_pppd.len)
                to_pppd.off = to_pppd.len = 0;
        }
    }
}
