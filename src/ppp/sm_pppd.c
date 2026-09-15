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

static void write_all_fd(int fd, const char *buf, int n)
{
    int off = 0;

    while (off < n)
    {
        int w = (int) write(fd, buf + off, (size_t) (n - off));
        if (w < 0)
        {
            if (errno == EINTR)
                continue;
            return;
        }
        off += w;
    }
}

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
    struct pollfd pfds[2];
    char buf[4096];

    for (;;)
    {
        int n, i;

        pfds[0].fd = 0;
        pfds[0].events = POLLIN;
        pfds[0].revents = 0;
        pfds[1].fd = fd;
        pfds[1].events = POLLIN;
        pfds[1].revents = 0;
        n = poll(pfds, 2, -1);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            return 1;
        }
        for (i = 0; i < 2; i++)
        {
            if (pfds[i].revents & (POLLIN | POLLHUP | POLLERR))
            {
                int out = (i == 0) ? fd : 1;
                int r = (int) read(pfds[i].fd, buf, sizeof(buf));
                if (r <= 0)
                {
                    close(fd);
                    return 0;
                }
                write_all_fd(out, buf, r);
            }
        }
    }
}
