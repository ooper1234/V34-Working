#ifndef SM_PPPD_H
#define SM_PPPD_H

#include <sys/types.h>

/* Spawns a real pppd attached to a fresh pty, using pppd's `pty` option and
   our own binary as the relay shim. Returns the pppd pid (>0) and stores the
   daemon-side socket fd in *out_fd. On failure returns -1. */
typedef struct {
    const char *pppd_path;        /* e.g. /usr/sbin/pppd */
    const char *shim_exe;         /* our own executable path */
    const char *local_ip;         /* address pppd assigns to this host */
    const char *peer_ip;          /* address pppd assigns to the client */
    const char *dns1;             /* primary DNS to hand out (may be NULL) */
    const char *dns2;             /* secondary DNS (may be NULL) */
    const char *log_path;         /* per-call pppd log (may be NULL) */
    const char *ip_up_script;     /* optional ip-up script */
    const char *ip_down_script;   /* optional ip-down script */
    int auth;                     /* 1 = require peer PAP/CHAP auth */
} sm_pppd_config_t;

pid_t sm_pppd_spawn(const sm_pppd_config_t *cfg, int *out_fd);

/* Relay shim main loop: copies bytes between fd 0/1 (pty master) and the
   socket fd. Returns exit status. */
int sm_pppd_shim_main(int fd);

#endif
