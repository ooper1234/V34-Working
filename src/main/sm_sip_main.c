#include "sip/sm_sip.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

static const char *prog = "sm_sip";

static void usage(void)
{
    fprintf(stderr,
            "softmodem SIP/RTP bridge (PAP2T ATA <-> AudioSocket daemon)\n"
            "usage: %s [options]\n"
            "  --bind ADDR[:PORT]    SIP listen address (default 0.0.0.0:5060)\n"
            "  --advertise IP        IP advertised in SDP/Contact (default 127.0.0.1)\n"
            "  --daemon HOST:PORT    sm_daemon AudioSocket address (default 127.0.0.1:9092)\n"
            "  --once                handle one call then exit (testing)\n"
            "  --debug               verbose logging\n"
            "  --help                this text\n",
            prog);
}

int main(int argc, char **argv)
{
    sm_sip_config_t cfg;
    sm_sip_t *sip;
    int once = 0;
    int i;

    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.bind_addr, sizeof(cfg.bind_addr), "0.0.0.0");
    snprintf(cfg.advertise_ip, sizeof(cfg.advertise_ip), "127.0.0.1");
    snprintf(cfg.daemon_host, sizeof(cfg.daemon_host), "127.0.0.1");
    cfg.bind_port = 5060;
    cfg.daemon_port = 9092;
    cfg.log_level = SM_LOG_INFO;

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--bind") == 0 && i + 1 < argc)
        {
            char *colon = strrchr(argv[++i], ':');
            if (colon)
            {
                size_t n = (size_t) (colon - argv[i]);
                if (n >= sizeof(cfg.bind_addr))
                    n = sizeof(cfg.bind_addr) - 1;
                memcpy(cfg.bind_addr, argv[i], n);
                cfg.bind_addr[n] = 0;
                cfg.bind_port = atoi(colon + 1);
            }
            else
            {
                snprintf(cfg.bind_addr, sizeof(cfg.bind_addr), "%s", argv[i]);
            }
        }
        else if (strcmp(argv[i], "--advertise") == 0 && i + 1 < argc)
            snprintf(cfg.advertise_ip, sizeof(cfg.advertise_ip), "%s", argv[++i]);
        else if (strcmp(argv[i], "--daemon") == 0 && i + 1 < argc)
        {
            char *colon = strrchr(argv[++i], ':');
            if (colon)
            {
                size_t n = (size_t) (colon - argv[i]);
                if (n >= sizeof(cfg.daemon_host))
                    n = sizeof(cfg.daemon_host) - 1;
                memcpy(cfg.daemon_host, argv[i], n);
                cfg.daemon_host[n] = 0;
                cfg.daemon_port = atoi(colon + 1);
            }
            else
                snprintf(cfg.daemon_host, sizeof(cfg.daemon_host), "%s", argv[i]);
        }
        else if (strcmp(argv[i], "--once") == 0)
            once = 1;
        else if (strcmp(argv[i], "--capture") == 0 && i + 1 < argc)
            cfg.capture_dir = argv[++i];
        else if (strcmp(argv[i], "--debug") == 0)
            cfg.log_level = SM_LOG_FLOW;
        else if (strcmp(argv[i], "--help") == 0)
        {
            usage();
            return 0;
        }
        else
        {
            fprintf(stderr, "%s: unknown option '%s'\n", prog, argv[i]);
            usage();
            return 2;
        }
    }

    srand((unsigned) time(NULL) ^ (unsigned) getpid());
    signal(SIGPIPE, SIG_IGN);

    sip = sm_sip_create(&cfg);
    if (!sip)
    {
        fprintf(stderr, "%s: cannot create SIP server on %s:%d: %s\n",
                prog, cfg.bind_addr, cfg.bind_port, strerror(errno));
        return 1;
    }

    fprintf(stderr, "[%s] SIP on %s:%d, advertise %s, daemon %s:%d\n",
            prog, cfg.bind_addr, cfg.bind_port, cfg.advertise_ip,
            cfg.daemon_host, cfg.daemon_port);

    sm_sip_run(sip, once ? 1 : -1);
    sm_sip_destroy(sip);
    return 0;
}
