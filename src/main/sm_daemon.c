#include "call/sm_call.h"
#include "ast_socket/sm_ast_socket.h"
#include "ppp/sm_pppd.h"
#include "logging/sm_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>

#define DEFAULT_PORT 9092
#define MAX_CALLS 32

static const char *prog = "sm_daemon";

static void usage(void)
{
    fprintf(stderr,
            "softmodem AudioSocket answerer daemon\n"
            "usage: %s [options]\n"
            "  --listen ADDR        bind address (default 0.0.0.0)\n"
            "  --port N             TCP port (default %d)\n"
            "  --rate 1200|2400     max modem rate (default 2400)\n"
            "  --tone-ms N          answer tone duration (default 3300, 0=off)\n"
            "  --pre-silence-ms N   silence before answer tone (default 500)\n"
            "  --pppd PATH          pppd binary (default /usr/sbin/pppd)\n"
            "  --local-ip A.B.C.D   IP assigned to this host (default 192.168.2.1)\n"
            "  --peer-ip A.B.C.D    IP assigned to the dial-in client (default 192.168.2.240)\n"
            "  --dns1 A.B.C.D       primary DNS offered to client\n"
            "  --dns2 A.B.C.D       secondary DNS offered to client\n"
            "  --auth               require PAP (needs pap-secrets)\n"
            "  --log-dir DIR        per-call log directory (default /tmp/softmodem)\n"
            "  --ip-up-script PATH  pppd ip-up script\n"
            "  --ip-down-script PATH  pppd ip-down script\n"
            "  --no-ppp             run without pppd (loopback/testing)\n"
            "  --bbs-db PATH        persistent BBS SQLite database (default bbs.db)\n"
            "  --echo               echo received data back (byte-exact test mode)\n"
            "  --v8                 negotiate with V.8 first (else plain answer tone)\n"
            "  --v34                offer V.34 in the V.8 menu (opt-in; the V.34\n"
            "                       receive data path is not finished yet, so a\n"
            "                       V.34 call will train but not carry usable data)\n"
            "  --binmodem           answer with the vendored BinModem engine: its\n"
            "                       own V.8 and V.34 start-up, 16 kHz internally.\n"
            "                       Needs --v34 to offer V.34.\n"
            "  --debug              verbose logging\n"
            "  --shim FD            internal pppd relay (do not use)\n",
            prog, DEFAULT_PORT);
}

static volatile sig_atomic_t g_run = 1;

static void on_term(int sig)
{
    (void) sig;
    g_run = 0;
}

static int parse_port(const char *opt, const char *arg)
{
    int p = atoi(arg);
    if (p <= 0 || p > 65535)
    {
        fprintf(stderr, "%s: bad %s '%s'\n", prog, opt, arg);
        exit(2);
    }
    return p;
}

int main(int argc, char **argv)
{
    int port = DEFAULT_PORT;
    const char *bind_addr = "0.0.0.0";
    const char *unix_path = NULL;
    sm_call_config_t cfg;
    int listen_fd;
    int call_seq = 0;
    int i;
    struct sigaction sa;

    /* pppd's `pty` command runs this same binary as a plain relay. */
    if (argc >= 3 && strcmp(argv[1], "--shim") == 0)
        return sm_pppd_shim_main(atoi(argv[2]));

    memset(&cfg, 0, sizeof(cfg));
    cfg.rate = 2400;
    cfg.answer_tone_ms = 3300;
    cfg.pre_tone_silence_ms = 500;
    cfg.pppd_path = "/usr/sbin/pppd";
    cfg.local_ip = "192.168.2.1";
    cfg.peer_ip = "192.168.2.240";
    cfg.dns1 = "192.168.2.1";
    cfg.dns2 = "8.8.8.8";
    cfg.log_dir = "/tmp/softmodem";
    cfg.bbs_db = "bbs.db";
    cfg.enable_ppp = 1;
    cfg.log_level = SM_LOG_INFO;

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--listen") == 0 && i + 1 < argc)
            bind_addr = argv[++i];
        else if (strcmp(argv[i], "--unix") == 0 && i + 1 < argc)
            unix_path = argv[++i];
        else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc)
            port = parse_port("--port", argv[++i]);
        else if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc)
        {
            cfg.rate = atoi(argv[++i]);
            if (cfg.rate != 1200 && cfg.rate != 2400)
            {
                fprintf(stderr, "%s: rate must be 1200 or 2400\n", prog);
                return 2;
            }
        }
        else if (strcmp(argv[i], "--tone-ms") == 0 && i + 1 < argc)
            cfg.answer_tone_ms = atoi(argv[++i]);
        else if (strcmp(argv[i], "--pre-silence-ms") == 0 && i + 1 < argc)
            cfg.pre_tone_silence_ms = atoi(argv[++i]);
        else if (strcmp(argv[i], "--pppd") == 0 && i + 1 < argc)
            cfg.pppd_path = argv[++i];
        else if (strcmp(argv[i], "--local-ip") == 0 && i + 1 < argc)
            cfg.local_ip = argv[++i];
        else if (strcmp(argv[i], "--peer-ip") == 0 && i + 1 < argc)
            cfg.peer_ip = argv[++i];
        else if (strcmp(argv[i], "--dns1") == 0 && i + 1 < argc)
            cfg.dns1 = argv[++i];
        else if (strcmp(argv[i], "--dns2") == 0 && i + 1 < argc)
            cfg.dns2 = argv[++i];
        else if (strcmp(argv[i], "--auth") == 0)
            cfg.auth = 1;
        else if (strcmp(argv[i], "--log-dir") == 0 && i + 1 < argc)
            cfg.log_dir = argv[++i];
        else if (strcmp(argv[i], "--ip-up-script") == 0 && i + 1 < argc)
            cfg.ip_up_script = argv[++i];
        else if (strcmp(argv[i], "--ip-down-script") == 0 && i + 1 < argc)
            cfg.ip_down_script = argv[++i];
        else if (strcmp(argv[i], "--no-ppp") == 0)
            cfg.enable_ppp = 0;
        else if (strcmp(argv[i], "--bbs-db") == 0 && i + 1 < argc)
            cfg.bbs_db = argv[++i];
        else if (strcmp(argv[i], "--echo") == 0)
            cfg.echo_data = 1;
        else if (strcmp(argv[i], "--v8") == 0)
            cfg.use_v8 = 1;
        else if (strcmp(argv[i], "--v34") == 0)
            cfg.use_v34 = 1;
        else if (strcmp(argv[i], "--binmodem") == 0)
            cfg.use_binmodem = 1;
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

    /* Our executable path, needed for the pppd pty shim. */
    {
        static char exe[512];
        ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
        if (n < 0)
        {
            fprintf(stderr, "%s: cannot read /proc/self/exe\n", prog);
            return 1;
        }
        exe[n] = 0;
        cfg.shim_exe = exe;
    }

    if (cfg.log_dir && cfg.log_dir[0])
        mkdir(cfg.log_dir, 0755);

    signal(SIGPIPE, SIG_IGN);
    signal(SIGCHLD, SIG_IGN);
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_term;
    sa.sa_flags = SA_RESTART;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    if (unix_path)
        listen_fd = sm_ast_listen_unix(unix_path);
    else
        listen_fd = sm_ast_listen(bind_addr, port);
    if (listen_fd < 0)
    {
        fprintf(stderr, "%s: cannot listen on %s: %s\n",
                prog, unix_path ? unix_path : bind_addr, strerror(errno));
        return 1;
    }

    if (unix_path)
        fprintf(stderr, "[%s] listening on unix:%s (rate=%d, ppp %s:%s -> %s)\n",
                prog, unix_path, cfg.rate,
                cfg.enable_ppp ? cfg.local_ip : "disabled",
                cfg.enable_ppp ? cfg.peer_ip : "-",
                cfg.enable_ppp ? cfg.pppd_path : "-");
    else
        fprintf(stderr, "[%s] listening on %s:%d (rate=%d, ppp %s:%s -> %s)\n",
                prog, bind_addr, port, cfg.rate,
                cfg.enable_ppp ? cfg.local_ip : "disabled",
                cfg.enable_ppp ? cfg.peer_ip : "-",
                cfg.enable_ppp ? cfg.pppd_path : "-");

    while (g_run)
    {
        fprintf(stderr, "[PARENT %d] about to call accept\n", getpid());
        int fd = sm_ast_accept(listen_fd);
        pid_t pid;

        if (fd == -2)
        {
            fprintf(stderr, "[PARENT %d] accept returned EINTR\n", getpid());
            continue;               /* EINTR: check g_run */
        }
        if (fd < 0)
        {
            fprintf(stderr, "[%s] accept failed: %s (errno=%d)\n", prog, strerror(errno), errno);
            break;
        }
        fprintf(stderr, "[PARENT %d] accepted fd=%d, forking\n", getpid(), fd);

        pid = fork();
        fprintf(stderr, "[PARENT %d] fork returned pid=%d\n", getpid(), pid);
        if (pid < 0)
        {
            fprintf(stderr, "[%s] fork failed: %s\n", prog, strerror(errno));
            close(fd);
            continue;
        }
        if (pid == 0)
        {
            /* Child: one call, fully independent. */
            sm_call_t *call;
            int rc;

            close(listen_fd);
            signal(SIGPIPE, SIG_IGN);
            signal(SIGTERM, SIG_DFL);
            signal(SIGINT, SIG_DFL);
            setvbuf(stderr, NULL, _IONBF, 0);
            fprintf(stderr, "[CHILD %d] Starting call %d\n", getpid(), call_seq);

            call = calloc(1, sizeof(*call));
            if (!call)
                _exit(1);
            sm_call_init(call, &cfg, call_seq);
            fprintf(stderr, "[CHILD %d] sm_call_init done, phase=%d\n", getpid(), call->phase);
            rc = sm_call_run(call, fd);
            fprintf(stderr, "[CHILD %d] sm_call_run returned rc=%d\n", getpid(), rc);
            free(call);
            _exit(rc == 0 ? 0 : 1);
        }
        fprintf(stderr, "[PARENT %d] closing fd=%d, call_seq=%d\n", getpid(), fd, call_seq);
        close(fd);
        call_seq++;

        /* Reap finished calls. */
        while (waitpid(-1, NULL, WNOHANG) > 0)
            ;
        fprintf(stderr, "[PARENT %d] loop iteration done, g_run=%d\n", getpid(), g_run);
    }

    close(listen_fd);
    fprintf(stderr, "[%s] shutting down\n", prog);
    return 0;
}
