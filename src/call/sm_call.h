#ifndef SM_CALL_H
#define SM_CALL_H

#include "sm_common.h"
#include "sm_log.h"
#include "modem/v22bis/v22bis.h"
#include "serial/sm_async.h"
#include "ast_socket/sm_ast_socket.h"

#include <sys/types.h>

#ifdef SM_HAVE_V8
#include "modem/v8/sm_v8.h"
#endif

/* One dial-up call: AudioSocket PCM <-> V.22bis answerer <-> async serial
   <-> pty <-> pppd. */
typedef enum {
    SM_CALL_IDLE = 0,
    SM_CALL_ANSWER_TONE,      /* sending 2100 Hz ANS */
    SM_CALL_V8,               /* V.8 negotiation */
    SM_CALL_HANDSHAKE,        /* V.22bis training running */
    SM_CALL_DATA,             /* both directions in NORMAL_OPERATION */
    SM_CALL_HANGUP
} sm_call_phase_t;

typedef struct {
    /* configuration */
    int rate;                       /* 1200 or 2400 */
    int answer_tone_ms;             /* 2100 Hz answer tone duration (0=off) */
    int pre_tone_silence_ms;        /* silence before answer tone */
    const char *pppd_path;
    const char *shim_exe;
    const char *local_ip;
    const char *peer_ip;
    const char *dns1;
    const char *dns2;
    const char *log_dir;
    const char *ip_up_script;
    const char *ip_down_script;
    int auth;
    int enable_ppp;
    int echo_data;                  /* loop data back instead of using pppd */
    int use_v8;                     /* run V.8 negotiation first */
    sm_log_level_t log_level;
} sm_call_config_t;

typedef struct {
    sm_call_config_t cfg;
    int call_id;

    sm_ast_socket_t as;
    v22bis_state_t modem;

    sm_bitq_t txbits;               /* pppd bytes -> modem TX bits */
    sm_deframer_t deframer;         /* modem RX bits -> pppd bytes */

    /* answer tone generator */
    int tone_samples_left;
    int silence_samples_left;
    double tone_phase;
    double tone_phase_inc;
    double tone_amplitude;

    /* pppd */
    int ppp_fd;
    pid_t pppd_pid;
    int ppp_started;

#ifdef SM_HAVE_V8
    sm_v8_t *v8;
    int v8_done;
    int v8_ok;
    int rx_guard;                   /* samples to ignore after V.8 (FSK tail) */
#endif

    /* state */
    sm_call_phase_t phase;
    int rx_normal;
    int tx_normal;
    int negotiated_rate;
    long long samples_in;
    long long samples_out;
    long long data_bytes_rx;
    long long data_bytes_tx;
    sm_log_t log;

    /* RX byte assembly for the pty */
    uint8_t ppy_out[256];
    int ppy_out_len;

    /* per-frame TX scratch (max AudioSocket payload / 2 samples) */
    int16_t txbuf[SM_AS_MAX_PAYLOAD / 2];
    int16_t rxsquelch[SM_AS_MAX_PAYLOAD / 2];
} sm_call_t;

/* Initialise a call with the given configuration. */
void sm_call_init(sm_call_t *c, const sm_call_config_t *cfg, int call_id);

/* Run one call to completion on the accepted AudioSocket fd. Returns 0 on a
   clean hangup. Blocks until the call ends. */
int sm_call_run(sm_call_t *c, int socket_fd);

#endif
