#ifndef SM_SIP_H
#define SM_SIP_H

#include <stdint.h>
#include <stddef.h>
#include "sm_common.h"
#include "logging/sm_log.h"
#include "ast_socket/sm_ast_socket.h"

/* Minimal SIP user-agent server + RTP media bridge for the PAP2T ATA.
 *
 * Replaces the Asterisk hop in installations where Asterisk is not available:
 * accepts REGISTER/INVITE/ACK/BYE over UDP, negotiates G.711 PCMU audio, and
 * bridges the audio to the softmodem daemon over the AudioSocket protocol
 * (same interface Asterisk uses), so the daemon is unchanged.
 *
 *    PAP2T --SIP/RTP--> sm_sip --AudioSocket(TCP)--> sm_daemon --pty--> pppd
 */

#define SM_SIP_MAX_MSG      4096
#define SM_SIP_MAX_HEADERS  32
#define SM_SIP_RTP_MAXPAY   1024

typedef struct {
    int bind_port;                  /* SIP signalling port (default 5060) */
    char bind_addr[64];             /* default 0.0.0.0 */
    char advertise_ip[64];          /* IP we advertise in SDP/Contact */
    char daemon_host[64];           /* sm_daemon host (default 127.0.0.1) */
    int daemon_port;                /* sm_daemon AudioSocket port (9092) */
    char uuid[37];                  /* UUID string for the AudioSocket call */
    sm_log_level_t log_level;
} sm_sip_config_t;

typedef struct sm_sip sm_sip_t;

sm_sip_t *sm_sip_create(const sm_sip_config_t *cfg);
void sm_sip_destroy(sm_sip_t *s);

/* Run the server event loop forever (until return_on_hangup calls exhausted).
   max_calls < 0 = run forever. Returns 0 on clean exit. */
int sm_sip_run(sm_sip_t *s, int max_calls);

/* G.711 mu-law codec, exposed for tests. */
int16_t sm_ulaw_decode(uint8_t u);
uint8_t sm_ulaw_encode(int16_t pcm);

#endif
