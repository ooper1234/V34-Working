#ifndef SM_AST_SOCKET_H
#define SM_AST_SOCKET_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/* Asterisk AudioSocket protocol.
 *
 * TCP stream. Every message: 1-byte type, 2-byte big-endian length, payload.
 *
 *   0x00 hangup          no payload
 *   0x01 UUID            16-byte binary UUID (sent by Asterisk first)
 *   0x03 DTMF            1-byte ASCII digit
 *   0x10 audio           signed linear 16-bit LE, 8 kHz, mono PCM
 *   0xFF error           error notification
 */
#define SM_AS_KIND_HANGUP   0x00
#define SM_AS_KIND_UUID     0x01
#define SM_AS_KIND_DTMF     0x03
#define SM_AS_KIND_AUDIO    0x10
#define SM_AS_KIND_ERROR    0xFF

#define SM_AS_MAX_PAYLOAD   4096

/* A buffered AudioSocket connection. Reads are framed and fully resumable:
   partial TCP segments are accumulated until a whole message is available,
   and a timeout mid-message preserves all state for the next call. */
typedef struct {
    int fd;
    uint8_t rxbuf[SM_AS_MAX_PAYLOAD];
    size_t rxlen;                   /* payload bytes buffered so far */
    size_t rxhdr;                   /* header bytes buffered so far (0..3) */
    uint8_t rxhead[3];              /* partial header */
    size_t rx_expected;             /* pending payload length */
    uint8_t rx_kind;                /* pending message kind */
    int rx_pending;                 /* 1 = header parsed, payload incomplete */
} sm_ast_socket_t;

/* Listening socket on bind_addr:port. Returns fd or -1. */
int sm_ast_listen(const char *bind_addr, int port);

/* Unix-domain variants (used for namespace-isolated end-to-end tests, since
   the AudioSocket protocol is just a TCP byte stream). */
int sm_ast_listen_unix(const char *path);
int sm_ast_connect_unix(const char *path);

/* TCP connect. Returns fd or -1. */
int sm_ast_connect(const char *host, int port);

/* Accept a connection; returns a new fd, -2 on EINTR, or -1. */
int sm_ast_accept(int listen_fd);

/* Initialise a connection wrapper around an accepted fd. */
void sm_ast_init(sm_ast_socket_t *s, int fd);

/* Read the next complete message. Blocks up to timeout_ms (0 = block forever,
   -1 = poll once without waiting). Returns:
     1   message read: *kind and *payload_len set; payload copied to buf
         (truncated to buf_len; excess discarded)
     0   timeout (no message; state preserved for the next call)
     -1  EOF or error                                                    */
int sm_ast_read(sm_ast_socket_t *s, uint8_t *kind, uint8_t *buf, size_t buf_len,
                size_t *payload_len, int timeout_ms);

/* Write one message; handles partial writes. Returns 0 on success, -1 error. */
int sm_ast_write(sm_ast_socket_t *s, uint8_t kind, const uint8_t *payload, size_t len);

/* Convenience: write PCM samples as an audio message. */
int sm_ast_write_audio(sm_ast_socket_t *s, const int16_t *samples, size_t count);

/* Parse the 16-byte UUID payload into a printable string. */
void sm_ast_uuid_string(const uint8_t uuid[16], char out[37]);

#endif
