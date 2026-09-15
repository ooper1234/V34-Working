#ifndef SM_ASYNC_H
#define SM_ASYNC_H

#include <stdint.h>
#include <stddef.h>

/* Asynchronous serial framing over a synchronous bit pipe, as used between a
   classic dial-up modem and a DTE:
     start bit (0), 8 data bits LSB first, stop bit (1). Idle line = 1.
   The modem's TX bit callback pulls framed bits from a bit queue; the modem's
   RX bit callback pushes received bits into a deframer that outputs bytes. */

#define SM_BITQ_SIZE 65536            /* must be a power of two */

typedef struct {
    uint8_t bits[SM_BITQ_SIZE];
    uint32_t head;
    uint32_t tail;
} sm_bitq_t;

void sm_bitq_init(sm_bitq_t *q);
size_t sm_bitq_count(const sm_bitq_t *q);
/* Push a bit; returns 0 on success, -1 if full. */
int sm_bitq_push(sm_bitq_t *q, int bit);
/* Number of complete bytes (= 10 bits each) waiting. */
size_t sm_bitq_bytes(const sm_bitq_t *q);
/* Frame one byte into the queue (start, 8 data LSB-first, stop).
   Returns 0 on success, -1 if there is no room for 10 bits. */
int sm_bitq_push_byte(sm_bitq_t *q, uint8_t byte);
/* Pop one bit; returns -1 when empty. */
int sm_bitq_pop(sm_bitq_t *q);

/* Deframer: consumes a bit stream and assembles bytes. */
typedef struct {
    int state;                        /* 0=idle 1=data 2=stop */
    int nbits;
    int byte;
    int prev;                         /* previous bit, for start edge detect */
    uint8_t out[64];
    int out_len;
    uint64_t framing_errors;
    uint64_t bytes_out;
} sm_deframer_t;

void sm_deframer_init(sm_deframer_t *d);
/* Feed one bit; completed bytes accumulate in out[]. */
void sm_deframer_bit(sm_deframer_t *d, int bit);
/* Take up to len bytes out of the deframer; returns count. */
int sm_deframer_take(sm_deframer_t *d, uint8_t *dst, int len);

#endif
