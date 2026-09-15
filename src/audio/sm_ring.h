#ifndef SM_RING_H
#define SM_RING_H

#include "sm_common.h"
#include <assert.h>
#include <pthread.h>

/* Single-producer single-consumer lock-free ring buffer of int16_t PCM samples.
   Head is written by the producer, tail by the consumer. Pointer arithmetic is
   done with sequence numbers to avoid the classic ABA wrap edge case. */
typedef struct {
    int16_t *buf;
    size_t capacity;
    uint64_t head;   /* absolute write position (producer only) */
    uint64_t tail;   /* absolute read position  (consumer only) */
    int spsc;        /* if 1, no locking (single producer + single consumer) */
    pthread_mutex_t lock;
} sm_ring_t;

void sm_ring_init(sm_ring_t *r, size_t capacity, int spsc);
void sm_ring_destroy(sm_ring_t *r);

size_t sm_ring_readable(const sm_ring_t *r);
size_t sm_ring_writable(const sm_ring_t *r);
size_t sm_ring_write(sm_ring_t *r, const int16_t *data, size_t len);
size_t sm_ring_read(sm_ring_t *r, int16_t *data, size_t len);
size_t sm_ring_drop(sm_ring_t *r, size_t len);
void sm_ring_clear(sm_ring_t *r);
/* Returns a contiguous read span at the current tail. */
int16_t *sm_ring_peek(sm_ring_t *r, size_t *avail);

#endif