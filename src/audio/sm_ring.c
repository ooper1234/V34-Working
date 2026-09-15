#include "sm_ring.h"

#include <stdlib.h>
#include <string.h>

void sm_ring_init(sm_ring_t *r, size_t capacity, int spsc)
{
    memset(r, 0, sizeof(*r));
    r->capacity = capacity;
    r->spsc = spsc;
    r->buf = (int16_t *) malloc(capacity * sizeof(int16_t));
    if (spsc == 0)
        pthread_mutex_init(&r->lock, NULL);
}

void sm_ring_destroy(sm_ring_t *r)
{
    free(r->buf);
    r->buf = NULL;
    if (r->spsc == 0)
        pthread_mutex_destroy(&r->lock);
}

size_t sm_ring_readable(const sm_ring_t *r)
{
    size_t n;
    if (r->spsc)
    {
        n = (size_t) (r->head - r->tail);
        return (n > r->capacity) ? r->capacity : n;
    }
    pthread_mutex_lock((pthread_mutex_t *) &r->lock);
    n = (size_t) (r->head - r->tail);
    pthread_mutex_unlock((pthread_mutex_t *) &r->lock);
    return (n > r->capacity) ? r->capacity : n;
}

size_t sm_ring_writable(const sm_ring_t *r)
{
    size_t fill = sm_ring_readable(r);
    return r->capacity - fill;
}

size_t sm_ring_write(sm_ring_t *r, const int16_t *data, size_t len)
{
    size_t i;
    size_t space;
    size_t mask;

    if (r->spsc == 0)
        pthread_mutex_lock(&r->lock);
    space = r->capacity - sm_ring_readable(r);
    if (len > space)
        len = space;
    mask = r->capacity - 1;
    for (i = 0; i < len; i++)
        r->buf[(r->head + i) & mask] = data[i];
    r->head += len;
    if (r->spsc == 0)
        pthread_mutex_unlock(&r->lock);
    return len;
}

size_t sm_ring_read(sm_ring_t *r, int16_t *data, size_t len)
{
    size_t i;
    size_t avail;
    size_t mask;

    if (r->spsc == 0)
        pthread_mutex_lock(&r->lock);
    avail = sm_ring_readable(r);
    if (len > avail)
        len = avail;
    mask = r->capacity - 1;
    if (data)
        for (i = 0; i < len; i++)
            data[i] = r->buf[(r->tail + i) & mask];
    r->tail += len;
    if (r->spsc == 0)
        pthread_mutex_unlock(&r->lock);
    return len;
}

size_t sm_ring_drop(sm_ring_t *r, size_t len)
{
    size_t avail = sm_ring_readable(r);
    if (len > avail)
        len = avail;
    r->tail += len;
    return len;
}

void sm_ring_clear(sm_ring_t *r)
{
    if (r->spsc == 0)
        pthread_mutex_lock(&r->lock);
    r->tail = r->head;
    if (r->spsc == 0)
        pthread_mutex_unlock(&r->lock);
}

int16_t *sm_ring_peek(sm_ring_t *r, size_t *avail)
{
    size_t mask = r->capacity - 1;
    size_t off = (size_t) (r->tail & mask);
    size_t to_end = r->capacity - off;
    size_t fill;

    fill = sm_ring_readable(r);
    *avail = (to_end < fill) ? to_end : fill;
    return &r->buf[off];
}