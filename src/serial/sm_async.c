#include "sm_async.h"

#include <string.h>

void sm_bitq_init(sm_bitq_t *q)
{
    memset(q, 0, sizeof(*q));
}

size_t sm_bitq_count(const sm_bitq_t *q)
{
    return (size_t) (q->head - q->tail);
}

int sm_bitq_push(sm_bitq_t *q, int bit)
{
    if (q->head - q->tail >= SM_BITQ_SIZE)
        return -1;
    q->bits[q->head & (SM_BITQ_SIZE - 1)] = (uint8_t) (bit & 1);
    q->head++;
    return 0;
}

size_t sm_bitq_bytes(const sm_bitq_t *q)
{
    return sm_bitq_count(q) / 10;
}

int sm_bitq_push_byte(sm_bitq_t *q, uint8_t byte)
{
    int i;

    if (q->head - q->tail + 10 > SM_BITQ_SIZE)
        return -1;
    sm_bitq_push(q, 0);                       /* start bit */
    for (i = 0; i < 8; i++)
        sm_bitq_push(q, (byte >> i) & 1);     /* data LSB first */
    sm_bitq_push(q, 1);                       /* stop bit */
    return 0;
}

int sm_bitq_pop(sm_bitq_t *q)
{
    if (q->head == q->tail)
        return -1;
    {
        int bit = q->bits[q->tail & (SM_BITQ_SIZE - 1)];
        q->tail++;
        return bit;
    }
}

void sm_deframer_init(sm_deframer_t *d)
{
    memset(d, 0, sizeof(*d));
    d->prev = 1;
}

void sm_deframer_bit(sm_deframer_t *d, int bit)
{
    bit &= 1;

    /* Flush the accumulated output before it can overflow. The caller is
       expected to drain frequently (once per audio frame). */
    if (d->out_len >= (int) sizeof(d->out))
        d->out_len = 0;

    switch (d->state)
    {
    case 0:
        /* Hunt for a start bit: a 1->0 edge. */
        if (d->prev == 1 && bit == 0)
        {
            d->state = 1;
            d->nbits = 0;
            d->byte = 0;
        }
        d->prev = bit;
        break;

    default:
        /* Collect 8 data bits, LSB first. */
        d->byte |= bit << d->nbits;
        d->nbits++;
        if (d->nbits >= 8)
            d->state = 2;
        d->prev = bit;
        break;

    case 2:
        /* Stop bit must be 1. */
        d->prev = bit;
        if (bit == 1)
        {
            if (d->out_len < (int) sizeof(d->out))
                d->out[d->out_len++] = (uint8_t) d->byte;
            d->bytes_out++;
        }
        else
        {
            d->framing_errors++;
        }
        d->state = 0;
        break;
    }
}

int sm_deframer_take(sm_deframer_t *d, uint8_t *dst, int len)
{
    if (len > d->out_len)
        len = d->out_len;
    if (len > 0)
    {
        memcpy(dst, d->out, (size_t) len);
        memmove(d->out, d->out + len, (size_t) (d->out_len - len));
        d->out_len -= len;
    }
    return len;
}
