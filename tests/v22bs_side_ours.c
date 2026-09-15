#include "v22bs_impl.h"
#include "modem/v22bis/v22bis.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    v22bis_state_t modem;
    const char *name;
    v22bs_src_fn src;
    void *src_ud;
    v22bs_sink_fn sink;
    void *sink_ud;
    v22bs_status_fn status;
    void *status_ud;
    int normal;
    int reported;
} ours_t;

static void our_status(void *user_data, v22bis_status_t status)
{
    ours_t *o = user_data;

    if (!o->status)
        return;
    switch (status)
    {
    case V22BIS_STATUS_TRAINING_SUCCEEDED:
        break;
    case V22BIS_STATUS_CARRIER_UP:
        o->status(o->status_ud, V22BS_ST_CARRIER_UP);
        break;
    case V22BIS_STATUS_CARRIER_DOWN:
        o->status(o->status_ud, V22BS_ST_CARRIER_DOWN);
        break;
    case V22BIS_STATUS_RETRAIN_OCCURRED:
        o->status(o->status_ud, V22BS_ST_RETRAIN);
        break;
    default:
        break;
    }
}

static int our_get_bit(void *user_data)
{
    ours_t *o = user_data;
    return o->src(o->src_ud);
}

static void our_put_bit(void *user_data, int bit)
{
    ours_t *o = user_data;
    o->sink(o->sink_ud, bit);
}

static int our_tx(void *cb, int16_t amp[], int len)
{
    ours_t *o = cb;
    return v22bis_tx(&o->modem, amp, len);
}

static int our_rx(void *cb, const int16_t amp[], int len)
{
    ours_t *o = cb;
    return v22bis_rx(&o->modem, amp, len);
}

static int our_normal(void *cb)
{
    ours_t *o = cb;

    if (o->modem.rx.training == V22BIS_RX_TRAINING_NORMAL_OPERATION
        && o->modem.tx.training == V22BIS_TX_TRAINING_NORMAL_OPERATION)
    {
        if (!o->reported)
        {
            if (o->status)
                o->status(o->status_ud, V22BS_ST_TRAINING_OK);
        }
        o->reported = 1;
        o->normal = 1;
    }
    return o->normal;
}

static int our_bit_rate(void *cb)
{
    ours_t *o = cb;
    return o->modem.negotiated_bit_rate;
}

static void our_tx_power(void *cb, float dbm0)
{
    ours_t *o = cb;
    v22bis_tx_power(&o->modem, dbm0);
}

static double our_carrier_freq(void *cb)
{
    ours_t *o = cb;
    return (double) v22bis_rx_carrier_frequency(&o->modem);
}

static double our_symbol_timing(void *cb)
{
    ours_t *o = cb;
    return (double) o->modem.rx.total_baud_timing_correction;
}

static const char *our_tx_training_names[] = {
    "NORMAL", "INIT_SIL", "INIT_T_SIL", "U11", "U0011", "S11", "TIMED_S11", "S1111", "PARKED"
};
static const char *our_rx_training_names[] = {
    "NORMAL", "SYM_ACQ", "UNSCR_ON", "UNSCR_ON_SUST", "SCR_ON_1200", "SCR_ON_1200_SUST",
    "WAIT_SCR_2400", "PARKED"
};

static void our_annotate(void *cb)
{
    ours_t *o = cb;
    int tx = o->modem.tx.training;
    int rx = o->modem.rx.training;

    printf("  [%s] TX=%s RX=%s rate=%d carrier=%.1f Hz\n",
           o->name,
           our_tx_training_names[tx < 0 ? 0 : (tx > 8 ? 8 : tx)],
           our_rx_training_names[rx < 0 ? 0 : (rx > 7 ? 7 : rx)],
           o->modem.negotiated_bit_rate,
           v22bis_rx_carrier_frequency(&o->modem));
}

v22bs_t *v22bs_ours_create(int calling_party, int bit_rate,
                           v22bs_src_fn src, void *src_ud,
                           v22bs_sink_fn sink, void *sink_ud,
                           v22bs_status_fn status, void *status_ud)
{
    v22bs_t *m;
    ours_t *o;

    m = calloc(1, sizeof(*m));
    o = calloc(1, sizeof(*o));
    o->name = "ours";
    o->src = src;
    o->src_ud = src_ud;
    o->sink = sink;
    o->sink_ud = sink_ud;
    o->status = status;
    o->status_ud = status_ud;

    v22bis_init(&o->modem, calling_party != 0, bit_rate,
                our_get_bit, o, our_put_bit, o, our_status, o);

    m->name = "ours";
    m->cb = o;
    m->tx = our_tx;
    m->rx = our_rx;
    m->normal = our_normal;
    m->bit_rate = our_bit_rate;
    m->tx_power = our_tx_power;
    m->carrier_freq = our_carrier_freq;
    m->symbol_timing = our_symbol_timing;
    m->annotate_training = our_annotate;
    return m;
}

void v22bs_destroy(v22bs_t *m)
{
    if (!m)
        return;
    free(m->cb);
    free(m);
}