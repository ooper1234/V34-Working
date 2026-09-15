#include "v22bs_impl.h"

#include <stdbool.h>
#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/complex.h"
#include "spandsp/async.h"
#include "spandsp/v29rx.h"
#include "spandsp/v22bis.h"
#include "spandsp/power_meter.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/v22bis.h"

/* The reference library is compiled with its v22bis_* symbols renamed to
   ref_v22bis_* so we can link it alongside our own v22bis_* implementation. */
extern v22bis_state_t *ref_v22bis_init(v22bis_state_t *s, int bit_rate, int options,
                                       bool calling_party,
                                       span_get_bit_func_t get_bit, void *get_bit_user_data,
                                       span_put_bit_func_t put_bit, void *put_bit_user_data);
extern int ref_v22bis_rx(v22bis_state_t *s, const int16_t amp[], int len);
extern int ref_v22bis_tx(v22bis_state_t *s, int16_t amp[], int len);
extern void ref_v22bis_set_modem_status_handler(v22bis_state_t *s,
                                                span_modem_status_func_t handler,
                                                void *user_data);
extern int ref_v22bis_get_current_bit_rate(v22bis_state_t *s);
extern void ref_v22bis_tx_power(v22bis_state_t *s, float power);
extern float ref_v22bis_rx_carrier_frequency(v22bis_state_t *s);
extern float ref_v22bis_rx_symbol_timing_correction(v22bis_state_t *s);
extern logging_state_t *ref_v22bis_get_logging_state(v22bis_state_t *s);

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    v22bis_state_t modem_s;
    const char *name;
    v22bs_src_fn src;
    void *src_ud;
    v22bs_sink_fn sink;
    void *sink_ud;
    v22bs_status_fn status;
    void *status_ud;
    int normal;
    int reported;
    int bit_rate;
} ref_t;

static void ref_status(void *user_data, int status)
{
    ref_t *r = user_data;

    switch (status)
    {
    case SIG_STATUS_CARRIER_UP:
        r->status(r->status_ud, V22BS_ST_CARRIER_UP);
        break;
    case SIG_STATUS_CARRIER_DOWN:
        r->status(r->status_ud, V22BS_ST_CARRIER_DOWN);
        break;
    case SIG_STATUS_TRAINING_SUCCEEDED:
        r->normal = 1;
        if (!r->reported)
            r->status(r->status_ud, V22BS_ST_TRAINING_OK);
        r->reported = 1;
        break;
    case SIG_STATUS_TRAINING_FAILED:
        r->status(r->status_ud, V22BS_ST_TRAINING_FAILED);
        break;
    case SIG_STATUS_MODEM_RETRAIN_OCCURRED:
        r->status(r->status_ud, V22BS_ST_RETRAIN);
        break;
    default:
        break;
    }
}

static int ref_get_bit(void *user_data)
{
    ref_t *r = user_data;
    return r->src(r->src_ud);
}

static void ref_put_bit(void *user_data, int bit)
{
    ref_t *r = user_data;
    r->sink(r->sink_ud, bit);
}

static int ref_tx(void *cb, int16_t amp[], int len)
{
    ref_t *r = cb;
    return ref_v22bis_tx(&r->modem_s, amp, len);
}

static int ref_rx(void *cb, const int16_t amp[], int len)
{
    ref_t *r = cb;
    return ref_v22bis_rx(&r->modem_s, amp, len);
}

static int ref_normal(void *cb)
{
    ref_t *r = cb;
    return r->normal;
}

static int ref_bit_rate(void *cb)
{
    ref_t *r = cb;
    return ref_v22bis_get_current_bit_rate(&r->modem_s);
}

static void ref_tx_power(void *cb, float dbm0)
{
    ref_t *r = cb;
    ref_v22bis_tx_power(&r->modem_s, dbm0);
}

static double ref_carrier_freq(void *cb)
{
    ref_t *r = cb;
    return (double) ref_v22bis_rx_carrier_frequency(&r->modem_s);
}

static double ref_symbol_timing(void *cb)
{
    ref_t *r = cb;
    return (double) ref_v22bis_rx_symbol_timing_correction(&r->modem_s);
}

static void ref_annotate(void *cb)
{
    ref_t *r = cb;

    printf("  [%s] rate=%d carrier=%.1f Hz\n",
           r->name,
           ref_v22bis_get_current_bit_rate(&r->modem_s),
           ref_v22bis_rx_carrier_frequency(&r->modem_s));
}

v22bs_t *v22bs_ref_create(int calling_party, int bit_rate,
                          v22bs_src_fn src, void *src_ud,
                          v22bs_sink_fn sink, void *sink_ud,
                          v22bs_status_fn status, void *status_ud)
{
    v22bs_t *m;
    ref_t *r;

    m = calloc(1, sizeof(*m));
    r = calloc(1, sizeof(*r));
    r->name = "ref";
    r->src = src;
    r->src_ud = src_ud;
    r->sink = sink;
    r->sink_ud = sink_ud;
    r->status = status;
    r->status_ud = status_ud;
    r->bit_rate = bit_rate;

    ref_v22bis_init(&r->modem_s, bit_rate, 0, calling_party != 0,
                ref_get_bit, r, ref_put_bit, r);
    ref_v22bis_set_modem_status_handler(&r->modem_s, ref_status, r);
    span_log_set_level(ref_v22bis_get_logging_state(&r->modem_s), SPAN_LOG_FLOW);

    m->name = "ref";
    m->cb = r;
    m->tx = ref_tx;
    m->rx = ref_rx;
    m->normal = ref_normal;
    m->bit_rate = ref_bit_rate;
    m->tx_power = ref_tx_power;
    m->carrier_freq = ref_carrier_freq;
    m->symbol_timing = ref_symbol_timing;
    m->annotate_training = ref_annotate;
    return m;
}