#include "sm_v8.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* spanDSP's v8.h requires the core headers first. */
#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/complex.h"
#include "spandsp/async.h"
#include "spandsp/dds.h"
#include "spandsp/power_meter.h"
#include "spandsp/fsk.h"
#include "spandsp/queue.h"
#include "spandsp/tone_generate.h"
#include "spandsp/super_tone_rx.h"
#include "spandsp/modem_connect_tones.h"
#include "spandsp/v8.h"

struct sm_v8 {
    v8_state_t *v8;
    sm_v8_result_fn handler;
    void *user_data;
};

static void v8_result_bridge(void *user_data, v8_parms_t *result)
{
    sm_v8_t *v = user_data;

    if (v->handler)
        v->handler(v->user_data, result->status,
                   (int) result->jm_cm.modulations,
                   (int) result->jm_cm.call_function,
                   (int) result->jm_cm.protocols);
}

sm_v8_t *sm_v8_create(bool calling_party,
                      int allowed_modulations,
                      sm_v8_result_fn handler,
                      void *user_data,
                      int log_level)
{
    sm_v8_t *v = calloc(1, sizeof(*v));
    v8_parms_t parms;

    if (!v)
        return NULL;
    v->handler = handler;
    v->user_data = user_data;

    memset(&parms, 0, sizeof(parms));
    /* Plain ANSam, NOT ANSam with phase reversals. The phase-reversal
       variant signals a V.92-capable answerer; a V.92 calling modem then
       sends V.92 "CP" packets and waits for a V.92 answer signal that this
       endpoint does not implement. Plain ANSam makes the caller skip the
       V.92 exchange and send its V.8 CM directly. */
    parms.modem_connect_tone = calling_party ? MODEM_CONNECT_TONES_NONE
                                             : MODEM_CONNECT_TONES_ANSAM;
    parms.gateway_mode = false;
    parms.send_ci = true;
    parms.v92 = -1;
    parms.jm_cm.call_function = V8_CALL_V_SERIES;
    parms.jm_cm.modulations = allowed_modulations;
    parms.jm_cm.protocols = V8_PROTOCOL_LAPM_V42;
    parms.jm_cm.pcm_modem_availability = 0;
    parms.jm_cm.pstn_access = 0;
    parms.jm_cm.nsf = -1;
    parms.jm_cm.t66 = -1;

    v->v8 = v8_init(NULL, calling_party, &parms, v8_result_bridge, v);
    if (!v->v8)
    {
        free(v);
        return NULL;
    }
    (void) log_level;
    return v;
}

void sm_v8_destroy(sm_v8_t *v)
{
    if (!v)
        return;
    v8_free(v->v8);
    free(v);
}

int sm_v8_tx(sm_v8_t *v, int16_t *amp, int max_len)
{
    return v8_tx(v->v8, amp, max_len);
}

int sm_v8_rx(sm_v8_t *v, const int16_t *amp, int len)
{
    return v8_rx(v->v8, amp, len);
}
