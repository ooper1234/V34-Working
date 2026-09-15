#ifndef SM_LOG_H
#define SM_LOG_H

#include <stdarg.h>
#include <stdint.h>

typedef enum {
    SM_LOG_NONE    = 0,
    SM_LOG_ERROR   = 1,
    SM_LOG_WARNING = 2,
    SM_LOG_INFO    = 3,
    SM_LOG_DEBUG   = 4,
    SM_LOG_TRACE   = 5,
    SM_LOG_FLOW    = 6,
} sm_log_level_t;

typedef struct {
    sm_log_level_t level;
    int call_id;
    const char *protocol;   /* e.g. "V.8", "V22BIS", "V42", "PPP", "DSP" */
    const char *file;
} sm_log_t;

void sm_log_init(sm_log_t *l, sm_log_level_t level, const char *protocol, int call_id);
void sm_log_set_level(sm_log_t *l, sm_log_level_t level);
sm_log_level_t sm_log_get_level(const sm_log_t *l);
void sm_log_set_call_id(sm_log_t *l, int call_id);
void sm_log_message(sm_log_t *l, sm_log_level_t level, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

/* State-transition evidence logging helper. Each line documents the received
   signal/detector responsible for a transition (spec section 30/31). */
void sm_log_transition(sm_log_t *l,
                       const char *state, const char *detector,
                       double duration_ms, double frequency_hz, double confidence,
                       const char *decoded, const char *action, const char *next_state);

#endif