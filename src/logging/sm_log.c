#include "sm_log.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *level_name(sm_log_level_t level)
{
    switch (level)
    {
    case SM_LOG_ERROR:   return "ERROR";
    case SM_LOG_WARNING: return "WARN";
    case SM_LOG_INFO:    return "INFO";
    case SM_LOG_DEBUG:   return "DEBUG";
    case SM_LOG_TRACE:   return "TRACE";
    case SM_LOG_FLOW:    return "FLOW";
    default:             return "?????";
    }
}

void sm_log_init(sm_log_t *l, sm_log_level_t level, const char *protocol, int call_id)
{
    memset(l, 0, sizeof(*l));
    l->level = level;
    l->call_id = call_id;
    l->protocol = protocol;
}

void sm_log_set_level(sm_log_t *l, sm_log_level_t level) { l->level = level; }
sm_log_level_t sm_log_get_level(const sm_log_t *l) { return l->level; }
void sm_log_set_call_id(sm_log_t *l, int call_id) { l->call_id = call_id; }

void sm_log_message(sm_log_t *l, sm_log_level_t level, const char *fmt, ...)
{
    va_list ap;
    struct timespec ts;
    struct tm tm;

    if ((int) level > (int) l->level)
        return;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm);
    fprintf(stderr, "[%04d-%02d-%02d %02d:%02d:%02d.%03ld] call=%d %s: ",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec/1000000L,
            l->call_id, l->protocol ? l->protocol : "SYS");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

void sm_log_transition(sm_log_t *l, const char *state, const char *detector,
                       double duration_ms, double frequency_hz, double confidence,
                       const char *decoded, const char *action, const char *next_state)
{
    sm_log_message(l, SM_LOG_DEBUG,
                   "%s\n"
                   "    state=%s detector=%s duration_ms=%.1f freq_hz=%.1f "
                   "confidence=%.3f decoded=%s action=%s next_state=%s",
                   state, state, detector, duration_ms, frequency_hz, confidence,
                   decoded ? decoded : "-", action ? action : "-",
                   next_state ? next_state : "-");
}