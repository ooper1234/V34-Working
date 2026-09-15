#ifndef SM_V8_H
#define SM_V8_H

#include <stdbool.h>
#include <stdint.h>

/* Thin wrapper around spanDSP's V.8 implementation for the answerer role.
   Used by the daemon when enabled; the V.22bis fallback path does not need it. */

typedef struct sm_v8 sm_v8_t;

typedef void (*sm_v8_result_fn)(void *user_data, int status,
                                int modulations, int call_function, int protocols);

sm_v8_t *sm_v8_create(bool calling_party,
                      int allowed_modulations,
                      sm_v8_result_fn handler,
                      void *user_data,
                      int log_level);
void sm_v8_destroy(sm_v8_t *v);

/* Generate up to max_len samples; returns the number generated. */
int sm_v8_tx(sm_v8_t *v, int16_t *amp, int max_len);
/* Process received samples; returns number unprocessed. */
int sm_v8_rx(sm_v8_t *v, const int16_t *amp, int len);

#endif
