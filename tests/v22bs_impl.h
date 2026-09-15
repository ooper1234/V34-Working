#ifndef V22BS_IMPL_H
#define V22BS_IMPL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define V22BS_CHUNK 160

/* Generic modem status codes, normalised across implementations. */
enum {
    V22BS_ST_CARRIER_UP = 0,
    V22BS_ST_CARRIER_DOWN,
    V22BS_ST_TRAINING_OK,
    V22BS_ST_TRAINING_FAILED,
    V22BS_ST_RETRAIN
};

/* Callbacks, both implementations use identical signatures. */
typedef int  (*v22bs_src_fn)(void *user_data);       /* -1 = end of data */
typedef void (*v22bs_sink_fn)(void *user_data, int bit);
typedef void (*v22bs_status_fn)(void *user_data, int status_int);

typedef struct v22bs v22bs_t;

struct v22bs {
    const char *name;                /* "ours-1200/2400" or "ref-1200/2400" */
    void *cb;                        /* opaque implementation state */
    int  (*tx)(void *cb, int16_t amp[], int len);
    int  (*rx)(void *cb, const int16_t amp[], int len);
    int  (*normal)(void *cb);        /* 1 once in NORMAL_OPERATION */
    int  (*bit_rate)(void *cb);      /* current operating bit rate */
    void (*tx_power)(void *cb, float dbm0);
    double (*carrier_freq)(void *cb);       /* recovered RX carrier, Hz */
    double (*symbol_timing)(void *cb);      /* total symbol timing correction */
    void (*annotate_training)(void *cb);    /* print current TX/RX training state */
};

v22bs_t *v22bs_ours_create(int calling_party, int bit_rate,
                           v22bs_src_fn src, void *src_ud,
                           v22bs_sink_fn sink, void *sink_ud,
                           v22bs_status_fn status, void *status_ud);
v22bs_t *v22bs_ref_create(int calling_party, int bit_rate,
                          v22bs_src_fn src, void *src_ud,
                          v22bs_sink_fn sink, void *sink_ud,
                          v22bs_status_fn status, void *status_ud);
void v22bs_destroy(v22bs_t *m);

#ifdef __cplusplus
}
#endif

#endif