#ifndef BM_ANSWERER_H
#define BM_ANSWERER_H

/* The BinModem V.34 answerer (Rust, third_party/BinModem/crates/ffi), as the
   daemon sees it: one line sample at a time in and out at 8 kHz, plus a bit
   interface identical in shape to spanDSP's get_bit/put_bit -- the C side
   frames DTE bytes itself (sm_bitq) and deframes recovered bits itself
   (sm_deframer), so this layer never sees a byte.
   Status codes BM_* are shared with crates/ffi/src/lib.rs. */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bm_answerer bm_answerer;

#define BM_RUNNING      0   /* V.8 or V.34 start-up still going */
#define BM_CONNECTED    1   /* in V.34 data mode */
#define BM_FAILED       2   /* terminal failure; bm_failure says why */
#define BM_AGREED_V22   3   /* V.8 chose V.22bis: caller takes over */
#define BM_AGREED_OTHER 4   /* no V.8 on the line: caller takes over */
#define BM_RETRAINING   5   /* back from data mode for a retrain */

/* Next bit to transmit (idle line is ones); called only while the pump
   accepts bits. Recovered RX bits go to put_bit from data mode onward. */
typedef int (*bm_get_bit_fn)(void *user);
typedef void (*bm_put_bit_fn)(void *user, int bit);

/* answer: nonzero for the answering side. want_v34: nonzero to offer V.34
   in the V.8 menu and to fall back to V.34 directly when V.8 does not
   settle. Returns NULL only on allocation failure. */
bm_answerer *bm_create(int answer, int want_v34,
                       bm_get_bit_fn get_bit, void *get_ud,
                       bm_put_bit_fn put_bit, void *put_ud);

/* One 8 kHz line sample in, the corresponding outgoing sample out (s16).
   Deterministic: exactly one out per in, after a few milliseconds of
   resampler priming at the start. */
int bm_step(bm_answerer *a, int sample);

/* Move data bits: top the transmitter up from get_bit and hand recovered
   bits to put_bit. Call once per audio chunk, before bm_step. */
void bm_service(bm_answerer *a);

/* Throw away what the receiver made of the handshake. Call once, at the
   moment BM_CONNECTED is first observed, before bm_service runs again. */
void bm_flush_rx(bm_answerer *a);

int bm_status(bm_answerer *a);
int bm_pending(bm_answerer *a);     /* bits queued in the engine's transmitter */
unsigned long long bm_rx_total(bm_answerer *a); /* bits the receiver handed up */
int bm_found_again(bm_answerer *a);  /* decoder re-acquisitions */
int bm_slips(bm_answerer *a);        /* sample slips seen by the receiver */
unsigned long long bm_underruns(bm_answerer *a); /* dry transmit queue */
unsigned long long bm_clips(bm_answerer *a);     /* engine output past full scale */
double bm_peak(bm_answerer *a);                  /* largest |output| seen */
/* Boundary echo canceller: locked delay in samples (0 = never locked),
   correlation peak of the lock, filter energy (0 = nothing learned). */
void bm_echo(bm_answerer *a, int *delay, double *peak, double *energy);
unsigned long long bm_up_odd(bm_answerer *a, unsigned char *last);
int bm_rate_tx(bm_answerer *a);     /* bit/s, valid once BM_CONNECTED */
int bm_rate_rx(bm_answerer *a);
int bm_carrier(bm_answerer *a);     /* far end's data signal present */
int bm_error_control(bm_answerer *a); /* V.42 LAPM established */
int bm_compression(bm_answerer *a);   /* 0 none, 1 V.42bis, 2 V.44 */
unsigned long long bm_damaged_frames(bm_answerer *a);
int bm_ec_phase(bm_answerer *a);
int bm_lapm_declared(bm_answerer *a);
int bm_ec_observed(bm_answerer *a);

/* Human-readable start-up phase / failure reason. Pointers are into `a`
   and valid until the next call on it. */
const char *bm_phase(bm_answerer *a);
const char *bm_failure(bm_answerer *a);

void bm_destroy(bm_answerer *a);

#ifdef __cplusplus
}
#endif

#endif /* BM_ANSWERER_H */
