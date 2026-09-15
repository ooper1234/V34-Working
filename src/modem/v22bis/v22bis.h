#ifndef SM_V22BIS_H
#define SM_V22BIS_H

#include "sm_common.h"
#include "../../logging/sm_log.h"
#include "../../dsp/detectors/sm_power_meter.h"

/* V.22bis modem - full-duplex FDM, 600 symbols/s.
   Calling party TX carrier 1200 Hz / RX 2400 Hz; answering party TX 2400 Hz / RX 1200 Hz. */
#define V22BIS_EQUALIZER_LEN       17
#define V22BIS_EQUALIZER_PRE_LEN    8
#define V22BIS_TX_FILTER_STEPS      9
#define V22BIS_RX_FILTER_STEPS     27
#define V22BIS_TX_PHASES           40
#define V22BIS_TX_PHASE_STEP        3
#define V22BIS_RX_COEFF_SETS       12
#define V22BIS_PULSESHAPER_STEPS   12
#define V22BIS_COEFFS_PER_FILTER   27

#define V22BIS_CARRIER_1200  1200.0f
#define V22BIS_CARRIER_2400  2400.0f
#define V22BIS_BAUD_RATE     600.0f

#define ms_to_symbols(t)  (((t)*600)/1000)

typedef enum {
    V22BIS_RX_TRAINING_NORMAL_OPERATION,
    V22BIS_RX_TRAINING_SYMBOL_ACQUISITION,
    V22BIS_RX_TRAINING_UNSCRAMBLED_ONES,
    V22BIS_RX_TRAINING_UNSCRAMBLED_ONES_SUSTAINING,
    V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200,
    V22BIS_RX_TRAINING_SCRAMBLED_ONES_AT_1200_SUSTAINING,
    V22BIS_RX_TRAINING_WAIT_FOR_SCRAMBLED_ONES_AT_2400,
    V22BIS_RX_TRAINING_PARKED
} v22bis_rx_training_t;

typedef enum {
    V22BIS_TX_TRAINING_NORMAL_OPERATION = 0,
    V22BIS_TX_TRAINING_INITIAL_SILENCE,
    V22BIS_TX_TRAINING_INITIAL_TIMED_SILENCE,
    V22BIS_TX_TRAINING_U11,
    V22BIS_TX_TRAINING_U0011,
    V22BIS_TX_TRAINING_S11,
    V22BIS_TX_TRAINING_TIMED_S11,
    V22BIS_TX_TRAINING_S1111,
    V22BIS_TX_TRAINING_PARKED
} v22bis_tx_training_t;

typedef enum {
    V22BIS_STATUS_TRAINING_SUCCEEDED,
    V22BIS_STATUS_TRAINING_FAILED,
    V22BIS_STATUS_CARRIER_UP,
    V22BIS_STATUS_CARRIER_DOWN,
    V22BIS_STATUS_RETRAIN_OCCURRED
} v22bis_status_t;

typedef int (*v22bis_get_bit_fn)(void *user_data);
typedef void (*v22bis_put_bit_fn)(void *user_data, int bit);
typedef void (*v22bis_status_fn)(void *user_data, v22bis_status_t status);

typedef struct {
    int bit_rate;            /* 1200 or 2400 maximum */
    int options;             /* reserved / guard tone flags */
    bool calling_party;
    v22bis_get_bit_fn get_bit;
    void *get_bit_user_data;
    v22bis_put_bit_fn put_bit;
    void *put_bit_user_data;
    v22bis_status_fn status_handler;
    void *status_user_data;
    sm_log_t log;
    int negotiated_bit_rate;

    /* --- receive --- */
    struct {
        int rrc_filter_step;
        uint32_t scramble_reg;
        int scrambler_pattern_count;
        int training;
        int training_count;
        int signal_present;
        uint32_t carrier_phase;      /* DDS phase (32-bit) */
        int32_t carrier_phase_rate;
        sm_power_meter_t rx_power;
        double carrier_on_power;
        double carrier_off_power;
        int constellation_state;
        float agc_scaling;
        float rrc_filter[V22BIS_RX_FILTER_STEPS];
        float eq_delta;
        sm_complexf_t eq_coeff[V22BIS_EQUALIZER_LEN];
        sm_complexf_t eq_buf[V22BIS_EQUALIZER_LEN];
        float training_error;
        float carrier_track_p;
        float carrier_track_i;
        int eq_step;
        int eq_put_step;
        int gardner_integrate;
        int gardner_step;
        int total_baud_timing_correction;
        int baud_phase;
        int sixteen_way_decisions;
        int pattern_repeats;
        int last_raw_bits;
    } rx;

    /* --- transmit --- */
    struct {
        float guard_tone_gain;
        float gain;
        float rrc_filter_re[V22BIS_TX_FILTER_STEPS];
        float rrc_filter_im[V22BIS_TX_FILTER_STEPS];
        int rrc_filter_step;
        uint32_t scramble_reg;
        int scrambler_pattern_count;
        int training;
        int training_count;
        uint32_t carrier_phase;
        int32_t carrier_phase_rate;
        uint32_t guard_tone_phase;
        int32_t guard_tone_phase_rate;
        int baud_phase;
        int constellation_state;
        int shutdown;
        v22bis_get_bit_fn current_get_bit;
    } tx;
} v22bis_state_t;

#ifdef __cplusplus
extern "C" {
#endif

void v22bis_init(v22bis_state_t *s, bool calling_party, int bit_rate,
                 v22bis_get_bit_fn get_bit, void *get_bit_user_data,
                 v22bis_put_bit_fn put_bit, void *put_bit_user_data,
                 v22bis_status_fn status_handler, void *status_user_data);
void v22bis_restart(v22bis_state_t *s);
void v22bis_set_bit_rate(v22bis_state_t *s, int bit_rate);
void v22bis_rx_restart(v22bis_state_t *s);
void v22bis_rx_set_signal_cutoff(v22bis_state_t *s, float cutoff);
int  v22bis_tx(v22bis_state_t *s, int16_t amp[], int len);
int  v22bis_rx(v22bis_state_t *s, const int16_t amp[], int len);
void v22bis_rx_fillin(v22bis_state_t *s, int len);
float v22bis_rx_carrier_frequency(const v22bis_state_t *s);
int  v22bis_equalizer_state(v22bis_state_t *s, sm_complexf_t **coeffs);
void v22bis_tx_power(v22bis_state_t *s, float power_dbm0);

#ifdef __cplusplus
}
#endif

#endif