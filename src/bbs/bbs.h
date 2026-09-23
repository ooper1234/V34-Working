#ifndef SM_BBS_H
#define SM_BBS_H

#include <stddef.h>
#include <stdint.h>
#include <time.h>

typedef struct bbs_session bbs_session_t;
typedef void (*bbs_write_fn)(void *user, const uint8_t *data, size_t len);

typedef struct {
    const char *protocol;
    int tx_bps;
    int rx_bps;
    int connected;
    time_t connected_at;
    int carrier_state;              /* -1 unknown, 0 down, 1 up */
    const char *audio_codec;        /* NULL when the modem layer cannot know */
    int sample_rate;                /* 0 when unavailable */
    int packet_time_ms;             /* 0 when unavailable */
    long long packets_rx;           /* -1 when unavailable */
    long long packets_tx;
    long long packets_lost;
    double jitter_ms;               /* negative when unavailable */
} bbs_connection_meta_t;

enum {
    BBS_ROUTE_SELECT = 0,
    BBS_ROUTE_BBS,
    BBS_ROUTE_PPP,
    BBS_ROUTE_HANGUP
};

bbs_session_t *bbs_session_create(const char *db_path,
                                  const bbs_connection_meta_t *meta,
                                  bbs_write_fn write_fn, void *write_user);
void bbs_session_destroy(bbs_session_t *s);
void bbs_session_start(bbs_session_t *s);
void bbs_session_feed(bbs_session_t *s, const uint8_t *data, size_t len);
void bbs_session_tick(bbs_session_t *s, unsigned elapsed_ms);
int bbs_session_route(const bbs_session_t *s);
size_t bbs_session_take_ppp(bbs_session_t *s, uint8_t *dst, size_t cap);

#endif
