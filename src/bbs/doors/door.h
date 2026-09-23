#ifndef SM_BBS_DOOR_H
#define SM_BBS_DOOR_H

#include <stddef.h>
#include <stdint.h>

typedef struct door_context door_context_t;

typedef struct {
    const char *key;
    const char *name;
    void (*open)(door_context_t *ctx);
    void (*input)(door_context_t *ctx, const char *line);
} door_t;

struct door_context {
    void *session;
    int user_id;
    const char *handle;
    int security_level;
    int ansi;
    int color;
    int columns;
    int rows;
    const char *protocol;
    int tx_bps;
    int rx_bps;
    void (*write)(door_context_t *, const char *);
    long long (*load_int)(door_context_t *, const char *game,
                          const char *key, long long fallback);
    void (*save_int)(door_context_t *, const char *game,
                     const char *key, long long value);
    void (*return_to_menu)(door_context_t *);
    unsigned rng;
    int state;
    long long values[8];
    char text[128];
};

const door_t *door_find(const char *key);
void door_show_menu(door_context_t *ctx);

extern const door_t door_blackjack;
extern const door_t door_hangman;
extern const door_t door_space_trader;
extern const door_t door_dungeon;
extern const door_t door_speed_challenge;

#endif
