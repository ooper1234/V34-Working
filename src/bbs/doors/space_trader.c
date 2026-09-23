#include "door.h"
#include <stdio.h>
#include <stdlib.h>

static void show(door_context_t *c)
{
    char b[320];
    int price = 18 + (int)(c->values[2] % 17);
    snprintf(b, sizeof(b),
        "\r\nORBIT MERCHANT\r\nPort %lld  Credits %lld  Cargo %lld/20  Ore price %d\r\n"
        "[B] Buy ore  [S] Sell ore  [J] Jump port  [Q] Return: ",
        c->values[2], c->values[0], c->values[1], price);
    c->write(c, b);
}

static void open_game(door_context_t *c)
{
    c->values[0] = c->load_int(c, "space", "credits", 500);
    c->values[1] = c->load_int(c, "space", "cargo", 0);
    c->values[2] = c->load_int(c, "space", "port", 1);
    show(c);
}

static void save(door_context_t *c)
{
    c->save_int(c, "space", "credits", c->values[0]);
    c->save_int(c, "space", "cargo", c->values[1]);
    c->save_int(c, "space", "port", c->values[2]);
}

static void input(door_context_t *c, const char *line)
{
    int price = 18 + (int)(c->values[2] % 17);
    if (*line == 'q' || *line == 'Q') { save(c); c->return_to_menu(c); return; }
    if ((*line == 'b' || *line == 'B') && c->values[1] < 20 && c->values[0] >= price) {
        c->values[0] -= price; c->values[1]++;
    } else if ((*line == 's' || *line == 'S') && c->values[1] > 0) {
        c->values[0] += price; c->values[1]--;
    } else if (*line == 'j' || *line == 'J') {
        c->rng = c->rng * 1664525u + 1013904223u;
        c->values[2] = 1 + ((c->rng >> 16) % 12);
    }
    save(c); show(c);
}

const door_t door_space_trader = { "3", "Orbit Merchant", open_game, input };
