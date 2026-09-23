#include "door.h"
#include <stdio.h>
#include <stdlib.h>

static int card(door_context_t *c)
{
    c->rng = c->rng * 1664525u + 1013904223u;
    return 1 + (int)((c->rng >> 16) % 10u);
}

static void prompt(door_context_t *c)
{
    char b[160];
    snprintf(b, sizeof(b), "You: %lld  Dealer: %lld  [H]it [S]tand [Q]uit: ",
             c->values[1], c->values[2]);
    c->write(c, b);
}

static void open_game(door_context_t *c)
{
    c->values[0] = c->load_int(c, "blackjack", "credits", 1000);
    c->values[1] = card(c) + card(c);
    c->values[2] = card(c);
    c->state = 1;
    c->write(c, "\r\nBLACKJACK -- wager 25 credits per hand\r\n");
    prompt(c);
}

static void input(door_context_t *c, const char *line)
{
    char b[160];
    if (*line == 'q' || *line == 'Q') { c->return_to_menu(c); return; }
    if (*line == 'h' || *line == 'H') {
        c->values[1] += card(c);
        if (c->values[1] > 21) {
            c->values[0] -= 25;
            snprintf(b, sizeof(b), "Bust. Balance: %lld\r\n", c->values[0]);
            c->write(c, b);
            c->save_int(c, "blackjack", "credits", c->values[0]);
            open_game(c);
            return;
        }
        prompt(c);
        return;
    }
    if (*line == 's' || *line == 'S') {
        while (c->values[2] < 17) c->values[2] += card(c);
        if (c->values[2] > 21 || c->values[1] > c->values[2]) c->values[0] += 25;
        else if (c->values[1] < c->values[2]) c->values[0] -= 25;
        snprintf(b, sizeof(b), "Dealer has %lld. Balance: %lld\r\n",
                 c->values[2], c->values[0]);
        c->write(c, b);
        c->save_int(c, "blackjack", "credits", c->values[0]);
        open_game(c);
        return;
    }
    prompt(c);
}

const door_t door_blackjack = { "1", "Blackjack", open_game, input };
