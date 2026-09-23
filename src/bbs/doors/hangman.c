#include "door.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

static const char *words[] = {"CARRIER", "PACKET", "MODEM", "TERMINAL", "GALAXY", "SIGNAL"};

static void draw(door_context_t *c)
{
    char b[256]; size_t i, p = 0; int won = 1;
    for (i = 0; c->text[i] && p + 3 < sizeof(b); i++) {
        char ch = c->text[i];
        if (strchr((char *)(c->text + 64), ch)) b[p++] = ch;
        else { b[p++] = '_'; won = 0; }
        b[p++] = ' ';
    }
    b[p] = 0;
    c->write(c, b);
    if (won) {
        long long wins = c->load_int(c, "hangman", "wins", 0) + 1;
        c->save_int(c, "hangman", "wins", wins);
        c->write(c, "\r\nSolved! Press Q to return or ENTER for another: ");
        c->state = 2;
    } else {
        snprintf(b, sizeof(b), "  misses %lld/6. Letter or Q: ", c->values[0]);
        c->write(c, b);
    }
}

static void open_game(door_context_t *c)
{
    c->rng = c->rng * 1103515245u + 12345u;
    memset(c->text, 0, sizeof(c->text));
    snprintf(c->text, 63, "%s", words[(c->rng >> 16) % (sizeof(words)/sizeof(words[0]))]);
    c->values[0] = 0; c->state = 1;
    c->write(c, "\r\nHANGMAN\r\n"); draw(c);
}

static void input(door_context_t *c, const char *line)
{
    char ch;
    if (*line == 'q' || *line == 'Q') { c->return_to_menu(c); return; }
    if (c->state == 2) { open_game(c); return; }
    ch = (char)toupper((unsigned char)*line);
    if (ch >= 'A' && ch <= 'Z' && !strchr(c->text + 64, ch)) {
        size_t n = strlen(c->text + 64);
        c->text[64 + n] = ch; c->text[65 + n] = 0;
        if (!strchr(c->text, ch)) c->values[0]++;
    }
    if (c->values[0] >= 6) {
        char b[192]; snprintf(b, sizeof(b), "The word was %.63s. ENTER to retry or Q: ", c->text);
        c->write(c, b); c->state = 2;
    } else draw(c);
}

const door_t door_hangman = { "2", "Hangman", open_game, input };
