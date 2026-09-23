#include "door.h"
#include <strings.h>

static const door_t *doors[] = {
    &door_blackjack, &door_hangman, &door_space_trader,
    &door_dungeon, &door_speed_challenge, NULL
};

const door_t *door_find(const char *key)
{
    int i;
    for (i = 0; doors[i]; i++)
        if (strcasecmp(key, doors[i]->key) == 0)
            return doors[i];
    return NULL;
}

void door_show_menu(door_context_t *c)
{
    c->write(c,
        "\r\n+------------------------------------------+\r\n"
        "|               DOOR GAMES                 |\r\n"
        "+------------------------------------------+\r\n"
        "| [1] Blackjack                            |\r\n"
        "| [2] Hangman                              |\r\n"
        "| [3] Orbit Merchant                       |\r\n"
        "| [4] Ember Depths                         |\r\n"
        "| [S] Modem Speed Challenge                |\r\n"
        "| [Q] Return to BBS                        |\r\n"
        "+------------------------------------------+\r\nDoor: ");
}
