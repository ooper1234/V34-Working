#include "door.h"
#include <stdio.h>

static void show(door_context_t *c)
{
    char b[256];
    snprintf(b, sizeof(b),
        "\r\nEMBER DEPTHS\r\nLevel %lld  XP %lld  HP %lld  Gold %lld\r\n"
        "[E] Explore  [R] Rest  [Q] Return: ",
        c->values[0], c->values[1], c->values[2], c->values[3]);
    c->write(c, b);
}

static void save(door_context_t *c)
{
    c->save_int(c,"dungeon","level",c->values[0]);
    c->save_int(c,"dungeon","xp",c->values[1]);
    c->save_int(c,"dungeon","hp",c->values[2]);
    c->save_int(c,"dungeon","gold",c->values[3]);
}

static void open_game(door_context_t *c)
{
    c->values[0]=c->load_int(c,"dungeon","level",1);
    c->values[1]=c->load_int(c,"dungeon","xp",0);
    c->values[2]=c->load_int(c,"dungeon","hp",20);
    c->values[3]=c->load_int(c,"dungeon","gold",0); show(c);
}

static void input(door_context_t *c, const char *line)
{
    if (*line=='q'||*line=='Q') { save(c); c->return_to_menu(c); return; }
    if (*line=='r'||*line=='R') c->values[2]=20+c->values[0]*3;
    if (*line=='e'||*line=='E') {
        long long hit, gain;
        c->rng=c->rng*1103515245u+12345u;
        hit=1+(c->rng>>16)%8; gain=4+(c->rng>>20)%12;
        c->values[2]-=hit;
        if(c->values[2]<=0){ c->write(c,"You wake at the gate, poorer but alive.\r\n"); c->values[2]=20+c->values[0]*3; c->values[3]/=2; }
        else { c->values[1]+=gain; c->values[3]+=gain/2; }
        if(c->values[1]>=c->values[0]*50){c->values[0]++; c->write(c,"You gained a level!\r\n");}
    }
    save(c); show(c);
}

const door_t door_dungeon = { "4", "Ember Depths", open_game, input };
