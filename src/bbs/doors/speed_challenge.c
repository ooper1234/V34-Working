#include "door.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static void open_game(door_context_t *c)
{
    char b[256]; int i;
    snprintf(b,sizeof(b),"\r\nMODEM SPEED CHALLENGE\r\nProtocol: %s\r\nNegotiated TX: %d bps  RX: %d bps\r\n\r\nReceiving 1024 test characters...\r\n",
             c->protocol?c->protocol:"N/A",c->tx_bps,c->rx_bps);
    c->write(c,b); c->values[0]=(long long)time(NULL);
    /* 62 visible characters plus CRLF, sixteen times: exactly 1024 bytes. */
    for(i=0;i<16;i++) c->write(c,"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz\r\n");
    c->write(c,"Test complete. Press ENTER when received: "); c->state=1;
}

static void input(door_context_t *c,const char *line)
{
    char b[256]; long long elapsed=(long long)time(NULL)-c->values[0];
    (void)line; if(elapsed<1)elapsed=1;
    snprintf(b,sizeof(b),"\r\nNegotiated RX rate : %d bps\r\nCharacters sent    : 1024\r\nUser response time : %lld s\r\nMeasured BBS data  : N/A (no remote delivery acknowledgement)\r\n",
             c->rx_bps,elapsed);
    c->write(c,b); c->return_to_menu(c);
}

const door_t door_speed_challenge = { "S", "Modem Speed Challenge", open_game, input };
