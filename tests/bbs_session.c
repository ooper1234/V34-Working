#include "bbs/bbs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static char output[131072];
static size_t output_len;

static void collect(void *u, const uint8_t *p, size_t n)
{
    (void)u;
    if (n > sizeof(output) - output_len - 1) n = sizeof(output) - output_len - 1;
    memcpy(output + output_len, p, n); output_len += n; output[output_len] = 0;
}

static void send_line(bbs_session_t *s, const char *p)
{
    bbs_session_feed(s, (const uint8_t *)p, strlen(p));
    bbs_session_feed(s, (const uint8_t *)"\r", 1);
}

int main(void)
{
    const char *db = "/tmp/softmodem-bbs-test.db";
    bbs_connection_meta_t m;
    bbs_session_t *s;
    unlink(db);
    memset(&m, 0, sizeof(m));
    m.protocol="V.34"; m.tx_bps=28800; m.rx_bps=31200; m.connected=1;
    m.packets_rx=m.packets_tx=m.packets_lost=-1; m.jitter_ms=-1;
    s=bbs_session_create(db,&m,collect,NULL); assert(s);
    bbs_session_start(s); assert(strstr(output,"type BBS"));
    send_line(s,"BBS"); assert(bbs_session_route(s)==BBS_ROUTE_BBS);
    send_line(s,"NEW"); send_line(s,"testcaller"); send_line(s,"secret");
    bbs_session_tick(s,1500); assert(strstr(output,"Does your terminal support ANSI"));
    send_line(s,"N"); assert(strstr(output,"SOFTMODEM BBS"));
    send_line(s,"C"); assert(strstr(output,"31200 bps"));
    send_line(s,"D"); send_line(s,"1"); assert(strstr(output,"BLACKJACK"));
    send_line(s,"Q"); send_line(s,"Q");
    bbs_session_destroy(s);

    output_len=0; output[0]=0;
    s=bbs_session_create(db,&m,collect,NULL); assert(s);
    bbs_session_start(s); send_line(s,"dialup.world");
    assert(bbs_session_route(s)==BBS_ROUTE_PPP);
    assert(bbs_session_take_ppp(s,(uint8_t *)output,sizeof(output))==0);
    bbs_session_destroy(s);
    unlink(db);
    puts("bbs_session: ok");
    return 0;
}
