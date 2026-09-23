#include "bbs.h"
#include "doors/door.h"

#include <ctype.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

typedef struct sqlite3 sqlite3;
typedef struct sqlite3_stmt sqlite3_stmt;
#define SQLITE_OK 0
#define SQLITE_ROW 100
#define SQLITE_DONE 101
#define SQLITE_TRANSIENT ((void(*)(void*))-1)

typedef struct {
    void *so;
    int (*open)(const char *, sqlite3 **);
    int (*close)(sqlite3 *);
    int (*exec)(sqlite3 *, const char *, int(*)(void*,int,char**,char**), void *, char **);
    int (*prepare)(sqlite3 *, const char *, int, sqlite3_stmt **, const char **);
    int (*step)(sqlite3_stmt *);
    int (*finalize)(sqlite3_stmt *);
    int (*bind_int64)(sqlite3_stmt *, int, long long);
    int (*bind_text)(sqlite3_stmt *, int, const char *, int, void(*)(void*));
    long long (*column_int64)(sqlite3_stmt *, int);
    const unsigned char *(*column_text)(sqlite3_stmt *, int);
    long long (*last_id)(sqlite3 *);
} sql_api_t;

enum stage {
    ST_SELECT, ST_LOGIN, ST_PASSWORD, ST_NEW_NAME, ST_NEW_PASSWORD,
    ST_TERM_QUERY, ST_TERM_ASK, ST_MAIN, ST_TERM_MENU, ST_TERM_ANSI,
    ST_TERM_SIZE, ST_TERM_CHARSET, ST_DOORS, ST_DOOR
};

struct bbs_session {
    bbs_write_fn write_fn;
    void *write_user;
    bbs_connection_meta_t meta;
    char protocol[32], codec[32];
    int route;
    enum stage stage;
    char line[160];
    size_t line_len;
    unsigned query_ms;
    char query[96];
    size_t query_len;
    uint8_t ppp[1024];
    size_t ppp_len;
    sql_api_t api;
    sqlite3 *db;
    int user_id;
    int security;
    char handle[48];
    char pending_name[48];
    int ansi, color, columns, rows, cp437;
    long long connection_id;
    door_context_t door_ctx;
    const door_t *door;
};

static void outn(bbs_session_t *s, const void *p, size_t n)
{
    if (s->write_fn && n) s->write_fn(s->write_user, p, n);
}
static void out(bbs_session_t *s, const char *p) { outn(s, p, strlen(p)); }

static int sql_load(bbs_session_t *s)
{
#define LOAD(name, sym) do { *(void **)(&s->api.name) = dlsym(s->api.so, sym); if (!s->api.name) return -1; } while (0)
    s->api.so = dlopen("libsqlite3.so.0", RTLD_NOW | RTLD_LOCAL);
    if (!s->api.so) return -1;
    LOAD(open,"sqlite3_open"); LOAD(close,"sqlite3_close"); LOAD(exec,"sqlite3_exec");
    LOAD(prepare,"sqlite3_prepare_v2"); LOAD(step,"sqlite3_step"); LOAD(finalize,"sqlite3_finalize");
    LOAD(bind_int64,"sqlite3_bind_int64"); LOAD(bind_text,"sqlite3_bind_text");
    LOAD(column_int64,"sqlite3_column_int64"); LOAD(column_text,"sqlite3_column_text");
    LOAD(last_id,"sqlite3_last_insert_rowid");
#undef LOAD
    return 0;
}

static void schema(bbs_session_t *s)
{
    const char *q =
        "PRAGMA journal_mode=WAL;"
        "CREATE TABLE IF NOT EXISTS users("
        "id INTEGER PRIMARY KEY,handle TEXT UNIQUE COLLATE NOCASE,password_hash TEXT NOT NULL,"
        "security INTEGER NOT NULL DEFAULT 10,terminal_type TEXT,ansi INTEGER,color INTEGER,"
        "columns INTEGER,rows INTEGER,cp437 INTEGER,created_at INTEGER NOT NULL);"
        "CREATE TABLE IF NOT EXISTS connections("
        "id INTEGER PRIMARY KEY,user_id INTEGER,started_at INTEGER NOT NULL,ended_at INTEGER,"
        "protocol TEXT,tx_bps INTEGER,rx_bps INTEGER,audio_codec TEXT,packets_rx INTEGER,"
        "packets_tx INTEGER,packets_lost INTEGER,average_jitter_ms REAL,duration_seconds INTEGER,"
        "terminal_type TEXT,ansi_enabled INTEGER);"
        "CREATE TABLE IF NOT EXISTS game_state("
        "user_id INTEGER NOT NULL,game TEXT NOT NULL,key TEXT NOT NULL,value INTEGER NOT NULL,"
        "PRIMARY KEY(user_id,game,key));";
    s->api.exec(s->db, q, NULL, NULL, NULL);
}

static unsigned long long password_hash(const char *user, const char *pass)
{
    unsigned long long h = 1469598103934665603ULL;
    const unsigned char *p;
    for (p=(const unsigned char*)user; *p; p++) { h ^= (unsigned char)tolower(*p); h *= 1099511628211ULL; }
    h ^= 0x5a; h *= 1099511628211ULL;
    for (p=(const unsigned char*)pass; *p; p++) { h ^= *p; h *= 1099511628211ULL; }
    return h;
}

static int prepare(bbs_session_t *s, sqlite3_stmt **st, const char *q)
{ return s->db && s->api.prepare(s->db,q,-1,st,NULL)==SQLITE_OK; }

static void terminal_save(bbs_session_t *s)
{
    sqlite3_stmt *st;
    if (!prepare(s,&st,"UPDATE users SET terminal_type=?,ansi=?,color=?,columns=?,rows=?,cp437=? WHERE id=?")) return;
    s->api.bind_text(st,1,s->ansi?"ANSI":"ASCII",-1,SQLITE_TRANSIENT);
    s->api.bind_int64(st,2,s->ansi); s->api.bind_int64(st,3,s->color);
    s->api.bind_int64(st,4,s->columns); s->api.bind_int64(st,5,s->rows);
    s->api.bind_int64(st,6,s->cp437); s->api.bind_int64(st,7,s->user_id);
    s->api.step(st); s->api.finalize(st);
}

static void record_connection(bbs_session_t *s)
{
    sqlite3_stmt *st;
    if (s->connection_id || !s->user_id || !prepare(s,&st,
        "INSERT INTO connections(user_id,started_at,protocol,tx_bps,rx_bps,audio_codec,packets_rx,packets_tx,packets_lost,average_jitter_ms,terminal_type,ansi_enabled) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)")) return;
    s->api.bind_int64(st,1,s->user_id); s->api.bind_int64(st,2,(long long)s->meta.connected_at);
    s->api.bind_text(st,3,s->protocol,-1,SQLITE_TRANSIENT); s->api.bind_int64(st,4,s->meta.tx_bps);
    s->api.bind_int64(st,5,s->meta.rx_bps);
    if (s->codec[0]) s->api.bind_text(st,6,s->codec,-1,SQLITE_TRANSIENT);
    s->api.bind_int64(st,7,s->meta.packets_rx); s->api.bind_int64(st,8,s->meta.packets_tx);
    s->api.bind_int64(st,9,s->meta.packets_lost);
    /* A negative integer is retained as unknown rather than made up. */
    s->api.bind_int64(st,10,s->meta.jitter_ms < 0 ? -1 : (long long)(s->meta.jitter_ms*1000));
    s->api.bind_text(st,11,s->ansi?"ANSI":"ASCII",-1,SQLITE_TRANSIENT);
    s->api.bind_int64(st,12,s->ansi); s->api.step(st); s->api.finalize(st);
    s->connection_id=s->api.last_id(s->db);
}

static void prompt_main(bbs_session_t *s)
{
    if (s->ansi) out(s,"\x1b[2J\x1b[H\x1b[36m");
    out(s,
        "==========================================\r\n"
        "          SOFTMODEM BBS\r\n"
        "==========================================\r\n"
        "[D] Doors / Games\r\n[C] Dial-up Connection Speed\r\n"
        "[T] Terminal Settings\r\n[H] Connection History\r\n"
        "[P] Switch to PPP\r\n[Q] Log off\r\nChoice: ");
    if (s->ansi) out(s,"\x1b[0m");
    s->stage=ST_MAIN;
}

static void begin_terminal(bbs_session_t *s)
{
    if (s->columns > 0 && s->rows > 0) { record_connection(s); prompt_main(s); return; }
    s->stage=ST_TERM_QUERY; s->query_ms=0; s->query_len=0;
    /* DSR and xterm-compatible text-area-size query. Both are ignored by a
       conforming ANSI terminal and never enter the caller's input line. */
    out(s,"\r\nDetecting terminal...\x1b[6n\x1b[18t");
}

static int login_lookup(bbs_session_t *s,const char *name,const char *pass)
{
    sqlite3_stmt *st; char hash[32]; const unsigned char *stored;
    snprintf(hash,sizeof(hash),"%016llx",password_hash(name,pass));
    if(!prepare(s,&st,"SELECT id,password_hash,security,ansi,color,columns,rows,cp437 FROM users WHERE handle=?"))return 0;
    s->api.bind_text(st,1,name,-1,SQLITE_TRANSIENT);
    if(s->api.step(st)!=SQLITE_ROW){s->api.finalize(st);return 0;}
    stored=s->api.column_text(st,1);
    if(!stored||strcmp((const char*)stored,hash)){s->api.finalize(st);return 0;}
    s->user_id=(int)s->api.column_int64(st,0); s->security=(int)s->api.column_int64(st,2);
    s->ansi=(int)s->api.column_int64(st,3); s->color=(int)s->api.column_int64(st,4);
    s->columns=(int)s->api.column_int64(st,5); s->rows=(int)s->api.column_int64(st,6); s->cp437=(int)s->api.column_int64(st,7);
    snprintf(s->handle,sizeof(s->handle),"%s",name); s->api.finalize(st); return 1;
}

static int register_user(bbs_session_t *s,const char *name,const char *pass)
{
    sqlite3_stmt *st; char hash[32]; int rc;
    if(!*name||strlen(name)>32||!*pass)return 0;
    snprintf(hash,sizeof(hash),"%016llx",password_hash(name,pass));
    if(!prepare(s,&st,"INSERT INTO users(handle,password_hash,created_at) VALUES(?,?,?)"))return 0;
    s->api.bind_text(st,1,name,-1,SQLITE_TRANSIENT); s->api.bind_text(st,2,hash,-1,SQLITE_TRANSIENT);
    s->api.bind_int64(st,3,(long long)time(NULL)); rc=s->api.step(st); s->api.finalize(st);
    if (rc != SQLITE_DONE)
        return 0;
    s->user_id=(int)s->api.last_id(s->db); s->security=10;
    snprintf(s->handle,sizeof(s->handle),"%s",name); return 1;
}

static void connection_screen(bbs_session_t *s)
{
    char b[1024]; long sec=(long)(time(NULL)-s->meta.connected_at); int rate=s->meta.rx_bps>s->meta.tx_bps?s->meta.rx_bps:s->meta.tx_bps;
    int bars=rate>0?(rate*24+33599)/33600:0, i; char graph[25];
    if (bars > 24) bars=24;
    for(i=0;i<24;i++)graph[i]=i<bars?'#':'-';
    graph[24]=0;
    snprintf(b,sizeof(b),
        "\r\n==========================================\r\nDIAL-UP CONNECTION SPEED\r\n==========================================\r\n"
        "Modem Protocol : %s\r\nTX Speed       : %s%d%s\r\nRX Speed       : %s%d%s\r\nDirection      : Full Duplex\r\n"
        "Audio Codec    : %s\r\nSample Rate    : %s\r\nPacket Time    : %s\r\nCall Duration  : %02ld:%02ld:%02ld\r\n"
        "Packets RX     : %s\r\nPackets TX     : %s\r\nPackets Lost   : %s\r\nJitter         : %s\r\n\r\n[%s] %d bps\r\n\r\n[Q] Return: ",
        s->protocol[0]?s->protocol:"N/A",
        s->meta.tx_bps>0?"":"N/A",s->meta.tx_bps>0?s->meta.tx_bps:0,s->meta.tx_bps>0?" bps":"",
        s->meta.rx_bps>0?"":"N/A",s->meta.rx_bps>0?s->meta.rx_bps:0,s->meta.rx_bps>0?" bps":"",
        s->codec[0]?s->codec:"N/A",s->meta.sample_rate?"available":"N/A",s->meta.packet_time_ms?"available":"N/A",
        sec/3600,(sec/60)%60,sec%60,
        s->meta.packets_rx>=0?"available":"N/A",s->meta.packets_tx>=0?"available":"N/A",
        s->meta.packets_lost>=0?"available":"N/A",s->meta.jitter_ms>=0?"available":"N/A",graph,rate);
    out(s,b);
}

static void history(bbs_session_t *s)
{
    sqlite3_stmt *st; char b[256];
    out(s,"\r\nDate         Protocol   TX       RX       Time\r\n");
    if(prepare(s,&st,"SELECT started_at,protocol,tx_bps,rx_bps,duration_seconds FROM connections WHERE user_id=? ORDER BY started_at DESC LIMIT 10")){
        s->api.bind_int64(st,1,s->user_id);
        while(s->api.step(st)==SQLITE_ROW){time_t t=(time_t)s->api.column_int64(st,0);struct tm tm;const unsigned char *p=s->api.column_text(st,1);localtime_r(&t,&tm);
            snprintf(b,sizeof(b),"%04d-%02d-%02d   %-8s %7lld  %7lld  %lldm\r\n",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,p?(const char*)p:"N/A",s->api.column_int64(st,2),s->api.column_int64(st,3),s->api.column_int64(st,4)/60);out(s,b);}
        s->api.finalize(st);
    }
    out(s,"Press ENTER: ");
}

static long long game_load(door_context_t *c,const char *game,const char *key,long long fallback)
{
    bbs_session_t *s=c->session;sqlite3_stmt *st;long long v=fallback;
    if(prepare(s,&st,"SELECT value FROM game_state WHERE user_id=? AND game=? AND key=?")){s->api.bind_int64(st,1,s->user_id);s->api.bind_text(st,2,game,-1,SQLITE_TRANSIENT);s->api.bind_text(st,3,key,-1,SQLITE_TRANSIENT);if(s->api.step(st)==SQLITE_ROW)v=s->api.column_int64(st,0);s->api.finalize(st);}return v;
}
static void game_save(door_context_t *c,const char *game,const char *key,long long value)
{
    bbs_session_t *s=c->session;sqlite3_stmt *st;
    if(prepare(s,&st,"INSERT INTO game_state(user_id,game,key,value) VALUES(?,?,?,?) ON CONFLICT(user_id,game,key) DO UPDATE SET value=excluded.value")){s->api.bind_int64(st,1,s->user_id);s->api.bind_text(st,2,game,-1,SQLITE_TRANSIENT);s->api.bind_text(st,3,key,-1,SQLITE_TRANSIENT);s->api.bind_int64(st,4,value);s->api.step(st);s->api.finalize(st);}
}
static void door_write(door_context_t *c,const char *p){out(c->session,p);}
static void door_return(door_context_t *c){bbs_session_t*s=c->session;s->door=NULL;s->stage=ST_DOORS;door_show_menu(c);}

static void open_doors(bbs_session_t *s)
{
    door_context_t *c=&s->door_ctx;memset(c,0,sizeof(*c));c->session=s;c->user_id=s->user_id;c->handle=s->handle;c->security_level=s->security;c->ansi=s->ansi;c->color=s->color;c->columns=s->columns;c->rows=s->rows;c->protocol=s->protocol;c->tx_bps=s->meta.tx_bps;c->rx_bps=s->meta.rx_bps;c->write=door_write;c->load_int=game_load;c->save_int=game_save;c->return_to_menu=door_return;c->rng=(unsigned)time(NULL)^(unsigned)s->user_id;s->stage=ST_DOORS;door_show_menu(c);
}

static void handle_line(bbs_session_t *s,char *line)
{
    while(*line&&isspace((unsigned char)*line))line++;
    switch(s->stage){
    case ST_SELECT:
        s->ppp_len = 0; /* text selection is not an early PPP frame */
        if(!strcasecmp(line,"BBS")){s->route=BBS_ROUTE_BBS;s->stage=ST_LOGIN;out(s,"\r\nLogin (or NEW): ");}
        else{s->route=BBS_ROUTE_PPP;}
        break;
    case ST_LOGIN:
        if(!strcasecmp(line,"NEW")){s->stage=ST_NEW_NAME;out(s,"Choose handle: ");}
        else{snprintf(s->pending_name,sizeof(s->pending_name),"%.47s",line);s->stage=ST_PASSWORD;out(s,"Password: ");}break;
    case ST_PASSWORD:
        if(login_lookup(s,s->pending_name,line)){out(s,"\r\nWelcome back, ");out(s,s->handle);out(s,".\r\n");begin_terminal(s);}
        else{s->stage=ST_LOGIN;out(s,"\r\nLogin incorrect. Login (or NEW): ");}break;
    case ST_NEW_NAME: snprintf(s->pending_name,sizeof(s->pending_name),"%.47s",line);s->stage=ST_NEW_PASSWORD;out(s,"Choose password: ");break;
    case ST_NEW_PASSWORD:
        if(register_user(s,s->pending_name,line)){out(s,"\r\nAccount created.\r\n");begin_terminal(s);}else{s->stage=ST_LOGIN;out(s,"\r\nThat handle is unavailable. Login (or NEW): ");}break;
    case ST_TERM_ASK:
    case ST_TERM_ANSI: s->ansi=(*line=='y'||*line=='Y'||!strcasecmp(line,"ANSI"));s->color=s->ansi;s->columns=80;s->rows=24;terminal_save(s);record_connection(s);prompt_main(s);break;
    case ST_MAIN:
        if(*line=='d'||*line=='D')open_doors(s);
        else if(*line=='c'||*line=='C'){connection_screen(s);}
        else if(*line=='t'||*line=='T'){s->stage=ST_TERM_MENU;out(s,"\r\nTerminal Mode : ");out(s,s->ansi?"ANSI":"ASCII");out(s,"\r\nColor         : ");out(s,s->color?"Yes":"No");{char b[180];snprintf(b,sizeof(b),"\r\nColumns       : %d\r\nRows          : %d\r\nCharacter Set : %s\r\n[A] ANSI / ASCII  [S] Screen Size  [C] Character Set  [T] Test ANSI  [Q] Return: ",s->columns,s->rows,s->cp437?"CP437":"UTF-8/ASCII");out(s,b);}}
        else if(*line=='h'||*line=='H'){history(s);}
        else if(*line=='p'||*line=='P'){s->route=BBS_ROUTE_PPP;out(s,"\r\nEntering PPP mode.\r\n");}
        else if(*line=='q'||*line=='Q'){s->route=BBS_ROUTE_HANGUP;out(s,"\r\nGoodbye.\r\n");}
        else prompt_main(s);
        break;
    case ST_TERM_MENU:
        if(*line=='a'||*line=='A'){s->stage=ST_TERM_ANSI;out(s,"ANSI support? (Y/N): ");}
        else if(*line=='s'||*line=='S'){s->stage=ST_TERM_SIZE;out(s,"Columns Rows (example 80 24): ");}
        else if(*line=='c'||*line=='C'){s->stage=ST_TERM_CHARSET;out(s,"Character set [C]P437 or [A]SCII: ");}
        else if(*line=='t'||*line=='T'){out(s,"\x1b[31mRED \x1b[32mGREEN \x1b[34mBLUE\x1b[0m\r\nANSI test complete.\r\n");prompt_main(s);}
        else prompt_main(s);
        break;
    case ST_TERM_SIZE: {int c=0,r=0;if(sscanf(line,"%d %d",&c,&r)==2&&c>=20&&c<=300&&r>=10&&r<=120){s->columns=c;s->rows=r;terminal_save(s);}prompt_main(s);}break;
    case ST_TERM_CHARSET:s->cp437=(*line=='c'||*line=='C');terminal_save(s);prompt_main(s);break;
    case ST_DOORS:
        if(*line=='q'||*line=='Q')prompt_main(s);else{s->door=door_find(line);if(s->door){s->stage=ST_DOOR;s->door->open(&s->door_ctx);}else door_show_menu(&s->door_ctx);}break;
    case ST_DOOR:if(s->door)s->door->input(&s->door_ctx,line);break;
    default: prompt_main(s);break;
    }
}

bbs_session_t *bbs_session_create(const char *path,const bbs_connection_meta_t *m,bbs_write_fn fn,void *u)
{
    bbs_session_t*s=calloc(1,sizeof(*s));if(!s)return NULL;s->write_fn=fn;s->write_user=u;s->meta=*m;s->route=BBS_ROUTE_SELECT;s->stage=ST_SELECT;s->columns=0;s->rows=0;
    s->meta.connected_at=m->connected_at?m->connected_at:time(NULL);s->meta.packets_rx=m->packets_rx;s->meta.packets_tx=m->packets_tx;s->meta.packets_lost=m->packets_lost;s->meta.jitter_ms=m->jitter_ms;
    snprintf(s->protocol,sizeof(s->protocol),"%s",m->protocol?m->protocol:"");snprintf(s->codec,sizeof(s->codec),"%s",m->audio_codec?m->audio_codec:"");
    if(sql_load(s)==0&&s->api.open(path&&*path?path:"bbs.db",&s->db)==SQLITE_OK)
        schema(s);
    return s;
}

void bbs_session_start(bbs_session_t*s){out(s,"\r\nSoftmodem connected.\r\nIf you are here for the BBS, type BBS.\r\nOtherwise type your PPP service name (for example dialup.world),\r\nor start PPP directly.\r\nSelection: ");}

static int ppp_prefix(const uint8_t*p,size_t n){static const uint8_t a[]={0x7e,0xff,0x03,0xc0,0x21},b[]={0x7e,0xff,0x7d,0x23,0xc0,0x21};return(n>=sizeof(a)&&!memcmp(p,a,sizeof(a)))||(n>=sizeof(b)&&!memcmp(p,b,sizeof(b)));}

void bbs_session_feed(bbs_session_t*s,const uint8_t*d,size_t n)
{
    size_t i;if(!s||s->route==BBS_ROUTE_PPP||s->route==BBS_ROUTE_HANGUP)return;
    if(s->stage==ST_SELECT&&n){size_t keep=n<sizeof(s->ppp)-s->ppp_len?n:sizeof(s->ppp)-s->ppp_len;memcpy(s->ppp+s->ppp_len,d,keep);s->ppp_len+=keep;if(ppp_prefix(s->ppp,s->ppp_len)){s->route=BBS_ROUTE_PPP;return;}}
    for(i=0;i<n;i++){
        unsigned char ch=d[i];
        if(s->stage==ST_TERM_QUERY){if(s->query_len<sizeof(s->query)-1)s->query[s->query_len++]=(char)ch;s->query[s->query_len]=0;continue;}
        if(ch=='\r'||ch=='\n'){if(s->line_len){s->line[s->line_len]=0;handle_line(s,s->line);s->line_len=0;}continue;}
        if((ch==8||ch==127)&&s->line_len){s->line_len--;out(s,"\x08 \x08");continue;}
        if(ch>=32&&ch<127&&s->line_len<sizeof(s->line)-1){s->line[s->line_len++]=(char)ch;if(s->stage!=ST_PASSWORD&&s->stage!=ST_NEW_PASSWORD)outn(s,&ch,1);}
    }
}

void bbs_session_tick(bbs_session_t*s,unsigned ms)
{
    int row=0,col=0;char*p;if(!s||s->stage!=ST_TERM_QUERY)return;s->query_ms+=ms;
    p=strstr(s->query,"\x1b[");
    if(p&&sscanf(p,"\x1b[%d;%dR",&row,&col)==2&&row>0&&col>0){s->ansi=1;s->color=1;}
    {char*p=strstr(s->query,"\x1b[8;");if(p&&sscanf(p,"\x1b[8;%d;%dt",&row,&col)==2){s->rows=row;s->columns=col;s->ansi=1;s->color=1;}}
    if(s->query_ms>=1500){if(!s->columns)s->columns=80;if(!s->rows)s->rows=24;if(s->ansi){out(s,"\r\nTerminal detected: ANSI\r\n");terminal_save(s);record_connection(s);prompt_main(s);}else{s->stage=ST_TERM_ASK;out(s,"\r\nTerminal detected: Unknown\r\nDoes your terminal support ANSI? (Y/N): ");}}
}

int bbs_session_route(const bbs_session_t*s){return s?s->route:BBS_ROUTE_PPP;}
size_t bbs_session_take_ppp(bbs_session_t*s,uint8_t*d,size_t cap){size_t n=s->ppp_len<cap?s->ppp_len:cap;if(n)memcpy(d,s->ppp,n);if(n<s->ppp_len)memmove(s->ppp,s->ppp+n,s->ppp_len-n);s->ppp_len-=n;return n;}

void bbs_session_destroy(bbs_session_t*s)
{
    if(!s)return;
    if(s->db&&s->connection_id){sqlite3_stmt*st;if(prepare(s,&st,"UPDATE connections SET ended_at=?,duration_seconds=?,terminal_type=?,ansi_enabled=? WHERE id=?")){time_t now=time(NULL);s->api.bind_int64(st,1,(long long)now);s->api.bind_int64(st,2,(long long)(now-s->meta.connected_at));s->api.bind_text(st,3,s->ansi?"ANSI":"ASCII",-1,SQLITE_TRANSIENT);s->api.bind_int64(st,4,s->ansi);s->api.bind_int64(st,5,s->connection_id);s->api.step(st);s->api.finalize(st);}}
    if(s->db)s->api.close(s->db);
    if(s->api.so)dlclose(s->api.so);
    free(s);
}
