#include "sm_ast_socket.h"

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

int sm_ast_listen(const char *bind_addr, int port)
{
    int fd;
    int one = 1;
    struct sockaddr_in addr;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t) port);
    if (bind_addr == NULL || bind_addr[0] == 0 || strcmp(bind_addr, "*") == 0)
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    else if (inet_pton(AF_INET, bind_addr, &addr.sin_addr) != 1)
    {
        close(fd);
        return -1;
    }
    if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0 || listen(fd, 4) < 0)
    {
        close(fd);
        return -1;
    }
    return fd;
}

int sm_ast_listen_unix(const char *path)
{
    int fd;
    struct sockaddr_un addr;

    unlink(path);
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0 || listen(fd, 4) < 0)
    {
        close(fd);
        return -1;
    }
    return fd;
}

int sm_ast_connect_unix(const char *path)
{
    int fd;
    struct sockaddr_un addr;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (connect(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        close(fd);
        return -1;
    }
    return fd;
}

int sm_ast_connect(const char *host, int port)
{
    int fd;
    struct sockaddr_in addr;
    int one = 1;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t) port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1)
    {
        close(fd);
        return -1;
    }
    if (connect(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        close(fd);
        return -1;
    }
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return fd;
}

int sm_ast_accept(int listen_fd)
{
    int fd;
    int one = 1;

    for (;;)
    {
        fd = accept(listen_fd, NULL, NULL);
        if (fd >= 0)
            break;
        if (errno == EINTR)
            return -2;              /* let the caller check a shutdown flag */
        if (errno == ECONNABORTED)
            continue;
        return -1;
    }
    /* Audio is small messages at 20 ms cadence; disable Nagle so a 20 ms
       frame never waits on an ACK. */
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return fd;
}

void sm_ast_init(sm_ast_socket_t *s, int fd)
{
    memset(s, 0, sizeof(*s));
    s->fd = fd;
}

/* Read up to `want` bytes into buf+off, with deadline handling.
   Returns bytes read (>0), 0 on timeout, -1 on error/EOF. */
static int recv_into(int fd, uint8_t *buf, size_t want, int *timeout_left)
{
    struct pollfd pfd;
    int r;
    ssize_t n;

    for (;;)
    {
        pfd.fd = fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        r = poll(&pfd, 1, *timeout_left);
        if (r < 0)
        {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (r == 0)
            return 0;
        n = recv(fd, buf, want, 0);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            return -1;
        }
        return (n == 0) ? -1 : (int) n;
    }
}

int sm_ast_read(sm_ast_socket_t *s, uint8_t *kind, uint8_t *buf, size_t buf_len,
                size_t *payload_len, int timeout_ms)
{
    int timeout_left = (timeout_ms < 0) ? 0 : timeout_ms;

    /* Phase 1: accumulate the 3-byte header (only when no message pending). */
    if (!s->rx_pending)
    {
        while (s->rxhdr < 3)
        {
            int n = recv_into(s->fd, s->rxhead + s->rxhdr, 3 - s->rxhdr, &timeout_left);
            if (n <= 0)
                return n;
            s->rxhdr += (size_t) n;
        }
        s->rx_kind = s->rxhead[0];
        s->rx_expected = ((size_t) s->rxhead[1] << 8) | s->rxhead[2];
        s->rxlen = 0;
        s->rxhdr = 0;
        s->rx_pending = 1;
    }

    /* Phase 2: accumulate the payload; excess beyond the caller's buffer is
       drained and discarded. */
    while (s->rxlen < s->rx_expected)
    {
        size_t remaining = s->rx_expected - s->rxlen;
        size_t bufcap = sizeof(s->rxbuf);
        int n;

        if (s->rxlen < bufcap)
        {
            size_t room = bufcap - s->rxlen;
            size_t want = (remaining < room) ? remaining : room;
            n = recv_into(s->fd, s->rxbuf + s->rxlen, want, &timeout_left);
            if (n <= 0)
                return n;
            s->rxlen += (size_t) n;
        }
        else
        {
            uint8_t scratch[256];
            size_t chunk = (remaining < sizeof(scratch)) ? remaining : sizeof(scratch);
            n = recv_into(s->fd, scratch, chunk, &timeout_left);
            if (n <= 0)
                return n;
            s->rxlen += (size_t) n;
        }
    }

    {
        size_t copy_len = (s->rx_expected < buf_len) ? s->rx_expected : buf_len;
        if (buf && copy_len > 0)
            memcpy(buf, s->rxbuf, copy_len);
    }
    if (kind)
        *kind = s->rx_kind;
    if (payload_len)
        *payload_len = s->rx_expected;

    /* Reset for the next message. */
    s->rxhdr = 0;
    s->rxlen = 0;
    s->rx_expected = 0;
    s->rx_pending = 0;
    return 1;
}

static int send_all(int fd, const uint8_t *buf, size_t len, int timeout_ms)
{
    size_t off = 0;
    int deadline_left = timeout_ms;

    while (off < len)
    {
        struct pollfd pfd;
        ssize_t n;
        int r;

        pfd.fd = fd;
        pfd.events = POLLOUT;
        pfd.revents = 0;
        r = poll(&pfd, 1, deadline_left);
        if (r < 0)
        {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (r == 0)
            return -1;
        n = send(fd, buf + off, len - off, MSG_NOSIGNAL);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            return -1;
        }
        off += (size_t) n;
    }
    return 0;
}

int sm_ast_write(sm_ast_socket_t *s, uint8_t kind, const uint8_t *payload, size_t len)
{
    uint8_t hdr[3];

    if (len > 0xFFFF)
        return -1;
    hdr[0] = kind;
    hdr[1] = (uint8_t) ((len >> 8) & 0xFF);
    hdr[2] = (uint8_t) (len & 0xFF);
    if (send_all(s->fd, hdr, 3, 5000) < 0)
        return -1;
    if (len > 0 && send_all(s->fd, payload, len, 5000) < 0)
        return -1;
    return 0;
}

int sm_ast_write_audio(sm_ast_socket_t *s, const int16_t *samples, size_t count)
{
    return sm_ast_write(s, SM_AS_KIND_AUDIO, (const uint8_t *) samples, count * 2);
}

void sm_ast_uuid_string(const uint8_t uuid[16], char out[37])
{
    snprintf(out, 37,
             "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             uuid[0], uuid[1], uuid[2], uuid[3],
             uuid[4], uuid[5], uuid[6], uuid[7],
             uuid[8], uuid[9], uuid[10], uuid[11],
             uuid[12], uuid[13], uuid[14], uuid[15]);
}
