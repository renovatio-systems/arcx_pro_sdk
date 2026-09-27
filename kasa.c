/*
 * kasa.c
 *
 * TP-Link Kasa bulb client, see kasa.h. Only the fields this program needs are
 * parsed out of the JSON replies.
 */

#include "kasa.h"

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define KASA_TIMEOUT_MS 3000
#define KASA_MAX_REPLY  (64 * 1024)
#define KASA_KEY        171

static void set_err(char *err, size_t err_size, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void set_err(char *err, size_t err_size, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(err, err_size, fmt, ap);
    va_end(ap);
}

/* ---------------------------------------------------------------------------
 * Transport
 * ------------------------------------------------------------------------- */

static void kasa_encrypt(uint8_t *buf, size_t len)
{
    uint8_t key = KASA_KEY;
    for (size_t i = 0; i < len; i++)
        key = buf[i] ^= key;
}

static void kasa_decrypt(uint8_t *buf, size_t len)
{
    uint8_t key = KASA_KEY;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t c = buf[i];
        buf[i] ^= key;
        key = c;
    }
}

static bool wait_fd(int fd, short events)
{
    struct pollfd pfd = {fd, events, 0};
    int           rc;
    do
        rc = poll(&pfd, 1, KASA_TIMEOUT_MS);
    while (rc < 0 && errno == EINTR);
    return rc > 0 && !(pfd.revents & (POLLERR | POLLNVAL));
}

static int tcp_connect(const char *host, char *err, size_t err_size)
{
    struct addrinfo  hints = {0}, *res, *ai;
    int              fd    = -1;
    int              rc;

    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    if ((rc = getaddrinfo(host, KASA_PORT, &hints, &res)) != 0)
    {
        set_err(err, err_size, "%s: %s", host, gai_strerror(rc));
        return -1;
    }

    for (ai = res; ai; ai = ai->ai_next)
    {
        fd = socket(ai->ai_family, ai->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC, ai->ai_protocol);
        if (fd < 0)
            continue;
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0)
            break;
        if (errno == EINPROGRESS)
        {
            /* A refused connection shows up as POLLERR, so read SO_ERROR on any wake-up. */
            struct pollfd pfd = {fd, POLLOUT, 0};
            int           so_err = ETIMEDOUT;
            socklen_t     len    = sizeof(so_err);
            int           rc;
            do
                rc = poll(&pfd, 1, KASA_TIMEOUT_MS);
            while (rc < 0 && errno == EINTR);
            if (rc > 0)
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_err, &len);
            if (so_err == 0)
                break;
            errno = so_err;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0)
        set_err(err, err_size, "connect to %s:%s failed: %s", host, KASA_PORT, strerror(errno));
    return fd;
}

static bool send_all(int fd, const uint8_t *buf, size_t len)
{
    while (len)
    {
        ssize_t n = send(fd, buf, len, MSG_NOSIGNAL);
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
        {
            if (!wait_fd(fd, POLLOUT))
                return false;
            continue;
        }
        if (n <= 0)
            return false;
        buf += n;
        len -= (size_t)n;
    }
    return true;
}

static bool recv_all(int fd, uint8_t *buf, size_t len)
{
    while (len)
    {
        if (!wait_fd(fd, POLLIN))
            return false;
        ssize_t n = recv(fd, buf, len, 0);
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            continue;
        if (n <= 0)
            return false;
        buf += n;
        len -= (size_t)n;
    }
    return true;
}

/* Sends one JSON request and returns the decrypted, NUL-terminated reply (caller frees). */
static char *kasa_request(const char *host, const char *json, char *err, size_t err_size)
{
    size_t   len = strlen(json);
    uint8_t *req = malloc(4 + len);
    uint8_t  hdr[4];
    char    *reply = NULL;
    int      fd;

    if (!req)
    {
        set_err(err, err_size, "out of memory");
        return NULL;
    }
    req[0] = (uint8_t)(len >> 24);
    req[1] = (uint8_t)(len >> 16);
    req[2] = (uint8_t)(len >> 8);
    req[3] = (uint8_t)len;
    memcpy(req + 4, json, len);
    kasa_encrypt(req + 4, len);

    if ((fd = tcp_connect(host, err, err_size)) < 0)
        goto out;

    if (!send_all(fd, req, 4 + len))
    {
        set_err(err, err_size, "send to %s failed", host);
        goto out;
    }
    if (!recv_all(fd, hdr, sizeof(hdr)))
    {
        set_err(err, err_size, "no reply from %s", host);
        goto out;
    }

    uint32_t rlen = (uint32_t)hdr[0] << 24 | (uint32_t)hdr[1] << 16 | (uint32_t)hdr[2] << 8 | hdr[3];
    if (rlen == 0 || rlen > KASA_MAX_REPLY)
    {
        set_err(err, err_size, "bad reply length %u from %s", rlen, host);
        goto out;
    }
    if (!(reply = malloc(rlen + 1)))
    {
        set_err(err, err_size, "out of memory");
        goto out;
    }
    if (!recv_all(fd, (uint8_t *)reply, rlen))
    {
        set_err(err, err_size, "short reply from %s", host);
        free(reply);
        reply = NULL;
        goto out;
    }
    kasa_decrypt((uint8_t *)reply, rlen);
    reply[rlen] = '\0';

out:
    if (fd >= 0)
        close(fd);
    free(req);
    return reply;
}

/* ---------------------------------------------------------------------------
 * JSON field lookup
 * ------------------------------------------------------------------------- */

/* Returns the end of the JSON string starting at the opening quote p (past the closing quote). */
static const char *skip_string(const char *p)
{
    for (p++; *p && *p != '"'; p++)
        if (*p == '\\' && p[1])
            p++;
    return *p ? p + 1 : p;
}

/* Finds key among the direct members of the object starting at obj ('{') and returns a
 * pointer to its value, or NULL. */
static const char *json_member(const char *obj, const char *key)
{
    size_t klen  = strlen(key);
    int    depth = 0;

    if (!obj || *obj != '{')
        return NULL;
    for (const char *p = obj; *p; )
    {
        if (*p == '"')
        {
            const char *end = skip_string(p);
            if (depth == 1 && (size_t)(end - p) == klen + 2 && strncmp(p + 1, key, klen) == 0)
            {
                while (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')
                    end++;
                if (*end == ':')
                {
                    end++;
                    while (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')
                        end++;
                    return end;
                }
            }
            p = end;
            continue;
        }
        if (*p == '{' || *p == '[')
            depth++;
        else if ((*p == '}' || *p == ']') && --depth == 0)
            return NULL;
        p++;
    }
    return NULL;
}

/* Follows a path of object keys, e.g. {"system", "get_sysinfo", NULL}. */
static const char *json_path(const char *json, const char *const *keys)
{
    const char *p = json;
    while (p && *keys)
        p = json_member(p, *keys++);
    return p;
}

static bool json_int(const char *obj, const char *key, int *out)
{
    const char *v = json_member(obj, key);
    char       *end;
    long        n;

    if (!v)
        return false;
    n = strtol(v, &end, 10);
    if (end == v)
        return false;
    *out = (int)n;
    return true;
}

static void json_str(const char *obj, const char *key, char *out, size_t out_size)
{
    const char *v = json_member(obj, key);
    size_t      n = 0;

    if (v && *v == '"')
        for (v++; *v && *v != '"' && n + 1 < out_size; v++)
        {
            if (*v == '\\' && v[1])
                v++;
            out[n++] = *v;
        }
    out[n] = '\0';
}

/* Checks the err_code of a reply section such as system.get_sysinfo. */
static int check_err_code(const char *section, const char *what, char *err, size_t err_size)
{
    int  code = 0;
    char msg[96];

    if (!section)
    {
        set_err(err, err_size, "%s: unexpected reply", what);
        return -1;
    }
    if (json_int(section, "err_code", &code) && code != 0)
    {
        json_str(section, "err_msg", msg, sizeof(msg));
        set_err(err, err_size, "%s: error %d %s", what, code, msg);
        return -1;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Commands
 * ------------------------------------------------------------------------- */

static const char *const SYSINFO_PATH[] = {"system", "get_sysinfo", NULL};
static const char *const LIGHT_PATH[]   = {"smartlife.iot.smartbulb.lightingservice",
                                           "transition_light_state", NULL};

int kasa_get_state(const char *host, kasa_state_t *state, char *err, size_t err_size)
{
    char       *reply = kasa_request(host, "{\"system\":{\"get_sysinfo\":{}}}", err, err_size);
    const char *info, *light, *dft;
    int         v;
    int         rc = -1;

    if (!reply)
        return -1;

    memset(state, 0, sizeof(*state));
    info = json_path(reply, SYSINFO_PATH);
    if (check_err_code(info, "get_sysinfo", err, err_size) != 0)
        goto out;

    json_str(info, "alias", state->alias, sizeof(state->alias));
    json_str(info, "model", state->model, sizeof(state->model));
    if (json_int(info, "is_dimmable", &v))
        state->dimmable = v != 0;

    if (!(light = json_member(info, "light_state")))
    {
        set_err(err, err_size, "%s (%s) is not a Kasa bulb (no light_state)", host, state->model);
        goto out;
    }
    if (json_int(light, "on_off", &v))
        state->on = v != 0;

    /* When the bulb is off, its brightness lives in dft_on_state. */
    state->brightness = 100;
    if (!json_int(light, "brightness", &state->brightness) &&
        (dft = json_member(light, "dft_on_state")))
        json_int(dft, "brightness", &state->brightness);
    rc = 0;

out:
    free(reply);
    return rc;
}

int kasa_set_light(const char *host, bool on, int brightness, int transition_ms, char *err,
                   size_t err_size)
{
    char  req[192];
    char *reply;
    int   rc;

    if (brightness > 0)
        snprintf(req, sizeof(req),
                 "{\"smartlife.iot.smartbulb.lightingservice\":{\"transition_light_state\":"
                 "{\"ignore_default\":1,\"on_off\":%d,\"brightness\":%d,\"transition_period\":%d}}}",
                 on, brightness, transition_ms);
    else
        snprintf(req, sizeof(req),
                 "{\"smartlife.iot.smartbulb.lightingservice\":{\"transition_light_state\":"
                 "{\"ignore_default\":0,\"on_off\":%d,\"transition_period\":%d}}}",
                 on, transition_ms);

    if (!(reply = kasa_request(host, req, err, err_size)))
        return -1;
    rc = check_err_code(json_path(reply, LIGHT_PATH), "transition_light_state", err, err_size);
    free(reply);
    return rc;
}
