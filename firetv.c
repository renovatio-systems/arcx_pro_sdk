/*
 * firetv.c - see firetv.h
 */

#include "firetv.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define API_KEY "0987654321" /* fixed key used by the Fire TV remote app */

/* Runs curl, returns the HTTP status (or -1) and the response body in out. */
static int run_curl(char *const argv[], char *out, size_t out_size)
{
    int pfd[2];
    if (pipe(pfd) != 0)
        return -1;

    pid_t pid = fork();
    if (pid < 0)
        return -1;
    if (pid == 0)
    {
        dup2(pfd[1], STDOUT_FILENO);
        close(pfd[0]);
        close(pfd[1]);
        execvp("curl", argv);
        _exit(127);
    }
    close(pfd[1]);

    char   buf[2048];
    size_t n = 0;
    ssize_t r;
    while ((r = read(pfd[0], buf + n, sizeof(buf) - 1 - n)) > 0)
    {
        n += (size_t)r;
        if (n >= sizeof(buf) - 1)
            break;
    }
    close(pfd[0]);
    buf[n] = '\0';

    int status;
    waitpid(pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0 || n < 3)
        return -1;

    /* The last 3 characters are the status code written by -w. */
    int code = atoi(buf + n - 3);
    buf[n - 3] = '\0';
    snprintf(out, out_size, "%s", buf);
    return code;
}

static int request(const char *method, const char *url, const char *token, const char *body,
                   char *resp, size_t resp_size, char *err, size_t err_size)
{
    char *argv[32];
    int   i = 0;
    argv[i++] = "curl";
    argv[i++] = "-s";
    argv[i++] = "-k";
    argv[i++] = "--max-time";
    argv[i++] = "5";
    argv[i++] = "-w";
    argv[i++] = "%{http_code}";
    argv[i++] = "-X";
    argv[i++] = (char *)method;
    argv[i++] = "-H";
    argv[i++] = "X-Api-Key: " API_KEY;
    argv[i++] = "-H";
    argv[i++] = "user-agent: okhttp/4.10.0";

    char tokhdr[160];
    if (token)
    {
        snprintf(tokhdr, sizeof(tokhdr), "X-Client-Token: %s", token);
        argv[i++] = "-H";
        argv[i++] = tokhdr;
    }
    if (body)
    {
        argv[i++] = "-H";
        argv[i++] = "Content-Type: application/json; charset=utf-8";
        argv[i++] = "-d";
        argv[i++] = (char *)body;
    }
    argv[i++] = (char *)url;
    argv[i]   = NULL;

    char out[2048];
    int  code = run_curl(argv, out, sizeof(out));
    if (resp)
        snprintf(resp, resp_size, "%.*s", (int)resp_size - 1, code < 0 ? "" : out);
    if (code < 200 || code >= 300)
    {
        if (code < 0)
            snprintf(err, err_size, "no response from %s (is curl installed and the TV awake?)", url);
        else
            snprintf(err, err_size, "HTTP %d from %s: %.300s", code, url, out);
        return -1;
    }
    return 0;
}

int firetv_wake(const char *host, char *err, size_t err_size)
{
    char url[160];
    snprintf(url, sizeof(url), "http://%s:8009/apps/FireTVRemote", host);
    return request("POST", url, NULL, NULL, NULL, 0, err, err_size);
}

int firetv_pin_display(const char *host, const char *friendly_name, char *err, size_t err_size)
{
    char url[160], body[160];
    snprintf(url, sizeof(url), "https://%s:8080/v1/FireTV/pin/display", host);
    snprintf(body, sizeof(body), "{\"friendlyName\":\"%s\"}", friendly_name);
    return request("POST", url, NULL, body, NULL, 0, err, err_size);
}

int firetv_pin_verify(const char *host, const char *pin, char *token, size_t token_size,
                      char *err, size_t err_size)
{
    char url[160], body[64], resp[512];
    snprintf(url, sizeof(url), "https://%s:8080/v1/FireTV/pin/verify", host);
    snprintf(body, sizeof(body), "{\"pin\":\"%s\"}", pin);
    if (request("POST", url, NULL, body, resp, sizeof(resp), err, err_size) != 0)
        return -1;

    /* Response: {"description":"<token>"} */
    const char *p = strstr(resp, "\"description\"");
    p = p ? strchr(p + 13, '"') : NULL;
    const char *q = p ? strchr(p + 1, '"') : NULL;
    if (!q || (size_t)(q - p - 1) >= token_size || q == p + 1)
    {
        snprintf(err, err_size, "unexpected pin/verify response: %s", resp);
        return -1;
    }
    memcpy(token, p + 1, (size_t)(q - p - 1));
    token[q - p - 1] = '\0';
    return 0;
}

int firetv_send(const char *host, const char *token, const char *action, char *err,
                size_t err_size)
{
    char url[200];

    if (!strcmp(action, "play"))
    {
        snprintf(url, sizeof(url), "https://%s:8080/v1/media?action=play", host);
        return request("POST", url, token, NULL, NULL, 0, err, err_size);
    }
    if (!strcmp(action, "scan_forward") || !strcmp(action, "scan_back"))
    {
        snprintf(url, sizeof(url), "https://%s:8080/v1/media?action=scan", host);
        const char *body = !strcmp(action, "scan_forward")
            ? "{\"direction\":\"forward\",\"keyAction\":{\"keyActionType\":\"keyDown\"}}"
            : "{\"direction\":\"back\",\"keyAction\":{\"keyActionType\":\"keyDown\"}}";
        return request("POST", url, token, body, NULL, 0, err, err_size);
    }
    snprintf(url, sizeof(url), "https://%s:8080/v1/FireTV?action=%s", host, action);
    return request("POST", url, token, NULL, NULL, 0, err, err_size);
}

static void token_path(const char *host, char *path, size_t size)
{
    const char *home = getenv("HOME");
    snprintf(path, size, "%s/.config/arcx/firetv_%s.token", home ? home : ".", host);
}

int firetv_token_load(const char *host, char *token, size_t token_size)
{
    char path[512];
    token_path(host, path, sizeof(path));
    FILE *f = fopen(path, "r");
    if (!f)
        return -1;
    int ok = fgets(token, (int)token_size, f) != NULL;
    fclose(f);
    token[strcspn(token, "\r\n")] = '\0';
    return ok && token[0] ? 0 : -1;
}

int firetv_token_save(const char *host, const char *token)
{
    char path[512], dir[512];
    const char *home = getenv("HOME");
    snprintf(dir, sizeof(dir), "%s/.config", home ? home : ".");
    mkdir(dir, 0700);
    snprintf(dir, sizeof(dir), "%s/.config/arcx", home ? home : ".");
    mkdir(dir, 0700);
    token_path(host, path, sizeof(path));
    FILE *f = fopen(path, "w");
    if (!f)
        return -1;
    fprintf(f, "%s\n", token);
    fclose(f);
    chmod(path, 0600);
    return 0;
}
