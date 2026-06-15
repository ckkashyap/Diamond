/*
 * http.c - HTTP/1.0 GET client
 *
 * Sends a minimal HTTP/1.0 GET request and reads the response.
 * The full response (headers + body) is stored in resp_buf.
 * Connection: close causes the server to close after the response,
 * so we read until tcp_read returns 0 (EOF).
 */

#include <stdint.h>
#include "netpriv.h"
#include "tcp.h"
#include "http.h"

/* Freestanding string helpers */
static int s_strlen(const char *s)  { int n=0; while(s[n]) n++; return n; }
static void s_strcat(char *d, const char *s) {
    while (*d) d++;
    while ((*d++ = *s++));
}
static void s_strcpy(char *d, const char *s) { while ((*d++ = *s++)); }

/*
 * http_get - perform HTTP GET and return the full response in resp_buf.
 * Returns the HTTP status code (e.g. 200), or -1 on connection failure.
 */
int http_get(uint32_t server_ip, uint16_t port,
             const char *host, const char *path,
             char *resp_buf, uint32_t resp_max, uint32_t *resp_len) {
    if (resp_max < 2) return -1;
    resp_buf[0] = '\0';

    /* Build request */
    char req[512];
    s_strcpy(req, "GET ");
    s_strcat(req, path);
    s_strcat(req, " HTTP/1.0\r\nHost: ");
    s_strcat(req, host);
    s_strcat(req, "\r\nUser-Agent: DiamondOS/1.0\r\nConnection: close\r\n\r\n");
    uint16_t req_len = (uint16_t)s_strlen(req);

    tcp_conn_t *c = tcp_connect(server_ip, port);
    if (!c) return -1;

    if (tcp_write(c, req, req_len) != 0) {
        tcp_close(c);
        return -1;
    }

    /* Read response until EOF */
    uint32_t total = 0;
    int      n;
    while ((n = tcp_read(c, resp_buf + total,
                          (uint16_t)(resp_max - 1 - total))) > 0) {
        total += (uint32_t)n;
        if (total >= resp_max - 1) break;
    }
    tcp_close(c);
    resp_buf[total] = '\0';
    if (resp_len) *resp_len = total;

    /* Parse status code from "HTTP/1.x NNN ..." */
    int status = -1;
    const char *p = resp_buf;
    /* Skip "HTTP/" and version */
    if (p[0]=='H' && p[1]=='T' && p[2]=='T' && p[3]=='P' && p[4]=='/') {
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
        status = 0;
        while (*p >= '0' && *p <= '9') { status = status*10 + (*p - '0'); p++; }
    }
    return status;
}
