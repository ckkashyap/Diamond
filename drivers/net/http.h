/*
 * http.h - HTTP/1.0 GET client public API
 */
#pragma once
#include <stdint.h>

/*
 * Perform an HTTP GET request.
 *
 *   server_ip  — destination IP in host byte order
 *   port       — TCP port (usually 80)
 *   host       — Host header value (hostname or IP string)
 *   path       — Request path, e.g. "/"
 *   resp_buf   — Buffer to receive the full HTTP response (headers + body)
 *   resp_max   — Size of resp_buf
 *   resp_len   — Set to number of bytes written (may be NULL)
 *
 * Returns the HTTP status code (e.g. 200), or -1 on connection error.
 */
int http_get(uint32_t server_ip, uint16_t port,
             const char *host, const char *path,
             char *resp_buf, uint32_t resp_max, uint32_t *resp_len);
