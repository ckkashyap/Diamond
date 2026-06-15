/*
 * tcp.h - TCP client public API
 */
#pragma once
#include <stdint.h>

typedef struct tcp_conn tcp_conn_t;

#define TCP_CONNECT_NONE        0
#define TCP_CONNECT_OK          1
#define TCP_CONNECT_NO_IP       2
#define TCP_CONNECT_SEND_FAILED 3
#define TCP_CONNECT_TIMEOUT     4
#define TCP_CONNECT_RESET       5
#define TCP_CONNECT_UNEXPECTED  6

#define TCP_CONNECT_DROP_NONE      0
#define TCP_CONNECT_DROP_BAD_HDR   1
#define TCP_CONNECT_DROP_DST_PORT  2
#define TCP_CONNECT_DROP_SRC_PORT  3
#define TCP_CONNECT_DROP_SRC_IP    4
#define TCP_CONNECT_DROP_BAD_ACK   5

struct tcp_connect_diag {
    int      status;
    uint32_t attempts;
    uint32_t rx_candidates;
    uint32_t rx_packets;
    uint32_t last_src_ip;
    uint16_t last_src_port;
    uint16_t last_dst_port;
    uint16_t last_seg_len;
    uint8_t  last_drop;
    uint8_t  last_flags;
    uint32_t last_seq;
    uint32_t last_ack;
    uint16_t src_port;
};

/*
 * Open a TCP connection to dst_ip:dst_port.
 * Returns a connection handle on success, NULL on failure/timeout.
 * dst_ip is in host byte order (e.g. 0x0A000202 = 10.0.2.2).
 */
tcp_conn_t *tcp_connect(uint32_t dst_ip, uint16_t dst_port);

/* Send len bytes; returns 0 on success, -1 on error. */
int tcp_write(tcp_conn_t *c, const void *buf, uint16_t len);

/*
 * Receive up to maxlen bytes into buf.
 * Returns bytes read (>0), 0 on EOF (connection closed), -1 on error/timeout.
 */
int tcp_read(tcp_conn_t *c, void *buf, uint16_t maxlen);

/* Graceful close: sends FIN and waits for peer's FIN. */
void tcp_close(tcp_conn_t *c);

/* Clear the single TCP connection and local ephemeral-port state. */
void tcp_reset(void);

/* Details for the most recent tcp_connect() attempt. */
void tcp_last_connect_diag(struct tcp_connect_diag *diag);
