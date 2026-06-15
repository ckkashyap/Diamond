/*
 * tcp.c - Minimal TCP client state machine
 *
 * Supports one connection at a time.  All operations are blocking/polling;
 * they call net_pump() in a tight loop until the expected state is reached
 * or a timeout fires.
 *
 * Sequence number space is maintained correctly.  A 4-byte MSS option is
 * included in the SYN to advertise MSS=1460.  Window is fixed at 8 KiB.
 *
 * State machine (client-side only):
 *   CLOSED → SYN_SENT → ESTABLISHED ⇄ CLOSE_WAIT → LAST_ACK → CLOSED
 *                                    ↘ FIN_WAIT_1 → FIN_WAIT_2 → CLOSED
 */

#include <stdint.h>
#include "netpriv.h"
#include "tcp.h"

/* ── TCP states ──────────────────────────────────────────────────────────── */
#define ST_CLOSED      0
#define ST_SYN_SENT    1
#define ST_ESTABLISHED 2
#define ST_FIN_WAIT_1  3
#define ST_FIN_WAIT_2  4
#define ST_CLOSE_WAIT  5
#define ST_LAST_ACK    6
#define ST_TIME_WAIT   7

/* ── Single static connection ────────────────────────────────────────────── */
struct tcp_conn {
    int      state;
    uint32_t src_ip, dst_ip;
    uint16_t src_port, dst_port;
    uint32_t snd_seq;   /* next byte we will send */
    uint32_t snd_una;   /* oldest unacknowledged byte */
    uint32_t rcv_nxt;   /* next byte we expect from peer */
    int      peer_fin;  /* received FIN from peer */
    /* Circular receive buffer */
    uint8_t  rxbuf[8192];
    uint32_t rx_head;   /* read cursor */
    uint32_t rx_tail;   /* write cursor */
};

static struct tcp_conn s_conn;
static struct tcp_connect_diag s_diag;

#define RX_MASK  (sizeof(s_conn.rxbuf) - 1u)

static uint32_t rx_avail(void) { return s_conn.rx_tail - s_conn.rx_head; }
static void rx_put(const uint8_t *d, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        s_conn.rxbuf[s_conn.rx_tail & RX_MASK] = d[i];
        s_conn.rx_tail++;
    }
}
static uint32_t rx_get(uint8_t *out, uint32_t max) {
    uint32_t av = rx_avail();
    if (av > max) av = max;
    for (uint32_t i = 0; i < av; i++) {
        out[i] = s_conn.rxbuf[s_conn.rx_head & RX_MASK];
        s_conn.rx_head++;
    }
    return av;
}

/* ── Build and send a TCP segment ────────────────────────────────────────── */

static uint16_t s_next_port = 49152;

static int tcp_send_seg(uint8_t flags, const void *data, uint16_t dlen,
                         int with_mss) {
    uint8_t seg[20 + 4 + 1460];
    uint8_t hdr_len = (uint8_t)(with_mss ? 24 : 20);
    uint16_t seg_len = (uint16_t)(hdr_len + dlen);
    if (dlen > 1460) return -1;

    struct tcp_hdr *th = (struct tcp_hdr *)seg;
    th->src_port  = htons(s_conn.src_port);
    th->dst_port  = htons(s_conn.dst_port);
    th->seq       = htonl(s_conn.snd_seq);
    th->ack       = (flags & TCP_ACK) ? htonl(s_conn.rcv_nxt) : 0;
    th->data_off  = (uint8_t)((hdr_len / 4u) << 4);
    th->flags     = flags;
    th->window    = htons(8192);
    th->checksum  = 0;
    th->urgent    = 0;

    if (with_mss) {
        /* MSS option: kind=2, len=4, mss=1460 */
        seg[20] = 0x02; seg[21] = 0x04;
        seg[22] = 0x05; seg[23] = 0xB4;  /* 0x05B4 = 1460 */
    }

    if (dlen) nmemcpy(seg + hdr_len, data, dlen);

    th->checksum = tcp_cksum(s_conn.src_ip, s_conn.dst_ip, seg, seg_len);
    return ip_send(s_conn.dst_ip, IP_PROTO_TCP, seg, seg_len);
}

/* ── Incoming TCP segment handler (called from net_pump via ipv4.c) ──────── */

void tcp_rx(uint32_t src_ip, const uint8_t *seg, uint16_t seg_len) {
    if (seg_len < 20) return;
    const struct tcp_hdr *th = (const struct tcp_hdr *)seg;

    if (s_conn.state == ST_CLOSED) return;
    if (tcp_cksum(src_ip, net_cfg_our_ip(), seg, seg_len) != 0) return;
    uint8_t  flags   = th->flags;
    uint32_t seq     = ntohl(th->seq);
    uint32_t ack_val = ntohl(th->ack);
    uint16_t src_port = ntohs(th->src_port);
    uint16_t dst_port = ntohs(th->dst_port);
    uint8_t  hdr_len = (uint8_t)((th->data_off >> 4) * 4u);

    s_diag.rx_candidates++;
    s_diag.last_src_ip = src_ip;
    s_diag.last_src_port = src_port;
    s_diag.last_dst_port = dst_port;
    s_diag.last_seg_len = seg_len;
    s_diag.last_drop = TCP_CONNECT_DROP_NONE;
    s_diag.last_flags = flags;
    s_diag.last_seq = seq;
    s_diag.last_ack = ack_val;

    if (hdr_len < 20 || hdr_len > seg_len) {
        s_diag.last_drop = TCP_CONNECT_DROP_BAD_HDR;
        return;
    }
    if (dst_port != s_conn.src_port) {
        s_diag.last_drop = TCP_CONNECT_DROP_DST_PORT;
        return;
    }
    if (src_port != s_conn.dst_port) {
        s_diag.last_drop = TCP_CONNECT_DROP_SRC_PORT;
        return;
    }
    if (src_ip != s_conn.dst_ip) {
        s_diag.last_drop = TCP_CONNECT_DROP_SRC_IP;
        return;
    }

    uint16_t data_len = (uint16_t)(seg_len - hdr_len);
    const uint8_t *data = seg + hdr_len;

    s_diag.rx_packets++;
    s_diag.last_flags = flags;
    s_diag.last_seq = seq;
    s_diag.last_ack = ack_val;

    /* RST: close unconditionally */
    if (flags & TCP_RST) {
        s_diag.status = TCP_CONNECT_RESET;
        s_conn.state = ST_CLOSED;
        return;
    }

    switch (s_conn.state) {

    case ST_SYN_SENT:
        if ((flags & (TCP_SYN|TCP_ACK)) == (TCP_SYN|TCP_ACK) &&
            ack_val == s_conn.snd_seq) {
            s_conn.rcv_nxt = seq + 1;
            s_conn.snd_una = ack_val;
            s_conn.snd_seq = ack_val;  /* our next send = peer's ack */
            s_conn.state   = ST_ESTABLISHED;
            s_diag.status  = TCP_CONNECT_OK;
            (void)tcp_send_seg(TCP_ACK, (void*)0, 0, 0);  /* complete handshake */
        } else {
            if ((flags & (TCP_SYN|TCP_ACK)) == (TCP_SYN|TCP_ACK))
                s_diag.last_drop = TCP_CONNECT_DROP_BAD_ACK;
            s_diag.status = TCP_CONNECT_UNEXPECTED;
        }
        break;

    case ST_ESTABLISHED:
    case ST_CLOSE_WAIT:
        /* ACK: advance snd_una */
        if (flags & TCP_ACK)
            if ((int32_t)(ack_val - s_conn.snd_una) > 0)
                s_conn.snd_una = ack_val;

        /* Data */
        if (data_len > 0 && seq == s_conn.rcv_nxt) {
            /* Only buffer if space permits */
            if (rx_avail() + data_len <= sizeof(s_conn.rxbuf)) {
                rx_put(data, data_len);
                s_conn.rcv_nxt += data_len;
            }
            (void)tcp_send_seg(TCP_ACK, (void*)0, 0, 0);
        }

        /* FIN from peer */
        if ((flags & TCP_FIN) && s_conn.state == ST_ESTABLISHED) {
            s_conn.rcv_nxt++;          /* FIN consumes one seq number */
            s_conn.peer_fin = 1;
            s_conn.state    = ST_CLOSE_WAIT;
            (void)tcp_send_seg(TCP_ACK, (void*)0, 0, 0);
        }
        break;

    case ST_FIN_WAIT_1:
        if (flags & TCP_ACK) s_conn.snd_una = ack_val;
        if (flags & TCP_FIN) {
            s_conn.rcv_nxt++;
            (void)tcp_send_seg(TCP_ACK, (void*)0, 0, 0);
            s_conn.state = ST_CLOSED;   /* skip TIME_WAIT for simplicity */
        } else if ((int32_t)(ack_val - s_conn.snd_seq) >= 0) {
            s_conn.state = ST_FIN_WAIT_2;
        }
        break;

    case ST_FIN_WAIT_2:
        if (flags & TCP_FIN) {
            s_conn.rcv_nxt++;
            (void)tcp_send_seg(TCP_ACK, (void*)0, 0, 0);
            s_conn.state = ST_CLOSED;
        }
        break;

    case ST_LAST_ACK:
        if (flags & TCP_ACK) {
            s_conn.state = ST_CLOSED;
        }
        break;

    default: break;
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void tcp_reset(void) {
    nmemset(&s_conn, 0, sizeof(s_conn));
    nmemset(&s_diag, 0, sizeof(s_diag));
    s_next_port = 49152;
}

void tcp_last_connect_diag(struct tcp_connect_diag *diag) {
    if (!diag) return;
    *diag = s_diag;
}

tcp_conn_t *tcp_connect(uint32_t dst_ip, uint16_t dst_port) {
    /* Reset state */
    nmemset(&s_conn, 0, sizeof(s_conn));
    nmemset(&s_diag, 0, sizeof(s_diag));
    s_conn.src_ip   = net_cfg_our_ip();
    if (!s_conn.src_ip) {
        s_diag.status = TCP_CONNECT_NO_IP;
        return (void*)0;
    }
    s_conn.dst_ip   = dst_ip;
    s_conn.src_port = s_next_port++;
    s_conn.dst_port = dst_port;
    s_diag.src_port = s_conn.src_port;

    /* Random-ish ISN from TSC */
    uint32_t isn;
    __asm__ volatile ("rdtsc" : "=a"(isn) :: "edx");
    s_conn.snd_seq  = isn;
    s_conn.snd_una  = isn;
    s_conn.state    = ST_SYN_SENT;

    /* Send SYN (with MSS option), retransmitting for real Wi-Fi latency/loss. */
    for (int attempt = 0; attempt < 3; attempt++) {
        s_diag.attempts = (uint32_t)(attempt + 1);
        s_conn.snd_seq = isn;
        if (tcp_send_seg(TCP_SYN, (void*)0, 0, 1) != 0) {
            s_diag.status = TCP_CONNECT_SEND_FAILED;
            s_conn.state = ST_CLOSED;
            return (void*)0;
        }
        s_conn.snd_seq = isn + 1;  /* SYN consumes one seq number */

        for (int i = 0; i < 50000000; i++) {
            net_pump();
            if (s_conn.state == ST_ESTABLISHED) return &s_conn;
            if (s_conn.state == ST_CLOSED)      return (void*)0;
            __asm__ volatile ("pause");
        }
    }
    s_diag.status = TCP_CONNECT_TIMEOUT;
    s_conn.state = ST_CLOSED;
    return (void*)0;
}

int tcp_write(tcp_conn_t *c, const void *buf, uint16_t len) {
    if (!c || c->state != ST_ESTABLISHED) return -1;

    const uint8_t *p   = (const uint8_t *)buf;
    uint16_t        rem = len;

    while (rem > 0) {
        uint16_t chunk = (rem > 1460) ? 1460 : rem;
        uint32_t seq_before = c->snd_seq;
        int acked = 0;

        for (int attempt = 0; attempt < 3 && !acked; attempt++) {
            c->snd_seq = seq_before;
            if (tcp_send_seg(TCP_PSH | TCP_ACK, p, chunk, 0) != 0)
                return -1;
            c->snd_seq = seq_before + chunk;

            /* Wait for ACK */
            for (int i = 0; i < 50000000; i++) {
                net_pump();
                if ((int32_t)(c->snd_una - (seq_before + chunk)) >= 0) {
                    acked = 1;
                    break;
                }
                if (c->state != ST_ESTABLISHED) return -1;
                __asm__ volatile ("pause");
            }
        }
        if (!acked) return -1;

        p   += chunk;
        rem -= chunk;
    }
    return 0;
}

int tcp_read(tcp_conn_t *c, void *buf, uint16_t maxlen) {
    if (!c) return -1;

    /* Drain any already-buffered data first */
    uint32_t got = rx_get((uint8_t *)buf, maxlen);
    if (got > 0) return (int)got;

    /* Connection already closed by peer and buffer empty */
    if (c->peer_fin || c->state == ST_CLOSE_WAIT ||
        c->state == ST_CLOSED) return 0;

    /* Poll for incoming data or FIN */
    for (int i = 0; i < 50000000; i++) {
        net_pump();
        got = rx_get((uint8_t *)buf, maxlen);
        if (got > 0) return (int)got;
        if (c->peer_fin || c->state == ST_CLOSE_WAIT ||
            c->state == ST_CLOSED) return 0;
        __asm__ volatile ("pause");
    }
    return -1;  /* timeout */
}

void tcp_close(tcp_conn_t *c) {
    if (!c || c->state == ST_CLOSED) return;

    if (c->state == ST_ESTABLISHED) {
        (void)tcp_send_seg(TCP_FIN | TCP_ACK, (void*)0, 0, 0);
        c->snd_seq++;
        c->state = ST_FIN_WAIT_1;
        /* Wait for close to complete */
        for (int i = 0; i < 5000000; i++) {
            net_pump();
            if (c->state == ST_CLOSED) return;
            __asm__ volatile ("pause");
        }
    } else if (c->state == ST_CLOSE_WAIT) {
        (void)tcp_send_seg(TCP_FIN | TCP_ACK, (void*)0, 0, 0);
        c->snd_seq++;
        c->state = ST_LAST_ACK;
        for (int i = 0; i < 5000000; i++) {
            net_pump();
            if (c->state == ST_CLOSED) return;
            __asm__ volatile ("pause");
        }
    }
    c->state = ST_CLOSED;
}
