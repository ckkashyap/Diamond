/*
 * netpriv.h - Internal header for the TCP/IP stack
 *
 * All IP addresses are uint32_t in HOST byte order throughout the stack.
 * htonl/htons convert to network byte order only when writing packet fields.
 */
#pragma once
#include <stdint.h>
#include "net.h"
#include "netstack.h"

/* ── Byte-order ──────────────────────────────────────────────────────────── */

static inline uint16_t swap16(uint16_t x) { return (uint16_t)((x>>8)|(x<<8)); }
static inline uint32_t swap32(uint32_t x) {
    return ((x>>24)&0xFF)|((x>>8)&0xFF00)|((x<<8)&0xFF0000)|((x<<24)&0xFF000000);
}
#define htons(x) swap16((uint16_t)(x))
#define ntohs(x) swap16((uint16_t)(x))
#define htonl(x) swap32((uint32_t)(x))
#define ntohl(x) swap32((uint32_t)(x))

/* ── Network config defaults (QEMU user-mode defaults) ───────────────────── */

#define CFG_DEFAULT_OUR_IP  0x0A00020Fu  /* 10.0.2.15  host-byte-order */
#define CFG_DEFAULT_GW_IP   0x0A000202u  /* 10.0.2.2   host-byte-order */
#define CFG_DEFAULT_NETMASK 0xFFFFFF00u

/* ── Ethernet ────────────────────────────────────────────────────────────── */

#define ETH_TYPE_IP   0x0800u
#define ETH_TYPE_ARP  0x0806u

/* ── IPv4 protocol numbers ───────────────────────────────────────────────── */

#define IP_PROTO_ICMP 1u
#define IP_PROTO_TCP  6u
#define IP_PROTO_UDP  17u

/* ── Packet header structs (packed, network byte order in wire fields) ───── */

struct eth_hdr {
    uint8_t  dst[6];
    uint8_t  src[6];
    uint16_t type;
} __attribute__((packed));

struct ip_hdr {
    uint8_t  ver_ihl;     /* 0x45 = version 4, IHL 5 */
    uint8_t  dscp;
    uint16_t total_len;
    uint16_t ident;
    uint16_t flags_frag;  /* 0x4000 = DF */
    uint8_t  ttl;
    uint8_t  protocol;
    uint16_t checksum;
    uint32_t src_ip;      /* network byte order */
    uint32_t dst_ip;
} __attribute__((packed));

struct arp_pkt {
    uint16_t htype;         /* 1 = Ethernet */
    uint16_t ptype;         /* 0x0800 = IPv4 */
    uint8_t  hlen;          /* 6 */
    uint8_t  plen;          /* 4 */
    uint16_t oper;          /* 1=request, 2=reply */
    uint8_t  sender_mac[6];
    uint8_t  sender_ip[4];
    uint8_t  target_mac[6];
    uint8_t  target_ip[4];
} __attribute__((packed));

struct udp_hdr {
    uint16_t src_port;
    uint16_t dst_port;
    uint16_t len;
    uint16_t checksum;
} __attribute__((packed));

struct icmp_hdr {
    uint8_t  type;
    uint8_t  code;
    uint16_t checksum;
    uint16_t ident;
    uint16_t seq;
} __attribute__((packed));

struct tcp_hdr {
    uint16_t src_port;
    uint16_t dst_port;
    uint32_t seq;
    uint32_t ack;
    uint8_t  data_off;   /* header_len_in_words << 4 */
    uint8_t  flags;
    uint16_t window;
    uint16_t checksum;
    uint16_t urgent;
} __attribute__((packed));

/* TCP flags */
#define TCP_FIN  0x01u
#define TCP_SYN  0x02u
#define TCP_RST  0x04u
#define TCP_PSH  0x08u
#define TCP_ACK  0x10u

/* ── Checksums ───────────────────────────────────────────────────────────── */

static inline uint16_t inet_cksum(const void *data, int len) {
    const uint16_t *p = (const uint16_t *)data;
    uint32_t sum = 0;
    while (len > 1) { sum += *p++; len -= 2; }
    if (len) sum += *(const uint8_t *)p;
    while (sum >> 16) sum = (sum & 0xffffu) + (sum >> 16);
    return (uint16_t)~sum;
}

/* TCP checksum via pseudo-header (src/dst in HOST byte order) */
static inline uint16_t tcp_cksum(uint32_t src, uint32_t dst,
                                  const void *seg, uint16_t seg_len) {
    /* Pseudo-header bytes: sum word-by-word to avoid packed-alignment warning */
    uint32_t s = htonl(src), d = htonl(dst);
    uint16_t l = htons(seg_len);
    uint32_t sum = 0;
    /* src IP */
    sum += (uint16_t)( s >> 16);
    sum += (uint16_t)( s & 0xFFFFu);
    /* dst IP */
    sum += (uint16_t)( d >> 16);
    sum += (uint16_t)( d & 0xFFFFu);
    /* zero + proto + length */
    sum += htons(IP_PROTO_TCP);
    sum += l;
    /* TCP segment */
    const uint16_t *p = (const uint16_t *)seg;
    int len = seg_len;
    while (len > 1) { sum += *p++; len -= 2; }
    if (len) sum += *(const uint8_t *)p;
    while (sum >> 16) sum = (sum & 0xffffu) + (sum >> 16);
    return (uint16_t)~sum;
}

/* ── Internal cross-module calls ─────────────────────────────────────────── */

/* ipv4.c: send an IP packet (builds Eth+IP, resolves MAC) */
int ip_send(uint32_t dst_ip, uint8_t proto,
            const void *payload, uint16_t plen);

/* ipv4.c: pump rx ring, dispatch to arp_rx / ip_rx */
void net_pump(void);

uint32_t net_cfg_our_ip(void);
uint32_t net_cfg_gateway(void);
uint32_t net_cfg_netmask(void);
uint32_t net_cfg_dns(void);

/* arp.c: process an incoming Ethernet frame (full frame including eth hdr) */
void arp_rx(const uint8_t *frame, uint16_t flen);

void arp_learn(uint32_t ip, const uint8_t mac[6]);
void arp_cache_clear(void);

/* tcp.c: process an incoming TCP segment */
void tcp_rx(uint32_t src_ip, const uint8_t *seg, uint16_t seg_len);

/* icmp (inline in ipv4.c): handle echo request / echo reply */
void icmp_rx(uint32_t src_ip, const uint8_t *pkt, uint16_t len);

/* ── Helpers (inline, no stdlib) ─────────────────────────────────────────── */

static inline void *nmemcpy(void *d, const void *s, int n) {
    uint8_t *dd = (uint8_t *)d;
    const uint8_t *ss = (const uint8_t *)s;
    while (n--) *dd++ = *ss++;
    return d;
}
static inline void *nmemset(void *d, int c, int n) {
    uint8_t *dd = (uint8_t *)d;
    while (n--) *dd++ = (uint8_t)c;
    return d;
}
static inline int nmemcmp(const void *a, const void *b, int n) {
    const uint8_t *aa = (const uint8_t *)a, *bb = (const uint8_t *)b;
    while (n--) { if (*aa != *bb) return *aa - *bb; aa++; bb++; }
    return 0;
}
