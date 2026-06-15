/*
 * ipv4.c - IPv4 send/receive, ICMP echo, and the receive pump
 *
 * net_pump() is the single drain point: it calls net_recv() in a loop,
 * dispatching each Ethernet frame to the appropriate handler.  Every
 * blocking operation in the stack (ARP resolve, TCP connect/read) calls
 * net_pump() internally.
 */

#include <stdint.h>
#include "netpriv.h"

/* ── State ───────────────────────────────────────────────────────────────── */

static uint16_t s_ip_id = 1;
static uint32_t s_cfg_our_ip  = CFG_DEFAULT_OUR_IP;
static uint32_t s_cfg_gateway = CFG_DEFAULT_GW_IP;
static uint32_t s_cfg_netmask = CFG_DEFAULT_NETMASK;
static uint32_t s_cfg_dns     = 0;

/* ICMP echo reply tracking */
static volatile int      s_icmp_waiting = 0;
static volatile uint16_t s_icmp_expect_id;
static volatile uint16_t s_icmp_expect_seq;
static volatile int      s_icmp_got_reply = 0;

uint32_t net_cfg_our_ip(void)  { return s_cfg_our_ip; }
uint32_t net_cfg_gateway(void) { return s_cfg_gateway; }
uint32_t net_cfg_netmask(void) { return s_cfg_netmask; }
uint32_t net_cfg_dns(void)     { return s_cfg_dns; }

void net_config_set(uint32_t ip, uint32_t netmask,
                    uint32_t gateway, uint32_t dns) {
    s_cfg_our_ip  = ip;
    s_cfg_netmask = netmask ? netmask : CFG_DEFAULT_NETMASK;
    s_cfg_gateway = gateway;
    s_cfg_dns     = dns;
    arp_cache_clear();
}

void net_config_get(struct net_ipv4_config *cfg) {
    if (!cfg) return;
    cfg->ip      = s_cfg_our_ip;
    cfg->netmask = s_cfg_netmask;
    cfg->gateway = s_cfg_gateway;
    cfg->dns     = s_cfg_dns;
}

void net_stack_reset(void) {
    s_ip_id = 1;
    s_icmp_waiting = 0;
    s_icmp_expect_id = 0;
    s_icmp_expect_seq = 0;
    s_icmp_got_reply = 0;
    net_config_set(0, CFG_DEFAULT_NETMASK, 0, 0);
    tcp_reset();
}

/* ── Receive pump ────────────────────────────────────────────────────────── */

void net_pump(void) {
    uint8_t  frame[2048];
    uint16_t flen;
    while (net_recv(frame, sizeof(frame), &flen) == 0 && flen >= 14) {
        uint16_t etype = (uint16_t)((frame[12] << 8) | frame[13]);
        if (etype == ETH_TYPE_ARP) {
            arp_rx(frame, flen);
        } else if (etype == ETH_TYPE_IP && flen >= 14 + 20) {
            const struct ip_hdr *ih = (const struct ip_hdr *)(frame + 14);
            uint8_t ihl = (ih->ver_ihl & 0x0Fu) << 2;
            if (ihl < 20 || ihl > flen - 14u) continue;
            uint16_t ip_total = ntohs(ih->total_len);
            if (ip_total < ihl || ip_total > flen - 14u) continue;
            if (inet_cksum(ih, ihl) != 0) continue;
            uint32_t src_ip = ntohl(ih->src_ip);
            arp_learn(src_ip, frame + 6);
            const uint8_t *payload = (const uint8_t *)ih + ihl;
            uint16_t plen = (uint16_t)(ip_total - ihl);
            if (ih->protocol == IP_PROTO_ICMP) {
                icmp_rx(src_ip, payload, plen);
            } else if (ih->protocol == IP_PROTO_TCP) {
                tcp_rx(src_ip, payload, plen);
            }
        }
    }
}

/* ── IP send ─────────────────────────────────────────────────────────────── */

int ip_send(uint32_t dst_ip, uint8_t proto,
            const void *payload, uint16_t plen) {
    uint32_t our_ip = net_cfg_our_ip();
    uint32_t mask   = net_cfg_netmask();
    /* Route: same subnet → ARP direct, otherwise via gateway */
    uint32_t next_hop = ((dst_ip & mask) == (our_ip & mask))
                        ? dst_ip : net_cfg_gateway();
    if (!our_ip || !next_hop) return -1;

    uint8_t dst_mac[6];
    if (!arp_resolve(next_hop, dst_mac)) return -1;  /* ARP failed */

    uint8_t  our_mac[6]; net_mac(our_mac);
    uint16_t ip_total = (uint16_t)(20 + plen);

    /* Build frame: Eth(14) + IP(20) + payload */
    uint8_t frame[14 + 20 + 1500];
    if (plen > 1480) return -1;  /* oversized */

    struct eth_hdr *eh = (struct eth_hdr *)frame;
    struct ip_hdr  *ih = (struct ip_hdr  *)(frame + 14);

    nmemcpy(eh->dst, dst_mac, 6);
    nmemcpy(eh->src, our_mac, 6);
    eh->type = htons(ETH_TYPE_IP);

    ih->ver_ihl    = 0x45;
    ih->dscp       = 0;
    ih->total_len  = htons(ip_total);
    ih->ident      = htons(s_ip_id++);
    ih->flags_frag = htons(0x4000);  /* DF */
    ih->ttl        = 64;
    ih->protocol   = proto;
    ih->checksum   = 0;
    ih->src_ip     = htonl(our_ip);
    ih->dst_ip     = htonl(dst_ip);
    ih->checksum   = inet_cksum(ih, 20);

    nmemcpy(frame + 34, payload, plen);
    return net_send(frame, (uint16_t)(14 + 20 + plen));
}

/* ── ICMP ────────────────────────────────────────────────────────────────── */

void icmp_rx(uint32_t src_ip, const uint8_t *pkt, uint16_t len) {
    if (len < 8) return;
    if (inet_cksum(pkt, len) != 0) return;
    const struct icmp_hdr *ih = (const struct icmp_hdr *)pkt;

    /* Echo reply (type=0): signal waiting ping */
    if (ih->type == 0 && s_icmp_waiting) {
        if (ntohs(ih->ident) == s_icmp_expect_id &&
            ntohs(ih->seq)   == s_icmp_expect_seq) {
            (void)src_ip;
            s_icmp_got_reply = 1;
        }
    }

    /* Echo request (type=8): send reply */
    if (ih->type == 8) {
        uint8_t reply[1500];
        if (len > sizeof(reply)) return;
        nmemcpy(reply, pkt, len);
        struct icmp_hdr *rh = (struct icmp_hdr *)reply;
        rh->type     = 0;
        rh->checksum = 0;
        rh->checksum = inet_cksum(reply, len);
        (void)ip_send(src_ip, IP_PROTO_ICMP, reply, len);
    }
}

/* ── Public: ICMP ping ───────────────────────────────────────────────────── */

int ping(uint32_t dst_ip) {
    static uint16_t s_seq = 0;
    uint16_t ident = 0xD1A5;  /* "DIAS" */
    uint16_t seq   = ++s_seq;

    /* Build one echo request and retransmit it if needed.  Keeping the same
     * ICMP sequence lets a delayed reply to any attempt satisfy the command. */
    uint8_t pkt[16];
    struct icmp_hdr *ih = (struct icmp_hdr *)pkt;
    nmemset(pkt + 8, 0xAB, 8);  /* 8 bytes of payload */
    ih->type     = 8;
    ih->code     = 0;
    ih->checksum = 0;
    ih->ident    = htons(ident);
    ih->seq      = htons(seq);
    ih->checksum = inet_cksum(pkt, sizeof(pkt));

    net_pump();  /* drain stale traffic from a previous command first */

    s_icmp_expect_id  = ident;
    s_icmp_expect_seq = seq;
    s_icmp_got_reply  = 0;
    s_icmp_waiting    = 1;

    int sent_any = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (ip_send(dst_ip, IP_PROTO_ICMP, pkt, sizeof(pkt)) == 0) {
            sent_any = 1;
            for (int i = 0; i < 100000000; i++) {
                net_pump();
                if (s_icmp_got_reply) break;
                __asm__ volatile ("pause");
            }
            if (s_icmp_got_reply) {
                s_icmp_waiting = 0;
                return 1;
            }
        }
    }

    s_icmp_waiting = 0;
    return sent_any ? -1 : -2;
}
