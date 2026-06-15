/*
 * arp.c - ARP (Address Resolution Protocol) for IPv4 over Ethernet
 *
 * Maintains an 8-entry cache.  arp_resolve() sends a broadcast request and
 * polls for the reply.  Incoming ARP requests addressed to our IP are
 * answered with a unicast reply.
 */

#include <stdint.h>
#include "netpriv.h"

/* ── ARP cache ───────────────────────────────────────────────────────────── */

#define ARP_CACHE 8

static struct {
    uint32_t ip;   /* host byte order */
    uint8_t  mac[6];
} s_cache[ARP_CACHE];
static int s_nentries = 0;

void arp_cache_clear(void) {
    s_nentries = 0;
}

static int arp_mac_unicast(const uint8_t mac[6]) {
    int any = 0;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) any = 1;
    }
    return any && !(mac[0] & 0x01u);
}

static void cache_put(uint32_t ip, const uint8_t mac[6]) {
    /* Update existing entry if present */
    for (int i = 0; i < s_nentries; i++) {
        if (s_cache[i].ip == ip) {
            nmemcpy(s_cache[i].mac, mac, 6);
            return;
        }
    }
    /* Evict oldest (slot 0) if full */
    int slot = s_nentries < ARP_CACHE ? s_nentries++ : 0;
    s_cache[slot].ip = ip;
    nmemcpy(s_cache[slot].mac, mac, 6);
}

static int cache_get(uint32_t ip, uint8_t mac_out[6]) {
    for (int i = 0; i < s_nentries; i++) {
        if (s_cache[i].ip == ip) {
            nmemcpy(mac_out, s_cache[i].mac, 6);
            return 1;
        }
    }
    return 0;
}

void arp_learn(uint32_t ip, const uint8_t mac[6]) {
    if (!ip || ip == 0xFFFFFFFFu || !arp_mac_unicast(mac)) return;
    cache_put(ip, mac);
}

/* ── Send an ARP packet ──────────────────────────────────────────────────── */

static void arp_send(uint16_t oper, const uint8_t dst_eth[6],
                     const uint8_t target_mac[6], uint32_t target_ip) {
    uint8_t  our_mac[6]; net_mac(our_mac);
    uint32_t our_ip = net_cfg_our_ip();

    uint8_t frame[14 + 28];
    struct eth_hdr *eh = (struct eth_hdr *)frame;
    struct arp_pkt *ap = (struct arp_pkt *)(frame + 14);

    nmemcpy(eh->dst, dst_eth, 6);
    nmemcpy(eh->src, our_mac, 6);
    eh->type = htons(ETH_TYPE_ARP);

    ap->htype = htons(1);
    ap->ptype = htons(0x0800);
    ap->hlen  = 6;
    ap->plen  = 4;
    ap->oper  = htons(oper);
    nmemcpy(ap->sender_mac, our_mac, 6);
    ap->sender_ip[0] = (uint8_t)(our_ip >> 24);
    ap->sender_ip[1] = (uint8_t)(our_ip >> 16);
    ap->sender_ip[2] = (uint8_t)(our_ip >>  8);
    ap->sender_ip[3] = (uint8_t)(our_ip);
    nmemcpy(ap->target_mac, target_mac, 6);
    ap->target_ip[0] = (uint8_t)(target_ip >> 24);
    ap->target_ip[1] = (uint8_t)(target_ip >> 16);
    ap->target_ip[2] = (uint8_t)(target_ip >>  8);
    ap->target_ip[3] = (uint8_t)(target_ip);

    net_send(frame, sizeof(frame));
}

/* ── Process an incoming Ethernet frame for ARP ──────────────────────────── */

void arp_rx(const uint8_t *frame, uint16_t flen) {
    if (flen < 14 + 28) return;
    const struct arp_pkt *ap = (const struct arp_pkt *)(frame + 14);
    if (ntohs(ap->htype) != 1 || ntohs(ap->ptype) != 0x0800) return;

    uint32_t sender_ip = ((uint32_t)ap->sender_ip[0] << 24) |
                         ((uint32_t)ap->sender_ip[1] << 16) |
                         ((uint32_t)ap->sender_ip[2] <<  8) |
                          (uint32_t)ap->sender_ip[3];
    uint32_t target_ip = ((uint32_t)ap->target_ip[0] << 24) |
                         ((uint32_t)ap->target_ip[1] << 16) |
                         ((uint32_t)ap->target_ip[2] <<  8) |
                          (uint32_t)ap->target_ip[3];

    /* Always update cache from sender */
    arp_learn(sender_ip, ap->sender_mac);

    /* If it's a request targeting our IP, send a reply */
    if (ntohs(ap->oper) == 1 && target_ip == net_cfg_our_ip()) {
        uint8_t our_mac[6]; net_mac(our_mac);
        arp_send(2, ap->sender_mac, ap->sender_mac, sender_ip);
        (void)our_mac;
    }
}

/* ── Public: resolve IP → MAC ────────────────────────────────────────────── */

int arp_resolve(uint32_t ip, uint8_t mac_out[6]) {
    if (!net_cfg_our_ip()) return 0;
    if (cache_get(ip, mac_out)) return 1;

    static const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    static const uint8_t zeros[6] = {0};

    /* Wi-Fi broadcast ARP can be lossy; retry instead of failing after one. */
    for (int attempt = 0; attempt < 5; attempt++) {
        arp_send(1, bcast, zeros, ip);
        for (int i = 0; i < 50000000; i++) {
            net_pump();
            if (cache_get(ip, mac_out)) return 1;
            __asm__ volatile ("pause");
        }
    }
    return 0;  /* timeout */
}
