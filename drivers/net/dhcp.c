/*
 * dhcp.c - Minimal blocking DHCPv4 client.
 *
 * Sends DISCOVER/REQUEST as raw Ethernet+IPv4+UDP broadcasts so it can run
 * before the IPv4 stack has an address.  On ACK, installs the received IPv4
 * config for ARP, ICMP, TCP, and HTTP.
 */

#include <stdint.h>
#include "netpriv.h"

#define DHCP_CLIENT_PORT 68u
#define DHCP_SERVER_PORT 67u
#define DHCP_MAGIC       0x63825363u
#define DHCP_MIN_PAYLOAD 300u

#define DHCP_OP_BOOTREQUEST 1u
#define DHCP_OP_BOOTREPLY   2u

#define DHCP_MSG_DISCOVER 1u
#define DHCP_MSG_OFFER    2u
#define DHCP_MSG_REQUEST  3u
#define DHCP_MSG_ACK      5u
#define DHCP_MSG_NAK      6u

#define DHCP_OPT_SUBNET_MASK 1u
#define DHCP_OPT_ROUTER      3u
#define DHCP_OPT_DNS         6u
#define DHCP_OPT_HOSTNAME    12u
#define DHCP_OPT_REQ_IP      50u
#define DHCP_OPT_MSG_TYPE    53u
#define DHCP_OPT_SERVER_ID   54u
#define DHCP_OPT_PARAM_REQ   55u
#define DHCP_OPT_CLIENT_ID   61u
#define DHCP_OPT_END         255u

struct dhcp_msg {
    uint8_t  op;
    uint8_t  htype;
    uint8_t  hlen;
    uint8_t  hops;
    uint32_t xid;
    uint16_t secs;
    uint16_t flags;
    uint32_t ciaddr;
    uint32_t yiaddr;
    uint32_t siaddr;
    uint32_t giaddr;
    uint8_t  chaddr[16];
    uint8_t  sname[64];
    uint8_t  file[128];
    uint32_t magic;
    uint8_t  options[60];
} __attribute__((packed));

struct dhcp_offer {
    uint8_t  msg_type;
    uint8_t  server_mac[6];
    uint8_t  have_server_mac;
    uint32_t yiaddr;
    uint32_t src_ip;
    uint32_t server_id;
    uint32_t netmask;
    uint32_t router;
    uint32_t dns;
};

static void wr32be(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static uint32_t rd32be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

static int opt_put(uint8_t *opts, uint16_t *pos, uint16_t max,
                   uint8_t code, const uint8_t *data, uint8_t len) {
    if ((uint16_t)(*pos + 2u + len) > max) return 0;
    opts[(*pos)++] = code;
    opts[(*pos)++] = len;
    for (uint8_t i = 0; i < len; i++) opts[(*pos)++] = data[i];
    return 1;
}

static int opt_put_ip(uint8_t *opts, uint16_t *pos, uint16_t max,
                      uint8_t code, uint32_t ip) {
    uint8_t b[4];
    wr32be(b, ip);
    return opt_put(opts, pos, max, code, b, 4);
}

static uint32_t dhcp_xid(void) {
    uint32_t lo, hi;
    __asm__ volatile ("rdtsc" : "=a"(lo), "=d"(hi));
    return lo ^ (hi << 16) ^ 0xD1A50000u;
}

static int dhcp_send(uint8_t msg_type, uint32_t xid,
                     uint32_t req_ip, uint32_t server_id) {
    uint8_t mac[6];
    net_mac(mac);

    uint8_t frame[14 + 20 + 8 + DHCP_MIN_PAYLOAD];
    nmemset(frame, 0, sizeof(frame));

    struct eth_hdr *eh = (struct eth_hdr *)frame;
    for (int i = 0; i < 6; i++) {
        eh->dst[i] = 0xFF;
        eh->src[i] = mac[i];
    }
    eh->type = htons(ETH_TYPE_IP);

    struct ip_hdr *ih = (struct ip_hdr *)(frame + 14);
    ih->ver_ihl    = 0x45;
    ih->dscp       = 0;
    ih->total_len  = htons((uint16_t)(20 + 8 + DHCP_MIN_PAYLOAD));
    ih->ident      = htons((uint16_t)xid);
    ih->flags_frag = 0;
    ih->ttl        = 64;
    ih->protocol   = IP_PROTO_UDP;
    ih->checksum   = 0;
    ih->src_ip     = 0;
    ih->dst_ip     = htonl(0xFFFFFFFFu);
    ih->checksum   = inet_cksum(ih, 20);

    struct udp_hdr *uh = (struct udp_hdr *)(frame + 14 + 20);
    uh->src_port = htons(DHCP_CLIENT_PORT);
    uh->dst_port = htons(DHCP_SERVER_PORT);
    uh->len      = htons((uint16_t)(8 + DHCP_MIN_PAYLOAD));
    uh->checksum = 0;  /* Optional for IPv4; keep bring-up simple. */

    struct dhcp_msg *m = (struct dhcp_msg *)(frame + 14 + 20 + 8);
    m->op     = DHCP_OP_BOOTREQUEST;
    m->htype  = 1;
    m->hlen   = 6;
    m->xid    = htonl(xid);
    m->flags  = htons(0x8000u);
    for (int i = 0; i < 6; i++) m->chaddr[i] = mac[i];
    m->magic = htonl(DHCP_MAGIC);

    uint8_t *o = m->options;
    uint16_t p = 0;
    uint8_t mt = msg_type;
    if (!opt_put(o, &p, 60, DHCP_OPT_MSG_TYPE, &mt, 1)) return -1;

    uint8_t client_id[7];
    client_id[0] = 1;
    for (int i = 0; i < 6; i++) client_id[1 + i] = mac[i];
    if (!opt_put(o, &p, 60, DHCP_OPT_CLIENT_ID, client_id, sizeof(client_id)))
        return -1;

    static const uint8_t params[] = {
        DHCP_OPT_SUBNET_MASK, DHCP_OPT_ROUTER, DHCP_OPT_DNS
    };
    if (!opt_put(o, &p, 60, DHCP_OPT_PARAM_REQ, params, sizeof(params)))
        return -1;

    static const uint8_t host[] = {'d','i','a','m','o','n','d'};
    if (!opt_put(o, &p, 60, DHCP_OPT_HOSTNAME, host, sizeof(host)))
        return -1;

    if (msg_type == DHCP_MSG_REQUEST) {
        if (!opt_put_ip(o, &p, 60, DHCP_OPT_REQ_IP, req_ip)) return -1;
        if (server_id && !opt_put_ip(o, &p, 60, DHCP_OPT_SERVER_ID, server_id))
            return -1;
    }
    if (p >= 60) return -1;
    o[p++] = DHCP_OPT_END;

    return net_send(frame, sizeof(frame));
}

static void dhcp_parse_opts(const struct dhcp_msg *m, uint16_t len,
                            struct dhcp_offer *out) {
    if (len < 240u) return;
    const uint8_t *p = (const uint8_t *)m + 240u;
    uint16_t rem = (uint16_t)(len - 240u);

    while (rem) {
        uint8_t code = *p++;
        rem--;
        if (code == 0) continue;
        if (code == DHCP_OPT_END) break;
        if (rem < 1) break;
        uint8_t opt_len = *p++;
        rem--;
        if (opt_len > rem) break;

        if (code == DHCP_OPT_MSG_TYPE && opt_len >= 1) {
            out->msg_type = p[0];
        } else if (code == DHCP_OPT_SERVER_ID && opt_len >= 4) {
            out->server_id = rd32be(p);
        } else if (code == DHCP_OPT_SUBNET_MASK && opt_len >= 4) {
            out->netmask = rd32be(p);
        } else if (code == DHCP_OPT_ROUTER && opt_len >= 4) {
            out->router = rd32be(p);
        } else if (code == DHCP_OPT_DNS && opt_len >= 4) {
            out->dns = rd32be(p);
        }

        p += opt_len;
        rem = (uint16_t)(rem - opt_len);
    }
}

static int dhcp_parse_frame(const uint8_t *frame, uint16_t flen,
                            uint32_t xid, struct dhcp_offer *out) {
    if (flen < 14u + 20u + 8u + 240u) return 0;
    const struct eth_hdr *eh = (const struct eth_hdr *)frame;
    if (ntohs(eh->type) != ETH_TYPE_IP) return 0;

    const struct ip_hdr *ih = (const struct ip_hdr *)(frame + 14);
    if ((ih->ver_ihl >> 4) != 4 || ih->protocol != IP_PROTO_UDP) return 0;
    uint8_t ihl = (uint8_t)((ih->ver_ihl & 0x0Fu) << 2);
    if (ihl < 20u) return 0;
    uint16_t ip_total = ntohs(ih->total_len);
    if (ip_total < ihl + 8u + 240u || 14u + ip_total > flen) return 0;

    const struct udp_hdr *uh = (const struct udp_hdr *)((const uint8_t *)ih + ihl);
    if (ntohs(uh->src_port) != DHCP_SERVER_PORT ||
        ntohs(uh->dst_port) != DHCP_CLIENT_PORT)
        return 0;
    uint16_t udp_len = ntohs(uh->len);
    if (udp_len < 8u + 240u || ihl + udp_len > ip_total) return 0;

    const struct dhcp_msg *m =
        (const struct dhcp_msg *)((const uint8_t *)uh + 8u);
    uint16_t dhcp_len = (uint16_t)(udp_len - 8u);
    if (m->op != DHCP_OP_BOOTREPLY || m->htype != 1 ||
        m->hlen != 6 || ntohl(m->xid) != xid ||
        ntohl(m->magic) != DHCP_MAGIC)
        return 0;

    uint8_t mac[6];
    net_mac(mac);
    if (nmemcmp(m->chaddr, mac, 6) != 0) return 0;

    nmemset(out, 0, sizeof(*out));
    out->yiaddr = ntohl(m->yiaddr);
    out->src_ip = ntohl(ih->src_ip);
    nmemcpy(out->server_mac, eh->src, 6);
    out->have_server_mac = 1;
    dhcp_parse_opts(m, dhcp_len, out);
    arp_learn(ntohl(ih->src_ip), eh->src);
    if (out->server_id) arp_learn(out->server_id, eh->src);
    return out->msg_type ? 1 : 0;
}

static int dhcp_wait(uint32_t xid, uint8_t want, struct dhcp_offer *out) {
    uint8_t frame[1600];
    for (uint32_t i = 0; i < 50000000u; i++) {
        uint16_t len = 0;
        if (net_recv(frame, sizeof(frame), &len) == 0) {
            struct dhcp_offer got;
            if (dhcp_parse_frame(frame, len, xid, &got)) {
                if (got.msg_type == DHCP_MSG_NAK) return -1;
                if (got.msg_type == want) {
                    *out = got;
                    return 1;
                }
            }
        }
        __asm__ volatile ("pause");
    }
    return 0;
}

int dhcp_configure(struct net_ipv4_config *cfg_out) {
    if (!net_link_up()) return -1;

    uint32_t xid = dhcp_xid();
    struct dhcp_offer offer;
    int got_offer = 0;
    for (int attempt = 0; attempt < 4 && !got_offer; attempt++) {
        if (dhcp_send(DHCP_MSG_DISCOVER, xid, 0, 0) != 0) return -1;
        int r = dhcp_wait(xid, DHCP_MSG_OFFER, &offer);
        if (r < 0) return 0;
        got_offer = (r > 0);
    }
    if (!got_offer || !offer.yiaddr) return 0;

    struct dhcp_offer ack;
    int got_ack = 0;
    for (int attempt = 0; attempt < 4 && !got_ack; attempt++) {
        if (dhcp_send(DHCP_MSG_REQUEST, xid, offer.yiaddr,
                      offer.server_id) != 0)
            return -1;
        int r = dhcp_wait(xid, DHCP_MSG_ACK, &ack);
        if (r < 0) return 0;
        got_ack = (r > 0);
    }
    if (!got_ack) return 0;

    uint32_t ip      = ack.yiaddr ? ack.yiaddr : offer.yiaddr;
    uint32_t netmask = ack.netmask ? ack.netmask :
                       (offer.netmask ? offer.netmask : CFG_DEFAULT_NETMASK);
    uint32_t router  = ack.router ? ack.router : offer.router;
    uint32_t dns     = ack.dns ? ack.dns : offer.dns;

    net_config_set(ip, netmask, router, dns);
    if (ack.have_server_mac) {
        arp_learn(ack.src_ip, ack.server_mac);
        if (ack.server_id) arp_learn(ack.server_id, ack.server_mac);
    }
    if (cfg_out) net_config_get(cfg_out);
    return 1;
}
