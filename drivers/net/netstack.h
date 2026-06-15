/*
 * netstack.h - TCP/IP stack public API
 *
 * Build on top of the raw NIC layer (net.h).  Call after net_init().
 *
 * All IP addresses are uint32_t in host byte order:
 *   10.0.2.2  → 0x0A000202
 *   1.1.1.1   → 0x01010101
 */
#pragma once
#include <stdint.h>
#include "tcp.h"
#include "http.h"

struct net_ipv4_config {
    uint32_t ip;       /* host byte order */
    uint32_t netmask;  /* host byte order */
    uint32_t gateway;  /* host byte order */
    uint32_t dns;      /* host byte order, 0 if unknown */
};

void net_config_set(uint32_t ip, uint32_t netmask,
                    uint32_t gateway, uint32_t dns);
void net_config_get(struct net_ipv4_config *cfg);

/* Clear IPv4, ARP, ICMP, and TCP runtime state. */
void net_stack_reset(void);

/* Run a blocking DHCPv4 discover/request exchange and install the result. */
int dhcp_configure(struct net_ipv4_config *cfg_out);

/* Resolve an IPv4 address to a MAC address, sending ARP if needed. */
int arp_resolve(uint32_t ip, uint8_t mac_out[6]);

/*
 * Send ICMP echo request to dst_ip; poll for reply.
 * Returns 1 on reply received, -1 on timeout, -2 if send/ARP failed.
 */
int ping(uint32_t dst_ip);
