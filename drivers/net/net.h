/*
 * net.h - Generic NIC public API
 *
 * net_init() probes all compiled-in drivers in order; the first one that
 * finds its hardware wins.  All subsequent calls dispatch to that driver.
 */
#pragma once
#include <stdint.h>

/*
 * Probe for a supported NIC.
 * hhdm_offset : value from the Limine HHDM response (phys → virt for MMIO)
 * kphys       : kernel physical base (from Limine kernel-address response)
 * kvirt       : kernel virtual  base (from Limine kernel-address response)
 * Returns 1 if a NIC was found and initialised, 0 otherwise.
 */
int net_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt);

/* Fill mac[6] with the NIC's MAC address (zeros if no NIC). */
void net_mac(uint8_t mac[6]);

/* Return 1 if the link is up, 0 otherwise. */
int net_link_up(void);

/* Transmit len bytes from data.  Returns 0 on success, -1 on error. */
int net_send(const void *data, uint16_t len);

/*
 * Receive one packet into buf (up to maxlen bytes).
 * Sets *len_out to the actual packet length.
 * Returns 0 if a packet was available, -1 if the RX ring was empty.
 */
int net_recv(void *buf, uint16_t maxlen, uint16_t *len_out);

/* Name of the active driver (e.g. "e1000"), or "none" if no NIC found. */
const char *net_driver_name(void);

/* Drop the active NIC binding.  Used by reset paths before re-probing. */
void net_detach_driver(void);
