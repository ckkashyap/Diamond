/*
 * nic.h - Internal NIC driver descriptor
 *
 * Each driver defines one static `struct nic_driver X_driver`.
 * net.c iterates the driver table and calls probe() on each until one
 * succeeds.  Not part of the public API — include net.h instead.
 */
#pragma once
#include <stdint.h>

struct nic_driver {
    const char *name;
    /* probe: initialise hardware; return 1 on success, 0 if not present */
    int  (*probe)(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt);
    void (*mac)(uint8_t out[6]);
    int  (*link_up)(void);
    int  (*send)(const void *data, uint16_t len);
    int  (*recv)(void *buf, uint16_t maxlen, uint16_t *len_out);
};

/* Attach a driver that was brought up outside net_init()'s probe loop. */
void net_attach_driver(struct nic_driver *drv);
