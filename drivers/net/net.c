/*
 * net.c - NIC driver probe loop
 *
 * Iterates the driver table and calls each driver's probe() until one
 * succeeds.  All public net_* calls then dispatch to that driver.
 */

#include <stdint.h>
#include "nic.h"
#include "net.h"

/* ── Driver declarations ─────────────────────────────────────────────────── */
/* Each driver defines one struct nic_driver in its own translation unit. */

extern struct nic_driver e1000_driver;
extern struct nic_driver e1000e_driver;
extern struct nic_driver virtio_net_driver;
extern struct nic_driver rtl8139_driver;
extern struct nic_driver pcnet_driver;
extern struct nic_driver ne2k_driver;

/* Probe order: most-capable / most-common first.
 * iwlwifi is NOT in this list while it's under development — a hang in its
 * probe would leave Diamond unable to reach the shell.  Instead the user
 * invokes it on demand via the `wifiinit` shell command.                  */
static struct nic_driver * const s_drivers[] = {
    &e1000_driver,
    &e1000e_driver,
    &virtio_net_driver,
    &rtl8139_driver,
    &pcnet_driver,
    &ne2k_driver,
};
#define N_DRIVERS ((int)(sizeof(s_drivers) / sizeof(s_drivers[0])))

/* ── Active driver ───────────────────────────────────────────────────────── */

static struct nic_driver *s_active = (void *)0;

/* ── Public API ──────────────────────────────────────────────────────────── */

int net_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    for (int i = 0; i < N_DRIVERS; i++) {
        if (s_drivers[i]->probe(hhdm_offset, kphys, kvirt)) {
            s_active = s_drivers[i];
            return 1;
        }
    }
    return 0;
}

void net_attach_driver(struct nic_driver *drv) {
    s_active = drv;
}

void net_detach_driver(void) {
    s_active = (void *)0;
}

const char *net_driver_name(void) {
    return s_active ? s_active->name : "none";
}

void net_mac(uint8_t mac[6]) {
    if (s_active) { s_active->mac(mac); return; }
    for (int i = 0; i < 6; i++) mac[i] = 0;
}

int net_link_up(void) {
    return s_active ? s_active->link_up() : 0;
}

int net_send(const void *data, uint16_t len) {
    return s_active ? s_active->send(data, len) : -1;
}

int net_recv(void *buf, uint16_t maxlen, uint16_t *len_out) {
    return s_active ? s_active->recv(buf, maxlen, len_out) : -1;
}
