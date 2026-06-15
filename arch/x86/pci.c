/*
 * pci.c - PCI configuration space access via I/O ports 0xCF8 / 0xCFC
 */

#include <stdint.h>
#include "io.h"
#include "pci.h"

#define PCI_ADDR 0x0CF8u
#define PCI_DATA 0x0CFCu

static uint32_t pci_addr(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg) {
    return (1u << 31)
         | ((uint32_t)bus << 16)
         | ((uint32_t)(dev & 0x1f) << 11)
         | ((uint32_t)(fn  & 0x07) <<  8)
         | (reg & 0xfc);          /* 4-byte aligned */
}

uint32_t pci_read32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg) {
    outl(PCI_ADDR, pci_addr(bus, dev, fn, reg));
    return inl(PCI_DATA);
}

void pci_write32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg, uint32_t v) {
    outl(PCI_ADDR, pci_addr(bus, dev, fn, reg));
    outl(PCI_DATA, v);
}

uint16_t pci_read16(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg) {
    uint32_t dword = pci_read32(bus, dev, fn, reg & ~3u);
    return (uint16_t)(dword >> ((reg & 2u) * 8u));
}

void pci_write16(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg, uint16_t v) {
    uint32_t dword = pci_read32(bus, dev, fn, reg & ~3u);
    unsigned shift = (reg & 2u) * 8u;
    dword = (dword & ~(0xffffu << shift)) | ((uint32_t)v << shift);
    pci_write32(bus, dev, fn, reg & ~3u, dword);
}

int pci_find(uint16_t vendor, uint16_t device,
             uint8_t *bus_out, uint8_t *dev_out, uint8_t *fn_out) {
    return pci_find_next(vendor, device, 0xFF, bus_out, dev_out, fn_out);
}

int pci_find_next(uint16_t vendor, uint16_t device,
                  uint8_t after_dev,
                  uint8_t *bus_out, uint8_t *dev_out, uint8_t *fn_out) {
    /* Scan bus 0 only — sufficient for QEMU's flat topology.
     * after_dev=0xFF means start from the beginning. */
    uint8_t start = (after_dev == 0xFF) ? 0 : (uint8_t)(after_dev + 1u);
    for (uint8_t d = start; d < 32; d++) {
        for (uint8_t f = 0; f < 8; f++) {
            uint32_t id = pci_read32(0, d, f, 0x00);
            if ((id & 0xffff) == vendor && (id >> 16) == device) {
                *bus_out = 0;
                *dev_out = d;
                *fn_out  = f;
                return 1;
            }
            if (f == 0) {
                uint8_t hdr = (uint8_t)(pci_read32(0, d, 0, 0x0c) >> 16);
                if (!(hdr & 0x80)) break;
            }
        }
    }
    return 0;
}
