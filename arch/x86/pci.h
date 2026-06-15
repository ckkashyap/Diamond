/*
 * pci.h - PCI configuration space access (I/O port mechanism #1)
 */
#pragma once
#include <stdint.h>

/* 32-bit config space read/write (register must be 4-byte aligned). */
uint32_t pci_read32 (uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg);
void     pci_write32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg, uint32_t v);

/* 16-bit helpers (read-modify-write internally). */
uint16_t pci_read16 (uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg);
void     pci_write16(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg, uint16_t v);

/*
 * Scan bus 0 (sufficient for QEMU) for a device matching vendor:device.
 * Fills bus_out, dev_out, fn_out and returns 1 if found, 0 otherwise.
 */
int pci_find(uint16_t vendor, uint16_t device,
             uint8_t *bus_out, uint8_t *dev_out, uint8_t *fn_out);

/*
 * Like pci_find but starts scanning after after_dev (pass 0xFF to start from 0).
 * Allows finding multiple devices with the same vendor:device ID.
 */
int pci_find_next(uint16_t vendor, uint16_t device,
                  uint8_t after_dev,
                  uint8_t *bus_out, uint8_t *dev_out, uint8_t *fn_out);
