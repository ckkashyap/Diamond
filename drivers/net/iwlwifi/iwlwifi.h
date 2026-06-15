/*
 * iwlwifi.h - Public API for the Intel Wi-Fi NIC driver (AX200 / AX201 class).
 *
 * This driver targets Surface Laptop 3's PCI 0:14.3 device [8086:34F0], which
 * is the Ice Lake-LP CNVi MAC.  The actual radio (RF) is on a separate CRF
 * module connected to the PCH via a proprietary link — the driver still
 * sees a single PCI device.
 *
 * Current scope (this commit):
 *   Phase 1 bring-up: PCI probe, BAR mapping (UC), D3→D0, prepare-card-HW
 *   handshake, APM init, MAC-access semaphore, read CSR_HW_REV and MAC from
 *   OTP.  No firmware loading, no TX/RX, no association yet.
 *
 * The driver descriptor registers with net.c's probe loop but its send/recv
 * hooks return -1 until a firmware is loaded and the device is associated.
 */
#pragma once
#include <stdint.h>

/* Probe (PCI find + reset + APM init).  Returns 1 on success. */
int  iwl_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt);

/* Filled with the hardware MAC address.  0-zero if not available. */
void iwl_mac(uint8_t out[6]);

/* Chip hardware revision word (CSR_HW_REV).  Zero if not probed. */
uint32_t iwl_hw_rev(void);

/* Human-readable family name (e.g. "AX201 (QuZ-A0)"). */
const char *iwl_family_name(void);

/* Firmware-blob summary — reflects the TLV parse of the embedded .ucode. */
struct iwl_fw_info;
const struct iwl_fw_info *iwl_fw_info_get(void);

/* Phase 2: load the INIT image into the chip and wait for ALIVE.
 * Must be called after iwl_probe() succeeded.  Returns 1 if ALIVE seen. */
int iwl_fw_load(uint32_t hw_rev, const struct iwl_fw_info *fw);

/* Called by the association path once Wi-Fi is usable as the net backend. */
void iwl_net_attach(void);

/* Stop the device and clear host-side Wi-Fi state before a fresh wifiup. */
void iwl_reset(void);
