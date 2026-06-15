/*
 * iwl_cmd.h - Host-firmware command queue + RX response dispatch.
 *
 * The chip exposes a single 32-entry TFD (Transmit Frame Descriptor) ring
 * for host-issued commands.  Each command payload is a pointer to a
 * struct iwl_device_cmd in DMA-coherent memory; the TFD references that
 * buffer via one or two TBs (Transmit Buffers).  Firmware processes the
 * command and posts a response back into our RX ring.
 *
 * Phase 4 scope:
 *   1. Allocate the TFD ring + per-slot command buffers
 *   2. Wire the ring into context_info.hcmd_cfg
 *   3. iwl_send_cmd(group, cmd, payload, len) — synchronous
 *   4. Drain the RX ring after each command, match seq# to find response
 */
#pragma once
#include <stdint.h>

/* Long-format TFD entries (Linux iwl-fh.h). */
struct iwl_tfh_tb {
    uint16_t tb_len;
    /* NOTE: addr is *unaligned* 64-bit per Linux spec — copy with memcpy. */
    uint8_t  addr[8];
} __attribute__((packed));

struct iwl_tfh_tfd {
    uint16_t           num_tbs;
    struct iwl_tfh_tb  tbs[25];
    uint32_t           pad;
} __attribute__((packed));        /* sizeof = 256 */

/* Wide command header — gen2 host commands (Linux fw/api/cmdhdr.h). */
struct iwl_cmd_header_wide {
    uint8_t  cmd;
    uint8_t  group_id;
    uint16_t sequence;
    uint16_t length;       /* payload bytes after the 8-byte header */
    uint8_t  reserved;
    uint8_t  version;
} __attribute__((packed));

/* RX packet header (Linux iwl-trans.h). */
struct iwl_cmd_header_short {
    uint8_t  cmd;
    uint8_t  group_id;
    uint16_t sequence;
} __attribute__((packed));

struct iwl_rx_packet {
    uint32_t                       len_n_flags;
    struct iwl_cmd_header_short    hdr;
    uint8_t                        data[];
} __attribute__((packed));

/* RB status writeback (Linux iwl-fh.h iwl_rb_status). */
struct iwl_rb_status {
    uint16_t closed_rb_num;        /* low 12 bits = producer index */
    uint16_t closed_fr_num;
    uint16_t finished_rb_num;
    uint16_t finished_fr_num;
    uint32_t reserved;
} __attribute__((packed));

/* ── Phase 4 API (called from iwl_fw_load.c after ALIVE) ──────────────── */

/* Build the TFD ring + per-slot command buffers.  Must run before
 * ctxt_init writes hcmd_cfg.cmd_queue_addr / size.
 * Returns 1 on success, 0 on alloc failure. */
int  iwl_cmd_q_alloc(uint64_t hhdm);

/* Fill the ctxt_info hcmd_cfg fields with the just-allocated TFD ring. */
void iwl_cmd_q_publish(uint64_t *out_phys, uint8_t *out_size_log);

/* Submit a host command and wait (~200 ms) for a response.  Returns the
 * length of the response payload (bytes after the 8-byte iwl_rx_packet
 * header) on success; -1 on timeout / RX error.
 *
 * `payload` may be NULL if `payload_len` is 0.  If `resp_buf` is non-NULL
 * the response payload is copied (clamped to resp_max bytes).            */
int  iwl_send_cmd(uint8_t group_id, uint8_t cmd_id,
                  const void *payload, uint16_t payload_len,
                  void *resp_buf,    uint16_t resp_max);

/* Single-shot host-command smoke test.  Uses Linux's first post-ALIVE
 * command (INIT_EXTENDED_CFG_CMD), not ECHO_CMD: this firmware appears
 * to assert on legacy ECHO before the init flow is established. */
int  iwl_cmd_echo_test(void);

/* Probe the next Linux init phase: unified init through NVM_GET_INFO. */
int  iwl_cmd_nvm_probe(void);

/* Probe the first regular-runtime setup commands after NVM init. */
int  iwl_cmd_runtime_probe(void);

/* Toward-scan setup: DQA_ENABLE + PHY_CONTEXT_CMD ADD ctx=0.
 * Auto-runs iwl_cmd_runtime_probe() if not already done. */
int  iwl_cmd_scan_setup_probe(void);

/* Issue a passive 2.4 GHz scan (channels 1, 6, 11) via SCAN_REQ_UMAC v14
 * and wait for SCAN_COMPLETE_UMAC.  Auto-runs iwl_cmd_scan_setup_probe()
 * if not already done.  Prints per-RX events along the way. */
int  iwl_cmd_scan_probe(void);

/* Allocate a 16-slot TX TFD ring + bc_tbl, register the AUX STA, and bind
 * the queue to it via SCD_QUEUE_CONFIG_CMD v3.  Firmware uses this queue
 * to TX scan probe-requests built from the SCAN_REQ_UMAC preq template;
 * the host never enqueues a TFD here directly.  Auto-runs scan setup. */
int  iwl_cmd_txq_setup_probe(void);

/* Discovered-AP cache, populated by iwl_cmd_scan_probe() from beacons and
 * probe-responses parsed during the scan.  Up to 16 unique BSSIDs.  Used
 * by `wifilist` for display and by `wificonnect` to look up a target. */
struct iwl_ap_info {
    uint8_t  bssid[6];
    uint8_t  channel;
    uint8_t  rssi;          /* positive dBm magnitude (energy_a) */
    uint16_t capability;    /* 16-bit capability_info from mgmt body */
    uint16_t beacon_interval; /* beacon interval in TUs */
    uint8_t  ssid_len;
    uint8_t  ssid[32];
    uint8_t  rates_len;
    uint8_t  rates[16];     /* Supported Rates + Extended Supported Rates */
    uint8_t  rsn_ie_len;
    uint8_t  rsn_ie[64];    /* Full RSN IE, including id/len */
};

/* Returns count of APs in the cache (0..16). */
int  iwl_ap_count(void);
/* Returns pointer to entry `i` (read-only), or NULL if i >= count. */
const struct iwl_ap_info *iwl_ap_get(int i);

/* Begin association with a cached AP matching `ssid` (length `ssid_len`).
 * Performs Linux-derived MLD link/STA/session-protection setup, allocates an
 * AP TX queue, TXes Open System AUTH, then TXes an Association Request and
 * waits for the AP's Association Response.  If `pass_len` is nonzero, also
 * derives the WPA2-PSK PMK/PTK and responds to EAPOL message 1/4 with M2.
 * Returns 1 if association succeeds, even if later WPA diagnostics are still
 * incomplete. */
int  iwl_cmd_assoc_probe(const char *ssid, uint8_t ssid_len,
                         const char *pass, uint8_t pass_len);

/* Ethernet-like hooks used by iwlwifi.c's nic_driver glue after WPA completes. */
int  iwl_cmd_net_link_up(void);
int  iwl_cmd_net_send(const void *data, uint16_t len);
int  iwl_cmd_net_recv(void *buf, uint16_t maxlen, uint16_t *len_out);

/* Clear host-command, scan, association, WPA, and data-path state. */
void iwl_cmd_reset_state(void);
