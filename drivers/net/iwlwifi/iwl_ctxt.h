/*
 * iwl_ctxt.h - Intel Wi-Fi "context info" structure (gen2 variant).
 *
 * Subset of Linux's drivers/net/wireless/intel/iwlwifi/iwl-context-info.h
 * — the master structure the AX201 silicon DMA-reads during bootstrap.
 * We build one copy of this in DMA-coherent memory and write its physical
 * address to CSR_CTXT_INFO_BA; the hardware then pulls the firmware
 * section binaries into its internal RAM, boots, and posts the ALIVE
 * event onto our RX queue.
 */
#pragma once
#include <stdint.h>

/* ── CSR registers used in the context-info load handshake ─────────────── */
/* Physical-address latch for the iwl_context_info struct.  64-bit register
 * written as two 32-bit halves.                                          */
#define CSR_CTXT_INFO_BA         0x40u

/* AUTO_FUNC_INIT bit in CSR_GP_CNTRL — kicks the firmware load once the
 * context info BA has been latched.                                      */
#define CSR_AUTO_FUNC_INIT_BIT   (1u << 7)

/* RB/RBD size encoding (Linux enum iwl_context_info_rb_size).  Nibble
 * that goes into control_flags[11:8]; NOT log2 of the byte size.       */
#define IWL_CTXT_INFO_RB_SIZE_1K    0x1u
#define IWL_CTXT_INFO_RB_SIZE_2K    0x2u
#define IWL_CTXT_INFO_RB_SIZE_4K    0x4u    /* <-- was 6, wrong */
#define IWL_CTXT_INFO_RB_SIZE_8K    0x8u

/* RB control-buffer size = ilog2(num_rbds), in bits [7:4] of control_flags.
 * AX201/HR uses Linux's IWL_NUM_RBDS_HE = 256 * 8 = 2048 RBDs.           */
#define IWL_CTXT_INFO_RB_CB_SIZE_256   8u
#define IWL_CTXT_INFO_RB_CB_SIZE_512   9u
#define IWL_CTXT_INFO_RB_CB_SIZE_2048  11u

/* Control-flag layout (Linux iwl-context-info.h enum iwl_context_info_flags) */
#define IWL_CTXT_INFO_AUTO_FUNC_INIT   0x0001u   /* bit 0 */
#define IWL_CTXT_INFO_EARLY_DEBUG      0x0002u   /* bit 1 */
#define IWL_CTXT_INFO_RB_CB_SIZE_MASK  0x00F0u   /* bits[7:4] */
#define IWL_CTXT_INFO_TFD_FORMAT_LONG  0x0100u   /* bit 8  — NOT bit 0 */
#define IWL_CTXT_INFO_RB_SIZE_MASK     0x1E00u   /* bits[12:9] */

#define IWL_CTXT_INFO_RB_CB_SIZE_SHIFT  4
#define IWL_CTXT_INFO_RB_SIZE_SHIFT     9        /* mask starts at bit 9 */

#define IWL_CTXT_INFO_RB_SIZE(x)     ((uint32_t)(x) << IWL_CTXT_INFO_RB_SIZE_SHIFT)
#define IWL_CTXT_INFO_RB_CB_SIZE(x)  ((uint32_t)(x) << IWL_CTXT_INFO_RB_CB_SIZE_SHIFT)

/* DRAM image section capacity per image (Linux's IWL_PCIE_MAX_FRAGS_PER_CB
 * — 64 entries).  We never come close on AX201's firmware.             */
#define IWL_MAX_DRAM_ENTRY   64u

/* ── iwl_context_info_* on-disk layout (gen2) ──────────────────────────── */
struct iwl_context_info_version {
    uint16_t mac_id;
    uint16_t version;
    uint16_t size;       /* size of the whole iwl_context_info in 32-bit words */
    uint16_t reserved;
} __attribute__((packed));

struct iwl_context_info_control {
    uint32_t control_flags;
    uint32_t reserved;
} __attribute__((packed));

struct iwl_context_info_rbd_cfg {
    uint64_t free_rbd_addr;    /* RBD "free list" ring   */
    uint64_t used_rbd_addr;    /* RBD "used list" ring   */
    uint64_t status_wr_ptr;    /* rb_stts: device-written progress */
} __attribute__((packed));

struct iwl_context_info_hcmd_cfg {
    uint64_t cmd_queue_addr;
    uint8_t  cmd_queue_size;
    uint8_t  reserved[7];
} __attribute__((packed));

struct iwl_context_info_dump_cfg {
    uint64_t core_dump_addr;
    uint32_t core_dump_size;
    uint32_t reserved;
} __attribute__((packed));

struct iwl_context_info_early_dbg_cfg {
    uint64_t early_debug_addr;
    uint32_t early_debug_size;
    uint32_t reserved;
} __attribute__((packed));

struct iwl_context_info_pnvm_cfg {
    uint64_t platform_nvm_addr;
    uint32_t platform_nvm_size;
    uint32_t reserved;
} __attribute__((packed));

struct iwl_context_info_dram {
    uint64_t umac_img[IWL_MAX_DRAM_ENTRY];
    uint64_t lmac_img[IWL_MAX_DRAM_ENTRY];
    uint64_t virtual_img[IWL_MAX_DRAM_ENTRY];
} __attribute__((packed));

/* Layout from Linux pcie/iwl-context-info.h:
 *   version       @0x00 ( 8B)
 *   control       @0x08 ( 8B)
 *   reserved0     @0x10 ( 8B)
 *   rbd_cfg       @0x18 (24B)
 *   hcmd_cfg      @0x30 (16B)
 *   reserved1[4]  @0x40 (16B)
 *   dump_cfg      @0x50 (16B)
 *   edbg_cfg      @0x60 (16B)
 *   pnvm_cfg      @0x70 (16B)
 *   reserved2[16] @0x80 (64B)
 *   dram          @0xC0
 *   reserved3[16] @end of dram
 */
struct iwl_context_info {
    struct iwl_context_info_version       version;     /* 0x00 */
    struct iwl_context_info_control       control;     /* 0x08 */
    uint64_t                              reserved0;   /* 0x10 */
    struct iwl_context_info_rbd_cfg       rbd_cfg;     /* 0x18 */
    struct iwl_context_info_hcmd_cfg      hcmd_cfg;    /* 0x30 */
    uint32_t                              reserved1[4];/* 0x40 */
    struct iwl_context_info_dump_cfg      dump_cfg;    /* 0x50 */
    struct iwl_context_info_early_dbg_cfg edbg_cfg;    /* 0x60 */
    struct iwl_context_info_pnvm_cfg      pnvm_cfg;    /* 0x70 */
    uint32_t                              reserved2[16];/* 0x80 */
    struct iwl_context_info_dram          dram;        /* 0xC0 */
    uint32_t                              reserved3[16];/* end */
} __attribute__((packed));

typedef char iwl_context_info_size_must_match_linux[
    (sizeof(struct iwl_context_info) == 1792u) ? 1 : -1
];

/* ── RX ring data structures ──────────────────────────────────────────── */
/* On 22000 MQ-RX, free RBD entries are RB phys | 12-bit virtual RB ID;
 * used RBD entries report that ID back.  The status writeback tells the
 * host how many used entries the device produced.  RFH base addresses
 * are page-aligned for the context-info validation path. */

#define IWL_RX_RING_SIZE   2048u
#define IWL_RX_RING_MASK   (IWL_RX_RING_SIZE - 1u)
#define IWL_RX_INITIAL_WRITE_PTR    (IWL_RX_RING_SIZE - 1u)
#define IWL_RX_INITIAL_WRITE_ACTUAL (IWL_RX_RING_SIZE - 8u)
#define IWL_RX_RB_SIZE     4096u
