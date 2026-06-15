/*
 * iwl_fw_load.c - Bootstrap firmware upload for AX201 (gen2 context info).
 *
 * This is the AX201/22000-series equivalent of what Linux does in
 * drivers/net/wireless/intel/iwlwifi/pcie/ctxt-info.c.  On this silicon
 * the driver doesn't stream firmware bytes through the Flow Handler
 * registers itself; instead:
 *
 *   1. Allocate DMA-coherent memory for each firmware section.
 *   2. memcpy the section bytes into that DMA memory.
 *   3. Build a single `struct iwl_context_info` in DMA-coherent memory
 *      containing arrays of physical pointers to those section buffers
 *      plus config for the RX ring and command queue.
 *   4. Set up a minimal RX ring (RBD list + RB pool + status writeback).
 *   5. Write the 64-bit PA of the context_info into CSR_CTXT_INFO_BA and
 *      pulse CSR_AUTO_FUNC_INIT_BIT.
 *   6. The firmware boots autonomously.  When it's ready it posts an
 *      ALIVE event (type 0x00, group 0x00, cmd 0x01) onto the first RX
 *      buffer; we poll the rb_stts writeback for the producer index to
 *      move.
 *
 * This first cut only targets the INIT image, since ALIVE is all we need
 * as proof that the upload worked.  REGULAR image bring-up happens after
 * we've processed ALIVE.
 */

#include <stdint.h>
#include "../../../arch/x86/vm.h"
#include "../../../drivers/terminal.h"
#include "../../../kernel/alloc.h"
#include "iwl_csr.h"
#include "iwl_ctxt.h"
#include "iwl_fw.h"
#include "iwl_cmd.h"

/* Driver globals from iwlwifi.c — we borrow the CSR base + HHDM offset. */
extern volatile uint8_t *iwl_csr_base(void);
extern uint64_t          iwl_hhdm(void);
extern void              iwl_delay(uint32_t usecs);
extern int               iwl_grab_nic_access_ext(void);
extern void              iwl_release_nic_access_ext(void);

/* Kernel-level hex printers (we mirror the two-char ph8 used elsewhere). */
static void fwph8(uint8_t v) {
    static const char h[] = "0123456789abcdef";
    term_putchar(h[v >> 4]); term_putchar(h[v & 0xf]);
}
static void fwph32(uint32_t v) {
    fwph8((uint8_t)(v >> 24)); fwph8((uint8_t)(v >> 16));
    fwph8((uint8_t)(v >>  8)); fwph8((uint8_t)(v));
}
static void fwph64(uint64_t v) {
    fwph32((uint32_t)(v >> 32)); fwph32((uint32_t)v);
}

/* ── Aligned DMA-coherent allocation ──────────────────────────────────── */
/* Diamond's kalloc doesn't provide alignment guarantees, so over-allocate
 * and round up.  The returned virtual pointer is HHDM-mapped (WB) and its
 * physical address is simply virt - hhdm.                                */
static void *dma_alloc(uint64_t size, uint64_t align, uint64_t *phys_out) {
    uint8_t *raw = (uint8_t *)kmalloc(size + align);
    if (!raw) return (void *)0;
    uint64_t addr = (uint64_t)(uintptr_t)raw;
    uint64_t aligned = (addr + align - 1) & ~(align - 1);
    /* zero-init */
    for (uint64_t i = 0; i < size; i++) ((uint8_t *)aligned)[i] = 0;
    if (phys_out) *phys_out = aligned - iwl_hhdm();
    return (void *)(uintptr_t)aligned;
}

static void fw_zero(void *p, uint64_t n) {
    uint8_t *b = (uint8_t *)p;
    for (uint64_t i = 0; i < n; i++) b[i] = 0;
}

/* ── RX ring state (single queue, polled) ─────────────────────────────── */
static uint64_t  *s_rx_free_rbd;
static uint64_t  *s_rx_used_rbd;
static uint32_t  *s_rx_stts;
static uint8_t  **s_rx_rb;

static uint64_t   s_rx_free_rbd_phys;
static uint64_t   s_rx_used_rbd_phys;
static uint64_t   s_rx_stts_phys;

static struct iwl_context_info *s_ctxt;
static uint64_t                 s_ctxt_phys;

/* SRAM pointers captured from the ALIVE notification.  Firmware writes
 * structured assert information here on SW_ERR; iwl_dump_fw_error() reads
 * them back via HBUS_TARG_MEM_RADDR/RDAT. */
static uint32_t s_lmac_err_ptr;
static uint32_t s_umac_err_ptr;
uint32_t iwl_lmac_err_ptr(void) { return s_lmac_err_ptr; }
uint32_t iwl_umac_err_ptr(void) { return s_umac_err_ptr; }

/* Accessors used by iwl_cmd.c so it can reach into our RX ring state
 * without exposing the underlying statics. */
uint64_t *iwl_rx_free_rbd(void) { return s_rx_free_rbd; }
uint64_t *iwl_rx_used_rbd(void) { return s_rx_used_rbd; }
uint8_t **iwl_rx_rb(void)       { return s_rx_rb; }
uint32_t *iwl_rx_stts_raw(void) { return s_rx_stts; }

void iwl_fw_reset_state(void) {
    s_lmac_err_ptr = 0;
    s_umac_err_ptr = 0;
    if (s_rx_free_rbd)
        fw_zero(s_rx_free_rbd, IWL_RX_RING_SIZE * 8u);
    if (s_rx_used_rbd)
        fw_zero(s_rx_used_rbd, IWL_RX_RING_SIZE * 8u);
    if (s_rx_stts)
        fw_zero(s_rx_stts, 4096);
    if (s_rx_rb) {
        for (uint32_t i = 0; i < IWL_RX_RING_SIZE; i++) {
            if (s_rx_rb[i])
                fw_zero(s_rx_rb[i], IWL_RX_RB_SIZE);
        }
    }
    if (s_ctxt)
        fw_zero(s_ctxt, sizeof(*s_ctxt));
}

/* ── CSR access helpers (duplicated here to avoid exposing iwlwifi.c internals) */
static inline uint32_t creg_r32(uint32_t off) {
    return *(volatile uint32_t *)(iwl_csr_base() + off);
}
static inline void creg_w32(uint32_t off, uint32_t v) {
    *(volatile uint32_t *)(iwl_csr_base() + off) = v;
}

/* PRPH indirect access.  Linux uses 0x000FFFFF mask for Qu (20 bits).   */
static void prph_w32(uint32_t addr, uint32_t val) {
    creg_w32(HBUS_TARG_PRPH_WADDR, (addr & PRPH_ADDR_MASK_QU) | PRPH_WR_ENABLE_BYTE3);
    creg_w32(HBUS_TARG_PRPH_WDAT, val);
}
static uint32_t prph_r32(uint32_t addr) {
    creg_w32(HBUS_TARG_PRPH_RADDR, (addr & PRPH_ADDR_MASK_QU) | PRPH_WR_ENABLE_BYTE3);
    return creg_r32(HBUS_TARG_PRPH_RDAT);
}

/* Dump the first ~64 bytes of RB[0].  Avoid post-ALIVE PRPH peeks here:
 * the firmware is already running, and bad/unsupported PRPH diagnostics can
 * set HW_ERR before the first real host command. */
static void dump_post_alive(void) {
    term_puts("  RBD[0]=0x");             fwph64(s_rx_free_rbd[0]);
    term_puts(" URBD[0]=0x");             fwph32(((uint32_t *)s_rx_used_rbd)[0]);
    term_putchar('\n');

    /* RB[0] hex dump — first 64 bytes.  iwl_rx_packet header is at the
     * very start: __le32 len_n_flags, then 4-byte cmd_header, then payload. */
    term_puts("  RB[0]:");
    if (s_rx_rb && s_rx_rb[0]) {
        for (int i = 0; i < 64; i++) {
            if ((i & 0xF) == 0) { term_puts("\n    "); }
            fwph8(s_rx_rb[0][i]);
            term_putchar(' ');
        }
        term_putchar('\n');
    } else {
        term_puts(" (no buffer)\n");
    }

}

/* One-line snapshot of firmware progress. */
static void dump_state(const char *label) {
    /* Explicitly assert MAC_ACCESS_REQ + INIT_DONE every time, then verify
     * the chip latched them.  PRPH access depends on bit 20 being held. */
    uint32_t gp_before = creg_r32(CSR_GP_CNTRL);
    creg_w32(CSR_GP_CNTRL,
             gp_before
             | CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ
             | CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
    iwl_delay(10);
    uint32_t gp_after = creg_r32(CSR_GP_CNTRL);

    term_puts("  ["); term_puts(label); term_puts("] ");
    term_puts("GP "); fwph32(gp_before);
    term_puts("→"); fwph32(gp_after);
    term_puts(" INT="); fwph32(creg_r32(CSR_INT));
    term_puts(" FHINT="); fwph32(creg_r32(CSR_FH_INT_STATUS));
    term_putchar('\n');

    /* Bit 20 = MAC_ACCESS_REQ.  If it stuck, PRPH should be reachable. */
    if (gp_after & CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ) {
        term_puts("    PRPH: ");
        term_puts("UMAC_PC="); fwph32(prph_r32(UREG_UMAC_CURRENT_PC));
        term_puts(" LMAC1_PC="); fwph32(prph_r32(UREG_LMAC1_CURRENT_PC));
        term_puts(" SCR0="); fwph32(prph_r32(IWL_PRPH_SCRATCH0));
    } else {
        term_puts("    MAC_ACCESS_REQ did NOT latch — chip refusing access");
    }
    term_puts(" stts="); fwph32(s_rx_stts[0]);
    term_putchar('\n');
}

/* (state vars moved above — kept here as a comment marker) */

/* ── Build the per-section DRAM buffers ───────────────────────────────── */
/*
 * Walk img->sections[], copy each into DMA-coherent memory, and publish
 * pairs into the context_info DRAM arrays.
 *
 * Linux's iwl_pcie_init_fw_sec classifies each section into umac / lmac /
 * virtual buckets based on a separator NID (0xFFFFCCCC) that appears in
 * the stream.  We don't track that yet — this first cut stuffs everything
 * into virtual_img[] to see whether the bootstrap accepts it that way.
 * If it doesn't (likely), the diagnostic prints below will show the
 * address range of each section and we can refine the split.
 */
#define IWL_CPU1_CPU2_SEPARATOR    0xFFFFCCCCu
#define IWL_PAGING_SEPARATOR       0xAAAABBBBu

static int build_dram_sections(const struct iwl_fw_image_info *img,
                               struct iwl_context_info_dram *dram) {
    static uint8_t  sec_ready[IWL_MAX_DRAM_ENTRY];
    static uint8_t *sec_buf[IWL_MAX_DRAM_ENTRY];
    static uint32_t sec_addr[IWL_MAX_DRAM_ENTRY];
    static uint32_t sec_len[IWL_MAX_DRAM_ENTRY];
    static uint64_t sec_phys[IWL_MAX_DRAM_ENTRY];

    if (img->n_sections == 0) {
        term_puts("  (image has 0 sections — wrong image slot?)\n");
        return 0;
    }
    if (img->n_sections > IWL_MAX_DRAM_ENTRY) {
        term_puts("  (too many sections: 0x"); fwph32(img->n_sections);
        term_putchar(')'); term_putchar('\n');
        return 0;
    }

    term_puts("  sections:\n");
    int bucket = 0;   /* 0=lmac, 1=umac, 2=virtual (paging) */
    uint32_t idx[3] = {0,0,0};
    for (int i = 0; i < img->n_sections; i++) {
        const struct iwl_fw_section *s = &img->sections[i];

        /* Handle the CPU1/CPU2 and paging separator markers. */
        if (s->addr == IWL_CPU1_CPU2_SEPARATOR) { bucket = 1; continue; }
        if (s->addr == IWL_PAGING_SEPARATOR)    { bucket = 2; continue; }

        uint64_t phys;
        uint8_t *buf;
        if (sec_ready[i] && sec_addr[i] == s->addr && sec_len[i] == s->len) {
            buf = sec_buf[i];
            phys = sec_phys[i];
        } else {
            buf = (uint8_t *)dma_alloc(s->len, 4096, &phys);
            if (!buf) { term_puts("  alloc FAIL\n"); return 0; }

            sec_ready[i] = 1;
            sec_buf[i] = buf;
            sec_addr[i] = s->addr;
            sec_len[i] = s->len;
            sec_phys[i] = phys;
        }

        const uint8_t *src = (const uint8_t *)s->data;
        for (uint32_t j = 0; j < s->len; j++) buf[j] = src[j];

        /* Log what we placed */
        term_puts("    ["); fwph32((uint32_t)i); term_puts("] bucket=");
        fwph8((uint8_t)bucket);
        term_puts(" addr=0x"); fwph32(s->addr);
        term_puts(" len=0x"); fwph32(s->len);
        term_puts(" → phys=0x"); fwph64(phys);
        term_putchar('\n');

        /* Publish pointer into the right bucket */
        if (bucket == 0) {
            if (idx[0] < IWL_MAX_DRAM_ENTRY) dram->lmac_img[idx[0]++] = phys;
        } else if (bucket == 1) {
            if (idx[1] < IWL_MAX_DRAM_ENTRY) dram->umac_img[idx[1]++] = phys;
        } else {
            if (idx[2] < IWL_MAX_DRAM_ENTRY) dram->virtual_img[idx[2]++] = phys;
        }
    }
    term_puts("  counts: lmac=0x"); fwph8((uint8_t)idx[0]);
    term_puts(" umac=0x"); fwph8((uint8_t)idx[1]);
    term_puts(" virtual=0x"); fwph8((uint8_t)idx[2]);
    term_putchar('\n');
    return 1;
}

/* ── RX queue allocation ──────────────────────────────────────────────── */
static int rx_init(void) {
    /* Linux iwl-fh.h says RFH_Q_*_RB_FRBDCB_BA_LSB requires bits 11:0 = 0
     * (i.e. page-aligned).  We use 4 KiB so firmware's full ctxt_info
     * validation path (only triggered when cmd_queue_size is in valid
     * 0..5 range) accepts our addresses. */
    if (!s_rx_free_rbd)
        s_rx_free_rbd = (uint64_t *)dma_alloc(IWL_RX_RING_SIZE * 8, 4096,
                                              &s_rx_free_rbd_phys);
    if (!s_rx_used_rbd)
        s_rx_used_rbd = (uint64_t *)dma_alloc(IWL_RX_RING_SIZE * 8, 4096,
                                              &s_rx_used_rbd_phys);
    if (!s_rx_stts)
        s_rx_stts = (uint32_t *)dma_alloc(4096, 4096, &s_rx_stts_phys);
    if (!s_rx_free_rbd || !s_rx_used_rbd || !s_rx_stts) return 0;

    fw_zero(s_rx_free_rbd, IWL_RX_RING_SIZE * 8u);
    fw_zero(s_rx_used_rbd, IWL_RX_RING_SIZE * 8u);
    fw_zero(s_rx_stts, 4096);

    if (!s_rx_rb) {
        s_rx_rb = (uint8_t **)kmalloc(IWL_RX_RING_SIZE * sizeof(uint8_t *));
        if (s_rx_rb)
            fw_zero(s_rx_rb, IWL_RX_RING_SIZE * sizeof(uint8_t *));
    }
    if (!s_rx_rb) return 0;

    for (uint32_t i = 0; i < IWL_RX_RING_SIZE; i++) {
        uint64_t rb_phys = 0;
        if (!s_rx_rb[i]) {
            s_rx_rb[i] = (uint8_t *)dma_alloc(IWL_RX_RB_SIZE, IWL_RX_RB_SIZE,
                                              &rb_phys);
        } else {
            rb_phys = (uint64_t)(uintptr_t)s_rx_rb[i] - iwl_hhdm();
        }
        if (!s_rx_rb[i]) return 0;
        fw_zero(s_rx_rb[i], IWL_RX_RB_SIZE);
        /* 22000-family MQ-RX RBDs carry a 12-bit virtual RB ID in the low
         * address bits.  The buffer is 4 KiB-aligned, so those bits are free;
         * ID 0 is invalid in Linux's global_table path. */
        s_rx_free_rbd[i] = rb_phys | ((uint64_t)i + 1u);
    }
    return 1;
}

/* ── Build context_info ───────────────────────────────────────────────── */
static int ctxt_init(uint32_t hw_rev) {
    if (!s_ctxt)
        s_ctxt = (struct iwl_context_info *)dma_alloc(sizeof(*s_ctxt), 4096,
                                                      &s_ctxt_phys);
    if (!s_ctxt) return 0;
    fw_zero(s_ctxt, sizeof(*s_ctxt));

    s_ctxt->version.mac_id  = (uint16_t)hw_rev;
    s_ctxt->version.version = 0;
    s_ctxt->version.size    = (uint16_t)(sizeof(*s_ctxt) / 4);

    /* TFD_FORMAT_LONG (bit 8) + 2048 RBs (cb_size = 11, in [7:4])
     * + 4 KiB RB size (enum 0x4 in [12:9]).  These positions matter:
     * Linux's u32_encode_bits(BIT(8), …) produces 0x100, NOT 0x001.   */
    s_ctxt->control.control_flags =
        IWL_CTXT_INFO_TFD_FORMAT_LONG
        | IWL_CTXT_INFO_RB_CB_SIZE(IWL_CTXT_INFO_RB_CB_SIZE_2048)
        | IWL_CTXT_INFO_RB_SIZE(IWL_CTXT_INFO_RB_SIZE_4K);

    s_ctxt->rbd_cfg.free_rbd_addr = s_rx_free_rbd_phys;
    s_ctxt->rbd_cfg.used_rbd_addr = s_rx_used_rbd_phys;
    s_ctxt->rbd_cfg.status_wr_ptr = s_rx_stts_phys;

    /* Linux-spec command queue: 32-entry TFD ring encoded as cb_size = 2. */
    uint64_t cmd_phys = 0;
    uint8_t  cmd_size_log = 0;
    iwl_cmd_q_publish(&cmd_phys, &cmd_size_log);
    (void)cmd_size_log;
    s_ctxt->hcmd_cfg.cmd_queue_addr = cmd_phys;
    s_ctxt->hcmd_cfg.cmd_queue_size = cmd_size_log;     /* Linux-spec: =2 */
    return 1;
}

/* Hex dump the first 0x40 bytes of ctxt_info + sizeof check.  Smaller
 * dump (4 rows) so it doesn't scroll off the screen.
 *   0x00: version (mac_id LE, version LE, size LE, reserved LE)
 *   0x08: control (control_flags LE32, reserved LE32)
 *   0x10: reserved0 LE64
 *   0x18: rbd_cfg.free_rbd_addr LE64
 *   0x20..0x2F: rbd_cfg.used_rbd_addr, status_wr_ptr
 *   0x30: hcmd_cfg.cmd_queue_addr / 0x38: cmd_queue_size + reserved[7]
 */
static void dump_ctxt_head(void) {
    term_puts("ctxt sizeof=0x"); fwph32((uint32_t)sizeof(*s_ctxt));
    term_puts(" ver.size=0x"); fwph32((uint32_t)s_ctxt->version.size);
    term_putchar('\n');
    term_puts("ctxt[0..0x40]:\n");
    const uint8_t *p = (const uint8_t *)s_ctxt;
    for (int row = 0; row < 4; row++) {
        term_puts("  "); fwph8((uint8_t)(row * 16)); term_puts(": ");
        for (int col = 0; col < 16; col++) {
            fwph8(p[row * 16 + col]);
            term_putchar(' ');
        }
        term_putchar('\n');
    }
}

/* ── Wait for ALIVE, with periodic state dumps ────────────────────────── */
/*
 * Sample the firmware's state registers at intervals so we can see (in a
 * single reboot cycle) whether/when the firmware actually starts executing
 * — UMAC_PC and LMAC1_PC become non-zero as soon as the chip starts
 * running uploaded code.  rb_stts[0] is the host-visible completion
 * indicator that ALIVE landed in the RX ring.
 */
static int wait_alive(void) {
    dump_state("t=0  ");
    int got = 0;
    for (int i = 1; i <= 20; i++) {
        iwl_delay(100000);    /* 100 ms */

        /* Firmware emits ALIVE either as bit 0 of CSR_INT (legacy) OR by
         * writing the producer index to rb_stts[0] (modern path).  Watch
         * both — whichever fires first means we won.                    */
        uint32_t interrupt = creg_r32(CSR_INT);
        uint32_t prod      = s_rx_stts[0] & 0xFFFu;
        if ((interrupt & CSR_INT_BIT_ALIVE) || prod != 0) {
            uint32_t fatal = interrupt & (CSR_INT_BIT_SW_ERR | CSR_INT_BIT_HW_ERR);
            char lbl[8] = "t=  ";
            lbl[2] = (char)('0' + (i / 10));
            lbl[3] = (char)('0' + (i % 10));
            dump_state(lbl);
            term_puts("iwl: fw ALIVE!  INT=0x");
            { static const char h[] = "0123456789abcdef";
              for (int k = 28; k >= 0; k -= 4) term_putchar(h[(interrupt >> k) & 0xF]); }
            term_puts(" stts=0x");
            { static const char h[] = "0123456789abcdef";
              for (int k = 12; k >= 0; k -= 4) term_putchar(h[(prod >> k) & 0xF]); }
            term_putchar('\n');
            /* Acknowledge the interrupt so it doesn't keep flashing. */
            creg_w32(CSR_INT, interrupt);

            /* Linux gen2 path: firmware programs the RFH itself; the host
             * just rings the free-RBD restock doorbell post-ALIVE so fw
             * has buffers to land the RX ALIVE notification in. */
            /* Linux initially restocks queue_size - 1 buffers and rings the
             * aligned RFH doorbell: round_down(IWL_RX_RING_SIZE - 1, 8). */
            creg_w32(CSR_RFH_Q_FRBDCB_WIDX_TRG, IWL_RX_INITIAL_WRITE_ACTUAL);

            /* Keep the context-info load mask until the RX ALIVE
             * notification arrives; Linux re-enables ALIVE|FH_RX here. */
            creg_w32(CSR_INT_MASK, CSR_INT_MASK_FW_LOAD);

            /* Wait up to 1 second for REPLY_ALIVE to land in RB[0]. */
            uint32_t prod_after = 0;
            for (int j = 0; j < 100; j++) {
                iwl_delay(10000);   /* 10 ms */
                prod_after = s_rx_stts[0] & 0xFFFu;
                if (prod_after != 0) break;
            }
            term_puts("  post-rfh: stts="); fwph32(s_rx_stts[0]);
            term_puts(" prod=0x"); fwph32(prod_after);
            term_putchar('\n');

            /* Peek at what firmware delivered so we can plan phase 3. */
            dump_post_alive();

            if (fatal || prod_after == 0) {
                term_puts("iwl: fw ALIVE incomplete");
                if (fatal) {
                    term_puts(" (fatal INT=0x");
                    fwph32(fatal);
                    term_putchar(')');
                }
                term_putchar('\n');
                return 0;
            }

            /* Parse the ALIVE notification — extract LMAC error_event_table
             * and UMAC error_info_addr.  These point into firmware SRAM and
             * are populated by firmware when it asserts.  Storing them now
             * lets iwl_dump_fw_error() read the assert reason later.
             *
             * Layout in RB[0] (Linux fw/api/alive.h iwl_alive_ntf_v6):
             *   off 0..7   : iwl_rx_packet hdr (len_n_flags + cmd_header)
             *   off 8..9   : status   (__le16)
             *   off 10..11 : flags    (__le16)
             *   off 12+12  = 24 : lmac_data[0].error_event_table_ptr
             *   off 12+48+12 = 72 : lmac_data[1].error_event_table_ptr
             *   off 12+96+8  =116 : umac_data.error_info_addr           */
            if (s_rx_rb && s_rx_rb[0]) {
                const uint8_t *p = s_rx_rb[0];
                uint32_t lmac0_err =
                    (uint32_t)p[8+24]      | ((uint32_t)p[8+25] << 8) |
                    ((uint32_t)p[8+26]<<16) | ((uint32_t)p[8+27] << 24);
                uint32_t umac_err =
                    (uint32_t)p[8+116]      | ((uint32_t)p[8+117] << 8) |
                    ((uint32_t)p[8+118]<<16) | ((uint32_t)p[8+119] << 24);
                s_lmac_err_ptr = lmac0_err;
                s_umac_err_ptr = umac_err;
                term_puts("  alive: lmac_err=0x"); fwph32(lmac0_err);
                term_puts(" umac_err=0x");         fwph32(umac_err);
                term_putchar('\n');
            }

            /* Start host-command tests from a clean interrupt snapshot.
             * FH_RX for REPLY_ALIVE is expected and should not be reported
             * as command-time state in iwl_send_cmd(). */
            creg_w32(CSR_INT, 0xFFFFFFFFu);
            creg_w32(CSR_FH_INT_STATUS, 0xFFFFFFFFu);
            got = 1; break;
        }
        if (i % 5 == 0) {
            char lbl[8] = "t=  ";
            lbl[2] = (char)('0' + (i / 10));
            lbl[3] = (char)('0' + (i % 10));
            dump_state(lbl);
        }
    }
    if (!got) {
        dump_state("END  ");
        term_puts("iwl: ALIVE timeout — fw did not respond.\n");
    }
    return got;
}

/* ── Public: bootstrap the firmware ───────────────────────────────────── */
int iwl_fw_load(uint32_t hw_rev, const struct iwl_fw_info *fw) {
    if (!fw || !fw->present) {
        term_puts("iwl_fw_load: no firmware parsed\n");
        return 0;
    }

    /* Linux start_fw clears stale RF-kill / command-block handshakes before
     * allocating queues and kicking context-info firmware. */
    creg_w32(CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);
    creg_w32(CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED);

    term_puts("iwl: rx_init ... ");
    if (!rx_init()) { term_puts("FAIL\n"); return 0; }
    term_puts("OK\n");

    term_puts("iwl: cmd_q_alloc ... ");
    if (!iwl_cmd_q_alloc(iwl_hhdm())) { term_puts("FAIL\n"); return 0; }
    term_puts("OK\n");

    term_puts("iwl: ctxt_init ... ");
    if (!ctxt_init(hw_rev)) { term_puts("FAIL\n"); return 0; }
    term_puts("OK\n");

    /* AX201 / 22000-series: INIT image doesn't exist, use REGULAR for
     * the bootstrap load.                                               */
    term_puts("iwl: build REGULAR dram sections:\n");
    if (!build_dram_sections(&fw->img[IWL_FW_IMG_REGULAR], &s_ctxt->dram)) {
        term_puts("iwl: build dram FAIL\n");
        return 0;
    }

    term_puts("iwl: ctxt phys=0x"); fwph64(s_ctxt_phys);
    term_putchar('\n');

    /* Print the ctxt_info header bytes for byte-by-byte verification
     * against Linux's published struct layout.  Stays visible — runs
     * after the DRAM section listing and immediately before the kick. */
    dump_ctxt_head();

    /* Clear any stale interrupts from a previous boot session.           */
    creg_w32(CSR_INT, 0xFFFFFFFFu);

    /* PRPH access needs MAC_ACCESS_REQ held.                             */
    if (!iwl_grab_nic_access_ext()) {
        term_puts("iwl: grab_nic_access (load) FAIL\n");
        return 0;
    }

    /* Wipe stale fw-load progress flag.                                  */
    prph_w32(IWL_PRPH_SCRATCH0, 0);

    /* End-of-init MAC shadow register write. */
    creg_w32(0x0A8u, CSR_MAC_SHADOW_REG_CTRL_VAL);

    /* LTR (latency tolerance) setup — required on integrated Qu silicon.  */
    prph_w32(HPM_MAC_LTR_CSR, HPM_MAC_LTR_ENABLE_ALL);
    prph_w32(HPM_UMAC_LTR,    HPM_UMAC_LTR_DEFAULT_VAL);

    /* Context-info BA latch. */
    creg_w32(CSR_CTXT_INFO_BA,     (uint32_t)s_ctxt_phys);
    creg_w32(CSR_CTXT_INFO_BA + 4, (uint32_t)(s_ctxt_phys >> 32));

    /* Pre-kick: set the SAME mask Linux's context-info loader uses:
     * ALIVE for the first bootstrap interrupt plus FH_RX for the RX ALIVE
     * notification that follows after RFH restock. */
    creg_w32(CSR_INT_MASK, CSR_INT_MASK_FW_LOAD);

    /* Kick the firmware. */
    prph_w32(UREG_CPU_INIT_RUN, 1);

    iwl_release_nic_access_ext();

    /* RFH PRPH programming is intentionally NOT done by the host on AX201.
     * Per Linux pcie/rx.c iwl_pcie_gen2_rx_init: "We don't configure the
     * RFH. Restock will be done at alive, after firmware configured the
     * RFH."  Firmware programs RFH itself using the RBD ring addresses we
     * passed via iwl_context_info.rbd_cfg.  Post-ALIVE we ONLY ring the
     * free-RBD restock doorbell. */

    term_puts("iwl: waiting for ALIVE ...\n");
    return wait_alive();
}
