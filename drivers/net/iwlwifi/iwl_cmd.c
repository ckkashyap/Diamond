/*
 * iwl_cmd.c - Host-firmware command queue + RX response dispatch.
 *
 * Mirrors Linux's iwl_pcie_gen2_enqueue_hcmd / iwl_pcie_hcmd_complete /
 * iwl_pcie_rx_handle_rb closely enough for AX201/Qu firmware to accept
 * commands and reply.  Polls — no interrupts.
 */

#include <stdint.h>
#include "../../../arch/x86/vm.h"
#include "../../../drivers/terminal.h"
#include "../../../kernel/alloc.h"
#include "iwl_csr.h"
#include "iwl_ctxt.h"
#include "iwl_cmd.h"
#include "iwl_fw.h"
#include "iwlwifi.h"

/* From iwlwifi.c (iwl_csr_base, iwl_hhdm, iwl_delay, grab/release wrappers). */
extern volatile uint8_t *iwl_csr_base(void);
extern uint64_t          iwl_hhdm(void);
extern void              iwl_delay(uint32_t usecs);
extern void              iwl_mac(uint8_t out[6]);

/* From iwl_fw_load.c — RX ring state we share. */
extern uint64_t  *iwl_rx_free_rbd(void);
extern uint64_t  *iwl_rx_used_rbd(void);
extern uint8_t  **iwl_rx_rb(void);
extern uint32_t  *iwl_rx_stts_raw(void);
extern uint32_t   iwl_lmac_err_ptr(void);
extern uint32_t   iwl_umac_err_ptr(void);

/* ── State ────────────────────────────────────────────────────────────── */
static struct iwl_tfh_tfd *s_tfd_ring;        /* 32 × 256 bytes */
static uint64_t            s_tfd_phys;
static uint8_t            *s_cmd_buf;          /* 32 × 512 bytes (cmd payload) */
static uint64_t            s_cmd_buf_phys;
static uint8_t            *s_first_tb;         /* 32 × 64 bytes (scratch for TB0) */
static uint64_t            s_first_tb_phys;
static uint8_t            *s_invalid_cmd;      /* Linux invalid_tx_cmd header */
static uint64_t            s_invalid_cmd_phys;

/* Per-slot cmd buffer must hold any host command's wide header + payload.
 * SCAN_REQ_UMAC v14 alone is 1940 bytes; round to 2048. */
#define IWL_CMD_BUF_STRIDE     2048u
#define IWL_FIRST_TB_STRIDE    64u    /* IWL_FIRST_TB_SIZE_ALIGN per Linux */

/* SW write pointer — counts mod 256 per Linux convention.  The slot in
 * the 32-entry ring is `write_ptr & 31`.                                 */
static uint8_t  s_write_ptr;
/* SW read pointer for RX drain — index into rxq->rb_pool. */
static uint16_t s_rx_read;
static uint16_t s_rx_free_write;
static uint16_t s_rx_free_write_actual;
static int      s_rx_bad_vid_warned;
static int      s_seen_init_complete;
static int      s_nvm_ready;
static int      s_rt_ready;
static int      s_scan_setup_ready;
static int      s_txq_ready;
static int      s_aux_sta_ready;
static uint16_t s_txq_id;          /* assigned by firmware in cfg response */
static uint16_t s_txq_write_ptr;   /* firmware-reported initial write_ptr */

/* TX management queue — separate from cmd queue.  Firmware uses this queue
 * to TX scan-probe-request frames built from the SCAN_REQ_UMAC preq template
 * (host never writes a TFD here).  16 entries × 256 B = 4 KiB. */
static struct iwl_tfh_tfd *s_txq_tfd_ring;
static uint64_t            s_txq_tfd_phys;
static uint8_t            *s_txq_bc_tbl;       /* 2048 bytes (1024 × __le16) */
static uint64_t            s_txq_bc_tbl_phys;

/* AP-STA TX queue — host-driven, used to TX AUTH / ASSOC_REQ / data frames
 * to the associated AP.  Allocated by iwl_cmd_assoc_probe() once we have a
 * target AP.  Distinct from the AUX queue (which is firmware-driven for
 * scan probes).  cmd_buf and first_tb scratch are needed because the host
 * builds the TFD payload here. */
#define IWL_AP_STA_ID              0u   /* Linux station-mode AP STA id */
#define IWL_AUX_STA_ID             1u   /* internal AUX scan/ROC STA id */
#define IWL_AP_TX_CMD_STRIDE       2048u
static int                 s_ap_ready;
static struct iwl_tfh_tfd *s_aptxq_tfd_ring;
static uint64_t            s_aptxq_tfd_phys;
static uint8_t            *s_aptxq_cmd_buf;
static uint64_t            s_aptxq_cmd_buf_phys;
static uint8_t            *s_aptxq_first_tb;
static uint64_t            s_aptxq_first_tb_phys;
static uint8_t            *s_aptxq_bc_tbl;
static uint64_t            s_aptxq_bc_tbl_phys;
static uint16_t            s_aptxq_id;
static uint8_t             s_aptxq_tid;
static uint16_t            s_aptxq_size;
static uint16_t            s_aptxq_rd_ptr;
static uint16_t            s_aptxq_wr_ptr;
static uint8_t             s_ap_bssid[6];
static uint8_t             s_ap_channel;
static int                 s_mld_mac_added;
static int                 s_mld_link_added;
static uint32_t            s_ap_rate_n_flags;
static uint16_t            s_aptxq_seq;
static int                 s_net_ready;
static int                 s_scan_collecting;

struct wpa_state {
    int     valid;
    uint8_t ptk[48];       /* KCK || KEK || TK for WPA2-CCMP */
    uint8_t snonce[32];
    uint8_t anonce[32];
};
static struct wpa_state s_wpa;

static int iwl_cmd_txq_teardown_probe(void);
static void iwl_aps_reset(void);
static void ap_txq_note_tx_status(const struct iwl_rx_packet *pkt, uint32_t plen);
static void print_beacon(const struct iwl_rx_packet *pkt, uint32_t plen);
static uint32_t s_nvm_sku;
static uint32_t s_nvm_tx_chains;
static uint32_t s_nvm_rx_chains;
static uint32_t s_nvm_lar;
static uint32_t s_nvm_n_channels;

/* ── Helpers ──────────────────────────────────────────────────────────── */
static void prh8(uint8_t v) {
    static const char h[] = "0123456789abcdef";
    term_putchar(h[v >> 4]); term_putchar(h[v & 0xf]);
}
static void prh16(uint16_t v) { prh8((uint8_t)(v >> 8)); prh8((uint8_t)v); }
static void prh32(uint32_t v) { prh16((uint16_t)(v >> 16)); prh16((uint16_t)v); }
static void prh64(uint64_t v) { prh32((uint32_t)(v >> 32)); prh32((uint32_t)v); }

static inline uint32_t creg_r32(uint32_t off) {
    return *(volatile uint32_t *)(iwl_csr_base() + off);
}
static inline void creg_w32(uint32_t off, uint32_t v) {
    *(volatile uint32_t *)(iwl_csr_base() + off) = v;
}

static uint16_t rx_closed_index(const struct iwl_rb_status *rs) {
    return (uint16_t)(rs->closed_rb_num & IWL_RX_RING_MASK);
}

static uint16_t rx_next_index(uint16_t idx) {
    return (uint16_t)((idx + 1u) & IWL_RX_RING_MASK);
}

static void rx_warn_bad_vid(uint16_t used_idx, uint16_t vid) {
    if (s_rx_bad_vid_warned)
        return;
    s_rx_bad_vid_warned = 1;
    term_puts("iwl: RX bad RBD vid used=");
    prh16(used_idx);
    term_puts(" vid=");
    prh16(vid);
    term_putchar('\n');
}

static struct iwl_rx_packet *rx_packet_for_read(uint16_t *vid_out) {
    volatile uint32_t *used = (volatile uint32_t *)iwl_rx_used_rbd();
    uint8_t **rb = iwl_rx_rb();
    uint16_t used_idx = (uint16_t)(s_rx_read & IWL_RX_RING_MASK);
    uint16_t vid = 0;

    if (used)
        vid = (uint16_t)(used[used_idx] & 0x0FFFu);
    if (vid_out)
        *vid_out = vid;

    if (!used || !rb || vid == 0 || vid > IWL_RX_RING_SIZE) {
        rx_warn_bad_vid(used_idx, vid);
        return (struct iwl_rx_packet *)0;
    }
    if (!rb[vid - 1u]) {
        rx_warn_bad_vid(used_idx, vid);
        return (struct iwl_rx_packet *)0;
    }
    return (struct iwl_rx_packet *)rb[vid - 1u];
}

static void rx_restock_vid(uint16_t vid) {
    uint64_t *free = iwl_rx_free_rbd();
    uint8_t **rb = iwl_rx_rb();

    if (!free || !rb || vid == 0 || vid > IWL_RX_RING_SIZE || !rb[vid - 1u]) {
        rx_warn_bad_vid((uint16_t)(s_rx_read & IWL_RX_RING_MASK), vid);
        return;
    }

    uint64_t phys = (uint64_t)(uintptr_t)rb[vid - 1u] - iwl_hhdm();
    free[s_rx_free_write & IWL_RX_RING_MASK] = phys | (uint64_t)vid;
    s_rx_free_write = rx_next_index(s_rx_free_write);

    uint16_t actual = (uint16_t)(s_rx_free_write & ~7u);
    if (actual != s_rx_free_write_actual) {
        __sync_synchronize();
        creg_w32(CSR_RFH_Q_FRBDCB_WIDX_TRG, actual);
        s_rx_free_write_actual = actual;
    }
}

static void rx_consume_read(uint16_t vid) {
    rx_restock_vid(vid);
    s_rx_read = rx_next_index(s_rx_read);
}

static inline uint32_t txq_doorbell(uint16_t qid, uint16_t wr_ptr) {
    return (((uint32_t)qid & 0x1Fu) << 16) | ((uint32_t)wr_ptr & 0xFFu);
}

/* Read `n` 32-bit words from firmware SRAM starting at `sram_addr` into
 * the destination buffer (Linux iwl_trans_pcie_read_mem semantics).  Each
 * RDAT read auto-increments RADDR by 4. */
static void iwl_sram_read(uint32_t sram_addr, uint32_t *dst, uint32_t n) {
    creg_w32(HBUS_TARG_MEM_RADDR, sram_addr);
    for (uint32_t i = 0; i < n; i++) dst[i] = creg_r32(HBUS_TARG_MEM_RDAT);
}

/* Dump the UMAC error event table.  Reads 15 × 32-bit words at the SRAM
 * address captured from ALIVE; layout per Linux fw/error-dump.h
 * iwl_umac_error_event_table.  This is the structured assertion log
 * firmware writes after SW_ERR — `error_id` is the key field. */
static void iwl_dump_fw_error(void) {
    uint32_t umac = iwl_umac_err_ptr();
    uint32_t lmac = iwl_lmac_err_ptr();
    if (!umac && !lmac) {
        term_puts("iwl: fw_error: no error pointers from ALIVE\n");
        return;
    }
    if (umac) {
        uint32_t e[16] = {0};
        iwl_sram_read(umac, e, 16);
        term_puts("iwl: UMAC error log @0x"); prh32(umac); term_putchar('\n');
        term_puts("  valid=0x");         prh32(e[0]);
        term_puts("  error_id=0x");      prh32(e[1]);
        term_putchar('\n');
        term_puts("  blink1=0x");        prh32(e[2]);
        term_puts(" blink2=0x");         prh32(e[3]);
        term_putchar('\n');
        term_puts("  ilink1=0x");        prh32(e[4]);
        term_puts(" ilink2=0x");         prh32(e[5]);
        term_putchar('\n');
        term_puts("  data1=0x");         prh32(e[6]);
        term_puts(" data2=0x");          prh32(e[7]);
        term_puts(" data3=0x");          prh32(e[8]);
        term_putchar('\n');
        term_puts("  umac_ver=");        prh32(e[9]);
        term_putchar('.');               prh32(e[10]);
        term_puts(" frame_ptr=0x");      prh32(e[11]);
        term_puts(" stack_ptr=0x");      prh32(e[12]);
        term_putchar('\n');
        term_puts("  cmd_header=0x");    prh32(e[13]);
        term_puts(" nic_isr_pref=0x");   prh32(e[14]);
        term_putchar('\n');
    }
    if (lmac) {
        uint32_t e[16] = {0};
        iwl_sram_read(lmac, e, 16);
        term_puts("iwl: LMAC error log @0x"); prh32(lmac); term_putchar('\n');
        term_puts("  valid=0x");         prh32(e[0]);
        term_puts("  error_id=0x");      prh32(e[1]);
        term_putchar('\n');
        term_puts("  trm_hw=0x");        prh32(e[2]);
        term_putchar('.');               prh32(e[3]);
        term_puts(" blink2=0x");         prh32(e[4]);
        term_putchar('\n');
        term_puts("  ilink1=0x");        prh32(e[5]);
        term_puts(" ilink2=0x");         prh32(e[6]);
        term_putchar('\n');
        term_puts("  data1=0x");         prh32(e[7]);
        term_puts(" data2=0x");          prh32(e[8]);
        term_puts(" data3=0x");          prh32(e[9]);
        term_putchar('\n');
    }
}

static void note_rx_packet(const struct iwl_rx_packet *pkt, uint32_t plen) {
    if (pkt->hdr.group_id == IWL_GROUP_LEGACY &&
        pkt->hdr.cmd == IWL_CMD_INIT_COMPLETE_NOTIF)
        s_seen_init_complete = 1;
    ap_txq_note_tx_status(pkt, plen);
    if (s_scan_collecting && pkt->hdr.cmd == IWL_CMD_REPLY_RX_MPDU)
        print_beacon(pkt, plen);
}

/* Aligned DMA-coherent allocation (heap-backed; phys = virt - hhdm). */
static void *dma_alloc_aligned(uint64_t size, uint64_t align, uint64_t *phys_out) {
    uint8_t *raw = (uint8_t *)kmalloc(size + align);
    if (!raw) return (void *)0;
    uint64_t addr = (uint64_t)(uintptr_t)raw;
    uint64_t aligned = (addr + align - 1) & ~(align - 1);
    for (uint64_t i = 0; i < size; i++) ((uint8_t *)aligned)[i] = 0;
    if (phys_out) *phys_out = aligned - iwl_hhdm();
    return (void *)(uintptr_t)aligned;
}

/* Write a 64-bit phys into a TB's unaligned addr field (memcpy-safe). */
static void tb_set_addr(struct iwl_tfh_tb *tb, uint64_t phys) {
    for (int i = 0; i < 8; i++) tb->addr[i] = (uint8_t)(phys >> (i * 8));
}

static void tfd_set_invalid(struct iwl_tfh_tfd *tfd) {
    for (uint64_t i = 0; i < sizeof(*tfd); i++) ((uint8_t *)tfd)[i] = 0;
    tfd->tbs[0].tb_len = (uint16_t)sizeof(struct iwl_cmd_header_wide);
    tb_set_addr(&tfd->tbs[0], s_invalid_cmd_phys);
    tfd->num_tbs = 1;
}

static void iwl_zero(void *p, uint64_t n) {
    uint8_t *b = (uint8_t *)p;
    for (uint64_t i = 0; i < n; i++) b[i] = 0;
}

void iwl_cmd_reset_state(void) {
    s_write_ptr = 0;
    s_rx_read = 0;
    s_rx_free_write = IWL_RX_INITIAL_WRITE_PTR;
    s_rx_free_write_actual = IWL_RX_INITIAL_WRITE_ACTUAL;
    s_rx_bad_vid_warned = 0;
    s_seen_init_complete = 0;
    s_nvm_ready = 0;
    s_rt_ready = 0;
    s_scan_setup_ready = 0;
    s_txq_ready = 0;
    s_aux_sta_ready = 0;
    s_txq_id = 0;
    s_txq_write_ptr = 0;
    s_ap_ready = 0;
    s_aptxq_id = 0;
    s_aptxq_tid = 0;
    s_aptxq_size = 0;
    s_aptxq_rd_ptr = 0;
    s_aptxq_wr_ptr = 0;
    s_ap_channel = 0;
    s_mld_mac_added = 0;
    s_mld_link_added = 0;
    s_ap_rate_n_flags = 0;
    s_aptxq_seq = 0;
    s_net_ready = 0;
    s_scan_collecting = 0;
    s_nvm_sku = 0;
    s_nvm_tx_chains = 0;
    s_nvm_rx_chains = 0;
    s_nvm_lar = 0;
    s_nvm_n_channels = 0;
    iwl_zero(s_ap_bssid, sizeof(s_ap_bssid));
    iwl_zero(&s_wpa, sizeof(s_wpa));
    iwl_aps_reset();

    if (s_tfd_ring) {
        for (uint32_t i = 0; i < IWL_CMD_QUEUE_SIZE; i++)
            tfd_set_invalid(&s_tfd_ring[i]);
    }
    if (s_cmd_buf)
        iwl_zero(s_cmd_buf, IWL_CMD_BUF_STRIDE * IWL_CMD_QUEUE_SIZE);
    if (s_first_tb)
        iwl_zero(s_first_tb, IWL_FIRST_TB_STRIDE * IWL_CMD_QUEUE_SIZE);
    if (s_txq_tfd_ring)
        iwl_zero(s_txq_tfd_ring,
                 sizeof(struct iwl_tfh_tfd) * IWL_MGMT_QUEUE_SIZE);
    if (s_txq_bc_tbl)
        iwl_zero(s_txq_bc_tbl, IWL_BC_TBL_SIZE * 2u);
    if (s_aptxq_tfd_ring) {
        for (uint32_t i = 0; i < IWL_DATA_QUEUE_SIZE; i++)
            tfd_set_invalid(&s_aptxq_tfd_ring[i]);
    }
    if (s_aptxq_cmd_buf)
        iwl_zero(s_aptxq_cmd_buf, IWL_AP_TX_CMD_STRIDE * IWL_DATA_QUEUE_SIZE);
    if (s_aptxq_first_tb)
        iwl_zero(s_aptxq_first_tb,
                 IWL_FIRST_TB_STRIDE * IWL_DATA_QUEUE_SIZE);
    if (s_aptxq_bc_tbl)
        iwl_zero(s_aptxq_bc_tbl, IWL_BC_TBL_SIZE * 2u);
}

/* ── Public: allocate the TFD ring + command buffers ──────────────────── */
int iwl_cmd_q_alloc(uint64_t hhdm) {
    (void)hhdm;
    /* Align the TFD ring base to its total size (32 × 256 = 8 KiB).
     * Firmware on Qu silicon appears to atomically reject ctxt_info when
     * cmd_queue_size encodes a "valid" cb_size (=2 for 32 entries) and the
     * cmd_queue_addr isn't aligned to the full ring extent — observed as
     * the RX ring never delivering REPLY_ALIVE despite the legacy ALIVE
     * interrupt firing. */
    if (!s_tfd_ring) {
        s_tfd_ring = (struct iwl_tfh_tfd *)
            dma_alloc_aligned(sizeof(struct iwl_tfh_tfd) * IWL_CMD_QUEUE_SIZE,
                              8192, &s_tfd_phys);
        if (!s_tfd_ring) return 0;
    }

    if (!s_cmd_buf) {
        s_cmd_buf = (uint8_t *)
            dma_alloc_aligned(IWL_CMD_BUF_STRIDE * IWL_CMD_QUEUE_SIZE,
                              4096, &s_cmd_buf_phys);
        if (!s_cmd_buf) return 0;
    }

    /* Per-slot 64-byte scratch.  Linux copies up to IWL_FIRST_TB_SIZE
     * bytes into `first_tb_bufs`; the TB length is the actual copy size. */
    if (!s_first_tb) {
        s_first_tb = (uint8_t *)
            dma_alloc_aligned(IWL_FIRST_TB_STRIDE * IWL_CMD_QUEUE_SIZE,
                              64, &s_first_tb_phys);
        if (!s_first_tb) return 0;
    }

    if (!s_invalid_cmd) {
        s_invalid_cmd = (uint8_t *)
            dma_alloc_aligned(sizeof(struct iwl_cmd_header_wide),
                              64, &s_invalid_cmd_phys);
        if (!s_invalid_cmd) return 0;
    }

    struct iwl_cmd_header_wide *bad = (struct iwl_cmd_header_wide *)s_invalid_cmd;
    bad->cmd      = IWL_CMD_INVALID_WR_PTR;
    bad->group_id = IWL_GROUP_DEBUG;
    bad->sequence = 0xFFFFu;
    bad->length   = 0;
    bad->reserved = 0;
    bad->version  = 0;

    iwl_cmd_reset_state();
    return 1;
}

void iwl_cmd_q_publish(uint64_t *out_phys, uint8_t *out_size_log) {
    *out_phys     = s_tfd_phys;
    *out_size_log = (uint8_t)IWL_CMD_QUEUE_SIZE_LOG;
}

/* ── Submit one host command + poll for the matching response ─────────── */
/*
 * Linux gen2 cmd format (iwl_pcie_gen2_enqueue_hcmd):
 *   - TB[0]: min(copy_size, IWL_FIRST_TB_SIZE) bytes from a per-slot
 *            64-byte scratch buffer (s_first_tb).  Empty ECHO is 8 bytes.
 *   - TB[1]: if cmd_size > 20, points to the remainder of the cmd at
 *            offset 20 in the per-slot main buffer.                    */
static int iwl_send_cmd_mode(uint8_t group_id, uint8_t cmd_id,
                             const void *payload, uint16_t payload_len,
                             void *resp_buf,    uint16_t resp_max,
                             int wait_resp) {
    uint8_t slot = s_write_ptr & 31u;

    /* Compose full command in the per-slot main buffer.
     *
     * Linux iwl-trans-pcie.c uses an 8-byte wide header for cmds in any
     * group != LEGACY (0), and a 4-byte short header for group 0.  Mixing
     * them up is undetectable until the firmware tries to dereference the
     * payload at the "wrong" offset and asserts SW_ERR — PHY_CONFIGURATION
     * (group 0) is one such cmd that needs the short form. */
    uint8_t *cb = s_cmd_buf + slot * IWL_CMD_BUF_STRIDE;
    uint16_t hdr_size;
    if (group_id == IWL_GROUP_LEGACY) {
        struct iwl_cmd_header_short *hs = (struct iwl_cmd_header_short *)cb;
        hs->cmd      = cmd_id;
        hs->group_id = 0;
        hs->sequence = (uint16_t)((0 << 8) | s_write_ptr);
        hdr_size = 4;
    } else {
        struct iwl_cmd_header_wide *hdr = (struct iwl_cmd_header_wide *)cb;
        hdr->cmd      = cmd_id;
        hdr->group_id = group_id;
        hdr->sequence = (uint16_t)((0 << 8) | s_write_ptr);
        hdr->length   = payload_len;
        hdr->reserved = 0;
        hdr->version  = 0;
        hdr_size = 8;
    }

    if (payload && payload_len) {
        const uint8_t *src = (const uint8_t *)payload;
        for (uint16_t i = 0; i < payload_len; i++) cb[hdr_size + i] = src[i];
    }
    uint16_t cmd_size = (uint16_t)(hdr_size + payload_len);

    /* Stage the first 20 bytes into the per-slot scratch (zero-pad if
     * the command is shorter than IWL_FIRST_TB_SIZE). */
    uint8_t *ftb = s_first_tb + (uint64_t)slot * IWL_FIRST_TB_STRIDE;
    for (uint8_t i = 0; i < IWL_FIRST_TB_STRIDE; i++) ftb[i] = 0;
    uint16_t copy0 = cmd_size < IWL_FIRST_TB_SIZE ? cmd_size : (uint16_t)IWL_FIRST_TB_SIZE;
    for (uint16_t i = 0; i < copy0; i++) ftb[i] = cb[i];

    /* Build TFD: TB[0] = first_tb scratch; TB[1] = remainder if any. */
    struct iwl_tfh_tfd *tfd = &s_tfd_ring[slot];
    for (uint64_t i = 0; i < sizeof(*tfd); i++) ((uint8_t *)tfd)[i] = 0;

    tfd->tbs[0].tb_len = copy0;
    tb_set_addr(&tfd->tbs[0],
                s_first_tb_phys + (uint64_t)slot * IWL_FIRST_TB_STRIDE);

    if (cmd_size > IWL_FIRST_TB_SIZE) {
        tfd->tbs[1].tb_len = (uint16_t)(cmd_size - IWL_FIRST_TB_SIZE);
        tb_set_addr(&tfd->tbs[1],
                    s_cmd_buf_phys + (uint64_t)slot * IWL_CMD_BUF_STRIDE
                    + IWL_FIRST_TB_SIZE);
        tfd->num_tbs = 2;
    } else {
        tfd->num_tbs = 1;
    }

    /* Capture the seq we just sent so we can match the response.  The seq
     * sits at bytes 2-3 of either header form, so read it directly from cb. */
    uint16_t expect_seq = (uint16_t)cb[2] | ((uint16_t)cb[3] << 8);
    uint32_t doorbell = txq_doorbell(0, (uint8_t)(s_write_ptr + 1u));

    term_puts("  tfd: n="); prh16(tfd->num_tbs);
    term_puts(" tb0="); prh16(tfd->tbs[0].tb_len);
    term_puts(" a0=0x"); prh64(s_first_tb_phys + (uint64_t)slot * IWL_FIRST_TB_STRIDE);
    term_puts(" wr=0x"); prh32(doorbell);
    term_putchar('\n');

    /* Capture the producer index BEFORE the doorbell so we can detect
     * even very fast responses (term_puts is slow — many MMIO writes —
     * so reading after would race the firmware). */
    uint32_t *stts32 = iwl_rx_stts_raw();
    if (!stts32) return -1;
    struct iwl_rb_status *rs = (struct iwl_rb_status *)stts32;
    uint16_t prev_prod = rx_closed_index(rs);
    uint32_t pre_int = creg_r32(CSR_INT);
    uint32_t pre_fhint = creg_r32(CSR_FH_INT_STATUS);
    uint32_t pre_gp1 = creg_r32(CSR_UCODE_DRV_GP1);

    term_puts("  pre: INT="); prh32(pre_int);
    term_puts(" FHINT="); prh32(pre_fhint);
    term_puts(" GP1="); prh32(pre_gp1);
    term_putchar('\n');

    if (pre_int) creg_w32(CSR_INT, pre_int);
    if (pre_fhint) creg_w32(CSR_FH_INT_STATUS, pre_fhint);

    /* Increment SW write_ptr (mod 256) and ring the doorbell. */
    s_write_ptr++;
    __sync_synchronize();
    creg_w32(HBUS_TARG_WRPTR, doorbell);

    term_puts("  cmd: g="); prh8(group_id); term_puts(" c="); prh8(cmd_id);
    term_puts(" seq="); prh16(expect_seq);
    term_puts(" len="); prh16(payload_len);
    term_puts(" prod="); prh16(prev_prod);
    term_puts(" rxread="); prh16(s_rx_read);
    term_putchar('\n');

    if (!wait_resp) {
        iwl_delay(10000);
        uint16_t cur_prod = rx_closed_index(rs);
        while (s_rx_read != cur_prod) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;
                term_puts("    RX@"); prh16(s_rx_read);
                term_puts(": cmd="); prh8(pkt->hdr.cmd);
                term_puts(" g=");    prh8(pkt->hdr.group_id);
                term_puts(" seq=");  prh16(pkt->hdr.sequence);
                term_puts(" len=");  prh32(plen); term_putchar('\n');
                note_rx_packet(pkt, plen);
            }
            rx_consume_read(vid);
        }

        uint32_t post_int = creg_r32(CSR_INT);
        uint32_t post_fhint = creg_r32(CSR_FH_INT_STATUS);
        term_puts("  async-post: INT="); prh32(post_int);
        term_puts(" FHINT="); prh32(post_fhint);
        term_putchar('\n');
        if (post_int) creg_w32(CSR_INT, post_int);
        if (post_fhint) creg_w32(CSR_FH_INT_STATUS, post_fhint);
        if (post_int & (CSR_INT_BIT_SW_ERR | CSR_INT_BIT_HW_ERR))
            return -1;
        return 0;
    }

    int found_resp_len = -1;
    uint16_t last_seen_prod = prev_prod;
    for (int t = 0; t < 50 && found_resp_len < 0; t++) {
        iwl_delay(10000);    /* 10 ms */
        uint16_t cur_prod = rx_closed_index(rs);
        last_seen_prod = cur_prod;
        if (cur_prod == prev_prod) continue;

        /* New RB(s) arrived — walk from s_rx_read up to cur_prod. */
        while (s_rx_read != cur_prod) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;

                term_puts("    RX@"); prh16(s_rx_read);
                term_puts(": cmd="); prh8(pkt->hdr.cmd);
                term_puts(" g=");    prh8(pkt->hdr.group_id);
                term_puts(" seq=");  prh16(pkt->hdr.sequence);
                term_puts(" len=");  prh32(plen); term_putchar('\n');
                note_rx_packet(pkt, plen);

                if (pkt->hdr.cmd == cmd_id &&
                    pkt->hdr.group_id == group_id &&
                    pkt->hdr.sequence == expect_seq)
                {
                    /* Matched the response we were waiting for. */
                    uint16_t payload_got = (uint16_t)(plen > 4 ? plen - 4 : 0);
                    uint16_t copy_len = payload_got;
                    if (copy_len > resp_max) copy_len = resp_max;
                    if (resp_buf && copy_len)
                        for (uint16_t i = 0; i < copy_len; i++)
                            ((uint8_t *)resp_buf)[i] = pkt->data[i];
                    found_resp_len = payload_got;
                }
            }
            rx_consume_read(vid);
        }
        prev_prod = cur_prod;
    }
    if (found_resp_len < 0) {
        term_puts("    (no response in 500 ms; prod ");
        prh16(prev_prod); term_puts("->"); prh16(last_seen_prod);
        term_puts(")\n");
        term_puts("    csr: INT="); prh32(creg_r32(CSR_INT));
        term_puts(" FHINT="); prh32(creg_r32(CSR_FH_INT_STATUS));
        term_puts(" WRPTR="); prh32(creg_r32(HBUS_TARG_WRPTR));
        term_putchar('\n');
    }
    return found_resp_len;
}

int iwl_send_cmd(uint8_t group_id, uint8_t cmd_id,
                 const void *payload, uint16_t payload_len,
                 void *resp_buf,    uint16_t resp_max) {
    return iwl_send_cmd_mode(group_id, cmd_id, payload, payload_len,
                             resp_buf, resp_max, 1);
}

static int iwl_send_cmd_async(uint8_t group_id, uint8_t cmd_id,
                              const void *payload, uint16_t payload_len) {
    return iwl_send_cmd_mode(group_id, cmd_id, payload, payload_len,
                             (void *)0, 0, 0);
}

static int iwl_wait_rx_cmd(uint8_t group_id, uint8_t cmd_id, uint32_t timeout_ms) {
    if (group_id == IWL_GROUP_LEGACY &&
        cmd_id == IWL_CMD_INIT_COMPLETE_NOTIF &&
        s_seen_init_complete)
        return 1;

    uint32_t *stts32 = iwl_rx_stts_raw();
    if (!stts32) return 0;
    struct iwl_rb_status *rs = (struct iwl_rb_status *)stts32;

    uint32_t loops = timeout_ms / 10u;
    if (!loops) loops = 1;
    for (uint32_t t = 0; t < loops; t++) {
        uint16_t cur_prod = rx_closed_index(rs);
        while (s_rx_read != cur_prod) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;
                term_puts("    RX@"); prh16(s_rx_read);
                term_puts(": cmd="); prh8(pkt->hdr.cmd);
                term_puts(" g=");    prh8(pkt->hdr.group_id);
                term_puts(" seq=");  prh16(pkt->hdr.sequence);
                term_puts(" len=");  prh32(plen); term_putchar('\n');
                note_rx_packet(pkt, plen);
                if (pkt->hdr.cmd == cmd_id && pkt->hdr.group_id == group_id) {
                    rx_consume_read(vid);
                    return 1;
                }
            }
            rx_consume_read(vid);
        }
        if (group_id == IWL_GROUP_LEGACY &&
            cmd_id == IWL_CMD_INIT_COMPLETE_NOTIF &&
            s_seen_init_complete)
            return 1;
        iwl_delay(10000);
    }
    return 0;
}

/* ── Host-command smoke test ──────────────────────────────────────────── */
int iwl_cmd_echo_test(void) {
    /* Linux's unified AX201 init flow starts with INIT_EXTENDED_CFG and
     * init_flags = BIT(IWL_INIT_NVM).  ECHO_CMD asserts in this state. */
    uint32_t init_flags = (1u << 1);   /* BIT(IWL_INIT_NVM) */

    term_puts("iwl: HCMD smoke test (INIT_EXTENDED_CFG) ...\n");
    int r = iwl_send_cmd(IWL_GROUP_SYSTEM, IWL_CMD_INIT_EXTENDED_CFG,
                         &init_flags, sizeof(init_flags),
                         (void *)0, 0);
    if (r >= 0) { term_puts("iwl: HCMD OK\n"); return 1; }
    term_puts("iwl: HCMD FAILED\n");
    return 0;
}

static uint16_t rd16(const uint8_t *p) {
    return (uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8);
}

static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t ap_txq_size_for_tid(uint8_t tid) {
    return (tid == IWL_MGMT_QUEUE_TID) ? IWL_MGMT_QUEUE_SIZE
                                       : IWL_DATA_QUEUE_SIZE;
}

static uint8_t ap_txq_cb_size_for_slots(uint16_t slots) {
    return (slots == IWL_DATA_QUEUE_SIZE) ? IWL_DATA_QUEUE_CB_SIZE
                                          : IWL_MGMT_QUEUE_CB_SIZE;
}

static uint8_t ap_txq_index(uint16_t ptr) {
    uint16_t size = s_aptxq_size ? s_aptxq_size : IWL_MGMT_QUEUE_SIZE;
    return (uint8_t)(ptr & (uint16_t)(size - 1u));
}

static uint8_t ap_txq_used(void) {
    return (uint8_t)((s_aptxq_wr_ptr - s_aptxq_rd_ptr) & 0xFFu);
}

static void ap_txq_invalidate_slot(uint8_t slot) {
    if (s_aptxq_tfd_ring)
        tfd_set_invalid(&s_aptxq_tfd_ring[slot]);
    if (s_aptxq_bc_tbl)
        ((uint16_t *)s_aptxq_bc_tbl)[slot] = 0;
}

static void ap_txq_reclaim_to(uint16_t ssn) {
    if (!s_ap_ready || !s_aptxq_tfd_ring)
        return;

    uint16_t new_rd = (uint16_t)(ssn & 0xFFu);
    uint8_t advance = (uint8_t)((new_rd - s_aptxq_rd_ptr) & 0xFFu);
    if (!advance)
        return;

    uint8_t used = ap_txq_used();
    if (advance > used) {
        term_puts("iwl: ap-txq reclaim out of range rd=");
        prh16(s_aptxq_rd_ptr);
        term_puts(" wr=");
        prh16(s_aptxq_wr_ptr);
        term_puts(" ssn=");
        prh16(ssn);
        term_putchar('\n');
        return;
    }

    for (uint8_t i = 0; i < advance; i++)
        ap_txq_invalidate_slot(ap_txq_index((uint16_t)(s_aptxq_rd_ptr + i)));
    s_aptxq_rd_ptr = new_rd;
}

static void ap_txq_note_tx_status(const struct iwl_rx_packet *pkt, uint32_t plen) {
    if (!s_ap_ready ||
        pkt->hdr.group_id != IWL_GROUP_LEGACY ||
        pkt->hdr.cmd != IWL_CMD_TX_CMD)
        return;

    const uint8_t *p = pkt->data;
    uint32_t payload_len = (plen >= 4u) ? (plen - 4u) : plen;
    if (payload_len < 48u)
        return;

    uint16_t tx_queue = rd16(p + 36);
    if (tx_queue != s_aptxq_id) {
        uint16_t seq_queue = (uint16_t)((pkt->hdr.sequence >> 8) & 0x1Fu);
        if (seq_queue != s_aptxq_id)
            return;
    }

    uint16_t status = rd16(p + 40);
    uint16_t seq = rd16(p + 42);
    uint16_t ssn = (uint16_t)(rd32(p + 44) & 0x0FFFu);
    uint8_t used_before = ap_txq_used();
    ap_txq_reclaim_to(ssn);

    term_puts("       txdone: qid=");
    prh16(s_aptxq_id);
    term_puts(" tid=");
    prh8(s_aptxq_tid);
    term_puts(" status=0x");
    prh16(status);
    term_puts(" seq=0x");
    prh16(seq);
    term_puts(" ssn=");
    prh16(ssn);
    term_puts(" fail=");
    prh8(p[3]);
    term_puts(" used=");
    prh8(used_before);
    term_puts("->");
    prh8(ap_txq_used());
    term_putchar('\n');
}

/* ── NVM probe: next Linux init milestone after INIT_EXTENDED_CFG ──────── */
int iwl_cmd_nvm_probe(void) {
    /* AX201 uses unified ucode.  Linux marks that NVM access commands may be
     * sent, closes that phase with NVM_ACCESS_COMPLETE, and only sends
     * PHY_CONFIGURATION for special SISO-diversity configs.  Surface Laptop 3
     * asserts SW_ERR if we send PHY_CONFIGURATION here. */
    uint32_t init_flags = (1u << 1);   /* BIT(IWL_INIT_NVM) */

    if (s_nvm_ready) {
        term_puts("iwl: NVM already ready: sku=0x"); prh32(s_nvm_sku);
        term_puts(" tx=0x"); prh32(s_nvm_tx_chains);
        term_puts(" rx=0x"); prh32(s_nvm_rx_chains);
        term_puts(" lar=0x"); prh32(s_nvm_lar);
        term_puts(" nchan=0x"); prh32(s_nvm_n_channels);
        term_putchar('\n');
        return 1;
    }

    s_seen_init_complete = 0;

    term_puts("iwl: NVM probe (unified init + NVM_GET_INFO) ...\n");
    int r = iwl_send_cmd(IWL_GROUP_SYSTEM, IWL_CMD_INIT_EXTENDED_CFG,
                         &init_flags, sizeof(init_flags),
                         (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: INIT_EXTENDED_CFG failed\n");
        return 0;
    }

    uint32_t nvm_complete = 0;
    term_puts("iwl: NVM_ACCESS_COMPLETE ...\n");
    r = iwl_send_cmd(IWL_GROUP_REGULATORY_AND_NVM, IWL_CMD_NVM_ACCESS_COMPLETE,
                     &nvm_complete, sizeof(nvm_complete), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: NVM_ACCESS_COMPLETE failed\n");
        return 0;
    }

    term_puts("iwl: skip PHY_CONFIGURATION (unified ucode)\n");

    term_puts("iwl: wait INIT_COMPLETE ...\n");
    if (!iwl_wait_rx_cmd(IWL_GROUP_LEGACY, IWL_CMD_INIT_COMPLETE_NOTIF, 2000)) {
        term_puts("iwl: INIT_COMPLETE timeout\n");
        return 0;
    }

    uint32_t get_info = 0;
    uint8_t resp[512];
    for (uint16_t i = 0; i < sizeof(resp); i++) resp[i] = 0;

    term_puts("iwl: NVM_GET_INFO ...\n");
    r = iwl_send_cmd(IWL_GROUP_REGULATORY_AND_NVM, IWL_CMD_NVM_GET_INFO,
                     &get_info, sizeof(get_info), resp, sizeof(resp));
    if (r < 24) {
        term_puts("iwl: NVM_GET_INFO failed/short len="); prh16((uint16_t)(r < 0 ? 0 : r));
        term_putchar('\n');
        return 0;
    }

    term_puts("  nvm_get: len="); prh16((uint16_t)r);
    term_puts(" flags=0x"); prh32(rd32(resp + 0));
    term_puts(" ver=0x"); prh16(rd16(resp + 4));
    term_puts(" board=0x"); prh8(resp[6]);
    term_puts(" addrs="); prh8(resp[7]);
    term_putchar('\n');
    term_puts("  sku=0x"); prh32(rd32(resp + 8));
    term_puts(" tx=0x"); prh32(rd32(resp + 12));
    term_puts(" rx=0x"); prh32(rd32(resp + 16));
    s_nvm_sku = rd32(resp + 8);
    s_nvm_tx_chains = rd32(resp + 12);
    s_nvm_rx_chains = rd32(resp + 16);
    if (r >= 28) {
        term_puts(" lar=0x"); prh32(rd32(resp + 20));
        term_puts(" nchan=0x"); prh32(rd32(resp + 24));
        s_nvm_lar = rd32(resp + 20);
        s_nvm_n_channels = rd32(resp + 24);
    }
    term_putchar('\n');

    s_nvm_ready = 1;
    term_puts("iwl: NVM probe OK\n");
    return 1;
}

/* ── Runtime probe: first low-risk iwl_mvm_up commands after NVM ───────── */
int iwl_cmd_runtime_probe(void) {
    if (!s_nvm_ready) {
        term_puts("iwl: runtime probe needs NVM; running wifinvm first\n");
        if (!iwl_cmd_nvm_probe())
            return 0;
    }

    uint8_t smem[512];
    for (uint16_t i = 0; i < sizeof(smem); i++) smem[i] = 0;

    term_puts("iwl: RT probe: SHARED_MEM_CFG ...\n");
    int r = iwl_send_cmd(IWL_GROUP_SYSTEM, IWL_CMD_SHARED_MEM_CFG,
                         (const void *)0, 0, smem, sizeof(smem));
    if (r < 0) {
        term_puts("iwl: SHARED_MEM_CFG failed\n");
        return 0;
    }
    term_puts("  smem: len="); prh16((uint16_t)r);
    if (r >= 16) {
        term_puts(" dw0=0x"); prh32(rd32(smem + 0));
        term_puts(" dw1=0x"); prh32(rd32(smem + 4));
        term_puts(" dw2=0x"); prh32(rd32(smem + 8));
        term_puts(" dw3=0x"); prh32(rd32(smem + 12));
    }
    term_putchar('\n');

    uint32_t sf_cmd[23];
    for (uint16_t i = 0; i < 23; i++) sf_cmd[i] = 0;
    sf_cmd[0] = 3;       /* SF_INIT_OFF: no active MAC contexts yet */
    sf_cmd[1] = 4096;    /* SF_W_MARK_SCAN */
    sf_cmd[2] = 8192;    /* SF_W_MARK_MIMO2 */
    for (uint16_t i = 0; i < 10; i++)
        sf_cmd[3 + i] = 1000000;  /* SF_LONG_DELAY_AGING_TIMER */
    for (uint16_t i = 0; i < 5; i++) {
        sf_cmd[13 + i * 2] = 400;
        sf_cmd[14 + i * 2] = 160;
    }

    term_puts("iwl: RT probe: REPLY_SF_CFG_CMD state=INIT_OFF ...\n");
    r = iwl_send_cmd_async(IWL_GROUP_LONG, IWL_CMD_REPLY_SF_CFG,
                           sf_cmd, sizeof(sf_cmd));
    if (r < 0) {
        term_puts("iwl: REPLY_SF_CFG_CMD failed\n");
        return 0;
    }

    uint32_t tx_ant = s_nvm_tx_chains & 0xFu;
    if (!tx_ant) {
        term_puts("iwl: no valid TX antenna mask from NVM\n");
        return 0;
    }

    term_puts("iwl: RT probe: TX_ANT_CONFIGURATION valid=0x");
    prh32(tx_ant); term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_LONG, IWL_CMD_TX_ANT_CONFIGURATION,
                     &tx_ant, sizeof(tx_ant), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: TX_ANT_CONFIGURATION failed\n");
        return 0;
    }

    uint32_t bt_cmd[2];
    bt_cmd[0] = 1;       /* BT_COEX_NW */
    bt_cmd[1] = 0x15;    /* SYNC2SCO | MPLUT | HIGH_BAND_RET */
    term_puts("iwl: RT probe: BT_CONFIG mode=0x");
    prh32(bt_cmd[0]); term_puts(" modules=0x"); prh32(bt_cmd[1]);
    term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_LONG, IWL_CMD_BT_CONFIG,
                     bt_cmd, sizeof(bt_cmd), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: BT_CONFIG failed\n");
        return 0;
    }

    s_rt_ready = 1;
    term_puts("iwl: RT probe OK\n");
    return 1;
}

/* ── Toward-scan setup: DQA_ENABLE + PHY_CONTEXT_CMD ADD ───────────────── */
/*
 * Linux iwl_mvm_up sequence after the RT-probe checkpoint:
 *   1. DQA_ENABLE_CMD       (DATA_PATH_GROUP cmd 0x00)
 *      payload: __le32 cmd_queue = IWL_MVM_DQA_CMD_QUEUE = 0
 *   2. PHY_CONTEXT_CMD ADD  (LONG_GROUP cmd 0x08, version 3)
 *      payload: 32 bytes — id/color, action, channel info, lmac_id, rxchain
 *
 * Both are sync.  If either asserts firmware (CSR_INT bit 25 / SW_ERR), we
 * stop and report so the operator can iterate without losing state.       */

struct iwl_phy_context_cmd_v3 {
    uint32_t id_and_color;
    uint32_t action;
    /* iwl_fw_channel_info v2: __le32 channel + 4 u8 (band, width, ctrl_pos, rsv) */
    uint32_t channel;
    uint8_t  band;
    uint8_t  width;
    uint8_t  ctrl_pos;
    uint8_t  reserved_ci;
    uint32_t lmac_id;
    uint32_t rxchain_info;
    uint32_t dsp_cfg_flags;
    uint32_t reserved;
} __attribute__((packed));    /* 32 bytes */

/* iwl_ac_qos — 8 bytes per Access Category. */
struct iwl_ac_qos {
    uint16_t cw_min;
    uint16_t cw_max;
    uint8_t  aifsn;
    uint8_t  fifos_mask;
    uint16_t edca_txop;
} __attribute__((packed));    /* 8 bytes */

/* iwl_mac_ctx_cmd — 148 bytes (Linux fw/api/mac.h:314-344).
 * Listener mode leaves the union 48 bytes of zeros — that's plenty for any
 * substruct (ap, sta, ibss, p2p_*).  We declare a fixed 48-byte buffer. */
struct iwl_mac_ctx_cmd {
    uint32_t id_and_color;
    uint32_t action;
    uint32_t mac_type;
    uint32_t tsf_id;
    uint8_t  node_addr[6];
    uint16_t reserved_for_node_addr;
    uint8_t  bssid_addr[6];
    uint16_t reserved_for_bssid_addr;
    uint32_t cck_rates;
    uint32_t ofdm_rates;
    uint32_t protection_flags;
    uint32_t cck_short_preamble;
    uint32_t short_slot;
    uint32_t filter_flags;
    uint32_t qos_flags;
    struct iwl_ac_qos ac[5];          /* AC_NUM(4) + 1 */
    uint8_t  union_data[48];          /* ap/sta/ibss/p2p_* — zero for listener */
} __attribute__((packed));    /* 60 + 40 + 48 = 148 bytes */

struct iwl_mac_data_sta {
    uint32_t is_assoc;
    uint32_t dtim_time;
    uint64_t dtim_tsf;
    uint32_t bi;
    uint32_t reserved1;
    uint32_t dtim_interval;
    uint32_t data_policy;
    uint32_t listen_interval;
    uint32_t assoc_id;
    uint32_t assoc_beacon_arrive_time;
} __attribute__((packed));    /* 40 bytes */

/* Linux fw/api/binding.h v2.  id_and_color and phy both carry the PHY context
 * id+color; macs[] carries the bound MAC context ids. */
struct iwl_binding_cmd {
    uint32_t id_and_color;
    uint32_t action;
    uint32_t macs[3];
    uint32_t phy;
    uint32_t lmac_id;
} __attribute__((packed));    /* 28 bytes */

/* MLD/new-station API commands used by Linux 6.17 on AX201 when associating.
 * The live iwlwifi "Sending command ... bytes" trace includes the 8-byte wide
 * host-command header; these packed structs are the firmware payloads only. */
struct iwl_mac_client_data_mld {
    uint8_t  is_assoc;
    uint8_t  esr_transition_timeout;
    uint16_t medium_sync_delay;
    uint16_t assoc_id;
    uint16_t reserved1;
    uint16_t data_policy;
    uint16_t reserved2;
    uint32_t ctwin;
} __attribute__((packed));     /* 16 bytes */

struct iwl_mac_wifi_gen_support_v2 {
    uint16_t he_support;
    uint16_t he_ap_support;
    uint32_t eht_support;
} __attribute__((packed));     /* 8 bytes */

struct iwl_mac_config_cmd {
    uint32_t id_and_color;          /* raw MAC id, not FW_ID_AND_COLOR */
    uint32_t action;
    uint32_t mac_type;
    uint8_t  local_mld_addr[6];
    uint16_t reserved_for_local_mld_addr;
    uint32_t filter_flags;
    struct iwl_mac_wifi_gen_support_v2 wifi_gen_v2;
    uint32_t nic_not_ack_enabled;
    struct iwl_mac_client_data_mld client;
} __attribute__((packed));     /* 52 bytes (trace: 60 incl. wide header) */

struct iwl_aux_sta_cmd {
    uint32_t sta_id;
    uint32_t lmac_id;
    uint8_t  mac_addr[6];
    uint16_t reserved_for_mac_addr;
} __attribute__((packed));     /* 16 bytes */

struct iwl_he_backoff_conf {
    uint16_t cwmin;
    uint16_t cwmax;
    uint16_t aifsn;
    uint16_t mu_time;
} __attribute__((packed));     /* 8 bytes */

struct iwl_npca_params {
    uint8_t  switch_delay;
    uint8_t  switch_back_delay;
    uint16_t min_dur_threshold;
    uint16_t flags;
    uint16_t reserved;
} __attribute__((packed));     /* 8 bytes */

struct iwl_link_config_cmd {
    uint32_t action;
    uint32_t link_id;              /* raw link id */
    uint32_t mac_id;               /* raw MAC id */
    uint32_t phy_id;               /* raw PHY id or IWL_FW_CTXT_INVALID */
    uint8_t  local_link_addr[6];
    uint16_t reserved_for_local_link_addr;
    uint32_t modify_mask;
    uint32_t active;
    uint8_t  block_tx;
    uint8_t  modify_bandwidth;
    uint8_t  reserved1[2];
    uint32_t cck_rates;
    uint32_t ofdm_rates;
    uint32_t cck_short_preamble;
    uint32_t short_slot;
    uint32_t protection_flags;
    uint32_t qos_flags;
    struct iwl_ac_qos ac[5];
    uint8_t  htc_trig_based_pkt_ext;
    uint8_t  rand_alloc_ecwmin;
    uint8_t  rand_alloc_ecwmax;
    uint8_t  ndp_fdbk_buff_th_exp;
    struct iwl_he_backoff_conf trig_based_txf[4];
    uint32_t bi;
    uint32_t dtim_interval;
    uint16_t puncture_mask;
    uint16_t frame_time_rts_th;
    uint32_t flags;
    uint32_t flags_mask;
    uint8_t  ref_bssid_addr[6];
    uint16_t reserved_for_ref_bssid_addr;
    uint8_t  bssid_index;
    uint8_t  bss_color;
    uint8_t  spec_link_id;
    uint8_t  ul_mu_data_disable;
    uint8_t  ibss_bssid_addr[6];
    uint16_t reserved_for_ibss_bssid_addr;
    struct iwl_npca_params npca_params;
    struct iwl_ac_qos prio_edca_params;
    uint32_t reserved3[4];
} __attribute__((packed));     /* 208 bytes (trace: 216 incl. wide header) */

struct iwl_sta_cfg_cmd_v1 {
    uint32_t sta_id;
    uint32_t link_id;
    uint8_t  peer_mld_address[6];
    uint16_t reserved_for_peer_mld_address;
    uint8_t  peer_link_address[6];
    uint16_t reserved_for_peer_link_address;
    uint32_t station_type;
    uint32_t assoc_id;
    uint32_t beamform_flags;
    uint32_t mfp;
    uint32_t mimo;
    uint32_t mimo_protection;
    uint32_t ack_enabled;
    uint32_t trig_rnd_alloc;
    uint32_t tx_ampdu_spacing;
    uint32_t tx_ampdu_max_size;
    uint32_t sp_length;
    uint32_t uapsd_acs;
    uint8_t  pkt_ext_qam_th[20];
    uint32_t htc_flags;
} __attribute__((packed));     /* 96 bytes (trace: 104 incl. wide header) */

struct iwl_mac_power_cmd {
    uint32_t id_and_color;
    uint16_t flags;
    uint16_t keep_alive_seconds;
    uint32_t rx_data_timeout;
    uint32_t tx_data_timeout;
    uint32_t rx_data_timeout_uapsd;
    uint32_t tx_data_timeout_uapsd;
    uint8_t  lprx_rssi_threshold;
    uint8_t  skip_dtim_periods;
    uint16_t snooze_interval;
    uint16_t snooze_window;
    uint8_t  snooze_step;
    uint8_t  qndp_tid;
    uint8_t  uapsd_ac_flags;
    uint8_t  uapsd_max_sp;
    uint8_t  heavy_tx_thld_packets;
    uint8_t  heavy_rx_thld_packets;
    uint8_t  heavy_tx_thld_percentage;
    uint8_t  heavy_rx_thld_percentage;
    uint8_t  limited_ps_threshold;
    uint8_t  reserved;
} __attribute__((packed));     /* 40 bytes (trace: 48 incl. wide header) */

struct iwl_tlc_config_cmd_v4 {
    uint8_t  sta_id;
    uint8_t  reserved1[3];
    uint8_t  max_ch_width;
    uint8_t  mode;
    uint8_t  chains;
    uint8_t  sgi_ch_width_supp;
    uint16_t flags;
    uint16_t non_ht_rates;
    uint16_t ht_rates[2][3];
    uint16_t max_mpdu_len;
    uint16_t max_tx_op;
} __attribute__((packed));     /* 28 bytes (trace: 36 incl. wide header) */

struct iwl_session_prot_cmd {
    uint32_t id_and_color;          /* command v1 uses raw MAC id */
    uint32_t action;
    uint32_t conf_id;
    uint32_t duration_tu;
    uint32_t repetition_count;
    uint32_t interval;
} __attribute__((packed));     /* 24 bytes (trace: 32 incl. wide header) */

struct iwl_rlc_config_cmd {
    uint32_t phy_id;
    uint32_t rx_chain_info;
    uint32_t rlc_reserved;
    uint32_t sad_chain_a_sad_mode;
    uint32_t sad_chain_b_sad_mode;
    uint32_t sad_mac_id;
    uint32_t sad_reserved;
    uint8_t  flags;
    uint8_t  reserved[3];
} __attribute__((packed));     /* 32 bytes (trace: 40 incl. wide header) */

typedef char iwl_assert_mac_config_cmd_size[(sizeof(struct iwl_mac_config_cmd) == 52) ? 1 : -1];
typedef char iwl_assert_aux_sta_cmd_size[(sizeof(struct iwl_aux_sta_cmd) == 16) ? 1 : -1];
typedef char iwl_assert_link_config_cmd_size[(sizeof(struct iwl_link_config_cmd) == 208) ? 1 : -1];
typedef char iwl_assert_sta_cfg_cmd_v1_size[(sizeof(struct iwl_sta_cfg_cmd_v1) == 96) ? 1 : -1];
typedef char iwl_assert_mac_power_cmd_size[(sizeof(struct iwl_mac_power_cmd) == 40) ? 1 : -1];
typedef char iwl_assert_tlc_config_cmd_v4_size[(sizeof(struct iwl_tlc_config_cmd_v4) == 28) ? 1 : -1];
typedef char iwl_assert_session_prot_cmd_size[(sizeof(struct iwl_session_prot_cmd) == 24) ? 1 : -1];
typedef char iwl_assert_rlc_config_cmd_size[(sizeof(struct iwl_rlc_config_cmd) == 32) ? 1 : -1];

int iwl_cmd_scan_setup_probe(void) {
    if (!s_rt_ready) {
        term_puts("iwl: scan setup needs RT probe; running wifirt first\n");
        if (!iwl_cmd_runtime_probe())
            return 0;
    }

    /* DQA_ENABLE_CMD intentionally skipped for now.
     * Linux sends DQA at iwl_mvm_up() line 1602, AFTER iwl_configure_rxq()
     * and iwl_send_rss_cfg_cmd().  Sending DQA before those two prerequisites
     * asserted firmware (CSR_INT=02000000 SW_ERR) on the first hardware run.
     * For a scan-only smoke test we may not need DQA at all — proceed
     * directly to PHY_CONTEXT_CMD ADD which is the minimum dependency for
     * scan request.  If ADD_STA / SCAN later fail, revisit and add the
     * full RXQ_CONFIG → RSS_CONFIG → DQA_ENABLE chain.                   */
    int r;

    /* ── PHY_CONTEXT_CMD ADD ctx=0 ─────────────────────────────────────── */
    /* Default: 2.4 GHz channel 1, 20 MHz, single antenna A (1x1).  Channel
     * is the smallest possible — we'll switch as needed when we run scan. */
    struct iwl_phy_context_cmd_v3 phy = {0};
    phy.id_and_color  = IWL_FW_ID_AND_COLOR(0, 1);
    phy.action        = FW_CTXT_ACTION_ADD;
    phy.channel       = 1;
    phy.band          = IWL_PHY_BAND_24;
    phy.width         = IWL_PHY_CHANNEL_MODE20;
    phy.ctrl_pos      = 0;
    phy.lmac_id       = 0;
    /* Rx chain config: ant A valid (=1), idle=1, active=1 → 0x1402.
     * NB: when we have multiple antennae from NVM we'd use s_nvm_rx_chains. */
    phy.rxchain_info  = IWL_RXCHAIN_VAL_BIT(1) |
                        IWL_RXCHAIN_IDLE(1)    |
                        IWL_RXCHAIN_ACTIVE(1);
    phy.dsp_cfg_flags = 0;
    phy.reserved      = 0;

    term_puts("iwl: scan setup: PHY_CONTEXT_CMD ADD ctx=0 ch=1 band=2.4 ...\n");
    r = iwl_send_cmd(IWL_GROUP_LONG, IWL_CMD_PHY_CONTEXT,
                     &phy, sizeof(phy), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: PHY_CONTEXT_CMD ADD failed\n");
        return 0;
    }

    /* ── MLD MAC_CONFIG_CMD ADD id=0 type=BSS_STA (unassociated) ───────────
     * Linux's AX201 path uses MLD MAC_CONFIG/LINK_CONFIG before scan and
     * association.  Mixing legacy MAC_CONTEXT with MLD LINK_CONFIG asserts
     * the firmware when LINK_CONFIG tries to reference a MAC that wasn't
     * created by the MLD API. */
    uint8_t my_mac[6]; iwl_mac(my_mac);
    struct iwl_mac_config_cmd mac_cfg = {0};
    mac_cfg.id_and_color = 0;
    mac_cfg.action       = FW_CTXT_ACTION_ADD;
    mac_cfg.mac_type     = FW_MAC_TYPE_BSS_STA;
    for (int i = 0; i < 6; i++) mac_cfg.local_mld_addr[i] = my_mac[i];
    mac_cfg.filter_flags = MAC_CFG_FILTER_ACCEPT_GRP |
                           MAC_CFG_FILTER_ACCEPT_BEACON;
    mac_cfg.client.is_assoc = 0;

    term_puts("iwl: scan setup: MAC_CONFIG ADD id=0 type=BSS_STA ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_MAC_CONFIG,
                     &mac_cfg, sizeof(mac_cfg), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: MAC_CONFIG ADD failed\n");
        return 0;
    }
    s_mld_mac_added = 1;

    /* ── SCAN_CFG_CMD (reduced, 12 bytes for Qu firmware) ──────────────────
     * REQUIRED before any SCAN_REQ_UMAC.  Linux sends this in iwl_mvm_up
     * via iwl_mvm_config_scan; without it firmware asserts on the first
     * scan request.  For Qu-c0-hr-b0-77 the reduced 12-byte form applies
     * (IWL_UCODE_TLV_API_REDUCED_SCAN_CONFIG).
     *
     *   struct iwl_scan_config {
     *       u8 enable_cam_mode; u8 enable_promiscouos_mode;
     *       u8 bcast_sta_id;    u8 reserved;
     *       __le32 tx_chains;   __le32 rx_chains;
     *   };
     */
    struct {
        uint8_t  enable_cam_mode;
        uint8_t  enable_promiscuous_mode;
        uint8_t  bcast_sta_id;
        uint8_t  reserved;
        uint32_t tx_chains;
        uint32_t rx_chains;
    } __attribute__((packed)) scan_cfg = {0};
    /* For ver >= 5 (this firmware), bcast_sta_id is ignored.  tx/rx chains
     * are the NVM-advertised antenna masks (NVM reported tx=3 rx=3 → 0x3). */
    scan_cfg.tx_chains = s_nvm_tx_chains & 0xFu;
    scan_cfg.rx_chains = s_nvm_rx_chains & 0xFu;
    term_puts("iwl: scan setup: SCAN_CFG tx="); prh32(scan_cfg.tx_chains);
    term_puts(" rx=");                          prh32(scan_cfg.rx_chains);
    term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_LONG, IWL_CMD_SCAN_CFG,
                     &scan_cfg, sizeof(scan_cfg), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: SCAN_CFG_CMD failed\n");
        return 0;
    }

    /* ── MLD LINK_CONFIG_CMD ADD link=0 on MAC 0, initially inactive. ────── */
    struct iwl_link_config_cmd link_cfg = {0};
    link_cfg.action  = FW_CTXT_ACTION_ADD;
    link_cfg.link_id = 0;
    link_cfg.mac_id  = 0;
    link_cfg.phy_id  = IWL_FW_CTXT_INVALID;
    for (int i = 0; i < 6; i++) link_cfg.local_link_addr[i] = my_mac[i];
    link_cfg.spec_link_id = 0;

    term_puts("iwl: scan setup: LINK_CONFIG ADD link=0 mac=0 ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_LINK_CONFIG,
                     &link_cfg, sizeof(link_cfg), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: LINK_CONFIG ADD failed\n");
        return 0;
    }
    s_mld_link_added = 1;

    s_scan_setup_ready = 1;
    term_puts("iwl: scan setup OK\n");
    return 1;
}

/* ── SCAN_REQ_UMAC v14 (Qu firmware) ──────────────────────────────────── */
/*
 * Linux uses TLV cmd-version dispatch — for Qu-c0-hr-b0-77 the cmd_ver is
 * 14, which uses the C struct iwl_scan_req_umac_v17 (covers v14..v17).
 * The fixed buffer is 1940 bytes regardless of channel count; firmware
 * iterates over channel_params.channel_config[0 .. count-1].
 */

#define IWL_SCAN_TWO_LMACS                2
#define IWL_SCAN_MAX_NUM_CHANS_V3         67
#define IWL_PROBE_OPTION_MAX              20
#define IWL_SCAN_SHORT_SSID_MAX_SIZE      8
#define IWL_SCAN_BSSID_MAX_SIZE           16
#define IWL_SCAN_NUM_BAND_PROBE_DATA_V_2  3
#define IWL_SCAN_OFFLOAD_PROBE_REQ_SIZE   512
#define IWL_MAX_SCHED_SCAN_PLANS          2

/* General-flags v2 enum (Linux fw/api/scan.h:996-1013). */
#define IWL_UMAC_SCAN_GEN_FLAGS_V2_PASS_ALL              (1u << 1)
#define IWL_UMAC_SCAN_GEN_FLAGS_V2_NTFY_ITER_COMPLETE    (1u << 2)
#define IWL_UMAC_SCAN_GEN_FLAGS_V2_ADAPTIVE_DWELL        (1u << 7)
#define IWL_UMAC_SCAN_GEN_FLAGS_V2_FORCE_PASSIVE         (1u << 11)

/* General-flags2 (Linux scan.h, set for v >= 14). */
#define IWL_UMAC_SCAN_GEN_FLAGS2_RESPECT_P2P_GO_LB       (1u << 0)
#define IWL_UMAC_SCAN_GEN_FLAGS2_RESPECT_P2P_GO_HB       (1u << 1)

/* Per-channel-cfg flags. */
#define IWL_SCAN_CHANNEL_FLAG_ENABLE_CHAN_ORDER          (1u << 5)

/* Scan dwell defaults: 10 TU active + 110 TU passive matches Linux's
 * unassoc regular scan timing for AX201/Qu. */
#define IWL_SCAN_DWELL_ACTIVE                            10   /* TU */
#define IWL_SCAN_DWELL_PASSIVE                           110  /* TU */
#define IWL_SCAN_ADWELL_DEFAULT_LB_N_APS                 2
#define IWL_SCAN_ADWELL_DEFAULT_HB_N_APS                 5
#define IWL_SCAN_ADWELL_DEFAULT_N_APS_SOCIAL             10
#define IWL_SCAN_ADWELL_MAX_BUDGET_FULL_SCAN             300
#define IWL_SCAN_PRIORITY_EXT_6                          6

struct iwl_scan_general_params_v11 {
    uint16_t flags;
    uint8_t  reserved;
    uint8_t  scan_start_mac_or_link_id;
    uint8_t  active_dwell[IWL_SCAN_TWO_LMACS];
    uint8_t  adwell_default_2g;
    uint8_t  adwell_default_5g;
    uint8_t  adwell_default_social_chn;
    uint8_t  flags2;
    uint16_t adwell_max_budget;
    uint32_t max_out_of_time[IWL_SCAN_TWO_LMACS];
    uint32_t suspend_time[IWL_SCAN_TWO_LMACS];
    uint32_t scan_priority;
    uint8_t  passive_dwell[IWL_SCAN_TWO_LMACS];
    uint8_t  num_of_fragments[IWL_SCAN_TWO_LMACS];
} __attribute__((packed));     /* 36 bytes */

struct iwl_scan_channel_cfg_umac_v2 {
    uint32_t flags;
    uint8_t  channel_num;
    uint8_t  band;
    uint8_t  iter_count;
    uint8_t  iter_interval;
} __attribute__((packed));     /* 8 bytes */

struct iwl_scan_channel_params_v7 {
    uint8_t  flags;
    uint8_t  count;
    uint8_t  n_aps_override[2];
    struct iwl_scan_channel_cfg_umac_v2 channel_config[IWL_SCAN_MAX_NUM_CHANS_V3];
} __attribute__((packed));     /* 4 + 67*8 = 540 bytes */

struct iwl_scan_umac_schedule {
    uint16_t interval;
    uint8_t  iter_count;
    uint8_t  reserved;
} __attribute__((packed));     /* 4 bytes */

struct iwl_scan_periodic_parms_v1 {
    struct iwl_scan_umac_schedule schedule[IWL_MAX_SCHED_SCAN_PLANS];
    uint16_t delay;
    uint16_t reserved;
} __attribute__((packed));     /* 12 bytes */

struct iwl_scan_probe_segment {
    uint16_t offset;
    uint16_t len;
} __attribute__((packed));     /* 4 bytes */

struct iwl_scan_probe_req {
    struct iwl_scan_probe_segment mac_header;
    struct iwl_scan_probe_segment band_data[IWL_SCAN_NUM_BAND_PROBE_DATA_V_2];
    struct iwl_scan_probe_segment common_data;
    uint8_t buf[IWL_SCAN_OFFLOAD_PROBE_REQ_SIZE];
} __attribute__((packed));     /* 4 + 12 + 4 + 512 = 532 bytes */

struct iwl_ssid_ie {
    uint8_t id;
    uint8_t len;
    uint8_t ssid[32];
} __attribute__((packed));     /* 34 bytes */

struct iwl_scan_probe_params_v4 {
    struct iwl_scan_probe_req preq;
    uint8_t  short_ssid_num;
    uint8_t  bssid_num;
    uint16_t reserved;
    struct iwl_ssid_ie direct_scan[IWL_PROBE_OPTION_MAX];
    uint32_t short_ssid[IWL_SCAN_SHORT_SSID_MAX_SIZE];
    uint8_t  bssid_array[IWL_SCAN_BSSID_MAX_SIZE][6];
} __attribute__((packed));     /* 532 + 4 + 680 + 32 + 96 = 1344 bytes */

struct iwl_scan_req_params_v17 {
    struct iwl_scan_general_params_v11 general_params;
    struct iwl_scan_channel_params_v7  channel_params;
    struct iwl_scan_periodic_parms_v1  periodic_params;
    struct iwl_scan_probe_params_v4    probe_params;
} __attribute__((packed));     /* 36 + 540 + 12 + 1344 = 1932 bytes */

struct iwl_scan_req_umac_v17 {
    uint32_t uid;
    uint32_t ooc_priority;
    struct iwl_scan_req_params_v17 scan_params;
} __attribute__((packed));     /* 8 + 1932 = 1940 bytes */

/* Notifications. */
struct iwl_umac_scan_complete {
    uint32_t uid;
    uint8_t  last_schedule;
    uint8_t  last_iter;
    uint8_t  status;
    uint8_t  ebs_status;
    uint32_t time_from_last_iter;
    uint32_t reserved;
} __attribute__((packed));     /* 16 bytes */

struct iwl_scan_results_notif {
    uint8_t  channel;
    uint8_t  band;
    uint8_t  probe_status;
    uint8_t  num_probe_not_sent;
    uint32_t duration;
} __attribute__((packed));     /* 8 bytes */

struct iwl_umac_scan_iter_complete_notif {
    uint32_t uid;
    uint8_t  scanned_channels;
    uint8_t  status;
    uint8_t  bt_status;
    uint8_t  last_channel;
    uint64_t start_tsf;
    /* struct iwl_scan_results_notif results[]; */
} __attribute__((packed));     /* 16 bytes header + N×8 results */

/* iwl_rx_mpdu_desc — Linux fw/api/rx.h.  AX201/Qu uses the v1 substruct
 * (pre-AX210), so total prefix size is 48 bytes.  802.11 frame begins at
 * offset 48 from the start of REPLY_RX_MPDU_CMD payload. */
struct iwl_rx_mpdu_desc_v1_min {
    uint32_t rss_hash_or_phy_data2;       /* @20 */
    uint32_t filter_match_or_phy_data3;   /* @24 */
    uint32_t rate_n_flags;                /* @28 */
    uint8_t  energy_a;                    /* @32 */
    uint8_t  energy_b;                    /* @33 */
    uint8_t  channel;                     /* @34 */
    uint8_t  mac_context;                 /* @35 */
    uint32_t gp2_on_air_rise;             /* @36 */
    uint64_t tsf_on_air_rise;             /* @40..47 */
} __attribute__((packed));    /* 28 bytes */

struct iwl_rx_mpdu_desc {
    uint16_t mpdu_len;          /* @0  — 802.11 frame length (excl prefix) */
    uint8_t  mac_flags1;        /* @2  */
    uint8_t  mac_flags2;        /* @3  */
    uint8_t  amsdu_info;        /* @4  */
    uint16_t phy_info;          /* @5..6 (unaligned LE16) */
    uint8_t  mac_phy_idx;       /* @7  */
    uint16_t raw_csum;          /* @8  */
    uint16_t l3l4_flags;        /* @10 */
    uint32_t status;            /* @12 */
    uint32_t reorder_data;      /* @16 */
    struct iwl_rx_mpdu_desc_v1_min v1;
} __attribute__((packed));    /* 48 bytes total */

#define IWL_RX_MPDU_STATUS_MIC_OK      (1u << 6)
#define IWL_RX_MPDU_STATUS_SEC_MASK    (0x7u << 8)
#define IWL_RX_MPDU_STATUS_SEC_NONE    (0x0u << 8)
#define IWL_RX_MPDU_STATUS_SEC_CCM     (0x2u << 8)
#define IWL_RX_MPDU_MFLG2_PAD          0x20u

/* Walk the 802.11 IE list starting at `ie`, total `ie_len` bytes, looking
 * for `wanted_id` (0 = SSID, 3 = DS Parameter Set, etc.).  Returns a
 * pointer to the IE data (NOT including the 2-byte [id,len] header), or
 * NULL if not found.  Sets *out_len to the IE data length. */
static const uint8_t *find_ie(const uint8_t *ie, uint16_t ie_len,
                              uint8_t wanted_id, uint8_t *out_len) {
    uint16_t i = 0;
    while (i + 2 <= ie_len) {
        uint8_t id  = ie[i];
        uint8_t len = ie[i + 1];
        if (i + 2 + len > ie_len) return (void *)0;   /* malformed */
        if (id == wanted_id) {
            if (out_len) *out_len = len;
            return ie + i + 2;
        }
        i = (uint16_t)(i + 2 + len);
    }
    return (void *)0;
}

/* Discovered-AP cache (see iwl_cmd.h).  Reset at the start of every scan
 * via iwl_aps_reset(); populated by print_beacon() as MPDUs arrive. */
#define IWL_AP_MAX  16
static struct iwl_ap_info s_aps[IWL_AP_MAX];
static uint8_t            s_n_aps;

static void iwl_aps_reset(void) {
    s_n_aps = 0;
    for (uint64_t i = 0; i < sizeof(s_aps); i++) ((uint8_t *)s_aps)[i] = 0;
}

int iwl_ap_count(void) { return (int)s_n_aps; }
const struct iwl_ap_info *iwl_ap_get(int i) {
    if (i < 0 || i >= (int)s_n_aps) return (const struct iwl_ap_info *)0;
    return &s_aps[i];
}

/* Find existing AP by BSSID, or return a fresh slot (0..IWL_AP_MAX-1) and
 * bump s_n_aps.  Returns NULL when the cache is full and the BSSID is new. */
static struct iwl_ap_info *aps_lookup_or_add(const uint8_t bssid[6]) {
    for (uint8_t i = 0; i < s_n_aps; i++) {
        const uint8_t *b = s_aps[i].bssid;
        if (b[0] == bssid[0] && b[1] == bssid[1] && b[2] == bssid[2] &&
            b[3] == bssid[3] && b[4] == bssid[4] && b[5] == bssid[5])
            return &s_aps[i];
    }
    if (s_n_aps >= IWL_AP_MAX) return (struct iwl_ap_info *)0;
    struct iwl_ap_info *a = &s_aps[s_n_aps++];
    for (int i = 0; i < 6; i++) a->bssid[i] = bssid[i];
    return a;
}

/* Print a beacon / probe response one-liner AND cache the AP info. */
static void print_beacon(const struct iwl_rx_packet *pkt, uint32_t plen) {
    /* iwl_rx_mpdu_desc starts at pkt->data; 802.11 frame at desc + 48. */
    if (plen < 4u + 48u + 36u) return;   /* too short — header + fixed body */
    const struct iwl_rx_mpdu_desc *d = (const struct iwl_rx_mpdu_desc *)pkt->data;
    const uint8_t *frame = pkt->data + 48;
    uint16_t frame_len = d->mpdu_len;
    if (4u + 48u + frame_len > plen) frame_len = (uint16_t)(plen - 4u - 48u);

    /* 802.11 mgmt frame header at offset 0:
     *   FC(2) Dur(2) DA(6) SA(6) BSSID(6) SeqCtl(2) = 24 bytes.
     * Then fixed beacon body: timestamp(8) bcn_int(2) cap(2) = 12 bytes.
     * IEs start at offset 36 within the frame. */
    if (frame_len < 36) return;
    const uint8_t *bssid = frame + 16;        /* offset 16 in 802.11 hdr */
    uint16_t ie_off = 36;
    uint16_t ie_len = (uint16_t)(frame_len - ie_off);

    uint8_t ssid_len = 0;
    const uint8_t *ssid = find_ie(frame + ie_off, ie_len, WLAN_EID_SSID,
                                  &ssid_len);

    /* capability_info is at body offset 10 (= frame offset 34): little-endian
     * 16-bit field. */
    uint16_t bi  = (uint16_t)frame[32] | ((uint16_t)frame[33] << 8);
    uint16_t cap = (uint16_t)frame[34] | ((uint16_t)frame[35] << 8);

    /* Supported Rates IE (id=1) + optional Extended Supported Rates (id=50).
     * Concatenate up to 16 bytes total into the cache. */
    uint8_t sr_len = 0, ext_len = 0, rsn_len = 0;
    const uint8_t *sr  = find_ie(frame + ie_off, ie_len, WLAN_EID_SUPP_RATES,
                                 &sr_len);
    const uint8_t *ext = find_ie(frame + ie_off, ie_len,
                                 WLAN_EID_EXT_SUPP_RATES, &ext_len);
    const uint8_t *rsn = find_ie(frame + ie_off, ie_len, WLAN_EID_RSN,
                                 &rsn_len);

    /* Cache (or update) the AP entry, keyed by BSSID.  Probe-responses
     * carry the real SSID for hidden APs — keep the more informative
     * value, never overwrite a known SSID with an empty one. */
    struct iwl_ap_info *a = aps_lookup_or_add(bssid);
    if (a) {
        a->channel    = d->v1.channel;
        a->rssi       = d->v1.energy_a;
        a->capability = cap;
        a->beacon_interval = bi ? bi : 100;
        if (ssid && ssid_len && ssid_len <= 32) {
            a->ssid_len = ssid_len;
            for (uint8_t i = 0; i < ssid_len; i++) a->ssid[i] = ssid[i];
        }
        a->rates_len = 0;
        if (sr && sr_len) {
            uint8_t n = sr_len > 8 ? 8 : sr_len;
            for (uint8_t i = 0; i < n; i++) a->rates[a->rates_len++] = sr[i];
        }
        if (ext && ext_len) {
            uint8_t room = (uint8_t)(sizeof(a->rates) - a->rates_len);
            uint8_t n = ext_len > room ? room : ext_len;
            for (uint8_t i = 0; i < n; i++) a->rates[a->rates_len++] = ext[i];
        }
        if (rsn && (uint16_t)rsn_len + 2u <= sizeof(a->rsn_ie)) {
            a->rsn_ie_len = (uint8_t)(rsn_len + 2u);
            a->rsn_ie[0] = WLAN_EID_RSN;
            a->rsn_ie[1] = rsn_len;
            for (uint8_t i = 0; i < rsn_len; i++) a->rsn_ie[2 + i] = rsn[i];
        }
    }

    /* Decode frame_control type/subtype.  802.11 mgmt subtypes:
     *   8 (FC[0]=0x80) = Beacon
     *   5 (FC[0]=0x50) = Probe Response
     *   4 (FC[0]=0x40) = Probe Request   */
    uint8_t fc0 = frame[0];
    const char *kind = "?";
    if (fc0 == 0x80) kind = "bcn ";
    else if (fc0 == 0x50) kind = "prsp";
    else if (fc0 == 0x40) kind = "preq";

    /* RSSI: Linux uses max(energy_a, energy_b) as a positive dBm magnitude.
     * For us we just print energy_a. */
    term_puts("    AP "); term_puts(kind); term_putchar(' ');
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(bssid[i]);
    }
    term_puts("  ch=");   prh8(d->v1.channel);
    term_puts(" rssi=-"); prh8(d->v1.energy_a);
    term_puts(" ssid=");
    if (ssid && ssid_len) {
        if (ssid_len > 32) ssid_len = 32;
        term_putchar('"');
        for (uint8_t i = 0; i < ssid_len; i++) {
            uint8_t c = ssid[i];
            if (c >= 0x20 && c < 0x7f) term_putchar((char)c);
            else                       term_putchar('.');
        }
        term_putchar('"');
    } else {
        term_puts("(hidden)");
    }
    term_putchar('\n');
}

int iwl_cmd_scan_probe(void) {
    if (!s_scan_setup_ready) {
        term_puts("iwl: scan needs setup; running wifisetup first\n");
        if (!iwl_cmd_scan_setup_probe())
            return 0;
    }
    /* Active scan: firmware TXes broadcast probe-requests via the AUX STA
     * queue.  We don't TX from the host — Linux's UMAC scan path delegates
     * all probe TX to firmware, which uses the AUX STA + queue allocated
     * by iwl_cmd_txq_setup_probe.  Without these, firmware silently skips
     * the probes (or asserts SW_ERR). */
    if (!s_txq_ready) {
        term_puts("iwl: scan needs AUX TX queue; running wifitxq first\n");
        if (!iwl_cmd_txq_setup_probe())
            return 0;
    }

    /* Reset the AP cache; results from beacons/probe-responses arriving
     * during this scan iteration repopulate it via print_beacon(). */
    iwl_aps_reset();

    /* Allocate the 1940-byte scan command in heap (too big for the stack).
     * The buffer is zero-init then we fill only the fields we need. */
    static struct iwl_scan_req_umac_v17 scan;
    for (uint64_t i = 0; i < sizeof(scan); i++) ((uint8_t *)&scan)[i] = 0;

    scan.uid          = 1;     /* arbitrary, matches us in the response */
    scan.ooc_priority = IWL_SCAN_PRIORITY_EXT_6;

    struct iwl_scan_general_params_v11 *gp = &scan.scan_params.general_params;
    /* ACTIVE scan: firmware TXes broadcast probe-requests using the embedded
     * preq template via the AUX STA queue.  PASS_ALL ensures probe responses
     * + beacons get forwarded as REPLY_RX_MPDU_CMD.  ADAPTIVE_DWELL extends
     * the per-channel dwell up to adwell_max_budget when APs are heard,
     * matching Linux's default and giving probe-responses time to arrive. */
    gp->flags = IWL_UMAC_SCAN_GEN_FLAGS_V2_PASS_ALL |
                IWL_UMAC_SCAN_GEN_FLAGS_V2_NTFY_ITER_COMPLETE |
                IWL_UMAC_SCAN_GEN_FLAGS_V2_ADAPTIVE_DWELL;
    gp->scan_start_mac_or_link_id = 0;
    gp->active_dwell[0]  = IWL_SCAN_DWELL_ACTIVE;
    gp->active_dwell[1]  = IWL_SCAN_DWELL_ACTIVE;
    gp->adwell_default_2g = IWL_SCAN_ADWELL_DEFAULT_LB_N_APS;
    gp->adwell_default_5g = IWL_SCAN_ADWELL_DEFAULT_HB_N_APS;
    gp->adwell_default_social_chn = IWL_SCAN_ADWELL_DEFAULT_N_APS_SOCIAL;
    gp->flags2 = 0;
    gp->adwell_max_budget = IWL_SCAN_ADWELL_MAX_BUDGET_FULL_SCAN;
    gp->scan_priority = IWL_SCAN_PRIORITY_EXT_6;
    gp->passive_dwell[0] = IWL_SCAN_DWELL_PASSIVE;
    gp->passive_dwell[1] = IWL_SCAN_DWELL_PASSIVE;
    gp->num_of_fragments[0] = 0;
    gp->num_of_fragments[1] = 0;

    /* 3 channels: 1, 6, 11 — active scan on 2.4 GHz only.
     * Per-channel cfg.flags bit 0 = "TX probe request using direct_scan[0]
     * on this channel".  direct_scan[0] = wildcard (id=0,len=0) below so
     * each channel gets a broadcast probe request. */
    struct iwl_scan_channel_params_v7 *cp = &scan.scan_params.channel_params;
    cp->flags = IWL_SCAN_CHANNEL_FLAG_ENABLE_CHAN_ORDER;
    cp->count = 3;
    cp->n_aps_override[0] = 2;
    cp->n_aps_override[1] = 10;
    static const uint8_t chans[3] = { 1, 6, 11 };
    for (uint8_t i = 0; i < 3; i++) {
        /* bit 0 = use direct_scan[0] on this channel (broadcast wildcard). */
        cp->channel_config[i].flags         = 0x01u;
        cp->channel_config[i].channel_num   = chans[i];
        cp->channel_config[i].band          = IWL_PHY_BAND_24;
        cp->channel_config[i].iter_count    = 1;
        cp->channel_config[i].iter_interval = 0;
    }

    /* One-shot: schedule[0] = {iter_count=1, interval=0}. */
    scan.scan_params.periodic_params.schedule[0].iter_count = 1;
    scan.scan_params.periodic_params.schedule[0].interval   = 0;

    /* ── Probe template (active scan, broadcast wildcard) ─────────────────
     * Linux iwl_mvm_build_scan_probe (mvm/scan.c:744-811):
     *   mac_header  = 24-byte 802.11 MAC hdr + 2-byte SSID IE (id=0, len=0)
     *   band_data[0] = Supported Rates IE for 2.4 GHz
     *   common_data = HT/VHT/HE caps; minimal probe → empty
     *
     * Firmware concatenates mac_header || band_data[band] || common_data
     * to build the on-air probe request.  Without rates IE in band_data,
     * many APs reject the probe.  direct_scan[] is left ZERO; the wildcard
     * lives in the embedded SSID IE.  Per-channel cfg.flags = 0 (no
     * directed-SSID bits set). */
    struct iwl_scan_probe_req *preq = &scan.scan_params.probe_params.preq;
    uint8_t mac[6];
    iwl_mac(mac);

    /* MAC header (24 bytes): FC + duration + DA + SA + BSSID + seq_ctl. */
    preq->buf[0] = 0x40;  /* FC subtype=Probe Req (4), type=Mgmt (0) */
    preq->buf[1] = 0x00;
    preq->buf[2] = 0x00;
    preq->buf[3] = 0x00;
    for (int i = 0; i < 6; i++) preq->buf[4  + i] = 0xFF;
    for (int i = 0; i < 6; i++) preq->buf[10 + i] = mac[i];
    for (int i = 0; i < 6; i++) preq->buf[16 + i] = 0xFF;
    preq->buf[22] = 0x00;
    preq->buf[23] = 0x00;
    /* Wildcard SSID IE (id=0, len=0). */
    preq->buf[24] = 0x00;
    preq->buf[25] = 0x00;
    /* 2.4 GHz Supported Rates IE: 1/2/5.5/11 Mbps basic + 6 Mbps non-basic. */
    preq->buf[26] = 0x01;          /* IE id = supported rates */
    preq->buf[27] = 0x05;          /* IE len = 5 rates */
    preq->buf[28] = 0x82;          /* 1.0  Mbps + basic */
    preq->buf[29] = 0x84;          /* 2.0  Mbps + basic */
    preq->buf[30] = 0x8B;          /* 5.5  Mbps + basic */
    preq->buf[31] = 0x96;          /* 11.0 Mbps + basic */
    preq->buf[32] = 0x0C;          /* 6.0  Mbps (non-basic) */

    preq->mac_header.offset = 0;
    preq->mac_header.len    = 26;   /* 24-byte MAC hdr + 2-byte wildcard SSID IE */
    preq->band_data[0].offset = 26;
    preq->band_data[0].len    = 7;  /* IE: id(1) + len(1) + 5 rates */
    preq->band_data[1].offset = 33;
    preq->band_data[1].len    = 0;
    preq->band_data[2].offset = 33;
    preq->band_data[2].len    = 0;
    preq->common_data.offset  = 33;
    preq->common_data.len     = 0;

    /* direct_scan[0] = wildcard SSID (id=0, len=0) — broadcast probe.
     * Per-channel cfg.flags bit 0 references this entry; firmware copies
     * the SSID IE bytes into the on-air probe-request.  Linux always
     * provisions at least one direct_scan entry for active scans. */
    scan.scan_params.probe_params.direct_scan[0].id  = 0;   /* WLAN_EID_SSID */
    scan.scan_params.probe_params.direct_scan[0].len = 0;   /* wildcard */

    term_puts("iwl: scan: SCAN_REQ_UMAC v15 ACTIVE 2.4 ch{1,6,11} ...\n");
    /* Hex dump scan[0..0x60] (covers uid+ooc+general_params+channel_params
     * header + first 3 channel_config entries 0x30..0x47).  Also print
     * the runtime sizeof for struct sanity. */
    term_puts("  sizeof: scan="); prh16((uint16_t)sizeof(scan));
    term_puts(" cfg=");           prh16((uint16_t)sizeof(struct iwl_scan_channel_cfg_umac_v2));
    term_puts(" cp=");            prh16((uint16_t)sizeof(struct iwl_scan_channel_params_v7));
    term_putchar('\n');
    {
        const uint8_t *p = (const uint8_t *)&scan;
        for (int row = 0; row < 6; row++) {
            term_puts("  scan@"); prh8((uint8_t)(row * 16)); term_puts(": ");
            for (int col = 0; col < 16; col++) {
                prh8(p[row * 16 + col]);
                term_putchar(' ');
            }
            term_putchar('\n');
        }
    }
    s_scan_collecting = 1;
    int r = iwl_send_cmd(IWL_GROUP_LONG, IWL_CMD_SCAN_REQ_UMAC,
                         &scan, sizeof(scan), (void *)0, 0);
    if (r < 0) {
        s_scan_collecting = 0;
        term_puts("iwl: SCAN_REQ_UMAC failed\n");
        (void)iwl_cmd_txq_teardown_probe();
        return 0;
    }

    /* Wait for SCAN_COMPLETE_UMAC — up to 5 seconds (3 channels × ~110 TU
     * passive + slack).  Drain all RX events; print iter-complete and any
     * MPDU notifications encountered along the way.
     *
     * iwl_send_cmd feeds probe responses to print_beacon while
     * s_scan_collecting is set, so the ring is never rewound. */

    term_puts("iwl: scan: waiting for SCAN_COMPLETE_UMAC ...\n");
    uint32_t *stts32 = iwl_rx_stts_raw();
    if (!stts32) {
        s_scan_collecting = 0;
        return 0;
    }
    struct iwl_rb_status *rs = (struct iwl_rb_status *)stts32;

    int saw_complete = 0;
    int rx_mpdu_count = 0;
    int iter_count = 0;
    for (int t = 0; t < 500 && !saw_complete; t++) {
        iwl_delay(10000);   /* 10 ms */
        uint16_t cur = rx_closed_index(rs);
        while (s_rx_read != cur) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;
                term_puts("    RX@"); prh16(s_rx_read);
                term_puts(": cmd="); prh8(pkt->hdr.cmd);
                term_puts(" g=");    prh8(pkt->hdr.group_id);
                term_puts(" seq=");  prh16(pkt->hdr.sequence);
                term_puts(" len=");  prh32(plen); term_putchar('\n');

                /* SCAN_COMPLETE_UMAC and SCAN_ITER_COMPLETE_UMAC arrive in
                 * GROUP=0 on this firmware (despite Linux registering them
                 * via WIDE_ID(LONG_GROUP, ...)).  Empirically firmware emits
                 * them as legacy short-header notifications.  Match by cmd
                 * id alone and ignore the group. */
                if (pkt->hdr.cmd == IWL_CMD_SCAN_COMPLETE_UMAC) {
                    saw_complete = 1;
                    if (plen >= 4 + sizeof(struct iwl_umac_scan_complete)) {
                        const struct iwl_umac_scan_complete *c =
                            (const struct iwl_umac_scan_complete *)pkt->data;
                        term_puts("    SCAN_COMPLETE: uid="); prh32(c->uid);
                        term_puts(" status="); prh8(c->status);
                        term_puts(" ebs="); prh8(c->ebs_status);
                        term_puts(" iters="); prh8(c->last_iter);
                        term_putchar('\n');
                    }
                } else if (pkt->hdr.cmd == IWL_CMD_SCAN_ITER_COMPLETE_UMAC) {
                    iter_count++;
                    if (plen >= 4 + sizeof(struct iwl_umac_scan_iter_complete_notif)) {
                        const struct iwl_umac_scan_iter_complete_notif *n =
                            (const struct iwl_umac_scan_iter_complete_notif *)pkt->data;
                        term_puts("    SCAN_ITER: scanned=");
                        prh8(n->scanned_channels);
                        term_puts(" status="); prh8(n->status);
                        term_putchar('\n');
                    }
                } else if (pkt->hdr.cmd == 0xC1 /* REPLY_RX_MPDU_CMD */) {
                    rx_mpdu_count++;
                    print_beacon(pkt, plen);
                }
            }
            rx_consume_read(vid);
        }
    }
    s_scan_collecting = 0;

    term_puts("iwl: scan: iter_complete×"); prh32((uint32_t)iter_count);
    term_puts(" rx_mpdu×"); prh32((uint32_t)rx_mpdu_count);
    term_putchar('\n');

    if (!saw_complete) {
        term_puts("iwl: scan: no SCAN_COMPLETE in 5s\n");
        (void)iwl_cmd_txq_teardown_probe();
        return 0;
    }
    if (!iwl_cmd_txq_teardown_probe()) {
        term_puts("iwl: scan: AUX TX queue teardown failed\n");
        return 0;
    }
    term_puts("iwl: scan OK\n");
    return 1;
}

/* ── TX management queue setup (toward active scan / association) ──────── */
/*
 * Linux gen2 TX path: a separate TFD ring for management/data frames,
 * registered with firmware via SCD_QUEUE_CONFIG.  Firmware returns the
 * assigned queue_id; live Linux encodes it in HBUS_TARG_WRPTR bits [20:16].
 *
 * For 22000-family, per Linux fw/api/txq.h:
 *   IWL_MGMT_QUEUE_SIZE = 16, IWL_DEFAULT_QUEUE_SIZE = 256
 *   cb_size = log2(size)-3
 *   bc_tbl: struct iwlagn_scd_bc_tbl = 1024 × __le16    (2048 bytes)
 *   tid    = 8 for AUX/internal, 15 for AP management, 0 for non-QoS data.
 *   sta_id = the aux/self STA index (1 here; AP peer uses STA id 0).
 */

/* iwl_mvm_add_sta_cmd v10/v12 (Linux fw/api/sta.h, 48 bytes packed).
 * Used for AUX STA setup before TX queue allocation. */
struct iwl_mvm_add_sta_cmd {
    uint8_t  add_modify;
    uint8_t  awake_acs;
    uint16_t tid_disable_tx;
    uint32_t mac_id_n_color;
    uint8_t  addr[6];
    uint16_t reserved2;
    uint8_t  sta_id;
    uint8_t  modify_mask;
    uint16_t reserved3;
    uint32_t station_flags;
    uint32_t station_flags_msk;
    uint8_t  add_immediate_ba_tid;
    uint8_t  remove_immediate_ba_tid;
    uint16_t add_immediate_ba_ssn;
    uint16_t sleep_tx_count;
    uint8_t  sleep_state_flags;
    uint8_t  station_type;
    uint16_t assoc_id;
    uint16_t beamform_flags;
    uint32_t tfd_queue_msk;
    uint16_t rx_ba_window;
    uint8_t  sp_length;
    uint8_t  uapsd_acs;
} __attribute__((packed));     /* 48 bytes */

/* SCD_QUEUE_CONFIG_CMD v3 (Linux fw/api/datapath.h:758-790).  Used on Qu
 * firmware which advertises this cmd at WIDE_ID(DATA_PATH_GROUP=0x5, 0x17)
 * with cmd_ver=3.  The legacy v0 path (LEGACY group cmd 0x1d) is NOT
 * advertised on this firmware and asserts SW_ERR.
 *
 * Discriminated by `operation` (ADD=0, REMOVE=1, MODIFY=2).  Only the ADD
 * variant is needed for our mgmt-queue setup. */
struct iwl_scd_queue_cfg_cmd_add {
    uint32_t sta_mask;          /* BIT(sta_id) — bitmap, not raw id */
    uint8_t  tid;
    uint8_t  reserved[3];
    uint32_t flags;             /* MUST be 0 for v3 (no ENABLE bit) */
    uint32_t cb_size;
    uint64_t bc_dram_addr;
    uint64_t tfdq_dram_addr;
} __attribute__((packed));      /* 32 bytes */

struct iwl_scd_queue_cfg_cmd {
    uint32_t operation;
    struct iwl_scd_queue_cfg_cmd_add add;
} __attribute__((packed));      /* 4 + 32 = 36 bytes */

struct iwl_sec_key_cmd {
    uint32_t action;
    union {
        struct {
            uint32_t sta_mask;
            uint32_t key_id;
            uint32_t key_flags;
            uint8_t  key[32];
            uint8_t  tkip_mic_rx_key[8];
            uint8_t  tkip_mic_tx_key[8];
            uint64_t rx_seq;
            uint64_t tx_seq;
        } __attribute__((packed)) add;
        struct {
            uint32_t sta_mask;
            uint32_t key_id;
            uint32_t key_flags;
        } __attribute__((packed)) remove;
    } __attribute__((packed)) u;
} __attribute__((packed));      /* 80 bytes */

#define IWL_SEC_KEY_FLAG_CIPHER_CCMP 0x02u
#define IWL_SEC_KEY_FLAG_MFP         0x20u
#define IWL_SEC_KEY_FLAG_MCAST_KEY   0x40u

struct iwl_tx_queue_cfg_rsp {
    uint16_t queue_number;
    uint16_t flags;
    uint16_t write_pointer;
    uint16_t reserved;
} __attribute__((packed));      /* 8 bytes */

static void ap_txq_clear_buffers(void) {
    if (s_aptxq_tfd_ring) {
        for (uint32_t i = 0; i < IWL_DATA_QUEUE_SIZE; i++)
            tfd_set_invalid(&s_aptxq_tfd_ring[i]);
    }
    if (s_aptxq_bc_tbl) {
        for (uint64_t i = 0; i < IWL_BC_TBL_SIZE * 2u; i++)
            s_aptxq_bc_tbl[i] = 0;
    }
    if (s_aptxq_cmd_buf)
        iwl_zero(s_aptxq_cmd_buf, IWL_AP_TX_CMD_STRIDE * IWL_DATA_QUEUE_SIZE);
    if (s_aptxq_first_tb)
        iwl_zero(s_aptxq_first_tb, IWL_FIRST_TB_STRIDE * IWL_DATA_QUEUE_SIZE);
}

static int ap_txq_add(uint8_t tid, const char *why) {
    ap_txq_clear_buffers();
    uint16_t slots = ap_txq_size_for_tid(tid);
    uint8_t cb_size = ap_txq_cb_size_for_slots(slots);

    struct iwl_scd_queue_cfg_cmd qc = {0};
    qc.operation             = IWL_SCD_QUEUE_ADD;
    qc.add.sta_mask          = (1u << IWL_AP_STA_ID);
    qc.add.tid               = tid;
    qc.add.flags             = 0;
    qc.add.cb_size           = cb_size;
    qc.add.bc_dram_addr      = s_aptxq_bc_tbl_phys;
    qc.add.tfdq_dram_addr    = s_aptxq_tfd_phys;

    term_puts("iwl: ");
    term_puts(why);
    term_puts(": SCD_QUEUE_CONFIG ADD sta_mask=0x");
    prh32(qc.add.sta_mask);
    term_puts(" tid=");
    prh8(tid);
    term_puts(" slots=");
    prh16(slots);
    term_puts(" cb=");
    prh8(cb_size);
    term_puts(" ...\n");

    struct iwl_tx_queue_cfg_rsp qrsp = {0};
    int r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_SCD_QUEUE_CONFIG,
                         &qc, sizeof(qc), &qrsp, sizeof(qrsp));
    if (r < 0) {
        term_puts("iwl: SCD_QUEUE_CONFIG AP failed\n");
        iwl_dump_fw_error();
        return 0;
    }

    s_aptxq_id     = qrsp.queue_number;
    s_aptxq_tid    = tid;
    s_aptxq_size   = slots;
    s_aptxq_rd_ptr = (uint16_t)(qrsp.write_pointer & 0xFFu);
    s_aptxq_wr_ptr = (uint16_t)(qrsp.write_pointer & 0xFFu);
    term_puts("       queue_id="); prh16(s_aptxq_id);
    term_puts(" tid=");             prh8(s_aptxq_tid);
    term_puts(" size=");            prh16(s_aptxq_size);
    term_puts(" rd_ptr=");          prh16(s_aptxq_rd_ptr);
    term_puts(" wr_ptr=");          prh16(s_aptxq_wr_ptr);
    term_putchar('\n');
    return 1;
}

static int ap_txq_remove(uint8_t tid, const char *why) {
    if (!s_ap_ready)
        return 1;

    struct iwl_scd_queue_cfg_cmd qc = {0};
    qc.operation    = IWL_SCD_QUEUE_REMOVE;
    qc.add.sta_mask = (1u << IWL_AP_STA_ID);
    qc.add.tid      = tid;

    term_puts("iwl: ");
    term_puts(why);
    term_puts(": SCD_QUEUE_CONFIG REMOVE sta_mask=0x");
    prh32(qc.add.sta_mask);
    term_puts(" tid=");
    prh8(tid);
    term_puts(" ...\n");

    int r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_SCD_QUEUE_CONFIG,
                         &qc, sizeof(qc), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: SCD_QUEUE_CONFIG AP REMOVE failed\n");
        iwl_dump_fw_error();
        return 0;
    }

    s_ap_ready = 0;
    s_aptxq_id = 0;
    s_aptxq_tid = 0;
    s_aptxq_size = 0;
    s_aptxq_rd_ptr = 0;
    s_aptxq_wr_ptr = 0;
    term_puts("iwl: AP TX queue removed\n");
    return 1;
}

int iwl_cmd_txq_setup_probe(void) {
    if (!s_scan_setup_ready) {
        term_puts("iwl: TX queue setup needs scan setup; running wifisetup\n");
        if (!iwl_cmd_scan_setup_probe()) return 0;
    }
    if (s_txq_ready) {
        term_puts("iwl: TX queue already configured (q="); prh16(s_txq_id);
        term_puts(")\n");
        return 1;
    }

    /* Allocate TX TFD ring (16 × 256 = 4 KiB) and the byte-count table
     * (2048 bytes for gen2 iwlagn_scd_bc_tbl).  Keep the buffers across
     * scan iterations; only the firmware queue is removed after each scan. */
    if (!s_txq_tfd_ring) {
        s_txq_tfd_ring = (struct iwl_tfh_tfd *)
            dma_alloc_aligned(sizeof(struct iwl_tfh_tfd) * IWL_MGMT_QUEUE_SIZE,
                              4096, &s_txq_tfd_phys);
        if (!s_txq_tfd_ring) { term_puts("iwl: txq tfd alloc FAIL\n"); return 0; }
    }

    if (!s_txq_bc_tbl) {
        s_txq_bc_tbl = (uint8_t *)
            dma_alloc_aligned(IWL_BC_TBL_SIZE * 2u, 4096, &s_txq_bc_tbl_phys);
        if (!s_txq_bc_tbl) { term_puts("iwl: txq bc_tbl alloc FAIL\n"); return 0; }
    }

    /* Mark all TX TFD slots invalid (zeroed); same convention as cmd queue. */
    for (uint32_t i = 0; i < IWL_MGMT_QUEUE_SIZE; i++)
        for (uint64_t b = 0; b < sizeof(struct iwl_tfh_tfd); b++)
            ((uint8_t *)&s_txq_tfd_ring[i])[b] = 0;
    for (uint64_t i = 0; i < IWL_BC_TBL_SIZE * 2u; i++)
        s_txq_bc_tbl[i] = 0;

    /* ── AUX_STA_CMD for AUX STA ───────────────────────────────────────────
     * The MLD path uses group-3 AUX_STA_CMD, not legacy ADD_STA.  Once scan
     * setup creates MLD MAC/LINK contexts, legacy ADD_STA asserts here. */
    int r;
    if (!s_aux_sta_ready) {
        struct iwl_aux_sta_cmd aux = {0};
        aux.sta_id = IWL_AUX_STA_ID;
        aux.lmac_id = 0;          /* single-LMAC AX201 */

        term_puts("iwl: txq: AUX_STA sta_id="); prh8(IWL_AUX_STA_ID);
        term_puts(" lmac=0 ...\n");
        r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_AUX_STA,
                         &aux, sizeof(aux), (void *)0, 0);
        if (r < 0) {
            term_puts("iwl: AUX_STA failed\n");
            return 0;
        }
        s_aux_sta_ready = 1;
    }

    /* ── SCD_QUEUE_CONFIG_CMD v3 (gen3 path) ───────────────────────────────
     *   operation = IWL_SCD_QUEUE_ADD (0)
     *   sta_mask  = BIT(IWL_AUX_STA_ID) — references the AUX STA
     *   tid       = 8 (IWL_MAX_TID_COUNT for internal/AUX STA)
     *   flags     = 0 (NO ENABLE bit on v3 — that's only legacy v0)
     *   cb_size   = 1 (log2(16)-3) for 16-entry queue
     *   bc_dram_addr / tfdq_dram_addr = our DMA-coherent allocations */
    struct iwl_scd_queue_cfg_cmd cmd = {0};
    cmd.operation             = IWL_SCD_QUEUE_ADD;
    cmd.add.sta_mask          = (1u << IWL_AUX_STA_ID);
    cmd.add.tid               = IWL_INTERNAL_QUEUE_TID;
    cmd.add.flags             = 0;
    cmd.add.cb_size           = IWL_MGMT_QUEUE_CB_SIZE;
    cmd.add.bc_dram_addr      = s_txq_bc_tbl_phys;
    cmd.add.tfdq_dram_addr    = s_txq_tfd_phys;

    term_puts("iwl: txq: SCD_QUEUE_CONFIG ADD sta_mask=0x");
    prh32(cmd.add.sta_mask); term_puts(" tid=8 cb=1 ...\n");
    term_puts("       tfd=0x");    prh64(s_txq_tfd_phys);
    term_puts("\n       bc_tbl=0x"); prh64(s_txq_bc_tbl_phys);
    term_putchar('\n');

    struct iwl_tx_queue_cfg_rsp rsp = {0};
    r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_SCD_QUEUE_CONFIG,
                     &cmd, sizeof(cmd), &rsp, sizeof(rsp));
    if (r < 0) {
        term_puts("iwl: SCD_QUEUE_CONFIG failed\n");
        return 0;
    }

    s_txq_id        = rsp.queue_number;
    s_txq_write_ptr = rsp.write_pointer;
    term_puts("       queue_id=");   prh16(s_txq_id);
    term_puts(" write_ptr=");        prh16(s_txq_write_ptr);
    term_puts(" flags=");            prh16(rsp.flags);
    term_putchar('\n');

    s_txq_ready = 1;
    term_puts("iwl: TX queue OK\n");
    return 1;
}

static int iwl_cmd_txq_teardown_probe(void) {
    if (!s_txq_ready)
        return 1;

    struct iwl_scd_queue_cfg_cmd cmd = {0};
    cmd.operation    = IWL_SCD_QUEUE_REMOVE;
    cmd.add.sta_mask = (1u << IWL_AUX_STA_ID);
    cmd.add.tid      = IWL_INTERNAL_QUEUE_TID;

    term_puts("iwl: txq: SCD_QUEUE_CONFIG REMOVE sta_mask=0x");
    prh32(cmd.add.sta_mask); term_puts(" tid=8 ...\n");
    int r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_SCD_QUEUE_CONFIG,
                         &cmd, sizeof(cmd), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: SCD_QUEUE_CONFIG REMOVE failed\n");
        iwl_dump_fw_error();
        return 0;
    }

    s_txq_ready = 0;
    s_txq_id = 0;
    s_txq_write_ptr = 0;
    term_puts("iwl: TX queue removed\n");
    return 1;
}

/* ── Association path: AUTH + ASSOC management TX/RX ───────────────────── */
/*
 * Linux flow (mvm/mac80211.c::iwl_mvm_mac_sta_state + mvm/sta.c +
 * mvm/binding.c):
 *   1. PHY_CONTEXT_CMD MODIFY  — retune to AP's channel.
 *   2. MAC_CONTEXT_CMD MODIFY  — set bssid_addr to AP, keep BSS_STA, unassoc.
 *   3. BINDING_CONTEXT_CMD ADD — bind MAC ctx 0 to PHY ctx 0 (single LMAC).
 *   4. ADD_STA for the AP STA — sta_id=0, addr=BSSID, station_type=LINK.
 *   5. SCD_QUEUE_CONFIG_CMD   — allocate per-AP TX queue, sta_mask=BIT(0).
 *   6. Build + TX AUTH (alg=0 Open, seq=1, status=0) on that queue.
 *   7. RX TX_CMD response (cmd=0x1c group=0) confirms the frame went out;
 *      RX REPLY_RX_MPDU_CMD (cmd=0xc1) carrying AUTH (alg=0 seq=2) is the
 *      AP's reply.  Status=0 in that reply = open auth accepted.
 *   8. TX Association Request and wait for Association Response status/AID.
 *
 * Most of the sequence is host-side glue.  The bit that previously asserted
 * SW_ERR (host TX through the AUX queue) is replaced here with TX through
 * a queue bound to a real STA on a bound MAC/PHY context — which IS the
 * sanctioned path for the firmware.
 */

/* iwl_dram_sec_info (Linux fw/api/tx.h) — encryption PN/IV.  Zero for
 * unencrypted mgmt frames. */
struct iwl_dram_sec_info {
    uint32_t pn_low;
    uint16_t pn_high;
    uint16_t aux_info;
} __attribute__((packed));      /* 8 bytes */

/* iwl_tx_cmd_v9 (Linux fw/api/tx.h, TX_CMD_API_S_VER_9 — pre-AX210).
 * 20 bytes header before the 802.11 frame.  Used by family 22000 (AX201). */
struct iwl_tx_cmd_v9 {
    uint16_t len;                   /* 802.11 frame length (header + body) */
    uint16_t offload_assist;
    uint32_t flags;                 /* IWL_TX_FLAGS_* — 32-bit in v9 */
    struct iwl_dram_sec_info dram_info;
    uint32_t rate_n_flags;
    /* struct ieee80211_hdr hdr[]; */
} __attribute__((packed));      /* 20 bytes */

static int tx_status_success(uint16_t status) {
    uint16_t code = (uint16_t)(status & IWL_TX_STATUS_MSK);
    return code == IWL_TX_STATUS_SUCCESS || code == IWL_TX_STATUS_DIRECT_DONE;
}

static uint32_t auth_ofdm_6m_rate_n_flags(uint8_t fw_rates_ver) {
    uint32_t low_bits = (fw_rates_ver == 1) ?
        IWL_RATE_LEGACY_OFDM_6M_PLCP : IWL_RATE_LEGACY_OFDM_6M_IDX;
    uint32_t rate = low_bits | IWL_RATE_MCS_ANT_A_MSK;

    if (fw_rates_ver >= 2)
        rate |= IWL_RATE_MCS_MOD_TYPE_LEGACY_OFDM;
    return rate;
}

static int print_tx_cmd_response(const struct iwl_rx_packet *pkt, uint32_t plen,
                                 int *out_tx_ok, uint16_t *out_status) {
    const uint8_t *p = pkt->data;
    uint32_t payload_len = (plen >= 4u) ? (plen - 4u) : plen;

    if (out_tx_ok) *out_tx_ok = 0;
    if (out_status) *out_status = 0xFFFFu;

    term_puts("    RX TX_CMD resp len="); prh32(plen);
    term_puts(" payload="); prh32(payload_len);
    if (payload_len < 44u) {
        term_puts(" short\n");
        return 0;
    }

    uint8_t frame_count = p[0];
    uint8_t failure_rts = p[2];
    uint8_t failure_frame = p[3];
    uint32_t initial_rate = rd32(p + 4);
    uint16_t wireless_media_time = rd16(p + 8);
    uint32_t tfd_info = rd32(p + 24);
    uint16_t seq_ctl = rd16(p + 28);
    uint16_t byte_cnt = rd16(p + 30);
    uint8_t tlc_info = p[32];
    uint8_t ra_tid = p[33];
    uint16_t frame_ctrl = rd16(p + 34);
    uint16_t tx_queue = rd16(p + 36);
    uint16_t status = rd16(p + 40);
    uint16_t status_seq = rd16(p + 42);
    int ok = tx_status_success(status);

    if (out_tx_ok) *out_tx_ok = ok;
    if (out_status) *out_status = status;

    term_puts(" cnt="); prh8(frame_count);
    term_puts(" fail_rts="); prh8(failure_rts);
    term_puts(" fail_frame="); prh8(failure_frame);
    term_puts(" status=0x"); prh16(status);
    term_puts(" ok="); prh8((uint8_t)ok);
    term_putchar('\n');

    term_puts("       rate=0x"); prh32(initial_rate);
    term_puts(" media="); prh16(wireless_media_time);
    term_puts(" tfd=0x"); prh32(tfd_info);
    term_puts(" seq_ctl="); prh16(seq_ctl);
    term_puts(" byte_cnt="); prh16(byte_cnt);
    term_puts(" tlc=0x"); prh8(tlc_info);
    term_puts(" ra_tid=0x"); prh8(ra_tid);
    term_puts(" fc=0x"); prh16(frame_ctrl);
    term_puts(" txq="); prh16(tx_queue);
    term_puts(" stseq="); prh16(status_seq);
    if (payload_len >= 48u) {
        term_puts(" ssn="); prh16((uint16_t)(rd32(p + 44) & 0x0FFFu));
    }
    term_putchar('\n');
    ap_txq_note_tx_status(pkt, plen);
    return 1;
}

/* Look up a cached AP by SSID (case-sensitive, exact length match). */
static const struct iwl_ap_info *find_ap_by_ssid(const char *ssid,
                                                  uint8_t ssid_len) {
    for (int i = 0; i < iwl_ap_count(); i++) {
        const struct iwl_ap_info *a = iwl_ap_get(i);
        if (a->ssid_len != ssid_len) continue;
        int eq = 1;
        for (uint8_t j = 0; j < ssid_len; j++)
            if (a->ssid[j] != (uint8_t)ssid[j]) { eq = 0; break; }
        if (eq) return a;
    }
    return (const struct iwl_ap_info *)0;
}

static void ap_basic_rate_masks(const struct iwl_ap_info *ap,
                                uint32_t *cck_rates,
                                uint32_t *ofdm_rates) {
    uint32_t cck = 0, ofdm = 0;
    for (uint8_t i = 0; i < ap->rates_len; i++) {
        uint8_t r = ap->rates[i];
        if (!(r & 0x80u)) continue;   /* not in BSSBasicRateSet */
        switch (r & 0x7fu) {
        case 2:   cck  |= (1u << 0); break; /* 1 Mbps */
        case 4:   cck  |= (1u << 1); break; /* 2 Mbps */
        case 11:  cck  |= (1u << 2); break; /* 5.5 Mbps */
        case 22:  cck  |= (1u << 3); break; /* 11 Mbps */
        case 12:  ofdm |= (1u << 0); break; /* 6 Mbps */
        case 18:  ofdm |= (1u << 1); break; /* 9 Mbps */
        case 24:  ofdm |= (1u << 2); break; /* 12 Mbps */
        case 36:  ofdm |= (1u << 3); break; /* 18 Mbps */
        case 48:  ofdm |= (1u << 4); break; /* 24 Mbps */
        case 72:  ofdm |= (1u << 5); break; /* 36 Mbps */
        case 96:  ofdm |= (1u << 6); break; /* 48 Mbps */
        case 108: ofdm |= (1u << 7); break; /* 54 Mbps */
        default: break;
        }
    }

    if (!cck && !ofdm) {
        cck = 0xFu;
        ofdm = 0x01u;
    }

    if (cck) {
        if (cck & (1u << 3)) cck |= (1u << 2);
        if (cck & (1u << 2)) cck |= (1u << 1);
        if (cck & (1u << 1)) cck |= (1u << 0);
    }
    if (ofdm) {
        if (ofdm & (0xFFu & ~0x0Fu)) ofdm |= (1u << 4);
        if (ofdm & (0xFFu & ~0x03u)) ofdm |= (1u << 2);
        ofdm |= (1u << 0);
    }

    *cck_rates = cck;
    *ofdm_rates = ofdm;
}

static int append_ie(uint8_t *frame, uint16_t *pos, uint16_t max_len,
                     uint8_t id, const uint8_t *data, uint8_t len) {
    if ((uint32_t)*pos + 2u + len > max_len) return 0;
    frame[(*pos)++] = id;
    frame[(*pos)++] = len;
    for (uint8_t i = 0; i < len; i++) frame[(*pos)++] = data[i];
    return 1;
}

static int ap_supports_rate(const struct iwl_ap_info *ap, uint8_t rate_500k) {
    if (!ap->rates_len) return 1;
    for (uint8_t i = 0; i < ap->rates_len; i++)
        if ((ap->rates[i] & 0x7Fu) == rate_500k) return 1;
    return 0;
}

static int append_assoc_rate_ies(const struct iwl_ap_info *ap, uint8_t *frame,
                                 uint16_t *pos, uint16_t max_len) {
    static const uint8_t hw_rates_24[] = {
        2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108
    };
    uint8_t rates[sizeof(hw_rates_24)];
    uint8_t n = 0;

    for (uint8_t i = 0; i < sizeof(hw_rates_24); i++) {
        if (ap_supports_rate(ap, hw_rates_24[i]))
            rates[n++] = hw_rates_24[i];
    }
    if (!n) return 0;

    uint8_t first = n > 8 ? 8 : n;
    if (!append_ie(frame, pos, max_len, WLAN_EID_SUPP_RATES, rates, first))
        return 0;
    if (n > first &&
        !append_ie(frame, pos, max_len, WLAN_EID_EXT_SUPP_RATES,
                   rates + first, (uint8_t)(n - first)))
        return 0;
    return 1;
}

static int rsn_suite_is(const uint8_t *suite, uint8_t type) {
    return suite[0] == 0x00 && suite[1] == 0x0F &&
           suite[2] == 0xAC && suite[3] == type;
}

static int build_sta_rsn_ie(const struct iwl_ap_info *ap,
                            uint8_t *out, uint8_t *out_len) {
    const uint8_t *ie = ap->rsn_ie;
    if (ap->rsn_ie_len < 12 || ie[0] != WLAN_EID_RSN ||
        (uint16_t)ie[1] + 2u != ap->rsn_ie_len)
        return 0;

    uint16_t total = ap->rsn_ie_len;
    uint16_t p = 2;
    if (p + 2u + 4u + 2u > total) return 0;
    uint8_t ver0 = ie[p++];
    uint8_t ver1 = ie[p++];
    const uint8_t *group = ie + p; p += 4;

    uint16_t pair_count = rd16(ie + p); p += 2;
    if (!pair_count || p + pair_count * 4u > total) return 0;
    const uint8_t *pair = ie + p;
    const uint8_t *sel_pair = pair;
    for (uint16_t i = 0; i < pair_count; i++) {
        const uint8_t *suite = pair + i * 4u;
        if (rsn_suite_is(suite, 4)) { sel_pair = suite; break; } /* CCMP-128 */
    }
    p = (uint16_t)(p + pair_count * 4u);

    if (p + 2u > total) return 0;
    uint16_t akm_count = rd16(ie + p); p += 2;
    if (!akm_count || p + akm_count * 4u > total) return 0;
    const uint8_t *akm = ie + p;
    const uint8_t *sel_akm = akm;
    for (uint16_t i = 0; i < akm_count; i++) {
        const uint8_t *suite = akm + i * 4u;
        if (rsn_suite_is(suite, 2)) { sel_akm = suite; break; } /* PSK */
        if (rsn_suite_is(suite, 8)) sel_akm = suite;            /* SAE fallback */
    }
    p = (uint16_t)(p + akm_count * 4u);

    uint16_t sta_caps = 0;
    if (p + 2u <= total) {
        uint16_t ap_caps = rd16(ie + p);
        if (ap_caps & (1u << 6)) sta_caps |= (1u << 7); /* AP requires PMF */
    }

    uint8_t q = 0;
    out[q++] = WLAN_EID_RSN;
    out[q++] = 0;                 /* filled below */
    out[q++] = ver0; out[q++] = ver1;
    for (uint8_t i = 0; i < 4; i++) out[q++] = group[i];
    out[q++] = 1; out[q++] = 0;   /* one pairwise cipher */
    for (uint8_t i = 0; i < 4; i++) out[q++] = sel_pair[i];
    out[q++] = 1; out[q++] = 0;   /* one AKM */
    for (uint8_t i = 0; i < 4; i++) out[q++] = sel_akm[i];
    out[q++] = (uint8_t)(sta_caps & 0xFFu);
    out[q++] = (uint8_t)(sta_caps >> 8);
    out[1] = (uint8_t)(q - 2);
    *out_len = q;
    return 1;
}

static uint8_t tx_tlc_chains_from_nvm(void) {
    uint32_t ant = s_nvm_tx_chains & 0x3u;
    return (uint8_t)(ant ? ant : 1u);
}

static uint32_t rxchain_info_from_nvm(void) {
    uint32_t ant = s_nvm_rx_chains & 0xFu;
    if (!ant) ant = 1u;
    return IWL_RXCHAIN_VAL_BIT(ant) |
           IWL_RXCHAIN_IDLE(1) |
           IWL_RXCHAIN_ACTIVE(1);
}

static int mac_eq6(const uint8_t a[6], const uint8_t b[6]) {
    for (int i = 0; i < 6; i++)
        if (a[i] != b[i]) return 0;
    return 1;
}

static int mac_valid_unicast(const uint8_t mac[6]) {
    int all_zero = 1, all_ff = 1;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0x00u) all_zero = 0;
        if (mac[i] != 0xFFu) all_ff = 0;
    }
    return !all_zero && !all_ff && !(mac[0] & 0x01u);
}

static const uint8_t *rx_mpdu_frame(const struct iwl_rx_packet *pkt,
                                    uint32_t plen, uint16_t *out_len) {
    if (plen < 4u + 48u) return (const uint8_t *)0;

    const struct iwl_rx_mpdu_desc *d =
        (const struct iwl_rx_mpdu_desc *)pkt->data;
    uint16_t frame_len = d->mpdu_len;
    if (4u + 48u + frame_len > plen)
        frame_len = (uint16_t)(plen - 4u - 48u);

    if (out_len) *out_len = frame_len;
    return pkt->data + 48;
}

/* Build and TX a fully-formed 802.11 frame on the AP-STA queue. */
static int aptxq_tx_frame_flags(const uint8_t *frame_buf, uint16_t frame_len,
                                uint32_t rate_n_flags, uint32_t flags,
                                uint16_t mac_hdr_len) {
    if (!s_ap_ready || !s_aptxq_cmd_buf || !s_aptxq_size ||
        frame_len < mac_hdr_len ||
        frame_len > (uint16_t)(IWL_AP_TX_CMD_STRIDE - 24u))
        return 0;
    if (ap_txq_used() >= (uint8_t)(s_aptxq_size - 1u)) {
        term_puts("iwl: ap-txq full rd=");
        prh16(s_aptxq_rd_ptr);
        term_puts(" wr=");
        prh16(s_aptxq_wr_ptr);
        term_puts(" size=");
        prh16(s_aptxq_size);
        term_putchar('\n');
        return 0;
    }
    uint8_t  slot = ap_txq_index(s_aptxq_wr_ptr);
    uint8_t *cb   = s_aptxq_cmd_buf + (uint64_t)slot * IWL_AP_TX_CMD_STRIDE;

    /* Compose: short cmd_header (4B) + iwl_tx_cmd_v9 (20B) + 802.11 frame. */
    struct iwl_cmd_header_short *hdr = (struct iwl_cmd_header_short *)cb;
    hdr->cmd      = IWL_CMD_TX_CMD;          /* 0x1c */
    hdr->group_id = IWL_GROUP_LEGACY;
    /* sequence: bits[12:8]=qid (5b), bits[7:0]=slot (Linux QUEUE_TO_SEQ). */
    hdr->sequence = (uint16_t)(((s_aptxq_id & 0x1Fu) << 8) | (slot & 0xFFu));

    struct iwl_tx_cmd_v9 *txc = (struct iwl_tx_cmd_v9 *)(cb + 4);
    txc->len            = frame_len;
    txc->offload_assist = (uint16_t)((mac_hdr_len / 2u) << IWL_TX_CMD_OFFLD_MH_SIZE);
    if (mac_hdr_len & 3u) txc->offload_assist |= (uint16_t)(1u << IWL_TX_CMD_OFFLD_PAD);
    txc->flags          = flags;
    txc->dram_info.pn_low   = 0;
    txc->dram_info.pn_high  = 0;
    txc->dram_info.aux_info = 0;
    txc->rate_n_flags   = rate_n_flags;

    /* Copy the 802.11 frame into cb[24..]. */
    for (uint16_t i = 0; i < frame_len; i++) cb[24 + i] = frame_buf[i];
    uint16_t seq_ctl = 0;
    if (frame_len >= 24u && (frame_buf[0] & 0x0Cu) != 0x04u) {
        seq_ctl = (uint16_t)((s_aptxq_seq++ & 0x0FFFu) << 4);
        cb[24 + 22] = (uint8_t)seq_ctl;
        cb[24 + 23] = (uint8_t)(seq_ctl >> 8);
    }

    /* 3-TB layout (Linux iwl_txq_gen2_build_tx, no AMSDU):
     *   TB[0] = 20 B from first_tb_buf scratch (hdr+first 16 of tx_cmd_v9)
     *   TB[1] = (4 + 20 + 24) - 20 = 28 B at cb+20  (rate_n_flags + 802.11 hdr)
     *   TB[2] = (frame_len - 24) at cb+48          (802.11 body)              */
    const uint16_t tx_cmd_size = (uint16_t)sizeof(struct iwl_tx_cmd_v9);
    const uint16_t cmd_hdr_size = 4;
    uint16_t tb1_len = (uint16_t)(tx_cmd_size + cmd_hdr_size + mac_hdr_len - 20);
    uint16_t tb2_len = (uint16_t)(frame_len - mac_hdr_len);

    uint8_t *ftb = s_aptxq_first_tb + (uint64_t)slot * IWL_FIRST_TB_STRIDE;
    for (uint8_t i = 0; i < IWL_FIRST_TB_STRIDE; i++) ftb[i] = 0;
    for (int i = 0; i < 20; i++) ftb[i] = cb[i];

    struct iwl_tfh_tfd *tfd = &s_aptxq_tfd_ring[slot];
    for (uint64_t i = 0; i < sizeof(*tfd); i++) ((uint8_t *)tfd)[i] = 0;

    tfd->tbs[0].tb_len = 20;
    tb_set_addr(&tfd->tbs[0],
                s_aptxq_first_tb_phys + (uint64_t)slot * IWL_FIRST_TB_STRIDE);
    tfd->tbs[1].tb_len = tb1_len;
    tb_set_addr(&tfd->tbs[1],
                s_aptxq_cmd_buf_phys + (uint64_t)slot * IWL_AP_TX_CMD_STRIDE + 20u);
    tfd->tbs[2].tb_len = tb2_len;
    tb_set_addr(&tfd->tbs[2],
                s_aptxq_cmd_buf_phys + (uint64_t)slot * IWL_AP_TX_CMD_STRIDE + 20u + tb1_len);
    tfd->num_tbs = 3;

    /* bc_tbl entry (gen2 / pre-AX210):
     *   tfd_offset[idx] = (skb_len_dwords | (num_fetch_chunks << 12)) LE.
     * num_fetch_chunks = DIV_ROUND_UP(2 + num_tbs*10, 64) - 1; for 3 TBs
     * that's (32/64=0.5→1)−1 = 0, so just len_dwords. */
    uint16_t len_dwords = (uint16_t)((frame_len + 3u) / 4u);
    ((uint16_t *)s_aptxq_bc_tbl)[slot] = len_dwords;

    s_aptxq_wr_ptr = (uint16_t)((s_aptxq_wr_ptr + 1u) & 0xFFu);
    uint32_t doorbell = txq_doorbell(s_aptxq_id, s_aptxq_wr_ptr);
    __sync_synchronize();
    creg_w32(HBUS_TARG_WRPTR, doorbell);

    term_puts("       tx: qid="); prh16(s_aptxq_id);
    term_puts(" tid=");            prh8(s_aptxq_tid);
    term_puts(" slot=");           prh8(slot);
    term_puts(" len=");            prh16(frame_len);
    term_puts(" tb1=");            prh16(tb1_len);
    term_puts(" tb2=");            prh16(tb2_len);
    term_puts(" seqctl=");         prh16(seq_ctl);
    term_puts(" used=");           prh8(ap_txq_used());
    term_puts(" flags=0x");        prh32(txc->flags);
    term_puts(" rate=0x");         prh32(txc->rate_n_flags);
    term_puts(" doorbell=0x");     prh32(doorbell);
    term_putchar('\n');
    return 1;
}

/* Pre-auth management/EAPOL frames need an explicit basic OFDM rate and must
 * stay plaintext even after keys are installed. */
static int aptxq_tx_mgmt_frame(const uint8_t *frame_buf, uint16_t frame_len,
                               uint32_t rate_n_flags) {
    return aptxq_tx_frame_flags(frame_buf, frame_len, rate_n_flags,
                                IWL_TX_FLAGS_CMD_RATE |
                                IWL_TX_FLAGS_ENCRYPT_DIS |
                                IWL_TX_FLAGS_HIGH_PRI,
                                24);
}

static int rx_mpdu_to_eth(const struct iwl_rx_packet *pkt, uint32_t plen,
                          void *buf, uint16_t maxlen, uint16_t *len_out) {
    if (plen < 4u + sizeof(struct iwl_rx_mpdu_desc)) return 0;
    const struct iwl_rx_mpdu_desc *d =
        (const struct iwl_rx_mpdu_desc *)pkt->data;
    uint16_t frame_len = d->mpdu_len;
    if (4u + 48u + frame_len > plen)
        frame_len = (uint16_t)(plen - 4u - 48u);
    const uint8_t *fp = pkt->data + 48;
    if (frame_len < 24u || (fp[0] & 0x0Cu) != 0x08u)
        return 0;

    uint8_t fc0 = fp[0], fc1 = fp[1];
    int to_ds = fc1 & 0x01u;
    int from_ds = fc1 & 0x02u;
    if (!from_ds || to_ds)
        return 0;

    uint8_t my_mac[6];
    iwl_mac(my_mac);
    if (!mac_eq6(fp + 4, my_mac) && !(fp[4] & 0x01u))
        return 0;
    if (!mac_eq6(fp + 10, s_ap_bssid))
        return 0;

    uint16_t hdr_len = 24;
    if (fc0 & 0x80u) hdr_len = (uint16_t)(hdr_len + 2u); /* QoS control */
    if (fc1 & 0x80u) hdr_len = (uint16_t)(hdr_len + 4u); /* HT control */

    if (fc1 & 0x40u) {
        uint32_t sec = d->status & IWL_RX_MPDU_STATUS_SEC_MASK;
        if (sec != IWL_RX_MPDU_STATUS_SEC_CCM ||
            !(d->status & IWL_RX_MPDU_STATUS_MIC_OK))
            return 0;
        hdr_len = (uint16_t)(hdr_len + 8u); /* CCMP header; MIC stripped by FW */
        if (d->mac_flags2 & IWL_RX_MPDU_MFLG2_PAD)
            hdr_len = (uint16_t)(hdr_len + 2u);
    }

    if (frame_len < hdr_len + 8u)
        return 0;
    const uint8_t *llc = fp + hdr_len;
    if (llc[0] != 0xAA || llc[1] != 0xAA || llc[2] != 0x03 ||
        llc[3] != 0x00 || llc[4] != 0x00 || llc[5] != 0x00)
        return 0;

    uint16_t payload_len = (uint16_t)(frame_len - hdr_len - 8u);
    uint16_t eth_len = (uint16_t)(14u + payload_len);
    if (eth_len > maxlen)
        return 0;

    uint8_t *eth = (uint8_t *)buf;
    for (uint8_t i = 0; i < 6; i++) eth[i] = fp[4 + i];       /* DA */
    for (uint8_t i = 0; i < 6; i++) eth[6 + i] = fp[16 + i];  /* SA */
    eth[12] = llc[6];
    eth[13] = llc[7];
    for (uint16_t i = 0; i < payload_len; i++)
        eth[14u + i] = llc[8u + i];
    if (len_out) *len_out = eth_len;
    return 1;
}

int iwl_cmd_net_link_up(void) {
    return s_net_ready;
}

int iwl_cmd_net_send(const void *data, uint16_t len) {
    if (!s_net_ready || len < 14u || len > 1514u)
        return -1;

    const uint8_t *eth = (const uint8_t *)data;
    uint8_t my_mac[6];
    iwl_mac(my_mac);

    uint8_t fr[1536];
    for (uint16_t i = 0; i < sizeof(fr); i++) fr[i] = 0;
    fr[0] = 0x08;                              /* Data */
    fr[1] = 0x41;                              /* ToDS + Protected */
    for (int i = 0; i < 6; i++) fr[4  + i] = s_ap_bssid[i]; /* RA/BSSID */
    for (int i = 0; i < 6; i++) fr[10 + i] = my_mac[i];     /* TA/SA */
    for (int i = 0; i < 6; i++) fr[16 + i] = eth[i];        /* DA */

    uint16_t pos = 24;
    fr[pos++] = 0xAA; fr[pos++] = 0xAA; fr[pos++] = 0x03;
    fr[pos++] = 0x00; fr[pos++] = 0x00; fr[pos++] = 0x00;
    fr[pos++] = eth[12]; fr[pos++] = eth[13];
    for (uint16_t i = 14; i < len; i++) fr[pos++] = eth[i];

    if (!aptxq_tx_frame_flags(fr, pos, s_ap_rate_n_flags,
                              IWL_TX_FLAGS_CMD_RATE, 24))
        return -1;
    return 0;
}

int iwl_cmd_net_recv(void *buf, uint16_t maxlen, uint16_t *len_out) {
    if (!s_net_ready || !buf || !len_out)
        return -1;

    uint32_t *stts32 = iwl_rx_stts_raw();
    if (!stts32) return -1;
    struct iwl_rb_status *rs = (struct iwl_rb_status *)stts32;
    uint16_t cur = rx_closed_index(rs);
    while (s_rx_read != cur) {
        uint16_t vid = 0;
        struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
        if (pkt) {
            uint32_t plen = pkt->len_n_flags & 0x3FFFu;
            if (pkt->hdr.cmd == IWL_CMD_REPLY_RX_MPDU &&
                rx_mpdu_to_eth(pkt, plen, buf, maxlen, len_out)) {
                rx_consume_read(vid);
                return 0;
            }
            note_rx_packet(pkt, plen);
        }
        rx_consume_read(vid);
    }
    return -1;
}

static uint16_t build_assoc_req_frame(const struct iwl_ap_info *ap,
                                      const uint8_t my_mac[6],
                                      uint8_t *fr, uint16_t max_len) {
    if (max_len < 64u || ap->ssid_len > 32u) return 0;
    for (uint16_t i = 0; i < max_len; i++) fr[i] = 0;

    fr[0] = 0x00; fr[1] = 0x00;                 /* FC: mgmt, Association Req */
    fr[2] = 0x00; fr[3] = 0x00;                 /* Duration */
    for (int i = 0; i < 6; i++) fr[4  + i] = ap->bssid[i];  /* DA = AP */
    for (int i = 0; i < 6; i++) fr[10 + i] = my_mac[i];     /* SA = us */
    for (int i = 0; i < 6; i++) fr[16 + i] = ap->bssid[i];  /* BSSID = AP */
    fr[22] = 0x00; fr[23] = 0x00;               /* SeqCtl */

    uint16_t cap = WLAN_CAPABILITY_ESS |
                   WLAN_CAPABILITY_SHORT_PREAMBLE |
                   WLAN_CAPABILITY_SHORT_SLOT_TIME;
    if (ap->capability & WLAN_CAPABILITY_PRIVACY)
        cap |= WLAN_CAPABILITY_PRIVACY;
    fr[24] = (uint8_t)(cap & 0xFFu);
    fr[25] = (uint8_t)(cap >> 8);
    fr[26] = 10; fr[27] = 0;                    /* listen interval */

    uint16_t pos = 28;
    if (!append_ie(fr, &pos, max_len, WLAN_EID_SSID, ap->ssid, ap->ssid_len))
        return 0;
    if (!append_assoc_rate_ies(ap, fr, &pos, max_len))
        return 0;

    if (ap->capability & WLAN_CAPABILITY_PRIVACY) {
        uint8_t rsn[64];
        uint8_t rsn_len = 0;
        if (build_sta_rsn_ie(ap, rsn, &rsn_len) &&
            (uint32_t)pos + rsn_len <= max_len) {
            for (uint8_t i = 0; i < rsn_len; i++) fr[pos++] = rsn[i];
            term_puts("iwl: assoc: ASSOC_REQ includes RSN len=");
            prh8(rsn_len);
            term_putchar('\n');
        } else {
            term_puts("iwl: assoc: privacy AP but no usable RSN IE\n");
        }
    }

    return pos;
}

struct sha1_ctx {
    uint32_t h[5];
    uint64_t bits;
    uint8_t  buf[64];
    uint8_t  len;
};

static uint32_t rol32(uint32_t v, uint8_t n) {
    return (v << n) | (v >> (32u - n));
}

static uint32_t be32_rd(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t be16_rd(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void be16_wr(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

static void sha1_init(struct sha1_ctx *c) {
    c->h[0] = 0x67452301u;
    c->h[1] = 0xEFCDAB89u;
    c->h[2] = 0x98BADCFEu;
    c->h[3] = 0x10325476u;
    c->h[4] = 0xC3D2E1F0u;
    c->bits = 0;
    c->len = 0;
}

static void sha1_transform(struct sha1_ctx *c, const uint8_t block[64]) {
    uint32_t w[80];
    for (uint8_t i = 0; i < 16; i++)
        w[i] = be32_rd(block + (uint16_t)i * 4u);
    for (uint8_t i = 16; i < 80; i++)
        w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

    uint32_t a = c->h[0], b = c->h[1], d = c->h[3], e = c->h[4];
    uint32_t cc = c->h[2];
    for (uint8_t i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20) {
            f = (b & cc) | ((~b) & d);
            k = 0x5A827999u;
        } else if (i < 40) {
            f = b ^ cc ^ d;
            k = 0x6ED9EBA1u;
        } else if (i < 60) {
            f = (b & cc) | (b & d) | (cc & d);
            k = 0x8F1BBCDCu;
        } else {
            f = b ^ cc ^ d;
            k = 0xCA62C1D6u;
        }
        uint32_t temp = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = cc;
        cc = rol32(b, 30);
        b = a;
        a = temp;
    }

    c->h[0] += a;
    c->h[1] += b;
    c->h[2] += cc;
    c->h[3] += d;
    c->h[4] += e;
}

static void sha1_update(struct sha1_ctx *c, const uint8_t *data, uint32_t len) {
    c->bits += (uint64_t)len * 8u;
    while (len) {
        uint8_t room = (uint8_t)(64u - c->len);
        uint8_t n = (len < room) ? (uint8_t)len : room;
        for (uint8_t i = 0; i < n; i++) c->buf[c->len + i] = data[i];
        c->len = (uint8_t)(c->len + n);
        data += n;
        len -= n;
        if (c->len == 64) {
            sha1_transform(c, c->buf);
            c->len = 0;
        }
    }
}

static void sha1_final(struct sha1_ctx *c, uint8_t out[20]) {
    uint64_t bits = c->bits;
    uint8_t one = 0x80;
    uint8_t zero = 0;
    sha1_update(c, &one, 1);
    while (c->len != 56)
        sha1_update(c, &zero, 1);
    for (int i = 7; i >= 0; i--) {
        uint8_t b = (uint8_t)(bits >> ((uint8_t)i * 8u));
        sha1_update(c, &b, 1);
    }
    for (uint8_t i = 0; i < 5; i++) {
        out[i * 4u + 0] = (uint8_t)(c->h[i] >> 24);
        out[i * 4u + 1] = (uint8_t)(c->h[i] >> 16);
        out[i * 4u + 2] = (uint8_t)(c->h[i] >> 8);
        out[i * 4u + 3] = (uint8_t)c->h[i];
    }
}

static void hmac_sha1(const uint8_t *key, uint16_t key_len,
                      const uint8_t *data, uint16_t data_len,
                      uint8_t out[20]) {
    uint8_t k0[64];
    for (uint8_t i = 0; i < 64; i++) k0[i] = 0;
    if (key_len > 64) {
        struct sha1_ctx hc;
        sha1_init(&hc);
        sha1_update(&hc, key, key_len);
        sha1_final(&hc, k0);
    } else {
        for (uint16_t i = 0; i < key_len; i++) k0[i] = key[i];
    }

    uint8_t ipad[64], opad[64];
    for (uint8_t i = 0; i < 64; i++) {
        ipad[i] = (uint8_t)(k0[i] ^ 0x36u);
        opad[i] = (uint8_t)(k0[i] ^ 0x5Cu);
    }

    uint8_t inner[20];
    struct sha1_ctx c;
    sha1_init(&c);
    sha1_update(&c, ipad, 64);
    sha1_update(&c, data, data_len);
    sha1_final(&c, inner);

    sha1_init(&c);
    sha1_update(&c, opad, 64);
    sha1_update(&c, inner, 20);
    sha1_final(&c, out);
}

static void pbkdf2_sha1(const uint8_t *pass, uint8_t pass_len,
                        const uint8_t *ssid, uint8_t ssid_len,
                        uint8_t out[32]) {
    uint8_t salt[36];
    for (uint8_t i = 0; i < ssid_len; i++) salt[i] = ssid[i];

    uint8_t done = 0;
    for (uint8_t block = 1; done < 32; block++) {
        salt[ssid_len + 0] = 0;
        salt[ssid_len + 1] = 0;
        salt[ssid_len + 2] = 0;
        salt[ssid_len + 3] = block;

        uint8_t u[20], t[20];
        hmac_sha1(pass, pass_len, salt, (uint16_t)(ssid_len + 4u), u);
        for (uint8_t i = 0; i < 20; i++) t[i] = u[i];
        for (uint16_t iter = 1; iter < 4096; iter++) {
            hmac_sha1(pass, pass_len, u, 20, u);
            for (uint8_t i = 0; i < 20; i++) t[i] ^= u[i];
        }
        for (uint8_t i = 0; i < 20 && done < 32; i++)
            out[done++] = t[i];
    }
}

static int lex_less_32(const uint8_t *a, const uint8_t *b, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        if (a[i] < b[i]) return 1;
        if (a[i] > b[i]) return 0;
    }
    return 0;
}

static uint8_t cstr_len8(const char *s) {
    uint8_t n = 0;
    while (s[n]) n++;
    return n;
}

static void wpa_prf_ptk(const uint8_t pmk[32],
                        const uint8_t aa[6], const uint8_t spa[6],
                        const uint8_t anonce[32], const uint8_t snonce[32],
                        uint8_t ptk[48]) {
    uint8_t data[76];
    uint16_t p = 0;
    const uint8_t *mac1 = lex_less_32(aa, spa, 6) ? aa : spa;
    const uint8_t *mac2 = (mac1 == aa) ? spa : aa;
    const uint8_t *nonce1 = lex_less_32(anonce, snonce, 32) ? anonce : snonce;
    const uint8_t *nonce2 = (nonce1 == anonce) ? snonce : anonce;
    for (uint8_t i = 0; i < 6; i++) data[p++] = mac1[i];
    for (uint8_t i = 0; i < 6; i++) data[p++] = mac2[i];
    for (uint8_t i = 0; i < 32; i++) data[p++] = nonce1[i];
    for (uint8_t i = 0; i < 32; i++) data[p++] = nonce2[i];

    const char *label = "Pairwise key expansion";
    uint8_t label_len = cstr_len8(label);
    uint8_t buf[128];
    uint8_t out = 0;
    for (uint8_t counter = 0; out < 48; counter++) {
        uint16_t q = 0;
        for (uint8_t i = 0; i < label_len; i++) buf[q++] = (uint8_t)label[i];
        buf[q++] = 0;
        for (uint8_t i = 0; i < sizeof(data); i++) buf[q++] = data[i];
        buf[q++] = counter;

        uint8_t hash[20];
        hmac_sha1(pmk, 32, buf, q, hash);
        for (uint8_t i = 0; i < 20 && out < 48; i++)
            ptk[out++] = hash[i];
    }
}

static const uint8_t aes_sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static uint8_t aes_inv_sbox[256];
static int aes_inv_sbox_ready;

static void aes_ensure_inv_sbox(void) {
    if (aes_inv_sbox_ready) return;
    for (uint16_t i = 0; i < 256; i++)
        aes_inv_sbox[aes_sbox[i]] = (uint8_t)i;
    aes_inv_sbox_ready = 1;
}

static uint8_t aes_xtime(uint8_t x) {
    return (uint8_t)((x << 1) ^ ((x & 0x80u) ? 0x1Bu : 0));
}

static uint8_t aes_mul(uint8_t a, uint8_t b) {
    uint8_t r = 0;
    while (b) {
        if (b & 1u) r ^= a;
        a = aes_xtime(a);
        b >>= 1;
    }
    return r;
}

static void aes128_expand_key(const uint8_t key[16], uint8_t rk[176]) {
    for (uint8_t i = 0; i < 16; i++) rk[i] = key[i];
    uint8_t rcon = 1;
    uint16_t bytes = 16;
    while (bytes < 176) {
        uint8_t t[4];
        for (uint8_t i = 0; i < 4; i++) t[i] = rk[bytes - 4u + i];
        if ((bytes & 15u) == 0) {
            uint8_t x = t[0];
            t[0] = (uint8_t)(aes_sbox[t[1]] ^ rcon);
            t[1] = aes_sbox[t[2]];
            t[2] = aes_sbox[t[3]];
            t[3] = aes_sbox[x];
            rcon = aes_xtime(rcon);
        }
        for (uint8_t i = 0; i < 4; i++) {
            rk[bytes] = (uint8_t)(rk[bytes - 16u] ^ t[i]);
            bytes++;
        }
    }
}

static void aes_add_round_key(uint8_t s[16], const uint8_t *rk) {
    for (uint8_t i = 0; i < 16; i++) s[i] ^= rk[i];
}

static void aes_inv_shift_rows(uint8_t s[16]) {
    uint8_t t;
    t = s[13]; s[13] = s[9]; s[9] = s[5]; s[5] = s[1]; s[1] = t;
    t = s[2]; s[2] = s[10]; s[10] = t;
    t = s[6]; s[6] = s[14]; s[14] = t;
    t = s[3]; s[3] = s[7]; s[7] = s[11]; s[11] = s[15]; s[15] = t;
}

static void aes_inv_sub_bytes(uint8_t s[16]) {
    aes_ensure_inv_sbox();
    for (uint8_t i = 0; i < 16; i++) s[i] = aes_inv_sbox[s[i]];
}

static void aes_inv_mix_columns(uint8_t s[16]) {
    for (uint8_t c = 0; c < 4; c++) {
        uint8_t *p = s + (uint16_t)c * 4u;
        uint8_t a0 = p[0], a1 = p[1], a2 = p[2], a3 = p[3];
        p[0] = (uint8_t)(aes_mul(a0, 0x0e) ^ aes_mul(a1, 0x0b) ^
                         aes_mul(a2, 0x0d) ^ aes_mul(a3, 0x09));
        p[1] = (uint8_t)(aes_mul(a0, 0x09) ^ aes_mul(a1, 0x0e) ^
                         aes_mul(a2, 0x0b) ^ aes_mul(a3, 0x0d));
        p[2] = (uint8_t)(aes_mul(a0, 0x0d) ^ aes_mul(a1, 0x09) ^
                         aes_mul(a2, 0x0e) ^ aes_mul(a3, 0x0b));
        p[3] = (uint8_t)(aes_mul(a0, 0x0b) ^ aes_mul(a1, 0x0d) ^
                         aes_mul(a2, 0x09) ^ aes_mul(a3, 0x0e));
    }
}

static void aes128_decrypt_block(const uint8_t key[16],
                                 const uint8_t in[16],
                                 uint8_t out[16]) {
    uint8_t rk[176];
    uint8_t s[16];
    aes128_expand_key(key, rk);
    for (uint8_t i = 0; i < 16; i++) s[i] = in[i];

    aes_add_round_key(s, rk + 160);
    for (int round = 9; round >= 1; round--) {
        aes_inv_shift_rows(s);
        aes_inv_sub_bytes(s);
        aes_add_round_key(s, rk + (uint16_t)round * 16u);
        aes_inv_mix_columns(s);
    }
    aes_inv_shift_rows(s);
    aes_inv_sub_bytes(s);
    aes_add_round_key(s, rk);

    for (uint8_t i = 0; i < 16; i++) out[i] = s[i];
}

static int aes_key_unwrap(const uint8_t kek[16], const uint8_t *cipher,
                          uint16_t cipher_len, uint8_t *plain,
                          uint16_t *plain_len) {
    if (cipher_len < 16u || (cipher_len & 7u)) return 0;
    uint8_t n = (uint8_t)(cipher_len / 8u - 1u);
    if (!n || n > 31u) return 0;

    uint8_t a[8];
    uint8_t r[31][8];
    for (uint8_t i = 0; i < 8; i++) a[i] = cipher[i];
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < 8; j++)
            r[i][j] = cipher[8u + (uint16_t)i * 8u + j];

    for (int j = 5; j >= 0; j--) {
        for (int i = n; i >= 1; i--) {
            uint64_t t = (uint64_t)n * (uint64_t)j + (uint64_t)i;
            uint8_t block[16], out[16];
            for (uint8_t k = 0; k < 8; k++) block[k] = a[k];
            for (int k = 7; k >= 0 && t; k--) {
                block[k] ^= (uint8_t)t;
                t >>= 8;
            }
            for (uint8_t k = 0; k < 8; k++) block[8u + k] = r[i - 1][k];
            aes128_decrypt_block(kek, block, out);
            for (uint8_t k = 0; k < 8; k++) a[k] = out[k];
            for (uint8_t k = 0; k < 8; k++) r[i - 1][k] = out[8u + k];
        }
    }

    for (uint8_t i = 0; i < 8; i++)
        if (a[i] != 0xA6u) return 0;
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < 8; j++)
            plain[(uint16_t)i * 8u + j] = r[i][j];
    *plain_len = (uint16_t)n * 8u;
    return 1;
}

static uint64_t rdtsc64(void) {
    uint32_t lo, hi;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
}

static int cpu_has_rdrand(void) {
    static int known, supported;
    if (!known) {
        uint32_t eax = 1, ebx, ecx, edx;
        __asm__ __volatile__("cpuid"
                             : "+a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
                             : "c"(0));
        supported = (ecx & (1u << 30)) ? 1 : 0;
        known = 1;
    }
    return supported;
}

static int rdrand32(uint32_t *out) {
    if (!cpu_has_rdrand()) {
        *out = 0;
        return 0;
    }
    uint8_t ok;
    uint32_t v;
    __asm__ __volatile__("rdrand %0; setc %1" : "=r"(v), "=qm"(ok));
    *out = v;
    return ok != 0;
}

static uint32_t s_wpa_nonce_seq;

static void wpa_make_snonce(uint8_t snonce[32]) {
    uint64_t t = rdtsc64();
    for (uint8_t i = 0; i < 8; i++) {
        uint32_t v;
        if (!rdrand32(&v)) {
            t ^= (uint64_t)creg_r32(CSR_INT) << 17;
            t ^= (uint64_t)creg_r32(CSR_FH_INT_STATUS) << 33;
            t ^= ++s_wpa_nonce_seq;
            t ^= t << 13;
            t ^= t >> 7;
            t ^= t << 17;
            v = (uint32_t)(t ^ (t >> 32));
        }
        snonce[i * 4u + 0] = (uint8_t)v;
        snonce[i * 4u + 1] = (uint8_t)(v >> 8);
        snonce[i * 4u + 2] = (uint8_t)(v >> 16);
        snonce[i * 4u + 3] = (uint8_t)(v >> 24);
    }
}

struct wpa_eapol_key_rx {
    uint8_t  eapol_ver;
    uint8_t  desc_type;
    uint8_t  desc_ver;
    uint16_t key_info;
    uint16_t key_len;
    uint16_t key_data_len;
    uint16_t eapol_total_len;
    uint8_t  replay[8];
    uint8_t  nonce[32];
    uint8_t  mic[16];
    uint8_t  eapol[256];
};

static const uint8_t *data_llc_payload(const uint8_t *fp, uint16_t frame_len,
                                       const uint8_t my_mac[6]) {
    if (frame_len < 24 || (fp[0] & 0x0Cu) != 0x08u)
        return (const uint8_t *)0;
    if (fp[1] & 0x40u)   /* Protected bit: not plaintext EAPOL. */
        return (const uint8_t *)0;
    if (!mac_eq6(fp + 4, my_mac) || !mac_eq6(fp + 10, s_ap_bssid))
        return (const uint8_t *)0;

    uint16_t hdr_len = 24;
    if ((fp[1] & 0x03u) == 0x03u) hdr_len = (uint16_t)(hdr_len + 6u);
    if (fp[0] & 0x80u) hdr_len = (uint16_t)(hdr_len + 2u);
    if (frame_len < hdr_len + 8u)
        return (const uint8_t *)0;

    const uint8_t *llc = fp + hdr_len;
    if (llc[0] != 0xAA || llc[1] != 0xAA || llc[2] != 0x03 ||
        llc[3] != 0x00 || llc[4] != 0x00 || llc[5] != 0x00 ||
        llc[6] != 0x88 || llc[7] != 0x8E)
        return (const uint8_t *)0;
    return llc + 8;
}

static int parse_eapol_key(const uint8_t *fp, uint16_t frame_len,
                           const uint8_t my_mac[6],
                           struct wpa_eapol_key_rx *out) {
    const uint8_t *e = data_llc_payload(fp, frame_len, my_mac);
    if (!e) return 0;

    uint16_t hdr_len = (uint16_t)(e - fp);
    if (frame_len < hdr_len + 99u || e[1] != 3)
        return 0;
    uint16_t eapol_len = be16_rd(e + 2);
    if (eapol_len < 95u || frame_len < hdr_len + 4u + eapol_len)
        return 0;
    uint16_t eapol_total_len = (uint16_t)(4u + eapol_len);
    if (eapol_total_len > sizeof(out->eapol))
        return 0;

    struct wpa_eapol_key_rx k;
    k.eapol_ver = e[0];
    k.desc_type = e[4];
    k.key_info = be16_rd(e + 5);
    k.desc_ver = (uint8_t)(k.key_info & 0x7u);
    k.key_len = be16_rd(e + 7);
    for (uint8_t i = 0; i < 8; i++) k.replay[i] = e[9 + i];
    for (uint8_t i = 0; i < 32; i++) k.nonce[i] = e[17 + i];
    for (uint8_t i = 0; i < 16; i++) k.mic[i] = e[81 + i];
    k.key_data_len = be16_rd(e + 97);
    k.eapol_total_len = eapol_total_len;
    for (uint16_t i = 0; i < eapol_total_len; i++) k.eapol[i] = e[i];
    if (out) *out = k;
    return 1;
}

static void print_eapol_key(const struct wpa_eapol_key_rx *k) {
    uint16_t info = k->key_info;
    term_puts("    RX EAPOL-Key info=0x"); prh16(info);
    term_puts(" desc="); prh8(k->desc_type);
    term_puts(" ver="); prh8(k->desc_ver);
    term_puts(" klen="); prh16(k->key_len);
    term_puts(" kdlen="); prh16(k->key_data_len);
    term_puts(" msg=");
    if ((info & (1u << 7)) && !(info & (1u << 8)))
        term_puts("M1");
    else if ((info & (1u << 7)) && (info & (1u << 8)))
        term_puts("M3");
    else if (!(info & (1u << 7)) && (info & (1u << 8)))
        term_puts("STA");
    else
        term_putchar('?');
    term_putchar('\n');
}

static int wpa_key_mic_ok(const struct wpa_eapol_key_rx *k) {
    if (!s_wpa.valid || k->eapol_total_len < 99u) {
        term_puts("iwl: wpa: no PTK available for EAPOL MIC check\n");
        return 0;
    }

    uint8_t buf[256];
    for (uint16_t i = 0; i < k->eapol_total_len; i++) buf[i] = k->eapol[i];
    for (uint8_t i = 0; i < 16; i++) buf[81 + i] = 0;

    uint8_t mic[20];
    hmac_sha1(s_wpa.ptk, 16, buf, k->eapol_total_len, mic);
    for (uint8_t i = 0; i < 16; i++) {
        if (mic[i] != k->mic[i]) {
            term_puts("iwl: wpa: EAPOL M3 MIC mismatch\n");
            return 0;
        }
    }
    term_puts("iwl: wpa: EAPOL M3 MIC OK\n");
    return 1;
}

static int iwl_install_pairwise_ccmp_key(void) {
    if (!s_wpa.valid) {
        term_puts("iwl: wpa: cannot install PTK before derivation\n");
        return 0;
    }

    struct iwl_sec_key_cmd cmd = {0};
    cmd.action = FW_CTXT_ACTION_ADD;
    cmd.u.add.sta_mask = (1u << IWL_AP_STA_ID);
    cmd.u.add.key_id = 0;
    cmd.u.add.key_flags = IWL_SEC_KEY_FLAG_CIPHER_CCMP;
    for (uint8_t i = 0; i < 16; i++) cmd.u.add.key[i] = s_wpa.ptk[32 + i];

    term_puts("iwl: wpa: SEC_KEY ADD pairwise CCMP sta_mask=0x");
    prh32(cmd.u.add.sta_mask);
    term_puts(" ...\n");
    int r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_SEC_KEY,
                         &cmd, sizeof(cmd), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: wpa: SEC_KEY pairwise install failed\n");
        iwl_dump_fw_error();
        return 0;
    }
    return 1;
}

static int iwl_install_gtk_ccmp_key(uint8_t key_id, const uint8_t gtk[16]) {
    struct iwl_sec_key_cmd cmd = {0};
    cmd.action = FW_CTXT_ACTION_ADD;
    cmd.u.add.sta_mask = (1u << IWL_AP_STA_ID);
    cmd.u.add.key_id = key_id;
    cmd.u.add.key_flags = IWL_SEC_KEY_FLAG_CIPHER_CCMP |
                          IWL_SEC_KEY_FLAG_MCAST_KEY;
    for (uint8_t i = 0; i < 16; i++) cmd.u.add.key[i] = gtk[i];

    term_puts("iwl: wpa: SEC_KEY ADD GTK CCMP key_id=");
    prh8(key_id);
    term_puts(" sta_mask=0x");
    prh32(cmd.u.add.sta_mask);
    term_puts(" ...\n");
    int r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_SEC_KEY,
                         &cmd, sizeof(cmd), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: wpa: SEC_KEY GTK install failed\n");
        iwl_dump_fw_error();
        return 0;
    }
    return 1;
}

static int wpa_install_gtk_from_m3(const struct wpa_eapol_key_rx *m3) {
    if (!s_wpa.valid) return 0;
    if (!(m3->key_info & (1u << 12))) {
        term_puts("iwl: wpa: M3 key data is not encrypted\n");
        return 0;
    }
    if (m3->eapol_total_len < 99u ||
        m3->key_data_len > (uint16_t)(m3->eapol_total_len - 99u)) {
        term_puts("iwl: wpa: malformed M3 key data length\n");
        return 0;
    }

    uint8_t plain[248];
    uint16_t plain_len = 0;
    const uint8_t *cipher = m3->eapol + 99;
    if (!aes_key_unwrap(s_wpa.ptk + 16, cipher, m3->key_data_len,
                        plain, &plain_len)) {
        term_puts("iwl: wpa: AES unwrap of M3 key data failed\n");
        return 0;
    }
    term_puts("iwl: wpa: M3 key data unwrapped len=");
    prh16(plain_len);
    term_putchar('\n');

    uint16_t p = 0;
    while (p + 2u <= plain_len) {
        uint8_t id = plain[p];
        uint8_t len = plain[p + 1u];
        if (id == 0 && len == 0)
            break;
        if (p + 2u + len > plain_len) {
            int only_pad = 1;
            for (uint16_t i = p; i < plain_len; i++)
                if (plain[i] != 0 && plain[i] != 0xDDu) only_pad = 0;
            if (only_pad) break;
            term_puts("iwl: wpa: malformed KDE in M3 key data\n");
            return 0;
        }

        if (id == WLAN_EID_VENDOR_SPECIFIC && len >= 22u &&
            plain[p + 2u] == 0x00 && plain[p + 3u] == 0x0F &&
            plain[p + 4u] == 0xAC && plain[p + 5u] == 0x01) {
            const uint8_t *kde = plain + p + 6u;
            uint8_t kde_data_len = (uint8_t)(len - 4u);
            if (kde_data_len < 18u) {
                term_puts("iwl: wpa: short GTK KDE\n");
                return 0;
            }
            uint8_t key_id = (uint8_t)(kde[0] & 0x03u);
            term_puts("iwl: wpa: GTK KDE key_id=");
            prh8(key_id);
            term_puts(" gtk_len=");
            prh8((uint8_t)(kde_data_len - 2u));
            term_putchar('\n');
            if (kde_data_len - 2u != 16u) {
                term_puts("iwl: wpa: only CCMP-128 GTK length supported\n");
                return 0;
            }
            return iwl_install_gtk_ccmp_key(key_id, kde + 2u);
        }
        p = (uint16_t)(p + 2u + len);
    }

    term_puts("iwl: wpa: no GTK KDE found in M3 key data\n");
    return 0;
}

static int wpa_send_m4(const struct iwl_ap_info *ap, const uint8_t my_mac[6],
                       const struct wpa_eapol_key_rx *m3,
                       uint32_t rate_n_flags) {
    if (!s_wpa.valid) return 0;

    uint8_t fr[160];
    for (uint16_t i = 0; i < sizeof(fr); i++) fr[i] = 0;
    fr[0] = 0x08; fr[1] = 0x01;                 /* Data, ToDS */
    for (int i = 0; i < 6; i++) fr[4  + i] = ap->bssid[i];  /* RA/BSSID */
    for (int i = 0; i < 6; i++) fr[10 + i] = my_mac[i];     /* TA/SA */
    for (int i = 0; i < 6; i++) fr[16 + i] = ap->bssid[i];  /* DA */

    uint16_t pos = 24;
    static const uint8_t llc_eapol[8] = {
        0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E
    };
    for (uint8_t i = 0; i < sizeof(llc_eapol); i++) fr[pos++] = llc_eapol[i];

    uint8_t *e = fr + pos;
    uint16_t eapol_len = 95;
    e[0] = m3->eapol_ver ? m3->eapol_ver : 2;
    e[1] = 3;                         /* EAPOL-Key */
    be16_wr(e + 2, eapol_len);
    e[4] = m3->desc_type;
    uint16_t key_info = (uint16_t)((m3->desc_ver & 0x7u) |
                                   (1u << 3) |       /* pairwise */
                                   (1u << 8) |       /* MIC */
                                   (1u << 9));       /* secure */
    be16_wr(e + 5, key_info);
    be16_wr(e + 7, m3->key_len ? m3->key_len : 16);
    for (uint8_t i = 0; i < 8; i++) e[9 + i] = m3->replay[i];
    be16_wr(e + 97, 0);

    uint8_t mic[20];
    hmac_sha1(s_wpa.ptk, 16, e, (uint16_t)(4u + eapol_len), mic);
    for (uint8_t i = 0; i < 16; i++) e[81 + i] = mic[i];

    uint16_t frame_len = (uint16_t)(pos + 4u + eapol_len);
    term_puts("iwl: wpa: TX EAPOL M4 len=");
    prh16(frame_len);
    term_putchar('\n');
    return aptxq_tx_mgmt_frame(fr, frame_len, rate_n_flags);
}

static int wpa_send_m2(const struct iwl_ap_info *ap, const uint8_t my_mac[6],
                       const char *pass, uint8_t pass_len,
                       const struct wpa_eapol_key_rx *m1,
                       uint32_t rate_n_flags) {
    if (!pass || !pass_len) return 0;
    if (pass_len < 8) {
        term_puts("iwl: wpa: passphrase too short for WPA2-PSK\n");
        return 0;
    }
    if (m1->desc_ver != 2) {
        term_puts("iwl: wpa: only HMAC-SHA1 EAPOL-Key descriptor v2 supported\n");
        return 0;
    }

    uint8_t rsn[64], rsn_len = 0;
    if (!build_sta_rsn_ie(ap, rsn, &rsn_len)) {
        term_puts("iwl: wpa: cannot build supplicant RSN IE for M2\n");
        return 0;
    }
    if (rsn_len < 20u || !rsn_suite_is(rsn + 10, 4)) {
        term_puts("iwl: wpa: AP pairwise cipher is not CCMP-128; M2 not supported\n");
        return 0;
    }
    if (rsn_len < 20u || !rsn_suite_is(rsn + 16, 2)) {
        term_puts("iwl: wpa: AP AKM is not WPA2-PSK; M2 not supported\n");
        return 0;
    }

    uint8_t pmk[32], ptk[48], snonce[32];
    term_puts("iwl: wpa: deriving PSK PMK (PBKDF2-SHA1) ...\n");
    pbkdf2_sha1((const uint8_t *)pass, pass_len, ap->ssid, ap->ssid_len, pmk);
    wpa_make_snonce(snonce);
    wpa_prf_ptk(pmk, ap->bssid, my_mac, m1->nonce, snonce, ptk);
    for (uint8_t i = 0; i < sizeof(s_wpa.ptk); i++) s_wpa.ptk[i] = ptk[i];
    for (uint8_t i = 0; i < sizeof(s_wpa.snonce); i++) s_wpa.snonce[i] = snonce[i];
    for (uint8_t i = 0; i < sizeof(s_wpa.anonce); i++) s_wpa.anonce[i] = m1->nonce[i];
    s_wpa.valid = 1;

    uint8_t fr[256];
    for (uint16_t i = 0; i < sizeof(fr); i++) fr[i] = 0;
    fr[0] = 0x08; fr[1] = 0x01;                 /* Data, ToDS */
    for (int i = 0; i < 6; i++) fr[4  + i] = ap->bssid[i];  /* RA/BSSID */
    for (int i = 0; i < 6; i++) fr[10 + i] = my_mac[i];     /* TA/SA */
    for (int i = 0; i < 6; i++) fr[16 + i] = ap->bssid[i];  /* DA */

    uint16_t pos = 24;
    static const uint8_t llc_eapol[8] = {
        0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E
    };
    for (uint8_t i = 0; i < sizeof(llc_eapol); i++) fr[pos++] = llc_eapol[i];

    uint8_t *e = fr + pos;
    uint16_t eapol_len = (uint16_t)(95u + rsn_len);
    e[0] = m1->eapol_ver ? m1->eapol_ver : 2;
    e[1] = 3;                         /* EAPOL-Key */
    be16_wr(e + 2, eapol_len);
    e[4] = m1->desc_type;
    uint16_t key_info = (uint16_t)((m1->desc_ver & 0x7u) |
                                   (1u << 3) |       /* pairwise */
                                   (1u << 8));       /* MIC */
    be16_wr(e + 5, key_info);
    be16_wr(e + 7, m1->key_len ? m1->key_len : 16);
    for (uint8_t i = 0; i < 8; i++) e[9 + i] = m1->replay[i];
    for (uint8_t i = 0; i < 32; i++) e[17 + i] = snonce[i];
    be16_wr(e + 97, rsn_len);
    for (uint8_t i = 0; i < rsn_len; i++) e[99 + i] = rsn[i];

    uint8_t mic[20];
    hmac_sha1(ptk, 16, e, (uint16_t)(4u + eapol_len), mic);
    for (uint8_t i = 0; i < 16; i++) e[81 + i] = mic[i];

    uint16_t frame_len = (uint16_t)(pos + 4u + eapol_len);
    term_puts("iwl: wpa: TX EAPOL M2 len=");
    prh16(frame_len);
    term_puts(" rsn=");
    prh8(rsn_len);
    term_putchar('\n');
    return aptxq_tx_mgmt_frame(fr, frame_len, rate_n_flags);
}

/* Linux-derived MLD association sequence used after the Surface 3 AX201 NVM
 * init path.  Scan still uses the known-working legacy BSS_STA MAC context,
 * but Linux's live reconnect trace shows AUTH is sent only after group-3
 * LINK_CONFIG/MAC_CONFIG/STA_CONFIG plus SESSION_PROTECTION are in place.
 *
 *   1. PHY_CONTEXT_CMD MODIFY → retune to the AP's channel
 *   2. RLC_CONFIG_CMD         → mirror Linux's PHY follow-up
 *   3. LINK_CONFIG_CMD        → add/activate link 0 on MAC 0 / PHY 0
 *   4. MAC_CONFIG_CMD         → keep BSS_STA MAC unassociated
 *   5. STA_CONFIG_CMD         → register AP STA (sta_id=0)
 *   6. SESSION_PROTECTION_CMD → association protection window
 *   7. SCD_QUEUE_CONFIG_CMD   → allocate TX queue bound to AP STA
 *   8. TX_CMD AUTH frame      → host TX through the AP queue
 *   9. TX_CMD ASSOC_REQ frame → wait for AP association status/AID
 */
int iwl_cmd_assoc_probe(const char *ssid, uint8_t ssid_len,
                        const char *pass, uint8_t pass_len) {
    s_wpa.valid = 0;
    s_net_ready = 0;
    if (s_txq_ready) {
        term_puts("iwl: assoc: removing scan AUX TX queue before AP queue\n");
        if (!iwl_cmd_txq_teardown_probe()) return 0;
    }

    /* ── 1. Look up the target AP in the scan cache. ─────────────────────── */
    const struct iwl_ap_info *ap = find_ap_by_ssid(ssid, ssid_len);
    if (!ap) {
        term_puts("iwl: assoc: SSID not in scan cache; run wifiscan first\n");
        return 0;
    }
    for (int i = 0; i < 6; i++) s_ap_bssid[i] = ap->bssid[i];
    s_ap_channel = ap->channel;
    term_puts("iwl: assoc: target ");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(ap->bssid[i]);
    }
    term_puts(" ch="); prh8(ap->channel);
    term_puts(" cap=0x"); prh16(ap->capability);
    term_putchar('\n');

    int r;
    uint8_t my_mac[6]; iwl_mac(my_mac);
    term_puts("iwl: assoc: local ");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(my_mac[i]);
    }
    term_putchar('\n');
    if (!mac_valid_unicast(my_mac)) {
        term_puts("iwl: assoc: invalid local MAC; cannot send AUTH\n");
        return 0;
    }
    const uint32_t assoc_rx_filter =
        MAC_CFG_FILTER_ACCEPT_GRP |
        MAC_CFG_FILTER_ACCEPT_BEACON |
        MAC_CFG_FILTER_ACCEPT_CONTROL_AND_MGMT;

    /* ── 2. PHY_CONTEXT_CMD MODIFY → AP's channel. ──────────────────────── */
    struct iwl_phy_context_cmd_v3 phy = {0};
    phy.id_and_color = IWL_FW_ID_AND_COLOR(0, 1);
    phy.action       = FW_CTXT_ACTION_MODIFY;
    phy.channel      = s_ap_channel;
    phy.band         = IWL_PHY_BAND_24;
    phy.width        = IWL_PHY_CHANNEL_MODE20;
    phy.ctrl_pos     = 0;
    phy.lmac_id      = 0;
    phy.rxchain_info = rxchain_info_from_nvm();
    phy.dsp_cfg_flags = 0;
    term_puts("iwl: assoc: PHY_CONTEXT MODIFY ch="); prh8(s_ap_channel);
    term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_LONG, IWL_CMD_PHY_CONTEXT,
                     &phy, sizeof(phy), (void *)0, 0);
    if (r < 0) { term_puts("iwl: PHY_CONTEXT MODIFY failed\n"); iwl_dump_fw_error(); return 0; }

    /* Linux 6.17 sends RLC_CONFIG after every PHY_CONTEXT update on this
     * firmware (RLC_CONFIG v2).  Keep the old rxchain field in PHY for scan
     * compatibility, but send the Linux-matching command before activating
     * the MLD link used for AUTH. */
    struct iwl_rlc_config_cmd rlc = {0};
    rlc.phy_id = 0;
    rlc.rx_chain_info = rxchain_info_from_nvm();
    term_puts("iwl: assoc: RLC_CONFIG phy=0 rxchain=0x");
    prh32(rlc.rx_chain_info); term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_RLC_CONFIG,
                     &rlc, sizeof(rlc), (void *)0, 0);
    if (r < 0) { term_puts("iwl: RLC_CONFIG failed\n"); iwl_dump_fw_error(); return 0; }

    /* ── 3. MLD MAC/LINK context: scan setup normally created both. ──────── */
    if (!s_mld_mac_added) {
        struct iwl_mac_config_cmd mac_add = {0};
        mac_add.id_and_color = 0;
        mac_add.action       = FW_CTXT_ACTION_ADD;
        mac_add.mac_type     = FW_MAC_TYPE_BSS_STA;
        for (int i = 0; i < 6; i++) mac_add.local_mld_addr[i] = my_mac[i];
        mac_add.filter_flags = assoc_rx_filter;
        mac_add.client.is_assoc = 0;
        term_puts("iwl: assoc: MAC_CONFIG ADD id=0 type=BSS_STA rxmgmt ...\n");
        r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_MAC_CONFIG,
                         &mac_add, sizeof(mac_add), (void *)0, 0);
        if (r < 0) { term_puts("iwl: MAC_CONFIG ADD failed\n"); iwl_dump_fw_error(); return 0; }
        s_mld_mac_added = 1;
    }

    /* Add an inactive link if this path was reached without scan setup doing
     * it already, then activate it for the AP's channel below. */
    struct iwl_link_config_cmd link = {0};
    if (!s_mld_link_added) {
        link.action  = FW_CTXT_ACTION_ADD;
        link.link_id = 0;
        link.mac_id  = 0;
        link.phy_id  = IWL_FW_CTXT_INVALID;
        for (int i = 0; i < 6; i++) link.local_link_addr[i] = my_mac[i];
        link.spec_link_id = 0;
        term_puts("iwl: assoc: LINK_CONFIG ADD link=0 mac=0 ...\n");
        r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_LINK_CONFIG,
                         &link, sizeof(link), (void *)0, 0);
        if (r < 0) { term_puts("iwl: LINK_CONFIG ADD failed\n"); iwl_dump_fw_error(); return 0; }
        s_mld_link_added = 1;
    }

    uint32_t cck_rates = 0, ofdm_rates = 0;
    ap_basic_rate_masks(ap, &cck_rates, &ofdm_rates);

    /* phy_id/local_link_addr are only accepted while the link remains
     * inactive.  Linux therefore sends one MODIFY to attach the PHY and set
     * rates, then a second MODIFY that only flips ACTIVE. */
    for (uint64_t i = 0; i < sizeof(link); i++) ((uint8_t *)&link)[i] = 0;
    link.action      = FW_CTXT_ACTION_MODIFY;
    link.link_id     = 0;
    link.mac_id      = 0;
    link.phy_id      = 0;
    for (int i = 0; i < 6; i++) link.local_link_addr[i] = my_mac[i];
    link.modify_mask = LINK_CONTEXT_MODIFY_RATES_INFO |
                       LINK_CONTEXT_MODIFY_PROTECT_FLAGS |
                       LINK_CONTEXT_MODIFY_QOS_PARAMS |
                       LINK_CONTEXT_MODIFY_BEACON_TIMING;
    link.active      = 0;
    link.cck_rates   = cck_rates;
    link.ofdm_rates  = ofdm_rates;
    link.cck_short_preamble =
        (ap->capability & WLAN_CAPABILITY_SHORT_PREAMBLE) ? 1u : 0u;
    link.short_slot =
        (ap->capability & WLAN_CAPABILITY_SHORT_SLOT_TIME) ? 1u : 0u;
    link.bi = ap->beacon_interval ? ap->beacon_interval : 100;
    link.dtim_interval = link.bi;
    link.spec_link_id = 0;
    term_puts("iwl: assoc: LINK_CONFIG MODIFY attach phy=0 rates cck=0x");
    prh32(cck_rates); term_puts(" ofdm=0x"); prh32(ofdm_rates);
    term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_LINK_CONFIG,
                     &link, sizeof(link), (void *)0, 0);
    if (r < 0) { term_puts("iwl: LINK_CONFIG attach failed\n"); iwl_dump_fw_error(); return 0; }

    for (uint64_t i = 0; i < sizeof(link); i++) ((uint8_t *)&link)[i] = 0;
    link.action      = FW_CTXT_ACTION_MODIFY;
    link.link_id     = 0;
    link.mac_id      = 0;
    link.phy_id      = 0;
    for (int i = 0; i < 6; i++) link.local_link_addr[i] = my_mac[i];
    link.modify_mask = LINK_CONTEXT_MODIFY_ACTIVE |
                       LINK_CONTEXT_MODIFY_RATES_INFO |
                       LINK_CONTEXT_MODIFY_PROTECT_FLAGS |
                       LINK_CONTEXT_MODIFY_QOS_PARAMS |
                       LINK_CONTEXT_MODIFY_BEACON_TIMING;
    link.active      = 1;
    link.cck_rates   = cck_rates;
    link.ofdm_rates  = ofdm_rates;
    link.cck_short_preamble =
        (ap->capability & WLAN_CAPABILITY_SHORT_PREAMBLE) ? 1u : 0u;
    link.short_slot =
        (ap->capability & WLAN_CAPABILITY_SHORT_SLOT_TIME) ? 1u : 0u;
    link.bi = ap->beacon_interval ? ap->beacon_interval : 100;
    link.dtim_interval = link.bi;
    link.spec_link_id = 0;
    term_puts("iwl: assoc: LINK_CONFIG MODIFY active=1 mask=0x");
    prh32(link.modify_mask); term_puts(" rates cck=0x"); prh32(cck_rates);
    term_puts(" ofdm=0x"); prh32(ofdm_rates); term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_LINK_CONFIG,
                     &link, sizeof(link), (void *)0, 0);
    if (r < 0) { term_puts("iwl: LINK_CONFIG active failed\n"); iwl_dump_fw_error(); return 0; }

    struct iwl_mac_power_cmd pm = {0};
    pm.id_and_color = 0;
    pm.flags = 1;                 /* Linux logs flags=0x1 with PM disabled. */
    pm.keep_alive_seconds = 25;
    term_puts("iwl: assoc: MAC_PM_POWER_TABLE mac=0 keepalive=25 ...\n");
    r = iwl_send_cmd(IWL_GROUP_LONG, IWL_CMD_MAC_PM_POWER_TABLE,
                     &pm, sizeof(pm), (void *)0, 0);
    if (r < 0) { term_puts("iwl: MAC_PM_POWER_TABLE failed\n"); iwl_dump_fw_error(); return 0; }

    /* ── 4. MLD MAC_CONFIG MODIFY: station MAC remains unassociated. ─────── */
    struct iwl_mac_config_cmd mac = {0};
    mac.id_and_color = 0;
    mac.action       = FW_CTXT_ACTION_MODIFY;
    mac.mac_type     = FW_MAC_TYPE_BSS_STA;
    for (int i = 0; i < 6; i++) mac.local_mld_addr[i] = my_mac[i];
    mac.filter_flags = assoc_rx_filter;
    mac.client.is_assoc = 0;
    term_puts("iwl: assoc: MAC_CONFIG MODIFY unassoc rxmgmt ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_MAC_CONFIG,
                     &mac, sizeof(mac), (void *)0, 0);
    if (r < 0) { term_puts("iwl: MAC_CONFIG MODIFY failed\n"); iwl_dump_fw_error(); return 0; }

    /* ── 5. MLD STA_CONFIG for the AP peer. ─────────────────────────────── */
    struct iwl_sta_cfg_cmd_v1 sta = {0};
    sta.sta_id       = IWL_AP_STA_ID;
    sta.link_id      = 0;
    for (int i = 0; i < 6; i++) sta.peer_mld_address[i] = s_ap_bssid[i];
    for (int i = 0; i < 6; i++) sta.peer_link_address[i] = s_ap_bssid[i];
    sta.station_type = IWL_FW_STA_TYPE_PEER;
    /* Linux sets MFP for not-yet-authorized peers when STA_EXP_MFP exists. */
    sta.mfp          = 1;
    term_puts("iwl: assoc: STA_CONFIG AP sta_id=");
    prh8((uint8_t)IWL_AP_STA_ID); term_puts(" link=0 mfp=1 ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_STA_CONFIG,
                     &sta, sizeof(sta), (void *)0, 0);
    if (r < 0) { term_puts("iwl: STA_CONFIG AP failed\n"); iwl_dump_fw_error(); return 0; }

    /* Linux's MLD state transition emits STA_CONFIG twice before TLC; the
     * second command refreshes the AP peer after mac80211 moves 0->1. */
    term_puts("iwl: assoc: STA_CONFIG AP refresh ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_STA_CONFIG,
                     &sta, sizeof(sta), (void *)0, 0);
    if (r < 0) { term_puts("iwl: STA_CONFIG AP refresh failed\n"); iwl_dump_fw_error(); return 0; }

    struct iwl_tlc_config_cmd_v4 tlc = {0};
    tlc.sta_id = IWL_AP_STA_ID;
    tlc.max_ch_width = 0;         /* 20 MHz */
    tlc.mode = 0;                 /* NON_HT/legacy */
    tlc.chains = tx_tlc_chains_from_nvm();
    tlc.non_ht_rates = 0x0FF0;    /* Linux pre-auth trace: OFDM basic set */
    term_puts("iwl: assoc: TLC_MNG_CONFIG sta="); prh8((uint8_t)IWL_AP_STA_ID);
    term_puts(" chains=0x"); prh8(tlc.chains); term_puts(" nonht=0x");
    prh16(tlc.non_ht_rates); term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_DATA_PATH, IWL_CMD_TLC_MNG_CONFIG,
                     &tlc, sizeof(tlc), (void *)0, 0);
    if (r < 0) { term_puts("iwl: TLC_MNG_CONFIG failed\n"); iwl_dump_fw_error(); return 0; }

    /* ── 6. SESSION_PROTECTION mirrors Linux's pre-auth association window. */
    struct iwl_session_prot_cmd prot = {0};
    prot.id_and_color = 0;                  /* cmd v1 uses raw MAC id */
    prot.action       = FW_CTXT_ACTION_ADD;
    prot.conf_id      = IWL_SESSION_PROT_CONF_ASSOC;
    prot.duration_tu  = 878;                /* MSEC_TO_TU(900) */
    term_puts("iwl: assoc: SESSION_PROTECTION assoc 878TU ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_SESSION_PROTECTION,
                     &prot, sizeof(prot), (void *)0, 0);
    if (r < 0) { term_puts("iwl: SESSION_PROTECTION failed\n"); iwl_dump_fw_error(); return 0; }

    uint32_t *stts32 = iwl_rx_stts_raw();
    if (!stts32) return 0;
    struct iwl_rb_status *rs = (struct iwl_rb_status *)stts32;

    /* ── 7. SCD_QUEUE_CONFIG_CMD ADD for AP STA. ────────────────────────── */
    if (!s_aptxq_tfd_ring) {
        s_aptxq_tfd_ring = (struct iwl_tfh_tfd *)
            dma_alloc_aligned(sizeof(struct iwl_tfh_tfd) * IWL_DATA_QUEUE_SIZE,
                              4096, &s_aptxq_tfd_phys);
        if (!s_aptxq_tfd_ring) { term_puts("iwl: aptxq tfd alloc FAIL\n"); return 0; }
        s_aptxq_first_tb = (uint8_t *)
            dma_alloc_aligned(IWL_FIRST_TB_STRIDE * IWL_DATA_QUEUE_SIZE,
                              64, &s_aptxq_first_tb_phys);
        if (!s_aptxq_first_tb) { term_puts("iwl: aptxq first_tb alloc FAIL\n"); return 0; }
        s_aptxq_cmd_buf = (uint8_t *)
            dma_alloc_aligned(IWL_AP_TX_CMD_STRIDE * IWL_DATA_QUEUE_SIZE,
                              4096, &s_aptxq_cmd_buf_phys);
        if (!s_aptxq_cmd_buf) { term_puts("iwl: aptxq cmd_buf alloc FAIL\n"); return 0; }
        s_aptxq_bc_tbl = (uint8_t *)
            dma_alloc_aligned(IWL_BC_TBL_SIZE * 2u, 4096, &s_aptxq_bc_tbl_phys);
        if (!s_aptxq_bc_tbl) { term_puts("iwl: aptxq bc_tbl alloc FAIL\n"); return 0; }
    }
    if (!ap_txq_add(IWL_MGMT_QUEUE_TID, "assoc"))
        return 0;
    s_ap_ready = 1;

    /* ── 8. Build + TX a 802.11 AUTH frame (Open System, seq=1). ────────── */
    uint8_t fr[256];
    for (uint16_t i = 0; i < sizeof(fr); i++) fr[i] = 0;
    fr[0] = 0xB0; fr[1] = 0x00;                 /* FC: mgmt(0), AUTH(0xB) */
    fr[2] = 0x00; fr[3] = 0x00;                 /* Duration */
    for (int i = 0; i < 6; i++) fr[4  + i] = s_ap_bssid[i];   /* DA = AP */
    for (int i = 0; i < 6; i++) fr[10 + i] = my_mac[i];       /* SA = us */
    for (int i = 0; i < 6; i++) fr[16 + i] = s_ap_bssid[i];   /* BSSID = AP */
    fr[22] = 0x00; fr[23] = 0x00;               /* SeqCtl */
    fr[24] = 0x00; fr[25] = 0x00;               /* algo = Open (0) */
    fr[26] = 0x01; fr[27] = 0x00;               /* seq = 1 */
    fr[28] = 0x00; fr[29] = 0x00;               /* status = 0 (req) */

    uint8_t fw_rates_ver = iwl_fw_rates_ver(iwl_fw_info_get());
    if (!fw_rates_ver) {
        term_puts("iwl: assoc: inconsistent firmware rate API table\n");
        return 0;
    }
    uint32_t auth_rate = auth_ofdm_6m_rate_n_flags(fw_rates_ver);
    term_puts("iwl: assoc: fw_rates_ver=");
    prh8(fw_rates_ver);
    term_puts(" auth_rate=0x");
    prh32(auth_rate);
    term_putchar('\n');
    s_ap_rate_n_flags = auth_rate;

    uint32_t pre_int  = creg_r32(CSR_INT);
    uint32_t pre_fhint = creg_r32(CSR_FH_INT_STATUS);
    if (pre_int)   creg_w32(CSR_INT, pre_int);
    if (pre_fhint) creg_w32(CSR_FH_INT_STATUS, pre_fhint);
    term_puts("iwl: assoc: TX AUTH (Open seq=1) da=");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(fr[4 + i]);
    }
    term_puts(" sa=");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(fr[10 + i]);
    }
    term_puts(" bssid=");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(fr[16 + i]);
    }
    term_puts(" ...\n");
    if (!aptxq_tx_mgmt_frame(fr, 30, auth_rate)) return 0;

    /* Poll RX up to 2 s for TX completion status and the AUTH response. */
    int saw_tx_done = 0, saw_tx_success = 0, saw_auth_resp = 0, saw_assert = 0;
    uint32_t rx_mpdu = 0, rx_beacon = 0, rx_other = 0;
    uint16_t tx_status = 0xFFFFu;
    uint8_t auth_status_lo = 0, auth_status_hi = 0, auth_seq_lo = 0;
    for (int t = 0; t < 200 && !(saw_tx_done && saw_auth_resp); t++) {
        iwl_delay(10000);
        if (creg_r32(CSR_INT) & 0x02000000u) { saw_assert = 1; break; }
        uint16_t cur = rx_closed_index(rs);
        while (s_rx_read != cur) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;
                if (pkt->hdr.cmd == IWL_CMD_TX_CMD &&
                    pkt->hdr.group_id == IWL_GROUP_LEGACY) {
                    saw_tx_done = 1;
                    int tx_ok = 0;
                    uint16_t st = 0xFFFFu;
                    if (print_tx_cmd_response(pkt, plen, &tx_ok, &st)) {
                        saw_tx_success = tx_ok;
                        tx_status = st;
                    }
                } else if (pkt->hdr.cmd == IWL_CMD_REPLY_RX_MPDU) {
                    uint16_t frame_len = 0;
                    const uint8_t *fp = rx_mpdu_frame(pkt, plen, &frame_len);
                    rx_mpdu++;
                    if (fp && frame_len >= 24u) {
                        uint8_t fc0 = fp[0];
                        if (fc0 == 0xB0 && frame_len >= 30u) {
                            int match = mac_eq6(fp + 4, my_mac) &&
                                        mac_eq6(fp + 10, s_ap_bssid) &&
                                        mac_eq6(fp + 16, s_ap_bssid);
                            if (match) {
                                saw_auth_resp = 1;
                                auth_status_lo = fp[28];
                                auth_status_hi = fp[29];
                                auth_seq_lo    = fp[26];
                            }
                            term_puts("    RX AUTH_RESP da=");
                            for (int i = 0; i < 6; i++) {
                                if (i) term_putchar(':');
                                prh8(fp[4 + i]);
                            }
                            term_puts(" sa=");
                            for (int i = 0; i < 6; i++) {
                                if (i) term_putchar(':');
                                prh8(fp[10 + i]);
                            }
                            term_puts(" bssid=");
                            for (int i = 0; i < 6; i++) {
                                if (i) term_putchar(':');
                                prh8(fp[16 + i]);
                            }
                            term_puts(" match=");
                            prh8((uint8_t)match);
                            term_putchar('\n');
                        } else if (fc0 == 0x80 || fc0 == 0x50) {
                            rx_beacon++;
                        } else {
                            rx_other++;
                            if (rx_other <= 4) {
                                term_puts("    RX MPDU fc=");
                                prh8(fc0);
                                term_puts(" len=");
                                prh16(frame_len);
                                term_puts(" da=");
                                for (int i = 0; i < 6; i++) {
                                    if (i) term_putchar(':');
                                    prh8(fp[4 + i]);
                                }
                                term_putchar('\n');
                            }
                        }
                    }
                }
            }
            rx_consume_read(vid);
        }
    }

    uint32_t post_int  = creg_r32(CSR_INT);
    uint32_t post_fhint = creg_r32(CSR_FH_INT_STATUS);
    term_puts("  post-auth: INT=0x"); prh32(post_int);
    term_puts(" FHINT=0x");      prh32(post_fhint);
    term_putchar('\n');

    if (saw_assert) {
        term_puts("iwl: assoc: firmware SW_ERR after AUTH TX\n");
        iwl_dump_fw_error();
        return 0;
    }
    term_puts("iwl: assoc: tx_done="); prh32((uint32_t)saw_tx_done);
    term_puts(" tx_ok=");              prh32((uint32_t)saw_tx_success);
    term_puts(" tx_status=0x");        prh16(tx_status);
    term_puts(" auth_resp=");           prh32((uint32_t)saw_auth_resp);
    term_puts(" rx_mpdu=");             prh32(rx_mpdu);
    term_puts(" bcn/prsp=");            prh32(rx_beacon);
    term_puts(" other=");               prh32(rx_other);
    uint16_t auth_status = 0xFFFFu;
    if (saw_auth_resp) {
        auth_status = (uint16_t)((uint16_t)auth_status_lo |
                                 ((uint16_t)auth_status_hi << 8));
        term_puts(" auth_seq=");        prh8(auth_seq_lo);
        term_puts(" auth_status=");     prh16(auth_status);
    }
    term_putchar('\n');

    if (!saw_auth_resp || auth_seq_lo != 2 || auth_status != 0) {
        term_puts("iwl: assoc: AUTH not accepted; skip ASSOC_REQ\n");
        return 0;
    }

    /* ── 9. Build + TX an 802.11 Association Request. ───────────────────── */
    uint16_t assoc_len = build_assoc_req_frame(ap, my_mac, fr, sizeof(fr));
    if (!assoc_len) {
        term_puts("iwl: assoc: failed to build ASSOC_REQ\n");
        return 0;
    }

    pre_int  = creg_r32(CSR_INT);
    pre_fhint = creg_r32(CSR_FH_INT_STATUS);
    if (pre_int)   creg_w32(CSR_INT, pre_int);
    if (pre_fhint) creg_w32(CSR_FH_INT_STATUS, pre_fhint);
    term_puts("iwl: assoc: TX ASSOC_REQ len=");
    prh16(assoc_len);
    term_puts(" da=");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(fr[4 + i]);
    }
    term_puts(" sa=");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        prh8(fr[10 + i]);
    }
    term_puts(" ...\n");
    if (!aptxq_tx_mgmt_frame(fr, assoc_len, auth_rate)) return 0;

    int saw_assoc_tx_done = 0, saw_assoc_tx_success = 0, saw_assoc_resp = 0;
    int saw_eapol_m1 = 0;
    struct wpa_eapol_key_rx eapol_m1 = {0};
    uint16_t assoc_tx_status = 0xFFFFu;
    uint16_t assoc_status = 0xFFFFu, assoc_aid = 0;
    rx_mpdu = 0; rx_beacon = 0; rx_other = 0; saw_assert = 0;
    for (int t = 0; t < 200 &&
         !(saw_assoc_tx_done && saw_assoc_resp && (!pass_len || saw_eapol_m1));
         t++) {
        iwl_delay(10000);
        if (creg_r32(CSR_INT) & 0x02000000u) { saw_assert = 1; break; }
        uint16_t cur = rx_closed_index(rs);
        while (s_rx_read != cur) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;
                if (pkt->hdr.cmd == IWL_CMD_TX_CMD &&
                    pkt->hdr.group_id == IWL_GROUP_LEGACY) {
                    saw_assoc_tx_done = 1;
                    int tx_ok = 0;
                    uint16_t st = 0xFFFFu;
                    if (print_tx_cmd_response(pkt, plen, &tx_ok, &st)) {
                        saw_assoc_tx_success = tx_ok;
                        assoc_tx_status = st;
                    }
                } else if (pkt->hdr.cmd == IWL_CMD_REPLY_RX_MPDU) {
                    uint16_t frame_len = 0;
                    const uint8_t *fp = rx_mpdu_frame(pkt, plen, &frame_len);
                    rx_mpdu++;
                    if (fp && frame_len >= 24u) {
                        uint8_t fc0 = fp[0];
                        if (fc0 == 0x10 && frame_len >= 30u) {
                            int match = mac_eq6(fp + 4, my_mac) &&
                                        mac_eq6(fp + 10, s_ap_bssid) &&
                                        mac_eq6(fp + 16, s_ap_bssid);
                            uint16_t st = rd16(fp + 26);
                            uint16_t aid = (uint16_t)(rd16(fp + 28) & 0x3FFFu);
                            if (match) {
                                saw_assoc_resp = 1;
                                assoc_status = st;
                                assoc_aid = aid;
                            }
                            term_puts("    RX ASSOC_RESP da=");
                            for (int i = 0; i < 6; i++) {
                                if (i) term_putchar(':');
                                prh8(fp[4 + i]);
                            }
                            term_puts(" sa=");
                            for (int i = 0; i < 6; i++) {
                                if (i) term_putchar(':');
                                prh8(fp[10 + i]);
                            }
                            term_puts(" bssid=");
                            for (int i = 0; i < 6; i++) {
                                if (i) term_putchar(':');
                                prh8(fp[16 + i]);
                            }
                            term_puts(" match=");
                            prh8((uint8_t)match);
                            term_puts(" status=");
                            prh16(st);
                            term_puts(" aid=");
                            prh16(aid);
                            term_putchar('\n');
                        } else if (fc0 == 0x80 || fc0 == 0x50) {
                            rx_beacon++;
                        } else {
                            struct wpa_eapol_key_rx ek;
                            if (parse_eapol_key(fp, frame_len, my_mac, &ek)) {
                                print_eapol_key(&ek);
                                if ((ek.key_info & (1u << 7)) &&
                                    !(ek.key_info & (1u << 8))) {
                                    eapol_m1 = ek;
                                    saw_eapol_m1 = 1;
                                }
                            } else {
                                rx_other++;
                                if (rx_other <= 4) {
                                    term_puts("    RX MPDU fc=");
                                    prh8(fc0);
                                    term_puts(" len=");
                                    prh16(frame_len);
                                    term_puts(" da=");
                                    for (int i = 0; i < 6; i++) {
                                        if (i) term_putchar(':');
                                        prh8(fp[4 + i]);
                                    }
                                    term_putchar('\n');
                                }
                            }
                        }
                    }
                }
            }
            rx_consume_read(vid);
        }
    }

    post_int  = creg_r32(CSR_INT);
    post_fhint = creg_r32(CSR_FH_INT_STATUS);
    term_puts("  post-assoc: INT=0x"); prh32(post_int);
    term_puts(" FHINT=0x");            prh32(post_fhint);
    term_putchar('\n');

    if (saw_assert) {
        term_puts("iwl: assoc: firmware SW_ERR after ASSOC_REQ TX\n");
        iwl_dump_fw_error();
        return 0;
    }
    term_puts("iwl: assoc: assoc_tx_done=");
    prh32((uint32_t)saw_assoc_tx_done);
    term_puts(" assoc_tx_ok=");
    prh32((uint32_t)saw_assoc_tx_success);
    term_puts(" assoc_tx_status=0x");
    prh16(assoc_tx_status);
    term_puts(" assoc_resp=");
    prh32((uint32_t)saw_assoc_resp);
    term_puts(" assoc_status=");
    prh16(assoc_status);
    term_puts(" aid=");
    prh16(assoc_aid);
    term_puts(" rx_mpdu=");
    prh32(rx_mpdu);
    term_puts(" bcn/prsp=");
    prh32(rx_beacon);
    term_puts(" other=");
    prh32(rx_other);
    term_puts(" eapol_m1=");
    prh32((uint32_t)saw_eapol_m1);
    term_putchar('\n');
    if (!saw_assoc_resp || assoc_status != 0)
        return 0;

    for (uint64_t i = 0; i < sizeof(mac); i++) ((uint8_t *)&mac)[i] = 0;
    mac.id_and_color = 0;
    mac.action       = FW_CTXT_ACTION_MODIFY;
    mac.mac_type     = FW_MAC_TYPE_BSS_STA;
    for (int i = 0; i < 6; i++) mac.local_mld_addr[i] = my_mac[i];
    mac.filter_flags = MAC_CFG_FILTER_ACCEPT_GRP;
    mac.client.is_assoc = 1;
    mac.client.assoc_id = assoc_aid;
    term_puts("iwl: assoc: MAC_CONFIG MODIFY associated aid=");
    prh16(assoc_aid);
    term_puts(" ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_MAC_CONFIG,
                     &mac, sizeof(mac), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: MAC_CONFIG associated failed\n");
        iwl_dump_fw_error();
        return 0;
    }

    for (uint64_t i = 0; i < sizeof(sta); i++) ((uint8_t *)&sta)[i] = 0;
    sta.sta_id       = IWL_AP_STA_ID;
    sta.link_id      = 0;
    for (int i = 0; i < 6; i++) sta.peer_mld_address[i] = s_ap_bssid[i];
    for (int i = 0; i < 6; i++) sta.peer_link_address[i] = s_ap_bssid[i];
    sta.station_type = IWL_FW_STA_TYPE_PEER;
    sta.assoc_id     = assoc_aid;
    sta.mfp          = 1;
    term_puts("iwl: assoc: STA_CONFIG AP associated aid=");
    prh16(assoc_aid);
    term_puts(" mfp=1 ...\n");
    r = iwl_send_cmd(IWL_GROUP_MAC_CONF, IWL_CMD_STA_CONFIG,
                     &sta, sizeof(sta), (void *)0, 0);
    if (r < 0) {
        term_puts("iwl: STA_CONFIG AP associated failed\n");
        iwl_dump_fw_error();
        return 0;
    }

    if (!pass_len)
        return 1;

    if (!saw_eapol_m1) {
        term_puts("iwl: wpa: no EAPOL M1 after association\n");
        return 1;
    }
    if (!wpa_send_m2(ap, my_mac, pass, pass_len, &eapol_m1, auth_rate))
        return 1;

    int saw_m2_tx_done = 0, saw_m2_tx_success = 0, saw_eapol_m3 = 0;
    struct wpa_eapol_key_rx eapol_m3 = {0};
    uint16_t m2_tx_status = 0xFFFFu;
    rx_mpdu = 0; rx_other = 0; saw_assert = 0;
    for (int t = 0; t < 200 && !(saw_m2_tx_done && saw_eapol_m3); t++) {
        iwl_delay(10000);
        if (creg_r32(CSR_INT) & 0x02000000u) { saw_assert = 1; break; }
        uint16_t cur = rx_closed_index(rs);
        while (s_rx_read != cur) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;
                if (pkt->hdr.cmd == IWL_CMD_TX_CMD &&
                    pkt->hdr.group_id == IWL_GROUP_LEGACY) {
                    saw_m2_tx_done = 1;
                    int tx_ok = 0;
                    uint16_t st = 0xFFFFu;
                    if (print_tx_cmd_response(pkt, plen, &tx_ok, &st)) {
                        saw_m2_tx_success = tx_ok;
                        m2_tx_status = st;
                    }
                } else if (pkt->hdr.cmd == IWL_CMD_REPLY_RX_MPDU) {
                    uint16_t frame_len = 0;
                    const uint8_t *fp = rx_mpdu_frame(pkt, plen, &frame_len);
                    rx_mpdu++;
                    if (fp && frame_len >= 24u) {
                        struct wpa_eapol_key_rx ek;
                        if (parse_eapol_key(fp, frame_len, my_mac, &ek)) {
                            print_eapol_key(&ek);
                            if ((ek.key_info & (1u << 7)) &&
                                (ek.key_info & (1u << 8))) {
                                eapol_m3 = ek;
                                saw_eapol_m3 = 1;
                            }
                        } else {
                            rx_other++;
                            if (rx_other <= 4) {
                                term_puts("    RX MPDU fc=");
                                prh8(fp[0]);
                                term_puts(" len=");
                                prh16(frame_len);
                                term_putchar('\n');
                            }
                        }
                    }
                }
            }
            rx_consume_read(vid);
        }
    }

    if (saw_assert) {
        term_puts("iwl: wpa: firmware SW_ERR after EAPOL M2 TX\n");
        iwl_dump_fw_error();
        return 1;
    }
    term_puts("iwl: wpa: m2_tx_done=");
    prh32((uint32_t)saw_m2_tx_done);
    term_puts(" m2_tx_ok=");
    prh32((uint32_t)saw_m2_tx_success);
    term_puts(" m2_tx_status=0x");
    prh16(m2_tx_status);
    term_puts(" eapol_m3=");
    prh32((uint32_t)saw_eapol_m3);
    term_puts(" rx_mpdu=");
    prh32(rx_mpdu);
    term_puts(" other=");
    prh32(rx_other);
    term_putchar('\n');
    if (!saw_eapol_m3)
        return 1;

    if (!wpa_key_mic_ok(&eapol_m3))
        return 1;
    if (!iwl_install_pairwise_ccmp_key())
        return 1;
    if (!wpa_install_gtk_from_m3(&eapol_m3))
        return 1;
    if (!wpa_send_m4(ap, my_mac, &eapol_m3, auth_rate))
        return 1;

    int saw_m4_tx_done = 0, saw_m4_tx_success = 0;
    uint16_t m4_tx_status = 0xFFFFu;
    uint32_t rx_protected = 0;
    rx_mpdu = 0; rx_other = 0; saw_assert = 0;
    for (int t = 0; t < 100 && !saw_m4_tx_done; t++) {
        iwl_delay(10000);
        if (creg_r32(CSR_INT) & 0x02000000u) { saw_assert = 1; break; }
        uint16_t cur = rx_closed_index(rs);
        while (s_rx_read != cur) {
            uint16_t vid = 0;
            struct iwl_rx_packet *pkt = rx_packet_for_read(&vid);
            if (pkt) {
                uint32_t plen = pkt->len_n_flags & 0x3FFFu;
                if (pkt->hdr.cmd == IWL_CMD_TX_CMD &&
                    pkt->hdr.group_id == IWL_GROUP_LEGACY) {
                    saw_m4_tx_done = 1;
                    int tx_ok = 0;
                    uint16_t st = 0xFFFFu;
                    if (print_tx_cmd_response(pkt, plen, &tx_ok, &st)) {
                        saw_m4_tx_success = tx_ok;
                        m4_tx_status = st;
                    }
                } else if (pkt->hdr.cmd == IWL_CMD_REPLY_RX_MPDU) {
                    uint16_t frame_len = 0;
                    const uint8_t *fp = rx_mpdu_frame(pkt, plen, &frame_len);
                    rx_mpdu++;
                    if (fp && frame_len >= 24u) {
                        if (fp[1] & 0x40u) {
                            rx_protected++;
                        } else {
                            rx_other++;
                            if (rx_other <= 4) {
                                term_puts("    RX post-M4 MPDU fc=");
                                prh8(fp[0]);
                                term_puts(" flags=");
                                prh8(fp[1]);
                                term_puts(" len=");
                                prh16(frame_len);
                                term_putchar('\n');
                            }
                        }
                    }
                }
            }
            rx_consume_read(vid);
        }
    }

    if (saw_assert) {
        term_puts("iwl: wpa: firmware SW_ERR after EAPOL M4 TX\n");
        iwl_dump_fw_error();
        return 1;
    }
    term_puts("iwl: wpa: m4_tx_done=");
    prh32((uint32_t)saw_m4_tx_done);
    term_puts(" m4_tx_ok=");
    prh32((uint32_t)saw_m4_tx_success);
    term_puts(" m4_tx_status=0x");
    prh16(m4_tx_status);
    term_puts(" rx_mpdu=");
    prh32(rx_mpdu);
    term_puts(" protected=");
    prh32(rx_protected);
    term_puts(" other=");
    prh32(rx_other);
    term_putchar('\n');
    if (saw_m4_tx_success) {
        uint8_t old_tid = s_aptxq_tid;
        if (!ap_txq_remove(old_tid, "data"))
            return 1;
        if (!ap_txq_add(IWL_NON_QOS_QUEUE_TID, "data"))
            return 1;
        s_ap_ready = 1;
        s_net_ready = 1;
        iwl_net_attach();
        term_puts("iwl: net: iwlwifi attached; Ethernet data path ready\n");
    }
    return 1;
}
