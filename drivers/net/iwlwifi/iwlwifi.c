/*
 * iwlwifi.c - Intel Wi-Fi driver (AX200 / AX201 class).
 *
 * Phase 1 bring-up only.  See iwlwifi.h for current scope.
 *
 * Sequence this file implements (mirrors drivers/net/wireless/intel/iwlwifi
 * in Linux, specifically iwl-trans.c, iwl-io.c, and the pcie/trans.c
 * apm_init path):
 *
 *   1. pci_find + D3→D0 + BAR map (UC)
 *   2. prepare_card_hw:
 *         set CSR_HW_IF_CONFIG_REG.PREPARE, poll NIC_READY.
 *   3. apm_init:
 *         apply Linux gen2 PCIe/FH workarounds, disable L0s, set INIT_DONE.
 *   4. grab MAC access:
 *         set MAC_ACCESS_REQ, poll MAC_CLOCK_READY && !GOING_TO_SLEEP.
 *         Now PRPH / SRAM reads work.
 *   5. read HW_REV and the 22000/Qu CSR hardware MAC address window.
 */

#include <stdint.h>
#include "../../../arch/x86/io.h"
#include "../../../arch/x86/pci.h"
#include "../../../arch/x86/vm.h"
#include "../../../drivers/terminal.h"
#include "../../../kernel/alloc.h"
#include "../nic.h"
#include "iwl_csr.h"
#include "iwl_cmd.h"
#include "iwl_fw.h"
#include "iwlwifi.h"

/* ── PCI identity ─────────────────────────────────────────────────────── */
#define IWL_VID                  0x8086u
#define IWL_DID_ICL_LP_CNVi      0x34F0u   /* Surface Laptop 3 AX201 */

/* ── Driver state ─────────────────────────────────────────────────────── */
static volatile uint8_t *s_mmio;        /* BAR0, UC-mapped */
static uint64_t          s_mmio_phys;
static uint8_t           s_bus, s_dev, s_fn;
static uint32_t          s_hw_rev;
static uint8_t           s_mac[6];
static const char       *s_mac_source = "none";
static int               s_present;
static const char       *s_family = "unknown";
static struct iwl_fw_info s_fw;
static uint64_t           s_hhdm_saved;

extern void iwl_fw_reset_state(void);

/* Accessors for the phase-2 firmware loader (lives in its own file). */
volatile uint8_t *iwl_csr_base(void) { return s_mmio; }
uint64_t          iwl_hhdm(void)     { return s_hhdm_saved; }

/* ── Diagnostic hex printers ──────────────────────────────────────────── */
static void ph8(uint8_t v) {
    static const char h[] = "0123456789abcdef";
    term_putchar(h[v >> 4]); term_putchar(h[v & 0xf]);
}
static void ph32(uint32_t v) {
    ph8((uint8_t)(v >> 24)); ph8((uint8_t)(v >> 16));
    ph8((uint8_t)(v >>  8)); ph8((uint8_t)(v));
}

/* ── MMIO accessors ───────────────────────────────────────────────────── */
static inline uint32_t csr_r32(uint32_t off) {
    return *(volatile uint32_t *)(s_mmio + off);
}
static inline void csr_w32(uint32_t off, uint32_t v) {
    *(volatile uint32_t *)(s_mmio + off) = v;
}
static inline void csr_set(uint32_t off, uint32_t bits) {
    csr_w32(off, csr_r32(off) | bits);
}
static inline void csr_clr(uint32_t off, uint32_t bits) {
    csr_w32(off, csr_r32(off) & ~bits);
}

/* ── Short busy-wait ──────────────────────────────────────────────────── */
/* On Skylake+ pause is ~100 cycles (~50 ns at 2 GHz).  20 pauses ≈ 1 µs. */
void iwl_delay(uint32_t usecs) {
    uint32_t n = usecs * 20u;
    for (volatile uint32_t i = 0; i < n; i++) __asm__ volatile ("pause");
}

/* Poll `off` until (val & mask) == want.  Count is in iterations — each
 * UC MMIO read is serialized on the bus (~200-500 ns) so a generous
 * iteration count still covers tens of ms of real time without needing
 * any explicit delay.  Returns 1 on match, 0 on timeout.                */
static int poll_bit(uint32_t off, uint32_t mask, uint32_t want, uint32_t iters) {
    for (uint32_t i = 0; i < iters; i++) {
        if ((csr_r32(off) & mask) == want) return 1;
    }
    return 0;
}

/* ── 1. prepare_card_hw ────────────────────────────────────────────────── */
/*
 * Mirror of Linux's iwl_pcie_prepare_card_hw + iwl_pcie_set_hw_ready.
 *
 * Sequence:
 *   a. Try "set HW ready": set NIC_READY bit, poll for the chip to hold it
 *      (it will if the device is already in a good state).
 *   b. If that fails, disable link power management, then enter the full
 *      retry loop: pulse PREPARE, poll for NIC_READY up to 150 ms, up to
 *      10 retries.
 *   c. On success, write OS_ALIVE into the mailbox to tell the CSME/ME
 *      firmware that the OS has taken ownership of the radio.
 */
static int set_hw_ready(void) {
    csr_set(CSR_HW_IF_CONFIG_REG, CSR_HW_IF_CONFIG_REG_BIT_NIC_READY);
    /* ~50 ms worth of MMIO polls */
    if (!poll_bit(CSR_HW_IF_CONFIG_REG,
                  CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
                  CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
                  200000))
        return 0;
    csr_set(CSR_MBOX_SET_REG, CSR_MBOX_SET_REG_OS_ALIVE);
    return 1;
}

static int prepare_card_hw(void) {
    if (set_hw_ready()) return 1;

    /* Disable link power management while we nudge the chip. */
    csr_w32(CSR_DBG_LINK_PWR_MGMT_REG, CSR_RESET_LINK_PWR_MGMT_DISABLED);
    iwl_delay(1000);

    for (int retry = 0; retry < 10; retry++) {
        csr_set(CSR_HW_IF_CONFIG_REG, CSR_HW_IF_CONFIG_REG_PREPARE);
        /* Up to ~150 ms with no explicit per-iteration delay */
        if (poll_bit(CSR_HW_IF_CONFIG_REG,
                     CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
                     CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
                     500000)) {
            csr_set(CSR_MBOX_SET_REG, CSR_MBOX_SET_REG_OS_ALIVE);
            return 1;
        }
        iwl_delay(25000);    /* 25 ms between retries */
    }
    return 0;
}

static uint8_t find_pm_cap(void) {
    uint8_t cap = (uint8_t)(pci_read32(s_bus, s_dev, s_fn, 0x34) & 0xFCu);
    for (int guard = 0; cap && guard < 48; guard++) {
        uint32_t c = pci_read32(s_bus, s_dev, s_fn, cap);
        if ((c & 0xFFu) == 0x01u)
            return cap;
        cap = (uint8_t)((c >> 8) & 0xFCu);
    }
    return 0;
}

static void pci_d3hot_d0_cycle(void) {
    uint8_t cap = find_pm_cap();
    if (!cap) {
        term_puts("iwl: PM cap missing; skip PCI power cycle\n");
        return;
    }

    uint16_t cmd = pci_read16(s_bus, s_dev, s_fn, 0x04);
    pci_write16(s_bus, s_dev, s_fn, 0x04, (uint16_t)(cmd & ~(uint16_t)0x0006u));

    uint16_t pmcs = pci_read16(s_bus, s_dev, s_fn, (uint8_t)(cap + 4u));
    term_puts("iwl: PCI D3hot->D0 PMCS=0x");
    ph32(pmcs);
    term_puts(" ...\n");

    pci_write16(s_bus, s_dev, s_fn, (uint8_t)(cap + 4u),
                (uint16_t)((pmcs & ~(uint16_t)0x0003u) | 0x0003u));
    iwl_delay(50000);

    pmcs = pci_read16(s_bus, s_dev, s_fn, (uint8_t)(cap + 4u));
    pci_write16(s_bus, s_dev, s_fn, (uint8_t)(cap + 4u),
                (uint16_t)(pmcs & ~(uint16_t)0x0003u));
    iwl_delay(20000);

    pci_write16(s_bus, s_dev, s_fn, 0x04, (uint16_t)(cmd | 0x0006u));
}

/* Stop the device's bus-master DMA so any leftover state from the previous
 * owner (UEFI, prior Linux session, or a previous Diamond run) is cleared.
 * AX201 is pre-Bz, so Linux uses CSR_RESET STOP_MASTER/MASTER_DISABLED. */
static void apm_stop_master(void) {
    csr_set(CSR_RESET, CSR_RESET_REG_FLAG_STOP_MASTER);
    for (int i = 0; i < 1000; i++) {
        if (csr_r32(CSR_RESET) & CSR_RESET_REG_FLAG_MASTER_DISABLED) break;
        iwl_delay(1);
    }
}

static void sw_reset(void) {
    csr_set(CSR_RESET, CSR_RESET_REG_FLAG_SW_RESET);
    iwl_delay(6000);
}

static void stop_device(void) {
    csr_w32(CSR_INT_MASK, 0);
    csr_w32(CSR_INT, 0xFFFFFFFFu);
    csr_w32(CSR_FH_INT_STATUS, 0xFFFFFFFFu);
    csr_clr(CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
    apm_stop_master();
    sw_reset();
    csr_clr(CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
}

static void retake_ownership_after_stop(void) {
    sw_reset();
    term_puts("iwl: retake ownership ... ");
    if (prepare_card_hw()) {
        term_puts("OK\n");
    } else {
        term_puts("FAILED (CFG=0x");
        ph32(csr_r32(CSR_HW_IF_CONFIG_REG));
        term_puts(")\n");
    }
}

/* ── 2. apm_init (gen2 variant — AX201 / 22000 series) ────────────────── */
/*
 * Mirror of Linux's iwl_pcie_gen2_apm_init / apm_config ordering closely
 * enough for the context-info firmware path.
 */
static int apm_init(void) {
    /* Disable L0s without affecting L1, and max the FH wait threshold.
     * Linux carries both as gen2 workarounds before firmware start. */
    csr_set(CSR_GIO_CHICKEN_BITS,
            CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX);
    csr_set(CSR_DBG_HPET_MEM_REG, CSR_DBG_HPET_MEM_REG_VAL);

    /* HAP wakeup support — chip doesn't bring up MAC clock without this. */
    csr_set(CSR_HW_IF_CONFIG_REG, CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A);

    /* Driver→firmware: "OS init done, MAC may run." */
    csr_set(CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
    iwl_delay(20);

    /* apm_config: device-side L0s disable. */
    csr_set(CSR_GIO_REG, (1u << 1));

    /* If the chip ended up in GOING_TO_SLEEP, clear that. */
    csr_clr(CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP);

    return 1;
}

/* ── 3. Grab MAC access semaphore ─────────────────────────────────────── */
/*
 * After the MAC is out of reset it can still be in low-power sleep.  To
 * safely read chip internals (PRPH, OTP, etc) we set MAC_ACCESS_REQ and
 * wait for MAC_CLOCK_READY && !GOING_TO_SLEEP.
 */
static int grab_nic_access(void) {
    csr_set(CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
    /* Linux's gen2 grab_nic_access has a 2 µs delay before polling */
    iwl_delay(5);
    /* Need ALL THREE bits to confirm we actually own the MAC: REQ honoured,
     * clock ready, and chip not on its way to sleep.                      */
    for (uint32_t t = 0; t < 200000u; t++) {
        uint32_t v = csr_r32(CSR_GP_CNTRL);
        if ((v & CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ) &&
            (v & CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY) &&
            !(v & CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP))
            return 1;
    }
    return 0;
}

static void release_nic_access(void) {
    csr_clr(CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
}

/* External wrappers for the firmware-load path in iwl_fw_load.c. */
int  iwl_grab_nic_access_ext(void)    { return grab_nic_access(); }
void iwl_release_nic_access_ext(void) { release_nic_access(); }

/* ── 4. MAC address read ───────────────────────────────────────────────── */
static int valid_mac(const uint8_t mac[6]) {
    int all_zero = 1, all_ff = 1;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0x00u) all_zero = 0;
        if (mac[i] != 0xFFu) all_ff = 0;
    }
    return !all_zero && !all_ff && !(mac[0] & 0x01u);
}

static void flip_hw_address(uint32_t mac0, uint32_t mac1, uint8_t out[6]) {
    out[0] = (uint8_t)(mac0 >> 24);
    out[1] = (uint8_t)(mac0 >> 16);
    out[2] = (uint8_t)(mac0 >> 8);
    out[3] = (uint8_t)mac0;
    out[4] = (uint8_t)(mac1 >> 8);
    out[5] = (uint8_t)mac1;
}

/* AX201/Qu mirrors Linux's 22000 path: base->mac_addr_from_csr = 0x380.
 * Prefer the strapped OEM address at 0x388/0x38c, then fall back to OTP at
 * 0x380/0x384.  The old CSR_EEPROM_REG word offsets are for earlier chips. */
static void read_mac_from_csr_window(void) {
    uint8_t mac[6];
    for (int i = 0; i < 6; i++) s_mac[i] = 0;
    s_mac_source = "none";

    uint32_t strap0 = csr_r32(CSR_MAC_ADDR0_STRAP);
    uint32_t strap1 = csr_r32(CSR_MAC_ADDR1_STRAP);
    flip_hw_address(strap0, strap1, mac);
    if (valid_mac(mac)) {
        for (int i = 0; i < 6; i++) s_mac[i] = mac[i];
        s_mac_source = "csr-strap";
        return;
    }

    uint32_t otp0 = csr_r32(CSR_MAC_ADDR0_OTP);
    uint32_t otp1 = csr_r32(CSR_MAC_ADDR1_OTP);
    flip_hw_address(otp0, otp1, mac);
    if (valid_mac(mac)) {
        for (int i = 0; i < 6; i++) s_mac[i] = mac[i];
        s_mac_source = "csr-otp";
    }
}

/*
 * The MAC address on iwlwifi cards lives in OTP (One-Time-Programmable)
 * fuses exposed through CSR_EEPROM_REG (same interface as the older EEPROM
 * path, hence the name).  Address 0x15 / 0x16 / 0x17 hold the 6 bytes
 * (2 bytes per 16-bit word).  We use polled synchronous mode — tiny data.
 *
 * Returns 1 on success, 0 on timeout / bad-read.
 */
static int otp_read_word(uint16_t addr, uint16_t *out) {
    csr_w32(CSR_EEPROM_REG,
            ((uint32_t)addr << 2) & CSR_EEPROM_REG_MSK_ADDR);
    csr_set(CSR_EEPROM_REG, CSR_EEPROM_REG_BIT_CMD);

    for (uint32_t t = 0; t < 10000; t++) {      /* 10 ms */
        uint32_t r = csr_r32(CSR_EEPROM_REG);
        if (r & CSR_EEPROM_REG_READ_VALID_MSK) {
            *out = (uint16_t)(r >> 16);
            return 1;
        }
        iwl_delay(1);
    }
    return 0;
}

/* Read the MAC address from OTP word offsets 0x15..0x17.
 *
 * NOTE: on 22000/AX2xx CNVi silicon (AX201 included), Linux reads the MAC
 * from the CSR hardware-address window above, not these legacy EEPROM words.
 * Keep this fallback for older iwlwifi generations and for diagnostics if a
 * future device lacks the 22000 CSR window.                               */
static void read_mac_from_otp(void) {
    for (int i = 0; i < 6; i++) s_mac[i] = 0;

    /* Check for OTP ECC-uncorrectable errors before we start */
    uint32_t otp_gp = csr_r32(CSR_OTP_GP_REG);
    if (otp_gp & CSR_OTP_GP_REG_ECC_UNCORR_STATUS_MSK) return;

    uint16_t w[3];
    for (int i = 0; i < 3; i++)
        if (!otp_read_word((uint16_t)(0x15 + i), &w[i])) return;

    s_mac[0] = (uint8_t)(w[0] & 0xFF); s_mac[1] = (uint8_t)(w[0] >> 8);
    s_mac[2] = (uint8_t)(w[1] & 0xFF); s_mac[3] = (uint8_t)(w[1] >> 8);
    s_mac[4] = (uint8_t)(w[2] & 0xFF); s_mac[5] = (uint8_t)(w[2] >> 8);
    if (valid_mac(s_mac)) s_mac_source = "eeprom-otp";
}

/* Map HW_REV type bits to a readable family name. */
static void classify_hw_rev(uint32_t rev) {
    uint32_t type = (rev >> CSR_HW_REV_TYPE_SHIFT) & CSR_HW_REV_TYPE_MASK;
    switch (type) {
    case IWL_HW_REV_TYPE_QU:  s_family = "Qu (AX201, firmware: Qu-c0-hr-b0)"; break;
    case IWL_HW_REV_TYPE_QUZ: s_family = "QuZ (AX201 variant)"; break;
    case IWL_HW_REV_TYPE_PU:  s_family = "Pu (9000-series)"; break;
    case IWL_HW_REV_TYPE_TH:  s_family = "Th (Thunder Peak)"; break;
    case IWL_HW_REV_TYPE_QNJ: s_family = "QnJ"; break;
    case IWL_HW_REV_TYPE_SO:  s_family = "So (AX210/AX211)"; break;
    default:                  s_family = "unknown iwlwifi silicon"; break;
    }
}

/* ── Probe entry point ─────────────────────────────────────────────────── */
int iwl_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    (void)kphys; (void)kvirt;
    s_hhdm_saved = hhdm_offset;

    term_puts("iwl: probe start\n");

    if (!pci_find(IWL_VID, IWL_DID_ICL_LP_CNVi, &s_bus, &s_dev, &s_fn)) {
        term_puts("iwl: no 8086:34F0 device found\n");
        return 0;
    }

    term_puts("iwl: found "); ph8(s_bus); term_putchar(':');
    ph8(s_dev); term_putchar('.'); ph8(s_fn); term_putchar('\n');

    term_puts("iwl: D3->D0\n");
    uint8_t cap = (uint8_t)(pci_read32(s_bus, s_dev, s_fn, 0x34) & 0xFCu);
    while (cap) {
        uint32_t c = pci_read32(s_bus, s_dev, s_fn, cap);
        if ((c & 0xFFu) == 0x01u) {
            uint32_t pmcs = pci_read32(s_bus, s_dev, s_fn, cap + 4u);
            if (pmcs & 0x03u) {
                pci_write32(s_bus, s_dev, s_fn, cap + 4u, pmcs & ~0x03u);
                iwl_delay(10000);
            }
            break;
        }
        cap = (uint8_t)((c >> 8) & 0xFCu);
    }

    /* Bus master + memory-space enable */
    uint16_t cmd = pci_read16(s_bus, s_dev, s_fn, 0x04);
    pci_write16(s_bus, s_dev, s_fn, 0x04, (uint16_t)(cmd | 0x0006u));

    /* BAR0 is 64-bit on ICL-LP */
    uint32_t lo = pci_read32(s_bus, s_dev, s_fn, 0x10);
    uint32_t hi = pci_read32(s_bus, s_dev, s_fn, 0x14);
    s_mmio_phys = ((uint64_t)hi << 32) | (lo & ~0xFu);
    if (s_mmio_phys == 0) {
        term_puts("iwl: BAR0 not programmed\n");
        return 0;
    }

    uint64_t mmio_va = s_mmio_phys + hhdm_offset;
    /* 16 KiB BAR; map all four 4 KiB pages as UC (WB reads stall CNVi) */
    ioremap_uc(mmio_va,           s_mmio_phys,           hhdm_offset);
    ioremap_uc(mmio_va + 0x1000u, s_mmio_phys + 0x1000u, hhdm_offset);
    ioremap_uc(mmio_va + 0x2000u, s_mmio_phys + 0x2000u, hhdm_offset);
    ioremap_uc(mmio_va + 0x3000u, s_mmio_phys + 0x3000u, hhdm_offset);
    s_mmio = (volatile uint8_t *)(uintptr_t)mmio_va;

    term_puts("iwl: BAR0=0x"); ph32(hi); ph32(lo);
    term_putchar('\n');

    /* Sanity sniff: if the first CSR read is 0xFFFFFFFF the device is not
     * responding (bad mapping / still in D3 / MSE not enabled).  Bail out
     * instead of entering a 150 ms×10 poll loop that can't succeed.      */
    uint32_t sniff = csr_r32(CSR_HW_IF_CONFIG_REG);
    term_puts("iwl: CFG_REG=0x"); ph32(sniff); term_putchar('\n');
    if (sniff == 0xFFFFFFFFu) {
        term_puts("iwl: device not responding — aborting probe\n");
        return 0;
    }

    term_puts("iwl: prepare_card_hw ... ");
    if (!prepare_card_hw()) {
        term_puts("FAIL (CFG=0x"); ph32(csr_r32(CSR_HW_IF_CONFIG_REG));
        term_puts(")\n");
        return 0;
    }
    term_puts("OK\n");

    term_puts("iwl: apm_init ... ");
    if (!apm_init()) {
        term_puts("FAIL (GP_CNTRL=0x"); ph32(csr_r32(CSR_GP_CNTRL));
        term_puts(")\n");
        return 0;
    }
    term_puts("OK\n");

    term_puts("iwl: grab_nic_access ... ");
    if (!grab_nic_access()) {
        term_puts("FAIL (GP_CNTRL=0x"); ph32(csr_r32(CSR_GP_CNTRL));
        term_puts(")\n");
        return 0;
    }
    term_puts("OK\n");

    s_hw_rev = csr_r32(CSR_HW_REV);
    classify_hw_rev(s_hw_rev);
    read_mac_from_csr_window();
    if (!valid_mac(s_mac))
        read_mac_from_otp();

    release_nic_access();

    term_puts("iwl: HW_REV=0x"); ph32(s_hw_rev);
    term_puts(" family="); term_puts(s_family);
    term_putchar('\n');
    term_puts("iwl: MAC=");
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        ph8(s_mac[i]);
    }
    term_puts(" source=");
    term_puts(s_mac_source);
    term_putchar('\n');

    s_present = 1;

    /* Parse the embedded firmware TLV blob so we know what sections are
     * available for the upload path — doesn't touch the device.          */
    if (iwl_fw_parse(&s_fw)) {
        term_puts("iwl: fw "); term_puts(s_fw.human);
        term_puts(" size=0x"); ph32(s_fw.raw_size);
        term_putchar('\n');
        term_puts("iwl: fw images: init=");
        ph8(s_fw.img[IWL_FW_IMG_INIT].n_sections);
        term_puts("sec reg=");
        ph8(s_fw.img[IWL_FW_IMG_REGULAR].n_sections);
        term_puts("sec wol=");
        ph8(s_fw.img[IWL_FW_IMG_WOWLAN].n_sections);
        term_puts("sec cpus=");
        ph8(s_fw.num_cpus);
        term_putchar('\n');
        term_puts("iwl: fw api: txcmd=");
        ph8(iwl_fw_cmd_ver(&s_fw, IWL_GROUP_LEGACY, IWL_CMD_TX_CMD));
        term_puts(" txnotif=");
        ph8(iwl_fw_notif_ver(&s_fw, IWL_GROUP_LONG, IWL_CMD_TX_CMD));
        term_puts(" rxmpdu=");
        ph8(iwl_fw_notif_ver(&s_fw, IWL_GROUP_LEGACY, IWL_CMD_REPLY_RX_MPDU));
        term_puts(" tlc=");
        ph8(iwl_fw_notif_ver(&s_fw, IWL_GROUP_DATA_PATH,
                              IWL_CMD_TLC_MNG_UPDATE_NOTIF));
        term_puts(" rates=v");
        ph8(iwl_fw_rates_ver(&s_fw));
        term_putchar('\n');
    } else {
        term_puts("iwl: no embedded firmware (see tools/iwl_diag.sh)\n");
    }

    /* Phase 1 is transport-level only — the device is up and we can read
     * its registers, but firmware loading / TX / RX / association are not
     * implemented yet.  Future commits will layer:
     *   - firmware TLV parser and DMA upload
     *   - TFD and RX ring setup
     *   - host-firmware command/response protocol
     *   - NVM init, regulatory, PHY configuration
     *   - scan / auth / assoc state machine
     *   - data path with 802.11 ↔ 802.3 conversion
     *   - WPA2/WPA3 4-way handshake + AES-CCMP
     * Until then, net.c's send/recv return -1.                             */
    return 1;
}

/* ── Public getters ────────────────────────────────────────────────────── */
void iwl_mac(uint8_t out[6]) { for (int i = 0; i < 6; i++) out[i] = s_mac[i]; }
uint32_t iwl_hw_rev(void)    { return s_hw_rev; }
const char *iwl_family_name(void) { return s_present ? s_family : "absent"; }
const struct iwl_fw_info *iwl_fw_info_get(void) { return &s_fw; }

/* ── nic_driver glue ───────────────────────────────────────────────────── */
static int iwl_nic_link_up(void) {
    return iwl_cmd_net_link_up();
}
static int iwl_nic_send(const void *d, uint16_t l) {
    return iwl_cmd_net_send(d, l);
}
static int iwl_nic_recv(void *b, uint16_t m, uint16_t *o) {
    return iwl_cmd_net_recv(b, m, o);
}

struct nic_driver iwl_driver = {
    .name    = "iwlwifi",
    .probe   = iwl_probe,
    .mac     = iwl_mac,
    .link_up = iwl_nic_link_up,
    .send    = iwl_nic_send,
    .recv    = iwl_nic_recv,
};

void iwl_net_attach(void) {
    net_attach_driver(&iwl_driver);
}

void iwl_reset(void) {
    term_puts("iwl: reset\n");

    if (s_mmio) {
        uint32_t sniff = csr_r32(CSR_HW_IF_CONFIG_REG);
        if (sniff != 0xFFFFFFFFu) {
            term_puts("iwl: stopping device ...\n");
            stop_device();
            pci_d3hot_d0_cycle();
            retake_ownership_after_stop();
        } else {
            term_puts("iwl: device not responding; clearing software state only\n");
        }
    }

    iwl_cmd_reset_state();
    iwl_fw_reset_state();

    s_present = 0;
    s_hw_rev = 0;
    s_family = "unknown";
    s_mac_source = "none";
    for (int i = 0; i < 6; i++) s_mac[i] = 0;
}
