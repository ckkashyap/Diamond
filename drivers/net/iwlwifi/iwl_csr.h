/*
 * iwl_csr.h - Intel Wi-Fi CSR (Control and Status Register) definitions
 *
 * Subset of Linux's drivers/net/wireless/intel/iwlwifi/iwl-csr.h needed for
 * PCIe bring-up on the AX201 (ICL-LP CNVi, device 0x34F0).  Register offsets
 * are stable across the 7000 / 9000 / 22000 series — only the interpretation
 * of some status bits differs per generation.
 */
#pragma once
#include <stdint.h>

/* ── CSR (always accessible, hardware-owned) registers ─────────────────── */
#define CSR_HW_IF_CONFIG_REG      0x000u
#define CSR_INT_COALESCING        0x004u
#define CSR_INT                   0x008u
#define CSR_INT_MASK              0x00Cu
#define CSR_FH_INT_STATUS         0x010u
#define CSR_GPIO_IN               0x018u
#define CSR_RESET                 0x020u
#define CSR_GP_CNTRL              0x024u
#define CSR_HW_REV                0x028u
#define CSR_EEPROM_REG            0x02Cu
#define CSR_EEPROM_GP             0x030u
#define CSR_OTP_GP_REG            0x034u
#define CSR_GIO_REG               0x03Cu
#define CSR_GP_UCODE_REG          0x048u
#define CSR_GP_DRIVER_REG         0x050u
#define CSR_UCODE_DRV_GP1         0x054u
#define CSR_UCODE_DRV_GP1_SET     0x058u
#define CSR_UCODE_DRV_GP1_CLR     0x05Cu
#define CSR_UCODE_DRV_GP2         0x060u
#define CSR_GIO_CHICKEN_BITS      0x100u
#define CSR_ANA_PLL_CFG           0x20Cu
#define CSR_HW_REV_WA_REG         0x22Cu
#define CSR_DBG_HPET_MEM_REG      0x240u
#define CSR_DBG_LINK_PWR_MGMT_REG 0x250u
#define CSR_MBOX_SET_REG          0x0B0u
#define CSR_LED_REG               0x094u

/* HW address window used by 22000/Qu (Linux iwl_22000_base.mac_addr_from_csr).
 * STRAP is preferred; OTP is the fallback if the strapped address is invalid. */
#define CSR_MAC_ADDR0_OTP         0x380u
#define CSR_MAC_ADDR1_OTP         0x384u
#define CSR_MAC_ADDR0_STRAP       0x388u
#define CSR_MAC_ADDR1_STRAP       0x38Cu

/* ── CSR_HW_IF_CONFIG_REG bits (from Linux iwl-csr.h) ──────────────────── */
#define CSR_HW_IF_CONFIG_REG_BIT_NIC_READY    (1u << 22)   /* 0x00400000 */
#define CSR_HW_IF_CONFIG_REG_BIT_PREPARE_DONE (1u << 25)   /* 0x02000000 */
#define CSR_HW_IF_CONFIG_REG_PREPARE          (1u << 27)   /* 0x08000000 */
#define CSR_HW_IF_CONFIG_REG_ENABLE_PME       (1u << 28)
#define CSR_HW_IF_CONFIG_REG_PERSIST_MODE     (1u << 30)
#define CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A (1u << 19)   /* 0x00080000 */

/* CSR_MBOX_SET_REG bits */
#define CSR_MBOX_SET_REG_OS_ALIVE             (1u << 5)

/* CSR_DBG_LINK_PWR_MGMT_REG value to disable link PM during probe */
#define CSR_RESET_LINK_PWR_MGMT_DISABLED      (1u << 31)

/* Linux gen2 APM workarounds. */
#define CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX 0x00800000u
#define CSR_DBG_HPET_MEM_REG_VAL                    0xFFFF0000u

/* UCODE-DRIVER GP1 handshake bits.  Linux clears RFKILL/CMD_BLOCKED before
 * every firmware start so stale ownership state cannot block host commands. */
#define CSR_UCODE_DRV_GP1_BIT_MAC_SLEEP             0x00000001u
#define CSR_UCODE_SW_BIT_RFKILL                     0x00000002u
#define CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED           0x00000004u

/* ── PRPH (indirect peripheral) access ────────────────────────────────── */
/* HBUS_BASE = 0x400, then offsets per Linux iwl-csr.h. */
#define HBUS_TARG_MEM_RADDR      0x40Cu   /* SRAM read address */
#define HBUS_TARG_MEM_RDAT       0x41Cu   /* SRAM read data */
#define HBUS_TARG_PRPH_WADDR     0x444u
#define HBUS_TARG_PRPH_RADDR     0x448u
#define HBUS_TARG_PRPH_WDAT      0x44Cu
#define HBUS_TARG_PRPH_RDAT      0x450u

/* PRPH addresses in the cookie's upper byte.  Linux uses (3 << 24).     */
#define PRPH_WR_ENABLE_BYTE3     (0x3u << 24)
/* PRPH address mask for Qu / 22000 family (20 bits — matches Linux).    */
#define PRPH_ADDR_MASK_QU        0x000FFFFFu

/* UREG — firmware load kick-off PRPH register (gen2).  Writing 1 here
 * after the context-info BA is latched tells the device to start pulling
 * firmware sections into internal RAM.                                  */
#define UREG_CPU_INIT_RUN        0xA05C44u

/* PRPH scratch register — Linux writes SCRATCH0 = 0 before ctxt info
 * load to clear any stale fw-load progress indicator.                  */
#define IWL_PRPH_SCRATCH0        0xA0EC00u

/* Firmware program-counter registers (Linux iwl-prph.h).               */
#define UREG_UMAC_CURRENT_PC     0xA05C18u
#define UREG_LMAC1_CURRENT_PC    0xA05C1Cu
#define UREG_LMAC2_CURRENT_PC    0xA05C20u

/* Post-ALIVE diagnostic registers. */
#define UREG_CHICK               0xA05C00u   /* status / control bits */
#define WFPM_GP2                 0xA030B4u   /* fw lifecycle status code */
#define UMAG_GEN_HW_STATUS       0xA038C8u   /* UMAC general status */

/* LTR (latency tolerance) registers — must be configured for integrated
 * Qu silicon before kicking firmware.  Values mirror Linux's
 * iwl_pcie_set_ltr(): enable all MAC LTRs and set UMAC LTR to ~250 usec. */
#define HPM_MAC_LTR_CSR          0xA0348Cu
#define HPM_MAC_LTR_ENABLE_ALL   0x0000000Fu
#define HPM_UMAC_LTR             0xA03480u
#define HPM_UMAC_LTR_DEFAULT_VAL 0x88FA88FAu

/* CSR_MAC_SHADOW_REG_CTRL — live readback on a working Linux session
 * shows 0x802FFFFF (bits 0..19 + 21 + 31).  Linux source uses iwl_set_bit
 * to OR 0x800FFFFF in; bit 21 is set elsewhere (or already by the chip).
 * Our plain w32 must include all observed bits. */
#define CSR_MAC_SHADOW_REG_CTRL_VAL 0x802FFFFFu

/* CSR_INT bits — Linux iwl-csr.h. */
#define CSR_INT_BIT_ALIVE        0x00000001u  /* (1<<0)  fw ALIVE notification */
#define CSR_INT_BIT_WAKEUP       0x00000002u  /* (1<<1)  uCode wakeup */
#define CSR_INT_BIT_RF_KILL      0x00000080u  /* (1<<7)  RF kill switch */
#define CSR_INT_BIT_SW_ERR       0x02000000u  /* (1<<25) software error */
#define CSR_INT_BIT_SCD          0x04000000u  /* (1<<26) scheduler interrupt */
#define CSR_INT_BIT_FH_TX        0x08000000u  /* (1<<27) TX FH (fw-load signal) */
#define CSR_INT_BIT_RX_PERIODIC  0x10000000u  /* (1<<28) RX periodic poll */
#define CSR_INT_BIT_HW_ERR       0x20000000u  /* (1<<29) hardware error */
#define CSR_INT_BIT_FH_RX        0x80000000u  /* (1<<31) RX FIFO ready */

/* Context-info firmware load uses iwl_enable_fw_load_int_ctx_info():
 * first ALIVE arrives as a CSR bit, then FH_RX delivers the RX ALIVE
 * notification after the driver restocks RFH.  The older streaming loader
 * uses FH_TX only, but AX201/22000 context-info does not. */
#define CSR_INT_MASK_FW_LOAD     (CSR_INT_BIT_ALIVE | CSR_INT_BIT_FH_RX)
#define CSR_INT_MASK_RUNTIME     (CSR_INT_BIT_FH_RX | CSR_INT_BIT_HW_ERR)

/* CSR_INT_COALESCING — sets the legacy interrupt coalescing timer (units
 * of 32 µs).  Value 0x40 = 64 = 2048 µs, the iwlwifi default.            */
#define CSR_INT_COALESCING_OFF   0x004u
#define CSR_INT_COALESCING_DEF   0x40u

/* ── RFH (RX Flow Handler) — PRPH addresses, queue 0 only. ─────────────
 * Linux drivers/net/wireless/intel/iwlwifi/iwl-fh.h.  These addresses
 * are needed by iwl_pcie_rx_mq_hw_init() and the chip uses them at
 * runtime to deliver received frames into our host RX ring.            */
#define RFH_RXF_DMA_CFG          0xA09820u
#define RFH_RXF_RXQ_ACTIVE       0xA0980Cu
#define RFH_GEN_CFG              0xA09800u
#define RFH_Q_FRBDCB_BA_LSB_Q0   0xA08000u  /* free RBD list base, 64-bit */
#define RFH_Q_URBDCB_BA_LSB_Q0   0xA08100u  /* used RBD list base, 64-bit */
#define RFH_Q_URBD_STTS_WPTR_Q0  0xA08200u  /* status writeback ptr, 64-bit */
#define RFH_Q_FRBDCB_WIDX_Q0     0xA08080u  /* free RBD write index */
#define RFH_Q_FRBDCB_RIDX_Q0     0xA080C0u  /* free RBD read  index */
#define RFH_Q_URBDCB_WIDX_Q0     0xA08180u  /* used RBD write index */

/* The free-RBD DOORBELL is at CSR (NOT PRPH) offset 0x1C80 — writing the
 * count of installed buffers (rounded down to multiple of 8) to this
 * register tells the chip RX is armed.                                  */
#define CSR_RFH_Q_FRBDCB_WIDX_TRG 0x1C80u

/* RFH_RXF_DMA_CFG composed value for 4K RB / min 4-8 RBs /
 * drop-too-large / DMA enabled.  Firmware programs RFH on AX201.       */
#define RFH_RXF_DMA_CFG_DEFAULT  0x87940000u

/* RFH_GEN_CFG for integrated Qu silicon: SERVICE_DMA_SNOOP +
 * RFH_DMA_SNOOP, default queue 0, 64-byte chunk size.                  */
#define RFH_GEN_CFG_DEFAULT      0x00000003u

/* RFH_RXF_RXQ_ACTIVE bitmask for "queue 0 active" only. */
#define RFH_RXF_RXQ_ACTIVE_Q0    0x00010001u

/* ── TX command-queue doorbell (CSR, shadow-register path) ─────────────
 * Live Linux on Surface Laptop 3 writes (write_ptr | (qid << 16)). */
#define HBUS_TARG_WRPTR          0x460u

/* TX TFD format constants (Linux iwl-fh.h). */
#define IWL_TFH_NUM_TBS          25      /* max TBs per gen2 TFD */
#define IWL_FIRST_TB_SIZE        20      /* max bytes copied to first_tb_buf */
#define IWL_CMD_QUEUE_SIZE       32      /* TFD ring entries — fixed */

/* Linux's TFD_QUEUE_CB_SIZE(x) = ilog2(x) - 3.  For 32 entries → 2. */
#define IWL_CMD_QUEUE_SIZE_LOG   2

/* Sequence-number layout in iwl_cmd_header_wide.sequence (16 bits):
 *   bits[ 7:0]  = TFD write_ptr (raw, mod 256)
 *   bits[12:8]  = TX queue id   (0 for cmd queue)
 *   bit 15       = SEQ_RX_FRAME (set by uCode on unsolicited frames)  */
#define IWL_SEQ_RX_FRAME         0x8000u

/* iwlwifi command-group ids (Linux fw/api/commands.h). */
#define IWL_GROUP_LEGACY                  0x00
#define IWL_GROUP_LONG                    0x01
#define IWL_GROUP_SYSTEM                  0x02
#define IWL_GROUP_MAC_CONF                0x03   /* MLD: MAC_CONFIG, LINK_CONFIG, STA_CONFIG */
#define IWL_GROUP_PHY_OPS                 0x04
#define IWL_GROUP_DATA_PATH               0x05
#define IWL_GROUP_REGULATORY_AND_NVM      0x0C
#define IWL_GROUP_DEBUG                   0x0F

/* Specific command ids. */
#define IWL_CMD_ECHO                      0x03   /* group 0, no payload */
#define IWL_CMD_INIT_COMPLETE_NOTIF       0x04   /* group 0, fw-emitted */
#define IWL_CMD_PHY_CONFIGURATION         0x6A   /* group 0 */
#define IWL_CMD_TX_ANT_CONFIGURATION      0x98   /* group 0 */
#define IWL_CMD_BT_CONFIG                 0x9B   /* group 0/1 per TLV */
#define IWL_CMD_REPLY_SF_CFG              0xD1   /* group 0 */
#define IWL_CMD_SHARED_MEM_CFG            0x00   /* group SYSTEM */
#define IWL_CMD_INIT_EXTENDED_CFG         0x03   /* group SYSTEM */
#define IWL_CMD_NVM_ACCESS                0x88   /* group 0, cache/OTP read */
#define IWL_CMD_NVM_ACCESS_COMPLETE       0x00   /* group REG_AND_NVM */
#define IWL_CMD_NVM_GET_INFO              0x02   /* group REG_AND_NVM */
#define IWL_CMD_DQA_ENABLE                0x00   /* group DATA_PATH */
#define IWL_CMD_SCD_QUEUE_CFG             0x1D   /* group LEGACY (gen2 path) */
#define IWL_CMD_RLC_CONFIG                0x08   /* group DATA_PATH — MLD prerequisite */
#define IWL_CMD_TLC_MNG_CONFIG            0x0F   /* group DATA_PATH — FW rate-control cfg */
#define IWL_CMD_TLC_MNG_UPDATE_NOTIF      0xF7   /* group DATA_PATH */
#define IWL_CMD_RX_NO_DATA_NOTIF          0xF5   /* group DATA_PATH */
#define IWL_CMD_SCD_QUEUE_CONFIG          0x17   /* group DATA_PATH (gen3 path, AX210+) */
#define IWL_CMD_SEC_KEY                   0x18   /* group DATA_PATH — MLD key install */
#define IWL_CMD_INVALID_WR_PTR            0x06   /* group DEBUG */

/* TX_QUEUE_CFG_CMD flags (Linux fw/api/txq.h:69) — legacy v0 path only.
 * The v3 path (used by Qu firmware) does NOT set this bit; flags = 0. */
#define TX_QUEUE_CFG_ENABLE_QUEUE         (1u << 0)

/* SCD_QUEUE_CONFIG_CMD v3 operation values (Linux fw/api/datapath.h:727-733). */
#define IWL_SCD_QUEUE_ADD                 0
#define IWL_SCD_QUEUE_REMOVE              1
#define IWL_SCD_QUEUE_MODIFY              2

/* ADD_STA station_type values (Linux fw/api/sta.h enum iwl_sta_type). */
#define IWL_STA_LINK                      0
#define IWL_STA_GENERAL_PURPOSE           1
#define IWL_STA_MULTICAST                 2
#define IWL_STA_TDLS_LINK                 3
#define IWL_STA_AUX_ACTIVITY              4

/* ADD_STA response status (Linux fw/api/sta.h). */
#define IWL_ADD_STA_STATUS_MASK           0xFFu
#define IWL_ADD_STA_SUCCESS               1
#define IWL_MVM_INVALID_STA               0xFF

/* TVQM queue sizes (Linux fw/api/txq.h): management queues are 16 entries,
 * station data queues are 256 entries. cb_size is log2(size)-3. */
#define IWL_MGMT_QUEUE_SIZE               16
#define IWL_MGMT_QUEUE_CB_SIZE            1
#define IWL_DATA_QUEUE_SIZE               256
#define IWL_DATA_QUEUE_CB_SIZE            5
/* Linux uses IWL_MAX_TID_COUNT (8) for internal/AUX queues, IWL_MGMT_TID (15)
 * for AP management TX, and TID 0 for non-QoS station data. */
#define IWL_NON_QOS_QUEUE_TID             0
#define IWL_INTERNAL_QUEUE_TID            8
#define IWL_MGMT_QUEUE_TID                15
/* Byte-count table: 1024 entries × __le16 = 2048 bytes (gen2 iwlagn_scd_bc_tbl). */
#define IWL_BC_TBL_SIZE                   1024
#define IWL_CMD_PHY_CONTEXT               0x08   /* group LONG */
#define IWL_CMD_ADD_STA                   0x18   /* group LONG (legacy STA table) */
#define IWL_CMD_BINDING                   0x2B   /* group LONG (legacy MAC/PHY binding) */
#define IWL_CMD_MAC_CONTEXT               0x28   /* group LONG (legacy MAC context) */
#define IWL_CMD_TX_CMD                    0x1C   /* group LEGACY — TX response */
#define IWL_CMD_REPLY_RX_MPDU             0xC1   /* group LEGACY — RX MPDU notification */
#define IWL_CMD_MAC_PM_POWER_TABLE        0xA9   /* group LONG — per-MAC power cfg */

/* MLD command set (group MAC_CONF = 0x03).  Linux uses these for the AX201
 * association path on the Surface Laptop 3. */
#define IWL_CMD_SESSION_PROTECTION        0x05   /* group MAC_CONF */
#define IWL_CMD_MAC_CONFIG                0x08   /* group MAC_CONF — replaces MAC_CONTEXT */
#define IWL_CMD_LINK_CONFIG               0x09   /* group MAC_CONF — replaces BINDING */
#define IWL_CMD_STA_CONFIG                0x0A   /* group MAC_CONF — replaces ADD_STA */
#define IWL_CMD_AUX_STA                   0x0B   /* group MAC_CONF — MLD aux STA */
#define IWL_CMD_SESSION_PROT_NOTIF        0xFB   /* group MAC_CONF — notif */

/* MLD STA station_type — enum iwl_fw_sta_type in mvm/sta.h.  NOTE: this is
 * a SEPARATE enum from the legacy IWL_STA_* values (which were for ADD_STA);
 * for MLD's STA_CONFIG_CMD use these. */
#define IWL_FW_STA_TYPE_PEER              0   /* AP STA / peer STA */
#define IWL_FW_STA_TYPE_BCAST_MGMT        1
#define IWL_FW_STA_TYPE_MCAST             2
#define IWL_FW_STA_TYPE_AUX               3   /* aux scan STA (MLD) */

/* MLD MAC filter flags (enum iwl_mac_config_filter_flags). */
#define MAC_CFG_FILTER_PROMISC                  (1u << 0)
#define MAC_CFG_FILTER_ACCEPT_CONTROL_AND_MGMT  (1u << 1)
#define MAC_CFG_FILTER_ACCEPT_GRP               (1u << 2)
#define MAC_CFG_FILTER_ACCEPT_BEACON            (1u << 3)
#define MAC_CFG_FILTER_ACCEPT_BCAST_PROBE_RESP  (1u << 4)
#define MAC_CFG_FILTER_ACCEPT_PROBE_REQ         (1u << 5)

/* MLD LINK_CONFIG_CMD modify_mask bits (enum iwl_link_ctx_modify_flags). */
#define LINK_CONTEXT_MODIFY_ACTIVE              (1u << 0)
#define LINK_CONTEXT_MODIFY_RATES_INFO          (1u << 1)
#define LINK_CONTEXT_MODIFY_PROTECT_FLAGS       (1u << 2)
#define LINK_CONTEXT_MODIFY_QOS_PARAMS          (1u << 3)
#define LINK_CONTEXT_MODIFY_BEACON_TIMING       (1u << 4)
#define LINK_CONTEXT_MODIFY_HE_PARAMS           (1u << 5)
#define LINK_CONTEXT_MODIFY_BSS_COLOR_DISABLE   (1u << 6)
#define LINK_CONTEXT_MODIFY_EHT_PARAMS          (1u << 7)
#define LINK_CONTEXT_MODIFY_BANDWIDTH           (1u << 8)

/* SESSION_PROTECTION conf_id values (enum iwl_session_prot_conf_id). */
#define IWL_SESSION_PROT_CONF_ASSOC             0
#define IWL_SESSION_PROT_CONF_GO_CLIENT_ASSOC   1
#define IWL_SESSION_PROT_CONF_P2P_DEVICE_DISCOV 2
#define IWL_SESSION_PROT_CONF_P2P_GO_NEGOTIATION 3

/* iwl_tx_cmd_flags (Linux fw/api/tx.h:99-106). */
#define IWL_TX_FLAGS_CMD_RATE             (1u << 0)
#define IWL_TX_FLAGS_ENCRYPT_DIS          (1u << 1)
#define IWL_TX_FLAGS_HIGH_PRI             (1u << 2)

/* TX_CMD offload_assist fields. */
#define IWL_TX_CMD_OFFLD_MH_SIZE          8
#define IWL_TX_CMD_OFFLD_PAD              13

/* rate_n_flags fields used by Qu/AX201 TX_CMD v9.  Linux chooses v1/v2/v3
 * from firmware command versions; AX201's -77 firmware advertises v2. */
#define IWL_RATE_LEGACY_OFDM_6M_PLCP      0x0Du  /* v1 low bits */
#define IWL_RATE_LEGACY_OFDM_6M_IDX       0x00u  /* v2/v3 low bits */
#define IWL_RATE_MCS_MOD_TYPE_LEGACY_OFDM (1u << 8)
#define IWL_RATE_MCS_ANT_POS              14
#define IWL_RATE_MCS_ANT_A_MSK            (1u << 14)
#define IWL_RATE_MCS_ANT_B_MSK            (2u << 14)

/* 802.11 capability_info bits used in Association Request frames. */
#define WLAN_CAPABILITY_ESS               (1u << 0)
#define WLAN_CAPABILITY_PRIVACY           (1u << 4)
#define WLAN_CAPABILITY_SHORT_PREAMBLE    (1u << 5)
#define WLAN_CAPABILITY_SHORT_SLOT_TIME   (1u << 10)

/* 802.11 information element ids used by scan/association. */
#define WLAN_EID_SSID                     0
#define WLAN_EID_SUPP_RATES               1
#define WLAN_EID_RSN                      48
#define WLAN_EID_EXT_SUPP_RATES           50
#define WLAN_EID_VENDOR_SPECIFIC          221

/* TX_CMD response status (Linux enum iwl_tx_status). */
#define IWL_TX_STATUS_MSK                 0x00FFu
#define IWL_TX_STATUS_SUCCESS             0x0001u
#define IWL_TX_STATUS_DIRECT_DONE         0x0002u

/* BIND_CMD invalid-MAC sentinel (Linux fw/api/binding.h FW_CTXT_INVALID). */
#define IWL_FW_CTXT_INVALID               0xFFFFFFFFu
#define IWL_CMD_SCAN_CFG                  0x0C   /* group LONG — REQUIRED before SCAN_REQ_UMAC */
#define IWL_CMD_SCAN_REQ_UMAC             0x0D   /* group LONG */
#define IWL_CMD_SCAN_COMPLETE_UMAC        0x0F   /* group LONG */
#define IWL_CMD_SCAN_ITER_COMPLETE_UMAC   0xB5   /* group LONG */

/* MAC context types (Linux fw/api/mac.h:63-75). */
#define FW_MAC_TYPE_AUX                   0x01
#define FW_MAC_TYPE_LISTENER              0x02
#define FW_MAC_TYPE_PIBSS                 0x03
#define FW_MAC_TYPE_IBSS                  0x04
#define FW_MAC_TYPE_BSS_STA               0x05

/* MAC filter flags (Linux fw/api/mac.h iwl_mac_filter_flags enum). */
#define MAC_FILTER_IN_PROMISC             (1u <<  0)
#define MAC_FILTER_IN_CONTROL_AND_MGMT    (1u <<  1)
#define MAC_FILTER_ACCEPT_GRP             (1u <<  2)
#define MAC_FILTER_IN_BEACON              (1u <<  6)
#define MAC_FILTER_OUT_BCAST              (1u <<  8)
#define MAC_FILTER_IN_CRC32               (1u << 11)
#define MAC_FILTER_IN_PROBE_REQUEST       (1u << 12)

/* MAC context short-slot / short-preamble fields are flag words, not bools. */
#define MAC_FLG_SHORT_SLOT                (1u << 4)
#define MAC_FLG_SHORT_PREAMBLE            (1u << 5)

/* 802.11 capability_info bits used when targeting an AP. */
#define WLAN_CAPABILITY_SHORT_PREAMBLE    (1u << 5)
#define WLAN_CAPABILITY_SHORT_SLOT_TIME   (1u << 10)
/* Listener-mode value matches Linux mac-ctxt.c:738-743:
 *   PROMISC | CONTROL_AND_MGMT | ACCEPT_GRP | BEACON | CRC32 | PROBE_REQUEST
 *   = 0x0001 | 0x0002 | 0x0004 | 0x0040 | 0x0800 | 0x1000 = 0x1847 */
#define MAC_FILTER_LISTENER \
    (MAC_FILTER_IN_PROMISC | MAC_FILTER_IN_CONTROL_AND_MGMT | \
     MAC_FILTER_ACCEPT_GRP | MAC_FILTER_IN_BEACON           | \
     MAC_FILTER_IN_CRC32   | MAC_FILTER_IN_PROBE_REQUEST)

/* PHY band / channel-width / context-action constants (Linux fw/api/phy-ctxt.h
 * + fw/api/context.h). */
#define IWL_PHY_BAND_24                   0x01
#define IWL_PHY_BAND_5                    0x00
#define IWL_PHY_CHANNEL_MODE20            0x00
#define IWL_PHY_CHANNEL_MODE40            0x01
#define IWL_PHY_CHANNEL_MODE80            0x02
#define IWL_PHY_CHANNEL_MODE160           0x03
#define FW_CTXT_ACTION_ADD                0x01
#define FW_CTXT_ACTION_MODIFY             0x02
#define FW_CTXT_ACTION_REMOVE             0x03

/* Build id_and_color for context commands.  Linux fw/api/context.h:
 *   FW_CMD_ID_AND_COLOR(id, color) = id | (color << 8) */
#define IWL_FW_ID_AND_COLOR(id, color)    ((uint32_t)((id) | ((color) << 8)))

/* Linux iwl_mvm_phy_ctxt_set_rxchain encoding for valid_rx_ant / idle / active.
 *   bits 1..3   = valid_rx_ant
 *   bits 10..11 = idle_cnt
 *   bits 12..13 = active_cnt */
#define IWL_RXCHAIN_VAL_BIT(ant)          ((uint32_t)(ant) << 1)
#define IWL_RXCHAIN_IDLE(n)               ((uint32_t)(n)   << 10)
#define IWL_RXCHAIN_ACTIVE(n)             ((uint32_t)(n)   << 12)

/* ── CSR_RESET bits ────────────────────────────────────────────────────── */
#define CSR_RESET_REG_FLAG_NEVO_RESET      (1u << 0)
#define CSR_RESET_REG_FLAG_FORCE_NMI       (1u << 1)
#define CSR_RESET_REG_FLAG_SW_RESET        (1u << 7)
#define CSR_RESET_REG_FLAG_MASTER_DISABLED (1u << 8)
#define CSR_RESET_REG_FLAG_STOP_MASTER     (1u << 9)
#define CSR_RESET_LINK_PWR_MGMT_DISABLED   (1u << 31)

/* ── CSR_GP_CNTRL bits ─────────────────────────────────────────────────── */
/* These are the same on 7000/8000/9000/22000 silicon (Qu/AX201 included).
 * The "GEN2" bit relocations only apply to the Bz family (post-22000); for
 * Qu we use the legacy positions.  Confirmed against Linux iwl-csr.h.   */
#define CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY    0x00000001u
#define CSR_GP_CNTRL_REG_FLAG_INIT_DONE          0x00000004u
#define CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ     0x00000008u
#define CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP     0x00000010u
#define CSR_GP_CNTRL_REG_FLAG_XTAL_ON            0x00000400u
#define CSR_GP_CNTRL_REG_FLAG_MAC_POWER_SAVE     0x01000000u
#define CSR_GP_CNTRL_REG_FLAG_HW_RF_KILL_SW      0x08000000u   /* 1 = radio on */

/* ── CSR_HW_REV decoding ───────────────────────────────────────────────── */
/* HW_REV layout (Linux iwl-csr.h):
 *   bits[3:0]  = stepping/dash (chip revision)
 *   bits[15:4] = MAC type (shifted right 4 = IWL_CFG_MAC_TYPE_* constant)
 *   bits[31:16]= reserved / MAC address bits on older silicon
 * e.g. raw 0x332 → type = (0x332 >> 4) & 0xFFF = 0x33 = MAC_TYPE_QU      */
#define CSR_HW_REV_TYPE_SHIFT    4
#define CSR_HW_REV_TYPE_MASK     0xFFFu        /* applied after shift */
#define CSR_HW_REV_STEP_MASK     0x0Fu         /* applied to raw rev */

/* iwl_cfg_mac_type_ids — decoded HW family (Linux IWL_CFG_MAC_TYPE_*). */
#define IWL_HW_REV_TYPE_PU       0x31
#define IWL_HW_REV_TYPE_TH       0x32
#define IWL_HW_REV_TYPE_QU       0x33   /* AX201 on Surface Laptop 3 (rev 0x332) */
#define IWL_HW_REV_TYPE_QUZ      0x35
#define IWL_HW_REV_TYPE_QNJ      0x36
#define IWL_HW_REV_TYPE_SNJ      0x42
#define IWL_HW_REV_TYPE_SO       0x43

/* ── OTP / NVM (for reading MAC address) ───────────────────────────────── */
#define CSR_OTP_GP_REG_DEVICE_SELECT         (1u << 16)
#define CSR_OTP_GP_REG_OTP_ACCESS_MODE       (1u << 17)
#define CSR_OTP_GP_REG_ECC_CORR_STATUS_MSK   (1u << 20)
#define CSR_OTP_GP_REG_ECC_UNCORR_STATUS_MSK (1u << 21)

#define CSR_EEPROM_REG_READ_VALID_MSK        0x00000001u
#define CSR_EEPROM_REG_MSK_ADDR              0x0000FFFCu
#define CSR_EEPROM_REG_BIT_CMD               (1u << 1)
