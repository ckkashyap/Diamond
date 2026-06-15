/*
 * iwl_fw.h - Intel Wi-Fi firmware TLV (Type-Length-Value) format.
 *
 * Subset of Linux's drivers/net/wireless/intel/iwlwifi/fw/file.h describing
 * the on-disk .ucode layout for 9000-/22000-series silicon.  The firmware
 * blob from linux-firmware (e.g. iwlwifi-Qu-c0-hr-b0-77.ucode) is a
 * zero-prefixed header followed by a stream of TLV records, each carrying
 * either a firmware image section, a metadata field, or a capability flag.
 *
 * Layout on disk:
 *   [4 bytes]  zero
 *   [4 bytes]  magic 0x0a4c5749 ("IWL\n")
 *   [64 bytes] ASCII human-readable version (e.g. "release/core74::b4...")
 *   [4 bytes]  legacy version (ignored)
 *   [4 bytes]  legacy build (ignored)
 *   [8 bytes]  ignored
 *   then a stream of TLVs:
 *     struct iwl_ucode_tlv {
 *         __le32 type;     // TYPE_*
 *         __le32 length;   // of `data` below
 *         u8 data[length]; // padded up to 4-byte alignment
 *     }
 *
 * Section TLVs (TYPE_SEC_RT / TYPE_SEC_INIT / TYPE_SEC_WOWLAN) carry a
 * 4-byte load-address followed by the binary section data.  Sections
 * addressed >= 0x45000000 are CPU2 (WoWLAN CPU) data.
 */
#pragma once
#include <stdint.h>

#define IWL_TLV_UCODE_MAGIC   0x0a4c5749u      /* "IWL\n" */

struct iwl_tlv_ucode_header {
    uint32_t zero;
    uint32_t magic;
    char     human_readable[64];
    uint32_t ver;        /* legacy, unused */
    uint32_t build;      /* legacy, unused */
    uint64_t ignore;
    /* TLV records follow */
} __attribute__((packed));

struct iwl_ucode_tlv {
    uint32_t type;
    uint32_t length;
    /* u8 data[length]; — padded up to 4 bytes */
} __attribute__((packed));

/* ── TLV types we actually care about ──────────────────────────────────── */
/* Legacy "one big blob" sections (pre-9000) */
#define IWL_UCODE_TLV_INST                1
#define IWL_UCODE_TLV_DATA                2
#define IWL_UCODE_TLV_INIT                3
#define IWL_UCODE_TLV_INIT_DATA           4
#define IWL_UCODE_TLV_BOOT                5

/* Modern per-section records (9000+) — this is what Qu/QuZ firmware uses. */
#define IWL_UCODE_TLV_SEC_RT              19   /* Regular (runtime) */
#define IWL_UCODE_TLV_SEC_INIT            20   /* Init */
#define IWL_UCODE_TLV_SEC_WOWLAN          21   /* WoWLAN */

/* Separator placed between section TLVs of different firmware images */
#define IWL_UCODE_TLV_CMD_VERSIONS        48
#define IWL_UCODE_TLV_FW_RECOVERY_INFO    57
#define IWL_UCODE_TLV_FW_FSEQ_VERSION     60

/* Capability / flags records (metadata, not binary data) */
#define IWL_UCODE_TLV_API_CHANGES_SET     29
#define IWL_UCODE_TLV_ENABLED_CAPABILITIES 30
#define IWL_UCODE_TLV_FLAGS               18
#define IWL_UCODE_TLV_DEF_CALIB           22
#define IWL_UCODE_TLV_PHY_SKU             23
#define IWL_UCODE_TLV_N_SCAN_CHANNELS     31
#define IWL_UCODE_TLV_FW_VERSION          36
#define IWL_UCODE_TLV_FW_MEM_SEG          51

/* Image boundaries / metadata */
#define IWL_UCODE_TLV_PAGING              32
#define IWL_UCODE_TLV_SEC_RT_USNIFFER     50
#define IWL_UCODE_TLV_NUM_OF_CPU          27
#define IWL_UCODE_TLV_CSCHEME             28

/* iwl_fw image slots — we partition TLVs into these three during parsing. */
enum iwl_fw_image {
    IWL_FW_IMG_INIT    = 0,
    IWL_FW_IMG_REGULAR = 1,
    IWL_FW_IMG_WOWLAN  = 2,
    IWL_FW_IMG_COUNT   = 3,
};

/* AX201 REGULAR image has 51 SEC_RT records (including the
 * CPU1/CPU2 separator marker).  128 gives headroom for newer firmware. */
#define IWL_FW_MAX_SECTIONS  128

struct iwl_fw_section {
    uint32_t    addr;          /* CPU1 DRAM target address */
    uint32_t    len;           /* bytes */
    const void *data;          /* pointer inside the embedded blob */
};

struct iwl_fw_image_info {
    struct iwl_fw_section sections[IWL_FW_MAX_SECTIONS];
    uint8_t               n_sections;
};

struct iwl_fw_calib_ctrl {
    uint32_t flow_trigger;
    uint32_t event_trigger;
};

/* IWL_UCODE_TLV_CMD_VERSIONS entry — Linux fw/api/dbg-tlv.h
 * iwl_fw_cmd_version: 4 bytes per entry. */
struct iwl_fw_cmd_version_entry {
    uint8_t cmd;
    uint8_t group;
    uint8_t cmd_ver;
    uint8_t notif_ver;
} __attribute__((packed));

#define IWL_FW_MAX_CMD_VERSIONS  256u

struct iwl_fw_info {
    int                      present;     /* 1 if firmware blob was embedded */
    const void              *raw;         /* start of embedded blob */
    uint32_t                 raw_size;    /* total bytes */
    char                     human[65];   /* NUL-terminated version string */
    uint32_t                 fw_version;  /* IWL_UCODE_TLV_FW_VERSION if present */
    uint8_t                  num_cpus;    /* 1 or 2 */
    uint32_t                 phy_config;  /* IWL_UCODE_TLV_PHY_SKU */
    uint8_t                  valid_tx_ant;
    uint8_t                  valid_rx_ant;
    struct iwl_fw_calib_ctrl default_calib[IWL_FW_IMG_COUNT];
    struct iwl_fw_image_info img[IWL_FW_IMG_COUNT];

    /* Parsed cmd-version table (TLV type 48). */
    struct iwl_fw_cmd_version_entry cmd_vers[IWL_FW_MAX_CMD_VERSIONS];
    uint16_t                        n_cmd_vers;
};

/* Parse the embedded firmware into `info`.  Returns 1 on success, 0 if the
 * blob is absent or has a bad magic.  The returned pointers in
 * `info->img[x].sections[y].data` reference bytes inside the embedded
 * section directly — do NOT free them.                                    */
int iwl_fw_parse(struct iwl_fw_info *info);

/* Look up the firmware-advertised command version for (group, cmd).
 * Matches Linux's lookup rule: legacy command IDs are versioned in LONG_GROUP.
 * Returns the cmd_ver byte, or 0xFF if not present in the TLV table. */
uint8_t iwl_fw_cmd_ver(const struct iwl_fw_info *info,
                       uint8_t group, uint8_t cmd);

/* Look up the firmware-advertised notification version for (group, cmd).
 * Returns the notif_ver byte, or 0xFF if not present in the TLV table. */
uint8_t iwl_fw_notif_ver(const struct iwl_fw_info *info,
                         uint8_t group, uint8_t cmd);

/* Linux-derived rate_n_flags API selector:
 *   1 = legacy PLCP encoding, 2/3 = legacy rate-index encoding.
 * Returns 0 if the firmware advertises an inconsistent partial transition. */
uint8_t iwl_fw_rates_ver(const struct iwl_fw_info *info);
