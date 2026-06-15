/*
 * iwl_fw.c - Parse the iwlwifi TLV firmware blob supplied by Limine as a
 * module (cmdline "iwlwifi").  See iwl_fw.h for the on-disk layout.
 *
 * The firmware file is too large (~1.4 MB) to embed in the kernel ELF
 * without tripping boot-loader size limits, so it lives alongside
 * kernel.elf on the boot medium and Limine loads it with a protocol-level
 * "module" entry.  The kernel walks the module list at boot, finds the
 * "iwlwifi" cmdline tag, and points this parser at the bytes.
 *
 * If no such module is loaded the build still succeeds and the driver just
 * reports "no firmware" at boot.
 */

#include <stdint.h>
#include "iwl_fw.h"

#define IWL_FW_CMD_VER_UNKNOWN  99u

/* Private copy of the small command IDs needed to mirror Linux's
 * fw_rates_ver selection without coupling the TLV parser to iwl_csr.h. */
#define FW_GROUP_LEGACY             0x00u
#define FW_GROUP_LONG               0x01u
#define FW_GROUP_DATA_PATH          0x05u
#define FW_CMD_TX                   0x1Cu
#define FW_CMD_REPLY_RX_MPDU        0xC1u
#define FW_CMD_RX_NO_DATA_NOTIF     0xF5u
#define FW_CMD_TLC_MNG_UPDATE_NOTIF 0xF7u

/* Provided by kernel/main.c */
extern int limine_find_module(const char *cmdline_match,
                              const void **data_out, uint64_t *size_out);

static inline uint32_t le32(const uint8_t *p) {
    return ((uint32_t)p[0])       |
           ((uint32_t)p[1] <<  8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline uint32_t roundup4(uint32_t n) { return (n + 3u) & ~3u; }

static void record_section(struct iwl_fw_info *info,
                           enum iwl_fw_image slot,
                           const uint8_t *sec_data, uint32_t sec_len) {
    if (sec_len < 4) return;
    struct iwl_fw_image_info *img = &info->img[slot];
    if (img->n_sections >= IWL_FW_MAX_SECTIONS) return;

    struct iwl_fw_section *s = &img->sections[img->n_sections++];
    s->addr = le32(sec_data);
    s->len  = sec_len - 4u;
    s->data = sec_data + 4;
}

int iwl_fw_parse(struct iwl_fw_info *info) {
    for (int i = 0; i < IWL_FW_IMG_COUNT; i++) info->img[i].n_sections = 0;
    for (int i = 0; i < IWL_FW_IMG_COUNT; i++) {
        info->default_calib[i].flow_trigger = 0;
        info->default_calib[i].event_trigger = 0;
    }
    info->present    = 0;
    info->raw        = 0;
    info->raw_size   = 0;
    info->human[0]   = 0;
    info->fw_version = 0;
    info->num_cpus   = 1;
    info->phy_config = 0;
    info->valid_tx_ant = 0;
    info->valid_rx_ant = 0;
    info->n_cmd_vers = 0;

    const void *blob = 0;
    uint64_t blob_size = 0;
    if (!limine_find_module("iwlwifi", &blob, &blob_size)) return 0;
    if (blob_size < sizeof(struct iwl_tlv_ucode_header))   return 0;

    info->raw      = blob;
    info->raw_size = (uint32_t)blob_size;

    const uint8_t *start = (const uint8_t *)blob;
    const uint8_t *end   = start + blob_size;

    const struct iwl_tlv_ucode_header *hdr = (const void *)start;
    if (le32((const uint8_t *)&hdr->zero)  != 0) return 0;
    if (le32((const uint8_t *)&hdr->magic) != IWL_TLV_UCODE_MAGIC) return 0;

    for (int i = 0; i < 64; i++) info->human[i] = hdr->human_readable[i];
    info->human[64] = 0;

    const uint8_t *p = start + sizeof(struct iwl_tlv_ucode_header);

    while (p + 8 <= end) {
        uint32_t type = le32(p);
        uint32_t len  = le32(p + 4);
        const uint8_t *data = p + 8;
        uint32_t step = 8u + roundup4(len);

        if (data + len > end) break;

        switch (type) {
        case IWL_UCODE_TLV_SEC_INIT:
            record_section(info, IWL_FW_IMG_INIT,    data, len); break;
        case IWL_UCODE_TLV_SEC_RT:
            record_section(info, IWL_FW_IMG_REGULAR, data, len); break;
        case IWL_UCODE_TLV_SEC_WOWLAN:
            record_section(info, IWL_FW_IMG_WOWLAN,  data, len); break;

        case IWL_UCODE_TLV_INST:
        case IWL_UCODE_TLV_DATA:
            if (info->img[IWL_FW_IMG_REGULAR].n_sections < IWL_FW_MAX_SECTIONS) {
                struct iwl_fw_section *s =
                    &info->img[IWL_FW_IMG_REGULAR]
                    .sections[info->img[IWL_FW_IMG_REGULAR].n_sections++];
                s->addr = 0; s->len = len; s->data = data;
            }
            break;
        case IWL_UCODE_TLV_INIT:
        case IWL_UCODE_TLV_INIT_DATA:
            if (info->img[IWL_FW_IMG_INIT].n_sections < IWL_FW_MAX_SECTIONS) {
                struct iwl_fw_section *s =
                    &info->img[IWL_FW_IMG_INIT]
                    .sections[info->img[IWL_FW_IMG_INIT].n_sections++];
                s->addr = 0; s->len = len; s->data = data;
            }
            break;

        case IWL_UCODE_TLV_FW_VERSION:
            if (len >= 4) info->fw_version = le32(data);
            break;
        case IWL_UCODE_TLV_DEF_CALIB:
            if (len >= 12) {
                uint32_t linux_type = le32(data);
                enum iwl_fw_image slot = IWL_FW_IMG_COUNT;
                if (linux_type == 0) slot = IWL_FW_IMG_REGULAR;
                else if (linux_type == 1) slot = IWL_FW_IMG_INIT;
                else if (linux_type == 2) slot = IWL_FW_IMG_WOWLAN;
                if (slot < IWL_FW_IMG_COUNT) {
                    info->default_calib[slot].flow_trigger = le32(data + 4);
                    info->default_calib[slot].event_trigger = le32(data + 8);
                }
            }
            break;
        case IWL_UCODE_TLV_PHY_SKU:
            if (len >= 4) {
                info->phy_config = le32(data);
                info->valid_tx_ant = (uint8_t)((info->phy_config >> 16) & 0xFu);
                info->valid_rx_ant = (uint8_t)((info->phy_config >> 20) & 0xFu);
            }
            break;
        case IWL_UCODE_TLV_NUM_OF_CPU:
            if (len >= 4) info->num_cpus = (uint8_t)le32(data);
            break;
        case IWL_UCODE_TLV_CMD_VERSIONS: {
            /* Array of struct iwl_fw_cmd_version_entry (4 bytes each).
             * Each tells the (cmd_ver, notif_ver) for a (group, cmd) pair.
             * We parse all entries up to our local table capacity. */
            uint32_t n = len / 4;
            if (n > IWL_FW_MAX_CMD_VERSIONS - info->n_cmd_vers)
                n = IWL_FW_MAX_CMD_VERSIONS - info->n_cmd_vers;
            for (uint32_t i = 0; i < n; i++) {
                const uint8_t *e = data + i * 4;
                info->cmd_vers[info->n_cmd_vers].cmd       = e[0];
                info->cmd_vers[info->n_cmd_vers].group     = e[1];
                info->cmd_vers[info->n_cmd_vers].cmd_ver   = e[2];
                info->cmd_vers[info->n_cmd_vers].notif_ver = e[3];
                info->n_cmd_vers++;
            }
            break;
        }
        default:
            break;
        }

        p += step;
    }

    info->present = 1;
    return 1;
}

static uint8_t lookup_cmd_ver(const struct iwl_fw_info *info,
                              uint8_t group, uint8_t cmd,
                              uint8_t def) {
    if (!info || !info->present) return def;
    uint8_t lookup_group = group ? group : FW_GROUP_LONG;
    for (uint16_t i = 0; i < info->n_cmd_vers; i++) {
        if (info->cmd_vers[i].group == lookup_group &&
            info->cmd_vers[i].cmd   == cmd) {
            uint8_t ver = info->cmd_vers[i].cmd_ver;
            return (ver == IWL_FW_CMD_VER_UNKNOWN) ? def : ver;
        }
    }
    return def;
}

static uint8_t lookup_notif_ver(const struct iwl_fw_info *info,
                                uint8_t group, uint8_t cmd,
                                uint8_t def) {
    if (!info || !info->present) return def;
    for (uint16_t i = 0; i < info->n_cmd_vers; i++) {
        if (info->cmd_vers[i].group == group &&
            info->cmd_vers[i].cmd   == cmd) {
            uint8_t ver = info->cmd_vers[i].notif_ver;
            return (ver == IWL_FW_CMD_VER_UNKNOWN) ? def : ver;
        }
    }
    return def;
}

uint8_t iwl_fw_cmd_ver(const struct iwl_fw_info *info,
                       uint8_t group, uint8_t cmd) {
    return lookup_cmd_ver(info, group, cmd, 0xFFu);
}

uint8_t iwl_fw_notif_ver(const struct iwl_fw_info *info,
                         uint8_t group, uint8_t cmd) {
    return lookup_notif_ver(info, group, cmd, 0xFFu);
}

uint8_t iwl_fw_rates_ver(const struct iwl_fw_info *info) {
    uint8_t rates_ver = 1;

    uint8_t ratecheck =
        (lookup_cmd_ver(info, FW_GROUP_LEGACY, FW_CMD_TX, 0) >= 8) +
        (lookup_notif_ver(info, FW_GROUP_DATA_PATH,
                          FW_CMD_TLC_MNG_UPDATE_NOTIF, 0) >= 3) +
        (lookup_notif_ver(info, FW_GROUP_LEGACY,
                          FW_CMD_REPLY_RX_MPDU, 0) >= 4) +
        (lookup_notif_ver(info, FW_GROUP_LONG, FW_CMD_TX, 0) >= 6);
    if (ratecheck != 0 && ratecheck != 4) return 0;
    if (ratecheck == 4) rates_ver = 2;

    ratecheck =
        (lookup_cmd_ver(info, FW_GROUP_LEGACY, FW_CMD_TX, 0) >= 11) +
        (lookup_notif_ver(info, FW_GROUP_DATA_PATH,
                          FW_CMD_TLC_MNG_UPDATE_NOTIF, 0) >= 4) +
        (lookup_notif_ver(info, FW_GROUP_LEGACY,
                          FW_CMD_REPLY_RX_MPDU, 0) >= 6) +
        (lookup_notif_ver(info, FW_GROUP_DATA_PATH,
                          FW_CMD_RX_NO_DATA_NOTIF, 0) >= 4) +
        (lookup_notif_ver(info, FW_GROUP_LONG, FW_CMD_TX, 0) >= 9);
    if (ratecheck != 0 && ratecheck != 5) return 0;
    if (ratecheck == 5) rates_ver = 3;

    return rates_ver;
}
