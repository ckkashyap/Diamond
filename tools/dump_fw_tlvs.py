#!/usr/bin/env python3
"""Dump every TLV in an iwlwifi ucode file.  Focused on info we need for
the driver: API changes, capabilities, per-command version table.

Usage: ./dump_fw_tlvs.py <path-to-iwlwifi.ucode>
"""
import struct, sys, os

# Subset of Linux iwl-fw-file.h enum iwl_ucode_tlv_type that we care about.
TLV = {
    1:  "INST",            2:  "DATA",            3:  "INIT",
    4:  "INIT_DATA",       18: "FLAGS",           19: "SEC_RT",
    20: "SEC_INIT",        21: "SEC_WOWLAN",      22: "DEF_CALIB",
    23: "PHY_SKU",         29: "API_CHANGES_SET", 30: "ENABLED_CAPABILITIES",
    31: "N_SCAN_CHANNELS", 36: "FW_VERSION",      37: "FW_DBG_DEST",
    38: "FW_DBG_CONF",     39: "FW_DBG_TRIGGER",  40: "CSCHEME",
    42: "FW_GSCAN_CAPA",   43: "FW_MEM_SEG",      48: "CMD_VERSIONS",
    49: "FW_RECOVERY_INFO",50: "FW_DBG_DUMP_LST", 53: "PAGING",
    57: "FW_RECOVERY_INFO",58: "TYPE_BUFFER_ALLOC",
    59: "FW_FSEQ_VERSION", 60: "FW_FSEQ_VERSION",
}

# Subset of IWL_UCODE_TLV_CAPA_* — bit indexes into the 4×32 bit capa bitmap.
# Linux fw-api.h iwl_ucode_tlv_capa_t.
CAPA_BITS = {
    0:  "EARLY_PM",                 1:  "P2P_SCM_UAPSD",            2:  "BEACON_FILTER",
    3:  "BEACON_STORING",           4:  "LAR_MULTI_MCC",            5:  "TXPOWER_INSERTION_SUPPORT",
    6:  "DS_DISABLE_SUPPORT",       7:  "DS_DISABLE_SCAN_SUPPORT",  8:  "LMAC_CMD_VERSION",
    9:  "UMAC_SCAN",                10: "BEAMFORMER",               11: "TOF_SUPPORT",
    12: "TDLS_SUPPORT",             13: "TDLS_CHANNEL_SWITCH",      14: "TX_POWER_ACK",
    15: "STA_TYPE",                 16: "TLC_OFFLOAD",              17: "DYNAMIC_QUOTA",
    18: "COEX_SCHEMA_2",            19: "STA_PM_NOTIF",             20: "WFA_TPC_REP_IE_SUPPORT",
    21: "BINDING_CDB_SUPPORT",      22: "CDB_SUPPORT",              23: "D0I3_END_FIRST",
    24: "TLC_OFFLOAD",              25: "CHANNEL_SWITCH_CMD",       26: "FTM_CALIBRATED",
    27: "ULTRA_HB_CHANNELS",        28: "REGULATORY_NVM_INFO",      29: "TIME_SYNC_BOTH_FTM_TM",
    30: "FTM_NEW_RANGE_REQ",        31: "SCAN_TSF_REPORT",
    32: "TLC_OFFLOAD",              33: "DYNAMIC_QUOTA",            34: "STATIC_QUOTA_OVERRIDE",
    35: "STATIC_QUOTA_PSM",         36: "TDLS_CHANNEL_SWITCH",      37: "CNSLDTD_D3_D0_IMG",
    38: "HW_TIMESTAMP_REPORTING",   39: "RFIM_SUPPORT",             40: "BAID_ML_SUPPORT",
    41: "TKIP_MIC_KEYS",            42: "EXTENDED_DTS_MEASURE",     43: "SHORT_PM_TIMEOUTS",
    44: "BT_MPLUT_SUPPORT",         45: "MULTI_QUEUE_RX_SUPPORT",   46: "CSUM_SUPPORT",
    47: "RADIO_BEACON_STATS",       48: "WFA_TPC_REP_IE_SUPPORT",   49: "BT_COEX_PLCR",
    50: "LMAC_UPLOAD",              51: "EXTEND_SHARED_MEM_CFG",    52: "LQM_SUPPORT",
    53: "TX_POWER_DEV_SUPPORT",     54: "GSCAN_SUPPORT",            55: "CONT_RECORDING",
    56: "TRIG_TASS_NEW_DESCRIPTOR", 57: "PEER_MEASUREMENT_REPORTS", 58: "BEACON_ANT_SELECTION",
    59: "BEACON_STORING_FILTER",    60: "EXTENDED_PROTECTION",      61: "PROTECTED_TWT",
    62: "FW_API_VERSION",           63: "SOC_LATENCY_SUPPORT",
    64: "GEN3_TX_FRAGMENTATION",    65: "TX_RX_TIME_SYNC",          66: "PROTECTED_TWT_FT",
    67: "PROTECTED_TWT_NO_FT",      68: "BINDING_CDB_SUPPORT",
    73: "MBSSID_HE",                75: "TWT_MULTI_TID",            76: "RFIM_FORCE_TABLE",
    79: "NEW_TX_API",
    80: "TIME_SYNC_BOTH_FTM_TM",    81: "PROTECTED_TWT_FT_DONE",
    87: "FW_GET_FREE_TX_QUEUE",     88: "SCAN_DONT_TOGGLE_ANT",     89: "STA_EXP_MFP",
    90: "PASSIVE_SCAN_REPLY_IS_ACK",
}

# Subset of group ids (Linux fw/api/commands.h).
GROUP = {0:"LEGACY", 1:"LONG", 2:"SYS", 3:"MAC_CONF", 4:"PHY_OPS", 5:"DATA", 12:"REG_NVM", 15:"DBG"}

if len(sys.argv) != 2:
    print(__doc__); sys.exit(2)

path = sys.argv[1]
data = open(path, "rb").read()

# Header: u32 zero, u32 file_ver, ascii human_readable_ver up to first '\0',
# then a sequence of TLVs.  Format from iwl-fw-file.h struct iwl_tlv_ucode_header.
zero, magic = struct.unpack_from("<II", data, 0)
hum = data[8:8+64].split(b"\0",1)[0].decode("ascii", "replace")
ver, build, ign = struct.unpack_from("<IIQ", data, 72)
print(f"file: {path}  magic=0x{magic:08x}  human={hum!r}")
print(f"  ver=0x{ver:08x}  build=0x{build:08x}  ignore=0x{ign:016x}")

# Header is 8 + 64 + 4 + 4 + 8 = 88 bytes (see drivers/net/iwlwifi/iwl_fw.h).
off = 88
total = len(data)
n_caps = 0
api_caps = []           # list of (api_index, api_capa) tuples
cmd_versions = []       # (group, cmd, cmd_ver, notif_ver)

while off + 8 <= total:
    t, l = struct.unpack_from("<II", data, off)
    payload = data[off+8 : off+8+l]
    name = TLV.get(t, f"TLV_{t}")
    head = ""

    if t == 30:  # ENABLED_CAPABILITIES — struct iwl_ucode_capa { __le32 api_index; __le32 api_capa; }
        if len(payload) == 8:
            api_index, api_capa = struct.unpack("<II", payload)
            api_caps.append((api_index, api_capa))
            bits = []
            for b in range(32):
                if api_capa & (1<<b):
                    gbit = api_index*32 + b
                    bits.append(f"{gbit}({CAPA_BITS.get(gbit,'?')})")
            head = f"  api_index={api_index} api_capa=0x{api_capa:08x}  bits=[{', '.join(bits)}]"
            n_caps += 1
    elif t == 29:  # API_CHANGES_SET — same format
        if len(payload) == 8:
            api_index, api_capa = struct.unpack("<II", payload)
            head = f"  api_index={api_index} api_changes=0x{api_capa:08x}"
    elif t == 48:  # CMD_VERSIONS — array of struct iwl_fw_cmd_version
        ent_sz = 4
        for i in range(0, len(payload), ent_sz):
            if i+ent_sz > len(payload): break
            cmd, grp, cmd_ver, notif_ver = struct.unpack_from("<BBBB", payload, i)
            cmd_versions.append((grp, cmd, cmd_ver, notif_ver))
    elif t == 36:  # FW_VERSION — 3×u32: major, minor, build
        if len(payload) == 12:
            mj, mn, bu = struct.unpack("<III", payload)
            head = f"  {mj}.{mn}.{bu}"

    print(f"  TLV {t:3d} ({name:<26s})  len={l:5d}{head}")
    # 4-byte aligned padding
    advance = (l + 3) & ~3
    off += 8 + advance

print()
print("=" * 70)
print("CAPABILITIES (consolidated):")
all_caps = {}
for idx, capa in api_caps:
    for b in range(32):
        if capa & (1<<b):
            g = idx*32 + b
            all_caps[g] = CAPA_BITS.get(g, "?")
for gbit in sorted(all_caps):
    print(f"  bit {gbit:3d}: {all_caps[gbit]}")
# Spot-check the ones we care about for AUTH/ASSOC path:
print()
must_check = [15, 21, 22, 68, 79]  # STA_TYPE, BINDING_CDB, CDB, BINDING_CDB(legacy), NEW_TX_API
for b in must_check:
    name = CAPA_BITS.get(b, "?")
    print(f"  CAPA bit {b:3d} ({name}): {'SET' if b in all_caps else 'unset'}")

print()
print("=" * 70)
print("COMMAND VERSIONS:")
# Sort by group then cmd id
for grp, cmd, cv, nv in sorted(cmd_versions):
    gname = GROUP.get(grp, f"grp{grp}")
    print(f"  g=0x{grp:02x} ({gname:<8s}) c=0x{cmd:02x}  cmd_ver={cv}  notif_ver={nv}")
# Spot-check commands we use:
print()
print("commands of interest:")
of_interest = [
    (1, 0x2b, "BINDING_CONTEXT_CMD"),
    (1, 0x08, "PHY_CONTEXT_CMD"),
    (1, 0x28, "MAC_CONTEXT_CMD"),
    (1, 0x18, "ADD_STA"),
    (3, 0x05, "SESSION_PROTECTION_CMD"),
    (3, 0x08, "MAC_CONFIG_CMD"),
    (3, 0x09, "LINK_CONFIG_CMD"),
    (3, 0x0a, "STA_CONFIG_CMD"),
    (3, 0x0b, "AUX_STA_CMD"),
    (5, 0x08, "RLC_CONFIG_CMD"),
    (5, 0x0f, "TLC_MNG_CONFIG_CMD"),
    (5, 0x17, "SCD_QUEUE_CONFIG_CMD"),
    (0, 0x1c, "TX_CMD short-header"),
    (1, 0x1c, "TX_CMD LONG TLV"),
    (1, 0x0d, "SCAN_REQ_UMAC"),
    (1, 0x0c, "SCAN_CFG_CMD"),
]
have = {(g,c): (cv,nv) for g,c,cv,nv in cmd_versions}
for g, c, name in of_interest:
    v = have.get((g, c))
    print(f"  {name:<24s} g={g} c=0x{c:02x}: " +
          (f"cmd_ver={v[0]} notif_ver={v[1]}" if v else "NOT ADVERTISED (driver uses default)"))
