# iwlwifi AX201 — Phase 4 Handoff (scan working, association next)

## Goal

Bring up the Intel AX201 on Surface Laptop 3 from scratch in a bare-metal exokernel — no IRQs, all polling. Target: associate with a WPA2 AP, run DHCP, ping out.

## Current State (high level)

**Working end-to-end**:
- PCI probe / BAR map / firmware upload via context_info / ALIVE / unified NVM init / runtime probe / scan.
- Passive 2.4 GHz scan rounds 6 beacons in one pass and parses each as `BSSID + channel + RSSI + SSID`.
- Latest hardware run prints e.g.:
  ```
  AP 40:ca:63:a5:0e:22  ch=01 rssi=-4b ssid="[fridge]_E30AJT5123486X"
  AP 1e:9d:72:74:ff:1a  ch=06 rssi=-26 ssid=(hidden)
  AP 1e:9d:72:74:ff:1d  ch=06 rssi=-26 ssid="Actual-2.4"
  SCAN_COMPLETE: status=01 ebs=03 iters=00
  iwl: scan OK
  ```

**Not working yet**:
- Active scan (broadcast probe) — passive scan sees hidden BSSIDs but no SSID for them. Active scan TX is needed to elicit Probe Responses with SSIDs.
- Authentication / association — needs TX management-frame path.
- Data plane (802.11↔ethernet, encryption, DHCP).

## Hardware

- Intel Wi-Fi 6 AX201 — PCI 8086:34F0, ICL-LP CNVi, integrated, gen2 silicon (22000 family)
- Firmware: `iwlwifi-Qu-c0-hr-b0-77.ucode` (build hash `0xb405f9d4`)
- BAR0 16 KiB at 0x6001134000 (64-bit MMIO)
- Same chip as the Linux on this machine — Linux uses iwlwifi successfully, so the hardware/firmware can definitely do what we want.

## Source Files

```
drivers/net/iwlwifi/
  iwlwifi.c          PCI bring-up, MAC_ACCESS, PRPH access, OTP MAC read
  iwlwifi.h          public API (iwl_probe, iwl_hw_rev, iwl_mac, iwl_fw_info_get)
  iwl_fw.c/h         firmware TLV parser (loads as Limine module);
                     parses cmd-version table (TLV 48) → iwl_fw_cmd_ver(group,cmd)
  iwl_fw_load.c      ctxt_info init, RX ring alloc, firmware kick, ALIVE wait
  iwl_cmd.c          TFD ring + first_tb_buf scratch, iwl_send_cmd, all
                     phase-4/5 probes (NVM, RT, scan setup, scan, beacon parser)
  iwl_cmd.h          struct iwl_tfh_tb / iwl_tfh_tfd, public probe API
  iwl_csr.h          CSR/PRPH register addresses, control_flags, cmd opcodes
  iwl_ctxt.h         struct iwl_context_info layout

apps/shell/shell.c   wifiinit, wifiload, wifiup, wifiscan, wifiecho,
                     wifinvm, wifirt, wifisetup, wifivers, wifiinfo
Makefile             OBJ_IWL_*; IWL_HDRS dependency tracking
```

## Boot Flow (current)

The single-command `wifiup` runs the whole chain. Granular commands stay available for stepping.

1. `wifiinit` → `iwl_probe()` — PCI scan, BAR map, OTP MAC read.
2. `wifiload` → `iwl_fw_load()` — `rx_init` + `iwl_cmd_q_alloc` + `ctxt_init` + `build_dram_sections`, kick firmware via `UREG_CPU_INIT_RUN`, wait for ALIVE.
3. `wifinvm` — unified init through `NVM_GET_INFO` (sends `INIT_EXTENDED_CFG_CMD` system group, `NVM_ACCESS_COMPLETE` REGULATORY group, waits for `INIT_COMPLETE_NOTIF`, then `NVM_GET_INFO`).
4. `wifirt` — `SHARED_MEM_CFG_CMD` + async `REPLY_SF_CFG_CMD` + `TX_ANT_CONFIGURATION_CMD` + `BT_CONFIG`. All four are LONG_GROUP per the firmware's TLV.
5. `wifisetup` — `PHY_CONTEXT_CMD ADD ctx=0` + `MAC_CONTEXT_CMD ADD id=0 type=LISTENER` + `SCAN_CFG_CMD` (12-byte reduced struct, tx/rx chains = 0x3).
6. `wifiscan` — `SCAN_REQ_UMAC v15` (1940 bytes) for passive 2.4 GHz channels {1,6,11}. Polls for `SCAN_COMPLETE_UMAC` (~5 s). Per-RX-MPDU prints a one-line beacon decode.

## Key Bring-up Lessons (don't lose these)

1. **`cmd_queue_size = 2`** (Linux-spec `TFD_QUEUE_CB_SIZE(32)`), not raw `32`. With raw 32 firmware just skips cmd-queue setup and host commands never dispatch.
2. **All RX DMA bases page-aligned (4 KiB)**. `iwl-fh.h` says "bits 11:0 should be set to zero" for `RFH_Q_FRBDCB_BA_LSB`. Misalignment doesn't matter when firmware skips ctxt_info validation, but breaks RX once `cmd_queue_size` is valid.
3. **MAC_SHADOW_REG_CTRL = 0x802FFFFF** (the live readback value), not Linux source's `0x800FFFFF`. The chip self-sets bit 21 during init; we must include it.
4. **WIDX restock value must be a multiple of 8** (chip requirement). 504 = 63 × 8.
5. **CSR_INT_MASK = ALIVE | FH_RX** for the entire fw-load path. Don't switch masks between fw kick and ALIVE notification.
6. **No host-side RFH programming**. Linux gen2 explicitly skips it; firmware programs RFH itself based on `ctxt_info.rbd_cfg`.
7. **Free-RBD entries are `rb_phys | (i + 1)`** on 22000-family — the chip uses the low 12 bits as a virtual ID for RX dispatch.
8. **Group IDs depend on firmware TLVs, not Linux source.** Several commands moved from LEGACY (g=0) to LONG (g=1) on this firmware: SF_CFG, TX_ANT_CONFIGURATION, BT_CONFIG. The `wifivers` shell command dumps the table for our tracked commands.
9. **DQA_ENABLE_CMD not advertised** on this firmware — sending it asserts SW_ERR. The cmd is gated by `IWL_UCODE_TLV_CAPA_DQA_SUPPORT` which Qu-c0-hr-b0-77 does not advertise.
10. **ADD_STA not needed for scan.** `ADD_STA cmd_ver = 12 ≥ 12` triggers `iwl_mvm_has_new_station_api(fw) == true`, which means firmware allocates an internal aux STA itself.
11. **SCAN_CFG_CMD MUST run before SCAN_REQ_UMAC.** Linux's `iwl_mvm_config_scan` sends a 12-byte reduced struct (ver≥5 with `IWL_UCODE_TLV_API_REDUCED_SCAN_CONFIG`) one time per boot. Without it firmware asserts on the first scan request.
12. **SCAN_COMPLETE_UMAC and SCAN_ITER_COMPLETE_UMAC arrive at `group=0x00`** despite Linux registering them at `WIDE_ID(LONG_GROUP, ...)`. Match by cmd opcode alone.
13. **Beacon prefix on AX201/Qu is `iwl_rx_mpdu_desc` v1, 48 bytes total.** 802.11 frame begins at offset 48 from `pkt->data`. `mpdu_len` (LE16) at offset 0; `energy_a` at 32; `channel` at 34. BSSID at frame+16; IEs from frame+36.

## TLV cmd-version dump (verified hardware)

`wifivers` printed for our tracked commands:

```
fw advertises 191 cmd-version entries
  ECHO              g=0x00 c=0x03 ver=--
  INIT_EXTENDED_CFG g=0x02 c=0x03 ver=--
  SHARED_MEM_CFG    g=0x02 c=0x00 ver=0x63
  NVM_GET_INFO      g=0x0c c=0x02 ver=0x01
  TX_ANT_CFG        g=0x01 c=0x98 ver=0x01
  REPLY_SF_CFG      g=0x01 c=0xd1 ver=0x03
  BT_CONFIG         g=0x01 c=0x9b ver=0x06
  PHY_CONTEXT       g=0x01 c=0x08 ver=0x04
  MAC_CONTEXT       g=0x01 c=0x28 ver=0x05
  ADD_STA           g=0x01 c=0x18 ver=0x0c
  SCAN_CFG          g=0x01 c=0x0c ver=??  (not in our watch list when test ran)
  SCAN_REQ_UMAC     g=0x01 c=0x0d ver=0x0f
  SCAN_COMPLETE_UMAC   g=0x01 c=0x0f ver=--
  SCAN_ITER_COMPLETE   g=0x01 c=0xb5 ver=--
  DQA_ENABLE        g=0x05 c=0x00 ver=--
```

Notifications (SCAN_COMPLETE_UMAC / SCAN_ITER_COMPLETE / ECHO / INIT_EXTENDED_CFG) typically don't have a `cmd_ver` entry because the TLV represents commands; firmware delivers notifications via a separate path.

## Diagnostic Commands

Diamond shell:
```
wifiup               # full bring-up: init → load → NVM → RT → setup
wifiscan             # passive 2.4 GHz scan
wifivers             # dump fw-advertised cmd versions
wifiinfo             # chip rev + OTP MAC + firmware info

# Granular (each step is its own command for isolated debugging)
wifiinit             # PCI probe only
wifiload             # firmware upload + ALIVE
wifiecho             # first post-ALIVE host command (INIT_EXTENDED_CFG)
wifinvm              # unified init → NVM_GET_INFO
wifirt               # SHARED_MEM, SF_CFG, TX_ANT, BT_CONFIG
wifisetup            # PHY_CONTEXT, MAC_CONTEXT, SCAN_CFG
```

On running Linux (cross-reference, same hardware):
```
sudo bash tools/iwl_diag.sh    # extract firmware, dmesg, debugfs
sudo bash tools/iwl_verify.sh  # live CSR + pahole struct snapshot
```

Captured live state in repo:
- `iwl_diag_20260420-194507/` — dmesg + firmware blob extraction
- `iwl_verify_20260425-120149/` — CSR/PCI dump (HW_REV=0x332, GP_CNTRL=0x0c040005, MAC_SHADOW=0x802fffff, CTXT_INFO_BA_LO=0xff628000)

## Build & Run

```
make                # builds diamond.iso
make install        # installs to EFI partition (dual-boot)
make efi-register   # registers UEFI boot entry (one-time)
```

User reboots into Diamond from the EFI menu, runs the relevant `wifi*` command, photographs the screen (no log capture), sends a JPEG. Iteration cycle ≈ 3-5 minutes per test. **Each iteration is a hardware reboot — prioritize falsifiable hypotheses and always print enough diagnostic to localize the failure.**

## Commit History (current)

```
076e0fa iwlwifi: parse beacons — BSSID, channel, RSSI, SSID
1a7a1c9 iwlwifi: passive 2.4 GHz scan + 6 beacons received
7d52232 iwlwifi: add verified runtime setup probe
de4688f iwlwifi: complete unified NVM init probe
5cdeb36 iwlwifi: fix gen2 host command transport
a775f2f iwlwifi: phase 4 scaffolding + reliable RX checkpoint
be29bf3 iwlwifi: phase 3a — RFH bring-up + real RX delivery
5f1580c iwlwifi: phase 2 — firmware load + ALIVE on AX201
57262db iwlwifi: phase 1 bring-up for Surface Laptop 3 AX201
```

## Phase 5 — Plan toward an actual network connection

### 5a · Active scan with broadcast probe (immediate next step)

The `Actual-2.4` AP shows two BSSIDs (`1e:9d:72:74:ff:1a` hidden, `1e:9d:72:74:ff:1d` visible) — same physical AP, multi-VAP setup. Other hidden SSIDs across the network won't be discovered with passive scan. **Active scan** sends a broadcast probe request and waits for Probe Response frames which carry the SSID even on hidden SSIDs.

To do active scan:
1. **Drop FORCE_PASSIVE flag** in `iwl_scan_general_params_v11.flags`.
2. **Build a 24-byte 802.11 probe-request frame template** in `iwl_scan_probe_req.buf`:
   - frame_control = 0x40 0x00 (mgmt type, probe-req subtype)
   - duration = 0
   - DA = `ff:ff:ff:ff:ff:ff`
   - SA = our OTP MAC
   - BSSID = `ff:ff:ff:ff:ff:ff`
   - seq_ctrl = 0
3. **Set the segment offsets**: `mac_header.offset=0 len=24`. Then append a Supported Rates IE in `buf[24..]`: `01 04 02 04 0b 16` (IE id=1, len=4, rates 1/2/5.5/11 Mbps with the basic-rate bit). Set `common_data.offset=24 len=6`.
4. **`direct_scan[0]` = empty SSID** (id=0, len=0) for broadcast.
5. SSID match bit on the channel-cfg flags if needed.

Active scan probably needs the firmware-internal aux STA (which firmware already allocates per `iwl_mvm_has_new_station_api`). No host-side ADD_STA required — same as passive.

If first try asserts: most likely the probe template format. Compare against Linux's `iwl_mvm_build_scan_probe()`.

### 5b · Authentication + Association (open or PSK)

Pick one BSSID from the scan list (probably `Actual-2.4`'s `1e:9d:72:74:ff:1d`). Then:

1. **Tune PHY_CONTEXT_CMD MODIFY** to that AP's channel.
2. **MAC_CONTEXT_CMD MODIFY** to BSS_STA mode (mac_type=5) with the AP's BSSID in `bssid_addr`.
3. **ADD_STA** for the AP itself: type=BSS_STA, addr=AP's MAC, queues for QoS data + management.
4. **Build & TX an 802.11 AUTH frame** (algorithm=0=OPEN, seq=1) via the data TX TFD ring. This is where TX ring scaffolding begins:
   - We already have a cmd queue (queue 0). Need a separate data TX queue.
   - Per Linux `iwl_txq_alloc`, that's another TFD ring + first_tb_buf + bc_tbl. Allocated and registered with firmware via `TX_QUEUE_CFG_CMD` (DATA_PATH_GROUP).
5. **Wait for AUTH response** (RX MPDU) with status=0.
6. **Build & TX ASSOC_REQ** with capability info + supported rates + SSID + extended capabilities IEs.
7. **Wait for ASSOC_RESP** — extracts AID.
8. **Send `MAC_CONTEXT_CMD MODIFY` with `assoc=1`** and the assoc_id.

For WPA2 (`Actual-2.4` is presumably WPA2-PSK):
9. **EAPOL 4-way handshake** in software. M1 from AP, derive PTK, M2 with MIC, M3 from AP, M4 ack.
10. **Install PTK + GTK** via `WEP_KEY_CMD` (or whatever the modern variant is for AES-CCMP — likely `ADD_STA_KEY_CMD`).
11. **MAC_CONTEXT_CMD MODIFY** to install ucast filter + final state.

For an open AP (much simpler — find one for the first try), skip steps 9-11.

### 5c · Data plane

Once associated:
1. Encapsulate ARP/IP/etc. as 802.11 data frames with LLC/SNAP header (AA AA 03 00 00 00 + ethertype).
2. TX via the data queue.
3. RX 802.11 data frames carry LLC/SNAP; strip and present to the existing `drivers/net/arp.c` / `ipv4.c` / `tcp.c` stack.
4. DHCP discover/offer/request/ack to get an IP.
5. Ping out.

## Suggested Next Steps for a Fresh Agent

1. **Active scan first** (small, low-risk extension of working scan code). Build a probe-request template, drop FORCE_PASSIVE, set channel-cfg flags for SSID match if needed. Hidden BSSIDs should now report SSIDs.
2. **Pick the simplest open or WPA2 AP** in the scan results. (`Actual-2.4` is the user's own — credentials available.)
3. **TX ring** is the next big infrastructure piece — allocate a second TFD ring, set up bc_tbl, send `TX_QUEUE_CFG_CMD` to register it.
4. **Build management frames in software** — they're not big (24-byte header + 12-byte fixed body + IEs), well-documented in IEEE 802.11.
5. **Each step is a hardware reboot.** Always print enough diagnostic that one photo localizes any failure.

## What NOT To Lose Sight Of

- Each photo is the only feedback loop. Print **command-id + response-len + INT/FHINT before and after** for every command. That single discipline saved many iterations during scan bring-up.
- `wifivers` is the source of truth for group/cmd_id/version on this firmware. Trust the TLV table over Linux source claims.
- Notifications match by **cmd id alone** on this firmware — group_id may be 0x00 even though Linux uses LONG_GROUP for the handler.
- Most failures are firmware-asserts (`csr: INT=02000000` SW_ERR). The single byte that triggers the assert is almost always knowable from Linux source if you look at the *right* cmd version of the *right* struct for *this* firmware. The agent dispatches that landed correct answers always quoted Linux source verbatim with file:line refs; the ones that paraphrased were wrong.
