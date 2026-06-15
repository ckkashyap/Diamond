#!/bin/bash
#
# iwl_verify.sh — pull authoritative data from the running Linux's iwlwifi
# driver to cross-check our bare-metal implementation.
#
# Run as root.  Output goes to iwl_verify_<timestamp>/ and a tarball.
#
# NOTE: deliberately NOT using `set -e` — many of these probes are
# best-effort (pahole missing, debugfs disabled, dyndbg not built, etc.)
# and a single failure shouldn't abort the whole capture.
set -u

if [ "$(id -u)" -ne 0 ]; then
    echo "Run as root: sudo bash $0"
    exit 1
fi

PCI_BDF=$(lspci -nn -D | awk '/\[8086:34[af][0-9a-f]\]/ {print $1; exit}')
[ -z "$PCI_BDF" ] && { echo "no iwlwifi device"; exit 1; }
DBGFS="/sys/kernel/debug/iwlwifi/$PCI_BDF"
SYSDIR="/sys/bus/pci/devices/$PCI_BDF"

OUT="iwl_verify_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
echo "device: $PCI_BDF"
echo "out:    $OUT"

# ── 1. struct sizes via pahole (confirms our context_info layout) ─────────
echo "[1/5] pahole struct dumps ..."
{
    # Find the iwlwifi kernel module (may be plain .ko, .ko.zst, or .ko.xz).
    KMOD=""
    for cand in \
        "/lib/modules/$(uname -r)/kernel/drivers/net/wireless/intel/iwlwifi/iwlwifi.ko" \
        "/lib/modules/$(uname -r)/kernel/drivers/net/wireless/intel/iwlwifi/iwlwifi.ko.zst" \
        "/lib/modules/$(uname -r)/kernel/drivers/net/wireless/intel/iwlwifi/iwlwifi.ko.xz"
    do
        if [ -e "$cand" ]; then KMOD="$cand"; break; fi
    done
    echo "kmod path: $KMOD"

    if [ -n "$KMOD" ] && [[ "$KMOD" == *.zst ]]; then
        zstd -d -k -f "$KMOD" -o /tmp/iwlwifi.ko >/dev/null 2>&1 && KMOD=/tmp/iwlwifi.ko
    fi
    if [ -n "$KMOD" ] && [[ "$KMOD" == *.xz ]]; then
        xz -dc "$KMOD" > /tmp/iwlwifi.ko 2>/dev/null && KMOD=/tmp/iwlwifi.ko
    fi
    echo "kmod after decomp: $KMOD"
    echo "kmod size: $(stat -c '%s bytes' "$KMOD" 2>/dev/null || echo missing)"

    if ! command -v pahole >/dev/null; then
        echo "(pahole NOT installed — try: sudo apt install dwarves)"
    elif [ -z "$KMOD" ] || [ ! -f "$KMOD" ]; then
        echo "(iwlwifi.ko not found in /lib/modules)"
    else
        for sname in iwl_context_info iwl_context_info_dram \
                     iwl_context_info_rbd_cfg iwl_context_info_hcmd_cfg \
                     iwl_context_info_dump_cfg iwl_context_info_pnvm_cfg \
                     iwl_context_info_version iwl_context_info_control \
                     iwl_context_info_early_dbg_cfg; do
            echo "=== struct $sname ==="
            pahole -C "$sname" "$KMOD" 2>/dev/null \
                || echo "(not found / no DWARF info for $sname)"
            echo
        done
    fi
} > "$OUT/structs.txt" 2>&1
wc -l "$OUT/structs.txt" || true

# ── 2. Live CSR dump via /sys/.../resource0 ──────────────────────────────
# Read first 256 bytes of BAR0 — that's the CSR region.
echo "[2/5] live CSR snapshot from BAR0 ..."
{
    BAR_FILE="$SYSDIR/resource0"
    if [ -r "$BAR_FILE" ]; then
        # 64 dwords = 0x100 bytes — full CSR region
        python3 - <<PYEOF 2>/dev/null
import struct, mmap
with open("$BAR_FILE", "r+b") as f:
    m = mmap.mmap(f.fileno(), 0x4000, prot=mmap.PROT_READ)
    print("offset    value      meaning")
    print("-" * 50)
    csrs = {
        0x000: "HW_IF_CONFIG",
        0x008: "INT",
        0x00C: "INT_MASK",
        0x010: "FH_INT_STATUS",
        0x020: "RESET",
        0x024: "GP_CNTRL",
        0x028: "HW_REV",
        0x040: "CTXT_INFO_BA_LO",
        0x044: "CTXT_INFO_BA_HI",
        0x048: "GP_UCODE_REG",
        0x05C: "UCODE_DRV_GP1_CLR",
        0x094: "LED_REG",
        0x0A8: "MAC_SHADOW_REG_CTRL",
    }
    for off, name in sorted(csrs.items()):
        val = struct.unpack("<I", m[off:off+4])[0]
        print(f"  0x{off:03x}   0x{val:08x}   {name}")
PYEOF
    else
        echo "(BAR0 not readable)"
    fi
} > "$OUT/csrs.txt" 2>&1
cat "$OUT/csrs.txt"

# ── 3. PCI config space ───────────────────────────────────────────────────
echo "[3/5] PCI config + capabilities ..."
{
    lspci -vvv -D -s "$PCI_BDF"
    echo
    echo "=== raw config (256B) ==="
    setpci -s "$PCI_BDF" 0.b 1.b 2.b 3.b 4.b 5.b 6.b 7.b 8.b 9.b a.b b.b c.b d.b e.b f.b 2>/dev/null || true
    echo "=== full config dump ==="
    lspci -xxxx -D -s "$PCI_BDF"
} > "$OUT/pci.txt"

# ── 4. iwlwifi debugfs (now that you may have IWLWIFI_DEBUGFS) ────────────
echo "[4/5] debugfs dump ..."
mkdir -p "$OUT/debugfs"
if [ -d "$DBGFS" ]; then
    for f in fw_info fw_ver rf nvm_hw nvm_sw nvm_prod tx_queue rx_queue \
             prph_reg fh_reg ; do
        [ -r "$DBGFS/$f" ] && cp "$DBGFS/$f" "$OUT/debugfs/$f" 2>/dev/null || true
    done
    find "$DBGFS" -maxdepth 1 -type f -printf '%f\n' | sort > "$OUT/debugfs/_ls"
fi
ls "$OUT/debugfs/" || true

# ── 5. iwlwifi dyndbg trace of a fresh module load ────────────────────────
echo "[5/5] dyndbg + dmesg trace of fresh modprobe (interrupts WiFi ~5s) ..."
if [ "${1:-}" = "--trace" ]; then
    DMESG_TS=$(dmesg | tail -1 | grep -oP '^\[\K[0-9.]+')
    echo "+iwlwifi_dev_tx +iwlwifi_dev_hcmd +iwlwifi" \
        > /sys/kernel/debug/dynamic_debug/control 2>/dev/null || true
    rmmod iwlmvm 2>/dev/null || true
    rmmod iwlwifi 2>/dev/null || true
    sleep 1
    dmesg -C
    modprobe iwlwifi
    sleep 5
    dmesg > "$OUT/modprobe_dmesg.txt"
    echo "  captured $(wc -l < "$OUT/modprobe_dmesg.txt") dmesg lines"
else
    echo "(skip; pass --trace to bounce iwlwifi and capture full init log)"
fi

# ── tarball + chown back ──────────────────────────────────────────────────
tar -czf "${OUT}.tar.gz" "$OUT"
chown -R "$SUDO_USER:$SUDO_USER" "$OUT" "${OUT}.tar.gz" 2>/dev/null || true
echo
echo "Done: ${OUT}/   and   ${OUT}.tar.gz"
echo
echo "Most valuable items:"
echo "  $OUT/structs.txt   — confirms iwl_context_info offsets (vs my iwl_ctxt.h)"
echo "  $OUT/csrs.txt      — live CSR_GP_CNTRL etc. while iwlwifi is associated"
echo "  $OUT/debugfs/      — fw_info, NVM (real MAC), prph_reg if available"
