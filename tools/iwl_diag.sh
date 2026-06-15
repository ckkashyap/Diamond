#!/bin/bash
#
# iwl_diag.sh — gather everything needed to bring up the bare-metal iwlwifi
#               driver: firmware binary, dmesg init log, debugfs NVM/fw dumps,
#               PCI config, MAC address, and (optionally) an ftrace of a
#               real association.
#
# Usage:
#    sudo bash tools/iwl_diag.sh             # one-shot snapshot
#    sudo bash tools/iwl_diag.sh --trace     # also modprobe -r/modprobe and
#                                            # capture ftrace of iwlwifi events
#                                            # while a connection is (re)made
#
# Output goes to iwl_diag_<timestamp>/ in the current directory.
#
set -euo pipefail

# ── Root check ────────────────────────────────────────────────────────────
if [ "$(id -u)" -ne 0 ]; then
    echo "This script needs root (reads /dev/mem-adjacent debugfs files and"
    echo "copies kernel firmware).  Re-run with: sudo bash $0"
    exit 1
fi

# ── Locate the iwlwifi device + interface + driver dir ───────────────────
PCI_BDF=$(lspci -nn -D | awk '/\[8086:34[af][0-9a-f]\]/ {print $1; exit}')
if [ -z "$PCI_BDF" ]; then
    # Fallback to any iwlwifi-bound Intel NIC
    PCI_BDF=$(basename "$(readlink -f /sys/bus/pci/drivers/iwlwifi/0000:* 2>/dev/null | head -n1)" 2>/dev/null || true)
fi
if [ -z "$PCI_BDF" ]; then
    echo "No iwlwifi device found."
    exit 1
fi

WIFI_IFACE=$(find /sys/class/net -maxdepth 2 -name device \
                 -lname "*$PCI_BDF*" -printf '%h\n' 2>/dev/null \
             | head -n1 | xargs -r basename)

DBGFS="/sys/kernel/debug/iwlwifi/$PCI_BDF"
OUT="iwl_diag_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

echo "Device:       $PCI_BDF"
echo "Interface:    ${WIFI_IFACE:-<none>}"
echo "Output dir:   $OUT"
echo ""

# ── 1. dmesg slice for iwlwifi (full history) ────────────────────────────
echo "[1/6] Capturing dmesg ..."
dmesg | grep -iE 'iwlwifi|iwl-|iwl:' > "$OUT/dmesg.txt" || true
wc -l "$OUT/dmesg.txt"

# ── 2. PCI config (full 4 KiB) + lspci -vvvxxxx ──────────────────────────
echo "[2/6] Dumping PCI config ..."
{
    lspci -vvv -D -s "$PCI_BDF"
    echo
    echo "=== raw config space (4 KiB) ==="
    lspci -xxxx -D -s "$PCI_BDF"
} > "$OUT/pci.txt" 2>&1

# ── 3. MAC + iface + basic link info ──────────────────────────────────────
echo "[3/6] Reading MAC + link state ..."
{
    if [ -n "$WIFI_IFACE" ]; then
        echo "iface: $WIFI_IFACE"
        echo "MAC:   $(cat /sys/class/net/$WIFI_IFACE/address 2>/dev/null)"
        iw dev "$WIFI_IFACE" info 2>/dev/null || true
        iw dev "$WIFI_IFACE" link 2>/dev/null || true
    fi
} > "$OUT/iface.txt"

# ── 4. debugfs dumps (firmware, NVM, regulatory, fw phases) ──────────────
echo "[4/6] Copying debugfs ..."
mkdir -p "$OUT/debugfs"
if [ -d "$DBGFS" ]; then
    for f in fw_info fw_ver nvm_hw nvm_sw nvm_prod rf fw_nmi_result \
             frame_stats secret_sauce rx_queue tx_queue sta ; do
        [ -r "$DBGFS/$f" ] && cp "$DBGFS/$f" "$OUT/debugfs/$f" 2>/dev/null || true
    done
    # Flat text listing of everything else readable
    find "$DBGFS" -maxdepth 1 -type f -printf '%f\n' | sort > "$OUT/debugfs/_ls"
else
    echo "(debugfs not mounted — try 'mount -t debugfs none /sys/kernel/debug')" \
        > "$OUT/debugfs/UNAVAILABLE"
fi

# ── 5. Firmware binary: find the exact file the driver loaded ─────────────
echo "[5/6] Extracting firmware binary ..."
mkdir -p "$OUT/firmware"

# Parse the firmware basename.  dmesg can print either of:
#   "loaded firmware version 77.xxx Qu-c0-hr-b0-77.ucode op_mode ..."
#   "direct-loading firmware iwlwifi-QuZ-a0-hr-b0-77.ucode"
# Strip any leading "iwlwifi-" and re-add it so we end up with the on-disk name.
FW_TAG=$(grep -oE '[A-Za-z0-9]+-[a-z0-9]+-hr-b0-[0-9]+\.ucode' "$OUT/dmesg.txt" \
         | head -n1 | sed 's/^iwlwifi-//' || true)
if [ -n "$FW_TAG" ]; then
    FW_BASENAME="iwlwifi-$FW_TAG"
else
    # Fallback: newest hr-b0 firmware of any family
    FW_BASENAME=$(ls /lib/firmware/iwlwifi-*-hr-b0-*.ucode* 2>/dev/null \
                  | sort -V | tail -n1 | xargs -rn1 basename \
                  | sed 's/\.zst$//')
fi

if [ -n "$FW_BASENAME" ]; then
    echo "Firmware basename: $FW_BASENAME"
    FW_SRC="/lib/firmware/$FW_BASENAME"
    if [ -r "$FW_SRC" ]; then
        cp "$FW_SRC" "$OUT/firmware/$FW_BASENAME"
    elif [ -r "$FW_SRC.zst" ]; then
        zstd -d -k "$FW_SRC.zst" -o "$OUT/firmware/$FW_BASENAME"
    elif [ -r "$FW_SRC.xz" ]; then
        xz -dc "$FW_SRC.xz" > "$OUT/firmware/$FW_BASENAME"
    else
        echo "  (firmware blob $FW_SRC{,.zst,.xz} not found on disk)"
    fi
fi

# Always include the version-suffix directory listing so we know what
# alternatives exist if we need a different ucode later.
ls -la /lib/firmware/iwlwifi-QuZ-a0-* 2>/dev/null \
    > "$OUT/firmware/_available.txt" || true

# ── 6. Optional: ftrace of a real iwlwifi association ────────────────────
if [ "${1:-}" = "--trace" ]; then
    echo "[6/6] Recording ftrace of iwlwifi association ..."
    if ! command -v trace-cmd >/dev/null; then
        echo "  (install trace-cmd: apt install trace-cmd)" > "$OUT/trace.txt"
    else
        echo "  bouncing iwlwifi module to force a fresh association trace"
        modprobe -r iwlwifi 2>/dev/null || true
        sleep 1
        trace-cmd record -q -o "$OUT/iwl.trace.dat" \
            -e iwlwifi_dev_tx -e iwlwifi_dev_rx \
            -e iwlwifi_dev_hcmd -e iwlwifi_dev_ucode_cont_event \
            -e iwlwifi_dev_ucode_event -e iwlwifi_dev_ucode_error \
            -- sh -c "modprobe iwlwifi; sleep 20" >/dev/null 2>&1 &
        TCPID=$!
        wait "$TCPID" || true
        trace-cmd report "$OUT/iwl.trace.dat" > "$OUT/iwl.trace.txt" 2>/dev/null || true
    fi
else
    echo "[6/6] Skipping ftrace (pass --trace to include)"
fi

# ── Wrap up ───────────────────────────────────────────────────────────────
echo ""
echo "Packing tarball ..."
tar -czf "${OUT}.tar.gz" "$OUT"
chown "$SUDO_USER:$SUDO_USER" "${OUT}.tar.gz" 2>/dev/null || true
chown -R "$SUDO_USER:$SUDO_USER" "$OUT" 2>/dev/null || true
echo ""
echo "Done: $OUT/   and   ${OUT}.tar.gz"
echo "Share either directory listing or the tarball:"
ls -la "$OUT"
