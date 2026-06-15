#!/bin/bash
# Capture iwlwifi command/TX traces from Linux on the same AX201 hardware.
#
# Usage:
#   sudo ./tools/linux_capture_bind.sh                 # reconnect active Wi-Fi profile
#   sudo ./tools/linux_capture_bind.sh "SSID" [psk]    # reconnect/connect target SSID
#   sudo ./tools/linux_capture_bind.sh --passive [sec] # trace without disconnecting
#
# Output:
#   iwl_linux_capture_<timestamp>/ and .tar.gz in the current directory.
#
# The default mode intentionally disconnects/reconnects the iwlwifi interface
# to force a fresh AUTH/ASSOC sequence.  Use --passive to avoid disruption.

set -euo pipefail

MODE="reconnect"
SSID="${1:-}"
PSK="${2:-}"
PASSIVE_SECS=10
if [ "${1:-}" = "--passive" ]; then
    MODE="passive"
    PASSIVE_SECS="${2:-10}"
    SSID=""
    PSK=""
fi

[ "$(id -u)" -eq 0 ] || { echo "must run as root: sudo $0"; exit 1; }

OUT="iwl_linux_capture_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
exec > >(tee "$OUT/summary.txt") 2>&1

echo "output: $OUT"
echo "mode:   $MODE"
echo

PCI_BDF=$(lspci -nn -D | awk '/Network controller/ && /Intel/ && /\[8086:34f0\]/ {print $1; exit}')
if [ -z "$PCI_BDF" ]; then
    PCI_BDF=$(basename "$(readlink -f /sys/bus/pci/drivers/iwlwifi/0000:* 2>/dev/null | head -n1)" 2>/dev/null || true)
fi
[ -n "$PCI_BDF" ] || { echo "no iwlwifi PCI device found"; exit 1; }

IFACE=""
for d in /sys/class/net/*; do
    [ -L "$d/device" ] || continue
    if [ "$(basename "$(readlink -f "$d/device")")" = "$PCI_BDF" ]; then
        IFACE=$(basename "$d")
        break
    fi
done
[ -n "$IFACE" ] || { echo "no netdev for $PCI_BDF"; exit 1; }

ACTIVE_CONN=$(nmcli -t -f NAME,TYPE,DEVICE connection show --active 2>/dev/null |
              awk -F: -v dev="$IFACE" '$2=="802-11-wireless" && $3==dev {print $1; exit}')
ACTIVE_SSID=$(nmcli -t -f ACTIVE,SSID device wifi list ifname "$IFACE" 2>/dev/null |
              awk -F: '$1=="yes" {print $2; exit}')
if [ -z "$SSID" ]; then
    SSID="$ACTIVE_SSID"
fi

echo "kernel: $(uname -a)"
echo "pci:    $PCI_BDF"
echo "iface:  $IFACE"
echo "active connection: ${ACTIVE_CONN:-<none>}"
echo "target ssid:       ${SSID:-<none>}"
echo

{
    echo "== lspci =="
    lspci -vvv -D -s "$PCI_BDF"
    echo
    echo "== nmcli active =="
    nmcli -f all connection show --active
    echo
    echo "== wifi list =="
    nmcli -f ACTIVE,SSID,BSSID,CHAN,FREQ,RATE,SIGNAL,SECURITY device wifi list ifname "$IFACE"
} > "$OUT/linux_context.txt" 2>&1 || true

dmesg | grep -iE 'iwlwifi|iwl-|wlp|firmware|34f0' > "$OUT/dmesg_iwlwifi.txt" || true

FW_TAG=$(grep -oE '[A-Za-z0-9]+-[a-z0-9]+-hr-b0-[0-9]+\.ucode' "$OUT/dmesg_iwlwifi.txt" |
         head -n1 | sed 's/^iwlwifi-//' || true)
if [ -n "$FW_TAG" ]; then
    FW_BASENAME="iwlwifi-$FW_TAG"
    echo "loaded firmware: $FW_BASENAME"
    mkdir -p "$OUT/firmware"
    if [ -r "/lib/firmware/$FW_BASENAME" ]; then
        cp "/lib/firmware/$FW_BASENAME" "$OUT/firmware/" || true
    elif [ -r "/lib/firmware/$FW_BASENAME.zst" ]; then
        zstd -d -q -k "/lib/firmware/$FW_BASENAME.zst" -o "$OUT/firmware/$FW_BASENAME" || true
    fi
fi

TRACE=""
for cand in /sys/kernel/tracing /sys/kernel/debug/tracing; do
    if [ -d "$cand" ] && [ -w "$cand/tracing_on" ]; then TRACE=$cand; break; fi
done
if [ -z "$TRACE" ]; then
    mkdir -p /sys/kernel/tracing
    mount -t tracefs nodev /sys/kernel/tracing 2>/dev/null || true
    [ -w /sys/kernel/tracing/tracing_on ] && TRACE=/sys/kernel/tracing
fi
[ -n "$TRACE" ] || { echo "no writable tracefs found"; exit 1; }

EVENTS=$(grep -E '^iwlwifi(:|_)' "$TRACE"/available_events 2>/dev/null || true)
printf "%s\n" "$EVENTS" > "$OUT/available_iwlwifi_events.txt"
echo "tracefs: $TRACE"
echo "iwlwifi trace events: $(grep -c . "$OUT/available_iwlwifi_events.txt" || true)"
if ! grep -qE 'iwlwifi(:|_)iwlwifi_dev_hcmd' "$OUT/available_iwlwifi_events.txt"; then
    echo "WARNING: iwlwifi_dev_hcmd event is unavailable; kernel may lack iwlwifi tracing."
fi

cleanup_trace() {
    echo 0 > "$TRACE/tracing_on" 2>/dev/null || true
}
trap cleanup_trace EXIT

echo 0 > "$TRACE/tracing_on"
echo > "$TRACE/trace"
echo 131072 > "$TRACE/buffer_size_kb" 2>/dev/null || true
> "$TRACE/set_event"
while read -r ev; do
    [ -n "$ev" ] && echo "$ev" >> "$TRACE/set_event" 2>/dev/null || true
done <<< "$EVENTS"
cat "$TRACE/set_event" > "$OUT/enabled_events.txt" 2>/dev/null || true

echo "starting trace..."
echo 1 > "$TRACE/tracing_on"

if [ "$MODE" = "passive" ]; then
    sleep "$PASSIVE_SECS"
else
    [ -n "$SSID" ] || { echo "no active SSID and none provided"; exit 1; }
    nmcli device disconnect "$IFACE" >/dev/null 2>&1 || true
    sleep 1
    if [ -n "$PSK" ]; then
        nmcli device wifi connect "$SSID" password "$PSK" ifname "$IFACE" >/dev/null 2>&1 || true
    elif [ -n "$ACTIVE_CONN" ] && [ "$SSID" = "$ACTIVE_SSID" ]; then
        nmcli connection up "$ACTIVE_CONN" ifname "$IFACE" >/dev/null 2>&1 || true
    else
        nmcli device wifi connect "$SSID" ifname "$IFACE" >/dev/null 2>&1 || true
    fi
    sleep 8
fi

echo 0 > "$TRACE/tracing_on"
cp "$TRACE/trace" "$OUT/trace.txt"
LINES=$(wc -l < "$OUT/trace.txt")
echo "captured trace lines: $LINES"

echo
echo "===== filtered hcmd entries ====="
grep -E 'iwlwifi_dev_hcmd' "$OUT/trace.txt" | tee "$OUT/hcmd.txt" | head -120 || true

echo
echo "===== TX/RX entries ====="
grep -E 'iwlwifi_dev_(tx|rx)' "$OUT/trace.txt" | tee "$OUT/txrx.txt" | head -120 || true

echo
echo "===== key command hits ====="
grep -Ei 'hcmd|TX_CMD|SCD|ADD_STA|MAC_CONTEXT|BINDING|PHY_CONTEXT|txq|queue|auth|assoc' \
    "$OUT/trace.txt" | tee "$OUT/key_hits.txt" | head -200 || true

if ! grep -Eq 'ADD_STA|MAC_CONTEXT|BINDING|PHY_CONTEXT|SCD_QUEUE' "$OUT/key_hits.txt"; then
    echo
    echo "note: no fresh association setup commands were captured."
    echo "      For AUTH/ASSOC bring-up, run reconnect mode:"
    echo "        sudo $0 ${SSID:+"$SSID"}"
fi

tar -czf "$OUT.tar.gz" "$OUT"
chown -R "${SUDO_USER:-$USER}:${SUDO_USER:-$USER}" "$OUT" "$OUT.tar.gz" 2>/dev/null || true
echo
echo "done: $OUT.tar.gz"
