/*
 * shell.c - Diamond interactive command shell
 *
 * Accepts input from the PS/2 keyboard or COM1 serial port.
 * Output goes to both the framebuffer terminal and COM1.
 */

#include <stdint.h>
#include "../../kernel/alloc.h"
#include "../../drivers/terminal.h"
#include "../../arch/x86/io.h"
#include "../../arch/x86/smp.h"
#include "../../drivers/net/net.h"
#include "../../drivers/net/netstack.h"
#include "../../drivers/net/iwlwifi/iwlwifi.h"
#include "../../drivers/net/iwlwifi/iwl_fw.h"
#include "../../drivers/net/iwlwifi/iwl_cmd.h"
#include "../../drivers/sam/sam.h"
#include "../../drivers/video/video.h"
#include "../../drivers/audio/audio.h"
#include "../../drivers/mouse.h"
#include "../raytrace/raytrace.h"
#include "../piano/piano.h"
#include "shell.h"

/* ── Small utilities ─────────────────────────────────────────────────────── */

static int k_streq(const char *a, const char *b) {
    while (*a && *b && *a == *b) { a++; b++; }
    return *a == '\0' && *b == '\0';
}

static int k_strlen(const char *s) {
    int n = 0; while (s[n]) n++; return n;
}

static int k_strpfx(const char *s, const char *pfx) {
    while (*pfx) { if (*s++ != *pfx++) return 0; }
    return 1;
}

/* Print a 2-digit hex byte without leading 0x. */
static void print_hex8(uint8_t b) {
    static const char h[] = "0123456789abcdef";
    term_putchar(h[b >> 4]);
    term_putchar(h[b & 0xf]);
}

static void print_hex32(uint32_t v) {
    print_hex8((uint8_t)(v >> 24));
    print_hex8((uint8_t)(v >> 16));
    print_hex8((uint8_t)(v >> 8));
    print_hex8((uint8_t)v);
}

/* Print an unsigned byte as decimal (no leading zeros). */
static void print_u8(uint8_t v) {
    if (v >= 100) term_putchar('0' + (char)(v / 100));
    if (v >=  10) term_putchar('0' + (char)((v / 10) % 10));
    term_putchar('0' + (char)(v % 10));
}

/* Print a uint32 as decimal. */
static void print_u32(uint32_t v) {
    char buf[11];
    int  i = 10;
    buf[i] = '\0';
    if (v == 0) { buf[--i] = '0'; }
    else { while (v) { buf[--i] = '0' + (char)(v % 10); v /= 10; } }
    term_puts(buf + i);
}

static void print_mac(const uint8_t m[6]) {
    for (int i = 0; i < 6; i++) {
        if (i) term_putchar(':');
        print_hex8(m[i]);
    }
}

static void print_ip(const uint8_t ip[4]) {
    for (int i = 0; i < 4; i++) {
        if (i) term_putchar('.');
        print_u8(ip[i]);
    }
}

static void print_ip32(uint32_t ip) {
    uint8_t b[4] = {
        (uint8_t)(ip >> 24), (uint8_t)(ip >> 16),
        (uint8_t)(ip >> 8),  (uint8_t)ip
    };
    print_ip(b);
}

static void print_tcp_diag(const struct tcp_connect_diag *d) {
    term_puts("diag: attempts=");
    print_u32(d->attempts);
    term_puts(" sport=");
    print_u32(d->src_port);
    term_puts(" tcp_rx=");
    print_u32(d->rx_candidates);
    term_puts(" match=");
    print_u32(d->rx_packets);
    if (d->rx_candidates) {
        term_puts(" last=");
        print_ip32(d->last_src_ip);
        term_putchar(':');
        print_u32(d->last_src_port);
        term_puts("->");
        print_u32(d->last_dst_port);
        term_puts(" len=");
        print_u32(d->last_seg_len);
        term_puts(" flags=0x");
        print_hex8(d->last_flags);
        term_puts(" ack=0x");
        print_hex32(d->last_ack);
        term_puts(" drop=");
        print_u32(d->last_drop);
    }
    term_putchar('\n');
}

/* Parse "a.b.c.d" → uint32_t in host byte order. Returns 0 on bad input. */
static uint32_t parse_ip4(const char *s) {
    uint32_t ip = 0;
    for (int i = 0; i < 4; i++) {
        uint32_t b = 0;
        if (*s < '0' || *s > '9') return 0;
        while (*s >= '0' && *s <= '9') b = b*10 + (uint32_t)(*s++ - '0');
        if (i < 3) { if (*s != '.') return 0; s++; }
        if (b > 255) return 0;
        ip = (ip << 8) | b;
    }
    return ip;
}

/* ── readline ────────────────────────────────────────────────────────────── */

static int readline(char *buf, int size) {
    int len = 0;
    for (;;) {
        int c = term_getchar();
        if (c == '\n' || c == '\r') { term_putchar('\n'); buf[len] = '\0'; return len; }
        if ((c == '\b' || c == 127) && len > 0) { len--; term_putchar('\b'); continue; }
        if (c >= 32 && c <= 126 && len < size - 1) { buf[len++] = (char)c; term_putchar((char)c); }
    }
}

/* ── Built-in: help ──────────────────────────────────────────────────────── */

static const char * const s_help_lines[] = {
    "Commands:\n",
    "  help             show this message\n",
    "  clear            clear the screen\n",
    "  echo <text>      echo arguments\n",
    "  cpus             list all logical CPUs\n",
    "  meminfo          show heap usage\n",
    "  netinfo          show NIC driver, status, MAC, and IPv4 config\n",
    "  netreset         reset NIC binding, IPv4/ARP/TCP, and iwlwifi state\n",
    "  wifiup           full bring-up: init → load → NVM → RT → setup\n",
    "  wifivers         dump fw-advertised cmd versions for our key cmds\n",
    "  wifiscan         active 2.4 GHz scan (chans 1,6,11; reveals hidden SSIDs)\n",
    "  wifilist         dump cached APs from the last wifiscan\n",
    "  wificonnect <ssid>|<index> [password]  associate/WPA connect\n",
    "  wifiinit         (granular) PCI probe only\n",
    "  wifiload         (granular) firmware upload + wait for ALIVE\n",
    "  wifiecho         (granular) first post-ALIVE host command\n",
    "  wifinvm          (granular) unified init through NVM_GET_INFO\n",
    "  wifirt           (granular) first runtime setup commands\n",
    "  wifisetup        (granular) PHY_CONTEXT (toward scan)\n",
    "  wifitxq          (granular) allocate AUX STA + TX queue (used by wifiscan)\n",
    "  wifiinfo         show iwlwifi chip revision + hardware MAC\n",
    "  dhcp             obtain IPv4 config from the LAN\n",
    "  arp [ip]         send ARP who-has ip (default: gateway)\n",
    "  ping <ip>        ICMP echo to ip (e.g. 10.0.2.2)\n",
    "  tcpcheck <ip>[:port]  test a TCP connect (default port 80; ; also works)\n",
    "  http <ip> <path> HTTP/1.0 GET (e.g. http 93.184.216.34 /)\n",
    "  videoinfo        show video driver and resolution\n",
    "  videodemo        draw a colour bar on the framebuffer\n",
    "  audioinfo        show audio driver name\n",
    "  beep [freq [ms]] play a beep (default: 880 Hz, 200 ms)\n",
    "  raytrace         render scene using all CPUs\n",
    "  raytrace slow    render single-core then all-core, print speedup\n",
    "  piano            88-key interactive piano (mouse + click)\n",
    "  reboot           restart the machine\n",
    "  halt             halt the CPU\n",
};

static int help_render_rows(const char *s) {
    int cols = term_cols();
    if (cols <= 0) cols = 80;

    int rows = 0;
    int col = 0;
    while (*s) {
        unsigned char ch = (unsigned char)*s++;
        if (ch == '\n') {
            rows++;
            col = 0;
            continue;
        }
        if (ch == '\r') {
            col = 0;
            continue;
        }
        if (ch == '\t') {
            int spaces = 8 - (col & 7);
            while (spaces--) {
                col++;
                if (col >= cols) { rows++; col = 0; }
            }
            continue;
        }
        if (ch < 32 || ch > 126)
            continue;
        col++;
        if (col >= cols) { rows++; col = 0; }
    }
    if (col) rows++;
    return rows;
}

static int help_more(void) {
    term_puts("-- more -- (space/enter continues, q quits)");
    int c = term_getchar();
    term_putchar('\n');
    return c != 'q' && c != 'Q' && c != 27;
}

static void cmd_help(void) {
    int page_rows = term_rows() - 3;
    if (page_rows < 4) page_rows = 20;
    int rows_left = page_rows;

    for (uint32_t i = 0; i < sizeof(s_help_lines) / sizeof(s_help_lines[0]); i++) {
        int rows = help_render_rows(s_help_lines[i]);
        if (rows_left < rows) {
            if (!help_more()) return;
            rows_left = page_rows;
        }
        term_puts(s_help_lines[i]);
        rows_left -= rows;
    }
}

/* ── Built-in: meminfo ───────────────────────────────────────────────────── */

/* Print a byte count in human-readable form: e.g. 268431360 → "256 MiB". */
static void print_bytes(uint64_t n) {
    if (n >= 1024u*1024u*1024u) {
        print_u32((uint32_t)(n >> 30));
        term_puts(" GiB");
    } else if (n >= 1024u*1024u) {
        print_u32((uint32_t)(n >> 20));
        term_puts(" MiB");
    } else if (n >= 1024u) {
        print_u32((uint32_t)(n >> 10));
        term_puts(" KiB");
    } else {
        print_u32((uint32_t)n);
        term_puts(" B");
    }
}

static void cmd_meminfo(void) {
    uint64_t used, free, total;
    kalloc_stats(&used, &free, &total);
    term_puts("Heap total:  "); print_bytes(total); term_putchar('\n');
    term_puts("     used:   "); print_bytes(used);  term_putchar('\n');
    term_puts("     free:   "); print_bytes(free);  term_putchar('\n');
}

/* ── Built-in: cpus ──────────────────────────────────────────────────────── */

static void cmd_cpus(void) {
    int n = smp_cpu_count();
    term_puts("Logical CPUs: ");
    print_u32((uint32_t)n);
    term_putchar('\n');
    for (int i = 0; i < n; i++) {
        term_puts("  CPU ");
        print_u32((uint32_t)i);
        term_puts("  LAPIC 0x");
        print_hex8((uint8_t)smp_cpu_lapic(i));
        if (i == smp_this_cpu()) term_puts("  <-- this core");
        term_putchar('\n');
    }
}

/* ── Built-in: netinfo ───────────────────────────────────────────────────── */

static void cmd_netinfo(void) {
    const char *drv = net_driver_name();
    term_puts("Driver: "); term_puts(drv); term_putchar('\n');

    if (k_streq(drv, "none")) {
        term_puts("  No NIC detected.\n");
        term_puts("  Supported: -device e1000  e1000e  virtio-net-pci  rtl8139  pcnet  ne2k_pci\n");
        return;
    }

    uint8_t mac[6] = {0};
    net_mac(mac);
    term_puts("  MAC:  "); print_mac(mac); term_putchar('\n');
    term_puts("  Link: "); term_puts(net_link_up() ? "UP\n" : "DOWN\n");

    struct net_ipv4_config cfg;
    net_config_get(&cfg);
    term_puts("  IPv4: "); print_ip32(cfg.ip); term_putchar('\n');
    term_puts("  Mask: "); print_ip32(cfg.netmask); term_putchar('\n');
    term_puts("  GW:   "); print_ip32(cfg.gateway); term_putchar('\n');
    term_puts("  DNS:  ");
    if (cfg.dns) print_ip32(cfg.dns); else term_puts("<none>");
    term_putchar('\n');
}

static void cmd_netreset(void) {
    term_puts("net: reset start\n");
    net_detach_driver();
    net_stack_reset();
    iwl_reset();
    term_puts("net: reset OK — run wifiup next\n");
}

/* ── Built-in: wifiinit (on-demand iwlwifi probe) ──────────────────────── */

extern uint64_t g_hhdm_offset, g_kphys, g_kvirt;

static void cmd_wifiinit(void) {
    if (iwl_probe(g_hhdm_offset, g_kphys, g_kvirt)) {
        term_puts("wifi: probe OK\n");
    } else {
        term_puts("wifi: probe FAILED\n");
    }
}

static void cmd_wifiload(void) {
    const struct iwl_fw_info *fw = iwl_fw_info_get();
    if (!fw || !fw->present) {
        term_puts("wifi: run 'wifiinit' first (and ensure iwlwifi.ucode is loaded)\n");
        return;
    }
    if (iwl_fw_load(iwl_hw_rev(), fw))
        term_puts("wifi: firmware load OK\n");
    else
        term_puts("wifi: firmware load FAILED\n");
}

static void cmd_wifiecho(void) {
    if (iwl_cmd_echo_test())
        term_puts("wifi: host command OK\n");
    else
        term_puts("wifi: host command FAILED (run wifiload first?)\n");
}

static void cmd_wifinvm(void) {
    if (iwl_cmd_nvm_probe())
        term_puts("wifi: NVM probe OK\n");
    else
        term_puts("wifi: NVM probe FAILED (run wifiload first?)\n");
}

static void cmd_wifirt(void) {
    if (iwl_cmd_runtime_probe())
        term_puts("wifi: runtime probe OK\n");
    else
        term_puts("wifi: runtime probe FAILED (run wifiload first?)\n");
}

static void cmd_wifisetup(void) {
    if (iwl_cmd_scan_setup_probe())
        term_puts("wifi: scan setup OK\n");
    else
        term_puts("wifi: scan setup FAILED\n");
}

static void cmd_wifitxq(void) {
    if (iwl_cmd_txq_setup_probe())
        term_puts("wifi: TX queue OK\n");
    else
        term_puts("wifi: TX queue FAILED\n");
}

/* Run every bring-up step from PCI probe through scan-setup in sequence.
 * Bails on the first failure.  Each phase prints its own diagnostic, so
 * the operator sees exactly where (if anywhere) bring-up fails. */
static void cmd_wifiup(void) {
    term_puts("--- wifi: PCI probe ---\n");
    if (!iwl_probe(g_hhdm_offset, g_kphys, g_kvirt)) {
        term_puts("wifi: probe FAILED\n"); return;
    }
    const struct iwl_fw_info *fw = iwl_fw_info_get();
    if (!fw || !fw->present) {
        term_puts("wifi: no firmware (iwlwifi.ucode missing?)\n"); return;
    }
    term_puts("--- wifi: firmware load ---\n");
    if (!iwl_fw_load(iwl_hw_rev(), fw)) {
        term_puts("wifi: firmware load FAILED\n"); return;
    }
    term_puts("--- wifi: NVM probe ---\n");
    if (!iwl_cmd_nvm_probe()) {
        term_puts("wifi: NVM probe FAILED\n"); return;
    }
    term_puts("--- wifi: runtime probe ---\n");
    if (!iwl_cmd_runtime_probe()) {
        term_puts("wifi: runtime probe FAILED\n"); return;
    }
    term_puts("--- wifi: scan setup ---\n");
    if (!iwl_cmd_scan_setup_probe()) {
        term_puts("wifi: scan setup FAILED\n"); return;
    }
    term_puts("wifi: UP — ready for wifiscan\n");
}

static void cmd_wifivers(void) {
    const struct iwl_fw_info *fw = iwl_fw_info_get();
    if (!fw || !fw->present) {
        term_puts("wifi: no firmware (run wifiload first)\n");
        return;
    }
    term_puts("wifi: fw advertises "); print_u32(fw->n_cmd_vers);
    term_puts(" cmd-version entries\n");

    /* Print the ones that matter for our bring-up so we don't drown in
     * 200+ lines.  Format: g=GG c=CC ver=VV notif=NN */
    static const struct { uint8_t g; uint8_t c; const char *name; } watch[] = {
        { 0x00, 0x03, "ECHO" },
        { 0x02, 0x03, "INIT_EXTENDED_CFG" },
        { 0x02, 0x00, "SHARED_MEM_CFG" },
        { 0x0C, 0x02, "NVM_GET_INFO" },
        { 0x01, 0x98, "TX_ANT_CFG" },
        { 0x01, 0xD1, "REPLY_SF_CFG" },
        { 0x01, 0x9B, "BT_CONFIG" },
        { 0x01, 0x08, "PHY_CONTEXT" },
        { 0x01, 0x28, "MAC_CONTEXT" },
        { 0x01, 0x18, "ADD_STA" },
        { 0x01, 0x0C, "SCAN_CFG" },
        { 0x01, 0x0D, "SCAN_REQ_UMAC" },
        { 0x01, 0x0F, "SCAN_COMPLETE_UMAC" },
        { 0x01, 0xB5, "SCAN_ITER_COMPLETE" },
        { 0x03, 0x05, "SESSION_PROTECTION" },
        { 0x03, 0x08, "MAC_CONFIG" },
        { 0x03, 0x09, "LINK_CONFIG" },
        { 0x03, 0x0A, "STA_CONFIG" },
        { 0x03, 0x0B, "AUX_STA" },
        { 0x05, 0x08, "RLC_CONFIG" },
        { 0x05, 0x0F, "TLC_MNG_CONFIG" },
        { 0x05, 0x00, "DQA_ENABLE" },
        { 0x00, 0x1D, "SCD_QUEUE_CFG (legacy gen2 path)" },
        { 0x05, 0x17, "SCD_QUEUE_CONFIG (gen3 path)" },
        { 0x00, 0x1C, "TX_CMD short-header" },
        { 0x01, 0x1C, "TX_CMD LONG TLV" },
    };
    for (unsigned i = 0; i < sizeof(watch) / sizeof(watch[0]); i++) {
        uint8_t v = iwl_fw_cmd_ver(fw, watch[i].g, watch[i].c);
        term_puts("  "); term_puts(watch[i].name);
        term_puts("  g=0x"); print_hex8(watch[i].g);
        term_puts(" c=0x"); print_hex8(watch[i].c);
        term_puts(" ver=");
        if (v == 0xFF) term_puts("--");
        else { term_puts("0x"); print_hex8(v); }
        term_putchar('\n');
    }
}

static void cmd_wifiscan(void) {
    if (k_streq(net_driver_name(), "iwlwifi") && net_link_up()) {
        term_puts("wifi: link is up; run netreset before scanning again\n");
        return;
    }
    if (iwl_cmd_scan_probe())
        term_puts("wifi: scan OK\n");
    else
        term_puts("wifi: scan FAILED\n");
}

/* ── Built-in: wifilist ──────────────────────────────────────────────────── */
/* Dump the cached APs from the most recent scan: BSSID, channel, RSSI,
 * 16-bit capability, count of advertised rates, and SSID (or `(hidden)`).  */
static void cmd_wifilist(void) {
    int n = iwl_ap_count();
    if (n == 0) {
        term_puts("wifi: no APs cached — run wifiscan first\n");
        return;
    }
    term_puts("# BSSID              ch rssi  cap n  ssid\n");
    for (int i = 0; i < n; i++) {
        const struct iwl_ap_info *a = iwl_ap_get(i);
        if (!a) break;
        print_hex8((uint8_t)i);
        term_putchar(' ');
        print_mac(a->bssid);
        term_putchar(' ');
        print_hex8(a->channel);
        term_puts(" -");
        print_hex8(a->rssi);
        term_puts(" ");
        print_hex8((uint8_t)(a->capability >> 8));
        print_hex8((uint8_t)(a->capability & 0xFFu));
        term_puts(" ");
        print_hex8(a->rates_len);
        term_putchar(' ');
        if (a->ssid_len) {
            term_putchar('"');
            for (uint8_t j = 0; j < a->ssid_len; j++) {
                uint8_t c = a->ssid[j];
                if (c >= 0x20 && c < 0x7f) term_putchar((char)c);
                else                       term_putchar('.');
            }
            term_putchar('"');
        } else {
            term_puts("(hidden)");
        }
        term_putchar('\n');
    }
}

/* ── Built-in: wificonnect <ssid> [<password>] ───────────────────────────── */
/* Look up <ssid> in the wifilist cache, then drive association and, if a
 * password is supplied, the first WPA2 4-way handshake response. */
static void cmd_wificonnect(const char *args) {
    /* Skip leading whitespace. */
    while (*args == ' ' || *args == '\t') args++;

    /* Quote-strip: if the SSID is wrapped in matching ' or ", drop them. */
    char quote = 0;
    if (*args == '\'' || *args == '"') { quote = *args; args++; }

    const char *ssid = args;
    uint8_t ssid_len = 0;
    while (args[ssid_len] && ssid_len < 32) {
        char c = args[ssid_len];
        if (quote) { if (c == quote) break; }
        else { if (c == ' ' || c == '\t' || c == '\n' || c == '\r') break; }
        ssid_len++;
    }
    if (ssid_len == 0) {
        term_puts("usage: wificonnect <ssid>|<index> [<password>]\n"
                  "  <index> is the row number from wifilist (0-based)\n");
        return;
    }
    uint8_t token_len = ssid_len;
    const char *pw = args + token_len;
    if (quote && *pw == quote) pw++;
    while (*pw == ' ' || *pw == '\t') pw++;

    /* If the argument is purely digits, treat it as an index into the AP
     * cache.  Lets the operator pick an AP without typing the SSID — handy
     * when special chars or quoting get in the way. */
    int is_num = 1;
    int idx = 0;
    for (uint8_t i = 0; i < ssid_len; i++) {
        char c = ssid[i];
        if (c < '0' || c > '9') { is_num = 0; break; }
        idx = idx * 10 + (c - '0');
    }
    if (is_num) {
        const struct iwl_ap_info *a = iwl_ap_get(idx);
        if (!a) {
            term_puts("wifi: index out of range (run wifilist first)\n");
            return;
        }
        ssid     = (const char *)a->ssid;
        ssid_len = a->ssid_len;
        if (ssid_len == 0) {
            term_puts("wifi: that AP has a hidden SSID — type it explicitly\n");
            return;
        }
        term_puts("wifi: connecting to \"");
        for (uint8_t i = 0; i < ssid_len; i++) term_putchar(ssid[i]);
        term_puts("\"\n");
    }

    uint8_t pw_len = 0;
    if (*pw) {
        while (pw[pw_len] && pw[pw_len] != '\n' && pw[pw_len] != '\r' &&
               pw_len < 63)
            pw_len++;
        while (pw_len > 0 && (pw[pw_len - 1] == ' ' ||
                              pw[pw_len - 1] == '\t'))
            pw_len--;
        term_puts("wifi: password noted; will try WPA2 4-way handshake\n");
    }

    if (iwl_cmd_assoc_probe(ssid, ssid_len, pw, pw_len)) {
        if (net_link_up())
            term_puts("wifi: connected (WPA complete, link up)\n");
        else
            term_puts("wifi: ASSOC response accepted (no WPA link yet)\n");
    } else {
        term_puts("wifi: assoc FAILED — see iwl: lines above\n");
    }
}

/* ── Built-in: wifiinfo ──────────────────────────────────────────────────── */

static void cmd_wifiinfo(void) {
    uint32_t rev = iwl_hw_rev();
    term_puts("Family:  "); term_puts(iwl_family_name()); term_putchar('\n');
    term_puts("HW_REV:  0x");
    {
        static const char hex[] = "0123456789abcdef";
        for (int i = 28; i >= 0; i -= 4) term_putchar(hex[(rev >> i) & 0xF]);
    }
    term_putchar('\n');
    uint8_t mac[6] = {0};
    iwl_mac(mac);
    term_puts("MAC:     "); print_mac(mac); term_putchar('\n');

    const struct iwl_fw_info *fw = iwl_fw_info_get();
    if (!fw || !fw->present) {
        term_puts("FW:      <not embedded — run tools/iwl_diag.sh>\n");
        return;
    }
    term_puts("FW:      "); term_puts(fw->human); term_putchar('\n');
    term_puts("FW size: ");
    print_u32(fw->raw_size);
    term_puts(" bytes, "); print_u32(fw->num_cpus); term_puts(" cpu(s)\n");
    static const char *slot_name[IWL_FW_IMG_COUNT] = {"init", "regular", "wowlan"};
    for (int i = 0; i < IWL_FW_IMG_COUNT; i++) {
        uint32_t tot = 0;
        for (int j = 0; j < fw->img[i].n_sections; j++)
            tot += fw->img[i].sections[j].len;
        term_puts("  "); term_puts(slot_name[i]); term_puts(": ");
        print_u32(fw->img[i].n_sections); term_puts(" sec, ");
        print_u32(tot); term_puts(" bytes\n");
    }
}

/* ── Built-in: dhcp / arp ────────────────────────────────────────────────── */

static void cmd_dhcp(void) {
    const char *drv = net_driver_name();
    if (k_streq(drv, "none")) { term_puts("No NIC detected.\n"); return; }
    if (!net_link_up()) { term_puts("Link is down.\n"); return; }

    term_puts("DHCP discover/request ...\n");
    struct net_ipv4_config cfg;
    int r = dhcp_configure(&cfg);
    if (r > 0) {
        term_puts("  IPv4: "); print_ip32(cfg.ip); term_putchar('\n');
        term_puts("  Mask: "); print_ip32(cfg.netmask); term_putchar('\n');
        term_puts("  GW:   "); print_ip32(cfg.gateway); term_putchar('\n');
        term_puts("  DNS:  ");
        if (cfg.dns) print_ip32(cfg.dns); else term_puts("<none>");
        term_putchar('\n');
    } else if (r < 0) {
        term_puts("  DHCP failed: link/send error.\n");
    } else {
        term_puts("  DHCP timeout/no ACK.\n");
    }
}

static void cmd_arp(const char *args) {
    const char *drv = net_driver_name();
    if (k_streq(drv, "none")) { term_puts("No NIC detected.\n"); return; }

    struct net_ipv4_config cfg;
    net_config_get(&cfg);
    if (!cfg.ip) { term_puts("No IPv4 address; run dhcp first.\n"); return; }

    while (*args == ' ') args++;
    uint32_t target_ip = cfg.gateway;
    if (*args) {
        target_ip = parse_ip4(args);
        if (!target_ip) { term_puts("Bad IP address.\n"); return; }
    }
    if (!target_ip) { term_puts("No gateway configured; use arp <ip>.\n"); return; }

    term_puts("Sending ARP who-has ");
    print_ip32(target_ip);
    term_puts(" ...\n");

    uint8_t mac[6];
    if (arp_resolve(target_ip, mac)) {
        term_puts("  Reply: ");
        print_ip32(target_ip);
        term_puts(" is at ");
        print_mac(mac);
        term_putchar('\n');
        return;
    }
    term_puts("  No reply received.\n");
}

/* ── Built-in: videoinfo ─────────────────────────────────────────────────── */

static void cmd_videoinfo(void) {
    term_puts("Driver: "); term_puts(video_driver_name()); term_putchar('\n');
    term_puts("  Width:  "); print_u32(video_width());  term_putchar('\n');
    term_puts("  Height: "); print_u32(video_height()); term_putchar('\n');
}

/* ── Built-in: videodemo ─────────────────────────────────────────────────── */

static void cmd_videodemo(void) {
    uint32_t w = video_width();
    uint32_t h = video_height();
    if (w == 0 || h == 0) { term_puts("No framebuffer available.\n"); return; }

    /* Draw a simple colour-bar test pattern */
    static const uint32_t colours[] = {
        0xFFFFFFu, 0xFF0000u, 0xFF8000u, 0xFFFF00u,
        0x00FF00u, 0x00FFFFu, 0x0000FFu, 0xFF00FFu,
    };
    uint32_t n_bars = 8;
    uint32_t bar_w  = w / n_bars;

    for (uint32_t i = 0; i < n_bars; i++) {
        uint32_t x0 = i * bar_w;
        uint32_t x1 = (i == n_bars - 1) ? w : x0 + bar_w;
        video_rect(x0, 0, x1 - x0, h / 4, colours[i]);
    }
    /* White border around colour bars */
    video_rect_border(0, 0, (int)w, (int)(h / 4), 0xFFFFFFu);

    term_puts("Colour bars drawn in the top quarter of the screen.\n");
    term_puts("Press any key to continue...\n");
    (void)term_getchar();
}

/* ── Built-in: audioinfo ─────────────────────────────────────────────────── */

static void cmd_audioinfo(void) {
    term_puts("Driver: "); term_puts(audio_driver_name()); term_putchar('\n');
}

/* ── Built-in: beep ──────────────────────────────────────────────────────── */

/* Parse a simple unsigned integer from a string; returns 0 on failure. */
static uint32_t parse_u32(const char *s) {
    uint32_t v = 0;
    if (!*s) return 0;
    while (*s >= '0' && *s <= '9') { v = v * 10u + (uint32_t)(*s - '0'); s++; }
    return v;
}

static void cmd_beep(const char *args) {
    uint32_t freq = 880u;
    uint32_t ms   = 200u;

    /* Skip leading spaces */
    while (*args == ' ') args++;
    if (*args) {
        uint32_t f = parse_u32(args);
        if (f) { freq = f; while (*args >= '0' && *args <= '9') args++; }
        while (*args == ' ') args++;
        if (*args) {
            uint32_t m = parse_u32(args);
            if (m) ms = m;
        }
    }

    term_puts("Beep "); print_u32(freq); term_puts(" Hz, ");
    print_u32(ms); term_puts(" ms\n");
    audio_beep(freq, ms);
}

/* ── Built-in: ping ──────────────────────────────────────────────────────── */

static void cmd_ping(const char *args) {
    while (*args == ' ') args++;
    if (!*args) { term_puts("Usage: ping <ip>\n"); return; }

    uint32_t ip = parse_ip4(args);
    if (!ip) { term_puts("Bad IP address.\n"); return; }

    uint8_t a = (uint8_t)(ip >> 24), b = (uint8_t)(ip >> 16),
            c = (uint8_t)(ip >>  8), d = (uint8_t)ip;
    term_puts("PING ");
    print_u8(a); term_putchar('.');
    print_u8(b); term_putchar('.');
    print_u8(c); term_putchar('.');
    print_u8(d); term_puts(" ... ");

    int r = ping(ip);
    if (r > 0) term_puts("alive\n");
    else if (r == -2) term_puts("send failed (ARP unresolved?)\n");
    else term_puts("no reply\n");
}

/* ── Built-in: tcpcheck ──────────────────────────────────────────────────── */

static void cmd_tcpcheck(const char *args) {
    while (*args == ' ') args++;
    if (!*args) { term_puts("Usage: tcpcheck <ip>[:port]\n"); return; }

    uint32_t ip = parse_ip4(args);
    if (!ip) { term_puts("Bad IP address.\n"); return; }
    while (*args && *args != ':' && *args != ';' && *args != ' ') args++;

    uint16_t port = 80;
    if (*args == ':' || *args == ';') {
        args++;
        uint32_t p = parse_u32(args);
        if (p && p < 65536) port = (uint16_t)p;
    } else {
        while (*args == ' ') args++;
        if (*args) {
            uint32_t p = parse_u32(args);
            if (p && p < 65536) port = (uint16_t)p;
        }
    }

    term_puts("TCP connect ");
    print_ip32(ip);
    term_putchar(':');
    print_u32(port);
    term_puts(" ... ");

    tcp_conn_t *c = tcp_connect(ip, port);
    if (!c) {
        struct tcp_connect_diag d;
        tcp_last_connect_diag(&d);
        if (d.status == TCP_CONNECT_NO_IP) {
            term_puts("failed (no IPv4; run dhcp first)\n");
        } else if (d.status == TCP_CONNECT_SEND_FAILED) {
            term_puts("failed (send/ARP error)\n");
        } else if (d.status == TCP_CONNECT_RESET) {
            term_puts("refused/reset");
            if (d.rx_packets) {
                term_puts(" flags=0x"); print_hex8(d.last_flags);
            }
            term_putchar('\n');
        } else if (d.rx_packets) {
            term_puts("failed (unexpected TCP rx flags=0x");
            print_hex8(d.last_flags);
            term_puts(" ack=0x");
            print_hex32(d.last_ack);
            term_puts(")\n");
        } else if (d.rx_candidates) {
            term_puts("failed (TCP rx did not match connect)\n");
        } else {
            term_puts("timeout/no TCP response\n");
        }
        print_tcp_diag(&d);
        return;
    }
    term_puts("ok\n");
    tcp_close(c);
}

/* ── Built-in: http ──────────────────────────────────────────────────────── */

/* Shared response buffer (large; lives in BSS). */
static char s_http_buf[32768];

static void cmd_http(const char *args) {
    /* Syntax: http <ip>[:<port>] <path>
     * Example: http 93.184.216.34 /
     *          http 10.0.2.2:8080 /index.html       */
    while (*args == ' ') args++;
    if (!*args) {
        term_puts("Usage: http <ip>[:<port>] <path>\n");
        term_puts("  Example: http 93.184.216.34 /\n");
        return;
    }

    /* Parse IP */
    uint32_t ip = parse_ip4(args);
    if (!ip) { term_puts("Bad IP address.\n"); return; }
    while (*args && *args != ':' && *args != ';' && *args != ' ') args++;

    /* Parse optional :port */
    uint16_t port = 80;
    if (*args == ':' || *args == ';') {
        args++;
        uint32_t p = parse_u32(args);
        if (p && p < 65536) port = (uint16_t)p;
        while (*args >= '0' && *args <= '9') args++;
    }

    /* Path */
    while (*args == ' ') args++;
    const char *path = (*args) ? args : "/";

    /* Build host string from IP for the Host header */
    char host[20];
    uint8_t ha=(uint8_t)(ip>>24), hb=(uint8_t)(ip>>16),
            hc=(uint8_t)(ip>>8),  hd=(uint8_t)ip;
    /* Simple IP-to-string */
    char *hp = host;
    static const char digits[] = "0123456789";
    uint8_t parts[4] = {ha,hb,hc,hd};
    for (int i = 0; i < 4; i++) {
        if (i) *hp++ = '.';
        uint8_t v = parts[i];
        if (v >= 100) *hp++ = digits[v/100];
        if (v >=  10) *hp++ = digits[(v/10)%10];
        *hp++ = digits[v%10];
    }
    *hp = '\0';

    term_puts("HTTP GET http://"); term_puts(host);
    if (port != 80) { term_putchar(':'); print_u32(port); }
    term_puts(path); term_putchar('\n');

    uint32_t resp_len = 0;
    int status = http_get(ip, port, host, path,
                          s_http_buf, sizeof(s_http_buf), &resp_len);
    if (status < 0) {
        term_puts("Connection failed.\n");
        return;
    }

    term_puts("HTTP "); print_u32((uint32_t)status);
    term_puts(" ("); print_u32(resp_len); term_puts(" bytes)\n");

    /* Print up to first 2 KiB of the response */
    uint32_t print_len = resp_len < 2048 ? resp_len : 2048;
    for (uint32_t i = 0; i < print_len; i++) {
        char ch = s_http_buf[i];
        if (ch == '\r') continue;   /* strip bare CRs */
        term_putchar(ch);
    }
    if (resp_len > 2048) {
        term_puts("\n... (truncated, ");
        print_u32(resp_len - 2048);
        term_puts(" more bytes)\n");
    }
}

/* ── Shell entry point ───────────────────────────────────────────────────── */

void shell_run(void) {
    term_puts("Diamond OS\n");
    term_puts("Type 'help' for available commands.\n");
    term_putchar('\n');

    char buf[256];
    for (;;) {
        term_puts("Command> ");
        int len = readline(buf, (int)sizeof(buf));
        if (len == 0) continue;

        if (k_streq(buf, "help")) {
            cmd_help();
        } else if (k_streq(buf, "clear")) {
            term_clear();
        } else if (k_strlen(buf) >= 4 && k_strpfx(buf, "echo")) {
            const char *arg = buf + 4;
            while (*arg == ' ') arg++;
            term_puts(arg);
            term_putchar('\n');
        } else if (k_streq(buf, "cpus")) {
            cmd_cpus();
        } else if (k_streq(buf, "meminfo")) {
            cmd_meminfo();
        } else if (k_streq(buf, "netinfo")) {
            cmd_netinfo();
        } else if (k_streq(buf, "netreset") || k_streq(buf, "wifireset")) {
            cmd_netreset();
        } else if (k_streq(buf, "dhcp")) {
            cmd_dhcp();
        } else if (k_streq(buf, "wifiinit")) {
            cmd_wifiinit();
        } else if (k_streq(buf, "wifiload")) {
            cmd_wifiload();
        } else if (k_streq(buf, "wifiecho")) {
            cmd_wifiecho();
        } else if (k_streq(buf, "wifinvm")) {
            cmd_wifinvm();
        } else if (k_streq(buf, "wifirt")) {
            cmd_wifirt();
        } else if (k_streq(buf, "wifisetup")) {
            cmd_wifisetup();
        } else if (k_streq(buf, "wifitxq")) {
            cmd_wifitxq();
        } else if (k_streq(buf, "wifiup")) {
            cmd_wifiup();
        } else if (k_streq(buf, "wifivers")) {
            cmd_wifivers();
        } else if (k_streq(buf, "wifiscan")) {
            cmd_wifiscan();
        } else if (k_streq(buf, "samdbg")) {
            static int dbg = 0;
            dbg = !dbg;
            sam_set_dbg(dbg);
            term_puts(dbg ? "sam: debug ON (try shift+key)\n" : "sam: debug OFF\n");
        } else if (k_streq(buf, "wifilist")) {
            cmd_wifilist();
        } else if (k_strpfx(buf, "wificonnect")) {
            cmd_wificonnect(buf + 11);
        } else if (k_streq(buf, "wifiinfo")) {
            cmd_wifiinfo();
        } else if (k_streq(buf, "arp") || (k_strlen(buf) >= 4 && k_strpfx(buf, "arp "))) {
            cmd_arp(buf + 3);
        } else if (k_strpfx(buf, "ping")) {
            cmd_ping(buf + 4);
        } else if (k_strpfx(buf, "tcpcheck")) {
            cmd_tcpcheck(buf + 8);
        } else if (k_strpfx(buf, "http")) {
            cmd_http(buf + 4);
        } else if (k_streq(buf, "videoinfo")) {
            cmd_videoinfo();
        } else if (k_streq(buf, "videodemo")) {
            cmd_videodemo();
        } else if (k_streq(buf, "audioinfo")) {
            cmd_audioinfo();
        } else if (k_strpfx(buf, "beep")) {
            cmd_beep(buf + 4);
        } else if (k_strpfx(buf, "raytrace")) {
            const char *arg = buf + 8;
            while (*arg == ' ') arg++;
            raytrace_run(k_streq(arg, "slow"));
        } else if (k_streq(buf, "piano")) {
            piano_run();
        } else if (k_streq(buf, "reboot")) {
            term_puts("Rebooting...\n");
            __asm__ volatile ("cli");

            /* 1. Intel PCH reset register at I/O port 0xCF9.
             *     bit 1 = RCIN     (pulse this bit triggers the reset)
             *     bit 2 = SYS_RST  (0=warm / 1=hard)
             *     bit 3 = FULL_RST (full chipset+memory reset)
             * Arm with SYS_RST first, then pulse with FULL_RST | SYS_RST | RCIN.
             * Retry the pulse several times — some ICL-LP chipsets need more
             * than one pulse to actually trigger, and a ~1 ms delay between
             * writes for the bit change to latch.                          */
            {
                uint8_t cf9 = inb(0xCF9) & ~(uint8_t)0x0E;
                for (int tries = 0; tries < 8; tries++) {
                    outb(0xCF9, cf9 | 0x02);    /* arm: SYS_RST */
                    for (volatile int i = 0; i < 500000; i++) ;  /* ~1 ms */
                    outb(0xCF9, cf9 | 0x0E);    /* fire: FULL_RST+SYS_RST+RCIN */
                    for (volatile int i = 0; i < 500000; i++) ;
                }
            }

            /* 2. Legacy 8042 keyboard-controller reset (no-op if absent). */
            for (int i = 0; i < 1000 && (inb(0x64) & 0x02); i++)
                (void)inb(0x60);
            outb(0x64, 0xFE);
            for (volatile int i = 0; i < 500000; i++) {
            }

            /* 3. Triple fault: load a null IDT and force an invalid-opcode
             * exception.  With no IDT, the CPU can't dispatch it, faults
             * while trying to fault, and the final shutdown cycle tells
             * the chipset to reset.  UD2 is guaranteed-illegal; INT3
             * sometimes gets swallowed by firmware debug hooks.           */
            {
                struct {
                    uint16_t lim;
                    uint64_t base;
                } __attribute__((packed)) null_idt = { 0, 0 };
                __asm__ volatile (
                    "lidt %0\n\t"
                    "ud2\n\t"
                    :: "m"(null_idt)
                );
            }
            for (;;) __asm__ volatile ("cli; hlt");
        } else if (k_streq(buf, "halt")) {
            term_puts("Halting.\n");
            __asm__ volatile ("cli");
            for (;;) __asm__ volatile ("hlt");
        } else {
            term_puts("Unknown: "); term_puts(buf); term_putchar('\n');
            term_puts("(type 'help')\n");
        }
    }
}
