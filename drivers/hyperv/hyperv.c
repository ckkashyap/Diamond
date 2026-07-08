/*
 * hyperv.c — Hyper-V VMBus client + synthetic keyboard driver (fully polled)
 *
 * Diamond boots as a Generation 2 Hyper-V guest.  Gen 2 VMs have no PS/2 i8042
 * and no emulated USB, so the vmconnect window keyboard is a *synthetic*
 * keyboard delivered over VMBus.  This file implements just enough of the
 * Hyper-V TLFS + VMBus protocol to receive keystrokes:
 *
 *   1. CPUID Hyper-V detection.
 *   2. Guest-OS-ID + hypercall page (overlaid by the hypervisor into an
 *      executable .text page so we can CALL it — .bss is mapped NX by Limine).
 *   3. SynIC enable: SIMP (message page), SIEFP (event page), SINT2.  Because
 *      Diamond runs with IF=0 and no IDT, we never take the interrupt; we poll
 *      the SIMP message slot and the channel ring buffers directly.
 *   4. VMBus bring-up: INITIATE_CONTACT (version negotiate), REQUESTOFFERS,
 *      collect OFFERCHANNEL messages, GPADL_HEADER (ring buffer), OPENCHANNEL.
 *   5. Synthetic keyboard protocol: SYNTH_KBD_PROTOCOL_REQUEST/RESPONSE, then
 *      decode SYNTH_KBD_EVENT (scancode set 1 + flags) to ASCII.
 *
 * References: Linux drivers/hv/{hv,connection,channel,channel_mgmt,ring_buffer}.c,
 * include/linux/hyperv.h, arch/x86/include/asm/mshyperv.h, and
 * drivers/input/serio/hyperv-keyboard.c.  This code cannot be tested outside a
 * real Hyper-V host, so it follows those authoritative layouts exactly and
 * traces every step over COM1 (serial) for on-hardware debugging.
 */

#include <stdint.h>
#include "hyperv.h"
#include "../serial.h"
#include "../terminal.h"

/* ── Debug tracing (COM1) ─────────────────────────────────────────────────── */
#define HV_DEBUG 1

#if HV_DEBUG
static void D(const char *s) { serial_puts(s); }
static void Dx(const char *s, uint64_t v) {
    static const char hexd[] = "0123456789abcdef";
    char buf[19];
    buf[0] = '0'; buf[1] = 'x';
    for (int i = 0; i < 16; i++)
        buf[2 + i] = hexd[(v >> ((15 - i) * 4)) & 0xf];
    buf[18] = 0;
    serial_puts(s);
    serial_puts(buf);
    serial_putchar('\n');
}
#else
static void D(const char *s) { (void)s; }
static void Dx(const char *s, uint64_t v) { (void)s; (void)v; }
#endif

/* ── Small freestanding helpers (no libc; avoid emitting memset/memcpy) ────── */
static void hv_memset(void *d, int c, uint32_t n) {
    volatile uint8_t *p = (volatile uint8_t *)d;
    for (uint32_t i = 0; i < n; i++) p[i] = (uint8_t)c;
}
static void hv_memcpy(void *d, const void *s, uint32_t n) {
    uint8_t *dd = (uint8_t *)d;
    const uint8_t *ss = (const uint8_t *)s;
    for (uint32_t i = 0; i < n; i++) dd[i] = ss[i];
}
static int hv_memeq(const void *a, const void *b, uint32_t n) {
    const uint8_t *x = (const uint8_t *)a, *y = (const uint8_t *)b;
    for (uint32_t i = 0; i < n; i++) if (x[i] != y[i]) return 0;
    return 1;
}
static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void put_u64(uint8_t *p, uint64_t v) { put_u32(p, (uint32_t)v); put_u32(p + 4, (uint32_t)(v >> 32)); }
static inline uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void cpu_pause(void) { __asm__ volatile ("pause" ::: "memory"); }
static inline void mem_barrier(void) { __asm__ volatile ("mfence" ::: "memory"); }

/* ── CPU: CPUID / MSR ─────────────────────────────────────────────────────── */
static inline void cpuid(uint32_t leaf, uint32_t *a, uint32_t *b, uint32_t *c, uint32_t *d) {
    __asm__ volatile ("cpuid" : "=a"(*a), "=b"(*b), "=c"(*c), "=d"(*d) : "a"(leaf), "c"(0));
}
static inline uint64_t rdmsr(uint32_t msr) {
    uint32_t lo, hi;
    __asm__ volatile ("rdmsr" : "=a"(lo), "=d"(hi) : "c"(msr));
    return ((uint64_t)hi << 32) | lo;
}
static inline void wrmsr(uint32_t msr, uint64_t val) {
    uint32_t lo = (uint32_t)val, hi = (uint32_t)(val >> 32);
    __asm__ volatile ("wrmsr" :: "c"(msr), "a"(lo), "d"(hi));
}

/* ── Hyper-V constants (TLFS) ─────────────────────────────────────────────── */
#define HV_MSR_GUEST_OS_ID   0x40000000u
#define HV_MSR_HYPERCALL     0x40000001u
#define HV_MSR_VP_INDEX      0x40000002u
#define HV_MSR_SCONTROL      0x40000080u
#define HV_MSR_SIEFP         0x40000082u
#define HV_MSR_SIMP          0x40000083u
#define HV_MSR_EOM           0x40000084u
#define HV_MSR_SINT0         0x40000090u

#define VMBUS_MESSAGE_SINT   2u          /* SINT used for VMBus messages */
#define HV_MESSAGE_SIZE      256u        /* size of one SIMP message slot  */
#define HVMSG_NONE           0u

#define HVCALL_POST_MESSAGE  0x005cull
#define HVCALL_SIGNAL_EVENT  0x005dull
#define HV_HYPERCALL_FAST_BIT (1ull << 16)

#define HV_STATUS_SUCCESS               0
#define HV_STATUS_INSUFFICIENT_BUFFERS  0x13

#define HV_GUEST_OS_ID  0x8100000000000000ull  /* nonzero (required before hypercalls) */

/* VMBus message connection ids (well-known bootstrap ids) */
#define VMBUS_CONNID_MESSAGE     1u   /* legacy (< WIN10_V5) */
#define VMBUS_CONNID_MESSAGE_4   4u   /* WIN10_V5+           */

/* HvPostMessage "message type" argument for VMBus */
#define HV_MSGTYPE_VMBUS         1u

/* VMBus channel message types */
enum {
    CHANNELMSG_OFFERCHANNEL        = 1,
    CHANNELMSG_REQUESTOFFERS       = 3,
    CHANNELMSG_ALLOFFERS_DELIVERED = 4,
    CHANNELMSG_OPENCHANNEL         = 5,
    CHANNELMSG_OPENCHANNEL_RESULT  = 6,
    CHANNELMSG_GPADL_HEADER        = 8,
    CHANNELMSG_GPADL_BODY          = 9,
    CHANNELMSG_GPADL_CREATED       = 10,
    CHANNELMSG_INITIATE_CONTACT    = 14,
    CHANNELMSG_VERSION_RESPONSE    = 15,
    CHANNELMSG_UNLOAD              = 16,
    CHANNELMSG_UNLOAD_RESPONSE     = 17,
};

/* VMBus protocol versions ((major << 16) | minor) */
#define VERSION_WS2008   0x0000000Du
#define VERSION_WIN8_1   0x00030000u
#define VERSION_WIN10_V5 0x00050000u

/* VMBus packet descriptor */
#define VM_PKT_DATA_INBAND  0x0006u
#define VMPKT_DESC_SIZE     16u          /* sizeof(struct vmpacket_descriptor) */
#define VMBUS_PKT_TRAILER   8u

/* Synthetic keyboard protocol (hyperv-keyboard.c) */
#define SYNTH_KBD_VERSION            0x00010000u   /* MAKE_KBD_VERSION(1,0) */
#define SYNTH_KBD_PROTOCOL_REQUEST   1u
#define SYNTH_KBD_PROTOCOL_RESPONSE  2u
#define SYNTH_KBD_EVENT              3u
#define SYNTH_KBD_LED_INDICATORS     4u
#define SYNTH_KBD_PROTOCOL_ACCEPTED  1u            /* bit 0 of proto_status  */

#define KEYSTROKE_INFO_UNICODE  0x0001u
#define KEYSTROKE_INFO_BREAK    0x0002u
#define KEYSTROKE_INFO_E0       0x0004u
#define KEYSTROKE_INFO_E1       0x0008u

/* Synthetic keyboard device-class GUID {f912ad6d-2b17-48ea-bd65-f927a61c7684}
 * laid out as a Linux guid_t (mixed-endian): first three fields little-endian,
 * final eight bytes verbatim. */
static const uint8_t HV_KBD_GUID[16] = {
    0x6d, 0xad, 0x12, 0xf9, 0x17, 0x2b, 0xea, 0x48,
    0xbd, 0x65, 0xf9, 0x27, 0xa6, 0x1c, 0x76, 0x84
};

/* Busy-poll bound for one-shot handshake waits (boot-time only). */
#define HV_SPIN  300000000ull

/* ── Guest-physical pages (BSS, zeroed by Limine, 4 KiB aligned) ──────────── */
/* GPA = va - kvirt + kphys (identical to virtio_input's virt_to_phys). */
#define PAGE 4096u
#define SEND_DATA_PAGES 4u
#define RECV_DATA_PAGES 4u
#define SEND_PAGES (1u + SEND_DATA_PAGES)          /* header + data */
#define RECV_PAGES (1u + RECV_DATA_PAGES)
#define RING_PAGES (SEND_PAGES + RECV_PAGES)       /* single GPADL buffer */

static uint8_t g_simp[PAGE]      __attribute__((aligned(4096)));
static uint8_t g_siefp[PAGE]     __attribute__((aligned(4096)));
static uint8_t g_postmsg[PAGE]   __attribute__((aligned(4096)));  /* HvPostMessage input */
static uint8_t g_monitor1[PAGE]  __attribute__((aligned(4096)));
static uint8_t g_monitor2[PAGE]  __attribute__((aligned(4096)));
static uint8_t g_intpage[PAGE]   __attribute__((aligned(4096)));  /* legacy interrupt page */
static uint8_t g_ring[RING_PAGES * PAGE] __attribute__((aligned(4096)));

/* The hypercall page MUST be executable: the hypervisor overlays it with a
 * vmcall/vmmcall stub that we CALL.  .bss is NX under Limine, so place it in an
 * RX .text section (see kernel/linker.ld: .text.* → text PHDR, flags R+X). */
static uint8_t g_hcall_page[PAGE]
    __attribute__((aligned(4096), section(".text.hvcall"), used));

/* Scratch build/parse buffers (channel messages are <= 240 bytes). */
static uint8_t g_txbuf[256];
static uint8_t g_rxbuf[256];

/* ── Driver state ─────────────────────────────────────────────────────────── */
static uint64_t s_hhdm, s_kphys, s_kvirt;
static void    *s_hcall_pg;              /* VA of hypercall page (call target) */
static int      s_present;               /* running under Hyper-V              */
static uint32_t s_vp_index;
static uint32_t s_negotiated_ver;
static uint32_t s_msg_conn_id;           /* connection id for channel messages */

static int      s_kbd_found;
static uint32_t s_kbd_relid;             /* child_relid of the keyboard channel */
static uint32_t s_kbd_conn_id;           /* connection id to signal the host    */
static uint32_t s_gpadl_handle = 0x1D1Au;
static int      s_kbd_ready;

/* Keyboard decode state */
static int s_shift, s_caps;

static uint64_t virt_to_phys(const void *v) {
    return (uint64_t)(uintptr_t)v - s_kvirt + s_kphys;
}

/* ── Hypercall primitive ──────────────────────────────────────────────────── */
/* Microsoft x64 hypercall ABI: RCX=control, RDX=input, R8=output; RAX=status.
 * We call the hypervisor-overlaid page, which abstracts vmcall vs vmmcall. */
static uint64_t hv_hypercall(uint64_t control, uint64_t arg1, uint64_t arg2) {
    uint64_t status;
    register uint64_t r8 __asm__("r8") = arg2;
    __asm__ __volatile__(
        "call *%[pg]"
        : "=a"(status), "+c"(control), "+d"(arg1), "+r"(r8)
        : [pg] "m"(s_hcall_pg)
        : "r9", "r10", "r11", "memory", "cc"
    );
    return status;
}

/* HvSignalEvent (fast): tell the host our outbound ring went non-empty. */
static void hv_signal_event(uint32_t conn_id) {
    hv_hypercall(HVCALL_SIGNAL_EVENT | HV_HYPERCALL_FAST_BIT, (uint64_t)conn_id, 0);
}

/* HvPostMessage: post a VMBus channel message to the host. Returns HV status. */
static int hv_post_message(uint32_t conn_id, const void *msg, uint32_t len) {
    if (len > 240) return -1;
    uint8_t *in = g_postmsg;
    hv_memset(in, 0, 16 + 240);
    put_u32(in + 0, conn_id);            /* connectionid           */
    put_u32(in + 4, 0);                  /* reserved               */
    put_u32(in + 8, HV_MSGTYPE_VMBUS);   /* message_type           */
    put_u32(in + 12, len);               /* payload_size           */
    hv_memcpy(in + 16, msg, len);        /* payload[]              */
    uint64_t gpa = virt_to_phys(in);
    for (int retry = 0; retry < 1000; retry++) {
        uint64_t st = hv_hypercall(HVCALL_POST_MESSAGE, gpa, 0) & 0xffff;
        if (st == HV_STATUS_SUCCESS) return 0;
        if (st != HV_STATUS_INSUFFICIENT_BUFFERS) { Dx("[hv] post_message status=", st); return (int)st; }
        for (int i = 0; i < 10000; i++) cpu_pause();
    }
    D("[hv] post_message timed out (busy)\n");
    return -1;
}

/* ── SynIC message polling (SIMP slot for VMBUS_MESSAGE_SINT) ──────────────── */
/* SIMP is a page of 16 slots x 256 bytes.  hv_message layout:
 *   u32 message_type; u8 payload_size; u8 message_flags; u8 rsvd[2]; u64 sender;
 *   u64 payload[30];   (header = 16 bytes, payload at +16)                     */
static volatile uint8_t *simp_slot(void) {
    return (volatile uint8_t *)g_simp + VMBUS_MESSAGE_SINT * HV_MESSAGE_SIZE;
}

/* If a channel message is pending, copy its payload into out (<=240 bytes),
 * set *type to the VMBus channel msgtype, consume the slot, and return 1. */
static int hv_read_channel_msg(uint8_t *out, uint32_t *type) {
    volatile uint8_t *slot = simp_slot();
    uint32_t mt = get_u32((const uint8_t *)slot);   /* header.message_type */
    if (mt == HVMSG_NONE) return 0;

    uint8_t psize = slot[4];                         /* header.payload_size */
    if (psize > 240) psize = 240;
    for (uint32_t i = 0; i < psize; i++) out[i] = slot[16 + i];
    for (uint32_t i = psize; i < 16; i++) out[i] = 0; /* ensure header readable */

    *type = get_u32(out);

    uint8_t flags = slot[5];                         /* header.message_flags */
    put_u32((uint8_t *)slot, HVMSG_NONE);            /* mark slot consumed  */
    mem_barrier();
    if (flags & 1) wrmsr(HV_MSR_EOM, 0);             /* advance queue       */
    return 1;
}

/* ── Ring buffer (guest<->host), per drivers/hv/ring_buffer.c ─────────────── */
/* Header page (4 KiB): write_index, read_index, interrupt_mask, pending_send_sz.
 * Data starts at header + 4096.  Indices are byte offsets in [0, datasize). */
struct ring {
    volatile uint32_t *wi;    /* &write_index */
    volatile uint32_t *ri;    /* &read_index  */
    volatile uint32_t *imask; /* &interrupt_mask */
    uint8_t  *data;
    uint32_t  datasize;
};

static struct ring s_send, s_recv;

static void ring_init(struct ring *r, uint8_t *base, uint32_t data_pages) {
    r->wi    = (volatile uint32_t *)(base + 0);
    r->ri    = (volatile uint32_t *)(base + 4);
    r->imask = (volatile uint32_t *)(base + 8);
    r->data  = base + PAGE;
    r->datasize = data_pages * PAGE;
}

static uint32_t ring_put(struct ring *r, uint32_t off, const void *src, uint32_t len) {
    const uint8_t *s = (const uint8_t *)src;
    for (uint32_t i = 0; i < len; i++) {
        r->data[off] = s[i];
        if (++off >= r->datasize) off = 0;
    }
    return off;
}
static void ring_get(struct ring *r, uint32_t off, void *dst, uint32_t len) {
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t i = 0; i < len; i++) {
        d[i] = r->data[off];
        if (++off >= r->datasize) off = 0;
    }
}

/* Write one in-band VMBus packet carrying msg[msglen]; signal the host. */
static int ring_send(struct ring *r, const void *msg, uint32_t msglen, uint32_t conn_id) {
    uint32_t desclen = VMPKT_DESC_SIZE + msglen;
    uint32_t padded  = (desclen + 7u) & ~7u;
    uint32_t total   = padded + VMBUS_PKT_TRAILER;

    uint32_t w  = *r->wi;
    uint32_t rd = *r->ri;
    uint32_t avail = (w >= rd) ? (r->datasize - (w - rd)) : (rd - w);
    if (avail <= total) { D("[hv] send ring full\n"); return -1; }

    uint8_t desc[VMPKT_DESC_SIZE];
    put_u16(desc + 0, VM_PKT_DATA_INBAND);   /* type    */
    put_u16(desc + 2, VMPKT_DESC_SIZE >> 3); /* offset8 (payload after 16-byte desc) */
    put_u16(desc + 4, (uint16_t)(padded >> 3)); /* len8 (desc+payload, padded) */
    put_u16(desc + 6, 0);                    /* flags (no completion) */
    put_u64(desc + 8, 0);                    /* trans_id */

    uint32_t p = w;
    p = ring_put(r, p, desc, VMPKT_DESC_SIZE);
    p = ring_put(r, p, msg, msglen);
    for (uint32_t i = desclen; i < padded; i++) { r->data[p] = 0; if (++p >= r->datasize) p = 0; }
    uint8_t trailer[8];
    put_u64(trailer, (uint64_t)w << 32);     /* previous write offset */
    p = ring_put(r, p, trailer, 8);

    mem_barrier();
    *r->wi = p;
    mem_barrier();

    /* Signal only on the empty->non-empty transition (host DoS throttling). */
    if (*r->imask == 0 && w == rd) hv_signal_event(conn_id);
    return 0;
}

/* Read one packet from the inbound ring.  Returns the packet type (>=0) and
 * fills out[*outlen] with the message payload, or -1 if the ring is empty. */
static int ring_recv(struct ring *r, uint8_t *out, uint32_t outcap, uint32_t *outlen) {
    uint32_t w  = *r->wi;
    uint32_t rd = *r->ri;
    uint32_t avail = (w >= rd) ? (w - rd) : (r->datasize - rd + w);
    if (avail < VMPKT_DESC_SIZE) return -1;

    uint8_t desc[VMPKT_DESC_SIZE];
    ring_get(r, rd, desc, VMPKT_DESC_SIZE);
    uint16_t type = get_u16(desc + 0);
    uint32_t poff = (uint32_t)get_u16(desc + 2) << 3;
    uint32_t tlen = (uint32_t)get_u16(desc + 4) << 3;

    if (tlen < VMPKT_DESC_SIZE || tlen > avail) {   /* corrupt: drop everything */
        mem_barrier();
        *r->ri = w;
        return -1;
    }
    if (poff < VMPKT_DESC_SIZE || poff > tlen) poff = VMPKT_DESC_SIZE;

    uint32_t msglen = tlen - poff;
    if (msglen > outcap) msglen = outcap;
    ring_get(r, (rd + poff) % r->datasize, out, msglen);
    *outlen = msglen;

    uint32_t adv = tlen + VMBUS_PKT_TRAILER;
    rd += adv;
    if (rd >= r->datasize) rd -= r->datasize;
    mem_barrier();
    *r->ri = rd;
    return (int)type;
}

/* ── VMBus bring-up ───────────────────────────────────────────────────────── */

/* Record a keyboard offer if this OFFERCHANNEL matches the synth-kbd GUID.
 * offer_channel layout: header(8) offer(176) child_relid(4) monitorid(1)
 * monitor_allocated(1) is_dedicated(2) connection_id(4).  if_type is at
 * offset 8 (start of offer).  sizeof(struct vmbus_channel_offer) is 176
 * (sub_channel_index/reserved3 are u16), so child_relid is at 8+176=184 and
 * connection_id at 184+4+1+1+2=192 (verified against include/linux/hyperv.h). */
static void hv_note_offer(const uint8_t *m) {
    const uint8_t *if_type = m + 8;
    if (!hv_memeq(if_type, HV_KBD_GUID, 16)) return;
    if (s_kbd_found) return;
    s_kbd_relid   = get_u32(m + 184);
    s_kbd_conn_id = get_u32(m + 192);
    s_kbd_found   = 1;
    Dx("[hv] keyboard offer child_relid=", s_kbd_relid);
    Dx("[hv] keyboard offer connection_id=", s_kbd_conn_id);
}

/* Poll SIMP for a specific channel message type (dispatching offers we see). */
static int hv_wait_msg(uint32_t want, uint8_t *out) {
    for (uint64_t i = 0; i < HV_SPIN; i++) {
        uint32_t t;
        if (hv_read_channel_msg(out, &t)) {
            if (t == want) return 1;
            if (t == CHANNELMSG_OFFERCHANNEL) hv_note_offer(out);
        } else {
            cpu_pause();
        }
    }
    return 0;
}

/* Send INITIATE_CONTACT for one version; return 1 if the host accepts it. */
static int hv_try_version(uint32_t ver) {
    uint32_t boot_conn = (ver >= VERSION_WIN10_V5) ? VMBUS_CONNID_MESSAGE_4 : VMBUS_CONNID_MESSAGE;
    uint8_t *m = g_txbuf;
    hv_memset(m, 0, 40);
    put_u32(m + 0, CHANNELMSG_INITIATE_CONTACT);  /* header.msgtype */
    put_u32(m + 4, 0);                            /* header.padding */
    put_u32(m + 8, ver);                          /* vmbus_version_requested */
    put_u32(m + 12, s_vp_index);                  /* target_vcpu */
    if (ver >= VERSION_WIN10_V5) {
        /* union: msg_sint(1) msg_vtl(1) rsvd(2) feature_flags(4) */
        put_u64(m + 16, (uint64_t)VMBUS_MESSAGE_SINT);
    } else {
        put_u64(m + 16, virt_to_phys(g_intpage));  /* union: interrupt_page GPA */
    }
    put_u64(m + 24, virt_to_phys(g_monitor1));      /* monitor_page1 */
    put_u64(m + 32, virt_to_phys(g_monitor2));      /* monitor_page2 */

    Dx("[hv] INITIATE_CONTACT version=", ver);
    if (hv_post_message(boot_conn, m, 40) != 0) return 0;

    if (!hv_wait_msg(CHANNELMSG_VERSION_RESPONSE, g_rxbuf)) {
        D("[hv] no VERSION_RESPONSE\n");
        return 0;
    }
    uint8_t supported = g_rxbuf[8];                 /* version_supported */
    uint32_t host_conn = get_u32(g_rxbuf + 12);     /* msg_conn_id (V5+) */
    Dx("[hv] VERSION_RESPONSE supported=", supported);
    if (!supported) return 0;

    s_negotiated_ver = ver;
    if (ver >= VERSION_WIN10_V5 && host_conn) s_msg_conn_id = host_conn;
    else s_msg_conn_id = boot_conn;
    Dx("[hv] negotiated, msg_conn_id=", s_msg_conn_id);
    return 1;
}

static int hv_vmbus_connect(void) {
    static const uint32_t versions[] = { VERSION_WIN10_V5, VERSION_WIN8_1, VERSION_WS2008 };
    for (unsigned i = 0; i < sizeof(versions) / sizeof(versions[0]); i++)
        if (hv_try_version(versions[i])) return 0;
    D("[hv] version negotiation failed\n");
    return -1;
}

/* REQUESTOFFERS, then collect OFFERCHANNELs until ALLOFFERS_DELIVERED. */
static void hv_request_offers(void) {
    uint8_t *m = g_txbuf;
    hv_memset(m, 0, 8);
    put_u32(m + 0, CHANNELMSG_REQUESTOFFERS);
    put_u32(m + 4, 0);
    if (hv_post_message(s_msg_conn_id, m, 8) != 0) { D("[hv] REQUESTOFFERS post failed\n"); return; }

    for (uint64_t i = 0; i < HV_SPIN; i++) {
        uint32_t t;
        if (hv_read_channel_msg(g_rxbuf, &t)) {
            if (t == CHANNELMSG_OFFERCHANNEL) hv_note_offer(g_rxbuf);
            else if (t == CHANNELMSG_ALLOFFERS_DELIVERED) { D("[hv] all offers delivered\n"); return; }
        } else {
            cpu_pause();
        }
    }
    D("[hv] offer collection timed out\n");
}

/* Establish the ring buffer GPADL and open the keyboard channel. */
static int hv_open_kbd_channel(void) {
    /* GPADL_HEADER with a single range covering all RING_PAGES pages. */
    uint64_t base_gpa = virt_to_phys(g_ring);
    uint64_t base_pfn = base_gpa >> 12;
    uint8_t *m = g_txbuf;
    uint32_t off = 0;
    hv_memset(m, 0, sizeof(g_txbuf));
    put_u32(m + 0, CHANNELMSG_GPADL_HEADER);
    put_u32(m + 4, 0);
    put_u32(m + 8, s_kbd_relid);                       /* child_relid */
    put_u32(m + 12, s_gpadl_handle);                   /* gpadl       */
    put_u16(m + 16, (uint16_t)(8 + RING_PAGES * 8));   /* range_buflen */
    put_u16(m + 18, 1);                                /* rangecount   */
    put_u32(m + 20, RING_PAGES * PAGE);                /* range.byte_count  */
    put_u32(m + 24, 0);                                /* range.byte_offset */
    off = 28;
    for (uint32_t i = 0; i < RING_PAGES; i++) { put_u64(m + off, base_pfn + i); off += 8; }

    Dx("[hv] GPADL_HEADER pfn0=", base_pfn);
    if (hv_post_message(s_msg_conn_id, m, off) != 0) { D("[hv] GPADL post failed\n"); return -1; }
    if (!hv_wait_msg(CHANNELMSG_GPADL_CREATED, g_rxbuf)) { D("[hv] no GPADL_CREATED\n"); return -1; }
    uint32_t cstatus = get_u32(g_rxbuf + 16);          /* creation_status */
    Dx("[hv] GPADL_CREATED status=", cstatus);
    if (cstatus != 0) return -1;

    /* OPENCHANNEL: header(8) child_relid openid gpadl target_vp
     * downstream_ringbuffer_pageoffset userdata[120]. */
    hv_memset(m, 0, sizeof(g_txbuf));
    put_u32(m + 0, CHANNELMSG_OPENCHANNEL);
    put_u32(m + 4, 0);
    put_u32(m + 8, s_kbd_relid);      /* child_relid */
    put_u32(m + 12, 1);               /* openid      */
    put_u32(m + 16, s_gpadl_handle);  /* ringbuffer_gpadlhandle */
    put_u32(m + 20, s_vp_index);      /* target_vp   */
    put_u32(m + 24, SEND_PAGES);      /* downstream_ringbuffer_pageoffset */
    /* userdata[120] left zero */
    if (hv_post_message(s_msg_conn_id, m, 28 + 120) != 0) { D("[hv] OPENCHANNEL post failed\n"); return -1; }
    if (!hv_wait_msg(CHANNELMSG_OPENCHANNEL_RESULT, g_rxbuf)) { D("[hv] no OPENCHANNEL_RESULT\n"); return -1; }
    uint32_t ostatus = get_u32(g_rxbuf + 16);          /* open status */
    Dx("[hv] OPENCHANNEL_RESULT status=", ostatus);
    if (ostatus != 0) return -1;

    ring_init(&s_send, g_ring, SEND_DATA_PAGES);
    ring_init(&s_recv, g_ring + SEND_PAGES * PAGE, RECV_DATA_PAGES);
    /* We poll the inbound ring (IF=0, no IDT), so ask the host not to signal. */
    *s_recv.imask = 1;
    return 0;
}

/* ── Synthetic keyboard protocol ──────────────────────────────────────────── */
static int hv_kbd_handshake(void) {
    uint8_t req[8];
    put_u32(req + 0, SYNTH_KBD_PROTOCOL_REQUEST);
    put_u32(req + 4, SYNTH_KBD_VERSION);
    if (ring_send(&s_send, req, sizeof(req), s_kbd_conn_id) != 0) return -1;
    D("[hv] sent SYNTH_KBD_PROTOCOL_REQUEST\n");

    /* Best-effort wait for the accept; keystrokes stream after this. */
    for (uint64_t i = 0; i < HV_SPIN; i++) {
        uint8_t buf[64];
        uint32_t n;
        int t = ring_recv(&s_recv, buf, sizeof(buf), &n);
        if (t < 0) { cpu_pause(); continue; }
        if (t != VM_PKT_DATA_INBAND || n < 8) continue;
        if (get_u32(buf) == SYNTH_KBD_PROTOCOL_RESPONSE) {
            uint32_t st = get_u32(buf + 4);
            Dx("[hv] SYNTH_KBD_PROTOCOL_RESPONSE status=", st);
            return (st & SYNTH_KBD_PROTOCOL_ACCEPTED) ? 0 : -1;
        }
    }
    D("[hv] no protocol response (continuing anyway)\n");
    return 0;   /* proceed; getchar() will also handle a late response */
}

/* ── Scancode set 1 → ASCII (local copy; matches drivers/keyboard.c) ──────── */
static const char sc_lower[128] = {
/*00*/ 0,  27, '1','2','3','4','5','6','7','8','9','0','-','=',  8, '\t',
/*10*/'q','w','e','r','t','y','u','i','o','p','[',']','\n',  0, 'a', 's',
/*20*/'d','f','g','h','j','k','l',';','\'','`',  0,'\\','z','x', 'c', 'v',
/*30*/'b','n','m',',','.','/',  0, '*',  0, ' ',  0,   0,   0,   0,   0,  0,
/*40*/ 0,  0,  0,  0,  0,  0,  0, '7','8','9','-','4','5','6','+','1',
/*50*/'2','3','0','.', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};
static const char sc_upper[128] = {
/*00*/ 0,  27, '!','@','#','$','%','^','&','*','(',')','_','+',  8, '\t',
/*10*/'Q','W','E','R','T','Y','U','I','O','P','{','}','\n',  0, 'A', 'S',
/*20*/'D','F','G','H','J','K','L',':','"', '~',  0, '|','Z','X', 'C', 'V',
/*30*/'B','N','M','<','>','?',  0, '*',  0, ' ',  0,   0,   0,   0,   0,  0,
/*40*/ 0,  0,  0,  0,  0,  0,  0, '7','8','9','-','4','5','6','+','1',
/*50*/'2','3','0','.', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};

/* Decode one synthetic keystroke; return ASCII or -1 (modifier / release). */
static int hv_decode_keystroke(uint16_t make, uint32_t info) {
    int is_break = (info & KEYSTROKE_INFO_BREAK) != 0;

    /* Injected text arrives as Unicode code points rather than scancodes. */
    if (info & KEYSTROKE_INFO_UNICODE) {
        if (is_break) return -1;
        if (make >= 0x20 && make < 0x7f) return (int)make;
        return -1;
    }
    /* E0/E1-extended keys (arrows, nav, right-ctrl/alt) reuse set-1 codes that
     * collide with the numpad; ignore them so they don't emit stray digits. */
    if (info & (KEYSTROKE_INFO_E0 | KEYSTROKE_INFO_E1)) return -1;

    if (make == 0x2A || make == 0x36) { s_shift = !is_break; return -1; }  /* Shift */
    if (make == 0x3A) { if (!is_break) s_caps ^= 1; return -1; }           /* CapsLock */
    if (is_break) return -1;                                               /* releases */
    if (make >= 128) return -1;

    char c = s_shift ? sc_upper[make] : sc_lower[make];
    if (!c) return -1;
    if (s_caps) {
        if (c >= 'a' && c <= 'z') c = (char)(c - 32);
        else if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
    }
    return (int)(unsigned char)c;
}

/* ── Public API ───────────────────────────────────────────────────────────── */
int hv_present(void) { return s_present; }

int hv_kbd_getchar(void) {
    if (!s_kbd_ready) return -1;
    for (;;) {
        uint8_t buf[64];
        uint32_t n;
        int t = ring_recv(&s_recv, buf, sizeof(buf), &n);
        if (t < 0) return -1;                       /* ring empty */
        if (t != VM_PKT_DATA_INBAND || n < 4) continue;
        uint32_t mtype = get_u32(buf);
        if (mtype == SYNTH_KBD_EVENT && n >= 12) {
            uint16_t make = get_u16(buf + 4);
            uint32_t info = get_u32(buf + 8);
            int c = hv_decode_keystroke(make, info);
            if (c >= 0) return c;
        }
        /* PROTOCOL_RESPONSE / LED / non-printing keys: keep draining. */
    }
}

void hv_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    s_hhdm = hhdm_offset; s_kphys = kphys; s_kvirt = kvirt;

    /* 1. Detect Hyper-V.  Must happen BEFORE touching any HV MSR (rdmsr on a
     *    non-existent MSR would #GP -> triple fault, since we have no IDT). */
    uint32_t a, b, c, d;
    cpuid(0x40000000, &a, &b, &c, &d);
    if (!(b == 0x7263694D && c == 0x666F736F && d == 0x76482074)) {  /* "Microsoft Hv" */
        D("[hv] not running under Hyper-V\n");
        return;
    }
    cpuid(0x40000001, &a, &b, &c, &d);
    if (a != 0x31237648) {  /* "Hv#1" — hypercall interface */
        D("[hv] Hv#1 interface absent\n");
        return;
    }
    s_present = 1;
    D("[hv] Hyper-V detected\n");

    /* 2. Guest OS id (nonzero), then enable the hypercall page (must be RX). */
    wrmsr(HV_MSR_GUEST_OS_ID, HV_GUEST_OS_ID);
    s_hcall_pg = g_hcall_page;
    uint64_t hc_gpa = virt_to_phys(g_hcall_page);
    wrmsr(HV_MSR_HYPERCALL, (hc_gpa & ~0xfffull) | 1u);
    Dx("[hv] hypercall page gpa=", hc_gpa);

    s_vp_index = (uint32_t)rdmsr(HV_MSR_VP_INDEX);
    Dx("[hv] vp_index=", s_vp_index);

    /* 3. Enable SynIC: SIMP (messages), SIEFP (events), SINT2, control. */
    wrmsr(HV_MSR_SIMP,  (virt_to_phys(g_simp)  & ~0xfffull) | 1u);
    wrmsr(HV_MSR_SIEFP, (virt_to_phys(g_siefp) & ~0xfffull) | 1u);
    /* SINT2: vector 0x40, not masked (bit16=0), no auto-EOI.  We never take the
     * interrupt (IF=0); the SINT just needs to be armed for message delivery. */
    wrmsr(HV_MSR_SINT0 + VMBUS_MESSAGE_SINT, 0x40u);
    wrmsr(HV_MSR_SCONTROL, 1u);
    D("[hv] SynIC enabled\n");

    /* 4. VMBus connect + offers. */
    if (hv_vmbus_connect() != 0) return;
    hv_request_offers();
    if (!s_kbd_found) { D("[hv] no synthetic keyboard offered\n"); return; }

    /* 5. Open the channel and run the keyboard handshake. */
    if (hv_open_kbd_channel() != 0) { D("[hv] failed to open keyboard channel\n"); return; }
    if (hv_kbd_handshake() != 0)    { D("[hv] keyboard protocol rejected\n"); return; }

    s_kbd_ready = 1;
    D("[hv] synthetic keyboard READY\n");
    term_puts("[hv] synthetic keyboard ready\n");
}
