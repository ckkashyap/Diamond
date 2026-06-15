# Diamond OS — Project Context for Claude

## What this project is

Diamond is a bare-metal x86-64 exokernel written in C. Everything runs at CPL0
with no syscall boundary. Drivers are plain C libraries called directly by the
shell application. Boots via Limine UEFI on a Surface Laptop 3.

**Source layout:**
```
kernel/main.c          — boot entry, hardware init sequence
kernel/alloc.c         — bump allocator (kalloc_init / kmalloc)
arch/x86/              — CPU, I/O ports, spinlock, PCI, SMP
drivers/terminal.c     — framebuffer + serial console (shadow buffer)
drivers/sam/sam.c      — Surface SAM keyboard driver ← ACTIVE WORK
drivers/net/           — NIC drivers (e1000, virtio, etc.)
drivers/video/         — BGA, VirtIO GPU, Limine framebuffer
drivers/audio/         — HDA, AC97, PC speaker
drivers/usb/xhci.c    — xHCI (disabled on Surface, too slow)
apps/shell/shell.c     — interactive shell demo
```

**Build:**
```bash
make          # builds diamond.iso
make run      # boots in QEMU (needs ovmf, qemu-system-x86_64)
make clean
```

**Boot to hardware:**
Flash `diamond.iso` to a USB drive and boot the Surface Laptop 3 from it.
Or install as a UEFI boot entry alongside Linux for dual-boot.

---

## Hardware target: Surface Laptop 3 (ICL-LP)

- CPU: Intel Ice Lake LP (ICL-LP)
- Keyboard: connects via SAM (Surface Aggregator Module), a STM32-based
  Microsoft hub, over Intel LPSS **UART0** [8086:34a8] at PCI 0:1E.0
  (verified via `lspci -nn` on running Linux — this is UART#0, NOT UART2)
- LPSS UART BAR: 0x4010003000 (64-bit, above 4 GiB)
- DW APB UART (Synopsys), 16550A-compatible, 32-bit register stride
- Baud: 4 Mbaud, 8N1, hardware RTS/CTS
- Protocol: SSAM/SSH (Surface Serial Aggregator Module)
- GPIO pads: **GPP_C8(RXD), C9(TXD), C10(RTS), C11(CTS)** in ICL-LP
  Community 4 (PID 0x6A, base SBREG+0x6A0000)
- Pad register layout: PAD_CFG_DW0 = COM4_BASE + 0x600 + pin*0x10
  (stride 0x10 = 4 DWs per pad because ICL-LP has PINCTRL_FEATURE_DEBOUNCE)
- PAD_CFG_DW0 bits: [13:10]=PMODE (4-bit; 1=native UART0), bit8=GPIOTXDIS,
  bit9=GPIORXDIS

---

## Current task: SAM keyboard driver (drivers/sam/sam.c)

### What works
- LPSS UART found, D3→D0 PCI power transition done
- BAR reprogrammed to 0x4010003000, MMIO mapped
- LPSS_PRIV_CLK (BAR+0x200) configured: M=16, N=25 → 64 MHz from 100 MHz source
- LPSS_PRIV_RESETS (BAR+0x204) = 0x07 → UART out of reset
- LSR=0x60 (THRE+TEMT): UART is alive and idle
- MSR=0x10 (CTS=1): SAM is asserting its RTS (ready to receive from us)
- Loopback self-test passes (0xA5 echoed back) → TX/RX at 4 Mbaud confirmed
- SSAM frame TX/RX state machine implemented
- HID keycode → ASCII decode implemented

### What does NOT work yet
- SAM does not respond to our D0-entry / display-on / subscribe commands
- `SAM: prescan=[(none)]` — SAM sends nothing unprompted
- `SAM: rx=[(none)]` — SAM sends no ACKs after our commands

### Last change made (NOT YET TESTED on hardware)
Added `LPSS_GENERAL` register write (BAR+0x208, bit 3 = RTS override):

```c
/* Force RTS active when AFCE is disabled — mirrors Linux 8250_lpss.c */
uint32_t gen = U32(s_uart_va, 0x208u);
U32(s_uart_va, 0x208u) = gen | (1u << 3);
```

**Hypothesis:** Without this bit, the physical RTS pin is not driven by MCR
even when MCR.RTS=1. SAM checks our RTS before transmitting; if RTS is low,
SAM won't send ACKs. This is documented in Linux's `8250_lpss.c`
`lpss_uart_setup()` for ICL-LP.

### LPSS private register map (BAR+0x200 region)
```
BAR+0x200  LPSS_PRIV_CLK     clock enable + M/N fractional divider
             bit 0   = enable
             bits[15:1]  = M numerator   (M=16 → 64 MHz from 100 MHz)
             bits[30:16] = N denominator (N=25)
             bit 31  = update trigger
BAR+0x204  LPSS_PRIV_RESETS  bit[1:0]=FUNC, bit2=IDMA (write 0x03 to deassert; NOT 0x07)
BAR+0x208  LPSS_GENERAL      bit 3 = UART_RTS_OVRD (NOT used for ICL-LP; Linux uses AFCE instead)
BAR+0x2FC  LPSS_PRIV_CAPS    capabilities (bit8=no IDMA, bits[7:4]=type)
```

### SSAM/SSH wire protocol
```
[AA][55]                    sync
[type][len_lo][len_hi][seq] frame header (4 bytes)
[fcrc_lo][fcrc_hi]          CRC-16/CCITT-FALSE of frame header
[payload bytes...]          ssh_command struct (8 bytes) + optional data
[pcrc_lo][pcrc_hi]          CRC-16/CCITT-FALSE of payload (0xFFFF if empty)
```

Frame types: 0x80=DATA_SEQ (needs ACK), 0x40=ACK, 0x04=NAK

ssh_command (8 bytes):
```
[0x80][tc][tid=0x01][sid=0x00][iid][rqid_lo][rqid_hi][cid]
```

SSAM init sequence:
1. D0-entry:   TC=0x01, IID=0x00, CID=0x34
2. Display-on: TC=0x01, IID=0x00, CID=0x16
3. Subscribe:  TC=0x01, IID=0x00, CID=0x0B,
               payload={0x15, 0x01, 0x15, 0x00, 0x01}
               (target_category=HID, flags=SEQ, rqid=0x0015, iid=0x01)

Keyboard events arrive as DATA_SEQ frames with:
  TC=0x15 (HID), IID=0x01, rqid in [0x0001..0x0026]

### Useful Linux debugging (from Linux running on Surface Laptop 3)
```bash
# Trace SSAM frames live
sudo modprobe surface_aggregator
sudo cat /sys/kernel/debug/tracing/events/surface_aggregator/enable
echo 1 | sudo tee /sys/kernel/debug/tracing/events/surface_aggregator/enable
sudo cat /sys/kernel/debug/tracing/trace

# Check UART state
sudo cat /sys/bus/platform/devices/80860034:00/power/runtime_status

# See raw SSAM device
ls /sys/bus/surface_aggregator/devices/
```

### Diagnostic output produced by sam_init() (what to look for)
```
SAM: found LPSS UART
SAM: PMCS=0x0b D3->D0
SAM: BAR reprogrammed to 0x4010003000
SAM: UART mapped
SAM: PRIV_CLK=0x00000000          ← was 0, clock was off
SAM: PRIV_CLK_after=0x80190021    ← M=16,N=25,enable,update ✓
SAM: RESETS=0x00                  ← was in reset
SAM: RESETS_after=0x07            ← deasserted ✓
SAM: LSR=0x60                     ← UART alive ✓
SAM: loopback=0xa5 OK             ← TX/RX confirmed ✓
SAM: MSR=0x10 LSR=0x60            ← CTS from SAM ✓
SAM: prescan=[(none)]             ← SAM silent before our commands
SAM: rx=[aa 55 40 ...]            ← WANT: ACK frames here
SAM: keyboard ready
```

---

## Key files modified this session

- `drivers/sam/sam.c` — SAM/SSAM keyboard driver (main work area)
- `drivers/terminal.c` — added shadow framebuffer for fast scrolling
- `kernel/main.c` — added sam_init() call, removed xhci_init (too slow)

## Notes / gotchas

- xHCI (USB keyboard) disabled in main.c — too slow on Surface Laptop 3
  due to port-0x80 busy-waits. SAM is the only keyboard path.
- Shadow framebuffer: term_init_shadow() called after kalloc_init().
  Renders go to WB RAM, blitted to MMIO on scroll — much faster.
- UART registers use 32-bit stride (DW APB UART): offset = reg_num × 4
- DLF register at BAR+0xC0 (DW-specific fractional baud divisor)
- With 64 MHz UART clock: DLL=1, DLF=0 → exactly 4 Mbaud
- CRC: CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflection)
- Outgoing rqid starts at 39 (0x27), SAM event rqids are 0x01..0x26
- UEFI clears PCI BARs for ACPI devices at ExitBootServices — must reprogram
