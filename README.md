# Diamond

A bare-metal x86-64 exokernel written in C. Everything runs at ring 0 with no
syscall boundary — drivers are plain C libraries called directly by
applications. Boots via Limine (UEFI) and targets real hardware (Surface
Laptop 3) as well as QEMU.

## Architecture

Diamond follows the exokernel / OS-as-library model:

- **No kernel/user split** — all code runs at CPL 0.
- **No syscalls** — applications link directly against driver libraries.
- **Minimal kernel** — just enough to boot: memory allocator, page tables, SMP
  bring-up.
- **Shell as application** — the interactive shell is an ordinary C program that
  calls into drivers.

## Source Layout

```
kernel/            Boot entry point, bump allocator, linker script
arch/x86/          CPU, I/O ports, spinlock, PCI, SMP, virtual memory
drivers/           Hardware drivers
  terminal.c       Framebuffer + serial console (shadow buffer)
  keyboard.c       PS/2 keyboard
  serial.c         COM1 UART
  font.c           TrueType rasterizer (stb_truetype)
  sam/             Surface Aggregator Module (SAM) keyboard driver
  net/             NIC drivers (e1000, e1000e, virtio-net, rtl8139, pcnet, iwlwifi)
  video/           Display drivers (BGA, VirtIO GPU, Limine framebuffer)
  audio/           Audio drivers (Intel HDA, AC97, PC speaker)
apps/
  shell/           Interactive command shell
  raytrace/        Real-time ray tracer demo
  piano/           Piano / audio demo
boot/              Limine bootloader configuration
third_party/       Limine binaries, fonts, stb headers, firmware blobs
```

## Building

**Prerequisites:** GCC cross-compiler (or system GCC with freestanding support),
NASM, `ld`, `xorriso`, and `limine` CLI tool (included in `third_party/`).

```bash
make              # build diamond.iso
make run          # boot in QEMU (requires qemu-system-x86_64, OVMF)
make clean        # remove build artifacts
```

### QEMU Options

```bash
make run NIC=virtio-net-pci    # select NIC model (default: e1000)
make run AUDIODEV=alsa         # select audio backend (default: pa)
```

## Running on Real Hardware

Flash `diamond.iso` to a USB drive and boot from it (UEFI only):

```bash
sudo dd if=diamond.iso of=/dev/sdX bs=4M status=progress
```

The primary hardware target is the **Surface Laptop 3** (Intel Ice Lake LP).
The keyboard on this device connects through the Surface Aggregator Module
(SAM) over an LPSS UART — see `drivers/sam/` for the driver.

## Hardware Support

| Component | Driver | Notes |
|-----------|--------|-------|
| Display | Limine framebuffer / BGA / VirtIO GPU | 2560×1440 on Surface |
| Keyboard | PS/2, SAM (Surface), VirtIO | SAM = active development |
| Network | e1000, e1000e, virtio-net, rtl8139, pcnet, iwlwifi | Wi-Fi 6 on Surface |
| Audio | Intel HDA, AC97, PC speaker | |
| Mouse | PS/2, VirtIO tablet | |
| USB | xHCI | Disabled on Surface (perf) |

## License

This is a personal/research project. No license has been selected yet.
