# Diamond

A bare-metal x86-64 exokernel written in C. Everything runs at ring 0 with no
syscall boundary — drivers are plain C libraries called directly by
applications. Boots via Limine (UEFI + legacy BIOS) and targets real hardware
(Surface Laptop 3) as well as QEMU and Hyper-V (Generation 1 and 2).

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
  scancode.c       Shared PS/2 scan-code-set-1 decoder
  serial.c         COM1 UART
  font.c           TrueType rasterizer (stb_truetype)
  sam/             Surface Aggregator Module (SAM) keyboard driver
  hyperv/          Hyper-V synthetic keyboard (VMBus) + reference timer
  net/             NIC drivers (e1000, e1000e, virtio-net, rtl8139, pcnet, iwlwifi)
  video/           Display drivers (BGA, VirtIO GPU, Limine framebuffer)
  audio/           Audio drivers (Intel HDA, AC97, PC speaker)
apps/
  shell/           Interactive command shell
  raytrace/        Real-time ray tracer demo
  piano/           Piano / audio demo
boot/              Limine bootloader configuration
scripts/           Windows PowerShell: WSL build + Hyper-V boot helpers
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

## Windows: build in WSL, boot under Hyper-V

On Windows the toolchain runs inside **WSL2** and the OS boots as a **Hyper-V**
virtual machine. Everything is scripted in `scripts\` — clone the repo and run a
single command from PowerShell:

```powershell
# from the repo root: build diamond.vhdx via WSL, then create + start a Gen 2 VM
powershell -ExecutionPolicy Bypass -File scripts\Start-DiamondHyperV.ps1
```

`Start-DiamondHyperV.ps1` builds `diamond.vhdx` in WSL, then creates a Hyper-V VM
from it and opens the console. The Hyper-V step self-elevates (VM management needs
Administrator). Type directly in the VM window — Diamond ships a Hyper-V synthetic
keyboard (VMBus) driver for Generation 2.

**Prerequisites**

- Windows 10/11 with **Hyper-V** enabled (run as Administrator):
  `Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V -All`
- **WSL2** with a Linux distro (`wsl --install`) and the build tools:
  `sudo apt install -y build-essential nasm xorriso mtools dosfstools gdisk qemu-utils`

**The scripts** (all under `scripts\`, path-independent — run from any clone)

| Script | What it does |
|--------|--------------|
| `Build-Diamond.ps1` | Build `diamond.vhdx` (or `-Target iso`) via WSL. No Hyper-V needed. |
| `Start-DiamondHyperV.ps1` | Build **and** boot (Gen 2 default; `-Generation 1`; `-SkipBuild`). |
| `Boot-DiamondHyperV.ps1` | Boot an existing `diamond.vhdx` as a Gen 2 (UEFI) VM. |
| `Boot-DiamondHyperV-Gen1.ps1` | Boot as a Gen 1 (legacy BIOS, native PS/2 keyboard) VM. |
| `Connect-DiamondSerial.ps1` | Attach COM1 to a named pipe and open a serial console. |

The same `diamond.vhdx` is a hybrid GPT disk that boots **Gen 2 (UEFI)** and
**Gen 1 (BIOS)**, and also runs under QEMU. Secure Boot must be **off** on Gen 2
(the kernel and Limine are unsigned) — the script sets this for you.

### Boot the WSL build under QEMU

You don't need Hyper-V to try Diamond — from inside WSL:

```bash
make run                 # ISO + OVMF (UEFI) in a QEMU window
make run-vhdx            # boot the same diamond.vhdx via OVMF

# headless serial smoke test (no GUI needed):
make vhdx
qemu-system-x86_64 -machine pc -m 512 \
    -drive file=diamond.vhdx,format=vhdx,if=ide \
    -serial stdio -display none -no-reboot
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
| Keyboard | PS/2, SAM (Surface), VirtIO, Hyper-V synthetic (VMBus) | Gen 2 VM uses VMBus |
| Network | e1000, e1000e, virtio-net, rtl8139, pcnet, iwlwifi | Wi-Fi 6 on Surface |
| Audio | Intel HDA, AC97, PC speaker | |
| Mouse | PS/2, VirtIO tablet | |
| USB | xHCI | Disabled on Surface (perf) |

## License

This is a personal/research project. No license has been selected yet.
