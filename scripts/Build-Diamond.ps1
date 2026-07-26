<#
.SYNOPSIS
    Build Diamond from Windows by driving the WSL toolchain.

.DESCRIPTION
    Diamond is built with a Linux toolchain (gcc/ld/make + limine). On Windows the
    supported path is to build inside WSL. This script figures out where the repo
    lives (it sits in <repo>\scripts\), translates that path into the WSL mount
    namespace (e.g. C:\dev\Diamond -> /mnt/c/dev/Diamond) and runs `make` there.

    Nothing is hardcoded: run it from any clone, on any drive/path.

    Default target is `vhdx`, which produces diamond.vhdx - a hybrid GPT disk that
    boots BOTH Hyper-V Generation 2 (UEFI) and Generation 1 (legacy BIOS), and also
    boots under QEMU. Use -Target iso to build the bootable ISO instead.

.PARAMETER Target
    Make target to build. Default: vhdx. Common: vhdx, iso, all, clean.

.PARAMETER Distro
    Optional WSL distribution name (passed to `wsl -d`). Defaults to your default distro.

.PARAMETER Jobs
    Parallel make jobs (-j). Default: number of logical processors.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\scripts\Build-Diamond.ps1
    # builds diamond.vhdx via WSL

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\scripts\Build-Diamond.ps1 -Target iso
#>
[CmdletBinding()]
param(
    [string]$Target = 'vhdx',
    [string]$Distro,
    [int]   $Jobs   = [Environment]::ProcessorCount
)

$ErrorActionPreference = 'Stop'

# Repo root is the parent of the scripts\ folder this file lives in.
$RepoRoot = Split-Path -Parent $PSScriptRoot

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    throw 'wsl.exe not found. Install WSL2 (wsl --install) and a Linux distro first.'
}

# WSL invocation prefix (optionally pin a distro).
$wslPrefix = @()
if ($Distro) { $wslPrefix += @('-d', $Distro) }

# Translate the Windows repo path into the WSL mount namespace (e.g.
# C:\dev\Diamond -> /mnt/c/dev/Diamond). We do this in PowerShell rather than via
# `wslpath` because passing a backslash path through wsl.exe mangles the separators.
# Assumes the default WSL automount root (/mnt), which is the standard configuration.
function ConvertTo-WslPath([string]$p) {
    $full = [System.IO.Path]::GetFullPath($p)
    if ($full -match '^([A-Za-z]):[\\/](.*)$') {
        $drive = $matches[1].ToLower()
        $rest  = $matches[2] -replace '\\','/'
        return "/mnt/$drive/$rest"
    }
    throw "Cannot translate '$p' to a WSL path. Clone the repo onto a local drive (e.g. C:\...)."
}
$wslRepo = ConvertTo-WslPath $RepoRoot

Write-Host "Repo (Windows): $RepoRoot"           -ForegroundColor Cyan
Write-Host "Repo (WSL):     $wslRepo"             -ForegroundColor Cyan
Write-Host "Building target '$Target' via WSL (make -j$Jobs)..." -ForegroundColor Green

# Login shell (-l) so the user's toolchain PATH is set up. Single-quote the path
# so spaces survive; make runs from the repo root.
$bashCmd = "cd '$wslRepo' && make -j$Jobs $Target"
& wsl.exe @wslPrefix -- bash -lc $bashCmd
$rc = $LASTEXITCODE
if ($rc -ne 0) {
    throw "WSL build failed (exit $rc). Common cause: missing tools. In WSL run:`n" +
          "  sudo apt update && sudo apt install -y build-essential nasm xorriso mtools dosfstools gdisk qemu-utils"
}

if ($Target -eq 'vhdx') {
    $vhdx = Join-Path $RepoRoot 'diamond.vhdx'
    if (-not (Test-Path -LiteralPath $vhdx)) { throw "Build reported success but $vhdx is missing." }
    $mb = [math]::Round((Get-Item -LiteralPath $vhdx).Length / 1MB, 1)
    Write-Host ''
    Write-Host "Built: $vhdx ($mb MB)" -ForegroundColor Green
    Write-Host 'Next: boot it under Hyper-V with scripts\Boot-DiamondHyperV.ps1' -ForegroundColor Green
}
