<#
.SYNOPSIS
    One command: build Diamond in WSL, then boot it under Hyper-V.

.DESCRIPTION
    This is the "clone and go" entry point on Windows. It:

        1. builds diamond.vhdx by driving the WSL toolchain (scripts\Build-Diamond.ps1), then
        2. creates/refreshes a Hyper-V VM from that VHDX and opens its console.

    Generation 2 (UEFI, default) uses Diamond's synthetic keyboard - type in the
    vmconnect window. Generation 1 (-Generation 1) is legacy BIOS with an emulated PS/2
    keyboard. The same diamond.vhdx boots both.

    The build runs as the current user (WSL). The Hyper-V step self-elevates.

.PARAMETER Generation
    Hyper-V VM generation: 2 (UEFI, default) or 1 (legacy BIOS / native PS/2 keyboard).

.PARAMETER SkipBuild
    Skip the WSL build and boot the existing diamond.vhdx.

.PARAMETER Distro
    Optional WSL distribution name for the build (passed to `wsl -d`).

.PARAMETER VMName
    Override the VM name (defaults: Diamond for Gen 2, Diamond-Gen1 for Gen 1).

.PARAMETER MemoryMB
    Startup memory (static). Default: 1024

.PARAMETER CPUs
    Virtual processor count. Default: 2

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\scripts\Start-DiamondHyperV.ps1
    # build via WSL, then boot a Gen 2 VM

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\scripts\Start-DiamondHyperV.ps1 -Generation 1 -SkipBuild
#>
[CmdletBinding()]
param(
    [ValidateSet('2','1')]
    [string]$Generation = '2',
    [switch]$SkipBuild,
    [string]$Distro,
    [string]$VMName,
    [int]   $MemoryMB = 1024,
    [int]   $CPUs     = 2
)

$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Vhdx     = Join-Path $RepoRoot 'diamond.vhdx'

# --- 1) build (WSL) ----------------------------------------------------------
if (-not $SkipBuild) {
    Write-Host '=== Step 1/2: building diamond.vhdx via WSL ===' -ForegroundColor Magenta
    $buildArgs = @{ Target = 'vhdx' }
    if ($Distro) { $buildArgs.Distro = $Distro }
    & (Join-Path $PSScriptRoot 'Build-Diamond.ps1') @buildArgs
} else {
    Write-Host 'Skipping build (-SkipBuild).' -ForegroundColor Yellow
}

if (-not (Test-Path -LiteralPath $Vhdx)) {
    throw "diamond.vhdx not found at $Vhdx. Remove -SkipBuild, or build first with scripts\Build-Diamond.ps1."
}

# --- 2) boot (Hyper-V) -------------------------------------------------------
Write-Host "=== Step 2/2: booting Diamond under Hyper-V (Generation $Generation) ===" -ForegroundColor Magenta
$bootScript = if ($Generation -eq '2') { 'Boot-DiamondHyperV.ps1' } else { 'Boot-DiamondHyperV-Gen1.ps1' }

$bootParams = @{ SourceVhdx = $Vhdx; MemoryMB = $MemoryMB; CPUs = $CPUs }
if ($VMName) { $bootParams.VMName = $VMName }

& (Join-Path $PSScriptRoot $bootScript) @bootParams
