<#
.SYNOPSIS
    Create (or refresh) a Hyper-V Generation 1 VM that boots Diamond from diamond.vhdx.

.DESCRIPTION
    Hyper-V "Generation 1" VMs are legacy-BIOS machines with EMULATED hardware -
    including a PS/2 keyboard and IDE disks. Diamond's keyboard driver talks to the
    i8042 PS/2 controller, so on a Gen 1 VM you can type directly in the vmconnect
    window (no serial console needed).

    This works because `make vhdx` also runs `limine bios-install` on the disk, so the
    same diamond.vhdx boots both ways:
        Gen 2 (UEFI)  -> Boot-DiamondHyperV.ps1       (synthetic keyboard / COM1 serial)
        Gen 1 (BIOS)  -> Boot-DiamondHyperV-Gen1.ps1  (native PS/2 keyboard in window)

    The script self-elevates (Hyper-V management requires an elevated token), copies the
    VHDX to a stable location so the VM does not depend on the source checkout, creates a
    Gen 1 VM, attaches the disk to IDE, sets it first in the BIOS boot order, starts the
    VM and opens the VM console.

.PARAMETER SourceVhdx
    Path to the VHDX produced by `make vhdx`. Defaults to diamond.vhdx in the repo root
    (the parent of this scripts\ folder).

.PARAMETER VMName
    Name of the Hyper-V VM to create. Default: Diamond-Gen1

.PARAMETER MemoryMB
    Startup memory (static). Default: 1024

.PARAMETER CPUs
    Virtual processor count. Default: 2

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\scripts\Boot-DiamondHyperV-Gen1.ps1
#>
[CmdletBinding()]
param(
    [string]$SourceVhdx = (Join-Path (Split-Path -Parent $PSScriptRoot) 'diamond.vhdx'),
    [string]$VMName     = 'Diamond-Gen1',
    [int]   $MemoryMB   = 1024,
    [int]   $CPUs       = 2
)

$ErrorActionPreference = 'Stop'

# --- self-elevate ------------------------------------------------------------
$id = [Security.Principal.WindowsIdentity]::GetCurrent()
$isAdmin = (New-Object Security.Principal.WindowsPrincipal($id)).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host 'Elevating (Hyper-V management needs Administrator)...' -ForegroundColor Yellow
    $psi = @(
        '-NoProfile','-ExecutionPolicy','Bypass','-File',"`"$PSCommandPath`"",
        '-SourceVhdx',"`"$SourceVhdx`"",'-VMName',"`"$VMName`"",
        '-MemoryMB',"$MemoryMB",'-CPUs',"$CPUs"
    )
    Start-Process -FilePath 'powershell.exe' -Verb RunAs -ArgumentList $psi
    return
}

# --- checks ------------------------------------------------------------------
if (-not (Get-Command Get-VM -ErrorAction SilentlyContinue)) {
    throw 'Hyper-V PowerShell module not found. Enable Hyper-V first: ' +
          'Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V -All'
}
if (-not (Test-Path -LiteralPath $SourceVhdx)) {
    throw "VHDX not found: $SourceVhdx  (build it first from WSL with: make vhdx, " +
          'or run scripts\Build-Diamond.ps1)'
}

# --- remove any existing VM FIRST --------------------------------------------
# Must happen before we overwrite the destination VHDX: while the VM exists,
# Hyper-V (VMMS) holds a lock on its attached disk and Copy-Item would fail with
# "because it is being used by another process".
$existing = Get-VM -Name $VMName -ErrorAction SilentlyContinue
if ($existing) {
    Write-Host "Removing existing VM '$VMName'..." -ForegroundColor Yellow
    if ($existing.State -ne 'Off') { Stop-VM -Name $VMName -TurnOff -Force }
    Remove-VM -Name $VMName -Force
}

# --- stable copy of the disk (so the VM survives the checkout being cleaned) --
# Keep Gen 1 in its own folder so it never collides with the Gen 2 VM's disk.
$vmRoot   = Join-Path $env:USERPROFILE (Join-Path 'Hyper-V' $VMName)
$destVhdx = Join-Path $vmRoot 'diamond.vhdx'
New-Item -ItemType Directory -Force -Path $vmRoot | Out-Null

# VMMS can keep the old disk locked for a moment after Remove-VM; wait for the
# handle to drop, then delete it so the fresh copy can't be blocked.
if (Test-Path -LiteralPath $destVhdx) {
    for ($i = 0; $i -lt 20; $i++) {
        try {
            $s = [IO.File]::Open($destVhdx, 'Open', 'ReadWrite', 'None'); $s.Close(); break
        } catch { Start-Sleep -Milliseconds 500 }
    }
    Remove-Item -LiteralPath $destVhdx -Force -ErrorAction SilentlyContinue
}

Write-Host "Copying VHDX -> $destVhdx" -ForegroundColor Cyan
Copy-Item -LiteralPath $SourceVhdx -Destination $destVhdx -Force

# --- create the VM -----------------------------------------------------------
# Generation 1 -> legacy BIOS + emulated PS/2 keyboard. -VHDPath attaches the
# disk to IDE controller 0 (the only bootable controller on Gen 1).
Write-Host "Creating Generation 1 VM '$VMName'..." -ForegroundColor Cyan
$vm = New-VM -Name $VMName -Generation 1 -MemoryStartupBytes ($MemoryMB * 1MB) `
             -VHDPath $destVhdx -Path $vmRoot

Set-VMProcessor  -VMName $VMName -Count $CPUs
Set-VMMemory     -VMName $VMName -DynamicMemoryEnabled $false
Set-VM           -VMName $VMName -AutomaticCheckpointsEnabled $false `
                 -CheckpointType Disabled -AutomaticStartAction Nothing

# Gen 1 uses the legacy BIOS boot order (no UEFI / no Secure Boot to configure).
# Boot straight from the IDE hard drive.
Set-VMBios       -VMName $VMName -StartupOrder @('IDE','CD','LegacyNetworkAdapter','Floppy')

Write-Host "Starting '$VMName'..." -ForegroundColor Green
Start-VM -Name $VMName

# Open the VM console window - type directly here; the PS/2 keyboard is emulated.
Start-Process -FilePath 'vmconnect.exe' -ArgumentList $env:COMPUTERNAME, $VMName

Write-Host ''
Write-Host "Done. VM '$VMName' is booting Diamond from:" -ForegroundColor Green
Write-Host "    $destVhdx"
Write-Host 'Secure Boot: N/A   Generation: 1   (legacy BIOS, native PS/2 keyboard)'
Write-Host 'Click into the vmconnect window and type - no serial console required.'
