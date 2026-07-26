<#
.SYNOPSIS
    Create (or refresh) a Hyper-V Generation 2 VM that boots Diamond from diamond.vhdx.

.DESCRIPTION
    Diamond boots via the Limine UEFI bootloader. Hyper-V "Generation 2" VMs are UEFI
    machines, so no kernel changes are needed - we just attach the VHDX built by
    `make vhdx` (WSL) and turn OFF Secure Boot (the kernel/Limine aren't Microsoft-signed).

    The script self-elevates (Hyper-V management requires an elevated token), copies the
    VHDX to a stable location so the VM does not depend on the source checkout, creates a
    Gen 2 VM with Secure Boot disabled, attaches the disk, sets it first in the UEFI boot
    order, starts the VM and opens the VM console.

    Keyboard: Gen 2 has no PS/2 controller. Diamond ships a Hyper-V synthetic keyboard
    (VMBus) driver, so you can type directly in the vmconnect window. As a fallback you
    can also drive the shell over COM1 with Connect-DiamondSerial.ps1.

.PARAMETER SourceVhdx
    Path to the VHDX produced by `make vhdx`. Defaults to diamond.vhdx in the repo root
    (the parent of this scripts\ folder).

.PARAMETER VMName
    Name of the Hyper-V VM to create. Default: Diamond

.PARAMETER MemoryMB
    Startup memory (static). Default: 1024

.PARAMETER CPUs
    Virtual processor count. Default: 2

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\scripts\Boot-DiamondHyperV.ps1
#>
[CmdletBinding()]
param(
    [string]$SourceVhdx,
    [string]$VMName     = 'Diamond',
    [int]   $MemoryMB   = 1024,
    [int]   $CPUs       = 2
)

$ErrorActionPreference = 'Stop'

# Default the VHDX to diamond.vhdx in the repo root (parent of this scripts\ folder).
# Computed here rather than as a param default: with [CmdletBinding()], param default
# expressions are evaluated before $PSScriptRoot is bound, so it would be empty.
if (-not $SourceVhdx) {
    $SourceVhdx = Join-Path (Split-Path -Parent $PSScriptRoot) 'diamond.vhdx'
}

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
$vmRoot   = Join-Path $env:USERPROFILE 'Hyper-V\Diamond'
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
Write-Host "Creating Generation 2 VM '$VMName'..." -ForegroundColor Cyan
$vm = New-VM -Name $VMName -Generation 2 -MemoryStartupBytes ($MemoryMB * 1MB) `
             -VHDPath $destVhdx -Path $vmRoot

Set-VMProcessor  -VMName $VMName -Count $CPUs
Set-VMMemory     -VMName $VMName -DynamicMemoryEnabled $false
Set-VM           -VMName $VMName -AutomaticCheckpointsEnabled $false `
                 -CheckpointType Disabled -AutomaticStartAction Nothing

# Diamond / Limine are not Microsoft-signed -> Secure Boot MUST be off.
Set-VMFirmware   -VMName $VMName -EnableSecureBoot Off

# Boot straight from the hard drive.
$hd = Get-VMHardDiskDrive -VMName $VMName
Set-VMFirmware   -VMName $VMName -FirstBootDevice $hd

Write-Host "Starting '$VMName'..." -ForegroundColor Green
Start-VM -Name $VMName

# Open the VM console window.
Start-Process -FilePath 'vmconnect.exe' -ArgumentList $env:COMPUTERNAME, $VMName

Write-Host ''
Write-Host "Done. VM '$VMName' is booting Diamond from:" -ForegroundColor Green
Write-Host "    $destVhdx"
Write-Host 'Secure Boot: OFF   Generation: 2   (UEFI)'
Write-Host 'Type in the vmconnect window (synthetic keyboard), or use Connect-DiamondSerial.ps1 for a COM1 console.'
