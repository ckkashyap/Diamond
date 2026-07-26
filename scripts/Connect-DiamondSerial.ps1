<#
.SYNOPSIS
    Drive the Diamond Hyper-V VM over a COM1 serial console.

.DESCRIPTION
    On a Generation 2 VM you can normally type in the vmconnect window (Diamond ships a
    Hyper-V synthetic keyboard driver). This script is the serial fallback / trace
    console: Diamond mirrors all console output to COM1 and also polls COM1 for input.

    Hyper-V Gen 2 emulates a standard 16550 UART at 0x3F8 that can be surfaced to the
    host as a named pipe. So we:

        1. stop the VM,
        2. attach COM1 to a host named pipe (\\.\pipe\diamond),
        3. start the VM,
        4. open an interactive terminal bridged to that pipe.

    You then type into THIS window and drive the Diamond shell. The graphical VM
    console (vmconnect) keeps showing the framebuffer; input happens here.

    Press Ctrl+C (or Ctrl+]) to quit the serial terminal - the VM keeps running.

.PARAMETER VMName
    Name of the Diamond VM. Default: Diamond

.PARAMETER PipeName
    Host pipe name to use. Default: diamond  (=> \\.\pipe\diamond)

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\scripts\Connect-DiamondSerial.ps1
#>
[CmdletBinding()]
param(
    [string]$VMName   = 'Diamond',
    [string]$PipeName = 'diamond'
)

$ErrorActionPreference = 'Stop'

# --- self-elevate (Set-VMComPort / Start-VM need Administrator) ---------------
$id = [Security.Principal.WindowsIdentity]::GetCurrent()
$isAdmin = (New-Object Security.Principal.WindowsPrincipal($id)).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host 'Elevating (Hyper-V management needs Administrator)...' -ForegroundColor Yellow
    Start-Process -FilePath 'powershell.exe' -Verb RunAs -ArgumentList @(
        '-NoProfile','-ExecutionPolicy','Bypass','-File',"`"$PSCommandPath`"",
        '-VMName',"`"$VMName`"",'-PipeName',"`"$PipeName`""
    )
    return
}

$vm = Get-VM -Name $VMName -ErrorAction SilentlyContinue
if (-not $vm) { throw "VM '$VMName' not found. Run Boot-DiamondHyperV.ps1 first." }

$pipePath = "\\.\pipe\$PipeName"
$curPath  = (Get-VMComPort -VMName $VMName -Number 1).Path

if ($vm.State -eq 'Running' -and $curPath -eq $pipePath) {
    # COM1 is already wired to our pipe and the VM is up: just reconnect. Re-running
    # this script therefore never reboots / disturbs your current Diamond session.
    Write-Host "COM1 already attached and VM running - reconnecting (no reboot)." -ForegroundColor Cyan
} else {
    # Attaching a COM port requires the VM be off.
    if ($vm.State -ne 'Off') {
        Write-Host "Stopping '$VMName' to attach COM1..." -ForegroundColor Yellow
        Stop-VM -Name $VMName -TurnOff -Force
    }
    Write-Host "Attaching COM1 -> $pipePath" -ForegroundColor Cyan
    Set-VMComPort -VMName $VMName -Number 1 -Path $pipePath
    Write-Host "Starting '$VMName'..." -ForegroundColor Green
    Start-VM -Name $VMName
}

# --- console <-> named-pipe bridge (compiled for reliable two-way I/O) --------
Add-Type -TypeDefinition @'
using System;
using System.IO.Pipes;
using System.Threading;

public static class DiamondSerial {
    public static void Run(string server, string pipe) {
        var c = new NamedPipeClientStream(server, pipe,
                    PipeDirection.InOut, PipeOptions.Asynchronous);
        Console.Error.WriteLine("Connecting to \\\\" + server + "\\pipe\\" + pipe + " ...");
        c.Connect(30000);
        Console.Error.WriteLine("Connected. Type to drive the Diamond shell. Ctrl+C to quit.\n");

        // pipe -> console (background)
        var t = new Thread(delegate() {
            var buf = new byte[512];
            try {
                int n;
                while ((n = c.Read(buf, 0, buf.Length)) > 0)
                    for (int i = 0; i < n; i++) Console.Write((char)buf[i]);
            } catch { }
        });
        t.IsBackground = true;
        t.Start();

        // Ctrl+C / Ctrl+] must quit THIS bridge. By default .NET treats Ctrl+C as a
        // kill signal, but the PowerShell host swallows it during a blocking call, so
        // it neither quits nor reaches the guest. Take it as ordinary input instead.
        Console.TreatControlCAsInput = true;

        // console -> pipe (foreground)
        try {
            while (true) {
                var k = Console.ReadKey(true);
                bool ctrl = (k.Modifiers & ConsoleModifiers.Control) != 0;
                if (ctrl && (k.Key == ConsoleKey.C || k.Key == ConsoleKey.Oem6)) {
                    Console.Error.WriteLine("\r\n[disconnected - the VM keeps running]");
                    break;                                   // Ctrl+C or Ctrl+] quits
                }
                byte b;
                if      (k.Key == ConsoleKey.Enter)     b = 13;   // CR (shell accepts CR or LF)
                else if (k.Key == ConsoleKey.Backspace) b = 8;    // BS
                else if (k.Key == ConsoleKey.Escape)    b = 27;   // ESC
                else if (k.Key == ConsoleKey.Tab)       b = 9;    // TAB
                else                                    b = (byte)k.KeyChar;
                if (b == 0) continue;
                c.WriteByte(b);
                c.Flush();
            }
        } catch { }
        try { c.Close(); } catch { }
    }
}
'@

Write-Host ''
Write-Host "=== Diamond serial console ($pipePath) ===" -ForegroundColor Green
[DiamondSerial]::Run('.', $PipeName)
