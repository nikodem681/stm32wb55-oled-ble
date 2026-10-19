# Sync the NUCLEO-WB55 clock with this PC over USB.
#
# Finds the board's virtual COM port by its USB ID (VID 0483, PID 5740),
# waits for the start of the next second and sends:  time YYYY-MM-DD HH:MM:SS
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools\sync_time.ps1
#   powershell -ExecutionPolicy Bypass -File tools\sync_time.ps1 -Port COM3
#
# Note: a COM port can be open in only one program. Close PuTTY first.

param([string]$Port)

if (-not $Port) {
    $dev = Get-CimInstance Win32_PnPEntity |
           Where-Object { $_.PNPDeviceID -match 'VID_0483&PID_5740' -and $_.Name -match '\((COM\d+)\)' } |
           Select-Object -First 1
    if (-not $dev) { Write-Host "Board not found. Is the cable in the USB_USER connector?"; exit 1 }
    $null = $dev.Name -match '\((COM\d+)\)'
    $Port = $Matches[1]
}

$p = New-Object System.IO.Ports.SerialPort $Port, 115200
$p.DtrEnable = $true          # the board answers only when DTR ("terminal open") is set
try {
    $p.Open()
} catch {
    Write-Host "Cannot open $Port - is it open in PuTTY or another program?"
    exit 1
}

Start-Sleep -Milliseconds 600  # let the board print its greeting
$null = $p.ReadExisting()

# Wait for the start of the next second, so the board's seconds tick together with the PC
$ms = (Get-Date).Millisecond
Start-Sleep -Milliseconds (1000 - $ms)
$now = Get-Date
$p.Write("time " + $now.ToString("yyyy-MM-dd HH:mm:ss") + "`r")

Start-Sleep -Milliseconds 300
$reply = $p.ReadExisting()
$p.Close()

Write-Host "PC time : $($now.ToString('yyyy-MM-dd HH:mm:ss.fff')) ($Port)"
Write-Host ("Board   : " + ($reply -split "`r?`n" | Where-Object { $_ -match 'Clock set|Now:' } | Select-Object -First 1))
