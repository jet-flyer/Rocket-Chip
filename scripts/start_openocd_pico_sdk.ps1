# Start Pico SDK OpenOCD for RP2350 (CMSIS-DAP). Kills any running openocd.exe first.
# Use this on Windows when bash one-liners break -ArgumentList quoting.
# See .claude/DEBUG_PROBE_NOTES.md, docs/FLASHING.md.
$ErrorActionPreference = 'Stop'
Get-Process -Name openocd -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2
$sdk = Join-Path $env:USERPROFILE '.pico-sdk/openocd/0.12.0+dev'
$exe = Join-Path $sdk 'openocd.exe'
$scr = Join-Path $sdk 'scripts'
$psi = [System.Diagnostics.ProcessStartInfo]::new()
$psi.FileName = $exe
$psi.Arguments = "-s `"$scr`" -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c `"adapter speed 5000`""
$psi.UseShellExecute = $false
$psi.CreateNoWindow = $true
$null = [System.Diagnostics.Process]::Start($psi)
Start-Sleep -Seconds 4
Write-Host "OpenOCD started (expect TCP 127.0.0.1:3333)."
