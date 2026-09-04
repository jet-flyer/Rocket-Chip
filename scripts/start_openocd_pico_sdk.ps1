# Start Pico SDK OpenOCD for RP2350 (CMSIS-DAP). Kills any running openocd.exe first.
# Use this on Windows when bash one-liners break -ArgumentList quoting.
# See docs/agents/DEBUG_PROBE_NOTES.md, docs/FLASHING.md.
# Repo openocd_cmsis_dap.cfg: gdb-attach is halt-only (no SYSRESETREQ).
$ErrorActionPreference = 'Stop'
Get-Process -Name openocd -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2
$repo = Split-Path -Parent $PSScriptRoot
$sdk = Join-Path $env:USERPROFILE '.pico-sdk/openocd/0.12.0+dev'
$exe = Join-Path $sdk 'openocd.exe'
$scr = Join-Path $sdk 'scripts'
# Detach via ShellExecute. UseShellExecute=false + inherited stdin dies
# with EOF when this `powershell -File` process exits — netstat showed
# LISTENING then Python got WinError 10061 (desk 2026-09-03).
$proc = Start-Process -FilePath $exe -ArgumentList "-s `"$scr`" -s `"$repo`" -f openocd_cmsis_dap.cfg" -WorkingDirectory $repo -WindowStyle Hidden -PassThru
Start-Sleep -Seconds 4
if ($proc.HasExited) {
    Write-Host "OpenOCD exited $($proc.ExitCode)."
    exit 1
}
Write-Host "OpenOCD started pid $($proc.Id) (expect TCP 127.0.0.1:3333)."
