# Creates two desktop shortcuts (same icon, rc_gcs.ico), both via pythonw (no console):
#   "Rocket Chip GCS"                -> rc_gcs_app.pyw  (stand-alone app window with controls built in)
#   "Rocket Chip GCS (web launcher)" -> rc_gcs.pyw      (tk launcher + your browser)
# Any older "Rocket Chip GCS.lnk" is replaced, so there is no duplicate.
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $here '..\..\..\..')).Path
$pyw = (Get-Command pythonw).Source
$desk = [Environment]::GetFolderPath('Desktop')
$ico = (Join-Path $here 'rc_gcs.ico') + ',0'
$sh = New-Object -ComObject WScript.Shell

function New-RcLink($name, $script, $desc) {
    $lnk = Join-Path $desk ($name + '.lnk')
    if (Test-Path $lnk) { Remove-Item $lnk -Force }
    $s = $sh.CreateShortcut($lnk)
    $s.TargetPath = $pyw
    $s.Arguments = '"' + (Join-Path $here $script) + '"'
    $s.WorkingDirectory = $repo
    $s.IconLocation = $ico
    $s.Description = $desc
    $s.Save()
    "shortcut: $lnk -> $script"
}

New-RcLink 'Rocket Chip GCS' 'rc_gcs_app.pyw' 'Rocket Chip GCS - Open MCT glass in its own window, controls built in'
New-RcLink 'Rocket Chip GCS (web launcher)' 'rc_gcs.pyw' 'Rocket Chip GCS web launcher (tk window + browser dashboard)'
