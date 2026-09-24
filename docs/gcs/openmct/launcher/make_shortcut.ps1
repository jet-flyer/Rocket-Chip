# Creates "Rocket Chip GCS" on the desktop, pointing at rc_gcs.pyw via pythonw (no console).
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $here '..\..\..\..')).Path
$pyw = (Get-Command pythonw).Source
$lnk = Join-Path ([Environment]::GetFolderPath('Desktop')) 'Rocket Chip GCS.lnk'
$s = (New-Object -ComObject WScript.Shell).CreateShortcut($lnk)
$s.TargetPath = $pyw
$s.Arguments = '"' + (Join-Path $here 'rc_gcs.pyw') + '"'
$s.WorkingDirectory = $repo
$s.IconLocation = (Join-Path $here 'rc_gcs.ico') + ',0'
$s.Description = 'Rocket Chip GCS launcher (facsimile / live feeds + Open MCT)'
$s.Save()
"shortcut: $lnk"
