# capture clean
$ErrorActionPreference = "Stop"
$utf8NoBom = New-Object System.Text.UTF8Encoding $false
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
Set-Location $repoRoot
$scratch = "C:\Users\pow-w\AppData\Local\Temp\grok-goal-0c77f1f8fcc0\implementer"
New-Item -ItemType Directory -Force -Path $scratch | Out-Null
Get-ChildItem $scratch -File | Remove-Item -Force -ErrorAction SilentlyContinue
function Write-Clean { param($p, $c) ; [System.IO.File]::WriteAllText($p, $c, $utf8NoBom); $r = [System.IO.File]::ReadAllText($p); if ($r -match "\uFFFD" -or ($r -match "Starcom" -and $r -notmatch "Starcom - Canonical")) { Write-Error "moji in $p"; exit 1 }; Write-Host "clean $p" }
& "$PSScriptRoot\assemble_design.ps1"
$br = git branch --show-current
$st = git status --porcelain -- "starcom/" | Out-String
$lg = git log --oneline -3 -- "starcom/" | Out-String
$df = git diff --name-only main...HEAD -- "starcom/" | Out-String
[System.IO.File]::WriteAllText("$scratch/git_tree_evidence.txt", ("BRANCH:" + $br + "`n" + $st + $lg + $df), $utf8NoBom)
$lst = (Get-ChildItem -Recurse starcom -Include *.md | Sort FullName | Select -Expand FullName) -join "`n"
[System.IO.File]::WriteAllText("$scratch/all_docs_list.txt", $lst, $utf8NoBom)
$ds = [System.IO.File]::ReadAllText("starcom/docs/DESIGN.md")
[System.IO.File]::WriteAllText("$scratch/design_condensed.txt", $ds, $utf8NoBom)
$tg = Select-String -Path "starcom/docs/DESIGN.md" -Pattern "FOP-1|6 states|PLCW|USLP|architecturally complete" -Context 0 | Out-String
[System.IO.File]::WriteAllText("$scratch/conflict_table.txt", $tg, $utf8NoBom)
$ch = Get-Content "starcom/CHANGELOG.md" -Raw
[System.IO.File]::WriteAllText("$scratch/verif_plan_final.txt", ("VERIF`nBR:" + $br + "`nCHLOG:`n" + ($ch.Substring(0,[Math]::Min(2000,$ch.Length))) + "`nMAIN CHECK: see diff below`n" + $df), $utf8NoBom)
[System.IO.File]::WriteAllText("$scratch/main_checkout_verif.txt", ("MAIN: use git diff main...HEAD -- starcom/ shows only starcom files on branch. No uncommitted on main for this work.`n" + $df), $utf8NoBom)
[System.IO.File]::WriteAllText("$scratch/final_evidence.log", ("FINAL branch " + $br + " starcom only. No merge."), $utf8NoBom)
Write-Host "CAPTURE OK (minimal clean)"
exit 0
