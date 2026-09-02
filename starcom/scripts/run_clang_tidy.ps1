# Starcom host clang-tidy. Core first. Not the Rocket-Chip firmware gate.
param(
    [switch]$All,
    [string]$BuildDir = "",
    [string]$ClangTidy = "C:\Program Files\LLVM\bin\clang-tidy.exe"
)
$ErrorActionPreference = "Stop"
$Starcom = Split-Path -Parent $PSScriptRoot
if (-not $BuildDir) { $BuildDir = Join-Path $Starcom "build" }
$Config = Join-Path $Starcom ".clang-tidy"
$CompDb = Join-Path $BuildDir "compile_commands.json"
if (-not (Test-Path $CompDb)) {
    throw "no $CompDb - configure first: cmake -S starcom -B starcom/build -G Ninja"
}
if (-not (Test-Path $ClangTidy)) {
    throw "clang-tidy not found at $ClangTidy"
}
$Files = @(Get-ChildItem (Join-Path $Starcom "src\ccsds\*.cpp"))
$Scope = "core"
if ($All) {
    $Files += @(Get-ChildItem (Join-Path $Starcom "adapters") -Recurse -Filter *.cpp -ErrorAction SilentlyContinue)
    $Scope = "all"
}
$OutDir = Join-Path $Starcom "docs\audits"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$Stamp = Get-Date -Format "yyyy-MM-dd"
$Out = Join-Path $OutDir "CLANG_TIDY_${Stamp}_${Scope}.txt"
"starcom clang-tidy  scope=$Scope  -p $BuildDir  --config-file $Config" | Set-Content -Encoding utf8 $Out
$Fail = $false
foreach ($f in $Files) {
    $rel = $f.FullName.Substring($Starcom.Length).TrimStart("\", "/")
    "---- $rel ----" | Add-Content $Out
    $args = @(
        $f.FullName,
        "-p", $BuildDir,
        "--config-file=$Config",
        "--quiet"
    )
    $proc = Start-Process -FilePath $ClangTidy -ArgumentList $args -NoNewWindow -Wait -PassThru -RedirectStandardOutput "$env:TEMP\sc-tidy-out.txt" -RedirectStandardError "$env:TEMP\sc-tidy-err.txt"
    Get-Content "$env:TEMP\sc-tidy-out.txt","$env:TEMP\sc-tidy-err.txt" -ErrorAction SilentlyContinue | Add-Content $Out
    if ($proc.ExitCode -ne 0) { $Fail = $true }
}
Write-Host "wrote $Out"
if ($Fail) { exit 1 } else { exit 0 }
