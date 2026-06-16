<#
  run.ps1 - Windows/PowerShell version of run.sh.

  Requires gcc + make (MSYS2/MinGW) and Python on PATH.

    .\run.ps1 curve live
    .\run.ps1 movevp anim --save run.gif
    .\run.ps1 curve static

  scenario : movevp | curve            (default movevp)
  mode     : static | anim | live      (default anim)
  remaining args are forwarded to the Python visualiser.

  Tip: if scripts are blocked, run once:
    Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
  or invoke as:  powershell -ExecutionPolicy Bypass -File .\run.ps1 curve live
#>
param(
    [string]$Scenario = "movevp",
    [string]$Mode = "anim"
)
$Rest = $args   # extra args -> Python

$ErrorActionPreference = "Stop"
$Dir = Split-Path -Parent $MyInvocation.MyCommand.Path

# locate make (MSYS2 'make' or MinGW 'mingw32-make')
$mk = Get-Command make -ErrorAction SilentlyContinue
if (-not $mk) { $mk = Get-Command mingw32-make -ErrorAction SilentlyContinue }
if (-not $mk) {
    Write-Error "make not found. Install MSYS2/MinGW and add it to PATH (or use WSL)."
    exit 1
}

Write-Host ">> building C simulator"
& $mk.Path -C "$Dir\sim"

$bin = Join-Path $Dir "sim\motion_sim.exe"
$csv = Join-Path $Dir "sim\$($Scenario)_log.csv"

switch ($Mode) {
    "live" {
        Write-Host ">> live: C simulator -> Python (piped)"
        Push-Location "$Dir\python"
        python animate_live.py $Scenario --bin $bin @Rest
        Pop-Location
    }
    "static" {
        Write-Host ">> running C simulator -> $csv"
        & $bin $Scenario --out $csv
        Push-Location "$Dir\python"
        python visualize.py $csv @Rest
        Pop-Location
    }
    "anim" {
        Write-Host ">> running C simulator -> $csv"
        & $bin $Scenario --out $csv
        Push-Location "$Dir\python"
        python animate.py $csv @Rest
        Pop-Location
    }
    default {
        Write-Error "unknown mode '$Mode' (use static|anim|live)"
        exit 2
    }
}
