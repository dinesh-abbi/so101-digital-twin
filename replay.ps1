# Replay a sim recording onto the real arm.
#
#   .\replay.ps1 dataset_2 12 40 0.4
#   .\replay.ps1 dataset_2            <- auto-picks the window and speed
#
# Everything else (ports, --source sim, --bare, --skip-joints wrist_roll,
# --approach) is fixed, because it is the same every time on this bench.

param(
    [Parameter(Mandatory = $true)][string]$Name,
    [double]$Start = -1,
    [double]$End = -1,
    [double]$Speed = 0
)

$py = "D:\robotics\so101-digital-twin\.venv\Scripts\python.exe"
$root = "D:\robotics\so101-digital-twin"
$csv = "$root\recordings\$Name.csv"

if (-not (Test-Path $csv)) {
    Write-Host "No such recording: $csv" -ForegroundColor Red
    Write-Host "Available:"
    Get-ChildItem "$root\recordings\*.csv" | ForEach-Object { "  " + $_.BaseName }
    exit 1
}

# No window given -> ask the analyser for the best one.
if ($Start -lt 0) {
    Write-Host "Finding the clean window..." -ForegroundColor Cyan
    $picked = & $py "$root\scripts\pick_window.py" $csv
    if ($LASTEXITCODE -ne 0) { Write-Host $picked; exit 1 }
    $parts = $picked -split "\s+"
    $Start = [double]$parts[0]
    $End = [double]$parts[1]
    $Speed = [double]$parts[2]
    Write-Host "  window $Start..$End  speed $Speed" -ForegroundColor Green
}
if ($Speed -le 0) { $Speed = 0.4 }

& $py "$root\scripts\replay_teleop_real.py" $csv `
    --source sim --window $Start $End --speed $Speed `
    --follower-port COM14 --follower-id twin_follower_3 `
    --bare --wires --fps 60 --skip-joints wrist_roll
