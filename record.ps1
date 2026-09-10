# Record a leader -> sim episode. No follower involved, nothing can run away.
#
#   .\record.ps1 my_episode
#
# Move the leader by hand; the sim follows. Ctrl+C or close the viewer to
# stop. Then replay it with:  .\replay.ps1 my_episode

param(
    [Parameter(Mandatory = $true)][string]$Name
)

$py = "D:\robotics\so101-digital-twin\.venv\Scripts\python.exe"
$root = "D:\robotics\so101-digital-twin"
$csv = "$root\recordings\$Name.csv"

if (Test-Path $csv) {
    Write-Host "$Name.csv already exists. Overwrite? (y/N) " -NoNewline -ForegroundColor Yellow
    if ((Read-Host) -ne "y") { exit 1 }
}

Write-Host ""
Write-Host "  RECORD A CLEAN EPISODE" -ForegroundColor Cyan
Write-Host "  1. leave the arm at its resting pose, hold still ~3 s"
Write-Host "  2. move SLOWLY -- fast motion forces a slower replay later"
Write-Host "  3. stay out of tight folds (that is where sim and real disagree)"
Write-Host "  4. return to rest, hold still ~3 s, then Ctrl+C"
Write-Host ""

& $py "$root\scripts\m_leader_mirror_sim.py" `
    --port COM8 --id twin_leader_2 `
    --bare --wires --fps 60 --record $csv

if (Test-Path $csv) {
    Write-Host ""
    Write-Host "  Checking what was recorded..." -ForegroundColor Cyan
    & $py "$root\scripts\check_recording.py" $csv
}
