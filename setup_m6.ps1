# Add LeRobot to the digital-twin venv, for M6 (real hardware).
#
# M1-M5 need only mujoco/numpy/pynput. M6 additionally needs LeRobot's
# SO101Follower driver to talk to a real arm. This installs it HERE, in the
# digital-twin venv, and never touches D:\robotics\so101-vr\.venv - that is
# the separate, working VR/recording stack.
#
# torch is installed CPU-only FIRST, because LeRobot's metadata pulls torch
# and would otherwise fetch a ~2.5GB CUDA build that is useless on integrated
# graphics.
#
# Run:  .\setup_m6.ps1
# If PowerShell blocks it:
#       powershell -ExecutionPolicy Bypass -File .\setup_m6.ps1

$ErrorActionPreference = "Continue"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$VPy  = Join-Path $Root ".venv\Scripts\python.exe"
$Log  = Join-Path $Root "setup_m6_log.txt"

$TelegripVenv = "D:\robotics\so101-vr\.venv"

function Say  ($m) { Write-Host "`n==> $m" -ForegroundColor Cyan;   Add-Content $Log "==> $m" }
function Ok   ($m) { Write-Host "    $m"   -ForegroundColor Green;  Add-Content $Log "    OK: $m" }
function Warn ($m) { Write-Host "    $m"   -ForegroundColor Yellow; Add-Content $Log "    WARN: $m" }
function Fail ($m) {
    Write-Host "`n    FAILED: $m" -ForegroundColor Red
    Add-Content $Log "    FAILED: $m"
    Write-Host "`nFull log: $Log" -ForegroundColor Yellow
    Write-Host "Paste the log if you want help diagnosing it.`n"
    exit 1
}

"=== setup_m6 run $(Get-Date) ===" | Set-Content $Log

# --- 0. guard: make sure we are pointing at the RIGHT venv -------------------
Say "Checking which venv we are about to modify"
if (-not (Test-Path $VPy)) { Fail "digital-twin venv not found at $VPy - run setup.ps1 first" }

$prefix = & $VPy -c "import sys; print(sys.prefix)"
Write-Host "    target: $prefix"
if ($prefix -like "*so101-vr*") {
    Fail "REFUSING TO CONTINUE - that is the telegrip VR venv, not this project's."
}
Ok "correct venv (digital-twin), telegrip venv will not be touched"

# Record telegrip's state so we can prove afterwards that it is unchanged.
$tgPy = Join-Path $TelegripVenv "Scripts\python.exe"
$tgBefore = $null
if (Test-Path $tgPy) {
    $tgBefore = (& $tgPy -m pip list --format=freeze 2>$null) -join "`n"
    Ok "recorded telegrip venv package list for a before/after check"
}

# --- 1. CPU-only torch -------------------------------------------------------
# Must come before lerobot, or pip resolves torch from PyPI and may pick a
# CUDA build (~2.5GB) that this machine cannot use.
Say "Installing CPU-only torch (this is the big download, ~450MB)"
& $VPy -m pip install torch --index-url https://download.pytorch.org/whl/cpu 2>&1 | Tee-Object -Append $Log | Select-Object -Last 3
if ($LASTEXITCODE -ne 0) { Fail "torch install failed" }

$tv = & $VPy -c "import torch; print(torch.__version__)"
if ($tv -notlike "*+cpu*") { Warn "torch is '$tv' - expected a +cpu build. Check before running on hardware." }
else { Ok "torch $tv (CPU-only, no CUDA)" }

# --- 2. lerobot --------------------------------------------------------------
Say "Installing lerobot 0.4.4 (pinned - matches the calibration format in use)"
& $VPy -m pip install "lerobot==0.4.4" 2>&1 | Tee-Object -Append $Log | Select-Object -Last 5
if ($LASTEXITCODE -ne 0) { Fail "lerobot install failed" }

# lerobot imports scservo_sdk lazily inside FeetechMotorsBus but does not
# declare it as a dependency, so `lerobot-calibrate` fails with
# ModuleNotFoundError the first time it touches hardware. Install it explicitly.
Say "Installing feetech-servo-sdk (lerobot needs it but does not declare it)"
& $VPy -m pip install feetech-servo-sdk 2>&1 | Tee-Object -Append $Log | Select-Object -Last 2
if ($LASTEXITCODE -ne 0) { Fail "feetech-servo-sdk install failed" }

# --- 3. verify ---------------------------------------------------------------
Say "Verifying imports"
$check = @'
import sys
mods = ["mujoco", "numpy", "pynput", "serial", "torch", "lerobot"]
bad = 0
for m in mods:
    try:
        mod = __import__(m)
        print("    OK   %-10s %s" % (m, getattr(mod, "__version__", "")))
    except Exception as e:
        bad += 1
        print("    FAIL %-10s %s" % (m, e))
try:
    from lerobot.robots.so_follower.so_follower import SOFollower, SOFollowerRobotConfig
    print("    OK   SOFollower driver importable")
except Exception as e:
    bad += 1
    print("    FAIL SOFollower: %s" % e)
import torch
print("    torch CUDA compiled:", torch.version.cuda)
sys.exit(1 if bad else 0)
'@
$tmp = Join-Path $env:TEMP "m6_check.py"
Set-Content -Path $tmp -Value $check -Encoding utf8
& $VPy $tmp 2>&1 | Tee-Object -Append $Log
$checkFailed = ($LASTEXITCODE -ne 0)
Remove-Item $tmp -ErrorAction SilentlyContinue
if ($checkFailed) { Fail "one or more imports failed - see above" }
Ok "all imports resolve, mujoco and lerobot coexist"

# --- 4. scenes still load ----------------------------------------------------
# lerobot pins numpy; make sure that did not break MuJoCo.
Say "Re-validating scenes (did lerobot's numpy pin break anything?)"
& $VPy (Join-Path $Root "scripts\validate_scenes.py") 2>&1 | Tee-Object -Append $Log | Select-Object -Last 4
if ($LASTEXITCODE -ne 0) { Fail "scenes no longer load after installing lerobot" }
Ok "scenes still load"

# --- 5. prove telegrip is untouched -----------------------------------------
if ($tgBefore) {
    Say "Confirming the telegrip VR venv was not modified"
    $tgAfter = (& $tgPy -m pip list --format=freeze 2>$null) -join "`n"
    if ($tgAfter -eq $tgBefore) { Ok "telegrip venv unchanged" }
    else { Warn "telegrip venv package list CHANGED - this should not happen, investigate" }
}

# --- 6. serial ports ---------------------------------------------------------
Say "Serial ports currently visible"
$ports = [System.IO.Ports.SerialPort]::GetPortNames()
if ($ports) { $ports | ForEach-Object { Write-Host "    $_" } }
else { Warn "no COM ports - connect the arm and power it before M6" }

Write-Host "`nM6 dependencies installed." -ForegroundColor Green
Write-Host @"

  Still to do before driving a real arm:
    1. Transfer real_sim_joint_mapping.py into
       scripts\digital_twin_env\real_sim_mapping_test\   (currently empty)
    2. Calibrate THIS arm - every unit differs:
       .venv\Scripts\lerobot-calibrate.exe --robot.type=so101_follower --robot.port=COM8 --robot.id=<name>
    3. Add a focus guard to keyboard_robot.py - pynput's hook is GLOBAL, so
       as it stands a keystroke in ANY window would move real servos.

  Log: $Log

"@
