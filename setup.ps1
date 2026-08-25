# Set up so101-digital-twin on Windows.
#
# Creates a virtual environment inside the repo, installs mujoco/numpy/pynput,
# and headlessly validates every scene. Does not touch any other Python
# installation or virtual environment on the machine.
#
# Run:  .\setup.ps1
# If PowerShell blocks it:
#       powershell -ExecutionPolicy Bypass -File .\setup.ps1

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Venv = Join-Path $Root ".venv"

function Say  ($m) { Write-Host "`n==> $m" -ForegroundColor Cyan }
function Warn ($m) { Write-Host "    warning: $m" -ForegroundColor Yellow }
function Die  ($m) { Write-Host "    error: $m" -ForegroundColor Red; exit 1 }

# --- 1. find a usable Python -------------------------------------------------
# MuJoCo publishes wheels for 3.9-3.12. Newer versions have no wheel and pip
# would attempt a source build that fails without MSVC, so pick a supported
# interpreter explicitly rather than whatever `python` happens to be.
Say "Looking for Python 3.9-3.12"

$Py = $null
foreach ($v in @("3.12", "3.11", "3.10", "3.9")) {
    try {
        $out = & py "-$v" --version 2>&1
        if ($LASTEXITCODE -eq 0) { $Py = @("py", "-$v"); Write-Host "    found $out via py launcher"; break }
    } catch { }
}

if (-not $Py) {
    # No py launcher, or no suitable version registered with it.
    try {
        $ver = & python -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>&1
        if ($LASTEXITCODE -eq 0) {
            $parts = $ver.Split('.')
            if ([int]$parts[0] -eq 3 -and [int]$parts[1] -ge 9 -and [int]$parts[1] -le 12) {
                $Py = @("python"); Write-Host "    found python $ver on PATH"
            }
        }
    } catch { }
}

if (-not $Py) {
    Die @"
No Python 3.9-3.12 found. MuJoCo has no wheels for 3.13+ yet.

    Install Python 3.11 from https://www.python.org/downloads/release/python-3119/
    (Windows installer, 64-bit). Tick "py launcher" during install.
"@
}

# --- 2. sanity-check the layout ---------------------------------------------
# The robot scenes reach up three levels for the assets, so these two
# directories must be siblings or those scenes cannot load.
Say "Checking repository layout"
foreach ($p in @("so101_assets\assets", "so101_assets\so101.xml", "scripts\digital_twin_env")) {
    if (-not (Test-Path (Join-Path $Root $p))) { Die "$p is missing - was the repo cloned completely?" }
}
$meshes = @(Get-ChildItem (Join-Path $Root "so101_assets\assets") -Filter *.stl -ErrorAction SilentlyContinue)
Write-Host "    $($meshes.Count) meshes found"
if ($meshes.Count -eq 0) { Die "no STL meshes - if you cloned with git-lfs, run: git lfs pull" }

# --- 3. virtual environment --------------------------------------------------
Say "Creating virtual environment at .venv"
if (Test-Path $Venv) {
    Write-Host "    already exists, reusing"
} else {
    & $Py[0] $Py[1..($Py.Length-1)] -m venv $Venv
    if ($LASTEXITCODE -ne 0) { Die "venv creation failed" }
}
$VPy = Join-Path $Venv "Scripts\python.exe"
if (-not (Test-Path $VPy)) { Die "venv python not found at $VPy" }

# --- 4. dependencies ---------------------------------------------------------
Say "Installing dependencies"
& $VPy -m pip install --upgrade pip --quiet
& $VPy -m pip install --quiet mujoco numpy pynput
if ($LASTEXITCODE -ne 0) { Die "dependency install failed" }
& $VPy -c "import mujoco, numpy, pynput; print('    mujoco', mujoco.__version__, '| numpy', numpy.__version__)"

# --- 5. headless validation --------------------------------------------------
# Load every scene without opening a window: an asset-path problem is much
# easier to diagnose separately from a windowing one.
Say "Validating scenes (headless)"
& $VPy (Join-Path $Root "scripts\validate_scenes.py")
if ($LASTEXITCODE -ne 0) { Die "scene validation failed - see output above" }

# --- 6. OpenGL ---------------------------------------------------------------
# The interactive viewer needs an OpenGL 3.3 core context. Integrated graphics
# is usually fine, but old drivers are not, so probe rather than assume.
Say "Checking OpenGL (needed only for the interactive viewer)"
$glProbe = @'
import sys
try:
    import glfw
    if not glfw.init():
        print("    could not initialise GLFW"); sys.exit(0)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    w = glfw.create_window(320, 240, "probe", None, None)
    if not w:
        print("    no OpenGL 3.3 context - update your graphics driver")
    else:
        glfw.make_context_current(w)
        import OpenGL.GL as gl
        print("    " + gl.glGetString(gl.GL_RENDERER).decode())
        print("    OpenGL " + gl.glGetString(gl.GL_VERSION).decode())
        glfw.destroy_window(w)
    glfw.terminate()
except Exception as e:
    print("    OpenGL probe skipped:", e)
'@
$tmp = Join-Path $env:TEMP "so101_glprobe.py"
Set-Content -Path $tmp -Value $glProbe -Encoding utf8
& $VPy $tmp
Remove-Item $tmp -ErrorAction SilentlyContinue

Write-Host "`nSetup complete." -ForegroundColor Green
Write-Host @"

  Validate again:   .venv\Scripts\python.exe scripts\validate_scenes.py
  Table viewer:     .venv\Scripts\python.exe scripts\digital_twin_env\run_table.py
  Robot on table:   cd scripts\digital_twin_env\robot_on_table_test
                    ..\..\..\.venv\Scripts\python.exe run_robot_on_table.py
  Keyboard control: cd scripts\digital_twin_env\robot_keyboard_test
                    ..\..\..\.venv\Scripts\python.exe keyboard_robot.py

  Controls: Q/A W/S E/D R/F T/G Y/H for the 6 joints, Space reset, Esc quit.

  NOTE On Windows keep the TERMINAL focused while driving joints, not the
  viewer - MuJoCo's viewer reserves W/S/T and the number row for its own
  toggles, and both handlers fire when the viewer has focus.

"@
