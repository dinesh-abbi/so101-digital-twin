# CLAUDE.md — SO-101 Real-to-Sim Digital Twin (Windows)

Guidance for Claude Code on THIS machine (Windows 10, i3-7020U 2C/4T,
11.9GB RAM, Intel HD 620 integrated graphics, no CUDA). Read this before
touching anything — it explains what already exists, what's validated, and
what's actually left to do.

## What this project is

A real-to-sim digital twin of an SO-101 robot arm workspace: an IKEA
LINNMON/ADILS table modeled in MuJoCo, the SO-101 arm placed on it, and a
plan to synchronize a real physical follower arm with the simulated one —
first via keyboard input (standing in for a leader arm we don't have yet),
later via VR or an actual leader arm.

This machine's job: **run the real SO-101 follower arm** and help verify the
real↔sim joint mapping. The heavy MuJoCo development already happened on a
separate Linux server and is DONE and VALIDATED there — do not redo it.

## Two unrelated projects live on this PC — do not conflate them

1. **`D:\robotics\so101-vr\`** — an EARLIER, SEPARATE project: `telegrip`
   VR/keyboard teleoperation + LeRobot dataset recording. Python 3.11.9,
   `torch 2.10.0+cpu`, `lerobot==0.4.4`, `pybullet 3.2.7` (compiled from
   source with MSVC). Working: 6/6 servos calibrated, keyboard AND Quest 2
   VR control verified live, clean 20Hz dataset recordings on COM8.
   **Do not modify, reuse, or install into that venv.**
2. **`D:\robotics\so101-digital-twin\`** — THIS project. Separate venv,
   needs only `mujoco`, `numpy`, `pynput`. No `torch`, `lerobot`, or
   `pybullet`.

Both venvs are built from the same base interpreter
(`C:\Users\abbid\AppData\Local\Programs\Python\Python311`, 3.11.9) but are
fully isolated. Sharing the base interpreter is read-only and safe.

If a task here needs `lerobot`'s hardware driver (`SO101Follower`,
`lerobot-calibrate`), that's a deliberate bridge to project 1 — confirm
with the user first, since it changes this venv's weight substantially.

## Layout on this machine

```
D:\robotics\so101-digital-twin\
├── .venv\                          Python 3.11.9, mujoco+numpy+pynput only
├── so101_assets\                   SO-101 model + 20 STL meshes (18.3 MB)
│   ├── so101.xml                   NEVER edit — everything includes this
│   ├── assets\                     meshes (meshdir target)
│   ├── PROVENANCE.md               records the local fixes vs upstream
│   └── CHANGELOG.md
└── scripts\
    ├── validate_scenes.py          headless load-check for all 4 scenes
    └── digital_twin_env\
        ├── table_scene.xml, run_table.py, validate_table_scene.py
        ├── spawn_cube_test\
        ├── robot_on_table_test\
        ├── robot_keyboard_test\
        └── real_sim_mapping_test\
```

### The `scripts\` level is load-bearing

`robot_on_table_scene.xml` and `keyboard_robot_scene.xml` reference assets
three levels up:

```xml
<compiler meshdir="../../../so101_assets/assets"/>
<include file="../../../so101_assets/so101.xml"/>
```

From `scripts\digital_twin_env\robot_on_table_test\`, that walks
`digital_twin_env` → `scripts` → project root → finds `so101_assets\`.
**Removing the `scripts\` level breaks both robot scenes** with a
missing-file error. The folders were initially transferred to
`D:\robotics\` directly and had to be restructured for this reason.

`so101_assets\` is vendored from MuJoCo Menagerie (Apache 2.0, pinned
commit `4c358ef`) with project-specific fixes — the `fovy="48.5"` camera
change and the `gripperframe` quaternion fix (`1 0 1 0` → `0 0 1 0`).
**Never re-download from upstream** — a fresh copy lacks these.

## Status

| Milestone | What | Status |
|---|---|---|
| M1 | LINNMON/ADILS table (`table_scene.xml`) | DONE, validated on Linux + **loads on Windows** |
| M2 | Runtime cube spawning (`spawn_cube_test/`) | DONE, validated + **loads on Windows** |
| M3 | SO-101 + table (`robot_on_table_test/`) | DONE, validated + **loads on Windows** |
| M4 | Joint range/axis sanity check | DONE (part of M3) |
| M5 | Keyboard → MuJoCo (`robot_keyboard_test/`) | DONE, validated + **RUNS on Windows** (see hotkey note) |
| M6-prep | Real↔sim mapping math (`real_sim_mapping_test/`) | **FILES MISSING ON THIS MACHINE — see below** |
| M6 | Keyboard → real follower arm | **NOT STARTED — happens here** |
| M7 | Keyboard → real follower + MuJoCo together | Not started |

### ⚠️ `real_sim_mapping_test/` is EMPTY on this machine

The folder transferred but `real_sim_joint_mapping.py` did not. M6 depends on
it. **Transfer it from the Linux dev machine before starting M6** — do not
rewrite it from scratch; it is already unit-tested there and re-deriving the
normalization math invites subtle sign/range errors.

## Windows validation results (measured 2026-08-25, not assumed)

### Headless scene loading — 4/4 PASS

`python scripts\validate_scenes.py`

| Scene | bodies | joints | geoms | meshes | actuators |
|---|---:|---:|---:|---:|---:|
| `table_scene.xml` | 2 | 0 | 6 | 0 | 0 |
| `spawn_cube_scene.xml` | 3 | 1 | 7 | 0 | 0 |
| `robot_on_table_scene.xml` | 10 | 6 | 54 | 18 | 6 |
| `keyboard_robot_scene.xml` | 10 | 6 | 54 | 18 | 6 |

All integrate 100 steps with finite state. mujoco 3.12.0.

### OpenGL — works, but with ZERO headroom

```
GLFW runtime : 3.4.0 Win32 WGL Null EGL OSMesa VisualC DLL
OpenGL vendor: Intel
renderer     : Intel(R) HD Graphics 620
version      : 3.3.0 - Build 27.20.100.9664
GLSL         : 3.30
```

**MuJoCo requires OpenGL 3.3; this GPU reports exactly 3.3.0.** It works,
but there is no margin. If rendering misbehaves, suspect the driver before
the code. GLFW uses the **Win32 WGL** backend — no X11 anywhere, unlike the
Linux dev machine.

MuJoCo's offscreen `Renderer` produced a real image (320×240, pixel range
0–255, mean 107), so the GL pipeline is genuinely functional, not a stub.

### Sim joint ranges (read live from the compiled model)

| Joint | radians | degrees | span |
|---|---|---|---:|
| shoulder_pan | -1.91986 .. 1.91986 | -110.0 .. 110.0 | 220.0° |
| shoulder_lift | -1.74533 .. 1.74533 | -100.0 .. 100.0 | 200.0° |
| elbow_flex | -1.69000 .. 1.69000 | -96.8 .. 96.8 | 193.7° |
| wrist_flex | -1.65806 .. 1.65806 | -95.0 .. 95.0 | 190.0° |
| wrist_roll | -2.74385 .. 2.74385 | -157.2 .. 157.2 | 314.4° |
| gripper | -0.17453 .. 1.74533 | -10.0 .. 100.0 | 110.0° |

### Interactive viewer — WORKS

`mujoco.viewer.launch_passive` opens a real GLFW window on this machine.
Confirmed visually for M1 (table), M2 (cube), M3 (robot + table) and M5
(keyboard scene). Shadows, lighting and the full MuJoCo UI panels render
correctly on Intel HD 620.

### ⚠️ Keyboard hotkey collision — the docstring in `keyboard_robot.py` is WRONG on Windows

`keyboard_robot.py`'s docstring claims pynput gives "zero possible
collisions with any viewer-reserved key." **That is not true on Windows.**

Measured behaviour on this machine:

| Focused window | Result |
|---|---|
| **MuJoCo viewer** | Joints move **AND** MuJoCo's own hotkeys fire — camera POV jumps, wireframe/mesh/transparency toggle. Both handlers receive the key. |
| **Terminal** | Joints move cleanly, no render interference. ✅ |

pynput's global Win32 hook receives the key, but it does **not** prevent
MuJoCo from receiving it too when the viewer has focus. On Linux/X11 the
behaviour differed enough that this was not hit.

**Workaround in use: keep the TERMINAL focused while driving joints.**
No code change needed for sim-only work.

### 🔴 Safety requirement before M6

`pynput_keyboard.Listener` is created with no `suppress` argument, so the
hook is **global** — keys register regardless of focus. Harmless in
simulation. **Once M6 drives a real arm, any keystroke in any window
(browser, terminal, editor) could move real servos.**

Before connecting hardware, add a focus guard (`GetForegroundWindow()`) so
joint keys only apply when the intended window is focused, or rebind away
from MuJoCo's reserved keys (W/S/T and the number row). Do not connect the
real arm to this script as-is.

## The real↔sim joint mapping (read before M6)

Two coordinate systems that do NOT naturally agree:

- **Real servo (Feetech STS3215):** raw 0–4095 encoder ticks, arbitrary
  zero. `lerobot-calibrate` records each joint's observed min/max, then
  normalizes to `-100..100` for the 5 arm joints and `0..100` for the
  gripper. `SO101Follower.get_observation()` returns those normalized
  values — **not** ticks, **not** degrees.
- **Sim (`so101.xml`):** radians, from the kinematic model, unrelated to
  any servo's calibrated range.

`real_sim_joint_mapping.py` does a per-joint linear rescale, mirroring
LeRobot's `_normalize`/`_unnormalize`. This guarantees the same direction
and relative position in range — **but NOT that real 0% and sim 0 rad are
the same physical pose.** Verify by eye with hardware connected: command
known real positions, watch whether the sim matches. Do not trust the math
alone.

**Known span mismatches**, confirmed against this machine's compiled model
and a real calibration done 2026-08-22 on the arm at
`C:\Users\abbid\.cache\huggingface\lerobot\calibration\robots\so_follower\left_follower.json`:

| Joint | Real calibrated | Sim kinematic |
|---|---:|---:|
| wrist_roll | 359.9° | **314.4°** |
| gripper | 143.0° | **110.0°** |

These don't break the mapping, but "50% open" will not look identical
between real and sim until checked visually.

## Windows differences from the Linux dev machine

1. **No X11, no container.** The Linux setup ran in Docker
   (`sim-ubuntu-desktop`, `DISPLAY=:1`). **Ignore every `*_container.sh`
   file here** — they reference `../table_scan/container_venv`, which does
   not exist. Run the `.py` files with this project's venv Python.
2. **`pynput` uses Win32 hooks, not X11.** On Linux, pynput was chosen
   specifically to bypass `mujoco.viewer`'s `key_callback`, whose hotkeys
   collided with letter keys (W/S/T toggled wireframe/shadow/transparency).
   On Windows that collision may not exist — but Win32 hooks are *global*,
   so keypresses may register even when the viewer window is not focused.
   **Verify live; do not assume either way.**
3. **Serial ports are `COM*`.** The existing telegrip setup uses **COM8**
   for the real arm. No `/dev/ttyACM*` paths here.
4. **`__pycache__` from the source machine held cpython-312 bytecode** and
   was deleted during setup. Harmless (3.11 ignores it) but confusing.

## Working conventions

- Never re-derive validated facts by guessing. The tables above came from
  running `validate_scenes.py` and reading the compiled model. If something
  looks wrong, re-run the validation rather than assuming the docs are
  stale.
- Create a NEW subdirectory per milestone (the `*_test/` pattern) rather
  than editing validated scenes in place.
- **Never edit `so101_assets\so101.xml` or the table geometry directly** —
  every scene includes or copies from them; a change ripples everywhere.
  Copy into a new milestone folder instead.
- Always read joint limits from the compiled model (`model.jnt_range`),
  never hardcode a "safe-looking" number.
- Prefer small, physically safe joint increments.

## Commands

```powershell
# this project's Python
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe

# headless check of all four scenes
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe D:\robotics\so101-digital-twin\scripts\validate_scenes.py

# M1 table viewer  (WORKS)
cd D:\robotics\so101-digital-twin\scripts\digital_twin_env
..\..\.venv\Scripts\python.exe run_table.py

# M2 falling cube  (WORKS)
cd D:\robotics\so101-digital-twin\scripts\digital_twin_env\spawn_cube_test
..\..\..\.venv\Scripts\python.exe spawn_cube.py

# M3 robot on table  (WORKS)
cd D:\robotics\so101-digital-twin\scripts\digital_twin_env\robot_on_table_test
..\..\..\.venv\Scripts\python.exe run_robot_on_table.py

# M5 keyboard control  (WORKS - keep the TERMINAL focused, not the viewer)
cd D:\robotics\so101-digital-twin\scripts\digital_twin_env\robot_keyboard_test
..\..\..\.venv\Scripts\python.exe keyboard_robot.py
```

Keyboard controls: `Q/A W/S E/D R/F T/G Y/H` for the 6 joints ±,
`Space` reset, `Esc` quit. Movement is 0.1° per simulation step — slow by
design. Targets are clamped to the model's real `jnt_range`.

## Installed

`mujoco 3.12.0` · `numpy 2.4.6` · `pynput 1.8.2` · `glfw 2.10.2` ·
`pyopengl 3.1.10` · `absl-py 2.5.0` · `etils 1.14.0` — ~50 MB total, no
compilation required (unlike project 1's pybullet).
