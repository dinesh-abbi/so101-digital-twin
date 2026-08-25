# so101-digital-twin

A MuJoCo digital twin of an SO-101 robot arm workspace, built for real-to-sim
joint synchronization.

An IKEA LINNMON/ADILS table modeled in MuJoCo, the SO-101 arm mounted on it,
and keyboard-driven joint control — with the mapping layer needed to drive a
real follower arm and its simulated twin from the same input.

Developed on Linux, validated on Windows.

![status](https://img.shields.io/badge/M1--M5-working-brightgreen)
![status](https://img.shields.io/badge/M6-in%20progress-yellow)

---

## What works

| Milestone | Scene | Status |
|---|---|---|
| M1 | Table + floor | ✅ |
| M2 | Runtime cube spawning (no XML edits per spawn) | ✅ |
| M3 | SO-101 mounted on the table | ✅ |
| M4 | Joint range / axis sanity check | ✅ |
| M5 | Keyboard → simulated joints | ✅ |
| M6 | Keyboard → real follower arm | 🚧 |
| M7 | Keyboard → real arm + sim simultaneously | ⬜ |

---

## Quick start

### Linux / macOS

```bash
git clone https://github.com/dinesh-abbi/so101-digital-twin.git
cd so101-digital-twin
chmod +x setup.sh
./setup.sh
```

### Windows

```powershell
git clone https://github.com/dinesh-abbi/so101-digital-twin.git
cd so101-digital-twin
.\setup.ps1
```

If PowerShell refuses to run the script (execution policy):

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1
```

### What setup does

1. Finds a Python **3.9–3.12** interpreter (rejecting 3.13+, which has no
   MuJoCo wheel and would fail a source build)
2. Verifies the repository layout — specifically that `so101_assets/` and
   `scripts/` are siblings, and that the STL meshes are present
3. Creates `.venv/` **inside the repo** and installs `mujoco`, `numpy`,
   `pynput` there
4. Loads all four scenes headlessly and prints body/joint/geom counts
5. Probes for an OpenGL 3.3 context and reports your renderer

It does not modify your system Python, PATH, or any other virtual
environment. Re-running it is safe — an existing `.venv/` is reused.

---

## Running the scenes

Work up in order. Each adds one thing, so a failure is easy to isolate.

<table>
<tr><th></th><th>Linux / macOS</th><th>Windows</th></tr>
<tr><td><b>Validate all<br>(headless)</b></td>
<td><code>.venv/bin/python scripts/validate_scenes.py</code></td>
<td><code>.venv\Scripts\python.exe scripts\validate_scenes.py</code></td></tr>
<tr><td><b>M1 table</b></td>
<td><code>.venv/bin/python scripts/digital_twin_env/run_table.py</code></td>
<td><code>.venv\Scripts\python.exe scripts\digital_twin_env\run_table.py</code></td></tr>
<tr><td><b>M2 cube</b></td>
<td colspan="2"><code>cd scripts/digital_twin_env/spawn_cube_test</code> then run <code>spawn_cube.py</code></td></tr>
<tr><td><b>M3 robot</b></td>
<td colspan="2"><code>cd scripts/digital_twin_env/robot_on_table_test</code> then run <code>run_robot_on_table.py</code></td></tr>
<tr><td><b>M5 keyboard</b></td>
<td colspan="2"><code>cd scripts/digital_twin_env/robot_keyboard_test</code> then run <code>keyboard_robot.py</code></td></tr>
</table>

From a `*_test/` subdirectory the interpreter is three levels up:
`../../../.venv/bin/python` (or `..\..\..\.venv\Scripts\python.exe`).

**Always run `validate_scenes.py` first.** A headless load failure means an
asset-path problem; a failure only in the viewer means a graphics problem.
Separating the two saves a lot of guessing.

### Expected validation output

```
--- SO-101 + table ---
    robot_on_table_scene.xml
    bodies   10   joints    6   geoms   54   meshes   18
    actuators  6   qpos   6   dof   6   timestep 0.005
    100 steps OK, state finite

4/4 scenes loaded
```

`meshes 18` confirms the STL files resolved through `../../../so101_assets/`.
If that reads `0` on a robot scene, the layout is wrong — see below.

### The `*_container.sh` files

Ignore them. They are Docker wrappers from the original Linux development
environment and reference paths that do not exist in this repository. Run the
`.py` entry points directly.

---

## Requirements

- **Python 3.9–3.12** (3.11 recommended; MuJoCo has no 3.13/3.14 wheels yet)
- **OpenGL 3.3+** for the interactive viewer — integrated graphics is fine,
  Intel HD 620 works
- ~200 MB disk (19 MB assets, ~50 MB packages, rest is the venv)
- **No GPU or CUDA required.** MuJoCo's physics is CPU-only.

Dependencies: `mujoco`, `numpy`, `pynput`. That's all — no torch, no
simulation framework.

---

## Layout

```
so101-digital-twin/
├── setup.sh / setup.ps1            one-command setup
├── so101_assets/                   SO-101 model + 20 STL meshes (19 MB)
│   ├── so101.xml                   never edit — every scene includes it
│   ├── assets/                     meshes
│   ├── PROVENANCE.md               upstream source + local modifications
│   └── LICENSE                     Apache 2.0
└── scripts/
    ├── validate_scenes.py          headless load-check, all scenes
    └── digital_twin_env/
        ├── table_scene.xml         M1
        ├── spawn_cube_test/        M2
        ├── robot_on_table_test/    M3
        ├── robot_keyboard_test/    M5
        └── real_sim_mapping_test/  M6 — real↔sim conversion
```

### The `scripts/` level is load-bearing

The robot scenes reference assets three levels up:

```xml
<compiler meshdir="../../../so101_assets/assets"/>
<include file="../../../so101_assets/so101.xml"/>
```

`so101_assets/` and `scripts/` **must stay siblings**. Flattening the tree
breaks both robot scenes with a missing-file error.

---

## Controls

Run `scripts/digital_twin_env/robot_keyboard_test/keyboard_robot.py`:

| Key | Joint |
|:--:|---|
| `Q` / `A` | shoulder_pan ± |
| `W` / `S` | shoulder_lift ± |
| `E` / `D` | elbow_flex ± |
| `R` / `F` | wrist_flex ± |
| `T` / `G` | wrist_roll ± |
| `Y` / `H` | gripper ± |
| `Space` | reset to rest pose |
| `Esc` | quit |

Movement is 0.1° per simulation step — deliberately slow. Targets are clamped
to each joint's real `jnt_range`, read from the compiled model.

> **On Windows, keep the TERMINAL focused, not the viewer.** `pynput` uses a
> global keyboard hook, and MuJoCo's viewer reserves `W`/`S`/`T` and the
> number row for its own toggles. With the viewer focused, both fire — joints
> move *and* the render toggles wireframe/shadows/camera. With the terminal
> focused, control is clean. This does not occur the same way on Linux/X11.

---

## Real ↔ sim joint mapping

Two coordinate systems that do **not** naturally agree:

- **Real servo (Feetech STS3215):** raw 0–4095 encoder ticks with an
  arbitrary zero. LeRobot's calibration records each joint's observed
  min/max and normalizes to `-100..100` for the arm joints and `0..100` for
  the gripper. `SO101Follower.get_observation()` returns those normalized
  values — not ticks, not degrees.
- **Sim (`so101.xml`):** radians from the kinematic model, unrelated to any
  servo's calibrated range.

`real_sim_mapping_test/` performs a per-joint linear rescale between them.

### What this guarantees, and what it does not

The rescale gives the same direction and the same *relative* position within
each range. It does **not** guarantee that real 0% and sim 0 rad are the same
physical pose — that depends on where the real calibration's midpoint falls.

The ranges genuinely differ. Measured on one arm:

| Joint | Real calibrated | Sim kinematic |
|---|---:|---:|
| wrist_roll | 359.9° | 314.4° |
| gripper | 143.0° | 110.0° |

**Every physical arm needs its own calibration**, and the mapping reads it at
runtime — nothing is hardcoded. Two arms of the same model will produce
different calibration files, and that is expected.

**Verify by eye before trusting it.** Command a few known real positions and
watch whether the sim arm visually matches. If you need true physical
correspondence rather than proportional agreement, you also need a per-joint
offset established by posing the arm at a known configuration and reading
both sides. The linear rescale alone will not give you that.

---

## Sim joint ranges

Read from the compiled model, not hardcoded:

| Joint | radians | degrees | span |
|---|---|---|---:|
| shoulder_pan | -1.91986 .. 1.91986 | -110.0 .. 110.0 | 220.0° |
| shoulder_lift | -1.74533 .. 1.74533 | -100.0 .. 100.0 | 200.0° |
| elbow_flex | -1.69000 .. 1.69000 | -96.8 .. 96.8 | 193.7° |
| wrist_flex | -1.65806 .. 1.65806 | -95.0 .. 95.0 | 190.0° |
| wrist_roll | -2.74385 .. 2.74385 | -157.2 .. 157.2 | 314.4° |
| gripper | -0.17453 .. 1.74533 | -10.0 .. 100.0 | 110.0° |

---

## Working with real hardware (M6)

> **⚠️ Safety.** `keyboard_robot.py` uses a *global* keyboard hook — keys
> register regardless of which window has focus. Harmless in simulation.
> **With a real arm connected, a keystroke in any window could move real
> servos.** Add a focus guard before connecting hardware.
>
> Keep a hand on the arm's power connector. Removing power is the fastest
> stop and needs no screen.

M6 additionally requires [LeRobot](https://github.com/huggingface/lerobot)
for its `SO101Follower` driver, and a calibration for *your* arm:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=<PORT> --robot.id=<NAME>
```

Ports are `COM*` on Windows, `/dev/ttyACM*` on Linux.

---

## Troubleshooting

**`No Python 3.9-3.12 found`**
MuJoCo publishes no wheels for 3.13/3.14; pip would attempt a source build and
fail. Install 3.11 from [python.org](https://www.python.org/downloads/release/python-3119/).
On Windows tick *py launcher*; on Debian/Ubuntu also install
`python3.11-venv`.

**A robot scene reports `meshes 0`, or `Error: file not found`**
The layout is flattened. `so101_assets/` and `scripts/` must be siblings,
because the scenes reference `../../../so101_assets/`. Do not move
`digital_twin_env/` out of `scripts/`.

**`no OpenGL 3.3 context` / the viewer window never appears**
Update your graphics driver. Integrated graphics is sufficient — Intel HD 620
reports exactly 3.3.0 and works — but old drivers may expose less. Headless
scripts still run regardless.

On a headless Linux box, the viewer needs a display:

```bash
xvfb-run -s "-screen 0 1280x1024x24" .venv/bin/python scripts/digital_twin_env/run_table.py
```

**Keyboard keys move joints *and* change the render**
You have the viewer focused. Focus the terminal instead — see the note under
[Controls](#controls). This affects Windows; Linux/X11 behaves differently.

**Keys do nothing at all on Linux**
`pynput` needs an accessible input backend. Under Wayland it may not receive
events; try an X11 session. On some systems the user must be in the `input`
group.

**`ModuleNotFoundError: No module named 'mujoco'`**
You are running the system Python rather than the venv. Use the full
interpreter path shown in the table above.

---

## Contributing

The conventions below come from the original development and exist for
reasons that were learned the hard way:

- **Never edit `so101_assets/so101.xml` or the table geometry in place.**
  Every scene includes or copies from them, so a change ripples everywhere.
  Copy into a new milestone folder instead.
- **Create a new `*_test/` directory per milestone** rather than modifying a
  validated scene.
- **Read joint limits from the compiled model** (`model.jnt_range`) — never
  hardcode a "safe-looking" number.
- **Re-run the relevant `validate_*.py` rather than trusting the docs** if
  something looks inconsistent.
- **Check for hotkey collisions** before rebinding keyboard controls. MuJoCo's
  viewer reserves a large set of letters and the number row.

---

## Attribution

`so101_assets/` is vendored from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
(`robotstudio_so101`, pinned commit `4c358ef`), originally the SO-101 arm
from The Robot Studio, developed by I2RT Robotics. **Apache 2.0** — see
`so101_assets/LICENSE`.

Local modifications are documented in `so101_assets/PROVENANCE.md`: a camera
`fovy` fix and a `gripperframe` quaternion correction. **Do not replace this
directory with a fresh upstream copy** — the fixes would be lost.

---

## License

Apache 2.0.
