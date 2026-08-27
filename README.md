# so101-digital-twin

A MuJoCo digital twin of an SO-101 robot arm workspace, built for real-to-sim
joint synchronization.

An IKEA LINNMON/ADILS table modeled in MuJoCo, the SO-101 arm mounted on it,
and keyboard-driven joint control — with the mapping layer needed to drive a
real follower arm and its simulated twin from the same input.

Developed on Linux, validated on Windows.

![status](https://img.shields.io/badge/M1--M7-working-brightgreen)

---

## What works

| Milestone | Scene | Status |
|---|---|---|
| M1 | Table + floor | ✅ |
| M2 | Runtime cube spawning (no XML edits per spawn) | ✅ |
| M3 | SO-101 mounted on the table | ✅ |
| M4 | Joint range / axis sanity check | ✅ |
| M5 | Keyboard → simulated joints | ✅ |
| M6 | Keyboard → real follower arm | ✅ |
| M7 | Keyboard → real arm + sim simultaneously | ✅ |
| Task 1 | Real `elbow_flex` characterisation (latency, hysteresis, load) | ✅ |
| Task 2 | Replay real trajectories in sim, fit actuator gains | ✅ |

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
├── setup.sh / setup.ps1            M1-M5 setup (mujoco/numpy/pynput only)
├── setup_m6.ps1                    adds LeRobot to this venv, for M6/M7
├── so101_assets/                   SO-101 model + 20 STL meshes (19 MB)
│   ├── so101.xml                   fitted actuator gains (Task 2) — never
│   │                                hand-edit; re-run tune_actuator.py
│   ├── so101.xml.bak               pre-fit gains, kept for comparison
│   ├── assets/                     meshes
│   ├── PROVENANCE.md               upstream source + local modifications
│   └── LICENSE                     Apache 2.0
└── scripts/
    ├── validate_scenes.py          headless load-check, all scenes
    ├── check_pose.py               read-only: current pose vs safe envelope
    ├── goto_home.py                releases torque on exit — prefer M6 --recover
    ├── m6_keyboard_real.py         M6 — keyboard → real arm only
    ├── m7_mirror_sim.py            M7 — keyboard → real arm + mirrored sim
    ├── characterization/           Task 1/2 — real joint response, sim gain fit
    └── digital_twin_env/
        ├── table_scene.xml         M1
        ├── spawn_cube_test/        M2
        ├── robot_on_table_test/    M3
        ├── robot_keyboard_test/    M5
        └── real_sim_mapping_test/  M6-prep — real↔sim conversion + tests
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

## Working with real hardware (M6 / M7)

> **⚠️ Safety.** `m6_keyboard_real.py` and `m7_mirror_sim.py` gate every key
> behind a focus check — by default keys act only while a window whose title
> contains `VR-SO-101 - Antigravity ` is focused (override with
> `--require-focus`). **Click directly into that terminal window before
> pressing any control key** — a global keyboard hook can register the OS as
> not having given it real focus even if it looks focused, and every key
> (including `Esc`) will silently do nothing. `Ctrl+C` always works
> regardless of focus and disconnects cleanly.
>
> Keep a hand on the arm's power connector. Removing power is the fastest
> stop and needs no screen.
>
> **Torque drops to zero the instant either script exits or disconnects.**
> A loaded joint sags back toward its rest position (often outside the safe
> envelope) within seconds. This is expected, not a fault — see Recovery
> below.

M6/M7 additionally require [LeRobot](https://github.com/huggingface/lerobot)
for the `SOFollower` driver. Install it into *this* venv (never the
`so101-vr` project's venv) with:

```powershell
.\setup_m6.ps1
```

and a calibration for *your specific arm* (encoder zero is per-unit, set at
assembly — a previous arm's calibration file does not transfer):

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=<PORT> --robot.id=<NAME>
```

Ports are `COM*` on Windows, `/dev/ttyACM*` on Linux. During calibration,
sweep **every** joint fully to both physical extremes — including squeezing
the gripper fully shut and opening it fully wide. A joint whose sweep didn't
reach a true extreme will read a wrong percentage at that end for the rest
of the session (this happened with the gripper and needed a recalibration
to fix).

### M6 — keyboard → real arm only

```bash
python scripts/m6_keyboard_real.py --joints elbow_flex        # start with one joint
python scripts/m6_keyboard_real.py --joints all                # once confident
python scripts/m6_keyboard_real.py --joints all --recover      # fold out-of-range joints in first, torque never released
```

Controls: `Q/A` shoulder_pan · `W/S` shoulder_lift · `E/D` or `Up/Down`
elbow_flex · `R/F` wrist_flex · `T/G` wrist_roll · `Y/H` gripper · `Space`
return to start pose · `Esc` quit.

Four safety layers, all always on: the focus gate, LeRobot's
`max_relative_target` per-command clamp, a `±SAFE_LIMIT` (50 units) envelope
well inside the calibrated range, and one joint live at a time by default.

**`--recover`** walks every out-of-range joint back into the envelope
without ever releasing torque — use this instead of `goto_home.py`, which
releases torque on exit and lets a loaded joint sag straight back to its
hard stop.

**`--debug-joint <name>`** logs every control tick (target before/after the
leash clamp, measured position, whether the clamp fired) for one joint to
`debug_<name>.csv` — useful if a joint seems unresponsive and you need to
see whether the target is moving or the arm is.

### M7 — keyboard → real arm + mirrored sim, together

The sim mirrors the real arm's **measured** position, not the keyboard
target — so any gap you see between them is genuine tracking error (the
joint lag, backlash, and gravity droop Task 1/2 characterised), not hidden
by the display.

```bash
# step 1: prove the wiring with no hardware
python scripts/m7_mirror_sim.py --source sim --joints all

# step 2: the real thing
python scripts/m7_mirror_sim.py --source real --joints all

# with a recording
python scripts/m7_mirror_sim.py --source real --joints all --record recordings/session.csv
```

Same controls and safety envelope as M6. `--record <path>.csv` logs every
tick's target / real (measured) / sim (mirrored, degrees) for each live
joint — a plain CSV, not a LeRobot `Dataset`; see
[Real ↔ sim joint mapping](#real--sim-joint-mapping) for why real and sim
values are not expected to match exactly.

**Recommended routine:** pose the arm roughly centered by hand before
launching (torque is off between sessions) — this usually keeps every joint
inside the envelope so `--recover` isn't needed at all.

### Recovery pattern between sessions

Torque releases on exit, so a loaded joint (typically `shoulder_lift`,
`elbow_flex`, `wrist_flex`) sags out of the safe envelope within seconds of
any script quitting. Read-only check, safe any time:

```bash
python scripts/check_pose.py
```

If it reports joints outside the envelope, **ignore its own suggestion to
run `goto_home.py`** (that releases torque on exit and the joint sags right
back) — use M6's `--recover` instead, then go straight into whatever you
actually meant to run, minimizing the limp gap in between.

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

**`ModuleNotFoundError: No module named 'mujoco'`** (or `'lerobot'`)
You are running the system Python rather than the venv. Activate it
(`.venv\Scripts\Activate.ps1` / `source .venv/bin/activate`) or use the full
interpreter path shown in the table above.

**A joint reports `*** OUTSIDE +/-50 ***` at startup**
Expected after any session ends — torque releases and a loaded joint sags.
Run `python scripts/m6_keyboard_real.py --joints <name> --recover`, not
`goto_home.py` (see [Recovery pattern](#recovery-pattern-between-sessions)).

**A key (including `Esc`) does nothing at all with M6/M7 running**
The focus gate isn't seeing the terminal as focused, even if it looks
focused on screen — click directly into the terminal window. `Ctrl+C`
always works regardless and disconnects cleanly.

**One direction key (e.g. `S`) seems dead on a specific joint**
Check `check_pose.py` first — if that joint is pinned against the `±50`
safe envelope, only the key pointing back *into* range will do anything;
the other clamps to the same edge value every tick and looks broken.
This is the most common cause and is not a bug.

If the joint is well inside the envelope and one key still seems dead, it
may be real backlash/static friction rather than a software issue —
`shoulder_lift` in particular was measured (`scripts/characterization/`)
settling ~10x worse than `elbow_flex` with ~22 ticks of backlash, and can
stick for 10+ seconds under a held key before releasing. Use
`--debug-joint <name>` to see whether the *target* is moving (software
registering the key) while the *measured* position is not (the arm itself
sticking) — that distinguishes the two.

**Real and sim values don't match exactly in M7**
Expected — see [Real ↔ sim joint mapping](#real--sim-joint-mapping). The
mapping is a linear rescale between two different range spans, so it
preserves direction and relative position, not the exact physical angle.

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
