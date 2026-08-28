# CLAUDE.md — SO-101 Real-to-Sim Digital Twin (Windows)

Guidance for Claude Code on THIS machine (Windows 10, i3-7020U 2C/4T,
11.9GB RAM, Intel HD 620 integrated graphics, no CUDA). Read this before
touching anything — it explains what already exists, what's validated, and
what's actually left to do.

## What this project is

A real-to-sim digital twin of an SO-101 robot arm workspace: an IKEA
LINNMON/ADILS table modeled in MuJoCo, the SO-101 arm placed on it, and a
real physical follower arm kept synchronized with the simulated one —
currently via keyboard input (standing in for a leader arm we don't have
yet), later via VR or an actual leader arm.

This machine's job: **run the real SO-101 follower arm**, characterize its
real behaviour, fit the simulation to match it, and drive both together.
M1–M7 plus Task 1/2 characterization are all DONE on this machine — see
Status below. The heavy MuJoCo scene/asset development originally happened
on a separate Linux server; that groundwork is done and validated, this
machine has since built substantially past it (M6, M7, Task 1/2,
`twin.py`, and everything in `docs/` are Windows-native work).

### Read these before deep-diving into how the mapping/pipeline works

- `docs/PROJECT_STATUS.md` — current milestone + phase status, what's left
- `docs/MOTOR_CHECK_AND_MAPPING_SIMPLE.md` — per-motor table (all 6 IDs,
  ranges, which have been behavior-tested), simple explanation first
- `docs/REAL_SIM_MAPPING_DEEP_DIVE.md` — full technical writeup of the
  real↔sim mapping, characterization, and gain-fitting pipeline
- `docs/ROADMAP_TELEOP_TO_DATASET.md` — the larger goal (leader-arm teleop,
  object/scene parity, dataset recording + validation, then a MuJoCo→Unreal
  rendering bridge, synthetic data, mixed real/sim/Unreal-ratio training,
  and cross-domain inference) and what phase each part is in

## A separate, unrelated project lives on this PC — do not conflate it

`D:\robotics\so101-vr\` is an earlier, separate project with its own venv,
its own `lerobot`/`torch`/`pybullet` install, and its own hardware session
history. It is out of scope here. **Never modify, reuse, or install into
that venv, and do not pull its history or status into this file.** This
project (`D:\robotics\so101-digital-twin\`) has its own venv and, as of
M6/M7, does use `lerobot` directly (see Installed below) — that overlap in
dependency is coincidental, not a link between the two projects.

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
| M1 | LINNMON/ADILS table (`table_scene.xml`) | DONE, validated on Windows |
| M2 | Runtime cube spawning (`spawn_cube_test/`) | DONE, validated on Windows. Sim-only — no real-object connection (see `docs/ROADMAP_TELEOP_TO_DATASET.md` Phase B) |
| M3 | SO-101 + table (`robot_on_table_test/`) | DONE, validated on Windows |
| M4 | Joint range/axis sanity check | DONE (part of M3) |
| M5 | Keyboard → MuJoCo (`robot_keyboard_test/`) | DONE, validated on Windows (see hotkey note) |
| M6-prep | Real↔sim mapping math (`real_sim_mapping_test/real_sim_joint_mapping.py`) | DONE, unit tested — `validate_real_sim_mapping.py` |
| M6 | Keyboard → real follower arm (`m6_keyboard_real.py`) | DONE, validated on hardware |
| M7 | Keyboard → real follower + MuJoCo together (`m7_mirror_sim.py`) | DONE, validated on hardware |
| Task 1 | Real joint characterisation | DONE for `elbow_flex` + `shoulder_lift` (2026-08-26) — `scripts/characterization/RESULTS.md`. **NOT done** for `shoulder_pan`, `wrist_flex`, `wrist_roll`, `gripper` |
| Task 2 | Replay real trajectories in sim, fit the model | DONE for `elbow_flex` (2026-08-26) — sim RMSE 1.254° → 0.434°, gains applied to the shared `sts3215` class |
| `twin.py` | Unified CLI dispatcher (subcommands + interactive menu) for every script below | DONE — `python scripts\twin.py` |

Full phase-by-phase breakdown (including everything past M7 — object/scene
parity, leader-arm teleop, dataset recording/validation) is tracked in
`docs/PROJECT_STATUS.md`, not duplicated here.

### Model gains are now FITTED, not derived (2026-08-26)

`so101_assets/so101.xml` class `sts3215` shipped with
`kp=998.22 kv=2.731`, calculated from a formula assuming a servo
proportional gain of 16 and — by its own comment — "not a 1-to-1 mapping" of
the LeRobot gains. It had never been checked against hardware, and it is
badly underdamped: on a 20-unit elbow step the sim overshot to 23.8° against
a 19.6° target and peaked at 243 °/s where the real joint manages 150 °/s.

Now **`kp=400 kv=25`**, fitted against real recorded step responses
(`scripts/characterization/tune_actuator.py`). 68% better on the fit set,
62% better on held-out runs. Previous file kept at `so101.xml.bak`.

Fitted against `elbow_flex` only and applied to the whole `sts3215` class,
since every joint uses the same servo — the shoulder carries far more
inertia and may want its own fit. Re-run `tune_actuator.py` per joint as
each is characterised.

### shoulder_lift: do NOT re-fit gains for it (2026-08-26)

`shoulder_lift` settles ~10x worse than `elbow_flex` (0.65–1.5° vs
0.06–0.26°) and draws 2.4x the holding current. A gain sweep appears to
"improve" it 18% at kp=60 kv=6 — **do not apply that.** Held-out error got
*worse*, and the angle-dependent error spread is unchanged (1.205° → 1.194°):
soft gains only slide the curve to straddle zero.

Root cause: the real servo's settled position varies with gravity load
(15.6 ticks off at 0°, 2.1 ticks at 22.7°), while the sim at kp=400 varies
only 0.018° across the same range — 66x less. A MuJoCo `position` actuator
has steady-state droop of exactly `torque / kp`, so matching the real
compliance would need a kp ~66x lower and would wreck the transient response
kp=400 gets right. **No single linear gain satisfies both**, because the
STS3215's internal loop is not a PD law (deadband, no integral term).

**RESOLVED (staircase probe, same day):** the settled error is **backlash
1.955° plus gravity droop 1.009°**, measured by approaching 7 rungs across
±42.5° from both directions. Backlash = 22.2 encoder ticks, consistent across
rungs. Gravity fits `sin(angle)` at R² 0.880.

Two wrong readings preceded this and are worth not repeating: "gravity
compliance" (a confound — command 0 is the only one ever approached from
above in the Task 1 trajectories) and "1.337° pure backlash" (the same
confound seen from the other side). Neither could be settled by reanalysis;
it needed a different trajectory.

**`elbow_flex` measured too (same day):** backlash **1.196°** (13.6 ticks),
gravity droop **1.379°** (sin fit R² 0.975). Its earlier 0.166° figure was a
confounded lower bound, 10x too small — worth remembering before trusting any
number extracted from the Task 1 trajectories about direction.

Both joints therefore carry 1–2° of slop and ~1–1.4° of droop, against a
Task 2 sim-to-real RMSE of 0.434°. Neither is negligible.

**Open:** the elbow's gap is NOT constant (sd 0.510 on a 1.196 mean) — it is
smallest near 0° and grows toward both extremes, symmetric in |angle|. Not
travel saturation (558 ticks headroom both ends, even tick spacing). Cause
unknown, so 1.196° is an average, not a constant. Model the shoulder first.

Full results: `scripts/characterization/BACKLASH-RESULTS.md`.

**`servo_model.py` implements both effects** as a drive layer (MuJoCo has no
hinge-backlash primitive, and these are properties of THIS arm's servos, not
of the shared kinematic model). Validated against the recordings: settled
error 1.337° → 0.628° (shoulder), 0.802° → 0.409° (elbow).

**Do NOT stack servo_model on the Task 2 kp/kv fit.** Those gains were fitted
against step runs visiting only two commands, so ~90% of the backlash is
already inside them (measured 1.196° vs 0.118° residual gap). Stacking makes
settled error 245% worse. The model is for NEW command sequences, for
compensating commands, and for trajectories the fit never saw — and it
describes where the joint comes to REST, so never apply it mid-transient.

Note `replay_in_sim.py` now also picks up the staircase runs, which sweep
±49° and were never fitted: they score 0.82–0.84° consistently, so the
headline no_load RMSE moved 0.434 → 0.638°. That is a harder test set, not a
regression.

**Loaded `shoulder_lift` (arm extended horizontal):** backlash **grew** under
load — 2.812° vs 2.578° at the 0° rung, the only command both runs share
(+9%). Whole-sweep means suggest +27% but the two runs sampled different
rungs, so +9% is the defensible figure. Backlash growing at all means part of
it is **compliance** (flex under torque), not pure gear slop — the two are not
separable from position data.

Loaded **gravity was NOT measured**: the sweep had to be narrowed to ±34° for
safety, which halved the sin variation (0.771 vs 1.351) and left the fit at
R² 0.010. The analyzer correctly refused to report a number. Measuring it
needs a wide sweep from a pose where that is mechanically safe.

**`elbow_flex` loaded is unsafe from the extended pose** — the elbow sits at
+98 units there (holding the forearm horizontal), so a probe sweep to −50
would drop the forearm 145° onto the table. It needs a forearm-down pose.

Details: `scripts/characterization/LOADED-RESULTS.md`.

Full analysis, including the three hypotheses tested and rejected (gains,
stale calibration, mass error): `scripts/characterization/FINDINGS-shoulder_lift.md`.

**When fitting anything to gravity, check the trig basis.** Gravity torque
goes as sine of the angle from vertical. Assuming cosine (i.e. that the
joint's zero is horizontal) fitted this arm at R² 0.056 while still reporting
a confident 1.164° amplitude; sine fits at 0.880. `analyze_backlash.py` now
fits both and warns when neither works.

`tune_actuator.py` now warns instead of asserting success when held-out
error fails to improve, and `check_calibration_drift()` catches CSVs recorded
against a superseded calibration.

**Measured facts about the real arm that the model must respect:**
- Command-to-motion latency is **65 ms**, fixed, load-independent. Model it
  explicitly; it does not emerge from the dynamics.
- Load hits **accuracy, not speed** — steady-state error +269% under load,
  velocity −1%. A residual −0.6° bias in the loaded runs is gravity droop
  the sim still does not reproduce; that is the next modelling gap.
- Backlash is **unmeasured, not absent**. Every experiment steps up from 0
  and back down to 0, so no command is settled from both directions. Do not
  cite "no hysteresis" as a result. Repeatability sd 0.088° bounds it.

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

### 🔴 Safety requirement for M6/M7 (real hardware) — RESOLVED 2026-08-26

`pynput_keyboard.Listener` is created with no `suppress` argument, so the
hook is **global** — keys register regardless of focus. Harmless in
simulation; on real hardware, any keystroke in any window (browser,
terminal, editor) could move real servos without a gate.

`KeyboardInput(require_focus="<title substring>")` gates every key on that
window being focused, via `GetForegroundWindow()`. The gate is checked on
press and again in `snapshot_held()` at the point of use, and any held key
is dropped the moment focus is lost — so a joint cannot keep moving after
an alt-tab. Esc stays ungated so quitting always works; space (reset to
rest pose) is gated because it commands a large motion.

Simulation keeps the ungated default, so M5 behaviour is unchanged and
`validate_keyboard_robot.py` still passes. **Anything that drives hardware
must pass `require_focus`** — constructing it on a platform without the
focus check raises rather than silently running a global hook ungated.

`m6_keyboard_real.py` auto-detects the focused window title at startup
(`detect_focus_title()`) rather than requiring it to be typed in. `m7_mirror_sim.py`
originally shipped with a stale hardcoded default title left over from a
different project's window name, which silently ate every keystroke — fixed
2026-08-28 to use the same auto-detect approach as M6.

## The real↔sim joint mapping

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

### ⚠️ Do NOT compare calibrated span against sim span (2026-08-26)

A calibrated range and a kinematic design range are different quantities and
there is no reason for them to match. Comparing them produced a bogus "46°
shoulder_pan mismatch" that triggered a full recalibration to chase a
non-problem.

Evidence they are not comparable — `shoulder_pan` measured on two arms:

| Arm | shoulder_pan calibrated |
|---|---:|
| `left_follower` (VR arm) | 198.1° |
| `twin_follower` (this arm) | 175.2° |
| `so101.xml` kinematic | 220.0° |

Two physically identical arms differ by 23°, and both fall short of the
model. `twin_follower` was swept twice, months apart, landing within 1.4°
(173.8° then 175.2°) — that is a **real hard stop on this unit**, most
likely the servo horn splined on at a different angle during assembly.

**Never "fix" `so101.xml` joint ranges to match a calibration.** The model is
shared by both arms; hardcoding one arm's limits breaks the other. Per-arm
differences belong in the calibration file, which is exactly what the
mapping layer rescales against.

The only meaningful sim-to-real pose check is **visual, with hardware
connected** — command a known real pose and look at whether the sim matches.

Recalibration on 2026-08-26 did find one genuine gap: `shoulder_lift`
211.2° → 226.6° (+15.4°, the earlier sweep had not reached the stops).
Other joints moved ≤1.8°. `wrist_roll` reads 0..4095 — the full encoder
range, i.e. continuous rotation with no hard stop, so its 359.9° is not a
measured limit at all.

**Task 1 data predates this recalibration** (elbow span 2228 → 2240 ticks,
+0.54%). The CSVs store degrees already converted, and `replay_in_sim.py`
reads those directly rather than re-deriving from the calibration file, so
the fit and all five Task 1 numbers are unaffected. The calibration file is
consulted only for a printed span-ratio diagnostic.

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

**Preferred: the unified dispatcher, `scripts\twin.py`.** Covers every
command below plus calibration, pose checks, recovery, and the full
characterization pipeline — run with no arguments for an interactive menu
(pick a command by number, answer a few flag prompts, confirm before it
runs), or `--help` for the full subcommand list.

```powershell
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe D:\robotics\so101-digital-twin\scripts\twin.py
```

Individual scripts (what `twin.py` calls under the hood), for reference:

```powershell
# this project's Python
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe

# headless check of all four scenes
..\.venv\Scripts\python.exe scripts\validate_scenes.py

# M1 table viewer
cd scripts\digital_twin_env & ..\..\.venv\Scripts\python.exe run_table.py

# M2 falling cube (sim-only, no real-object connection)
cd scripts\digital_twin_env\spawn_cube_test & ..\..\..\.venv\Scripts\python.exe spawn_cube.py

# M3 robot on table
cd scripts\digital_twin_env\robot_on_table_test & ..\..\..\.venv\Scripts\python.exe run_robot_on_table.py

# M5 keyboard -> sim only (keep the TERMINAL focused, not the viewer)
cd scripts\digital_twin_env\robot_keyboard_test & ..\..\..\.venv\Scripts\python.exe keyboard_robot.py

# M6 keyboard -> real arm only
..\.venv\Scripts\python.exe scripts\m6_keyboard_real.py --joints elbow_flex

# M7 keyboard -> real arm + mirrored sim together
..\.venv\Scripts\python.exe scripts\m7_mirror_sim.py --source real --joints elbow_flex
```

Keyboard controls (M5/M6/M7 all share this scheme): `Q/A W/S E/D R/F T/G
Y/H` for the 6 joints ±, `Space` reset, `Esc` quit. M5 (sim-only) moves
0.1° per simulation step. M6/M7 (real hardware) move ~5 units/s while a key
is held, clamped to `±SAFE_LIMIT` (50 normalized units) and to LeRobot's
`max_relative_target` per-command clamp — see M6/M7's own docstrings for
the full safety-layer breakdown, not repeated here.

## Real hardware safety, in one place

M6 (`m6_keyboard_real.py`) and M7 (`m7_mirror_sim.py`) both require, always
on: a focus gate (auto-detected window title, keys act only while it's
focused), LeRobot's per-command `max_relative_target` clamp, a `±50`
normalized-unit safe envelope well inside the calibrated range, and one
joint live at a time by default (`--joints all` widens it). The arm goes
LIMP the instant either script exits — support it. Full detail, recovery
patterns, and troubleshooting: `README.md` → "Working with real hardware".

## Installed

`mujoco 3.12.0` · `numpy 2.4.6` · `pynput 1.8.2` · `glfw 2.10.2` ·
`pyopengl 3.1.10` · `absl-py 2.5.0` · `etils 1.14.0` · `lerobot 0.4.4`
(added for M6/M7 — `SOFollower` hardware driver, `lerobot-calibrate`, and
the other `lerobot-*` console scripts) — installed via `setup.ps1` (M1–M5)
then `setup_m6.ps1` (adds `lerobot` on top).
