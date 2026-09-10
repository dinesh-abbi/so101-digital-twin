# Leader teleop → dataset: what happened, what's unresolved, what's next

Running log of the leader-arm teleop work (2026-09-08 → 2026-09-10), kept
as **problem → what was tried → outcome**, including the attempts that
failed and were reverted. The failures are the point: three of them were
re-attempted more than once because nothing recorded why they were wrong
the first time.

Companion to `LEADER_TELEOP_RUNBOOK.md` (the operational procedure) and
`PROJECT_STATUS.md` (milestone status). This file is the narrative of the
debugging, not a substitute for either.

---

## Where things stand right now

**Working, validated on hardware:**

| Capability | Script | Evidence |
|---|---|---|
| Leader → real follower → sim, live | `m_lerobot_teleop_sim.py` | 6+ recorded sessions |
| Per-tick CSV logging | `--record PATH` | 31 columns, all 6 joints |
| Accuracy analysis | `analyze_teleop_accuracy.py` | RMSE + lag per joint |
| Accuracy plots | `plot_teleop_accuracy.py` | real vs sim + error trace |
| Replay to sim only | `replay_teleop_sim.py` | no hardware touched |
| Replay to real + sim | `replay_teleop_real.py` | start-pose gate + pre-flight |
| Workspace analysis | `analyze_workspace.py` | FK a recording, find grasp points |
| Cube scene + cameras | `robot_cube_test/` | 16/16 headless checks |

**Not working yet:** `robot_cube_test/grasp_cube.py` reaches the cube but
does not lift it (wrist orientation — see the 2026-09-10 session notes).
`replay_teleop_real.py` still has no `--source sim`, so **MuJoCo has never
driven the hardware.**

**Best measured accuracy** (`teleop_log_pick2.csv`, 60 fps, 33.5 s):

| Joint | twin RMSE | median | Verdict |
|---|---:|---:|---|
| shoulder_pan | 0.29° | 0.05° | good |
| wrist_roll | 0.16° | 0.00° | good |
| gripper | 0.60° | 0.00° | good |
| shoulder_lift | 3.18° | 0.27° | good except folds |
| elbow_flex | 3.99° | 1.06° | good except folds |
| wrist_flex | 7.85° | 1.92° | good except folds |
| **overall** | **3.83°** | | |

Project reference for comparison: held-out fit RMSE 0.6–0.8°
(`scripts/characterization/RESULTS.md`).

**Read the median, not just the RMSE.** The three larger numbers are
dominated by short windows where the arm folds tightly enough to
self-collide in sim but not in reality. During actual task motion every
joint tracks within ~1–2° — visible directly in
`teleop_log_pick2_accuracy.png`, where the error trace sits on the 1° line
for the middle of the session and spikes only at the folded ends.

---

## Solved problems

### 1. Leader arm reporting frozen rail values (2026-09-09)

**Symptom.** Teleop ran, but `shoulder_pan`/`wrist_flex`/`wrist_roll`
reported identical values (`-100.0` / `+100.0` / `-42.5`) for an entire
session. The follower drove hard toward those frozen commands, walked
`elbow_flex` into a mechanical stop, and threw `RuntimeError: Failed to
write 'Torque_Enable' ... Overload error!` on disconnect.

**Wrong turns, in order.** Blamed a stale COM port (ports *had* moved —
COM15→COM8, COM10→COM14 — but that was a separate issue); blamed a flaky
serial handle; misread the CSV as "every joint frozen" when only three
were.

**Root cause.** The calibration file no longer matched the arm. Every
joint except `wrist_roll` read *outside* its own recorded `range_min..max`
— `shoulder_pan` raw 7 vs min 1072, `elbow_flex` raw 3761 vs max 3136.
LeRobot clamps out-of-range raw ticks to the nearest rail, which is
exactly the frozen ±100 values.

**Fix.** Recalibrate both arms. Built `leader_raw_probe.py` and
`follower_raw_probe.py` (read-only, no motion) to compare live raw ticks
against the calibration file — this check is what found it and is worth
running whenever an arm behaves oddly.

**Lesson.** A joint pinned at exactly ±100 while others move normally is a
calibration symptom, not a wiring one. Check raw ticks against
`range_min`/`range_max` before touching anything else.

> Note: the `zero_in_range` heuristic (does `(min+max)/2 + homing_offset`
> land inside the range?) fails legitimately on `elbow_flex`, `wrist_flex`
> and `gripper` on these arms. It was treated as a hard failure and
> triggered two unnecessary recalibrations. **The test that matters is
> whether the current resting pose reads in-range.**

---

### 2. `shoulder_lift` could not reach the real rest pose (2026-09-09)

**Symptom.** The real arm's rest pose lays the upper arm down *onto* the
base housing. The sim's arm stopped visibly higher, hovering above it.

**Root cause.** `so101.xml` gives `shoulder_lift` ±100°. Both arms on this
bench travel further — `twin_follower_3` calibrates to a **219.4°** span
(±109.7), `twin_leader_2` to 209.2°. `--pose folded` was already
commanding −100°, the model's hard stop. The sim physically could not
represent the pose.

**Fix.** `widen_shoulder_lift()` in `real_sim_joint_mapping.py` sets the
range to ±110° (derived from the measured 219.4°, not guessed), applied at
runtime via `MjSpec` so **`so101.xml` is never edited** — per this
project's rule that per-arm differences live outside the shared model.

**Two traps found here:**

1. **Both the joint range AND the actuator's `ctrlrange` must be widened.**
   Widening the joint alone measurably does nothing — it still settles at
   exactly −100.0°, because the position actuator clamps the command
   before the joint limit is consulted.
2. **`load_sim_joint_ranges_rad()` re-reads the XML**, so it returns the
   original ±100 even after widening. Added
   `sim_joint_ranges_from_model(model)` to read from the *compiled* model
   instead. Without this the mapping keeps targeting a limit the model no
   longer has, and the widening has no visible effect at all.

**Result.** `shoulder_lift` twin RMSE **9.82° → 0.53°**. The clearest,
best-verified win of the whole effort.

---

### 3. Real/sim lag of 140–500 ms (2026-09-10)

**Symptom.** Visible delay between leader motion and follower/sim motion.
First instinct was that the laptop (i3-7020U) was too weak, and that
moving to an i9/4090 machine would fix it.

**What the data showed.** Measured tick period at 30 fps: **median 15.6 ms
against a 33.3 ms budget** — the loop was idle half the time. Not CPU
bound. Cross-correlating leader against follower gave 140–500 ms lag,
against a *measured* hardware floor of 65 ms.

**Root cause.** The per-command clamp (`max_relative_target = 4.0`). At 30
fps the follower can close at most 120 units/s. `wrist_flex` averaged 8.8
units behind the leader (peak 69.8) — persistently clamped.

**Fix.** `--fps 60`. Same clamp, twice the ticks, so 240 units/s.

| Joint | lag @30fps | lag @60fps |
|---|---:|---:|
| shoulder_lift | 500 ms | 60 ms |
| wrist_roll | 500 ms | 50 ms |
| wrist_flex | 440 ms | 140 ms |
| shoulder_pan | 270 ms | 120 ms |
| elbow_flex | 140 ms | 0 ms |

**Why 60 fps is the ceiling.** The 15.6 ms median is unchanged between 30
and 60 fps — it is the serial round-trip (read follower, read leader,
write follower ≈ 5.2 ms each), not compute. 70 fps needs 14.29 ms, below
that floor. **A faster PC would not change this**; the CPU is already
waiting on the bus. Real levers, none tried: raise the servo baud above
1 Mbps, drop a bus transaction per tick, or decouple render from control.

---

### 4. Assorted fixes

| Problem | Fix |
|---|---|
| LeRobot logged a clamp warning every tick; synchronous Windows console writes starved the loop and the sim looked frozen | `logging.getLogger().setLevel(logging.ERROR)` |
| Sim ran ~1/6 real speed (one `mj_step` per frame vs 5 ms timestep) | `sim_steps_per_frame = round((1/fps) / timestep)` |
| `--recover` could walk `wrist_roll` forever (no hard stop) | Cumulative travel cap + wrap-aware shortest-path distance (`f03e77a`) |
| `[fold]` messages fired on single-frame contact flicker — 85 messages in 62 s | `FOLD_DEBOUNCE_TICKS = 10`; 85 → 9 messages |
| Could not tell "mapping broken" from "sim stuck" from "render broken" | `--debug-track` prints `real`/`ctrl`/`qpos` side by side |

---

## Unresolved

### A. Table height mismatch — MEASURED, NOT FIXED

**The problem.** At reach-down poses the sim's gripper jaws sit **3–5 cm
below** the modeled tabletop, at poses the real arm reached cleanly
(224 of 351 sampled frames in `teleop_log_v8.csv`).

**What is NOT the cause** — each checked and ruled out:

- **Arm kinematics.** At the all-zeros pose the sim gripper spans
  19.8–26.6 cm above the tabletop; the real one measures 28 cm. Consistent.
- **Joint mapping.** At the worst-penetration pose every arm joint maps
  within 1.6° of its true angle.
- **A clamp/mount gap.** Confirmed by hand: the base plate sits flat on the
  table, no gap.
- **Corner vs centre placement.** Identical penetration either way
  (224 frames, 4.7 cm) — `--no-corner` changes x/y only.

**Three fixes attempted, all reverted:**

1. **Raise the base ~7.5 cm.** Whole robot floated above the table.
2. **Lower the table 7 cm.** Eliminated *all* penetration (224 → 0 frames)
   and made a solid table cost nothing measurable — but left the base
   hanging 7 cm above the surface it is bolted to. Looked worse than the
   problem.
3. **Make the table solid** (remove the contact exclusion). The arm stops
   *on* the table but contorts into the wrong pose — up to 38.9° of joint
   error, and 126° at stiff contact settings.

**Contact tuning does not help.** Softening contact parameters was swept
across a 40× stiffness range: the jaws penetrate ~5 cm at *every* setting.
Stiffer contact adds contortion without stopping penetration, because the
position actuators overpower any contact force. One real finding did come
out of it — softening the **arm's** geoms (not the table's) to
`solref 0.15` cut fold-window error (`elbow_flex` 24.6° → 12.3°), because
MuJoCo takes contact params from the geom with higher `priority`, and
`so101.xml` sets `priority=1` on the arm. Not applied; worth revisiting.

**Current state:** table-vs-arm contact is **excluded** in all three
scripts. Tracking is correct; the gripper visually phases through the
tabletop at low poses.

**Workaround in use:** `--bare` loads `robot_bare_scene.xml` — the robot on
MuJoCo's standard checkerboard groundplane, no table. Only **1 of 274
frames** touches the floor, so nothing needs excluding and nothing phases.
All recent recordings use this.

**To actually resolve it**, one of: model the real mount properly; measure
where the real gripper sits at several *reach* poses (not just the zero
pose) to find whether the offset is constant or pose-dependent; or accept
`--bare` and drop the table until Phase B needs scene parity.

---

### B. Self-collision at tight folds — ACCEPTED TRADEOFF

At tightly folded poses the sim's collision primitives touch
(`shoulder↔gripper`, `shoulder↔wrist`, `shoulder↔lower_arm`,
`shoulder↔camera_mount`, `upper_arm↔wrist`) where the real arm folds
fine. The model's collision shapes are coarser than the real arm's
clearance.

Both options were tried:

| Setting | Visual | Tracking |
|---|---|---|
| Self-collision **excluded** | arm passes through itself | accurate (elbow 24.6° → ~2°) |
| Self-collision **on** (current) | correct | settles 20–40° short at tight folds |

**Chosen: ON**, for correct-looking geometry. This is the entire source of
the `elbow_flex`/`wrist_flex` RMSE. `[fold]` messages mark the affected
windows live and in replay.

> `run_robot_on_table.py --pose folded` (`shoulder_lift −110°,
> elbow_flex 92°, wrist_flex 60°`) is a verified collision-free reference
> for how deep a fold stays clean.

---

### C. Watch out: idle time inflates RMSE

`teleop_log_pick4.csv` (600 s) reports **8.15° overall** — the worst
number recorded — but the arm actually **moved in only 2.1% of ticks**.
It sat parked in a self-colliding fold for ~98% of the session, so 95% of
`wrist_flex` ticks carry >5° error.

**That is not a regression.** Before comparing sessions, either trim idle
time or compare medians. A long recording of a stationary arm is not a
better dataset than a short one of real motion.

---

## Session 2026-09-10 (afternoon): the cube — what actually happened

Ran plan item 1 below. **The scene, cameras and analysis tooling are done
and committed; the grasp itself is NOT working.** The prediction in that
plan — "expect the tuning to be in grasp contact, not setup" — turned out
to be wrong in an instructive way, and that is the main result of the
session.

### THE HEADLINE: a recorded teleop session can never grasp anything

Not a tuning problem. Not fixable with friction, `solref`, `solimp`,
`condim`, or cube mass. **Measured across 8 cube sizes from 0.6 cm to
1.8 cm — none were lifted, at any size.**

Two gaps constrain the problem, and the second is the one that is easy to
miss:

**(1) The open gap** — how wide the jaws ever get. Tip-to-tip separation
vs the gripper's normalised command, read off the compiled model:

| gripper norm | jaw gap | |
|---:|---:|---|
| 1.7 | 0.6 cm | fully closed — where every recorded session starts |
| 21.2 | 3.4 cm | **the widest any recorded session opens** |
| 40 | 6.0 cm | first setting that clears a 5 cm cube |

**(2) The approach gap** — the real killer. The arm does **not** approach
with the gripper open. From `teleop_log_pick2.csv`, frame by frame:

```
t=23.0   arrives at the target, gripper 11.19  ->  gap 1.93 cm
t=23-25  sits there, still 1.93 cm
t=25.5   OPENS to 21.0                         ->  gap 3.38 cm
t=26.0   closes
t=26.5   fully shut                            ->  gap 0.59 cm
```

The jaws travel **into** the grasp position while only 1.93 cm apart, then
open around the object in place. So:

- any cube wide enough to be gripped is **bulldozed ~3 s before the
  grasp**, at t=22.7;
- any cube narrow enough to survive that **slips through the 0.59 cm
  closed jaws**.

The window between those two is empty. And the failure *looks* exactly
like bad contact tuning — the cube skitters away on contact — which is why
it is worth writing down. **Check both gaps before touching contact
parameters.** No friction value lets a cube pass through a gap narrower
than itself.

**Why the recordings are like this:** they were made with no object
present. There was nothing to avoid and nothing to open around, so the
operator drove the shape that felt natural against thin air.

**What to do differently when recording a graspable session:**

1. **Open the gripper wide BEFORE approaching** — 40+ normalised units,
   not 21.
2. **Approach from above**, descending onto the object, rather than
   sweeping in horizontally at object height.
3. Keep the jaws open until the object is between them.

### What was built and works

| Thing | Status |
|---|---|
| `scripts/analyze_workspace.py` | **works** — FK a recording, report where the gripper went and when it closed |
| `robot_cube_test/robot_cube_scene.xml` | **works** — bare scene + 2 cm cube + 3 cameras |
| `robot_cube_test/validate_robot_cube.py` | **works** — 16/16 headless checks pass |
| `replay_teleop_sim.py --cube` | **works** — replays into the cube scene, measures and reports lift |
| `robot_cube_test/grasp_cube.py` | **INCOMPLETE** — see below |

Cube is **2 cm**, matching the real bench cube (measured 2026-09-10), not
M2's 5 cm test cube. Cameras `overhead`, `grasp_view`, `workspace` all
verified to render real images — on this Intel HD 620 that is a separate
fact from being *defined*, so the validator checks both.

Where the recordings actually close the jaws, from `analyze_workspace.py`:

| Session | x | y | z |
|---|---:|---:|---:|
| `teleop_log_pick2.csv` | +0.373 | +0.027 | 0.094 (jaw midpoint) |
| `teleop_log_pick3.csv` | +0.359 | +0.022 | 0.103 (site frame) |
| `teleop_log_60fps.csv` | +0.347 | +0.020 | 0.069 (site frame) |

Note the `gripperframe` site sits ~2 cm above the jaw tips — do not use it
as the grasp point.

### `grasp_cube.py` — unfinished, and exactly where it stops

A scripted pick (open → hover above → descend → close → lift) that does
**not** yet lift the cube. Current state: the IK reaches the cube cleanly
(residual ~3 mm) and the cube twitches +0.3 cm on contact, but is never
held.

**Root cause, seen in a render:** the gripper arrives with the jaws
rotated into the wrong plane — the cube ends up *beside* the jaws rather
than between them, and the moving jaw swings toward the floor instead of
over the object. It is a wrist-orientation problem, not contact physics.

Two IK traps already found and worth not repeating:

1. **Targeting the jaw MIDPOINT with orientation unconstrained does not
   work.** With the jaws open the tips are ~5 cm apart, so "midpoint on
   the cube" is satisfied by poses with one jaw on the cube and the other
   6.5 cm away in mid-air. Measured: IK residual 0.03 mm, cube never
   moved.
2. **The pitch constraint introduces local minima.** Sweeping the target x
   gave 6 mm residuals at x=0.32 and x=0.36 but 110–170 mm at neighbouring
   values — bad starting guesses, not unreachable poses. `ik_solve()` now
   seeds several starts and keeps the best.

Also measured: a **top-down straddle is only achievable beyond x≈0.30**.
Below that the wrist cannot pitch over. The cube sits at x=0.32 for this
reason.

**Next step when resuming:** constrain the gripper's *roll* as well as its
pitch, so the jaw opening plane contains the cube. The position and pitch
rows are already in the Jacobian; a third orientation row is the likely
fix.

### Not done this session

- **Sim → real (`--source sim`)** — plan item 2 below, untouched. MuJoCo
  still has never driven the hardware.
- **Frame capture to disk** — cameras render, but nothing writes a
  PNG+state dataset yet.
- **Real cube / vision** — still Phase B, still unstarted.

---

## Original plan for 2026-09-10

Ordered so each step is independently useful.

### 1. Add a cube to the sim and grasp it — the main task

Version A of yesterday's question: **the object exists in sim only.** The
sim gripper physically grasps and carries a cube with MuJoCo contact
physics, while teleop drives the arm.

- Start from `spawn_cube_test/spawn_cube_scene.xml` (`size="0.025"`
  half-extent = a 5 cm cube, 50 g, already validated) and add it to
  `robot_bare_scene.xml`.
- Place it inside the workspace the recordings actually visit — derive
  reachable positions from `teleop_log_pick2.csv` rather than guessing.
- Expect the tuning to be in **grasp contact**, not setup: jaw/cube
  friction and closing force decide whether the cube is held, slips, or is
  flung. The gripper tracks at 0.60° RMSE, so the arm side is fine.
- Validate by replaying `teleop_log_pick2.csv` (a real pick-and-place,
  gripper travel 16.1 units) and checking the sim cube gets picked up.

> **Outcome: the last two bullets were both wrong.** The tuning is not in
> grasp contact, and `teleop_log_pick2.csv` cannot validate anything —
> see the session notes above. The 5 cm cube is also far too big: the
> jaws never open past 3.4 cm in any recording.

**Not** version B (sim cube mirroring the *real* cube's position) — that
needs vision, which is Phase B and unstarted.

### 2. Sim → real validation (`--source sim`)

Currently `replay_teleop_real.py` sends the CSV's `real_norm` column, i.e.
the real arm's own past positions. **MuJoCo has never driven the
hardware.** Feeding `sim_qpos_deg` back through `sim_to_real_vector()`
(already exists) would be a genuine closed-loop test of whether the sim is
accurate enough to command the real arm.

Gate it to the clean middle of a recording — the folded ends diverge
19–40° and must not be sent. Dry-run first.

### 3. Housekeeping

- ~~**Commit.** 12 untracked files including all the new tooling; nothing
  from the last two days is in git.~~ **DONE** — `1b7e12f`.
- ~~**Recordings are scattered** — several sit in
  `scripts/digital_twin_env/robot_clamped_test/` because that was the
  cwd. Move to one `recordings/` folder.~~ **DONE** — all in
  `recordings/`, which is gitignored.
- Fold `--bare`, `--wires`, `--record`, `--fps 60` and the replay scripts
  into `LEADER_TELEOP_RUNBOOK.md` and `twin.py`. **Still open** —
  `analyze_workspace.py`, `validate_robot_cube.py` and `grasp_cube.py`
  are not registered in `twin.py` either.

### Deferred

- Table height (§A) — blocked on a modelling decision, not effort.
- Servo baud rate — the only real lever left on lag, but it means writing
  a new baud to 12 servos across two arms, with a real risk of leaving one
  unresponsive mid-way.
- Filtering fold windows out of training data — matters before training,
  not before the cube works.

---

## Standing commands

```powershell
# live teleop, 60 fps, no table, recording
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe `
  D:\robotics\so101-digital-twin\scripts\m_lerobot_teleop_sim.py `
  --leader-port COM8 --leader-id twin_leader_2 `
  --follower-port COM14 --follower-id twin_follower_3 `
  --bare --wires --fps 60 --record teleop_log_NAME.csv

# check a recording
...\python.exe scripts\analyze_teleop_accuracy.py <csv>
...\python.exe scripts\plot_teleop_accuracy.py <csv>

# replay: sim only, then real + sim
...\python.exe scripts\replay_teleop_sim.py <csv> --bare --wires
...\python.exe scripts\replay_teleop_real.py <csv> `
  --follower-port COM14 --follower-id twin_follower_3 --bare --wires --fps 60

# where did the gripper actually go in a recording, and when did it close?
...\python.exe scripts\analyze_workspace.py recordings\teleop_log_pick2.csv

# cube scene: headless checks, then replay a session into it
...\python.exe scripts\digital_twin_env\robot_cube_test\validate_robot_cube.py
...\python.exe scripts\replay_teleop_sim.py recordings\teleop_log_pick2.csv --cube --wires

# scripted grasp (DOES NOT LIFT YET -- see the 2026-09-10 session notes)
...\python.exe scripts\digital_twin_env\robot_cube_test\grasp_cube.py --headless
```

**Ports move between plug-ins.** Re-check every session — the stable ids
are the adapter `InstanceId` tails: leader `5AB0158158`, follower
`5AE6057392`.

```powershell
Get-PnpDevice -Class Ports -PresentOnly |
  Where-Object { $_.FriendlyName -match "CH34" } |
  Select-Object FriendlyName, InstanceId
```

Replaying to the real arm reproduces **joint angles only** — it does not
know where the cube is. Reset the cube by hand first, or the arm grasps
empty air.
