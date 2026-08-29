# SO-101 Digital Twin: From Keyboard Control to Leader-Arm Teleop

**A complete overview — what we've built, how we validated it, and what comes next.**

**Who this is for:** written in three layers on purpose. The overview
sections (1–4) are readable by anyone — a mentor doing a first pass, a
teammate catching up. Section 5 goes deep enough for someone who wants the
actual formulas and code. Section 6 compares our approach against the
established robotics-industry toolchain (ROS/Gazebo), for readers who know
that world and want to place this project on a map they already understand.
Section 7 is the validation record — what was checked, how, and what's
still open. Skip to whichever altitude you need; each section stands alone.

**Source material:** this document synthesizes and cross-references
everything already written for this project — `CLAUDE.md`,
`docs/PROJECT_STATUS.md`, `docs/REAL_SIM_MAPPING_DEEP_DIVE.md`,
`docs/MOTOR_CHECK_AND_MAPPING_SIMPLE.md`, `docs/ROADMAP_TELEOP_TO_DATASET.md`,
and the characterization results in `scripts/characterization/`. Nothing
here contradicts those files; where a number is quoted, it traces back to
one of them.

---

## 1. The One-Paragraph Version

We built a physical robot arm (SO-101, 6 motors) and a matching copy of it
inside a physics simulator (MuJoCo). We proved the two can be made to agree
— not just "look similar," but measurably, with numbers — first by
matching where each joint sits (position), then by matching how each joint
*moves* (speed, lag, overshoot, mechanical slack), measured directly off
the real hardware rather than guessed. We built keyboard control for both
the simulated arm alone and the real arm alone, then combined them so the
simulator visibly mirrors the real arm's actual measured behavior — gaps
and all — rather than hiding imperfections. All 6 joints are now
individually measured and tuned. The next step is replacing the keyboard
with a leader arm (a second SO-101 moved by hand), which the existing
architecture was deliberately built to accept without being rebuilt.

---

## 2. Why This Exists — The Underlying Goal

A simulator is only useful for training or testing a robot if it actually
behaves like the real one. An idealized simulation — one where a "move to
X" command teleports the joint there instantly — teaches nothing true about
the real world. Any data generated from it, or any control logic tested
against it, would be built on a fiction.

So before doing anything ambitious (leader-arm teleop, dataset collection,
eventually training a model), we had to answer one boring but essential
question first: **when we tell a real joint to move, what does it actually
do — and can we make the simulated joint do the same thing?**

Everything in this document is either (a) answering that question, or (b)
things we can only responsibly build once that question is answered.

---

## 3. The Journey, In Order

### Stage 1 — Simulated arm, keyboard only (M5)

Built a keyboard-driven control loop entirely inside the simulator. No
hardware. Proved the control scheme (which key moves which joint, how far
per keystroke, safety clamps against the joint's own limits) works before
any hardware risk was introduced.

### Stage 2 — Real arm, keyboard only (M6)

The same keyboard scheme, now driving the real servos through an
open-source hardware library (LeRobot), with no simulator involved. This
isolated "can we safely move the real arm" from "does the simulator agree
with it" — two different questions, deliberately not tackled at the same
time.

Four safety layers, always on: a focus gate (keys only act while a specific
window is focused, since the underlying keyboard library listens
system-wide by default), a per-command motion clamp, a safe position
envelope well inside the arm's true limits, and only one joint "live" by
default.

### Stage 3 — Measuring the real arm's actual behavior (Task 1)

Before trying to make the simulator match the real arm, we measured what
the real arm actually does — not assumed it. A script commands one real
joint through a set of test movements (small steps, big steps, reversing
direction, smooth ramps, a repeating triangle pattern, with and without
extra load from gravity), logging position/speed/motor-load many times per
second. From that we extracted five numbers per joint: how long it takes to
start moving after a command, how far off target it settles, its top
speed, its mechanical slack, and how load changes its accuracy.

### Stage 4 — Fitting the simulator to match (Task 2)

Took the real measurements from Stage 3 and searched for simulator settings
that reproduce that real motion — not just the end position, but the shape
of the movement over time (acceleration, overshoot, settling). Checked the
result against movement patterns the fitting process had never seen, to
catch the difference between a real fit and a coincidence.

### Stage 5 — Both together, keyboard driving both (M7)

Keyboard drives the real arm; every fraction of a second, we read back what
the real servo *actually did* — not what we told it to do — and push that
measured position into the simulator. Any real-world imperfection (lag,
slack, gravity droop) stays visible on screen instead of being hidden by a
simulator that would otherwise just replay the command perfectly.

### Stage 6 — Completing all 6 joints (2026-08-29)

Stages 3–4 were originally done for one joint (`elbow_flex`) as a proof of
concept, then a second (`shoulder_lift`) to check whether the same fit
transfers between joints — it didn't, cleanly, which was itself an
important finding (see §5.4). As of 2026-08-29, all 6 joints have been
individually measured and fitted. See §5.5 for the full per-joint table.

### Stage 7 (next) — Leader-arm teleop

Not started yet, pending leader-arm hardware. See §8.

---

## 4. What "Matching Real and Sim" Actually Means — Two Separate Layers

This is worth stating plainly because it's easy to blur into one idea when
it's actually two:

**Layer 1 — Position matching.** Where is the joint, right now? Solved with
a percentage-of-range rescale: the real joint's position, expressed as a
percentage of its own calibrated travel, gets applied as that same
percentage of the simulator's own range. One formula, works identically for
all 6 joints, needs no per-joint tuning. Fully solved, unit-tested with
zero hardware risk before ever touching the arm.

**Layer 2 — Behavior matching.** When the joint moves from A to B, does it
accelerate, overshoot, and settle the way the real one does? This is not
solved by a formula — it required actually measuring the real joint (Stage
3) and tuning the simulator's internal physics settings to reproduce that
measured motion (Stage 4). This is per-joint work, because — as directly
proven in this project — joints that share the exact same motor part can
still behave meaningfully differently (see §5.4).

**A simulator can have Layer 1 correct and Layer 2 wrong.** That combination
looks fine at a glance (the arm ends up in the right place) but is
dynamically dishonest (it gets there in an unrealistic way). Both layers
are now done for all 6 joints as of 2026-08-29.

---

## 5. Technical Detail

### 5.1 The position-matching formula

Real joint position (from calibration, `-100..100`, or `0..100` for the
gripper) → percentage of that joint's own travel → same percentage applied
to the simulator's own radian range:

```python
def real_to_sim(joint_name, real_normalized, sim_range_rad):
    lo, hi = sim_range_rad
    if joint_name == "gripper":
        frac = real_normalized / 100.0
    else:
        frac = (real_normalized + 100.0) / 200.0
    return lo + frac * (hi - lo)
```

Lives in `scripts/digital_twin_env/real_sim_mapping_test/real_sim_joint_mapping.py`.
Validated headlessly (`validate_real_sim_mapping.py`): boundary values land
exactly on the simulator's limits, round-trip conversion returns the
original value to within `1e-6`, out-of-range inputs clamp rather than
extrapolate. Full derivation and the "what it does NOT guarantee" caveat
(percentage-matching doesn't guarantee the same *physical* angle, only the
same *proportion* of travel) in `docs/REAL_SIM_MAPPING_DEEP_DIVE.md` §5.

### 5.2 The characterization protocol (Task 1)

Per joint, five experiment types, run via `so101_joint_characterization.py`
(or `python scripts/twin.py characterize --joint <name>`):

| Experiment | What it reveals |
|---|---|
| Step (5/10/20 units) | Latency, overshoot, settling error, whether behavior scales with move size |
| Reverse direction | Direction-dependent behavior — an early signal for backlash |
| Ramp | The speed at which real position starts lagging behind commanded position — the effective velocity ceiling |
| Triangle (repeated zig-zag) | Lag and smoothing behavior, visible on one plot |
| Loaded vs. unloaded | How gravity/load changes accuracy and speed |

Sampled at ~95–100 Hz, with the actual elapsed time recorded per sample
rather than assumed — the serial round-trip to the servo, not a fixed
clock, sets the real rate.

### 5.3 Backlash — measured with a dedicated method, not the above

The step/reverse/triangle experiments above could **not** cleanly measure
backlash: every trajectory approached position 0 from the same direction,
so no command was ever settled from both directions — the one thing
backlash measurement requires. A separate staircase probe
(`backlash_probe.py`) walks each joint to the same rung from both
directions and holds until fully settled, specifically to separate two
effects that look identical in position data alone:

- **Backlash** — mechanical slack/play in the gears, roughly constant.
- **Gravity droop** — steady-state sag under the joint's own load, which
  varies with angle, following a sine curve of angle-from-vertical (not
  cosine — an early analysis mistakenly assumed cosine and got a
  confident-looking but wrong fit, R² 0.056 vs. sine's 0.880; caught by
  checking fit quality rather than trusting the number).

### 5.4 The key finding that shaped everything after it

`elbow_flex` and `shoulder_lift` use the physically identical servo part.
Fitting `elbow_flex`'s data and applying those settings to `shoulder_lift`
was tested and explicitly rejected — held-out error got *worse*, not
better, when `shoulder_lift` was force-fit with a softened gain. The root
cause: `shoulder_lift` carries far more gravitational load (holding up the
rest of the arm) than `elbow_flex` does, and its settled position varies
with load in a way no single linear gain could reproduce without breaking
the joint's fast-motion response. **This is the direct evidence that "same
motor part" does not mean "same behavior,"** and it's why every one of the
remaining 4 joints was independently characterized and independently
scored for whether its own fit was a real improvement or overfitting —
rather than assuming the elbow's numbers would simply transfer.

### 5.5 Full per-joint results table (2026-08-29, all 6 joints, `twin_follower` arm)

| Joint | Motor ID | Backlash | Gravity coupling | Peak velocity | Own gain fit vs. shared? | Sim-to-real RMSE |
|---|:--:|---:|---|---:|---|---:|
| `elbow_flex` | 3 | 1.196° | Sine fit R² 0.975 | ~150–190°/s | Original fit target: `kp=400 kv=25` | 0.434–0.638° |
| `shoulder_lift` | 2 | 1.955° | Sine fit R² 0.880 | lower | Own sweep rejected — held-out got worse; kept shared fit | — |
| `shoulder_pan` | 1 | 0.381° | Fit failed (R² 0.236) — expected, rotates about vertical axis | — | Own sweep improved fit 6% but held-out 8% *worse* (overfitting) — kept shared fit | 0.521° |
| `wrist_flex` | 4 | 1.134° | Weak (R² 0.294), small flat droop consistent with light gripper-only load | — | Genuine improvement, not overfitting: `kp=20 kv=1` (22%/21% better, held-out also improved) | 0.601° |
| `wrist_roll` | 5 | 0.658° | Weak (R² 0.406) — expected, rotates about its own long axis | 250.8°/s (fastest joint measured) | Genuine improvement: `kp=400 kv=35` (29%/7% better, held-out also improved) | 0.547° |
| `gripper` | 6 | 0.821° | Not applicable — pinch mechanism, not a lever against gravity | — | Genuine improvement: `kp=80 kv=6` (19%/12% better) | 0.465° |

Every "genuine improvement" claim above passed the same test: does
held-out error (data never shown during fitting) improve alongside fit
error? If only the fit-set number improves, that's overfitting, not a real
result — and this project has direct history of catching that distinction
matter (§5.4, and the `shoulder_pan` row above).

**The gripper required real code changes, not just new data** — the
existing characterization scripts hardcoded the `-100..100` convention used
by the other 5 joints, but the gripper is `0..100` with a different meaning
entirely. This was fixed and verified headlessly (non-gripper trajectories
reproduced byte-identical to before the fix) before ever running it on
hardware.

**A genuine hardware discovery came out of this work**: the gripper's
calibrated `range_max` implies it can open further than it physically can
— the real jaw hits a hard stop at ~92.7° (≈68 normalized units), confirmed
by eye against the hardware, not a software bug. Commanding past that
stalls the servo under load, which read as a large false "steady-state
error" until caught. The safe operating ceiling was tightened accordingly.

### 5.6 An open hardware issue, honestly reported

While validating M6/M7 after completing all 6 joints, `shoulder_lift`'s
`-1` direction was found to accept commands (the target value visibly
counts down correctly every tick) but the servo does not move — reproducible
after a full recalibration, in a fresh session, at any starting position.
Extensive isolation testing (documented in full in `docs/PROJECT_STATUS.md`)
ruled out: firmware limits, stale calibration, LeRobot's own safety clamp,
and every low-level write mechanism tested standalone — all of which worked
correctly outside the live control loop. The remaining suspect is an
interaction with the keyboard-listener's background thread, not yet
confirmed. Current workaround: `shoulder_lift` moved onto dedicated
`up`/`down` keys so its working `+1` direction isn't compromised by pairing
it with a non-working key.

**Why this matters for the leader-arm plan (§8):** a leader arm removes the
keyboard listener from the picture entirely. If the suspected cause is
correct, this specific symptom likely disappears with a leader arm — but
that has not been confirmed, and the honest position is "probably, not
certainly" until the parked isolation test (driving `shoulder_lift`
programmatically with no keyboard listener running) is actually run.

---

## 6. How This Compares to the Established Robotics Toolchain (ROS / Gazebo)

For anyone whose reference point is the industrial/ROS ecosystem — clients
or mentors who've worked with a "Manipulator X"-style arm, MoveIt, or
Gazebo — here's how our approach maps onto concepts that world already has
names for, and where it genuinely differs.

### 6.1 The same problem, by another name

What we call "matching real and sim" is what the ROS/Gazebo world calls
**sim-to-real transfer**, and what we did with `kp`/`kv` fitting is a form
of **system identification** — a decades-old control-theory technique, not
something novel to this project. In the ROS ecosystem, this typically
happens through:

| ROS/Gazebo concept | Our equivalent |
|---|---|
| `ros2_control` — the standard framework connecting a controller (position/velocity/effort) to both real hardware and a Gazebo-simulated joint through the same interface | LeRobot's `SOFollower`/`SOLeader` drivers + our own mapping module — a much smaller, arm-specific version of the same idea (one interface, real or simulated backend) |
| `gazebo_ros2_control`'s PID gain parameters (`kp`, `kd`, `ki` per joint) | Our `kp`/`kv` fit in `so101.xml` — MuJoCo's built-in position actuator uses a PD-style law, conceptually the same tuning problem, fewer terms (MuJoCo's actuator has no separate integral term the way a full ROS PID controller does) |
| `MoveIt` — motion planning, collision-aware trajectory generation, inverse kinematics | Not built yet. Everything so far is direct joint-space teleop (keyboard, soon leader arm); no path planning or IK layer exists in this project |
| A URDF (Unified Robot Description Format) | We use MJCF (MuJoCo's native XML format) instead — conceptually the same role (describing bodies, joints, actuators), different engine's native format. The official SO-101 URDF exists upstream (`TheRobotStudio/SO-ARM100`) and MuJoCo Menagerie's MJCF (which we use) is derived from it |
| Domain randomization for sim-to-real RL policies | Named explicitly as planned future work (Phase H of our roadmap), not yet built |

### 6.2 Where our approach is smaller in scope, on purpose

A full ROS/Gazebo setup is built to be **hardware-agnostic and
general-purpose** — the same `ros2_control` interface can drive a UR5, a
Franka arm, or a custom design, with the controller framework doing the
heavy lifting of abstraction. We deliberately did **not** build that
generality. Our mapping module, characterization scripts, and gain fits are
specific to the SO-101 and its STS3215 servos. This was a conscious
trade-off: a general framework is more work to build correctly and would
have delayed the actual measurement work this project prioritized (measure
the real hardware first, build tooling only as needed).

### 6.3 Where our approach goes further than a typical first Gazebo setup

Many Gazebo tutorials and even some production setups stop at "load the
published URDF/SDF, use whatever PID gains ship with it." Our own official
starting file's shipped gains (`kp=998.22 kv=2.731`, seen in the upstream
`so101_new_calib.xml`) were explicitly derived from an unrelated robot
project's formula — never validated against real SO-101 hardware, per that
file's own comment. **A very common failure mode across the wider SO-101
community is trusting those shipped defaults as if they were measured.**
This project did the opposite: treated the shipped file as an initial guess
only, then did the actual system-identification work (Stages 3–4 above) to
replace it with hardware-measured values, joint by joint, with an explicit
overfitting check at every step. That discipline is not automatic even in
mature ROS/Gazebo setups — it has to be deliberately done, and frequently
isn't, especially on hobbyist-cost hardware where "good enough to look
right" is often where the work stops.

### 6.4 The validation gate, compared

The ROS/Gazebo world doesn't have one universally standard acceptance test
for "is my simulated arm good enough," but the closest common practice —
replaying a real recorded trajectory through the simulated controller and
comparing tracking error — is exactly what our `replay_in_sim.py` does
(§7.3). Framing it explicitly as a **pass/fail gate before trusting
anything built on top** (rather than an optional sanity check) is the
project's own discipline, not a standard everyone in that ecosystem
enforces equally rigorously.

Sources on the ROS/Gazebo comparison points above:
[ros2_control PID gain configuration](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html),
[Gazebo ROS control tutorial](https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros),
[quantifying sim-to-real gaps via gain regularization](https://arxiv.org/pdf/2507.23445).

---

## 7. Validation — What Was Actually Checked, and How

Validation happened in layers, each one checkable independently, so a
failure anywhere is traceable to a specific cause rather than a mystery.

| Layer | What was checked | Method | Hardware needed? |
|---|---|---|---|
| Scene/model loads correctly | All 4 MuJoCo scenes load, integrate 100 physics steps, finite state | `validate_scenes.py` / `twin.py validate` | No |
| Position-mapping math | Boundary exactness, round-trip conversion, out-of-range clamping | `validate_real_sim_mapping.py` / `twin.py map-check` | No |
| Real joint behavior is trustworthy | Repeated trials, measured (not assumed) sample rate, retried serial writes, corrupted-packet filtering | `so101_joint_characterization.py` | Yes |
| Backlash isolated correctly, not confounded | Bidirectional staircase approach; both sine and cosine gravity fits attempted and compared, not just one assumed | `backlash_probe.py` / `analyze_backlash.py` | Yes |
| Sim dynamics match real dynamics | Same commands, same timestamps, real-vs-sim overlay, RMSE reported per run and in aggregate | `replay_in_sim.py` | No (reuses recorded data) |
| Gain fit isn't overfitting | Fit on step-response data only; scored separately against ramp/reverse/triangle data never used in fitting | `tune_actuator.py` | No (reuses recorded data) |
| Live system behaves as expected | Sim-only bring-up first (zero hardware risk), then real hardware, in that order | `m7_mirror_sim.py --source sim` then `--source real` | Progressive |
| **The one check nothing above replaces** | **Visual confirmation with hardware connected** — command a known real pose, watch whether the simulator visually matches | By eye | Yes |

### 7.1 A pattern worth naming: we caught our own mistakes, more than once

This project's history includes several conclusions that were reached,
then found wrong, then corrected — not glossed over, documented on the
record:

- A false **"46° shoulder_pan mismatch"** between real calibration and the
  simulator's kinematic range, which nearly triggered an unnecessary
  recalibration before being recognized as comparing two quantities that
  were never supposed to match in the first place.
- **Two wrong backlash readings** (11.3°, then a "confounded" 1.337°),
  both from the same underlying flaw (every test trajectory only ever
  approached zero from one direction) — fixed not by re-analyzing the same
  data harder, but by designing a genuinely different experiment.
- A **wrong trig-basis assumption** for gravity droop (cosine instead of
  sine) that produced a confident-looking number until fit quality was
  actually checked.
- An **overfitting trap on `shoulder_pan`**'s own gain sweep — it looked
  6% better on the data it was fit to, and was quietly rejected because
  held-out error got 8% worse.

The reason this section exists in a document meant partly for mentors: a
project that only reports clean successes is harder to trust than one that
shows its own error-catching process working, on the record.

### 7.2 What validation does NOT yet cover

- **Object/scene matching** — nothing in this project reads a real
  object's position and reflects it in the simulator. Position matching
  and behavior matching, as covered in this document, are entirely about
  the arm's own joints.
- **Cross-arm generalization** — every measurement so far is on one
  physical arm (`twin_follower`). A second arm of the same design would
  need its own calibration at minimum, and per the ROS/Gazebo-adjacent
  "unit-to-unit variance is real" principle, potentially its own
  characterization pass too, before its numbers could be trusted.
- **The `shoulder_lift` `-1` direction issue** (§5.6) — open, not yet
  root-caused.

---

## 8. What's Next — Leader-Arm Teleop

### 8.1 What a leader arm actually is, functionally

A second SO-101, but used purely as an input device — a human moves it by
hand, and its own 6 motors' encoders report position. It needs no torque
and no dynamics modeling of its own (LeRobot's leader-arm driver doesn't
even implement force feedback) — it is a **position sensor with the same
shape as the arm it's meant to control**, nothing more.

### 8.2 Why this doesn't require rebuilding anything

The keyboard-control code (`keyboard_robot.py`, mirrored in M6/M7) was
deliberately built with a strict separation: keyboard input produces a
six-joint target, and a separate function pushes that target into whatever
is being controlled. A leader arm produces the exact same shape of output
(a six-joint target, in the same normalized units) — so it becomes a
drop-in replacement for the keyboard's target-generation step. Everything
downstream — the safety envelope, the position-mapping formula, the
"mirror the measured position" principle from M7 — carries over unchanged.

**Software needed:** already installed. LeRobot ships a `SOLeader` driver
in this exact venv, unused until now, whose entire interface is
`get_action()` — read the leader's current position, once per tick. No new
packages required.

### 8.3 Planned stages, mirroring the same bring-up discipline used throughout this project

1. **Calibrate the leader arm** — same one-time wizard already used on the
   follower, run against the leader's own port/ID.
2. **Read-only proof** — confirm the leader reports sane values as it's
   moved by hand, nothing driven yet.
3. **Leader → sim only, no follower connected** — proves the leader's
   readings flow correctly through the existing position-mapping formula,
   with zero hardware risk to the follower arm.
4. **Leader → follower only, no sim** — proves safe hardware-to-hardware
   teleop, inside the same safety clamps M6 already established.
5. **Leader → follower + sim mirrored, together** — the leader-arm
   equivalent of M7: the simulator mirrors the follower's *measured*
   position, exactly as it does today for keyboard control.
6. **Wire into `twin.py`** as a new subcommand alongside `real`/`mirror`.

### 8.4 The one genuinely new risk a leader arm introduces

A keyboard's worst case is a stuck key, naturally rate-limited by small
per-tick increments. A leader arm can be moved by a human arbitrarily fast
— swung end-to-end in under a second. The existing per-command motion
clamp partially guards against this already, but "move the leader as fast
as physically possible" is a stress case the current control loop has
never actually been tested against, and should be explicitly tried during
bring-up stage 4 above before trusting it.

---

## 9. Quick Reference — Where Everything Lives

| Topic | File |
|---|---|
| This document | `docs/KEYBOARD_CONTROL_TO_LEADER_ARM_OVERVIEW.md` |
| Current milestone/phase status | `docs/PROJECT_STATUS.md` |
| Full mapping + characterization technical writeup | `docs/REAL_SIM_MAPPING_DEEP_DIVE.md` |
| Per-motor table, simple explanation first | `docs/MOTOR_CHECK_AND_MAPPING_SIMPLE.md` |
| The larger roadmap (leader arm → dataset → Unreal → mixed training) | `docs/ROADMAP_TELEOP_TO_DATASET.md` |
| Per-joint raw characterization data | `scripts/characterization/raw_data/` |
| Per-joint sim-vs-real comparison data | `scripts/characterization/sim_data/` |
| Narrative results for the original two joints | `scripts/characterization/RESULTS.md`, `BACKLASH-RESULTS.md`, `FINDINGS-shoulder_lift.md`, `LOADED-RESULTS.md` |
| Unified command-line entry point for every script | `scripts/twin.py` (run with no arguments for an interactive menu) |
