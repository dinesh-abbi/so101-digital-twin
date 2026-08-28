# Real ↔ Sim Joint Mapping — How We Built and Validated the SO-101 Digital Twin

**Purpose of this document:** a complete, presentation-ready walkthrough of how
a real SO-101 robot arm's joint positions were matched to its MuJoCo
simulation counterpart — starting from the upstream model source, through
characterizing the real hardware, to the mapping math, to live validation.
Written to be turned into slides (each `##` section is roughly one slide or
one slide group).

---

## 1. The Problem, In One Sentence

We have a **real robot arm** (SO-101, 6 joints, Feetech STS3215 servos) and a
**simulated twin** of it in MuJoCo. Both can report "where is joint X right
now?" — but they speak two completely different numeric languages, and
nothing guarantees they agree about what "0" means. This project builds and
validates the translator between them.

---

## 2. Where the Simulation Model Came From

### 2.1 Upstream source

The kinematic/visual model (`so101.xml` + STL meshes) was **not written from
scratch**. It originates from The Robot Studio's official SO-ARM100/SO-101
description:

> https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101

That original description was subsequently adopted into **MuJoCo Menagerie**
(Google DeepMind's curated collection of vetted robot models) as
`robotstudio_so101`. This project vendors the model from Menagerie, not
directly from the TheRobotStudio repo, because Menagerie provides a
version that has already been checked for MuJoCo-specific correctness
(inertials, joint limits, collision geometry) — but the lineage traces back
to that original SO-ARM100 description.

- **Vendored from:** `google-deepmind/mujoco_menagerie`, path
  `robotstudio_so101`
- **Pinned commit:** `4c358ef9d9d7f32ca58b40b490884a0c1726a440` (pinned, not
  "latest," so the model never silently changes underneath the project)
- **License:** Apache 2.0

### 2.2 What we changed locally, and why

We deliberately kept modifications minimal and documented every one in
`so101_assets/PROVENANCE.md`, so the model can always be diffed against its
upstream origin:

1. **Camera field-of-view fix.** The upstream wrist camera specified physical
   sensor intrinsics (`sensorsize`/`focal`). MuJoCo ignores the `fovy`
   attribute whenever those physical intrinsics are present, which broke
   FOV randomization in this simulator. Fix: removed the intrinsics, set an
   explicit `fovy="48.5"` (computed as the vertical-FOV equivalent of the
   original intrinsics: `2 * atan(0.00324 / (2 * 0.0036)) ≈ 48.5°`).

2. **Gripper frame orientation fix.** The `gripperframe` site's quaternion
   was `1 0 1 0` upstream. Changed to `0 0 1 0` so the site's local Z axis
   points along the gripper's actual "forward" (wrist-to-fingertip)
   direction — required by this project's convention that TCP-orientation
   readings are taken from that axis.

**Rule enforced throughout the project:** never re-download the model fresh
from upstream, and never hand-edit `so101.xml` directly in place. Both fixes
would be silently lost, and any experiment that copies from this file (every
milestone does) would break. Changes go into new milestone folders that
`<include>` the base file.

---

## 3. The Two Coordinate Systems That Don't Naturally Agree

This is the conceptual core of the whole mapping problem.

### 3.1 Real hardware side (Feetech STS3215 servo)

- Each servo has a **12-bit magnetic encoder**: raw values **0–4095** per
  revolution.
- The encoder's zero point is **arbitrary** — wherever it happened to be
  when the horn was bolted onto the gearbox during assembly. It is *not* a
  physical reference like "arm pointing straight up."
- `lerobot-calibrate` (a one-time wizard, run per physical arm) sweeps each
  joint through its full range of motion and records the **observed
  minimum and maximum tick** it saw.
- At runtime, every position reading is converted from raw ticks into a
  **normalized** value using those recorded min/max:
  - The 5 arm joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
    wrist_roll) normalize to **-100 .. +100**.
  - The gripper normalizes to **0 .. 100**.
- This is what `SO101Follower.get_observation()` actually returns — **not
  ticks, not degrees.** Every script in this project that talks to real
  hardware receives this normalized form.

### 3.2 Simulation side (`so101.xml`)

- Joints are represented in **radians**, straight from MuJoCo's own
  `model.jnt_range`.
- These limits come from the **kinematic design** of the arm (how far a
  link can physically rotate before it hits another part), and have
  **nothing to do with any individual servo's calibrated tick range.**

### 3.3 Why they don't automatically match

Two physically "identical" arms of the same model, calibrated separately,
produce **different tick ranges** — because the servo horn splines onto the
gearbox shaft at a slightly different angle every time it's assembled by
hand. This was measured directly on this project's two arms:

| Arm | `shoulder_pan` calibrated span |
|---|---:|
| `left_follower` | 198.1° |
| `twin_follower` | 175.2° |
| `so101.xml` kinematic design | 220.0° |

A 23° difference between two arms of the *same design*, and both fall short
of the kinematic model's theoretical range. This is a **hard mechanical
fact about each individual arm**, not a bug to "fix" in software, and
definitely not something to fix by editing `so101.xml` — that file is
shared across every arm; hardcoding one arm's limits into it would break
every other arm.

**Known measured span mismatches (real vs. sim), same arm:**

| Joint | Real calibrated span | Sim kinematic span |
|---|---:|---:|
| wrist_roll | 359.9° | 314.4° |
| gripper | 143.0° | 110.0° |

`wrist_roll`'s real span reads the *entire* encoder range (0–4095 ticks) —
this joint physically spins continuously with no hard stop, so "359.9°" is
not really a measured limit at all, it's just "the whole circle."

---

## 4. Task 1 — Characterizing the Real Joint Before Touching the Simulation

**Guiding principle (from the original spec we followed):** *"The first
low-level task should not be simulation work. Start with the real robot and
answer one basic question: when we command an SO-101 joint to move, what
does the physical joint actually do?"*

We built the mapping math and the simulator gain-fitting **only after**
collecting real, trustworthy measurements — not the other way around.

### 4.1 What the script does

`scripts/characterization/so101_joint_characterization.py`:

- Talks to **one motor directly over the Feetech serial protocol** (not
  through LeRobot's higher-level driver), because we needed raw registers
  LeRobot doesn't expose: load, current, voltage, temperature — not just
  position.
- Commands a single joint through a trajectory while sampling as fast as
  the serial bus allows (~94–100 Hz measured, never assumed).
- Every sample is a full row, written straight to CSV:

  ```
  timestamp, joint_name, commanded_position_deg, actual_position_deg,
  commanded_velocity, measured_velocity, load, current, voltage,
  temperature, experiment_id, load_condition
  ```

- Retries every write up to 3 times — a single dropped serial packet had
  previously aborted an entire calibration run, so retry logic was made
  mandatory rather than optional.
- Keeps every command inside a **safe envelope** well short of the joint's
  true hard stops, so a bug can't drive the servo into itself.

### 4.2 The five experiments run on the real arm

| # | Experiment | What it reveals |
|---|---|---|
| 1 | **Step response** (5°, 10°, 20° jumps, repeated) | Whether behavior changes with move size; latency; overshoot |
| 2 | **Reverse-direction** (up then back down, repeated) | Direction-dependent behavior, early signal for backlash |
| 3 | **Ramp trajectory** (slow / medium / fast) | The speed at which the joint starts falling behind command — the effective velocity limit |
| 4 | **Triangle trajectory** (repeated up-down zig-zag) | Lag and smoothing behavior on one clean plot |
| 5 | **Loaded vs. unloaded** (arm folded vs. extended) | How gravity/load changes accuracy and speed |

Every experiment was run on `elbow_flex` first (recommended by the original
spec because its behavior is easy to observe), then repeated on
`shoulder_lift`.

### 4.3 What we measured — the headline numbers

Measured 2026-08-26 on the `twin_follower` arm, ~10,800 samples per load
condition:

| Parameter | No load | Loaded | Change |
|---|---:|---:|---:|
| Command-to-motion latency | 65.1 ms | 64.8 ms | ~0% (fixed cost) |
| Steady-state error | 0.128° | 0.472° | **+269%** |
| Maximum velocity | 191.6 °/s | 189.7 °/s | −1% |
| Overshoot | 0.140° | 0.189° | +35% |

**Key finding: load hits accuracy, not speed.** The joint arrives just as
fast whether loaded or not — but under load it stops measurably short of
the target, because gravity opposes the final approach and the servo's
internal control loop settles wherever torque balances the load. This
directly contradicted the naive illustrative expectation (latency
40→65 ms, velocity 90→72°/s) from the original task brief — a genuine,
measured property of this specific actuator, not what was expected going
in.

**Latency is a fixed cost, not mechanical drag** — 65 ms regardless of
load, standard deviation only 3.6–3.8 ms across 69 measurements. This is
the servo's internal control loop plus serial round-trip time, and will not
improve with a lighter payload.

### 4.4 Backlash — a story about getting it wrong twice before getting it right

The original step/reverse experiments could **not** measure backlash
correctly, for a subtle reason: every trajectory always stepped *up* from
0° and returned *down* to 0°. That means position 0 was only ever
approached **from above**, and every other commanded position was only ever
approached **from one direction**. Backlash (mechanical slack/play) can
only be seen by comparing where a joint settles when approached from *two
different directions* — and the data simply didn't contain that comparison
anywhere.

Two earlier analysis attempts reported backlash numbers anyway (11.3°, then
a "confounded" 1.337°) by unintentionally comparing samples that weren't
truly settled, or that were the same directional-approach confound seen
from the other side. Both were later shown to be wrong.

**The fix:** a dedicated staircase probe
(`scripts/characterization/backlash_probe.py`) that walks the joint to the
*same* rung from both directions and holds long enough to fully settle,
specifically designed to separate two effects that look identical in
position data alone:

- **Backlash** — mechanical slack/play in the gears, a roughly constant
  offset.
- **Gravity droop** — the servo's steady-state position sagging under its
  own load, which varies with joint angle (it follows a sine curve of angle
  from vertical, not cosine — an easy trig mistake that was caught and
  corrected along the way, since assuming the wrong basis fitted at R² 0.056
  while the correct sine fit gave R² 0.880).

**Result**, once properly separated:

| Joint | Backlash | Gravity droop (sine fit R²) |
|---|---:|---:|
| shoulder_lift | 1.955° (22.2 ticks) | 1.009° (R² 0.880) |
| elbow_flex | 1.196° (13.6 ticks) | 1.379° (R² 0.975) |

Both joints carry meaningful slop and droop — not negligible next to the
eventual sim-to-real fit error of 0.434°.

**Loaded backlash grows under load** (measured on shoulder_lift: 2.812°
loaded vs. 2.578° unloaded at the one rung both runs shared, +9%) — meaning
part of the "backlash" is actually **compliance** (elastic flex under
torque), not pure gear slop, since pure slop wouldn't change with load.

---

## 5. The Mapping Math — How a Real Position Becomes a Sim Position

This is the file that actually does the translation:
**`scripts/digital_twin_env/real_sim_mapping_test/real_sim_joint_mapping.py`**

### 5.1 The core idea

A **per-joint linear rescale**, mirroring the exact same
normalize/unnormalize formulas LeRobot itself uses internally
(`motors_bus.py`'s `_normalize`/`_unnormalize`), so the mapping is
consistent with what the real hardware driver is already doing:

```
Real side (from calibration):        Sim side (from so101.xml):
  ticks -> normalized -100..100         radians, jnt_range[lo, hi]
  (0..100 for gripper)

                    THE BRIDGE
  frac = (real_normalized + 100) / 200        # arm joints -> 0..1
  frac = real_normalized / 100                # gripper -> 0..1

  sim_radians = lo + frac * (hi - lo)
```

In words: **take the real position as a percentage of that joint's own
calibrated travel, then apply that exact same percentage to the sim's
kinematic range.**

### 5.2 The actual code

```python
def real_to_sim(joint_name, real_normalized, sim_range_rad):
    lo, hi = sim_range_rad
    if joint_name == "gripper":
        frac = np.clip(real_normalized, 0.0, 100.0) / 100.0
    else:
        frac = (np.clip(real_normalized, -100.0, 100.0) + 100.0) / 200.0
    return float(lo + frac * (hi - lo))
```

Two details worth calling out:

- **Clamping, not extrapolation.** A real value like 250 (out of range,
  from a bug or a stale reading) gets clamped to 100 before conversion —
  it maps to the sim's hard limit, not to some radian value beyond the
  robot's physical range.
- **Sim joint ranges are never hardcoded.** They're read live from the
  compiled MuJoCo model (`model.jnt_range`) every time, via
  `load_sim_joint_ranges_rad()`. If `so101.xml` ever changes, the mapping
  picks it up automatically instead of silently going stale.

An inverse function, `sim_to_real()`, runs the same logic backward — needed
for M7's live "mirror" display (see §7).

### 5.3 What this guarantees, and what it explicitly does NOT

**Guaranteed:** the same *direction* and the same *relative position within
range*. Real 50% of travel always maps to sim 50% of its kinematic range,
whichever way that works out in actual radians.

**NOT guaranteed:** that real 0% and sim 0 radians are the *same physical
pose of the arm.* Why not — the sim's range is symmetric by construction
around its kinematic zero. The real calibration's midpoint is just
"whatever the middle of your calibration sweep happened to be." If a
calibration sweep wasn't perfectly centered on the joint's true mechanical
middle, the two zeros won't visually line up — exactly what the wrist_roll
and gripper span mismatches above demonstrate.

**The stated resolution:** this can only be confirmed by eye, with hardware
connected — command a known real position and watch whether the simulated
arm visually matches. The math alone cannot self-certify true physical
correspondence; it only certifies proportionality.

---

## 6. Validating the Mapping Math Itself (No Hardware Needed)

Before ever touching real hardware, the mapping module was fully unit
tested headlessly:
**`scripts/digital_twin_env/real_sim_mapping_test/validate_real_sim_mapping.py`**

What it checks, in order:

1. **Sim joint ranges load correctly** from the already-validated M3 scene.
2. **Boundary values map exactly.** For every arm joint: real `-100` must
   land exactly on the sim's low limit, real `0` exactly on the sim
   midpoint, real `+100` exactly on the sim high limit (asserted to within
   `1e-9`). Gripper boundaries (`0`, `100`) checked the same way against
   its `0..100` convention.
3. **Round-trip correctness.** Sweeping every joint across its full real
   range (41 points for arm joints, 21 for the gripper), converting
   real→sim→real must return the original value to within `1e-6`.
4. **Out-of-range clamping**, not extrapolation — feeding `250` or `-250`
   must clamp to the sim's hi/lo limits exactly, never produce a value
   outside the physical joint range.
5. **Full 6-joint vector conversion** end-to-end, checking shape and that
   every converted value falls inside that joint's valid bounds.
6. **A shape check against a real calibration file** from an actual session
   — confirming the file has 6 joints with distinct, valid min/max ticks
   that line up with the expected joint names (explicitly *not* claiming
   this proves the numeric mapping is correct against real ticks — ticks
   and normalized values are different things, and this check says so in
   its own output).

This is a genuinely useful separation of concerns: if something ever looks
wrong once hardware is connected, this test tells you immediately whether
the *math* is broken or whether the problem is somewhere else (a
miscalibration, a wiring issue, a sign error in a different script).

---

## 7. Watching the Mapping Work Live — The M5/M6/M7 Progression

The mapping module by itself doesn't move anything. It gets *used* by a
sequence of increasingly capable control scripts, built deliberately in
stages so each one could be proven safe before adding risk.

### 7.1 M5 — keyboard → simulated joints only

`scripts/digital_twin_env/robot_keyboard_test/keyboard_robot.py`

No real hardware at all. Keyboard presses (Q/A, W/S, E/D, R/F, T/G, Y/H for
the 6 joints) nudge a target position, clamped to the sim's own
`jnt_range`, and MuJoCo's viewer shows the result. This proves the control
scheme and the safety-clamping logic before any hardware risk is
introduced.

### 7.2 M6 — keyboard → real arm only

`scripts/m6_keyboard_real.py`

Same keyboard scheme, now driving the physical servos through LeRobot's
`SOFollower` driver. No simulation involved at all — this isolates
"can we safely drive the real arm" from "can we mirror it in sim." Four
independent safety layers, all always on:

1. **Focus gate** — keys only act while a specific window is focused
   (detected automatically at startup), because the underlying keyboard
   library hooks the OS globally; without this, typing anywhere on the
   machine could move the real arm.
2. **Per-command clamp** (`max_relative_target`) — every single command is
   limited to a few units of motion from the arm's *currently measured*
   position, enforced inside LeRobot itself.
3. **Safe envelope** (±50 normalized units) — well inside the calibrated
   range, so no joint can be driven into its hard mechanical stop.
4. **One joint live at a time**, by default — everything else stays held
   at its starting position until explicitly widened.

### 7.3 M7 — keyboard → real arm + mirrored sim, together

`scripts/m7_mirror_sim.py`

This is where the mapping module is actually exercised live, every control
tick (20 Hz):

```python
# Read the real arm's MEASURED position first
obs = robot.get_observation()
live_obs = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}

# ... keyboard updates target, target gets sent to the real servos ...

# Convert that same measured position into sim radians and step the sim
data.ctrl[:6] = real_to_sim_vector(live_obs, sim_ranges)
mujoco.mj_step(model, data)
```

**The one deliberate design decision worth explaining carefully:** the sim
mirrors the real arm's **measured** position — what the servo actually
reports back after moving — not the **commanded target**. If it mirrored
the target instead, the sim would always look "correct" by definition, and
every real-world imperfection this project spent Task 1 measuring
(backlash, gravity droop, 65 ms latency) would be invisible on screen. By
displaying what the servo actually did, any visible gap between the
keyboard's intent and the sim's pose *is* the real tracking error — made
visible instead of hidden.

M7 has a `--source sim` bring-up mode specifically for proving the wiring
safely: no hardware is opened at all, and the sim just mirrors the
commanded target instead (since there's no real measurement to read). This
lets the whole keyboard → mapping → viewer pipeline be checked before ever
touching the arm.

### 7.4 What actually crosses the real→sim boundary

Per control tick, exactly **one number per joint** — six floats total — the
real arm's current normalized position. No velocity, torque, load, or
current is fed into the simulation's motion; those get logged (in `--record`
CSVs) purely for analysis, never used to drive `data.ctrl`.

---

## 8. Task 2 — Making the Simulation *Behave* Like the Real Joint

Matching *positions* isn't the whole story — a joint that ends up in the
right place but gets there with wildly wrong dynamics (too fast, too much
overshoot) is still a bad twin. Task 2 closed that gap.

### 8.1 Replaying real commands inside MuJoCo

`scripts/characterization/replay_in_sim.py` takes the exact recorded
command sequence from Task 1's real-arm CSVs and re-issues it, at the same
timestamps, to the simulated joint — then computes:

```
E(t) = q_real(t) - q_sim(t)
```

**First result, with the sim's original (never-validated) gains:** accurate
at rest (median |E| 0.115°) but wrong in flight. On a 20-unit step, the sim
overshot to **23.8°** against a 19.6° target and rang at up to **243°/s**
where the real joint tops out around **150°/s**. Endpoints right, dynamics
far too aggressive.

### 8.2 Root cause

`so101.xml`'s actuator gains (`kp=998.22 kv=2.731`) were never derived from
this hardware at all — they came from a formula in an unrelated public
project (a different RL-arm gains derivation) assuming a generic servo
proportional gain of 16, explicitly described even by that source as "not
a 1-to-1 mapping" of the actual LeRobot gains. They had simply never been
checked against a real SO-101 before this project.

### 8.3 Fitting real gains

`scripts/characterization/tune_actuator.py` does a grid search over `kp`,
`kv`, and the force limit, scoring each combination by RMSE against the
real recorded **step** responses — then, critically, **validates** the
winning combination against the **ramp, reverse, and triangle** runs that
were deliberately held out of the fit. Held-out error improving alongside
fit error is what separates a genuine fit from curve-fitting noise.

| | Fit set RMSE | Held-out RMSE |
|---|---:|---:|
| kp=998.22 kv=2.731 (shipped, unfitted) | 1.148° | 1.470° |
| **kp=400 kv=25 (fitted)** | **0.366°** | **0.561°** |
| Improvement | **68%** | **62%** |

Peak overshoot on the 20-unit step dropped from 8.05° to 2.19°.

**A subtlety worth mentioning to a technical audience:** the RMSE surface
turned out to be a flat valley along the ratio `kp/kv ≈ 16` (the
critical-damping ridge for this joint's reflected inertia) — every pair of
values along that ratio scores within 0.01° of each other. It's the
*ratio*, not the absolute magnitude, that matters physically; `kp=400`
simply sits at the minimum of that valley.

These fitted gains were applied to the shared `sts3215` actuator class in
`so101.xml` — meaning every joint on the arm uses them, since they're all
built from the same physical servo — with the previous, unfitted file kept
as `so101.xml.bak` for comparison. **Explicit caveat carried forward:** the
fit was only performed against `elbow_flex`; the shoulder carries far more
inertia and may need its own fit, which is a known, stated, open item
rather than an oversight.

### 8.4 Whole-dataset result

| | Shipped gains | Fitted gains |
|---|---:|---:|
| no-load RMSE | 1.254° | **0.434°** (−65%) |
| loaded RMSE | 1.544° | **0.850°** (−45%) |
| worst single-sample error | 10.76° | **4.87°** |

### 8.5 What still isn't modeled

Loaded runs still carry a consistent **−0.6° bias** that the sim doesn't
reproduce — this is the gravity droop Task 1 measured directly (steady-state
error rising 269% under load). The dynamics-fitting problem and the
gravity-droop problem are now cleanly separated instead of tangled
together, which is itself a valuable result: it tells us exactly what to
model next.

### 8.6 A late but important correction: backlash vs. the gain fit

Once backlash was properly measured (§4.4), an important interaction was
discovered: the gain-fit CSVs only ever visited two commands per step
experiment, so **most of the joint's backlash was already baked into the
fitted gains without anyone asking it to be.** A drive-layer model
(`servo_model.py`) that explicitly simulates backlash + gravity droop was
built for other purposes (new command sequences the fit never saw), but
**must never be stacked on top of the Task 2 gain fit** — doing so made
settled error 245% *worse*, because the same physical effect would be
counted twice. This is documented explicitly in the codebase specifically
so it doesn't get re-discovered the hard way later.

---

## 9. Full Validation Story — How We Know Any of This Is Actually Correct

Validation happened at **every layer**, deliberately kept separate so a
failure at any point could be isolated to its actual cause rather than
guessed at:

| Layer | How it was validated | Hardware needed? |
|---|---|---|
| MuJoCo scene loads at all | `validate_scenes.py` — headless load + 100 integration steps, checks finite state, correct body/joint/geom/mesh counts | No |
| Mapping math is internally consistent | `validate_real_sim_mapping.py` — boundary exactness, round-trip, clamping, vector shape, all described in §6 | No |
| Real joint behavior is trustworthy | `so101_joint_characterization.py` + `analyze_joint_response.py` — repeated trials, measured (not assumed) sample rate, retried writes | Yes |
| Backlash isolated correctly | `backlash_probe.py` + `analyze_backlash.py` — staircase approached from both directions, both a sine and cosine gravity fit attempted and compared | Yes |
| Sim dynamics match real dynamics | `replay_in_sim.py` — same commands, same timestamps, real vs sim overlay, RMSE reported per run and in aggregate | No (reuses recorded data) |
| Gain fit isn't overfit | `tune_actuator.py` — fit on step runs only, scored separately against ramp/reverse/triangle runs never used in fitting | No (reuses recorded data) |
| The live system behaves as expected | `m7_mirror_sim.py --source sim` first (wiring proof, zero hardware risk), then `--source real` (full loop) | Progressive — sim-only first, then real |
| **The one check nothing above can replace** | **Visual confirmation with hardware connected** — command a known real pose, watch whether the sim visually matches, because the mapping's proportional guarantee does not by itself prove a shared physical zero | Yes |

**A recurring theme worth stating plainly, because it came up more than
once:** several wrong conclusions were reached and then *corrected* during
this project — a false "46° shoulder_pan mismatch" that nearly triggered an
unnecessary recalibration (turned out real calibration spans and sim
kinematic spans are simply different quantities that were never supposed to
match), a "gravity compliance" reading and later a "1.337° pure backlash"
reading that were both the same directional-approach confound seen from two
sides, and a cosine-vs-sine gravity fit mistake caught because the fit
quality (R²) was checked rather than trusted at face value. None of these
were caught by assuming the first plausible-looking number was right — they
were caught by re-running the *validated* measurement scripts and comparing,
which is the entire reason this project's philosophy is "read the compiled
model / re-run the validator, never assume."

---

## 10. Summary Diagram (for a slide)

```
  UPSTREAM MODEL                    REAL ARM
  TheRobotStudio/SO-ARM100     Feetech STS3215 x6
        |                            |
        v                            v
  MuJoCo Menagerie              lerobot-calibrate
  (robotstudio_so101,           (per-arm, one-time,
   pinned commit)                records tick min/max)
        |                            |
        v                            v
  so101.xml                    raw ticks -> normalized
  (local fixes: FOV,           (-100..100, 0..100 gripper)
   gripper frame quat)               |
        |                            v
        |                    so101_joint_characterization.py
        |                    (5 experiments, real behavior
        |                     measured: latency, error,
        |                     velocity, backlash, load effect)
        |                            |
        v                            v
  jnt_range (radians,   <----  real_sim_joint_mapping.py
  kinematic design)             (per-joint % rescale,
        |                       clamped, round-trip tested)
        |                            |
        v                            v
   tune_actuator.py  <----  replay_in_sim.py
  (fits kp/kv against         (E(t) = q_real - q_sim,
   real step data,             proves the fit, held-out
   validated held-out)         validation)
        |
        v
  m7_mirror_sim.py
  (live: real measured position -> mapping -> sim ctrl,
   20 Hz, mirrors MEASURED not commanded, so real tracking
   error stays visible on screen)
        |
        v
  VISUAL CONFIRMATION WITH HARDWARE
  (the one check nothing above replaces)
```

---

## 11. If a Mentor Asks: The Short Version

1. We started with a **vetted, pinned, minimally-modified** model (traceable
   back to The Robot Studio's own SO-101 description via MuJoCo Menagerie),
   not something we drew from scratch.
2. We characterized the **real arm first** — latency, accuracy, speed,
   backlash, load effect — before writing any simulation-tuning code,
   because tuning a simulation against unknown real behavior is guessing.
3. The **position mapping** is a simple, honest percentage rescale between
   two different ranges — it guarantees proportional agreement, and we are
   explicit about the one thing it does *not* guarantee (a shared physical
   zero), which can only be confirmed visually.
4. We **fitted the simulation's dynamics** (not just its final position) to
   match the real joint's measured step response, and proved the fit
   wasn't overfitting by checking it against trajectories it never saw
   during fitting.
5. Validation happened in **layers** — math tested with zero hardware risk,
   then sim-only wiring proof, then real hardware — so a failure is always
   traceable to one specific layer instead of being a mystery.
6. We caught and **corrected our own mistakes** along the way (a false
   range mismatch, two wrong backlash readings, a trig-basis error) by
   re-running validated tools rather than trusting the first number that
   looked plausible — and documented each correction so it can't be
   silently re-made.
