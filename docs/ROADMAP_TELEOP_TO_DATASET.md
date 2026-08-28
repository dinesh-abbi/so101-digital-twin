# Roadmap: Leader-Arm Teleop → Real+Sim+Unreal Parity → Mixed-Domain Training

**Status of this document:** a working plan, written from what actually
exists in this repo today (checked against the code, not assumed), plus
what would be genuinely new work. Meant to be read alongside
`docs/REAL_SIM_MAPPING_DEEP_DIVE.md`, which covers the joint-mapping piece
in full technical detail — this document is about everything **around**
that piece: the goal it serves, what's still missing, and in what order to
tackle it.

**The full intended end state** (added 2026-08-28, so it's written down
somewhere rather than living only in conversation): replicate real
recorded data into the MuJoCo twin, generate additional synthetic data
from the sim, pipe MuJoCo's physics into Unreal Engine for photorealistic
rendering of that same synthetic data, then train on **mixed ratios** of
real / MuJoCo-sim / Unreal-rendered data (sim+real, sim+unreal,
real+unreal, and combinations), and finally **infer with the same trained
policy in all three environments** (MuJoCo, the real arm, Unreal) to check
it generalizes rather than overfitting to one domain's quirks. Phases G–J
below cover this; Phases A–F remain the prerequisite foundation it's built
on.

---

## 1. The Actual End Goal, Stated Plainly

Joint mapping (real ↔ sim) was never the end goal — it was the
**prerequisite**. The real goal is:

1. Control the real arm with a **leader arm** (or VR) instead of a keyboard.
2. Have the **simulated environment match the real one** closely enough
   that training in sim transfers to reality — and that match has to cover
   **both the robot's joints AND the objects it interacts with**, not just
   the arm.
3. **Collect a dataset** of demonstrations (robot + object trajectories) in
   a form suitable for training a policy (e.g. imitation learning / LeRobot
   `Dataset` format).
4. **Validate** that dataset — confirm it's internally consistent and that
   it actually reflects what happened physically, before trusting it for
   training.

Everything built so far (M1–M7, Task 1/2 characterization) is the
**joint-level slice** of step 2. Steps 1, the object half of step 2, step
3, and step 4 are largely open.

---

## 2. What Already Exists (Verified Against the Code, Not Assumed)

### 2.1 Joint-level real↔sim matching — DONE, validated

Covered in full in `REAL_SIM_MAPPING_DEEP_DIVE.md`. Summary of what's
solid:

- Real arm's normalized position ↔ sim radians: linear rescale, unit
  tested, round-trip verified (`real_sim_joint_mapping.py`).
- Real joint *dynamics* (not just position) matched via fitted `kp`/`kv`
  gains, validated against held-out trajectories (`tune_actuator.py`,
  `replay_in_sim.py`). RMSE 1.254° → 0.434° on `elbow_flex`.
- Real joint imperfections measured and, where warranted, modeled:
  backlash, gravity droop, 65 ms fixed latency (`backlash_probe.py`,
  `servo_model.py`).
- Live mirrored control exists and works: `m7_mirror_sim.py` drives the
  real arm from keyboard and mirrors its *measured* position into the sim,
  20 Hz.

**Coverage gap inside this already-done piece:** only `elbow_flex` and
`shoulder_lift` have been characterized. The other four joints
(`shoulder_pan`, `wrist_flex`, `wrist_roll`, `gripper`) are using the
fitted-elsewhere `sts3215` class gains without their own validation. This
matters more as you move to actual manipulation tasks, since the gripper
and wrist orientation are usually load-bearing for grasping.

### 2.2 Input source — keyboard only, by design, for now

`m6_keyboard_real.py` / `m7_mirror_sim.py` use keyboard input as an
explicit **stand-in for a leader arm we don't have yet** (this is stated
directly in the project's own conventions). The architecture already
anticipates swapping the input source: `keyboard_robot.py`'s own docstring
describes the target/`apply_command()` split specifically so "the keyboard
is only an input device" and a future leader arm only needs to replace
*how the target gets nudged*, not the rest of the control loop.

**What this means practically:** when a leader arm arrives, it does not
require rebuilding M6/M7. It requires writing one new input-source module
that produces the same six-joint target dict the keyboard currently
produces, and wiring it in where `KeyboardInput` currently sits. The
control loop, safety envelope, focus gate concept (probably replaced by
"is the leader arm actually connected and calibrated" rather than a window
focus check), and the mapping layer stay as they are.

### 2.3 Object handling — sim-only, no real-world connection at all

`spawn_cube_test/spawn_cube.py` (M2) can spawn a cube in the sim at an
arbitrary or random position and let it fall/settle under MuJoCo physics.
This is a **pure simulation exercise** — there is currently:

- No camera or vision pipeline reading the real object's position.
- No mechanism to place a sim object at a pose corresponding to where a
  real object actually is.
- No object identity/pose tracking of any kind on the real-world side.

This is the **single largest genuine gap** relative to your stated goal.
Joint matching solves "does the arm agree between real and sim." Nothing
in the repo today solves "does the *scene* (table contents, object poses)
agree between real and sim." Training a manipulation policy needs both.

### 2.4 VR teleop — a different, newer attempt than the repo's own history

CLAUDE.md documents an *earlier, separate* project
(`D:\robotics\so101-vr\`, "telegrip") that got Quest 2 VR control working
and validated, with clean dataset recordings. You've indicated the VR
attempt you're referring to now is a **different, newer one** (not
telegrip), using Quest 2/3 with a different app or SDK, which did not go
well — attributed to inexperience with VR development and (implicitly)
not having settled on the right free/open tooling yet.

**What this means for the roadmap:** VR teleop is not "unsolved in
general" for this hardware family — `so101-vr`'s telegrip already proved
Quest-based teleop *can* work for an SO-101-class arm. The open problem is
specifically *this* project's VR attempt, on *this* newer tooling attempt,
without enough practice/experience yet. That's a tooling-and-practice gap,
not a research gap — genuinely different from the object-matching gap
above, which nothing in either project has solved yet.

### 2.5 Dataset recording and validation — not yet started in this project

Nothing in `so101-digital-twin` currently writes a `Dataset`-format
recording. `m7_mirror_sim.py --record` writes a **plain comparison CSV**
(target/real/sim degrees per tick) explicitly for debugging tracking error
— its own docstring says outright: "a plain log, not a LeRobot Dataset."
LeRobot itself (`lerobot-record`) is already installed in this venv and
capable of writing proper datasets, but it has not been wired into this
project's control scripts yet. The sibling `so101-vr` project does already
do LeRobot dataset recording at 20 Hz on real hardware — that's a proof
the mechanics work, on different tooling than this project's M6/M7.

---

## 3. Why Order Matters Here (Same Philosophy as Task 1 → Task 2)

The project's own working principle so far has been: **measure the real
thing first, validate the measurement, then build the simulated match, then
validate that match** — never skip straight to the fun part on an
unverified foundation. Task 1/2 (joint characterization → gain fitting) is
a direct example of this discipline paying off: it caught two wrong
backlash readings and a trig-basis error along the way, purely because
measurements were re-validated rather than trusted on first read.

The same discipline should carry into the object/scene and dataset work,
because the failure mode is identical in shape: **a dataset recorded
against a scene/object mapping that's subtly wrong will look fine and
train something, but the resulting policy will be learning a systematically
false relationship between what it sees and what it should do** — much
harder to detect after the fact than a wrong joint number, because there's
no simple RMSE check for "is this cube's sim position doing what its real
position did."

---

## 4. Proposed Phases

### Phase A — Finish joint-level parity (small, close to done)

- Characterize the remaining four joints (`shoulder_pan`, `wrist_flex`,
  `wrist_roll`, `gripper`) the same way `elbow_flex`/`shoulder_lift` were
  done: `twin.py characterize --joint <name>`, then `twin.py tune --joint
  <name>` (see caveat already on record: the shared `sts3215` gain fit may
  not suit the gripper, which has very different load/inertia
  characteristics than a rotating link).
- This closes the one open item explicitly flagged in the existing work,
  and it's cheap relative to what comes next — the tooling already exists
  end-to-end.

### Phase B — Object/scene parity (the real gap, needs new tooling)

This is genuinely new work, not a rerun of the joint-mapping playbook,
because the sensing problem is different in kind:

1. **Pick a sensing method for real object pose.** Realistic options for a
   low-cost setup:
   - A single fixed camera + fiducial markers (e.g. AprilTags/ArUco) glued
     to objects — cheap, deterministic, easy to validate (a marker's
     detected pose can be checked against a ruler by hand).
   - A depth camera + simple color/shape segmentation, if markers aren't
     acceptable for the target objects.
   - Manual pose entry for early bring-up (place an object at a
     hand-measured position, type it in) — not scalable, but useful as a
     zero-risk first step to validate the *sim side* of the pipeline before
     any vision code exists at all, exactly how M7's `--source sim` let the
     wiring be proven before hardware was involved.
2. **Build the real→sim object pose bridge**, structurally parallel to
   `real_sim_joint_mapping.py`: a small, hardware-independent module that
   takes a detected real-world pose and produces a MuJoCo `qpos` write for
   that object's free joint (the mechanism `spawn_cube.py` already uses
   internally — `CubeSpawner.spawn_cube()` is most of this half already).
3. **Validate it the same way the joint mapping was validated**: a
   headless unit test with synthetic poses first (no camera needed), then
   a live visual check — place a real object at a known position, confirm
   the sim shows it in the same place, the same "trust nothing without a
   look" principle used throughout Task 1/2.
4. **Table/workspace calibration.** The camera's frame of reference and the
   sim's world frame are two more coordinate systems that won't
   automatically agree — this needs its own explicit calibration step
   (e.g. a checkerboard or measured reference points), conceptually the
   same category of problem as the servo's calibrated tick range vs. the
   sim's kinematic range, just for a camera instead of a servo.

### Phase C — Leader-arm teleop (when hardware arrives)

- Write one new input-source module producing the same six-joint target
  dict `KeyboardInput` currently does, sourced from the leader arm's own
  `SOLeader`-equivalent LeRobot driver instead of `pynput`.
- Reuse M6/M7's control loop, safety envelope, and mapping layer unchanged
  — this is the payoff of the target/`apply_command()` separation already
  built in.
- Bring-up order should mirror M7's own `--source sim` → `--source real`
  discipline: prove the leader arm's readings drive the *sim* correctly
  first, with the follower's real servos untouched, before connecting it to
  real hardware.

### Phase D — VR teleop, revisited with practice + better tooling

- Treat this as a separate, lower-priority track from Phases A–C, since
  it's a skills/tooling gap rather than a blocker for the main goal.
- Before re-attempting: look concretely at what `so101-vr`'s telegrip setup
  did differently and got working (Quest 2, verified live control +
  20 Hz dataset recording) — it's a working reference point on the exact
  same class of hardware, even though it's a separate project's codebase
  you were told not to modify or reuse directly.
- Free/open tooling worth evaluating deliberately (rather than trial and
  error) before the next attempt: OpenXR-based controller input (avoids
  vendor lock-in to one headset's SDK), or simpler UDP/WebSocket pose
  streaming from a Quest app if a full native SDK integration is the part
  causing friction.

### Phase E — Dataset recording, in LeRobot's actual `Dataset` format

- Once Phase B (object parity) exists, wire real dataset recording into
  the control loop — likely via LeRobot's own `LeRobotDataset` writer
  (already installed, `lerobot-record` proves the mechanics work), rather
  than continuing with M7's plain debug CSV, which was explicitly built
  only for tracking-error comparison, not for training data.
- Each recorded episode should capture, per timestep: real joint
  positions, sim joint positions (for tracking-error QA), real object
  pose(s), and whatever the input source (leader arm / VR) was commanding.

### Phase F — Dataset validation (don't skip this)

Concretely, "validating a dataset" for this kind of project usually means
checking, before training on it:

- **Timestamp integrity** — no frame gaps, consistent sample rate (the
  same discipline Task 1 already applies: measure the real rate, never
  assume 30/50 Hz).
- **Physical plausibility** — no joint velocity/acceleration exceeding
  what Task 1's characterization showed the real arm can actually do; a
  suspicious jump is either a sensor glitch or a real world event worth
  flagging, not silently trusted.
- **Real vs. sim agreement within the already-established error bounds**
  — if `elbow_flex` tracking error suddenly spikes past what Task 2's
  0.434° RMSE would predict, that episode likely has a mapping or hardware
  problem, not a "the robot did something interesting" moment.
- **Object pose sanity** — objects shouldn't teleport, clip through the
  table, or appear at positions outside the workspace the camera/marker
  system was actually calibrated for.

`lerobot-dataset-viz` (already installed) can do visual spot-checking once
data exists; the numeric checks above would need small project-specific
scripts, same spirit as `validate_scenes.py` and `analyze_joint_response.py`.

### Phase G — MuJoCo → Unreal Engine bridge (photorealistic rendering)

**Status: not started, not decided.** The intended shape, settled here so
it's written down rather than assumed: **MuJoCo remains the single source
of physics truth; Unreal only renders.** Concretely, that means every
simulation step MuJoCo's joint angles and object poses get piped into
Unreal (socket / shared-memory / file handoff — not yet chosen) purely to
pose a visual scene and render a frame, and Unreal never runs its own
physics for this arm.

**Explicitly rejected alternative: dual simulation** (both engines running
physics independently, results compared/reconciled). Two physics engines
simulating the same scene will diverge from each other over time for the
same reason real and sim already diverge from each other — floating-point
integration, contact-solver differences, timestep differences — and now
you'd be debugging *three* domains' disagreements (real vs. MuJoCo, real
vs. Unreal, MuJoCo vs. Unreal) instead of one. Single-source-of-physics
avoids that entirely: Unreal is a camera, not a second opinion.

**What this phase actually requires, concretely:**
- A matching Unreal scene: the same table, the same SO-101 arm mesh, the
  same object models — visual assets that don't exist for Unreal today,
  only for MuJoCo's simpler rendering pipeline.
- A synchronization layer streaming MuJoCo's per-step pose data into
  Unreal in real time (or batch, for offline dataset generation, which is
  the more realistic mode for this use case — you don't need this to run
  live at 20 Hz, only to reproduce a recorded/generated trajectory
  frame-by-frame for rendering).
- This is realistically the **single largest new build** in the whole plan
  — bigger than the joint-characterization work already done, because it's
  new infrastructure (two engines talking to each other) rather than new
  measurements on an existing setup.

**Do not start this before Phase B is solid.** Rendering a twin
photorealistically doesn't fix an inaccurate twin — it just makes the
inaccuracy look convincing. Get MuJoCo's own object/scene parity right
first, on the simpler engine, before adding a second engine and a
synchronization problem on top of an unverified foundation.

### Phase H — Synthetic data generation + domain randomization

**Status: not started.** Once Phase G exists, MuJoCo can generate
trajectories beyond what was ever physically recorded (new object
positions, new arm configurations) and render them through Unreal.

**The one non-obvious trap here, worth stating plainly:** photorealistic
rendering alone does not automatically make synthetic data *useful* for
closing the sim-to-real gap — and can make it worse. If every synthetic
frame shares one fixed lighting setup, one fixed camera position, one
fixed set of textures, a trained model can learn to key off those
rendering-specific artifacts instead of the actual task, and will then
fail to transfer to the real camera's genuinely different lighting/noise/
angle. The standard fix is **domain randomization** — deliberately varying
lighting, textures, camera pose/noise, and object appearance across the
synthetic dataset, so the model is forced to learn the task rather than
the renderer's fingerprint. This needs to be designed into Phase H from
the start, not bolted on after a first attempt underperforms.

### Phase I — Mixed-ratio training (sim+real, sim+unreal, real+unreal, combined)

**Status: not started, depends entirely on G/H existing.** The experimental
question — "what ratio of real / MuJoCo-sim / Unreal-rendered data produces
a policy that generalizes best" — is a legitimate and standard question in
sim-to-real robotics research. But it can only answer that question cleanly
if the *ingredients* are already independently trustworthy:

**Why Phase A/B must be done before this phase, not just before Phase G:**
if a policy trained on some ratio performs badly on the real arm, there
are two possible explanations — the ratio was wrong, or the MuJoCo twin's
underlying dynamics were wrong to begin with (Phase A/B's job). Without
Phase A/B already closed, this experiment **cannot distinguish those two
causes** — a bad ratio and a bad twin look identical from the outside
("real-world performance is worse than expected"). This is the same
tangled-cause failure mode the project already hit once with
`shoulder_lift` (where an 18%-better-looking gain fit was actually
absorbing a mislabeled calibration-drift error, not a real dynamics
improvement) — same shape of mistake, higher cost if it happens at the
scale of a full training run instead of a single gain sweep.

### Phase J — Cross-domain inference (deploy the same trained policy in MuJoCo, real, and Unreal)

**Status: not started, final phase.** Run the one policy trained in Phase I
inside all three environments and compare behavior — this is the actual
test of whether the mixed-domain training worked, not just a nice-to-have
demo. A policy that performs well in MuJoCo and Unreal but poorly on the
real arm is telling you the sim-to-real gap is still open somewhere
upstream (most likely Phase A/B's remaining coverage gaps, or Phase H's
domain randomization being insufficient) — this phase is a diagnostic as
much as a deliverable.

---

## 5. Suggested Order, and Why

```
Phase A (finish joint parity)  →  cheap, closes a known gap, no new tooling
        |
Phase B (object/scene parity) →  the real blocker for anything env-level;
        |                         needed regardless of which input source
        |                         (keyboard/leader/VR) ends up driving the arm
        v
Phase C (leader arm)  ---\
Phase D (VR, revisited)  -+--> both are INPUT SOURCES, interchangeable once
                                the mapping+environment layer is solid;
                                order between C and D depends only on which
                                hardware/skill becomes available first
        |
        v
Phase E (dataset recording)   →  needs a trustworthy environment (B) and
        |                         a working input source (C or D) first
        v
Phase F (dataset validation)  →  must exist before any training is trusted
        |
        v
Phase G (MuJoCo -> Unreal bridge)     →  render a twin you already trust;
        |                                 do NOT start this before B is solid
        v
Phase H (synthetic data + domain      →  needs G; randomization must be
        randomization)                   designed in from the start, not
        |                                 bolted on after
        v
Phase I (mixed-ratio training:        →  needs A/B closed first, or a bad
        sim+real, sim+unreal,            ratio and a bad twin become
        real+unreal, combined)           indistinguishable from the outside
        |
        v
Phase J (cross-domain inference:      →  the actual test of whether any of
        deploy in MuJoCo + real +        this worked; a diagnostic as much
        Unreal, compare)                 as a deliverable
```

**Why Phases G–J are ordered strictly after A–F, not in parallel:**
every later phase inherits the earlier phases' errors silently. An
inaccurate twin (open item in A/B) doesn't just make the mirrored sim
"look a bit off" the way it did in early M7 testing — once it's feeding a
synthetic-data generator (Phase H) and a mixed-ratio training run
(Phase I), the same inaccuracy becomes baked into every synthetic sample
at scale, and there is no ratio of real/sim/Unreal data that can correct
for a systematic bias in one of the ingredients. Close the foundation
before building the pipeline on top of it.

**Why object/scene parity (B) comes before leader-arm/VR (C/D):** whichever
input source ends up driving the arm, the thing being recorded is
*"robot + objects in an environment that matches sim."* Solving input
source first would produce clean control but still leave you recording
demonstrations into an environment where the sim doesn't know where
anything is — the dataset would be joint-accurate but scene-fictional. The
project already has direct proof of what happens when tooling is built
before the underlying measurement is trustworthy: the pre-fit `so101.xml`
gains, and the two early wrong backlash readings.

---

## 6. What "success" looks like, concretely, at each phase

| Phase | Concrete success signal |
|---|---|
| A | All 6 joints have their own fitted gains and a held-out RMSE reported, same table format as `RESULTS.md` |
| B | Place a real object at a known position; the sim shows it there within an agreed tolerance, confirmed visually, then a numeric error logged the same way `replay_in_sim.py` logs `E(t)` for joints |
| C | `m7_mirror_sim.py`-equivalent runs with a leader arm as input, real follower tracks the leader, sim mirrors the follower's measured position — same three-way check (leader target / real measured / sim mirrored) M7 already does for keyboard |
| D | A VR session produces smooth, low-latency control comparable to what telegrip already achieved on Quest 2, without fighting the SDK the whole session |
| E | An episode recorded end-to-end lands in `LeRobotDataset` format and plays back correctly in `lerobot-dataset-viz` |
| F | A validation script flags a deliberately-corrupted test episode (bad timestamp, teleported object, impossible joint velocity) and passes a known-good one — i.e., the validator is itself tested, not just assumed to work |
| G | A MuJoCo trajectory (recorded or newly generated) renders correctly in Unreal, frame-for-frame, with no drift between what MuJoCo says the pose is and what Unreal shows |
| H | A domain-randomization sweep (lighting/texture/camera variation) is actually implemented and demonstrably varies output frames — not just a single fixed photorealistic render repeated |
| I | Multiple sim/real/Unreal ratios are trained and scored against a **held-out real-hardware test set**, the same fit-vs-held-out discipline `tune_actuator.py` already uses, so a ratio's apparent improvement can be trusted rather than being an artifact |
| J | The same trained policy is deployed unmodified in MuJoCo, on the real arm, and in Unreal, and the three performance numbers are reported side by side — not just "it worked in sim" |

---

## 7. One Thing Worth Saying Out Loud

None of Phases B through F are "harder research" in the sense of being
unsolved problems in robotics generally — object pose estimation, dataset
recording, and dataset validation are all well-trodden ground with existing
tools (fiducial marker libraries, LeRobot's own dataset format, standard
data-quality checks). What makes this project's version of them
non-trivial is the same thing that made the joint mapping non-trivial:
**gluing existing pieces together for this specific arm, this specific
sim, and validating that the glue is actually correct** — rather than
inventing new algorithms. That's a realistic, honest scope for the work
ahead, and it's the same scope that made the joint-mapping work legitimate
without needing to be called a breakthrough.

The same is true of Phases G–J, with one difference worth naming plainly:
mixed real/sim/photorealistic-render training with cross-domain inference
is a well-precedented *strategy* in sim-to-real robotics research (domain
randomization, mixed-domain training), so choosing to pursue it isn't a
wrong turn — but it is a strategy that **amplifies whatever it's built on**.
It cannot fix an inaccurate twin by blending in more real data, and it
cannot tell a bad training ratio apart from a bad twin unless the twin was
already independently verified first. That's the entire reason Phases A–F
are positioned as strictly prerequisite rather than parallel work: not
because rendering and mixed-domain training are unimportant, but because
this project's own history (the unfitted `so101.xml` gains sitting
unchecked, two wrong backlash readings, a mislabeled calibration-drift
error that looked like a real improvement) is direct, repeated proof of
what happens when something is built on top of a measurement that was
assumed correct rather than verified. Phases G–J are where that lesson
matters most, because they operate at dataset scale instead of a single
script's output.
