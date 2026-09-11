# Session handoff — 2026-09-11

Written so the next chat can start from here instead of re-deriving. Each
item is **problem → what was found → options → what to do next**.

Companion to `TELEOP_SESSION_LOG_AND_PLAN.md` (the running narrative) and
`CLAUDE.md` (machine/project rules). This file is one day's findings and
open decisions, not a replacement for either.

---

## 0. State of the repo right now

- Working tree **clean**, but **22 commits unpushed** (`main` ahead of
  `origin/main`). Nothing has left this machine.
- Baseline validators all pass: `validate_scenes.py` 4/4,
  `validate_real_sim_mapping.py` all checks, `validate_robot_cube.py`
  16/16.
- **Uncommitted work in progress:** the divergence-guard rewrite in
  `replay_teleop_real.py` and `m_lerobot_teleop_sim.py` is finished and
  syntax-clean but **never run against hardware**. See §1.
- Hardware was **not connected** during this session. Everything below is
  from existing recordings and source reading.

---

## 1. SOLVED: why replays kept aborting (and why `--speed 0.25` "fixed" it)

### Problem

Every `--source sim` replay aborted partway with a joint "LAGGING" its
command. The workaround was `--speed 0.25`–`0.4`, i.e. replaying at a
quarter of real speed. Three explanations were given on 2026-09-10 and
**all three were wrong** (servo too weak, actuator force limit, joint
damping).

### What was actually found

`LeRobot`'s `ensure_safe_goal_position` (in
`lerobot/utils/robot_utils.py`, called from `so_follower.py:215`) caps
every command to `present_pos ± max_relative_target` and **re-anchors to
the arm's ACTUAL position every tick**. So:

- the follower can never get *ahead* of its goal, and
- the trajectory can run arbitrarily far *ahead of the follower* whenever
  it moves faster than the clamp allows.

The resulting gap measures **how fast the trajectory is moving**, not
whether anything is wrong.

Measured across every recording on this bench that contains a real
follower, during ordinary human-driven teleop with the arm visibly
tracking fine (`wrist_roll` excluded — it has its own fault):

| recording | joint | worst gap |
|---|---|---:|
| `teleop_log_60fps` | shoulder_lift | 34.5 |
| `teleop_log_pick3` | wrist_flex | 51.1 |
| `teleop_log_v10` | wrist_flex | 83.8 |
| `teleop_log_v9` | shoulder_lift | **122.3** |

**Healthy operation reaches 122 units.** `DIVERGENCE_LIMIT` was 45. The
guard was never detecting a fault — it was detecting normal motion, and
`--speed 0.25` only slowed things until the gap dropped under an
arbitrary line.

### The discriminator that does work

Wait until the **command** is nearly still (so the arm has had time to
arrive), then watch which way the gap moves:

| case | behaviour | evidence |
|---|---|---|
| catching up | gap **shrinks** | 13 of 13 healthy cases, trends −11 to −78 |
| runaway | gap **grows** | `teleop_pick_v1` wrist_roll, 0.0 → 47.7 (**+47.7**) |

**Zero overlap.** The *sign of the trend* separates them with no magnitude
threshold at all — which is why `teleop_log_v9`'s wrist_roll (91.8 units
out, but closing at −25.3) is correctly not flagged.

### What was changed (UNCOMMITTED, UNTESTED ON HARDWARE)

Both scripts now use, in place of `DIVERGENCE_LIMIT`:

```python
DIVERGENCE_STILL  = 0.05   # units/tick: command counts as stationary
DIVERGENCE_HOLD   = 20     # ticks still before the gap is judged
DIVERGENCE_GROWTH = 5.0    # units the gap may grow over that window
```

- `scripts/replay_teleop_real.py` — GUARD 2 rewritten, per-joint
  `still_ticks` / `gap_at_still` / `prev_action` state added.
- `scripts/m_lerobot_teleop_sim.py` — same logic, `prev_cmd` state added.

### Next step

1. **Commit it** (it is currently at risk of being lost).
2. Test on hardware with a short recording.
3. **Then try `--speed 1.0`.** If the clamp was the whole story, the
   slowdown may no longer be needed at all. That is the real prize here.

---

## 2. OPEN: sim arm reaches the floor when the real arm does not

### Problem

Point the wrist at the floor/table and the sim's gripper touches the
ground while the real arm stops short. Same joint angles, different
height.

### What is known

This is the **table height mismatch** already documented in
`real_sim_joint_mapping.py` and `TELEOP_SESSION_LOG_AND_PLAN.md` §A. At
the all-joints-zero pose the real gripper sits **28 cm** above the table,
the sim's at **21 cm** — the sim arm is **7 cm too low relative to its
floor**. It is not a speed or timing issue.

**Ruled out this session:** the model's link lengths are correct. Measured
from the compiled model — upper arm **11.60 cm**, forearm **13.50 cm** —
against published SO-101 figures of ~11.2 and ~13.5. So any fix that
changes link geometry is off the table.

**Ruled out previously** (see §A of the session log): joint mapping (all
joints within 1.6° at the worst pose), a clamp/mount gap (checked by
hand), corner-vs-centre placement, and contact tuning (swept across a 40×
stiffness range, ~5 cm penetration at *every* setting).

### Three fixes already tried and reverted

1. **Raise the base ~7.5 cm** → whole robot floated above the table.
2. **Lower the table 7 cm** → eliminated all penetration (224 → 0 frames)
   but left the base hanging 7 cm above the surface it is bolted to.
3. **Make the table solid** → arm stops *on* the table but contorts, up to
   38.9° of joint error (126° at stiff contact settings).

All three failed the same way: each rigidly translated one body, which
only works if the error is a constant offset.

### THE MEASUREMENT NOBODY HAS TAKEN

**Is the 7 cm error constant, or does it grow with reach?** This single
answer picks the fix:

- **Constant** → a rigid shift *is* right, and both attempts moved the
  wrong body. Base and floor need adjusting *together*.
- **Grows with reach** → mount geometry or a joint origin is wrong, and no
  translation will ever fix it.

Hardware-free: forward-kinematics every pose in the recordings and plot
the gripper's lowest point against horizontal reach.

**A first attempt at this produced junk** — do not trust it. Two bugs to
avoid on the retry:

- the folded rest pose sits parked on the floor across ~843 frames and
  swamps any average → **exclude stationary frames**;
- `geom_rbound` is a bounding-*sphere* radius, not a true lowest point →
  **use real geom extents**.

### Alternatives

| option | cost | when it makes sense |
|---|---|---|
| **Keep `--bare`** (current) | zero | Already in use. Robot on a plain groundplane, no table. Only 1 frame in 274 touches the floor. This is why the issue is mostly invisible today. |
| Take the measurement, then fix properly | ~10 min + a fix | If scene parity matters — i.e. once the cube work needs a table. |
| Inherit a table from a borrowed scene | see §3 | If adopting `sim-engine` or `lerobot_mujoco_sim`, their table/object layout replaces ours and this may become moot. |

### Recommendation

**Do not fix this yet.** `--bare` sidesteps it entirely, and if a borrowed
cube scene is adopted (§3) the table changes anyway. Take the measurement
only when a table is actually needed.

---

## 3. DECIDED: what to borrow vs what is genuinely ours

### Question asked

"Is there open source that already does this, instead of hand-writing
scripts and debugging for hours?"

### Answer: partly — and the split matters

Searched specifically for the full workflow (real↔sim joint mapping →
leader→sim recording → replay on the real follower → sim-vs-real
comparison).

| piece of the workflow | exists publicly? |
|---|---|
| real↔sim joint mapping (ticks ↔ radians) | yes, several |
| real arm **mirrored into** sim, live | yes |
| leader → sim recording, no follower | partly |
| **replaying the sim's own joint values onto the real arm** | **not found** |
| **per-joint sim-vs-real agreement measurement** | **not found** |

The closest match is a [Hackaday SO-101 Isaac Sim twin][hackaday] —
SO-101, leader-follower, real arm mirrored into the simulator. But it
describes itself as *"real→sim mirroring only (not bidirectional)"* and
*"a visualization/monitoring tool rather than a full bidirectional
sim-to-real control system"*. Its sim→real step is listed as future work.

**So `--source sim` (MuJoCo commanding the real follower) plus the
per-joint divergence table is the genuinely novel part of this project.**
It is also why 2026-09-10 was so painful: nobody has published the
pitfalls.

### Cube + camera scenes ARE worth borrowing

| repo | SO-101 scene + object | cameras | real arm synced |
|---|---|---|---|
| [sim-engine][simengine] | red box, blue bowl, table | overhead + wrist | no, sim only |
| [lerobot_mujoco_sim][ee5108] | blue block → bin | yes | no, sim only |
| [gym-hil][gymhil] | Franka, not SO-101 | yes | no |
| [so100-mujoco-sim][so100] | SO-100 | — | partial |

Both SO-101 repos have **already solved grasp contact tuning** — exactly
where `grasp_cube.py` is stuck (it reaches the cube but will not lift it).

**Caution:** one SO-101 HIL-SERL write-up reports that tooling built
around SO-100/Koch meant *"basically nothing worked out of the box"* for
SO-101. Borrowing is not free.

### Recommendation

**Borrow the cube scene and cameras. Keep the twin layer.** The scene is
not the interesting part of this project; the mapping/replay/comparison
layer has no equivalent to copy from.

Start with `sim-engine` (cleanest, both cameras, single UI) unless the
LeRobot dataset format matters more, in which case `lerobot_mujoco_sim`.

[hackaday]: https://hackaday.io/project/204187-chefmate-kitchen-robot-using-vla-act-diffusion/log/243785-building-a-real-to-sim-digital-twin-for-so-101-robot-arm-in-isaac-sim
[simengine]: https://github.com/RajatDandekar/sim-engine
[ee5108]: https://github.com/EE5108-DigitalTwins/lerobot_mujoco_sim
[gymhil]: https://github.com/huggingface/gym-hil
[so100]: https://github.com/lachlanhurst/so100-mujoco-sim

---

## 4. NOTED: LeRobot already ships camera + dataset recording

`lerobot-record.exe` is installed, and `lerobot/cameras/` provides opencv,
realsense and zmq backends. It records leader→follower teleop with
synchronised camera frames into the standard LeRobot dataset format — the
format policies actually train on.

**Do not hand-write a parallel version.** If cameras and datasets are the
goal, start from `lerobot-record` and find out what it does not cover,
rather than building from scratch.

Not yet checked: which camera backend this hardware needs, and the exact
CLI. Worth ten minutes before any camera work begins.

---

## 5. Still open from previous sessions

- **`wrist_roll`** — excluded from every hardware run via
  `--skip-joints wrist_roll`. Ran away repeatedly on 2026-09-10, then held
  0.00 units across 8 consecutive hold tests and under motion. Five
  theories tested and rejected. Never yet survived a full replay. Note the
  new guard (§1) would catch its signature — a gap growing while the
  command sits still — which is exactly what `teleop_pick_v1` recorded.
- **Motor bus dropping** — three times on 2026-09-10, all 6 IDs silent at
  once. Looks like a marginal cable rather than software. Not seen since
  the bus-traffic fix (`0cb18a0`).
- **`grasp_cube.py`** — reaches the cube (~3 mm IK residual) but does not
  lift it. See §3: a borrowed scene may make this moot.
- **22 commits unpushed.**

---

## Suggested order for the next session

1. **Commit the guard rewrite** (§1) — it is finished and at risk.
2. **Test it on hardware**, then try `--speed 1.0`. Biggest single win
   available: it may remove the slowdown entirely.
3. **Look at `sim-engine`** (§3) for the cube scene and cameras.
4. Leave the floor/table issue (§2) alone unless a table becomes
   necessary.
