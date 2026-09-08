# Project Status — SO-101 Digital Twin

**Last updated:** 2026-08-29

**Purpose of this file:** the single place to check "where are we right now"
and "what's left to do." Update this whenever a phase or milestone changes
state — this is a living tracker, not a one-time snapshot like
`PLAN-2026-08-26.md` (that file was a single session's run-sheet and is now
historical; everything in it is done).

This file tracks status at two levels:
- **§1 Milestones (M1–M7 + Task 1/2)** — the granular, already-largely-done
  work, matching the tables in `CLAUDE.md` and `README.md`.
- **§2 Phases (A–J)** — the larger goal from
  `docs/ROADMAP_TELEOP_TO_DATASET.md` (leader-arm teleop → environment
  parity → dataset → validation → Unreal rendering → mixed-domain training
  → cross-domain inference), which milestones M1–M7 are only the first
  slice of.

---

## ✅ RESOLVED: shoulder_lift would not move in its -1 direction (2026-08-29)

Discovered while validating M6/M7 keyboard control after this session's
joint characterization work. `shoulder_lift`'s `-1` direction (the `s` key)
sent real commands — confirmed via `--debug-joint shoulder_lift` (target
counts down correctly every control tick) — but the servo's actual
position never changed for the whole duration of the hold, at any starting
position, in a fresh terminal session, after a full `lerobot-calibrate`
resync. The `+1` direction (`w`) worked normally throughout.

**Ruled out:**
- Stuck/jammed mechanism — moves freely by hand in both directions.
- Being outside the servo's firmware position limits — checked directly
  against `Min_Position_Limit`/`Max_Position_Limit` registers at the time
  of the freeze; present position was mid-range, nowhere near either edge.
- Stale calibration/firmware limit mismatch — a full `lerobot-calibrate`
  sweep resynced everything (firmware limits moved to match the new sweep,
  confirmed by direct register read) and the issue was unchanged afterward.
- A leash/software bug specific to M6/M7's control loop — a single raw
  low-level `Goal_Position` write in the `-1` direction, bypassing
  LeRobot's `SOFollower`/`send_action()` entirely, DID move the servo
  successfully (verified twice, at two different starting positions).

**Also ruled out:** a rate/threshold issue — held `s` continuously for
15-20 seconds and `actual` never moved even a single tick throughout. This
is a hard, complete block in the `-1` direction through this control path,
not a slow response.

**Extensive follow-up isolation testing (same session, still 2026-08-29)**
tried to reproduce the freeze with standalone scripts replicating each
piece of M6's write path individually, working outward from the simplest
mechanism to the most M6-like:
- `sync_write` (LeRobot's actual `GroupSyncWrite` broadcast protocol, not
  the addressed `write2ByteTxRx` used in the first direct test) — single
  `-30` write worked; repeated `-10` writes stalled after the first one
  (this looked like a real lead at first).
- Repeated small `+5` vs `-5` steps at matched 20 Hz timing, shoulder_lift
  only — **both directions worked identically**, contradicting the sync_write
  finding above.
- The exact read-present-position → accumulate target → leash → write
  sequence M6 uses per tick, negative direction — worked fine, smooth
  20-step descent.
- `send_action`'s actual return value (`sent` vs `requested`), instrumented
  directly inside `m6_keyboard_real.py` — **always equal**, so LeRobot's own
  `max_relative_target` clamp is not silently rewriting the target.
- The full 6-joint `sync_write` payload M6 actually sends every tick (5
  joints held at their exact starting ticks, shoulder_lift decreasing) —
  **also worked fine**, smooth 25-step descent.
- `P_Coefficient` (LeRobot's `configure()` lowers this to 16 from the
  servo's default 32 "to avoid shakiness") looked like a promising lead,
  but the multi-joint test above already succeeded at the same P=16, so
  this is ruled out too.

**Net result of all the above: every mechanism tested in isolation —
outside M6's actual running process — worked correctly in both
directions, including the exact wire payload and timing M6 uses.** Only
M6 itself, run live, reliably failed. The one component no standalone
script had replicated yet was the `pynput` keyboard listener's background
thread running concurrently with the control loop.

**Root cause, found by testing exactly that:** a script shaped like M6
(same `SOFollower` connect/configure, same leash math, same 6-joint
`sync_write` payload, same 20 Hz rate) but with NO `pynput` listener
started moved `shoulder_lift`'s `-1` direction smoothly for a full 3
seconds, no freeze. The identical script WITH a live `pynput.Listener`
running — not even processing real keypresses, just alive in the
background — froze after ~1.5s, reproducing the exact symptom. Ran the
same "listener alive" test against `elbow_flex` instead: it moved fine
the entire 3 seconds. So this isn't "pynput breaks everything" — it's
specific to `shoulder_lift`, the heaviest and most gravity-loaded joint
(2.4x `elbow_flex`'s holding current, documented earlier this file). The
listener thread introduces enough timing jitter that a joint needing
tight, consistent command cadence against gravity/backlash can't keep
making progress under the default leash; lighter joints have enough
margin to shrug the same jitter off.

**Fix:** `SOFollowerRobotConfig.max_relative_target` accepts a per-motor
dict, not just a single float. Gave `shoulder_lift` its own leash
(`SHOULDER_LIFT_LEASH = 12.0`, 3x the shared default `4.0`) in both
`m6_keyboard_real.py` and `m7_mirror_sim.py`, while every other joint
keeps the tighter default. Confirmed in isolated testing (3 full seconds,
pynput alive, no stall) and then verified live in `m6_keyboard_real.py`
itself via `--recover`, holding Down and watching it move. The `--recover`
flow's own clamp reassignments (on stall-abort and on successful recovery)
also had to switch from overwriting `robot.config.max_relative_target`
with a plain scalar to restoring the per-joint dict, or they would have
silently wiped the fix back out partway through a session.

**Keymap reverted** to the normal M5-matching layout (`W/S` = shoulder_lift,
`E/D`/`Up`/`Down` = elbow_flex) in both scripts, since the temporary
Up/Down-for-shoulder_lift workaround is no longer needed.

**Also fixed in passing (same investigation):**
- `twin.py calibrate` failed with "lerobot-calibrate not found on PATH"
  even though it's installed — `shutil.which()` only searches the `PATH`
  env var, which doesn't include the venv's own `Scripts` folder when the
  venv is invoked via an absolute `python.exe` path (this project's own
  convention) rather than activated. Fixed to fall back to checking next
  to `sys.executable`.
- `m6_keyboard_real.py --recover` never actually recovered the gripper —
  its outside-envelope check unconditionally excluded it (`j != "gripper"`)
  and its recovery target was hardcoded to `0.0` (gripper's fully-CLOSED
  end, itself outside `GRIPPER_MIN..MAX`). Both fixed: gripper is now
  correctly detected as out-of-range using its own `GRIPPER_MIN..MAX` test,
  and recovers to `GRIPPER_BASE=40` instead of `0`.
- `m7_mirror_sim.py`'s `UnboundLocalError: record_file` — the out-of-envelope
  early `return` in `main()` skipped the line that initialized
  `record_file = None`, so the cleanup code at the bottom crashed
  referencing it. Fixed by initializing `record_file = None` at the top of
  `main()`, alongside the existing `robot = None` / `keys = None`.

---

## 1. Milestone status (granular, joint/control level)

| Milestone | What | Status |
|---|---|---|
| M1 | LINNMON/ADILS table scene | ✅ Done, validated |
| M2 | Runtime cube spawning (sim-only, no real connection) | ✅ Done, validated |
| M3 | SO-101 + table scene | ✅ Done, validated |
| M4 | Joint range/axis sanity check | ✅ Done (part of M3) |
| M5 | Keyboard → simulated joints only | ✅ Done, validated |
| M6-prep | Real↔sim mapping math (`real_sim_joint_mapping.py`) | ✅ Done, unit tested |
| M6 | Keyboard → real follower arm | ✅ Done, validated on hardware |
| M7 | Keyboard → real arm + mirrored sim, together | ✅ Done, validated on hardware |
| Task 1 | Real joint characterization (latency, error, velocity, backlash, load) | ✅ Done for `elbow_flex` + `shoulder_lift`. ⚠️ NOT done for `shoulder_pan`, `wrist_flex`, `wrist_roll`, `gripper` |
| Task 2 | Sim actuator gain fitting against real data | ✅ Done for `elbow_flex` (kp=400 kv=25, RMSE 1.254°→0.434°), applied to the shared servo class. ⚠️ Other joints borrow this fit, unvalidated for themselves |
| `twin.py` | Unified CLI dispatcher for every script above | ✅ Done, interactive menu + direct subcommands both working |

**Open item inside "done" work:** the M6/M7 focus-gate default was found to
be a stale hardcoded string (`"VR-SO-101 - Antigravity "`, leftover from a
different project) — fixed 2026-08-28 to auto-detect the focused window at
startup, same as M6 already did correctly.

---

## 2. Phase status (the larger goal — teleop, environment parity, dataset)

Full detail on each phase: `docs/ROADMAP_TELEOP_TO_DATASET.md`.
Full detail on the joint-mapping math specifically:
`docs/REAL_SIM_MAPPING_DEEP_DIVE.md`.

```
Phase A: Finish joint parity        [==========]  6 of 6 joints characterized, DONE
Phase B: Object/scene parity        [..........]  Not started
Phase C: Leader-arm teleop          [==========]  DONE — leader->real follower->sim, 2026-09-04
Phase D: VR teleop (revisited)      [..........]  Attempted once (Quest, non-telegrip), unsuccessful
Phase E: Dataset recording          [..........]  Not started — current --record output is a debug CSV, not a training dataset
Phase F: Dataset validation         [..........]  Not started — depends on E existing first
Phase G: MuJoCo -> Unreal bridge    [..........]  Not started, not decided — do not start before B is solid
Phase H: Synthetic data + rand.     [..........]  Not started — depends on G
Phase I: Mixed-ratio training       [..........]  Not started — depends on A/B being closed first, or results are uninterpretable
Phase J: Cross-domain inference     [..........]  Not started — depends on I
```

**Phases G–J** (added 2026-08-28) are the full intended end state: replicate
real data into the MuJoCo twin, generate synthetic data from sim, render
that through Unreal for photorealism, train on mixed ratios of
real/sim/Unreal data, then deploy the same trained policy in all three
environments to check it generalizes. Full detail, including why these are
ordered strictly after A–F rather than in parallel:
`docs/ROADMAP_TELEOP_TO_DATASET.md` §4 (Phases G–J) and §5.

### Phase A — Finish joint-level parity
**Status: DONE (2026-08-29).** All 6 joints — `elbow_flex`,
`shoulder_lift`, `shoulder_pan`, `wrist_flex`, `wrist_roll`, `gripper` —
are fully characterized and validated. `shoulder_pan` kept the shared
`sts3215` class default (`kp=400 kv=25`); the other 4 non-original joints
each carry their own per-joint `kp`/`kv` override in `so101_assets/so101.xml`
(`elbow_flex` was the original fit target; `shoulder_lift` deliberately
left unfitted per the backlash/compliance findings above).

**`shoulder_pan` (2026-08-29):** backlash 0.381° — well below elbow's
1.196° and the shoulder's 1.955°, consistent with it being a base rotation
joint with less mechanical load path than a lever joint. Gravity fit
failed (R² 0.236) as expected: it rotates about a vertical axis, so
gravity droop shouldn't apply here the way it does to `elbow_flex`/
`shoulder_lift`. `tune_actuator.py` confirmed the shared `kp=400 kv=25`
fit is already near-optimal for this joint — its own sweep's best point
improved the fit set 6% but made held-out error 8% *worse* (overfitting),
so no gain change was applied. Sim-to-real RMSE 0.521° (whole dataset),
in line with elbow's 0.434–0.638° range. Full numbers:
`scripts/characterization/raw_data/shoulder_pan_*.csv`.

**`wrist_flex` (2026-08-29):** backlash 1.134° (half-width 0.567°) —
clearly dominant over gravity (verdict: "backlash dominates", 1.134° vs
0.046°). Gravity fit also weak (R² 0.294) but for a different reason than
shoulder_pan: this joint DOES pitch against gravity, but the settled error
is small and fairly flat across all 7 rungs (+0.4 to +0.5°), consistent
with a light gripper-only load rather than no gravity coupling at all.
Unlike shoulder_pan, this joint's own gain fit was genuinely better and
NOT overfitting: `tune_actuator.py` found `kp=20 kv=1`, confirmed as an
interior minimum (checked by extending the sweep below the grid boundary,
not just trusting the edge value), scoring 0.636°/0.577° fit/held-out vs
the shared gains' 0.817°/0.730° (22%/21% better, held-out improving too).
Applied as a per-joint override on `wrist_flex`'s `<position>` element in
`so101_assets/so101.xml` — the `sts3215` class default (`kp=400 kv=25`)
is unchanged and still applies to every other joint. All 4 scenes
re-validated after the edit; `elbow_flex`/`shoulder_pan` replay RMSEs
confirmed unaffected. Sim-to-real RMSE 0.601° (whole dataset, at the new
gains). Full numbers: `scripts/characterization/raw_data/wrist_flex_*.csv`.

**`wrist_roll` (2026-08-29):** fastest joint measured yet — 250.8°/s peak
velocity (vs 130–190°/s for the others), consistent with it being a
low-inertia axial-rotation joint like `shoulder_pan`, not a lever joint.
Backlash 0.658° (half-width 0.329°), between shoulder_pan's 0.381° and
wrist_flex's 1.134°. Gravity fit weak (R² 0.406, amplitude 0.070°) as
expected — it rotates the gripper about its own long axis, so gravity
coupling should be small, similar to shoulder_pan's story. Own gain fit
genuinely better and not overfitting: `tune_actuator.py` found
`kp=400 kv=35` (kp unchanged from the shared default, only kv moved),
scoring 0.469°/0.591° fit/held-out vs the shared gains' 0.657°/0.633°
(29%/7% better, held-out improving too). Applied as a per-joint `kv`
override in `so101_assets/so101.xml`; all 4 scenes re-validated,
`wrist_flex` replay RMSE confirmed unaffected. Sim-to-real RMSE 0.547°
(whole dataset) — driven up mostly by the `ramp` run alone (RMSE 1.352°,
every other run 0.36–0.83°), expected since `ramp` is the highest-velocity
trajectory and isn't in the fit set, same "harder held-out set" pattern
already seen with elbow_flex's staircases. Also reconfirms the known span
mismatch: real calibrated 359.91° vs sim kinematic 314.42° (ratio 0.874,
correctly not applied — continuous rotation has no real hard stop to
match). Full numbers: `scripts/characterization/raw_data/wrist_roll_*.csv`.

**`gripper` (2026-08-29) — required real code fixes, not just flags.**
`so101_joint_characterization.py` and `backlash_probe.py` both hardcoded
LeRobot's −100..100 normalisation for every joint, but the gripper is
actually 0..100 with 0 = fully CLOSED (confirmed against LeRobot's own
`so_follower.py` source and `check_pose.py`'s documented bug: the same
wrong formula once reported −97.0 where the true value was +1.5). Fixed:
`JointBus` now takes a `joint_name` and switches its tick↔norm formula for
gripper; every `EXPERIMENTS` trajectory generator and `move_gently` call
now centres on `GRIPPER_BASE=50` instead of the arm joints' 0; both
scripts' safety clamps and `backlash_probe.py`'s `--lo`/`--hi` defaults
are gripper-aware. Verified headlessly before touching hardware: non-gripper
trajectories reproduce byte-identical to before, gripper trajectories stay
in-bounds, all 4 scenes still load.

**First hardware run surfaced a real mechanical finding.** With the initial
`GRIPPER_SAFE_MAX=80`, `triangle` and `ramp` (which command up to base+30)
drove the real jaw to a hard physical stop at **92.7° (~68 units)** every
time — confirmed by eye against the hardware (photos), not an obstruction
or damage, just the true open limit sitting short of what the calibration's
`range_max` implies. The servo held there under real sustained load rather
than reaching the commanded target, producing a bogus −6.96° "steady error"
on `triangle` that was actually a stall, not backlash or gravity.
`GRIPPER_SAFE_MAX` tightened 80→65 (comment left in
`so101_joint_characterization.py` explaining why); every joint re-run
clean, max actual position 88.6° across all 9 experiments, well clear of
the stall. Backlash 0.821° (half-width 0.41°) — computed directly from
`analyze_backlash.py`'s printed table since its gravity-fit block requires
≥3 shared rungs and the narrower envelope leaves gripper with only 2; no
gravity droop figure is available or expected to matter (pinch mechanism,
not a lever against gravity). Own gain fit genuinely better and not
overfitting: `kp=80 kv=6`, scoring 0.395°/0.533° fit/held-out vs the shared
gains' 0.490°/0.603° (19%/12% better). Applied as a per-joint override;
all 4 scenes re-validated, `wrist_roll`/`wrist_flex` replay RMSEs confirmed
unaffected. Sim-to-real RMSE 0.465° (whole dataset). Full numbers:
`scripts/characterization/raw_data/gripper_*.csv`.

### Phase B — Object/scene parity
**Status: NOT STARTED.** No code in this repo reads a real object's
position and places it correspondingly in the sim. `spawn_cube_test/` (M2)
only spawns objects at arbitrary or random positions purely inside the
simulation, with zero connection to physical reality. This is currently the
single largest gap between "the arm matches" (true) and "the environment
matches" (not yet true) — see Roadmap §2.3 and §4 Phase B for the proposed
approach (fiducial markers or depth camera + a real→sim object pose bridge
module, structurally parallel to the joint mapping).

### Phase C — Leader-arm teleop
**Status: BLOCKED on hardware.** No leader arm owned yet; keyboard remains
the explicit stand-in, by design (see `CLAUDE.md`, "an SO-101 follower we
have, standing in for a leader arm we don't have yet"). The control
architecture (`JointCommand` / `apply_command()` separation in
`keyboard_robot.py`, mirrored in M6/M7) was deliberately built so a leader
arm can be swapped in as a new input source later without rebuilding the
control loop, safety envelope, or mapping layer.

### Phase D — VR teleop
**Status: ATTEMPTED, UNSUCCESSFUL.** A newer VR attempt (Quest 2/3, a
different SDK/app than the sibling `so101-vr` project's working
`telegrip` setup) did not go well, attributed to limited VR-development
experience and not yet having settled on the right free/open tooling.
Explicitly treated as a lower-priority, separate track from Phases A–C —
this is a tooling/practice gap, not a research gap, since `so101-vr`'s
telegrip already proved Quest-based control works on this arm family.

### Phase E — Dataset recording
**Status: NOT STARTED.** `m7_mirror_sim.py --record` writes a plain
comparison CSV (target/real/sim degrees per tick), explicitly built only
for debugging tracking error — its own docstring states it is "not a
LeRobot Dataset." LeRobot's dataset-writing machinery (`lerobot-record`,
`LeRobotDataset`, Parquet + separate video files) is already installed in
this venv but not yet wired into any control script here. Depends on
Phase B existing first — recording clean joint data into an
environment whose object layout the sim doesn't know about would produce
a dataset that's joint-accurate but scene-fictional.

### Phase F — Dataset validation
**Status: NOT STARTED.** Depends entirely on Phase E producing data to
validate. Planned checks (timestamp integrity, physical plausibility
against Task 1's measured real-world limits, real/sim agreement within
established error bounds, object pose sanity) are specified in the roadmap
but not yet implemented as scripts.

### Phase G — MuJoCo → Unreal Engine bridge
**Status: NOT STARTED, not decided.** MuJoCo stays the single source of
physics; Unreal only renders (dual simulation was explicitly considered
and rejected — two independent physics engines would diverge from each
other for the same reasons real and sim already diverge). Realistically
the largest single new build in the whole plan — new Unreal-side assets
plus a pose-sync layer, neither of which exist yet. **Do not start before
Phase B is solid** — rendering an inaccurate twin photorealistically just
makes the inaccuracy convincing, not correct.

### Phase H — Synthetic data generation + domain randomization
**Status: NOT STARTED.** Depends on Phase G. Domain randomization
(varying lighting/textures/camera pose across synthetic frames) needs to
be designed in from the start — photorealistic rendering alone can make
sim-to-real transfer worse if a model learns the renderer's fixed
fingerprint instead of the task.

### Phase I — Mixed-ratio training (sim+real, sim+unreal, real+unreal)
**Status: NOT STARTED.** Depends on G/H, and requires Phase A/B already
closed — without that, a bad training ratio and a bad twin produce the
same symptom (worse real-world performance) and can't be told apart.

### Phase J — Cross-domain inference (deploy in MuJoCo + real + Unreal)
**Status: NOT STARTED.** The actual test of whether the mixed-domain
training closed the sim-to-real gap — run one trained policy in all three
environments and compare, rather than trusting sim performance alone.

---

## 3. What to do next (recommended order, from the roadmap)

1. ~~Characterize the remaining 4 joints (Phase A)~~ — **DONE 2026-08-29**,
   all 6 joints characterized and gain-fitted.
2. **Start Phase B (object/scene parity)** — pick a sensing method
   (fiducial markers recommended as the cheapest reliable start), build the
   real→sim object pose bridge, validate headlessly first then visually —
   same discipline the joint mapping already used.
3. **Phases C/D (leader arm / VR) proceed independently, whenever hardware
   or practice time is available** — they don't block or get blocked by
   Phase B, since they're interchangeable input sources sitting on top of
   whatever environment-parity work exists at the time.
4. **Phase E (dataset recording)** once Phase B has something trustworthy
   to record.
5. **Phase F (dataset validation)** immediately alongside E — build the
   validator against the first real recorded episodes, don't defer it
   until "later."

---

## 4. How to keep this file honest

- Update the milestone table (§1) and phase progress bars (§2) whenever a
  phase's status genuinely changes — not on a schedule, on actual state
  change.
- If a number here (e.g. "2 of 6 joints") stops matching what
  `scripts/characterization/RESULTS.md` or the raw_data folder actually
  shows, trust the underlying data and fix this file, not the other way
  around — same rule the rest of this project already follows for
  `CLAUDE.md`.
- When a phase moves from "not started" to "in progress," add a one-line
  note on what was tried and what was learned, the same way Phase A's
  entry above already documents the gripper-inertia risk rather than
  just a checkbox.
