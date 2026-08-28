# Project Status — SO-101 Digital Twin

**Last updated:** 2026-08-28
**Purpose of this file:** the single place to check "where are we right now"
and "what's left to do." Update this whenever a phase or milestone changes
state — this is a living tracker, not a one-time snapshot like
`PLAN-2026-08-26.md` (that file was a single session's run-sheet and is now
historical; everything in it is done).

This file tracks status at two levels:
- **§1 Milestones (M1–M7 + Task 1/2)** — the granular, already-largely-done
  work, matching the tables in `CLAUDE.md` and `README.md`.
- **§2 Phases (A–F)** — the larger goal from
  `docs/ROADMAP_TELEOP_TO_DATASET.md` (leader-arm teleop → environment
  parity → dataset → validation), which milestones M1–M7 are only the first
  slice of.

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
Phase A: Finish joint parity        [====......]  2 of 6 joints characterized
Phase B: Object/scene parity        [..........]  Not started
Phase C: Leader-arm teleop          [..........]  Blocked — no leader-arm hardware yet
Phase D: VR teleop (revisited)      [..........]  Attempted once (Quest, non-telegrip), unsuccessful
Phase E: Dataset recording          [..........]  Not started — current --record output is a debug CSV, not a training dataset
Phase F: Dataset validation         [..........]  Not started — depends on E existing first
```

### Phase A — Finish joint-level parity
**Status: IN PROGRESS.** `elbow_flex` and `shoulder_lift` are fully
characterized and validated. `shoulder_pan`, `wrist_flex`, `wrist_roll`,
and `gripper` are running on the `elbow_flex`-derived gain fit
(`kp=400 kv=25`, applied to the whole `sts3215` actuator class) without
their own hardware validation. This is the cheapest remaining phase — the
tooling (`twin.py characterize`, `twin.py tune`, `twin.py analyze`)
already exists end-to-end and just needs to be pointed at each remaining
joint.

**Known risk already on record:** the gripper in particular has very
different load/inertia characteristics than a rotating link joint, so
reusing the elbow's fit for it is the least-safe of the four remaining
assumptions.

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

---

## 3. What to do next (recommended order, from the roadmap)

1. **Characterize the remaining 4 joints** (Phase A) — cheapest, closes a
   known gap, all tooling ready: `python scripts/twin.py characterize
   --joint <name>` then `twin.py tune --joint <name>`.
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
